// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

use core::any::Any;
use core::cell::RefCell;
use core::mem::{self, MaybeUninit};
use core::ops::Deref;
use core::ptr;

use kernel::ErrorCode;
use kernel::collections::queue::Queue;
use kernel::collections::ring_buffer::RingBuffer;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::utilities::cells::OptionalCell;

/// Client interface for accessing resources wrapped in a [`DriverMutex`]
///
/// This trait must be implemented by any kernel object wishing to access a resource managed by a
/// `DriverMutex`. The trait defines a single callback method `ready()` which will be invoked by the
/// `DriverMutex` when the resource becomes available.
///
/// Note that this contract does not directly provide a [`DriverMutexRef`], but rather an instance
/// of [`DriverMutexAny`]. It is the responsibility of the client to downcast the `DriverMutexAny`
/// into the appropriate concrete type `T` before it can be used.
///
/// The reason for this extra indirection/reflection is to facilitate complex clients needing to
/// interact with multiple `DriverMutex` instances. If we had made `DriverMutexClient` generic over
/// `T`, then it would be practically impossible for such clients to `impl` the trait multiple times
/// for each different `T` type, as doing so would cause Rust to complain about
/// ambiguous/conflicting trait implementations.
pub trait DriverMutexClient {
    /// Called by the [`DriverMutex`] when the resource becomes available.
    fn ready(&'static self, resource: DriverMutexAny);
}

/// Handle representing an individual client registration
///
/// Handles are used internally by [`DriverMutex`] to determine _which_ client is requesting access
/// to the resource. A new, unique handle value is returned from each call to
/// [`DriverMutex::add_client()`], and the client is expected to retain its handle so that it can be
/// used later to request the resource.
///
/// A handle produced by one `DriverMutex` instance cannot be used with any other `DriverMutex`.
/// Doing so will cause [`DriverMutex::request()`] to return [`ErrorCode::INVAL`].
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct DriverMutexHandle {
    /// Index of the client registration within the DriverMutex's client array
    client_index: usize,

    /// Raw pointer value of the DriverMutex in memory. Used to ensure handle values aren't confused
    /// between different mutexes. Relies on each instance having a globally unique memory address.
    mutex_ptr: *const (),
}

/// Internal state machine for a driver mutex
enum State {
    /// Resource is not in use
    Free,

    /// Resource is actively locked by a client
    Locked {
        /// Handle of the client which currently holds the lock
        handle: DriverMutexHandle,

        /// Counts the number of references held by the client
        ref_count: usize,

        /// Whether the current client has requested another ready callback
        pending: bool,
    },
}

/// Internal bookkeeping for a driver mutex
///
/// This bookkeeping is the same regardless of the generic type `T` of the underlying resource,
/// hence it does not need its own `T` parameter. This is important because it allows
/// [`DriverMutexAny`] to contain a reference to the inner data and implement proper drop behavior.
struct Inner {
    /// Used to dispatch `ready()` callbacks on a separate stack frame
    dc: DeferredCall,

    /// Queued client indices waiting for access
    queue: RefCell<RingBuffer<'static, usize>>,

    /// Current state of the mutex
    state: RefCell<State>,
}

impl Inner {
    /// Called whenever a [`DriverMutexRef`] or [`DriverMutexAny`] is dropped. Decrements the
    /// reference count. When no references remain and no callback is pending from the same client,
    /// releases the mutex and schedules a callback for next queued client.
    fn ref_dropped(&self) {
        let mut state = self.state.borrow_mut();
        let should_release = match &mut *state {
            // Internal invariant violated. Mutex _thinks_ it's free, yet clearly there was some ref
            // floating around that we didn't account for.
            State::Free => unreachable!(),

            State::Locked {
                ref_count, pending, ..
            } => {
                // Decrement the ref count
                *ref_count -= 1;

                // Keep the mutex locked if the active client has a pending ready callback. In that
                // state there may be no live references, but ownership should not pass to the next
                // queued client yet.
                *ref_count == 0 && !*pending
            }
        };

        // Release the mutex only when no references remain and no callback is pending.
        if should_release {
            mem::drop(state);
            self.state.replace(State::Free);

            // If another client is queued, then schedule its ready() callback.
            if self.queue.borrow().has_elements() {
                self.dc.set();
            }
        }
    }
}

/// Specialized mutex used to guard HIL driver implementations
///
/// `DriverMutex` is a container type which wraps and provides mutual exclusion semantics for a HIL
/// implementation within the Tock kernel. Even though Tock is fundamentally single-threaded, it is
/// nevertheless highly concurrent and therefore susceptible to race conditions within its event
/// loop when multiple kernel components are sharing the same underlying HIL driver.
///
/// ## Queuing
///
/// `DriverMutex` contains an integrated request queue. Multiple clients can
/// [request][DriverMutex::request] access to the underlying resource, and they will be given access
/// on a first-come first-served basis (as opposed to a traditional mutex, which would just return
/// some "busy" error code).
///
/// The length of the internal queue is fixed, meaning it can only support a limited number of
/// clients. Queue length is specified at construction time and may vary from instance to instance
/// as needed.
///
/// ## RAII
///
/// Access to the underlying resource is mediated via [`DriverMutexAny`] and [`DriverMutexRef`].
/// These are RAII guards which "release" the mutex upon being dropped. This provides some level of
/// assurance at compile time that the underlying resource can only be accessed _while_ the mutex is
/// held.
///
/// When the last guard for the active client is dropped, the mutex is effectively released. If
/// another client is queued, a new RAII guard is passed to its
/// [`ready()`][DriverMutexClient::ready] callback. This process is carried out within a
/// [deferred call][DeferredCall] to avoid doing too much work directly within the RAII `drop()`
/// method, which could otherwise lead to reentrancy hazards, poor performance, or overly deep call
/// stacks.
///
/// ## Reference Counting
///
/// To support more complex scenarios, the same client may request access to the mutex multiple
/// times. Instead of being added to the internal queue, the client's `ready()` callback is
/// scheduled again, even while the client still holds one or more `DriverMutexAny` or
/// `DriverMutexRef` instances.
///
/// Internally, the mutex maintains a count of each reference provided through the `ready()`
/// callback. This count is decremented whenever a reference is dropped. The mutex considers the
/// resource to be free only when this count reaches zero, at which point a new `ready()` callback
/// is scheduled for the next client from the queue.
///
/// Due to the asynchronous nature of this mutex, it is possible for a client to call `request()`,
/// then drop all its outstanding RAII guards _before_ the `ready()` callback is invoked, causing
/// the reference count to reach zero. The `DriverMutex` explicitly handles this case, ensuring the
/// resource remains locked until the pending `ready()` callback can be delivered.
///
/// Note that clients are still not permitted to have multiple outstanding requests. Each time a
/// client calls `request()`, it must wait until its `ready()` callback is invoked before calling
/// `request()` again.
///
/// ### Motivation for Reference Counting
///
/// Why go to the lengths of supporting this admittedly complex scheme?
///
/// It turns out there are some real world scenarios in which a kernel component needs to consume
/// services from multiple different HIL implementations, `A: Foo` and `B: Bar`. This component
/// would typically contain two references `&DriverMutex<A>` and `&DriverMutex<B>` which it can use
/// to access the corresponding drivers.
///
/// But depending on the topology of the underlying chip or board, the `Foo` and `Bar` traits may
/// either be implemented by different drivers or possibly by the same driver. For instance, some
/// chips may implement ECC and RSA acceleration using a generic "crypto" IP block, while other
/// chips may provide separate blocks (and thus separate drivers) for these functions.
///
/// By using the reference counting approach, drivers which consume `Foo` and `Bar` can be written
/// to support both topologies transparently. The respective mutexes for `A` and `B` may point to
/// different drivers. Or in the case that `A` and `B` are actually the same type, both of the
/// consumer's `&DriverMutex` references may literally refer back to the same individual mutex
/// instance. Either way, the consumer may use the same set of APIs to access the underlying
/// resource.
pub struct DriverMutex<T: 'static> {
    resource: &'static T,
    inner: Inner,
    clients: &'static [OptionalCell<&'static dyn DriverMutexClient>],
}

impl<T> DriverMutex<T> {
    /// Creates a new `DriverMutex`.
    pub fn new(
        resource: &'static T,
        clients: &'static [OptionalCell<&'static dyn DriverMutexClient>],
        queue_buffer: &'static mut [MaybeUninit<usize>],
    ) -> Self {
        DriverMutex {
            resource,
            inner: Inner {
                dc: DeferredCall::new(),
                queue: RefCell::new(RingBuffer::new(queue_buffer)),
                state: RefCell::new(State::Free),
            },
            clients,
        }
    }

    /// Registers a client with this `DriverMutex`.
    ///
    /// If the client was added successfully, a corresponding `DriverMutexHandle` is returned which
    /// can be used later when calling `DriverMutex::request()`.
    ///
    /// To support more complex scenarios, the same client _may_ be added multiple times. In this
    /// case, a new slot is _not_ consumed, and this function returns a copy of the handle that was
    /// originally returned when the client was first added.
    ///
    /// `DriverMutex` supports only a fixed number of clients. If there is no more room for another
    /// client, this method returns `None`.
    pub fn add_client(
        &'static self,
        client: &'static dyn DriverMutexClient,
    ) -> Option<DriverMutexHandle> {
        for (i, slot) in self.clients.iter().enumerate() {
            if slot.is_some() {
                // Check if the client in this slot matches the new incoming client
                let is_same_client = slot.map(|c| ptr::eq(client, c)).unwrap();
                if !is_same_client {
                    // This is a different client, try the next slot
                    continue;
                }
            } else {
                // Empty slot, go ahead and claim it
                slot.replace(client);
            }

            return Some(DriverMutexHandle {
                client_index: i,
                // N.B. The &self parameter has 'static lifetime, which guarantees this DriverMutex
                //      will have a unique memory address for the duration of the program. That
                //      allows us to use this pointer value to reliably detect handle confusion.
                mutex_ptr: ptr::from_ref(self).cast(),
            });
        }

        None
    }

    /// Requests access to the underlying resource.
    ///
    /// Client is added to the internal queue. When the resource becomes available, it will be
    /// passed to the client's [`ready()`][DriverMutexClient::ready] callback. To avoid reentrancy
    /// issues, this is guaranteed to happen in a separate DC.
    ///
    /// # Errors
    ///
    /// Returns [`ErrorCode::ALREADY`] if the client is already waiting in the queue.
    ///
    /// Returns [`ErrorCode::INVAL`] if passed a `handle` created from a different `DriverMutex`
    /// instance.
    pub fn request(&self, handle: DriverMutexHandle) -> Result<(), ErrorCode> {
        // Ensure the handle was created from this mutex instance
        let self_ptr: *const () = ptr::from_ref(self).cast();
        if self_ptr != handle.mutex_ptr {
            return Err(ErrorCode::INVAL);
        }

        let mut queue = self.inner.queue.borrow_mut();

        if queue.contains(handle.client_index) {
            return Err(ErrorCode::ALREADY);
        }

        // should_enqueue = Whether to add this request to the queue of clients awaiting callbacks
        // should_sched = Whether to schedule the deferred call.
        let (should_enqueue, should_sched) = match &mut *self.inner.state.borrow_mut() {
            // No outstanding references.
            State::Free => {
                // Enqueue and schedule a callback.
                (true, true)
            }

            // Currently active client requesting another reference
            State::Locked {
                handle: client,
                pending,
                ..
            } if *client == handle => {
                if *pending {
                    // Active client already has a pending callback
                    return Err(ErrorCode::ALREADY);
                }

                // Skip enqueuing and schedule another callback for the active client.
                *pending = true;
                (false, true)
            }

            // Requestor is different from currently active client
            State::Locked { .. } => {
                // Add to the queue, but wait for the active client to fully release the resource.
                (true, false)
            }
        };

        if should_enqueue && !queue.enqueue(handle.client_index) {
            // Queue length should match the component's client capacity, so this should be
            // unreachable if handles are valid and duplicate requests are rejected.
            return Err(ErrorCode::FAIL);
        }

        if should_sched {
            self.inner.dc.set();
        }

        Ok(())
    }
}

impl<T> DeferredCallClient for DriverMutex<T> {
    fn handle_deferred_call(&'static self) {
        let idx = match &mut *self.inner.state.borrow_mut() {
            // Mutex is currently free
            state @ State::Free => {
                // Pop next client from the queue
                let Some(client_index) = self.inner.queue.borrow_mut().dequeue() else {
                    // Unexpected callback: queue is empty
                    return;
                };

                // Mark mutex state as locked
                let mutex_ptr = ptr::from_ref(self).cast();
                *state = State::Locked {
                    handle: DriverMutexHandle {
                        client_index,
                        mutex_ptr,
                    },
                    ref_count: 1,
                    pending: false,
                };

                // Dispatch ready callback to the newly popped client
                client_index
            }

            // Mutex is currently locked by a client
            State::Locked {
                handle,
                ref_count,
                pending,
            } => {
                if !*pending {
                    // Unexpected callback: active client has no pending request
                    return;
                }

                // Increment reference count
                *ref_count += 1;

                // Ensure pending flag is cleared
                *pending = false;

                // Dispatch ready callback to the currently active client
                handle.client_index
            }
        };

        let resource_ref = DriverMutexAny {
            resource: self.resource,
            inner: &self.inner,
        };

        self.clients[idx].map(|c| {
            c.ready(resource_ref);
        });
    }

    fn register(&'static self) {
        self.inner.dc.register(self);
    }
}

/// A type-erased reference to a resource guarded by a [`DriverMutex`]
///
/// See [`DriverMutexClient`] for an explanation of how/why `DriverMutexAny` should be used.
pub struct DriverMutexAny {
    resource: &'static dyn Any,
    inner: &'static Inner,
}

impl DriverMutexAny {
    pub fn downcast<T>(self) -> Result<DriverMutexRef<T>, Self> {
        match self.resource.downcast_ref() {
            Some(resource) => {
                let new_ref = DriverMutexRef {
                    resource,
                    inner: self.inner,
                };

                // Skip running drop for self to prevent decrementing the reference count
                mem::forget(self);

                Ok(new_ref)
            }

            None => Err(self),
        }
    }
}

impl Drop for DriverMutexAny {
    fn drop(&mut self) {
        self.inner.ref_dropped();
    }
}

/// A smart pointer to a resource guarded by a [`DriverMutex`]
pub struct DriverMutexRef<T: 'static> {
    resource: &'static T,
    inner: &'static Inner,
}

impl<T> Deref for DriverMutexRef<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target {
        self.resource
    }
}

impl<T> Drop for DriverMutexRef<T> {
    fn drop(&mut self) {
        self.inner.ref_dropped();
    }
}
