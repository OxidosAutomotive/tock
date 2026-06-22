// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

// Virtualizer for the RNG
use core::cell::Cell;
use kernel::collections::list::{List, ListLink, ListNode};
use kernel::hil::rng::{Client, Continue, Rng};
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

use crate::virtualizers::selection_policy::{RoundRobinPolicy, SelectionPolicy};

#[derive(Copy, Clone, PartialEq)]
enum Op {
    Idle,
    Get,
}

// Struct to manage multiple rng requests
pub struct MuxRngMaster<
    'a,
    SP: SelectionPolicy<&'a VirtualRngMasterDevice<'a, SP>> = RoundRobinPolicy,
> {
    rng: &'a dyn Rng<'a>,
    devices: List<'a, VirtualRngMasterDevice<'a, SP>>,
    inflight: OptionalCell<&'a VirtualRngMasterDevice<'a, SP>>,
    selection_policy: SP,
}

impl<'a> MuxRngMaster<'a> {
    pub fn new(rng: &'a dyn Rng<'a>) -> MuxRngMaster<'a> {
        MuxRngMaster {
            rng,
            devices: List::new(),
            inflight: OptionalCell::empty(),
            selection_policy: RoundRobinPolicy::default(),
        }
    }
}

impl<'a, SP: SelectionPolicy<&'a VirtualRngMasterDevice<'a, SP>>> MuxRngMaster<'a, SP> {
    pub fn new_with_policy(rng: &'a dyn Rng<'a>, selection_policy: SP) -> MuxRngMaster<'a, SP> {
        MuxRngMaster {
            rng,
            devices: List::new(),
            inflight: OptionalCell::empty(),
            selection_policy,
        }
    }

    fn do_next_op(&self) -> Result<(), ErrorCode> {
        if self.inflight.is_none() {
            let mnode = self
                .selection_policy
                .select(self.devices.iter(), |node| node.operation.get() != Op::Idle);

            let return_code = mnode.map(|node| {
                let op = node.operation.get();
                let operation_code = match op {
                    Op::Get => {
                        let success_code = self.rng.get();

                        // Only set inflight to node if we successfully initiated rng
                        if success_code == Ok(()) {
                            self.inflight.set(node);
                        }
                        success_code
                    }
                    Op::Idle => unreachable!("Attempted to run idle operation in virtual_rng!"), // Can't get here...
                };

                // Mark operation as done
                node.operation.set(Op::Idle);
                operation_code
            });

            // Check if return code has a value
            if let Some(r) = return_code {
                r
            } else {
                Err(ErrorCode::FAIL)
            }
        } else {
            Ok(())
        }
    }
}

impl<'a, SP: SelectionPolicy<&'a VirtualRngMasterDevice<'a, SP>>> Client for MuxRngMaster<'a, SP> {
    fn randomness_available(
        &self,
        _randomness: &mut dyn Iterator<Item = u32>,
        _error: Result<(), ErrorCode>,
    ) -> Continue {
        // Try find if randomness is available, or return done
        self.inflight.take().map_or(Continue::Done, |device| {
            let cont_code = device.randomness_available(_randomness, _error);

            if cont_code == Continue::Done {
                let _ = self.do_next_op();
            }

            cont_code
        })
    }
}

// Struct for a single rng device
pub struct VirtualRngMasterDevice<'a, SP: SelectionPolicy<&'a Self> = RoundRobinPolicy> {
    //reference to the mux
    mux: &'a MuxRngMaster<'a, SP>,
    // Pointer to next element in the list of devices
    next: ListLink<'a, Self>,
    client: OptionalCell<&'a dyn Client>,
    operation: Cell<Op>,
}

// Implement ListNode trait for virtual rng device
impl<'a, SP: SelectionPolicy<&'a Self>> ListNode<'a, Self> for VirtualRngMasterDevice<'a, SP> {
    fn next(&self) -> &'a ListLink<'_, VirtualRngMasterDevice<'a, SP>> {
        &self.next
    }
}

impl<'a, SP: SelectionPolicy<&'a Self>> VirtualRngMasterDevice<'a, SP> {
    pub const fn new(mux: &'a MuxRngMaster<'a, SP>) -> VirtualRngMasterDevice<'a, SP> {
        VirtualRngMasterDevice {
            mux,
            next: ListLink::empty(),
            client: OptionalCell::empty(),
            operation: Cell::new(Op::Idle),
        }
    }
}

impl<'a, SP: SelectionPolicy<&'a Self>> PartialEq<Self> for VirtualRngMasterDevice<'a, SP> {
    fn eq(&self, other: &Self) -> bool {
        // Check whether two rng devices point to the same device
        core::ptr::eq(self, other)
    }
}

impl<'a, SP: SelectionPolicy<&'a Self>> Rng<'a> for VirtualRngMasterDevice<'a, SP> {
    fn get(&self) -> Result<(), ErrorCode> {
        self.operation.set(Op::Get);
        self.mux.do_next_op()
    }

    fn cancel(&self) -> Result<(), ErrorCode> {
        // Set current device to idle
        self.operation.set(Op::Idle);

        self.mux.inflight.map_or_else(
            || {
                // If no node inflight, just set node to idle and return
                Ok(())
            },
            |current_node| {
                // Find if current device is the one in flight or not
                if current_node == self {
                    self.mux.rng.cancel()
                } else {
                    Ok(())
                }
            },
        )
    }

    fn set_client(&'a self, client: &'a dyn Client) {
        self.mux.devices.push_head(self);

        // Set client to handle callbacks for current device
        self.client.set(client);

        // Set client for rng to be current virtualizer
        self.mux.rng.set_client(self.mux);
    }
}

impl<'a, SP: SelectionPolicy<&'a Self>> Client for VirtualRngMasterDevice<'a, SP> {
    fn randomness_available(
        &self,
        randomness: &mut dyn Iterator<Item = u32>,
        error: Result<(), ErrorCode>,
    ) -> Continue {
        self.client.map_or(Continue::Done, move |client| {
            client.randomness_available(randomness, error)
        })
    }
}
