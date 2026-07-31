// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Intrusive singly linked list implementation.
//!
//! A node's link has three possible states:
//!
//! - [`LinkState::Unlinked`]: the node is not in a list.
//! - [`LinkState::Tail`]: the node is linked and is the last node.
//! - [`LinkState::Next`]: the node is linked and has a successor.
//!
//! Distinguishing `Unlinked` from `Tail` is necessary because neither state
//! has a successor, but only an unlinked node may be inserted into a list.

use core::cell::Cell;

use crate::ErrorCode;

/// Internal state of a node's list link.
#[derive(Clone, Copy)]
enum LinkState<'a, T: 'a + ?Sized>
where
    T: Copy,
{
    /// The node is not currently linked in any list.
    Unlinked,

    /// The node is linked and is the final node in its list.
    Tail,

    /// The node is linked and points to its successor.
    Next(&'a T),
}

/// Link embedded in every node stored in a [`List`].
pub struct ListLink<'a, T: 'a + ?Sized>(Cell<LinkState<'a, T>>);

impl<T: ?Sized> ListLink<'_, T> {
    /// Creates an unlinked list link.
    pub const fn empty() -> Self {
        Self(Cell::new(LinkState::Unlinked))
    }

    /// Returns whether the owning node is linked in a list.
    ///
    /// A tail node is linked even though it has no successor.
    pub fn is_linked(&self) -> bool {
        !matches!(self.0.get(), LinkState::Unlinked)
    }
}

/// Trait implemented by nodes that can be stored in a [`List`].
pub trait ListNode<'a, T: ?Sized + 'a> {
    /// Returns this node's embedded list link.
    ///
    /// Implementations must always return the same [`ListLink`] instance for
    /// a particular node.
    fn next(&'a self) -> &'a ListLink<'a, T>;

    /// Returns whether this node is linked in any list.
    ///
    /// This does not determine which particular list contains the node.
    fn is_linked(&'a self) -> bool {
        self.next().is_linked()
    }
}

/// Intrusive singly linked list.
pub struct List<'a, T: 'a + ?Sized + ListNode<'a, T>> {
    head: Cell<Option<&'a T>>,
}

/// Iterator over a [`List`].
pub struct ListIterator<'a, T: 'a + ?Sized + ListNode<'a, T>> {
    cur: Option<&'a T>,
}

impl<'a, T: ?Sized + ListNode<'a, T>> Iterator for ListIterator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        match self.cur {
            Some(node) => {
                self.cur = match node.next().0.get() {
                    LinkState::Next(next) => Some(next),
                    LinkState::Tail | LinkState::Unlinked => None,
                };

                Some(node)
            }

            None => None,
        }
    }
}

impl<'a, T: ?Sized + ListNode<'a, T>> List<'a, T> {
    /// Creates an empty list.
    pub const fn new() -> Self {
        Self {
            head: Cell::new(None),
        }
    }

    /// Returns the first node in the list.
    pub fn head(&self) -> Option<&'a T> {
        self.head.get()
    }

    /// Inserts `node` at the beginning of the list after checking that it is
    /// not already linked.
    ///
    /// Returns [`ErrorCode::ALREADY`] if `node` is already linked in this or
    /// another list.
    pub fn try_push_head(&self, node: &'a T) -> Result<(), ErrorCode> {
        if node.is_linked() {
            return Err(ErrorCode::ALREADY);
        }

        // SAFETY: The check above established that `node` is unlinked.
        unsafe {
            self.push_head_unchecked(node);
        }

        Ok(())
    }

    /// Inserts `node` at the beginning of the list without checking whether
    /// it is already linked.
    ///
    /// This preserves the behavior of the original `push_head()` operation,
    /// but makes its required invariant explicit.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `node` is not currently linked in a list
    ///
    /// Inserting an already-linked node can corrupt list structure, create a
    /// cycle, or cause nodes to become unreachable through their original
    /// list.
    pub unsafe fn push_head_unchecked(&self, node: &'a T) {
        node.next().0.set(match self.head.get() {
            Some(head) => LinkState::Next(head),
            None => LinkState::Tail,
        });

        self.head.set(Some(node));
    }

    /// Inserts `node` at the end of the list after checking that it is not
    /// already linked.
    ///
    /// Returns [`ErrorCode::ALREADY`] if `node` is already linked in this or
    /// another list.
    pub fn try_push_tail(&self, node: &'a T) -> Result<(), ErrorCode> {
        if node.is_linked() {
            return Err(ErrorCode::ALREADY);
        }

        // SAFETY: The check above established that `node` is unlinked.
        unsafe {
            self.push_tail_unchecked(node);
        }

        Ok(())
    }

    /// Inserts `node` at the end of the list without checking whether it is
    /// already linked.
    ///
    /// This preserves the behavior of the original `push_tail()` operation,
    /// but makes its required invariant explicit.
    ///
    /// # Safety
    ///
    /// The caller must ensure that `node` is not currently linked in a list
    ///
    /// Inserting an already-linked node can corrupt list structure, create a
    /// cycle, or cause nodes to become unreachable through their original
    /// list.
    pub unsafe fn push_tail_unchecked(&self, node: &'a T) {
        // The inserted node becomes the new tail.
        node.next().0.set(LinkState::Tail);

        match self.iter().last() {
            Some(last) => {
                last.next().0.set(LinkState::Next(node));
            }

            None => {
                self.head.set(Some(node));
            }
        }
    }

    /// Removes and returns the first node in the list.
    ///
    /// The returned node is restored to the unlinked state and can therefore
    /// be inserted into a list again.
    pub fn pop_head(&self) -> Option<&'a T> {
        let node = self.head.get()?;

        self.head.set(match node.next().0.get() {
            LinkState::Next(next) => Some(next),
            LinkState::Tail => None,

            // A reachable list node must not be unlinked. Treat this as the
            // end of the list if the invariant has been violated.
            LinkState::Unlinked => None,
        });

        node.next().0.set(LinkState::Unlinked);

        Some(node)
    }

    /// Unlinks `node` if it is a member of this list.
    ///
    /// Returns `true` if the node was found and removed.
    /// Unlinks `node` if it is a member of this list.
    ///
    /// Returns `true` if the node was found and removed.
    pub fn remove(&self, node: &'a T) -> bool {
        // This only rejects a node that is definitely unlinked. A linked node
        // might belong to another list, so membership must still be verified
        // by traversing this list.
        if !node.is_linked() {
            return false;
        }

        let head = match self.head.get() {
            Some(head) => head,
            None => return false,
        };

        if core::ptr::addr_eq(head, node) {
            self.pop_head();
            return true;
        }

        let mut previous = head;

        while let LinkState::Next(current) = previous.next().0.get() {
            if core::ptr::addr_eq(current, node) {
                match current.next().0.get() {
                    LinkState::Next(next) => {
                        // Remove a middle node by linking its predecessor
                        // directly to its successor.
                        previous.next().0.set(LinkState::Next(next));
                    }

                    LinkState::Tail => {
                        // Removing the final node makes its predecessor the
                        // new tail.
                        previous.next().0.set(LinkState::Tail);
                    }

                    LinkState::Unlinked => {
                        // A node reachable from this list cannot legitimately
                        // be unlinked.
                        return false;
                    }
                }

                current.next().0.set(LinkState::Unlinked);
                return true;
            }

            previous = current;
        }

        false
    }
    /// Returns an iterator from the head to the tail.
    pub fn iter(&self) -> ListIterator<'a, T> {
        ListIterator {
            cur: self.head.get(),
        }
    }
}

impl<'a, T: ?Sized + ListNode<'a, T>> Default for List<'a, T> {
    fn default() -> Self {
        Self::new()
    }
}
