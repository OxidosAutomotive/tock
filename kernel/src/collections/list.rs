// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Linked list implementation.

use core::cell::Cell;

/// Compare two(possibly unsized) references by data address only, ignoring
/// any pointer metadata such as vtable pointer
fn same_node<T: ?Sized>(a: &T, b: &T) -> bool {
    let a: *const T = a;
    let b: *const T = b;
    core::ptr::eq(a.cast::<()>(), b.cast::<()>())
}

pub struct ListLink<'a, T: 'a + ?Sized>(Cell<Option<&'a T>>);

impl<'a, T: ?Sized> ListLink<'a, T> {
    pub const fn empty() -> ListLink<'a, T> {
        ListLink(Cell::new(None))
    }
}

pub trait ListNode<'a, T: ?Sized> {
    fn next(&'a self) -> &'a ListLink<'a, T>;
}

pub struct List<'a, T: 'a + ?Sized + ListNode<'a, T>> {
    head: ListLink<'a, T>,
}

pub struct ListIterator<'a, T: 'a + ?Sized + ListNode<'a, T>> {
    cur: Option<&'a T>,
}

impl<'a, T: ?Sized + ListNode<'a, T>> Iterator for ListIterator<'a, T> {
    type Item = &'a T;

    fn next(&mut self) -> Option<&'a T> {
        let cur = self.cur?;
        self.cur = match cur.next().0.get() {
            // A self-link marks the tail
            Some(n) if same_node(n, cur) => None,
            other => other,
        };
        Some(cur)
    }
}

impl<'a, T: ?Sized + ListNode<'a, T>> List<'a, T> {
    pub const fn new() -> List<'a, T> {
        List {
            head: ListLink(Cell::new(None)),
        }
    }

    pub fn head(&self) -> Option<&'a T> {
        self.head.0.get()
    }

    /// Returns `Err(()) if the node is already in a list`
    pub fn is_linked(node: &'a T) -> bool {
        node.next().0.get().is_some()
    }

    pub fn push_head(&self, node: &'a T) -> Result<(), ()> {
        if Self::is_linked(node) {
            return Err(());
        }
        match self.head.0.get() {
            Some(h) => node.next().0.set(Some(h)),
            // Empty list: `node` becomes the tail, so it links to itself
            None => node.next().0.set(Some(node)),
        }
        self.head.0.set(Some(node));
        Ok(())
    }

    pub fn push_tail(&self, node: &'a T) -> Result<(), ()> {
        if Self::is_linked(node) {
            return Err(());
        }

        node.next().0.set(Some(node));
        match self.iter().last() {
            Some(last) => last.next().0.set(Some(node)),
            None => self.head.0.set(Some(node)),
        }
        Ok(())
    }

    pub fn pop_head(&self) -> Option<&'a T> {
        let node = self.head.0.get()?;
        self.head.0.set(match node.next().0.get() {
            Some(n) if same_node(n, node) => None,
            other => other,
        });
        // Restoring the invariant for the returned node
        node.next().0.set(None);
        Some(node)
    }

    /// Unlinks `node` if it is a member of *this* list.
    pub fn remove(&self, node: &'a T) -> bool {
        if !Self::is_linked(node) {
            return false;
        }
        let head = match self.head.0.get() {
            Some(h) => h,
            None => return false,
        };
        if same_node(head, node) {
            self.pop_head();
            return true;
        }
        let mut prev = head;
        while let Some(cur) = {
            match prev.next().0.get() {
                Some(n) if same_node(n, prev) => None,
                other => other,
            }
        } {
            if same_node(cur, node) {
                // If `node` was the tail, `prev` becomes the new tail.
                prev.next().0.set(match cur.next().0.get() {
                    Some(n) if same_node(n, cur) => Some(prev),
                    other => other,
                });
                node.next().0.set(None);
                return true;
            }
            prev = cur;
        }
        false
    }

    pub fn iter(&self) -> ListIterator<'a, T> {
        ListIterator {
            cur: self.head.0.get(),
        }
    }
}
