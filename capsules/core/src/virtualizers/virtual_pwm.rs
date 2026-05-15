// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Virtualize a PWM interface.
//!
//! `MuxPwm` provides shared access to a single PWM interface for multiple
//! users. `PwmPinUser` provides access to a specific PWM pin.
//!
//! Usage
//! -----
//!
//! ```rust,ignore
//! # use kernel::static_init;
//!
//! let mux_pwm = static_init!(
//!     capsules_core::virtual_pwm::MuxPwm<'static, nrf52::pwm::Pwm>,
//!     capsules_core::virtual_pwm::MuxPwm::new(&base_peripherals.pwm0)
//! );
//! let virtual_pwm_buzzer = static_init!(
//!     capsules_core::virtual_pwm::PwmPinUser<'static, nrf52::pwm::Pwm>,
//!     capsules_core::virtual_pwm::PwmPinUser::new(mux_pwm, nrf5x::pinmux::Pinmux::new(31))
//! );
//! virtual_pwm_buzzer.add_to_mux();
//! ```

use kernel::collections::list::{List, ListLink, ListNode};
use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

use crate::virtualizers::selection_policy::{RoundRobinPolicy, SelectionPolicy};

pub struct MuxPwm<'a, P: hil::pwm::Pwm, SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>>> {
    pwm: &'a P,
    devices: List<'a, PwmPinUser<'a, P, SP>>,
    inflight: OptionalCell<&'a PwmPinUser<'a, P, SP>>,
    selection_policy: SP,
}

impl<'a, P: hil::pwm::Pwm, SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>>> MuxPwm<'a, P, SP> {
    // NOTE(frihetselsker): can we remove const?
    // pub const fn new(pwm: &'a P) -> MuxPwm<'a, P, RoundRobinPolicy> {
    pub fn new(pwm: &'a P) -> MuxPwm<'a, P, RoundRobinPolicy> {
        MuxPwm {
            pwm,
            devices: List::new(),
            inflight: OptionalCell::empty(),
            selection_policy: RoundRobinPolicy::default(),
        }
    }

    /// If we are not currently doing anything, scan the list of devices for
    /// one with an outstanding operation and run that.
    fn do_next_op(&self) {
        if self.inflight.is_none() {
            let mnode = self
                .selection_policy
                .select(self.devices.iter(), |node| node.operation.is_some());
            mnode.map(|node| {
                let started = node.operation.take().is_some_and(|operation| {
                    match operation {
                        Operation::Simple {
                            frequency_hz,
                            duty_cycle,
                        } => {
                            let _ = self.pwm.start(&node.pin, frequency_hz, duty_cycle);
                            true
                        }
                        Operation::Stop => {
                            // Can't stop if nothing is running
                            false
                        }
                    }
                });
                if started {
                    self.inflight.set(node);
                } else {
                    // Keep looking for something to do.
                    self.do_next_op();
                }
            });
        } else {
            // We are running so we do whatever the inflight user wants, if
            // there is some command there.
            self.inflight.map(|node| {
                node.operation.take().map(|operation| {
                    match operation {
                        Operation::Simple {
                            frequency_hz,
                            duty_cycle,
                        } => {
                            // Changed some parameter.
                            let _ = self.pwm.start(&node.pin, frequency_hz, duty_cycle);
                        }
                        Operation::Stop => {
                            // Ok we got a stop.
                            let _ = self.pwm.stop(&node.pin);
                            self.inflight.clear();
                        }
                    }
                    // Recurse in case there is more to do.
                    self.do_next_op();
                });
            });
        }
    }
}

#[derive(Copy, Clone, PartialEq)]
enum Operation {
    Simple {
        frequency_hz: usize,
        duty_cycle: usize,
    },
    Stop,
}

pub struct PwmPinUser<
    'a,
    P: hil::pwm::Pwm,
    SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>> = RoundRobinPolicy,
> {
    mux: &'a MuxPwm<'a, P, SP>,
    pin: P::Pin,
    operation: OptionalCell<Operation>,
    next: ListLink<'a, PwmPinUser<'a, P, SP>>,
}

impl<'a, P: hil::pwm::Pwm, SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>>> PwmPinUser<'a, P, SP> {
    pub const fn new(mux: &'a MuxPwm<'a, P, SP>, pin: P::Pin) -> PwmPinUser<'a, P, SP> {
        PwmPinUser {
            mux,
            pin,
            operation: OptionalCell::empty(),
            next: ListLink::empty(),
        }
    }

    pub fn add_to_mux(&'a self) {
        self.mux.devices.push_head(self);
    }
}

impl<'a, P: hil::pwm::Pwm, SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>>>
    ListNode<'a, PwmPinUser<'a, P, SP>> for PwmPinUser<'a, P, SP>
{
    fn next(&'a self) -> &'a ListLink<'a, PwmPinUser<'a, P, SP>> {
        &self.next
    }
}

impl<'a, P: hil::pwm::Pwm, SP: SelectionPolicy<&'a PwmPinUser<'a, P, SP>>> hil::pwm::PwmPin
    for PwmPinUser<'a, P, SP>
{
    fn start(&self, frequency_hz: usize, duty_cycle: usize) -> Result<(), ErrorCode> {
        self.operation.set(Operation::Simple {
            frequency_hz,
            duty_cycle,
        });
        self.mux.do_next_op();
        Ok(())
    }

    fn stop(&self) -> Result<(), ErrorCode> {
        self.operation.set(Operation::Stop);
        self.mux.do_next_op();
        Ok(())
    }

    fn get_maximum_frequency_hz(&self) -> usize {
        self.mux.pwm.get_maximum_frequency_hz()
    }

    fn get_maximum_duty_cycle(&self) -> usize {
        self.mux.pwm.get_maximum_duty_cycle()
    }
}
