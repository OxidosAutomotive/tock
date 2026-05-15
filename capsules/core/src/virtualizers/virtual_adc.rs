// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Virtual ADC Capsule
//!
//! Support Single Sample for now.

use kernel::collections::list::{List, ListLink, ListNode};
use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

use crate::virtualizers::selection_policy::{RoundRobinPolicy, SelectionPolicy};

/// ADC Mux
pub struct MuxAdc<
    'a,
    A: hil::adc::Adc<'a>,
    P: SelectionPolicy<&'a AdcDevice<'a, A, P>> = RoundRobinPolicy,
> {
    adc: &'a A,
    devices: List<'a, AdcDevice<'a, A, P>>,
    inflight: OptionalCell<&'a AdcDevice<'a, A, P>>,
    selection_policy: P,
}

impl<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>> hil::adc::Client
    for MuxAdc<'a, A, P>
{
    fn sample_ready(&self, sample: u16) {
        self.inflight.take().map(|inflight| {
            for node in self.devices.iter() {
                if node.channel == inflight.channel {
                    node.operation.take().map(|operation| match operation {
                        Operation::OneSample => {
                            node.client.map(|client| client.sample_ready(sample))
                        }
                    });
                }
            }
        });
        self.do_next_op();
    }
}

impl<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>> MuxAdc<'a, A, P> {
    pub fn new(adc: &'a A) -> MuxAdc<'a, A, RoundRobinPolicy> {
        MuxAdc {
            adc,
            devices: List::new(),
            inflight: OptionalCell::empty(),
            selection_policy: RoundRobinPolicy::default(),
        }
    }

    fn do_next_op(&self) {
        if self.inflight.is_none() {
            let mnode = self
                .selection_policy
                .select(self.devices.iter(), |node| node.operation.is_some());
            mnode.map(|node| {
                let started = node.operation.map_or(false, |operation| match operation {
                    Operation::OneSample => {
                        let _ = self.adc.sample(&node.channel);
                        true
                    }
                });
                if started {
                    self.inflight.set(node);
                } else {
                    self.do_next_op();
                }
            });
        }
    }

    pub fn get_resolution_bits(&self) -> usize {
        self.adc.get_resolution_bits()
    }

    pub fn get_voltage_reference_mv(&self) -> Option<usize> {
        self.adc.get_voltage_reference_mv()
    }
}

#[derive(Copy, Clone, PartialEq)]
pub(crate) enum Operation {
    OneSample,
}

/// Virtual ADC device
pub struct AdcDevice<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>> {
    mux: &'a MuxAdc<'a, A, P>,
    channel: A::Channel,
    operation: OptionalCell<Operation>,
    next: ListLink<'a, AdcDevice<'a, A, P>>,
    client: OptionalCell<&'a dyn hil::adc::Client>,
}

impl<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>> AdcDevice<'a, A, P> {
    pub const fn new(mux: &'a MuxAdc<'a, A, P>, channel: A::Channel) -> AdcDevice<'a, A, P> {
        let adc_user = AdcDevice {
            mux,
            channel,
            operation: OptionalCell::empty(),
            next: ListLink::empty(),
            client: OptionalCell::empty(),
        };
        adc_user
    }

    pub fn add_to_mux(&'a self) {
        self.mux.devices.push_head(self);
    }
}

impl<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>>
    ListNode<'a, AdcDevice<'a, A, P>> for AdcDevice<'a, A, P>
{
    fn next(&'a self) -> &'a ListLink<'a, AdcDevice<'a, A, P>> {
        &self.next
    }
}

impl<'a, A: hil::adc::Adc<'a>, P: SelectionPolicy<&'a AdcDevice<'a, A, P>>> hil::adc::AdcChannel<'a>
    for AdcDevice<'a, A, P>
{
    fn sample(&self) -> Result<(), ErrorCode> {
        self.operation.set(Operation::OneSample);
        self.mux.do_next_op();
        Ok(())
    }

    fn stop_sampling(&self) -> Result<(), ErrorCode> {
        self.operation.clear();
        self.mux.do_next_op();
        Ok(())
    }

    fn sample_continuous(&self) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn get_resolution_bits(&self) -> usize {
        self.mux.get_resolution_bits()
    }

    fn get_voltage_reference_mv(&self) -> Option<usize> {
        self.mux.get_voltage_reference_mv()
    }
    fn set_client(&self, client: &'a dyn hil::adc::Client) {
        self.client.set(client);
    }
}
