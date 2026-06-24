// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Components for the SI7021 Temperature/Humidity Sensor.
//!
//! This provides the SI7021Component which provides access to the SI7021 over
//! I2C.
//!
//! Usage
//! -----
//! ```rust
//! let si7021 = SI7021Component::new(mux_i2c, mux_alarm, 0x40).finalize(
//!     components::si7021_component_static!(sam4l::ast::Ast));
//! ```

// Author: Philip Levis <pal@cs.stanford.edu>
// Last modified: 6/20/2018

use core::mem::MaybeUninit;

use capsules_core::virtualizers::selection_policy::SelectionPolicy;
use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules_core::virtualizers::virtual_i2c::{I2CDevice, MuxI2C};
use capsules_extra::si7021::SI7021;
use kernel::component::Component;
use kernel::hil::i2c::{self, NoSMBus};
use kernel::hil::time::{self, Alarm};

// Setup static space for the objects.
#[macro_export]
macro_rules! si7021_component_static {
    ($A:ty, $I:ty, $SP:ty $(,)? ) => {{
        let alarm = kernel::static_buf!(
            capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>
        );
        let i2c_device = kernel::static_buf!(
            capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>
        );
        let si7021 = kernel::static_buf!(
            capsules_extra::si7021::SI7021<
                'static,
                capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>,
                capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>,
            >
        );
        let buffer = kernel::static_buf!([u8; 14]);

        (alarm, i2c_device, si7021, buffer)
    };};
    ($A:ty, $I:ty $(,)? ) => {{
        use capsules_core::virtualizers::selection_policy::RoundRobinPolicy;
        $crate::si7021_component_static!($A, $I, RoundRobinPolicy)
    };};
}

pub type SI7021ComponentType<A, I> = capsules_extra::si7021::SI7021<'static, A, I>;

pub struct SI7021Component<
    A: 'static + time::Alarm<'static>,
    I: 'static + i2c::I2CMaster<'static>,
    SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
> {
    i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>,
    alarm_mux: &'static MuxAlarm<'static, A>,
    i2c_address: u8,
}

impl<
        A: 'static + time::Alarm<'static>,
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > SI7021Component<A, I, SP>
{
    pub fn new(
        i2c: &'static MuxI2C<'static, I, NoSMBus, SP>,
        alarm: &'static MuxAlarm<'static, A>,
        i2c_address: u8,
    ) -> Self {
        SI7021Component {
            i2c_mux: i2c,
            alarm_mux: alarm,
            i2c_address,
        }
    }
}

impl<
        A: 'static + time::Alarm<'static>,
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > Component for SI7021Component<A, I, SP>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
        &'static mut MaybeUninit<I2CDevice<'static, I, SP>>,
        &'static mut MaybeUninit<
            SI7021<'static, VirtualMuxAlarm<'static, A>, I2CDevice<'static, I, SP>>,
        >,
        &'static mut MaybeUninit<[u8; 14]>,
    );
    type Output = &'static SI7021<'static, VirtualMuxAlarm<'static, A>, I2CDevice<'static, I, SP>>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let si7021_i2c = static_buffer
            .1
            .write(I2CDevice::new(self.i2c_mux, self.i2c_address));

        let si7021_alarm = static_buffer.0.write(VirtualMuxAlarm::new(self.alarm_mux));
        si7021_alarm.setup();

        let buffer = static_buffer.3.write([0; 14]);

        let si7021 = static_buffer
            .2
            .write(SI7021::new(si7021_i2c, si7021_alarm, buffer));

        si7021_i2c.set_client(si7021);
        si7021_alarm.set_alarm_client(si7021);
        si7021
    }
}
