// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! Component for the SHT4x sensor.
//!
//! I2C Interface
//!
//! Usage
//! -----
//!
//! ```rust
//! let sht4x = components::sht4x::SHT4xComponent::new(sensors_i2c_bus, capsules_extra::sht4x::BASE_ADDR, mux_alarm).finalize(
//!         components::sht4x_component_static!(nrf52::rtc::Rtc<'static>),
//!     );
//! sht4x.reset();
//! ```

use capsules_core::virtualizers::selection_policy::SelectionPolicy;
use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules_core::virtualizers::virtual_i2c::{I2CDevice, MuxI2C};
use capsules_extra::sht4x::SHT4x;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::i2c::{self, NoSMBus};
use kernel::hil::time::Alarm;

// Setup static space for the objects.
#[macro_export]
macro_rules! sht4x_component_static {
    ($A:ty, $I:ty, $SP:ty $(,)?) => {{
        let buffer = kernel::static_buf!([u8; 6]);
        let i2c_device = kernel::static_buf!(
            capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>
        );
        let sht4x_alarm = kernel::static_buf!(
            capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>
        );
        let sht4x = kernel::static_buf!(
            capsules_extra::sht4x::SHT4x<
                'static,
                capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>,
                capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>,
            >
        );

        (sht4x_alarm, i2c_device, sht4x, buffer)
    };};
    ($A:ty, $I:ty $(,)?) => {{
        use capsules_core::virtualizers::selection_policy::RoundRobinPolicy;
        $crate::sht4x_component_static!($A, $I, RoundRobinPolicy)
    };};
}

pub type SHT4xComponentType<A, I> = capsules_extra::sht4x::SHT4x<'static, A, I>;

pub struct SHT4xComponent<
    A: 'static + Alarm<'static>,
    I: 'static + i2c::I2CMaster<'static>,
    SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
> {
    i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>,
    i2c_address: u8,
    alarm_mux: &'static MuxAlarm<'static, A>,
}

impl<
        A: 'static + Alarm<'static>,
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > SHT4xComponent<A, I, SP>
{
    pub fn new(
        i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>,
        i2c_address: u8,
        alarm_mux: &'static MuxAlarm<'static, A>,
    ) -> SHT4xComponent<A, I, SP> {
        SHT4xComponent {
            i2c_mux,
            i2c_address,
            alarm_mux,
        }
    }
}

impl<
        A: 'static + Alarm<'static>,
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > Component for SHT4xComponent<A, I, SP>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
        &'static mut MaybeUninit<I2CDevice<'static, I, SP>>,
        &'static mut MaybeUninit<
            SHT4x<'static, VirtualMuxAlarm<'static, A>, I2CDevice<'static, I, SP>>,
        >,
        &'static mut MaybeUninit<[u8; 6]>,
    );
    type Output = &'static SHT4x<'static, VirtualMuxAlarm<'static, A>, I2CDevice<'static, I, SP>>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let sht4x_i2c = static_buffer
            .1
            .write(I2CDevice::new(self.i2c_mux, self.i2c_address));

        let buffer = static_buffer.3.write([0; 6]);

        let sht4x_alarm = static_buffer.0.write(VirtualMuxAlarm::new(self.alarm_mux));
        sht4x_alarm.setup();

        let sht4x = static_buffer
            .2
            .write(SHT4x::new(sht4x_i2c, buffer, sht4x_alarm));
        sht4x_i2c.set_client(sht4x);
        sht4x_alarm.set_alarm_client(sht4x);

        sht4x
    }
}
