// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! Component for LPS22HB pressure sensor.

use capsules_core::virtualizers::selection_policy::SelectionPolicy;
use capsules_core::virtualizers::virtual_i2c::{I2CDevice, MuxI2C};
use capsules_extra::lps22hb::Lps22hb;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::i2c::{self, NoSMBus};

#[macro_export]
macro_rules! lps22hb_component_static {
    ($I:ty, $SP:ty $(,)?) => {{
        let i2c_device =
            kernel::static_buf!(capsules_core::virtualizers::virtual_i2c::I2CDevice<$I, $SP>);
        let lps22hb = kernel::static_buf!(
            capsules_extra::lps22hb::Lps22hb<
                'static,
                capsules_core::virtualizers::virtual_i2c::I2CDevice<$I, $SP>,
            >
        );
        let buffer = kernel::static_buf!([u8; 4]);

        (i2c_device, lps22hb, buffer)
    };};
        ($I:ty $(,)?) => {{
            use capsules_core::virtualizers::selection_policy::RoundRobinPolicy;
            $crate::lps22hb_component_static!($I, RoundRobinPolicy)
        };};
}

pub struct Lps22hbComponent<
    I: 'static + i2c::I2CMaster<'static>,
    SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
> {
    i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>,
    i2c_address: u8,
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > Lps22hbComponent<I, SP>
{
    pub fn new(i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>, i2c_address: u8) -> Self {
        Lps22hbComponent {
            i2c_mux,
            i2c_address,
        }
    }
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > Component for Lps22hbComponent<I, SP>
{
    type StaticInput = (
        &'static mut MaybeUninit<I2CDevice<'static, I, SP>>,
        &'static mut MaybeUninit<Lps22hb<'static, I2CDevice<'static, I, SP>>>,
        &'static mut MaybeUninit<[u8; 4]>,
    );
    type Output = &'static Lps22hb<'static, I2CDevice<'static, I, SP>>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let lps22hb_i2c = s.0.write(I2CDevice::new(self.i2c_mux, self.i2c_address));

        let buffer = s.2.write([0; 4]);

        let lps22hb = s.1.write(Lps22hb::new(lps22hb_i2c, buffer));
        lps22hb_i2c.set_client(lps22hb);

        lps22hb
    }
}
