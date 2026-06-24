// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Components for the Chirp I2C Moisture Sensor.
//! <https://www.tindie.com/products/miceuz/i2c-soil-moisture-sensor/>
//!
//! Usage
//! -----
//! ```rust
//!    let chirp_moisture =
//!        components::chirp_i2c_moisture::ChirpI2cMoistureComponent::new(mux_i2c, 0x20).finalize(
//!            components::chirp_i2c_moisture_component_static!(apollo3::iom::Iom<'static>),
//!        );
//!
//!    let moisture = components::moisture::MoistureComponent::new(
//!        board_kernel,
//!        capsules_extra::moisture::DRIVER_NUM,
//!        chirp_moisture,
//!    )
//!    .finalize(components::moisture_component_static!(ChirpI2cMoistureType));
//! ```

use capsules_core::virtualizers::selection_policy::SelectionPolicy;
use capsules_core::virtualizers::virtual_i2c::{I2CDevice, MuxI2C};
use capsules_extra::chirp_i2c_moisture::{ChirpI2cMoisture, BUFFER_SIZE};
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::i2c::{self, NoSMBus};

// Setup static space for the objects.
#[macro_export]
macro_rules! chirp_i2c_moisture_component_static {
    ($I:ty, $SP:ty $(,)?) => {{
        let i2c_device = kernel::static_buf!(
            capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>
        );
        let i2c_buffer = kernel::static_buf!([u8; capsules_extra::chirp_i2c_moisture::BUFFER_SIZE]);
        let chirp_i2c_moisture = kernel::static_buf!(
            capsules_extra::chirp_i2c_moisture::ChirpI2cMoisture<
                'static,
                capsules_core::virtualizers::virtual_i2c::I2CDevice<$I, $SP>,
            >
        );

        (i2c_device, i2c_buffer, chirp_i2c_moisture)
    };};
    ($I:ty $(,)?) => {{
        use capsules_core::virtualizers::selection_policy::RoundRobinPolicy;
        $crate::chirp_i2c_moisture_component_static!($I, RoundRobinPolicy)
    };};
}

pub type ChirpI2cMoistureComponentType<I> =
    capsules_extra::chirp_i2c_moisture::ChirpI2cMoisture<'static, I>;

pub struct ChirpI2cMoistureComponent<
    I: 'static + i2c::I2CMaster<'static>,
    SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
> {
    i2c_mux: &'static MuxI2C<'static, I, NoSMBus, SP>,
    i2c_address: u8,
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > ChirpI2cMoistureComponent<I, SP>
{
    pub fn new(i2c: &'static MuxI2C<'static, I, NoSMBus, SP>, i2c_address: u8) -> Self {
        ChirpI2cMoistureComponent {
            i2c_mux: i2c,
            i2c_address,
        }
    }
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        SP: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SP>>,
    > Component for ChirpI2cMoistureComponent<I, SP>
{
    type StaticInput = (
        &'static mut MaybeUninit<I2CDevice<'static, I, SP>>,
        &'static mut MaybeUninit<[u8; BUFFER_SIZE]>,
        &'static mut MaybeUninit<ChirpI2cMoisture<'static, I2CDevice<'static, I, SP>>>,
    );
    type Output = &'static ChirpI2cMoisture<'static, I2CDevice<'static, I, SP>>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let chirp_i2c_moisture_i2c = s.0.write(I2CDevice::new(self.i2c_mux, self.i2c_address));
        let i2c_buffer = s.1.write([0; BUFFER_SIZE]);

        let chirp_i2c_moisture =
            s.2.write(ChirpI2cMoisture::new(chirp_i2c_moisture_i2c, i2c_buffer));

        chirp_i2c_moisture_i2c.set_client(chirp_i2c_moisture);
        chirp_i2c_moisture
    }
}
