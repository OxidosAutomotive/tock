// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Components for I2C.
//!
//! This provides two components.
//!
//! 1. `I2CMuxComponent` provides a virtualization layer for a I2C bus.
//!
//! 2. `I2CComponent` provides a virtualized client to the I2C bus.
//!
//! Usage
//! -----
//! ```rust
//! let mux_i2c = components::i2c::I2CMuxComponent::new(&stm32f3xx::i2c::I2C1, None, dynamic_deferred_caller)
//!     .finalize(components::i2c_mux_component_static!());
//! let client_i2c = components::i2c::I2CComponent::new(mux_i2c, 0x19)
//!     .finalize(components::i2c_component_static!());
//! ```

// Author: Alexandru Radovici <msg4alex@gmail.com>

use capsules_core::virtualizers::selection_policy::{RoundRobinPolicy, SelectionPolicy};
use capsules_core::virtualizers::virtual_i2c::{I2CDevice, MuxI2C, SMBusDevice};
use core::mem::MaybeUninit;
use kernel::capabilities;
use kernel::component::Component;
use kernel::create_capability;
use kernel::hil::i2c::{self, NoSMBus};

// Setup static space for the objects.
#[macro_export]
macro_rules! i2c_mux_component_static {
    ($I:ty $(,)?) => {{
        kernel::static_buf!(capsules_core::virtualizers::virtual_i2c::MuxI2C<'static, $I>)
    };};
    ($I:ty, $S:ty $(,)?) => {{
        kernel::static_buf!(capsules::virtual_i2c::MuxI2C<'static, $I, $S>)
    };};
    ($I:ty, policy_i2c: $SPI:ty $(,)?) => {{
        use kernel::hil::i2c::NoSMBus;
        kernel::static_buf!(capsules::virtual_i2c::MuxI2C<'static, $I, NoSMBus, $SPI>)
    };};
    ($I:ty, $SPI:ty, $S:ty, $SPS:ty $(,)?) => {{
        use kernel::hil::i2c::NoSMBus;
        kernel::static_buf!(capsules::virtual_i2c::MuxI2C<'static, $I, $S, $SPI, $SPS>)
    };};
}

#[macro_export]
macro_rules! i2c_component_static {
    ($I:ty $(,)?) => {{
        kernel::static_buf!(capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I>)
    };};
    ($I:ty, $SP:ty $(,)?) => {{
        kernel::static_buf!(capsules_core::virtualizers::virtual_i2c::I2CDevice<'static, $I, $SP>)
    };};
}

#[macro_export]
macro_rules! i2c_master_slave_component_static {
    ($I:ty $(,)?) => {{
        let i2c_master_buffer = kernel::static_buf!([u8; 32]);
        let i2c_slave_buffer1 = kernel::static_buf!([u8; 32]);
        let i2c_slave_buffer2 = kernel::static_buf!([u8; 32]);

        let driver = kernel::static_buf!(
            capsules_core::i2c_master_slave_driver::I2CMasterSlaveDriver<'static, $I>
        );

        (
            driver,
            i2c_master_buffer,
            i2c_slave_buffer1,
            i2c_slave_buffer2,
        )
    };};
}

#[macro_export]
macro_rules! i2c_master_driver_component_static {
    ($I:ty $(,)?) => {{
        let i2c_master_buffer = kernel::static_buf!([u8; 32]);

        let driver = kernel::static_buf!(capsules_core::i2c_master::I2CMasterDriver<'static, $I>);

        (driver, i2c_master_buffer)
    };};
}

pub struct I2CMuxComponent<
    I: 'static + i2c::I2CMaster<'static>,
    S: 'static + i2c::SMBusMaster<'static> = NoSMBus,
    SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>> = RoundRobinPolicy,
    SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>> = RoundRobinPolicy,

> {
    i2c: &'static I,
    smbus: Option<&'static S>,
    policy_i2c: SPI,
    policy_smbus: SPS
}

impl<I: 'static + i2c::I2CMaster<'static>, S: 'static + i2c::SMBusMaster<'static>>
    I2CMuxComponent<I, S>
{
    pub fn new(i2c: &'static I, smbus: Option<&'static S>) -> Self {
        I2CMuxComponent {
            i2c,
            smbus,
            policy_i2c: RoundRobinPolicy::default(),
            policy_smbus: RoundRobinPolicy::default(),
        }
    }
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        S: 'static + i2c::SMBusMaster<'static>,
        SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>>,
        SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>>,
    > I2CMuxComponent<I, S, SPI, SPS>
{
    pub fn new_with_policies(
        i2c: &'static I,
        smbus: Option<&'static S>,
        policy_i2c: SPI,
        policy_smbus: SPS,
    ) -> Self {
        I2CMuxComponent {
            i2c,
            smbus,
            policy_i2c,
            policy_smbus,
        }
    }
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        S: 'static + i2c::SMBusMaster<'static>,
        SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>>,
        SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>>,
    > Component for I2CMuxComponent<I, S, SPI, SPS>
{
    type StaticInput = &'static mut MaybeUninit<MuxI2C<'static, I, S, SPI, SPS>>;
    type Output = &'static MuxI2C<'static, I, S, SPI, SPS>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let mux_i2c = static_buffer.write(MuxI2C::new_with_policy(
            self.i2c,
            self.smbus,
            self.policy_i2c,
            self.policy_smbus,
        ));
        kernel::deferred_call::DeferredCallClient::register(mux_i2c);

        self.i2c.set_master_client(mux_i2c);

        mux_i2c
    }
}

pub struct I2CComponent<
    I: 'static + i2c::I2CMaster<'static>,
    S: 'static + i2c::SMBusMaster<'static>,
    SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>>,
    SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>>,
> {
    i2c_mux: &'static MuxI2C<'static, I, S, SPI, SPS>,
    address: u8,
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        S: 'static + i2c::SMBusMaster<'static>,
        SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>>,
        SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>>,
    > I2CComponent<I, S, SPI, SPS>
{
    pub fn new(mux: &'static MuxI2C<'static, I, S, SPI, SPS>, address: u8) -> Self {
        I2CComponent {
            i2c_mux: mux,
            address,
        }
    }
}

impl<
        I: 'static + i2c::I2CMaster<'static>,
        S: 'static + i2c::SMBusMaster<'static>,
        SPI: 'static + SelectionPolicy<&'static I2CDevice<'static, I, SPI, S, SPS>>,
        SPS: 'static + SelectionPolicy<&'static SMBusDevice<'static, I, S, SPS, SPI>>,
    > Component for I2CComponent<I, S, SPI, SPS>
{
    type StaticInput = &'static mut MaybeUninit<I2CDevice<'static, I, SPI, S, SPS>>;
    type Output = &'static I2CDevice<'static, I, SPI, S, SPS>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let i2c_device = static_buffer.write(I2CDevice::new(self.i2c_mux, self.address));

        i2c_device
    }
}

pub struct I2CMasterSlaveDriverComponent<I: 'static + i2c::I2CMasterSlave<'static>> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    i2c: &'static I,
}

impl<I: 'static + i2c::I2CMasterSlave<'static>> I2CMasterSlaveDriverComponent<I> {
    pub fn new(board_kernel: &'static kernel::Kernel, driver_num: usize, i2c: &'static I) -> Self {
        I2CMasterSlaveDriverComponent {
            board_kernel,
            driver_num,
            i2c,
        }
    }
}

impl<I: 'static + i2c::I2CMasterSlave<'static>> Component for I2CMasterSlaveDriverComponent<I> {
    type StaticInput = (
        &'static mut MaybeUninit<
            capsules_core::i2c_master_slave_driver::I2CMasterSlaveDriver<'static, I>,
        >,
        &'static mut MaybeUninit<[u8; 32]>,
        &'static mut MaybeUninit<[u8; 32]>,
        &'static mut MaybeUninit<[u8; 32]>,
    );
    type Output = &'static capsules_core::i2c_master_slave_driver::I2CMasterSlaveDriver<'static, I>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let i2c_master_buffer = static_buffer.1.write([0; 32]);
        let i2c_slave_buffer1 = static_buffer.2.write([0; 32]);
        let i2c_slave_buffer2 = static_buffer.3.write([0; 32]);

        let i2c_master_slave_driver = static_buffer.0.write(
            capsules_core::i2c_master_slave_driver::I2CMasterSlaveDriver::new(
                self.i2c,
                i2c_master_buffer,
                i2c_slave_buffer1,
                i2c_slave_buffer2,
                self.board_kernel.create_grant(self.driver_num, &grant_cap),
            ),
        );

        self.i2c.set_master_client(i2c_master_slave_driver);
        self.i2c.set_slave_client(i2c_master_slave_driver);

        i2c_master_slave_driver
    }
}

pub struct I2CMasterDriverComponent<I: 'static + i2c::I2CMaster<'static>> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    i2c: &'static I,
}

impl<I: 'static + i2c::I2CMaster<'static>> I2CMasterDriverComponent<I> {
    pub fn new(board_kernel: &'static kernel::Kernel, driver_num: usize, i2c: &'static I) -> Self {
        I2CMasterDriverComponent {
            board_kernel,
            driver_num,
            i2c,
        }
    }
}
impl<I: 'static + i2c::I2CMaster<'static>> Component for I2CMasterDriverComponent<I> {
    type StaticInput = (
        &'static mut MaybeUninit<capsules_core::i2c_master::I2CMasterDriver<'static, I>>,
        &'static mut MaybeUninit<[u8; 32]>,
    );
    type Output = &'static capsules_core::i2c_master::I2CMasterDriver<'static, I>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);

        let i2c_master_buffer = static_buffer.1.write([0; 32]);

        let i2c_master_driver =
            static_buffer
                .0
                .write(capsules_core::i2c_master::I2CMasterDriver::new(
                    self.i2c,
                    i2c_master_buffer,
                    self.board_kernel.create_grant(self.driver_num, &grant_cap),
                ));

        self.i2c.set_master_client(i2c_master_driver);

        i2c_master_driver
    }
}
