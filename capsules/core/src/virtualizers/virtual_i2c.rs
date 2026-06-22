// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Virtualize an I2C master bus.
//!
//! `MuxI2C` provides shared access to a single I2C Master Bus for multiple
//! users. `I2CDevice` provides access to a specific I2C address.

use core::cell::Cell;

use kernel::collections::list::{List, ListLink, ListNode};
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::i2c::{self, Error, I2CClient, I2CHwMasterClient, NoSMBus};
use kernel::utilities::cells::{OptionalCell, TakeCell};

use crate::virtualizers::selection_policy::{RoundRobinPolicy, SelectionPolicy};
// `NoSMBus` provides a placeholder for `SMBusMaster` in case the board doesn't have a SMBus
pub struct MuxI2C<
    'a,
    I: i2c::I2CMaster<'a>,
    S: i2c::SMBusMaster<'a> = NoSMBus,
    SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>> = RoundRobinPolicy,
    SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>> = RoundRobinPolicy,
> {
    i2c: &'a I,
    smbus: Option<&'a S>,
    i2c_devices: List<'a, I2CDevice<'a, I, SPI, S, SPS>>,
    smbus_devices: List<'a, SMBusDevice<'a, I, S, SPS, SPI>>,
    enabled: Cell<usize>,
    i2c_inflight: OptionalCell<&'a I2CDevice<'a, I, SPI, S, SPS>>,
    smbus_inflight: OptionalCell<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    deferred_call: DeferredCall,
    i2c_selection_policy: SPI,
    smbus_selection_policy: SPS,
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > I2CHwMasterClient for MuxI2C<'a, I, S, SPI, SPS>
{
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), Error>) {
        if self.i2c_inflight.is_some() {
            self.i2c_inflight.take().map(move |device| {
                device.command_complete(buffer, status);
            });
        } else if self.smbus_inflight.is_some() {
            self.smbus_inflight.take().map(move |device| {
                device.command_complete(buffer, status);
            });
        }
        self.do_next_op();
    }
}

impl<'a, I: i2c::I2CMaster<'a>, S: i2c::SMBusMaster<'a>> MuxI2C<'a, I, S> {
    pub fn new(i2c: &'a I, smbus: Option<&'a S>) -> MuxI2C<'a, I, S> {
        MuxI2C {
            i2c,
            smbus,
            i2c_devices: List::new(),
            smbus_devices: List::new(),
            enabled: Cell::new(0),
            i2c_inflight: OptionalCell::empty(),
            smbus_inflight: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
            i2c_selection_policy: RoundRobinPolicy::default(),
            smbus_selection_policy: RoundRobinPolicy::default(),
        }
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > MuxI2C<'a, I, S, SPI, SPS>
{
    pub fn new_with_policy(
        i2c: &'a I,
        smbus: Option<&'a S>,
        i2c_selection_policy: SPI,
        smbus_selection_policy: SPS,
    ) -> MuxI2C<'a, I, S, SPI, SPS> {
        MuxI2C {
            i2c,
            smbus,
            i2c_devices: List::new(),
            smbus_devices: List::new(),
            enabled: Cell::new(0),
            i2c_inflight: OptionalCell::empty(),
            smbus_inflight: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
            i2c_selection_policy,
            smbus_selection_policy,
        }
    }

    fn enable(&self) {
        let enabled = self.enabled.get();
        self.enabled.set(enabled + 1);
        if enabled == 0 {
            self.i2c.enable();
        }
    }

    fn disable(&self) {
        let enabled = self.enabled.get();
        self.enabled.set(enabled - 1);
        if enabled == 1 {
            self.i2c.disable();
        }
    }

    fn do_next_op(&self) {
        if self.i2c_inflight.is_none() && self.smbus_inflight.is_none() {
            // Nothing is currently in flight

            // Try to do the next I2C operation
            let mnode = self
                .i2c_selection_policy
                .select(self.i2c_devices.iter(), |node| {
                    node.operation.get() != Op::Idle
                });
            mnode.map(|node| {
                node.buffer.take().map(|buf| {
                    match node.operation.get() {
                        Op::Write(len) => match self.i2c.write(node.addr, buf, len) {
                            Ok(()) => {}
                            Err((error, buffer)) => {
                                node.buffer.replace(buffer);
                                node.operation.set(Op::CommandComplete(Err(error)));
                                node.mux.do_next_op_async();
                            }
                        },
                        Op::Read(len) => match self.i2c.read(node.addr, buf, len) {
                            Ok(()) => {}
                            Err((error, buffer)) => {
                                node.buffer.replace(buffer);
                                node.operation.set(Op::CommandComplete(Err(error)));
                                node.mux.do_next_op_async();
                            }
                        },
                        Op::WriteRead(wlen, rlen) => {
                            match self.i2c.write_read(node.addr, buf, wlen, rlen) {
                                Ok(()) => {}
                                Err((error, buffer)) => {
                                    node.buffer.replace(buffer);
                                    node.operation.set(Op::CommandComplete(Err(error)));
                                    node.mux.do_next_op_async();
                                }
                            }
                        }
                        Op::CommandComplete(err) => {
                            self.command_complete(buf, err);
                        }
                        Op::Idle => {} // Can't get here...
                    }
                });
                node.operation.set(Op::Idle);
                self.i2c_inflight.set(node);
            });

            if self.i2c_inflight.is_none() && self.smbus.is_some() {
                // No I2C operation in flight, try SMBus next
                let mnode = self
                    .smbus_selection_policy
                    .select(self.smbus_devices.iter(), |node| {
                        node.operation.get() != Op::Idle
                    });
                mnode.map(|node| {
                    node.buffer.take().map(|buf| match node.operation.get() {
                        Op::Write(len) => {
                            match self.smbus.unwrap().smbus_write(node.addr, buf, len) {
                                Ok(()) => {}
                                Err(e) => {
                                    node.buffer.replace(e.1);
                                    node.operation.set(Op::CommandComplete(Err(e.0)));
                                    node.mux.do_next_op_async();
                                }
                            }
                        }
                        Op::Read(len) => {
                            match self.smbus.unwrap().smbus_read(node.addr, buf, len) {
                                Ok(()) => {}
                                Err(e) => {
                                    node.buffer.replace(e.1);
                                    node.operation.set(Op::CommandComplete(Err(e.0)));
                                    node.mux.do_next_op_async();
                                }
                            }
                        }
                        Op::WriteRead(wlen, rlen) => {
                            match self
                                .smbus
                                .unwrap()
                                .smbus_write_read(node.addr, buf, wlen, rlen)
                            {
                                Ok(()) => {}
                                Err(e) => {
                                    node.buffer.replace(e.1);
                                    node.operation.set(Op::CommandComplete(Err(e.0)));
                                    node.mux.do_next_op_async();
                                }
                            }
                        }
                        Op::CommandComplete(err) => {
                            self.command_complete(buf, err);
                        }
                        Op::Idle => unreachable!(),
                    });
                    node.operation.set(Op::Idle);
                    self.smbus_inflight.set(node);
                });
            }
        }
    }

    /// Asynchronously executes the next operation, if any. Used by calls
    /// to trigger do_next_op such that it will execute after the call
    /// returns. This is important in case the operation triggers an error,
    /// requiring a callback with an error condition; if the operation
    /// is executed synchronously, the callback may be reentrant (executed
    /// during the downcall). Please see
    /// <https://github.com/tock/tock/issues/1496>
    fn do_next_op_async(&self) {
        self.deferred_call.set();
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > DeferredCallClient for MuxI2C<'a, I, S, SPI, SPS>
{
    fn handle_deferred_call(&self) {
        self.do_next_op();
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}

#[derive(Copy, Clone, PartialEq)]
enum Op {
    Idle,
    Write(usize),
    Read(usize),
    WriteRead(usize, usize),
    CommandComplete(Result<(), Error>),
}

pub struct I2CDevice<
    'a,
    I: i2c::I2CMaster<'a>,
    SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>> = RoundRobinPolicy,
    S: i2c::SMBusMaster<'a> = NoSMBus,
    SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>> = RoundRobinPolicy,
> {
    mux: &'a MuxI2C<'a, I, S, SPI, SPS>,
    addr: u8,
    enabled: Cell<bool>,
    buffer: TakeCell<'static, [u8]>,
    operation: Cell<Op>,
    next: ListLink<'a, I2CDevice<'a, I, SPI, S, SPS>>,
    client: OptionalCell<&'a dyn I2CClient>,
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        S: i2c::SMBusMaster<'a>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > I2CDevice<'a, I, SPI, S, SPS>
{
    pub fn new(mux: &'a MuxI2C<'a, I, S, SPI, SPS>, addr: u8) -> I2CDevice<'a, I, SPI, S, SPS> {
        I2CDevice {
            mux,
            addr,
            enabled: Cell::new(false),
            buffer: TakeCell::empty(),
            operation: Cell::new(Op::Idle),
            next: ListLink::empty(),
            client: OptionalCell::empty(),
        }
    }

    pub fn set_client(&'a self, client: &'a dyn I2CClient) {
        self.mux.i2c_devices.push_head(self);
        self.client.set(client);
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        S: i2c::SMBusMaster<'a>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > I2CClient for I2CDevice<'a, I, SPI, S, SPS>
{
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), Error>) {
        self.client.map(move |client| {
            client.command_complete(buffer, status);
        });
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        S: i2c::SMBusMaster<'a>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > ListNode<'a, I2CDevice<'a, I, SPI, S, SPS>> for I2CDevice<'a, I, SPI, S, SPS>
{
    fn next(&'a self) -> &'a ListLink<'a, I2CDevice<'a, I, SPI, S, SPS>> {
        &self.next
    }
}

impl<'a, I: i2c::I2CMaster<'a>, SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI>>> i2c::I2CDevice
    for I2CDevice<'a, I, SPI>
{
    fn enable(&self) {
        if !self.enabled.get() {
            self.enabled.set(true);
            self.mux.enable();
        }
    }

    fn disable(&self) {
        if self.enabled.get() {
            self.enabled.set(false);
            self.mux.disable();
        }
    }

    fn write_read(
        &self,
        data: &'static mut [u8],
        write_len: usize,
        read_len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::WriteRead(write_len, read_len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn write(&self, data: &'static mut [u8], len: usize) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::Write(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn read(
        &self,
        buffer: &'static mut [u8],
        len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(buffer);
            self.operation.set(Op::Read(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, buffer))
        }
    }
}

pub struct SMBusDevice<
    'a,
    I: i2c::I2CMaster<'a>,
    S: i2c::SMBusMaster<'a>,
    SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>> = RoundRobinPolicy,
    SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>> = RoundRobinPolicy,
> {
    mux: &'a MuxI2C<'a, I, S, SPI, SPS>,
    addr: u8,
    enabled: Cell<bool>,
    buffer: TakeCell<'static, [u8]>,
    operation: Cell<Op>,
    next: ListLink<'a, SMBusDevice<'a, I, S, SPS, SPI>>,
    client: OptionalCell<&'a dyn I2CClient>,
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > SMBusDevice<'a, I, S, SPS, SPI>
{
    pub fn new(mux: &'a MuxI2C<'a, I, S, SPI, SPS>, addr: u8) -> SMBusDevice<'a, I, S, SPS, SPI> {
        if mux.smbus.is_none() {
            panic!("There is no SMBus to attach to");
        }

        SMBusDevice {
            mux,
            addr,
            enabled: Cell::new(false),
            buffer: TakeCell::empty(),
            operation: Cell::new(Op::Idle),
            next: ListLink::empty(),
            client: OptionalCell::empty(),
        }
    }

    pub fn set_client(&'a self, client: &'a dyn I2CClient) {
        self.mux.smbus_devices.push_head(self);
        self.client.set(client);
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > I2CClient for SMBusDevice<'a, I, S, SPS, SPI>
{
    fn command_complete(&self, buffer: &'static mut [u8], status: Result<(), Error>) {
        self.client.map(move |client| {
            client.command_complete(buffer, status);
        });
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > ListNode<'a, SMBusDevice<'a, I, S, SPS, SPI>> for SMBusDevice<'a, I, S, SPS, SPI>
{
    fn next(&'a self) -> &'a ListLink<'a, SMBusDevice<'a, I, S, SPS, SPI>> {
        &self.next
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > i2c::I2CDevice for SMBusDevice<'a, I, S, SPS, SPI>
{
    fn enable(&self) {
        if !self.enabled.get() {
            self.enabled.set(true);
            self.mux.enable();
        }
    }

    fn disable(&self) {
        if self.enabled.get() {
            self.enabled.set(false);
            self.mux.disable();
        }
    }

    fn write_read(
        &self,
        data: &'static mut [u8],
        write_len: usize,
        read_len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::WriteRead(write_len, read_len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn write(&self, data: &'static mut [u8], len: usize) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::Write(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn read(
        &self,
        buffer: &'static mut [u8],
        len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(buffer);
            self.operation.set(Op::Read(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, buffer))
        }
    }
}

impl<
        'a,
        I: i2c::I2CMaster<'a>,
        S: i2c::SMBusMaster<'a>,
        SPI: 'a + SelectionPolicy<&'a I2CDevice<'a, I, SPI, S, SPS>>,
        SPS: 'a + SelectionPolicy<&'a SMBusDevice<'a, I, S, SPS, SPI>>,
    > i2c::SMBusDevice for SMBusDevice<'a, I, S, SPS, SPI>
{
    fn smbus_write_read(
        &self,
        data: &'static mut [u8],
        write_len: usize,
        read_len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::WriteRead(write_len, read_len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn smbus_write(
        &self,
        data: &'static mut [u8],
        len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(data);
            self.operation.set(Op::Write(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, data))
        }
    }

    fn smbus_read(
        &self,
        buffer: &'static mut [u8],
        len: usize,
    ) -> Result<(), (Error, &'static mut [u8])> {
        if self.operation.get() == Op::Idle {
            self.buffer.replace(buffer);
            self.operation.set(Op::Read(len));
            self.mux.do_next_op();
            Ok(())
        } else {
            Err((Error::ArbitrationLost, buffer))
        }
    }
}
