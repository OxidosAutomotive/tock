// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

//! Provides userspace with access to sound_pressure sensors.
//!
//! Userspace Interface
//! -------------------
//!
//! ### `subscribe` System Call
//!
//! The `subscribe` system call supports the single `subscribe_number` zero,
//! which is used to provide a callback that will return back the result of
//! a sound_pressure sensor reading.
//! The `subscribe`call return codes indicate the following:
//!
//! * `Ok(())`: the callback been successfully been configured.
//! * `ENOSUPPORT`: Invalid allow_num.
//! * `NOMEM`: No sufficient memory available.
//! * `INVAL`: Invalid address of the buffer or other error.
//!
//!
//! ### `command` System Call
//!
//! The `command` system call support one argument `cmd` which is used to specify the specific
//! operation, currently the following cmd's are supported:
//!
//! * `0`: check whether the driver exist
//! * `1`: read the sound_pressure
//!
//!
//! The possible return from the 'command' system call indicates the following:
//!
//! * `Ok(())`:    The operation has been successful.
//! * `BUSY`:      The driver is busy.
//! * `ENOSUPPORT`: Invalid `cmd`.
//! * `NOMEM`:     No sufficient memory available.
//! * `INVAL`:     Invalid address of the buffer or other error.
//!
//! Usage
//! -----
//!
//! You need a device that provides the `hil::sensors::SoundPressure` trait.
//!
//! ```rust,ignore
//! # use kernel::static_init;
//!
//! let grant_cap = create_capability!(capabilities::MemoryAllocationCapability);
//! let grant_sound_pressure = board_kernel.create_grant(&grant_cap);
//!
//! let temp = static_init!(
//!        capsules::sound_pressure::SoundPressureSensor<'static>,
//!        capsules::sound_pressure::SoundPressureSensor::new(si7021,
//!                                                 board_kernel.create_grant(&grant_cap)));
//!
//! kernel::hil::sensors::SoundPressure::set_client(si7021, temp);
//! ```

use core::cell::Cell;
use core::marker::PhantomData;
use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
use kernel::hil;
use kernel::hil::sensors::{SoundPressure, SoundPressureClient};
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::{ErrorCode, ProcessId};

/// Syscall driver number.
use capsules_core::driver;
pub const DRIVER_NUM: usize = driver::NUM::SoundPressure as usize;

#[derive(Default)]
pub struct App {
    subscribed: bool,
    enable: bool,
}

#[macro_export]
macro_rules! sound_pressure_driver {
    (@name => $name: ident, @sound_pressure => $sp: ty) => {
        use capsules_extra::sound_pressure::{App, SoundPressureSensor};
        use kernel::grant::{AllowRoCount, AllowRwCount, Grant, UpcallCount};
        use kernel::hil;
        use kernel::syscall::{CommandReturn, SyscallDriver};
        use kernel::{ErrorCode, ProcessId};

        #[repr(transparent)]
        struct $name<'a> {
            sound_pressure: SoundPressureSensor<'a, $sp, $name<'a>>,
        }

        impl<'a> $name<'a> {
            #[inline]
            pub fn new(
                driver: &'a $sp,
                grant: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<0>>,
            ) -> $name<'a> {
                $name {
                    sound_pressure: SoundPressureSensor::new(driver, grant),
                }
            }
        }

        impl<'a> hil::sensors::SoundPressureClient for $name<'a> {
            #[inline]
            fn callback(&self, ret: Result<(), ErrorCode>, sound_val: u8) {
                self.sound_pressure.callback(ret, sound_val)
            }
        }

        impl<'a> SyscallDriver for $name<'a> {
            #[inline]
            fn command(
                &self,
                command_num: usize,
                data1: usize,
                data2: usize,
                processid: ProcessId,
            ) -> CommandReturn {
                self.sound_pressure
                    .command(command_num, data1, data2, processid)
            }

            #[inline]
            fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
                self.sound_pressure.allocate_grant(processid)
            }
        }
    };
}

pub struct SoundPressureSensor<'a, S: SoundPressure<'a, C>, C: SoundPressureClient> {
    driver: &'a S,
    apps: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<0>>,
    busy: Cell<bool>,
    _client: PhantomData<C>,
}

impl<'a, S: SoundPressure<'a, C>, C: SoundPressureClient> SoundPressureSensor<'a, S, C> {
    pub fn new(
        driver: &'a S,
        grant: Grant<App, UpcallCount<1>, AllowRoCount<0>, AllowRwCount<0>>,
    ) -> SoundPressureSensor<'a, S, C> {
        SoundPressureSensor {
            driver,
            apps: grant,
            busy: Cell::new(false),
            _client: PhantomData,
        }
    }

    fn enqueue_command(&self, processid: ProcessId) -> CommandReturn {
        self.apps
            .enter(processid, |app, _| {
                if !self.busy.get() {
                    app.subscribed = true;
                    self.busy.set(true);
                    let res = self.driver.read_sound_pressure();
                    if let Ok(err) = ErrorCode::try_from(res) {
                        CommandReturn::failure(err)
                    } else {
                        CommandReturn::success()
                    }
                } else {
                    CommandReturn::failure(ErrorCode::BUSY)
                }
            })
            .unwrap_or_else(|err| CommandReturn::failure(err.into()))
    }

    fn enable(&self) {
        let mut enable = false;
        for app in self.apps.iter() {
            app.enter(|app, _| {
                if app.enable {
                    enable = true;
                }
            });
            if enable {
                let _ = self.driver.enable();
            } else {
                let _ = self.driver.disable();
            }
        }
    }
}

impl<'a, S: SoundPressure<'a, C>, C: SoundPressureClient> hil::sensors::SoundPressureClient
    for SoundPressureSensor<'a, S, C>
{
    fn callback(&self, ret: Result<(), ErrorCode>, sound_val: u8) {
        for cntr in self.apps.iter() {
            cntr.enter(|app, upcalls| {
                if app.subscribed {
                    self.busy.set(false);
                    app.subscribed = false;
                    if ret == Ok(()) {
                        upcalls.schedule_upcall(0, (sound_val.into(), 0, 0)).ok();
                    }
                }
            });
        }
    }
}

impl<'a, S: SoundPressure<'a, C>, C: SoundPressureClient> SyscallDriver
    for SoundPressureSensor<'a, S, C>
{
    fn command(
        &self,
        command_num: usize,
        _: usize,
        _: usize,
        processid: ProcessId,
    ) -> CommandReturn {
        match command_num {
            // check whether the driver exists!!
            0 => CommandReturn::success(),

            // read sound_pressure
            1 => self.enqueue_command(processid),

            // enable
            2 => {
                let res = self
                    .apps
                    .enter(processid, |app, _| {
                        app.enable = true;
                        CommandReturn::success()
                    })
                    .map_err(ErrorCode::from);
                if let Err(e) = res {
                    CommandReturn::failure(e)
                } else {
                    self.enable();
                    CommandReturn::success()
                }
            }

            // disable
            3 => {
                let res = self
                    .apps
                    .enter(processid, |app, _| {
                        app.enable = false;
                        CommandReturn::success()
                    })
                    .map_err(ErrorCode::from);
                if let Err(e) = res {
                    CommandReturn::failure(e)
                } else {
                    self.enable();
                    CommandReturn::success()
                }
            }
            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, processid: ProcessId) -> Result<(), kernel::process::Error> {
        self.apps.enter(processid, |_, _| {})
    }
}
