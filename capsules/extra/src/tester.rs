// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2022.

// Test-only capsule: an app asks for an MPU region over one of six
// kernel-owned buffers, and learns whether Process::add_mpu_region()
// succeeded. The buffers live in kernel RAM (allocated by the board with
// static_init!), so they never overlap the app's own MPU regions.
//
// WARNING: this deliberately gives an app read/write access to kernel RAM.
// Test boards only.
//
//   command(DRIVER_NUM, 1, i, 0)   add MPU region over buffer i (0..6)
//       -> Success          region granted
//       -> Failure(NOMEM)   add_mpu_region() returned None
//       -> Failure(INVAL)   i out of range
//   command(DRIVER_NUM, 2, 0, 0)   -> Success(u32): run number (0, 1, 2, ...)
//       Increments on every call. Lives in the kernel, so it survives app
//       restarts; call it once at the start of main().
//   command(DRIVER_NUM, 3, i, 0)   -> Success(u32): address of buffer i

use core::cell::Cell;
use kernel::capabilities::ProcessManagementCapability;
use kernel::syscall::{CommandReturn, SyscallDriver};
use kernel::{ErrorCode, Kernel, ProcessId};

pub const DRIVER_NUM: usize = 0x99999;
pub const NUM_BUFS: usize = 6;
pub const BUF_SIZE: usize = 256; // power of two, aligned to its size

#[derive(Clone, Copy)]
#[repr(C, align(256))]
pub struct Buf(pub [u8; BUF_SIZE]);

pub struct MpuRegionTest<C: ProcessManagementCapability + 'static> {
    kernel: &'static Kernel,
    cap: C,
    // Only the addresses are used; the kernel never reads or writes these.
    bufs: &'static mut [Buf; NUM_BUFS],
    run: Cell<u32>,
}

impl<C: ProcessManagementCapability> MpuRegionTest<C> {
    pub fn new(kernel: &'static Kernel, cap: C, bufs: &'static mut [Buf; NUM_BUFS]) -> Self {
        Self {
            kernel,
            cap,
            bufs,
            run: Cell::new(0),
        }
    }
}

impl<C: ProcessManagementCapability> SyscallDriver for MpuRegionTest<C> {
    fn command(&self, cmd: usize, idx: usize, _: usize, pid: ProcessId) -> CommandReturn {
        match cmd {
            0 => CommandReturn::success(),
            1 | 3 if idx >= NUM_BUFS => CommandReturn::failure(ErrorCode::INVAL),
            1 => {
                let ptr = self.bufs[idx].0.as_ptr();
                let granted = self.kernel.process_map_or_external(
                    None,
                    pid,
                    |p| p.add_mpu_region(ptr, BUF_SIZE, BUF_SIZE),
                    &self.cap,
                );
                kernel::debug!(
                    "[mpu-test] add_mpu_region(buf {}) -> {}",
                    idx,
                    if granted.is_some() { "OK" } else { "NONE" }
                );
                match granted {
                    Some(_) => CommandReturn::success(),
                    None => CommandReturn::failure(ErrorCode::NOMEM),
                }
            }
            2 => {
                let run = self.run.get();
                self.run.set(run + 1);
                kernel::debug!("[mpu-test] ===== run {} =====", run);
                CommandReturn::success_u32(run)
            }
            3 => CommandReturn::success_u32(self.bufs[idx].0.as_ptr() as u32),
            _ => CommandReturn::failure(ErrorCode::NOSUPPORT),
        }
    }

    fn allocate_grant(&self, _: ProcessId) -> Result<(), kernel::process::Error> {
        Ok(())
    }
}
