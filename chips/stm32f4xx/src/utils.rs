//! Copyright 2022 OxidOS Automotive SRL
//!
//! OxidOS Automotive SRL Confidential. This software is owned or controlled by
//!  OxidOS Automotive SRL and may only be used strictly in accordance with the
//!  applicable license terms.  By expressly accepting such terms or by
//! downloading, installing, activating and/or otherwise using the software, you
//!  are agreeing that you have read, and that you agree to comply with and are
//! bound by, such license terms.  If you do not agree to be bound by the
//! applicable license terms, then you may not retain, install, activate or
//! otherwise use the software.
//!
//! Author: Felix Mada <felix.mada@oxidos.io>

// use cortex_m_semihosting::hprintln;

// poll function `check_condition`, for a maximum of 'times` times,
// until it returns true. otherwise will return the Error from 'handle_error`
// Use:
//    wait_for_condition(1_000, || is_clk_stable(), || ClockError::Timeout)?;
//        checks `is_clk_stable` for a maximum of 1_000 times,
//        until it return true, otherwise returns Err(ClockError::Timeout)
#[inline(always)]
pub fn wait_for_condition<E>(
    times: u32,
    check_condition: impl Fn() -> bool,
    handle_error: impl FnOnce() -> E,
) -> Result<(), E> {
    let mut a: u32 = 0;
    while a < times {
        if check_condition() {
            // hprintln!("waited {}", a);
            return Ok(());
        }
        a = a + 1;
    }
    // hprintln!("waited and failed {}", a);
    Err(handle_error())
}

pub fn wait_for_eq<T: core::fmt::Display + core::cmp::PartialEq, E>(
    times: u32,
    left_side: impl Fn() -> T,
    right_side: T,
    handle_error: impl FnOnce() -> E,
) -> Result<(), E> {
    for _ in 0..times {
        if left_side() == right_side {
            // hprintln!("waited {}", a);
            return Ok(());
        }
    }
    // hprintln!("wait_for_eq: {} = {}", left_side(), right_side);
    Err(handle_error())
}

// small blocking delay
#[inline(never)]
pub fn wait(times: u32) {
    let mut count: u32 = times;
    while count > 0 {
        count = count - 1;
    }
}
