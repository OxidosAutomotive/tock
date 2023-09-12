// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive SRL.
//
// Author: Ioan-Cristian CÎRSTEA <ioan.cirstea@oxidos.io>

#![deny(dead_code)]
#![deny(missing_docs)]
#![deny(unused_imports)]
//! HSE (high-speed external) clock driver for the STM32F4xx family. [^doc_ref]
//!
//! # Usage
//!
//! For the purposes of brevity, any error checking has been removed. In real applications, always
//! check the return values of the [Hse] methods.
//!
//! First, get a reference to the [Hse] struct:
//! ```rust,ignore
//! let hse = &peripherals.stm32f4.clocks.hse;
//! ```
//!
//! ## Start the clock
//!
//! ```rust,ignore
//! hse.enable();
//! ```
//!
//! ## Stop the clock
//!
//! ```rust,ignore
//! hse.disable();
//! ```
//!
//! ## Check if the clock is enabled
//! ```rust,ignore
//! if hse.is_enabled() {
//!     /* Do something */
//! } else {
//!     /* Do something */
//! }
//! ```
//!
//! ## Get the frequency of the clock
//! ```rust,ignore
//! let hsi_frequency_mhz = hse.get_frequency().unwrap();
//! ```
//!
//! [^doc_ref]: See 6.2.2 in the documentation.

use core::cell::Cell;

use crate::rcc::Rcc;

use kernel::debug;
use kernel::ErrorCode;

/// Type of external clock connected to the HSE block
#[derive(Copy, Clone, PartialEq)]
pub enum HSEClockType {
    /// Clock source (square wave) connected directly to OSC_IN
    Direct,
    /// Crystal oscillator connected to OSC_IN and OSC_OUT
    Crystal,
}

/// Main HSE clock structure
pub struct Hse<'a> {
    rcc: &'a Rcc,
    typ: Cell<Option<(HSEClockType, usize)>>,
}

impl<'a> Hse<'a> {
    /// Create a new instance of the HSE clock.
    ///
    /// # Parameters
    ///
    /// + rcc: an instance of [crate::rcc]
    /// + nominal_frequency: the nominal frequency of the external clock (in Hz)
    ///
    /// # Returns
    ///
    /// An instance of the HSE clock.
    pub(in crate::clocks) fn new(
        rcc: &'a Rcc,
        clock_setting: Option<(HSEClockType, usize)>,
    ) -> Self {
        Self {
            rcc,
            typ: Cell::from(clock_setting),
        }
    }

    /// Start the HSE clock.
    ///
    /// # Errors
    ///
    /// + [Err]\([ErrorCode::BUSY]\): if enabling the HSE clock took too long. Recall this method to
    /// ensure the HSI clock is running.
    pub fn enable(&self) -> Result<(), ErrorCode> {
        match self.typ.get() {
            None => return Err(ErrorCode::INVAL),
            Some((typ, _freq)) => self.rcc.set_hse_clock_type(&typ),
        }
        self.rcc.enable_hse_clock();

        for _ in 0..100 {
            if self.rcc.is_ready_hse_clock() {
                return Ok(());
            }
        }

        Err(ErrorCode::BUSY)
    }

    /// Stop the HSE clock.
    ///
    /// # Errors
    ///
    /// + [Err]\([ErrorCode::FAIL]\): if the HSI clock is configured as the system clock.
    /// + [Err]\([ErrorCode::BUSY]\): disabling the HSI clock took to long. Retry to ensure it is
    /// not running.
    pub fn disable(&self) -> Result<(), ErrorCode> {
        if self.rcc.is_hse_clock_system_clock() {
            return Err(ErrorCode::FAIL);
        }

        self.rcc.disable_hse_clock();

        // TODO: check if makes sense
        for _ in 0..10 {
            if self.rcc.is_ready_hse_clock() == false {
                return Ok(());
            }
        }

        Err(ErrorCode::BUSY)
    }

    /// Check whether the HSE clock is enabled or not.
    ///
    /// # Returns
    ///
    /// + [false]: the HSE clock is not enabled
    /// + [true]: the HSE clock is enabled
    pub fn is_enabled(&self) -> bool {
        self.rcc.is_enabled_hse_clock()
    }

    /// Get the frequency in MHz of the HSI clock.
    ///
    /// # Returns
    ///
    /// + [Some]\(frequency_hz\): if the HSI clock is enabled.
    /// + [None]: if the HSI clock is disabled.
    pub fn get_frequency(&self) -> Option<usize> {
        if let Some((_, freq)) = self.typ.get() {
            if self.is_enabled() {
                Some(freq)
            } else {
                None
            }
        } else {
            None
        }
    }
}

/// Tests for the HSI clock
///
/// This module ensures that the HSE clock works as expected. If changes are brought to the HSE
/// clock, ensure to run all the tests to see if anything is broken.
///
/// # Usage
///
/// First, import the [crate::clocks::hse] module in the desired board main file:
///
/// ```rust,ignore
/// use stm32f429zi::hse;
/// ```
///
/// Then, to run the tests, put the following line before [kernel::process::load_processes]:
///
/// ```rust,ignore
/// hse::tests::run_all(&peripherals.stm32f4.clocks.hse);
/// ```
///
/// If everything works as expected, the following message should be printed on the kernel console:
///
/// ```text
/// ===============================================
/// Testing HSE...
/// Finished testing HSE. Everything is alright!
/// ===============================================
/// ```
///
/// **NOTE:** All these tests assume default boot configuration.
pub mod tests {
    use super::*;

    /// Run the entire test suite.
    pub fn run(hse: Hse) {
        debug!("");
        debug!("===============================================");
        debug!("Testing HSI...");

        // By default, the HSE clock is disabled
        assert_eq!(false, hse.is_enabled());

        // Enabling the HSE clock should succeed
        assert_eq!(Ok(()), hse.enable());

        assert_eq!(Some(hse.typ.get().unwrap().1), hse.get_frequency());

        debug!("Finished testing HSE. Everything is alright!");
        debug!("===============================================");
        debug!("");
    }
}
