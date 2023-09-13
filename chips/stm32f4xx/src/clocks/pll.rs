// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive SRL.
//
// Author: Ioan-Cristian CÎRSTEA <ioan.cirstea@oxidos.io>
// Author: Felix-Marius Mada <felix.mada@oxidos.io>

#![deny(dead_code)]
#![deny(missing_docs)]
#![deny(unused_imports)]
//! Main phase-locked loop (PLL) clock driver for the STM32F4xx family. [^doc_ref]
//!
//! Many boards of the STM32F4xx family provide several PLL clocks. However, all of them have a
//! main PLL clock. This driver is designed for the main PLL clock. It will be simply referred as
//! the PLL clock.
//!
//! The PLL clock is composed of two outputs:
//!
//! + the main one used for the system clock
//! + the PLL48CLK used for USB OTG FS, the random number generator and SDIO clocks
//!
//! # Implemented features
//!
//! - [x] Default configuration of 96MHz with reduced PLL jitter
//! - [x] 1Hz frequency precision
//! - [x] Support for 13-216MHz frequency range
//! - [x] Support for PLL48CLK output
//!
//! # Missing features
//!
//! - [ ] Control over the second (PLLI2S) and third (PLLSAI) PLLs (not the second output of the first PLL)
//!
//! # Usage
//!
//! For the purposes of brevity, any error checking has been removed. In real applications, always
//! check the return values of the [Pll] methods.
//!
//! First, get a reference to the [Pll] struct:
//! ```rust,ignore
//! let pll = &peripherals.stm32f4.clocks.pll;
//! ```
//!
//! ## Start the clock with a given frequency
//!
//! ```rust,ignore
//! pll.prepare_frequnecy(100_000_000, HSI_FREQUENCY_MHZ, PllSource::HSI); // 100 MHz output from the internal 16MHz oscillator (HSI)
//! pll.enable();
//! ```
//!
//! ## Precisely control all the PLL registers
//! pll.configure_pll( PllSource::HSI,   // PLL source
//!                     16_000_000,      // input frequenncy
//!                     PLLP::DivideBy8, // input divider
//!                     100,             // multiplier
//!                     4,               // main output divider
//!                     PLLQ::DivideBy4, // secondary output divider
//!                     1,               // PWR::VOS voltage scaling (see RefMan ch3.5)
//!                     false            // PWR::overdrive voltge (see RefMan ch3.5)
//! );
//!
//! ## Stop the clock
//!
//! ```rust,ignore
//! pll.disable();
//! ```
//!
//! ## Check whether the PLL clock is running or not
//! ```rust,ignore
//! if pll.is_enabled() {
//!     // do something...
//! } else {
//!     // do something...
//! }
//! ```
//!
//! ## Check the clock frequency
//!
//! ```rust,ignore
//! let optional_pll_frequency = pll.get_frequency();
//! if let None = optional_pll_frequency {
//!     /* Clock stopped */
//! }
//! let pll_frequency = optional_pll_frequency.unwrap();
//! /* Computations based on the PLL frequency */
//! ```
//!
//! ## Reconfigure the clock once started
//!
//! ```rust,ignore
//! pll.disable(); // The PLL clock can't be configured while running
//! pll.set_frequency(50); // 50MHz
//! pll.enable();
//! ```
//!
//! ## Configure the PLL clock so that PLL48CLK output is correctly calibrated
//! ```rust,ignore
//! // The frequency of the PLL clock must be 1, 1.5, 2, 2.5, 3, 3.5 or 4 x 48MHz in order to get
//! // 48MHz output. Otherwise, the driver will attempt to get the closest frequency lower than 48MHz
//! pll..prepare_frequnecy(72_000_000, HSI_FREQUENCY_MHZ, Hsi); // 72MHz = 48MHz * 1.5
//! pll.enable();
//! ```
//!
//! ## Check if the PLL48CLK output is calibrated.
//! ```rust,ignore
//! if !pll.is_pll48_calibrated() {
//!     /* Handle the case when it is not calibrated */
//! }
//! ```
//!
//! ## Get the frequency of the PLL48CLK output
//!
//! ```rust,ignore
//! let optional_pll48_frequency = pll.get_frequency();
//! if let None = optional_pll48_frequency {
//!     /* Clock stopped */
//! }
//! let pll48_frequency = optional_pll48_frequency.unwrap();
//! ```
//!
//! [^doc_ref]: See 6.2.3 in the documentation.

use crate::chip_specific::clock_constants::pll_constants::PLL_MIN_FREQ;
use crate::chip_specific::pwr_specific::get_vos_overdrive_basedon_on_frequency;
use crate::chip_specific::pwr_specific::VoltageScaling;
use crate::clocks::hsi::HSI_FREQUENCY;
use crate::rcc::PllSource;
use crate::rcc::Rcc;
use crate::rcc::SysClockSource;
use crate::rcc::RESET_PLLM_VALUE;
use crate::rcc::RESET_PLLN_VALUE;
use crate::rcc::RESET_PLLP_VALUE;
use crate::rcc::RESET_PLLQ_VALUE;
use crate::rcc::{PLLP, PLLQ};

use kernel::debug;
use kernel::utilities::cells::OptionalCell;
use kernel::ErrorCode;

use core::cell::Cell;

use super::pwr::Pwr;

/// Main PLL clock structure.
pub struct Pll<'a> {
    rcc: &'a Rcc,
    pwr: Pwr,
    main_frequency: OptionalCell<usize>,
    pll48_frequency: OptionalCell<usize>,
    pll48_calibrated: Cell<bool>,
    /// FLAG: pwr::overdrive needs to be enabled while pll is locking
    overdrive_needed: Cell<bool>,
}

/// PLL max frequency in MHz
pub const PLL_MAX_FREQ: usize = 216_000_000;

/// PLL frequency limit values (minimum and maximum)
pub mod limits {
    pub use super::PLL_MAX_FREQ;
    pub use crate::chip_specific::clock_constants::pll_constants::PLL_MIN_FREQ;
}

impl<'a> Pll<'a> {
    // Create a new instance of the PLL clock.
    //
    // The instance of the PLL clock is configured to run at 96MHz and with minimal PLL jitter
    // effects.
    //
    // # Parameters
    //
    // + rcc: an instance of [crate::rcc]
    //
    // # Returns
    //
    // An instance of the PLL clock.
    pub(in crate::clocks) fn new(rcc: &'a Rcc, pwr: Pwr) -> Self {
        Self {
            rcc,
            main_frequency: OptionalCell::new(
                HSI_FREQUENCY / RESET_PLLM_VALUE * RESET_PLLN_VALUE
                    / Into::<usize>::into(RESET_PLLP_VALUE),
            ),
            pll48_frequency: OptionalCell::new(
                HSI_FREQUENCY / RESET_PLLM_VALUE * RESET_PLLN_VALUE / (RESET_PLLQ_VALUE as usize),
            ),
            // reset values for PLL result in 48MHz on secondary output
            pll48_calibrated: Cell::new(true),
            overdrive_needed: Cell::new(false),
            pwr,
        }
    }

    /* Public functions for controling PLL clock */

    /// Start the PLL clock.
    ///
    /// # Errors
    ///
    /// + [Err]\([ErrorCode::BUSY]\): if enabling the PLL clock took too long. Recall this method to
    /// ensure the PLL clock is running.
    pub fn enable(&self) -> Result<(), ErrorCode> {
        // RefMan 5.1.4 Entering Over-drive mode step 2
        // Enable the PLL clock
        self.rcc.enable_pll_clock();

        // RefMan 5.1.4 Entering Over-drive mode steps 3-5
        if self.overdrive_needed.get() {
            self.pwr.enable_overdrive().map_err(|_| ErrorCode::FAIL)?
        }

        // Wait until the PLL clock is locked.
        // 200 was obtained by running tests in release mode
        for _ in 0..200 {
            if self.rcc.is_locked_pll_clock() {
                return Ok(());
            }
        }

        // If waiting for the PLL clock took too long, return ErrorCode::BUSY
        Err(ErrorCode::BUSY)
    }

    /// Stop the PLL clock.
    ///
    /// # Errors
    ///
    /// + [Err]\([ErrorCode::FAIL]\): if the PLL clock is configured as the system clock.
    /// + [Err]\([ErrorCode::BUSY]\): disabling the PLL clock took to long. Retry to ensure it is
    /// not running.
    pub fn disable(&self) -> Result<(), ErrorCode> {
        // Can't disable the PLL clock when it is used as the system clock
        if self.rcc.get_sys_clock_source() == SysClockSource::PLL {
            return Err(ErrorCode::FAIL);
        }

        // Disable the PLL clock
        self.rcc.disable_pll_clock();
        Ok(())
    }

    /// Check whether the PLL clock is enabled or not.
    ///
    /// # Returns
    ///
    /// + [false]: the PLL clock is not enabled
    /// + [true]: the PLL clock is enabled
    pub fn is_enabled(&self) -> bool {
        self.rcc.is_enabled_pll_clock()
    }

    /// Get the frequency in Hz of the PLL clock.
    ///
    /// # Returns
    ///
    /// + [Some]\(frequency_hz\): if the PLL clock is enabled.
    /// + [None]: if the PLL clock is disabled.
    pub fn get_frequency(&self) -> Option<usize> {
        if self.is_enabled() {
            self.main_frequency.get()
        } else {
            None
        }
    }

    /// Get the frequency in Hz of the PLL48 clock.
    ///
    /// **NOTE:** If the PLL clock was not configured with a frequency multiple of 48MHz, the
    /// returned value is inaccurate.
    ///
    /// # Returns
    ///
    /// + [Some]\(frequency_hz\): if the PLL clock is enabled.
    /// + [None]: if the PLL clock is disabled.
    pub fn get_frequency_pll48(&self) -> Option<usize> {
        if self.is_enabled() {
            self.pll48_frequency.get()
        } else {
            None
        }
    }

    /* Public functions for configuring the PLL */

    /// Computes the PLL configuration for the desired frequency.
    ///
    /// The PLL clock has two outputs:
    ///
    /// + main output used for configuring the system clock
    /// + a second output called PLL48CLK used by OTG USB FS (48MHz), the random number generator
    /// (≤ 48MHz) and the SDIO (≤ 48MHz) clocks.
    ///
    /// When calling this method, the given frequency is set for the main output. The method will
    /// attempt to configure the PLL48CLK output to 48MHz, or to the highest value less than 48MHz
    /// if it is not possible to get a precise 48MHz. In order to obtain a precise 48MHz frequency
    /// (for the OTG USB FS peripheral), one should call this method with a frequency of 1, 1.5, 2,
    /// 2.5 ... 4 x 48MHz.
    ///
    /// # Parameters
    ///
    /// + desired_frequency: the desired frequency in Hz. Supported values: 24-216MHz for
    /// STM32F401 and 13-216MHz for all the other chips
    ///
    /// # Errors
    ///
    /// + [Err]\([ErrorCode::INVAL]\): if the desired frequency can't be achieved
    /// + [Err]\([ErrorCode::FAIL]\): if the PLL clock is already enabled. It must be disabled before
    /// configuring it.
    pub fn prepare_frequnecy(
        &self,
        desired_frequency: usize,
        input_frequency: usize,
        source: PllSource,
    ) -> Result<(), ErrorCode> {
        // Check for errors:
        // + PLL clock running
        // + invalid frequency
        if self.rcc.is_enabled_pll_clock() {
            return Err(ErrorCode::FAIL);
        } else if desired_frequency < PLL_MIN_FREQ || desired_frequency > PLL_MAX_FREQ {
            return Err(ErrorCode::INVAL);
        }

        let voltage_scale = get_vos_overdrive_basedon_on_frequency(desired_frequency);
        let (voltage_scale, overdrive_needed) = voltage_scale.ok_or(ErrorCode::INVAL)?;

        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Self::compute_dividers(input_frequency, desired_frequency);

        self.configure_pll_internal(
            source,
            pllp,
            plln,
            pllm,
            pllq,
            voltage_scale,
            overdrive_needed,
        )?;

        // Compute and cache the resulting frequencies so it is not computed every time a get method is called
        let main_output_frequency = vco_output_frequency / Into::<usize>::into(pllp);
        let secondary_output_frequency = vco_output_frequency / pllq as usize;
        let secondary_output_calibrated = secondary_output_frequency % 48_000_000 == 0;

        self.set_frequency(
            main_output_frequency,
            Some((secondary_output_frequency, secondary_output_calibrated)),
        );

        Ok(())
    }

    /// Fine control for PLL configuration
    /// This function will try to configure the resiters and also update the `main_frequnecy` and `pll48_frequency` fields.
    /// The output frequencies for the PLL clock is computed as following:
    /// VCO input frequency = Source frequency / PLLM  (must range from 1MHz to 2MHz)
    /// VCO output frequency = VCO input frequency * PLLN (must range from 100MHz to 432MHz)
    /// PLL output frequency = VCO output frequency / PLLP
    /// PLL48CLK = VCO output frequency / PLLQ
    /// Voltage scaling and voltage overdrive are needed for certain high frequencies, according to the reference manual.
    /// # Parameters
    ///
    /// + input_frequency: input frequnecy for the PLL
    /// + source: input clock to the PLL (HSI or HSE)
    /// + input_divider, multiplier, main_output_divider, secondary_output_divider: configure the PLL clock dividers
    /// + voltage scaling, voltage_overdrive: for certain frequencies the PLL voltage supply needs to be configured according to the reference manual
    ///
    /// # Errors:
    ///
    /// + [Err]\([ErrorCode::FAIL]\): if the PLL is already running, can't be configured unless it is stopped, or if the voltage scaling process fails
    pub fn configure_pll(
        &self,
        source: PllSource,
        input_frequnecy: usize,
        input_divider: PLLP,
        multipler: usize,
        main_output_divider: usize,
        secondary_output_divider: PLLQ,
        voltage_scaling: u8,
        voltage_overdrive: bool,
    ) -> Result<(), ErrorCode> {
        let voltage_scaling = match voltage_scaling {
            1 => VoltageScaling::Scale1,
            2 => VoltageScaling::Scale2,
            3 => VoltageScaling::Scale3,
            _ => return Err(ErrorCode::INVAL),
        };

        self.configure_pll_internal(
            source,
            input_divider,
            multipler,
            main_output_divider,
            secondary_output_divider,
            voltage_scaling,
            voltage_overdrive,
        )?;
        let vco_output_frequency =
            (input_frequnecy / Into::<usize>::into(input_divider)) * multipler;
        let main_output_frequnecy = vco_output_frequency / main_output_divider;
        let secondary_output_frequency = vco_output_frequency / secondary_output_divider as usize;
        let secondary_output_calibrated = secondary_output_frequency % 48_000_000 == 0;

        self.set_frequency(
            main_output_frequnecy,
            Some((secondary_output_frequency, secondary_output_calibrated)),
        );
        Ok(())
    }

    /* Private functions for configuring the PLL */

    /// Fine control for PLL configuration, without changin the `main_frequency` and `pll48_frequency` fields.
    /// # Parameters
    ///
    /// + source: input clock to the PLL (HSI or HSE)
    /// + input_divider, multiplier, main_output_divider, secondary_output_divider: configure the PLL clock dividers
    /// + voltage scaling, voltage_overdrive: for certain frequencies the PLL voltage supply needs to be configured according to the reference manual
    ///
    /// # Errors:
    ///
    /// + [Err]\([ErrorCode::FAIL]\): if the PLL is already running, it can't be configured unless it is stopped, or if the voltage scaling process fails
    fn configure_pll_internal(
        &self,
        source: PllSource,
        input_divider: PLLP,
        multipler: usize,
        main_output_divider: usize,
        secondary_output_divider: PLLQ,
        voltage_scaling: VoltageScaling,
        voltage_overdrive: bool,
    ) -> Result<(), ErrorCode> {
        // Check for errors:
        // + PLL clock running
        // + invalid frequency
        if self.rcc.is_enabled_pll_clock() {
            return Err(ErrorCode::FAIL);
        }

        self.rcc.set_pll_clocks_source(source);

        // save a flag for overdrive
        self.overdrive_needed.set(voltage_overdrive);
        if voltage_overdrive {
            // disable all peripherals except pwr in order to be able to enable overdrive
            // RefMan 5.1.4 Note: During the Over-drive switch activation, no peripheral clocks should be enabled. The peripheral clocks must be enabled once the Over-drive mode is activated.
            self.rcc.disable_all_peripherals_except_pwr();
        }

        // change voltage scaling based on desired frequency
        self.pwr
            .configure_voltage_scaling(voltage_scaling)
            .map_err(|_| ErrorCode::FAIL)?;

        self.rcc
            .set_pll_main_dividers(main_output_divider, multipler, input_divider);
        self.rcc.set_pll_clock_q_divider(secondary_output_divider);

        Ok(())
    }

    /// Sets the frequency for the getter functions, doesn't actually configure the PLL registers
    fn set_frequency(&self, main_frequency: usize, secondary_frequnecy: Option<(usize, bool)>) {
        self.main_frequency.set(main_frequency);
        match secondary_frequnecy {
            Some((freq, calibrated)) => {
                self.pll48_frequency.set(freq);
                self.pll48_calibrated.set(calibrated);
            }
            None => {
                self.main_frequency.clear();
                self.pll48_frequency.clear();
                self.pll48_calibrated.set(false);
            }
        }
    }

    /// Check if the PLL48 clock is calibrated (its output is exactly 48MHz).
    ///
    /// A frequency of 48MHz is required for USB OTG FS.
    ///
    /// # Returns
    ///
    /// + [true]: the PLL48 clock frequency is exactly 48MHz.
    /// + [false]: the PLL48 clock is not exactly 48MHz.
    pub fn is_pll48_calibrated(&self) -> bool {
        self.pll48_calibrated.get()
    }

    /// computes ceil(x/y), usize::div_ceil form core lib is currently unstable in rust
    const fn div_ceil(x: usize, y: usize) -> usize {
        (x + y - 1) / y
    }

    /// computes a PLLM divider value such that:
    ///  + input_frequency / PLLM is between 1MHz and 2MHz, preferably 2MHz [RefMan 6.3.2 PLLCFGR::PLLM]
    ///  + PLLM is between 63 and 2, values under 2 are invalid for the HW
    /// # Returns
    /// (pllm, vco_input_frequency): returns the computed pllm divider and the VCO input_frequency
    fn compute_pllm(input_frequency: usize) -> (usize, usize) {
        let div = Self::div_ceil(input_frequency, 2_000_000);
        let divider = div.min(63).max(2);
        let vco_input_frequency = input_frequency / divider;
        (divider, vco_input_frequency)
    }

    // The caller must ensure the desired frequency lies between PLL_MIN_FREQ_MHZ and PLL_MAX_FREQ_MHZ. Otherwise, the
    // return value makes no sense.
    const fn compute_pllp(desired_frequency: usize) -> PLLP {
        if desired_frequency < 55_000_000 {
            PLLP::DivideBy8
        } else if desired_frequency < 73_000_000 {
            PLLP::DivideBy6
        } else if desired_frequency < 109_000_000 {
            PLLP::DivideBy4
        } else {
            PLLP::DivideBy2
        }
    }

    // The caller must ensure the desired frequency lies between PLL_MIN_FREQ_MHZ and PLL_MAX_FREQ_MHZ. Otherwise, the
    // return value makes no sense.
    fn compute_plln(desired_frequency: usize, vco_input_frequency: usize, pllp: PLLP) -> usize {
        desired_frequency * Into::<usize>::into(pllp) / vco_input_frequency
    }

    // The caller must ensure the VCO output frequency lies between 100 and 432MHz. Otherwise, the
    // return value makes no sense.
    fn compute_pllq(vco_output_frequency: usize) -> PLLQ {
        for pllq in 3..10 {
            if 48_000_000 * pllq >= vco_output_frequency {
                return match pllq {
                    3 => PLLQ::DivideBy3,
                    4 => PLLQ::DivideBy4,
                    5 => PLLQ::DivideBy5,
                    6 => PLLQ::DivideBy6,
                    7 => PLLQ::DivideBy7,
                    8 => PLLQ::DivideBy8,
                    _ => PLLQ::DivideBy9,
                };
            }
        }
        unreachable!("The previous for loop should always return");
    }

    fn compute_dividers(
        input_frequency: usize,
        desired_frequency: usize,
    ) -> (usize, PLLP, usize, PLLQ, usize) {
        // The output frequencies for the PLL clock is computed as following:
        // Source frequency / PLLM = VCO input frequency (must range from 1MHz to 2MHz)
        // VCO output frequency = VCO input frequency * PLLN (must range from 100MHz to 432MHz)
        // PLL output frequency = VCO output frequency / PLLP
        // PLL48CLK = VCO output frequency / PLLQ

        let (pllm, vco_input_frequency) = Self::compute_pllm(input_frequency);

        // Compute PLLP
        let pllp = Self::compute_pllp(desired_frequency);

        // Compute PLLN
        let plln = Self::compute_plln(desired_frequency, vco_input_frequency, pllp);

        // Compute PLLQ
        let vco_output_frequency = vco_input_frequency * plln;
        let pllq = Self::compute_pllq(vco_output_frequency);
        (pllm, pllp, plln, pllq, vco_output_frequency)
    }
}

/// Tests for the PLL clock
///
/// This module ensures that the PLL clock works as expected. If changes are brought to the PLL
/// clock, ensure to run all the tests to see if anything is broken.
///
/// # Usage
///
/// First, import the [crate::clocks::pll] module inside the board main file:
///
/// ```rust,ignore
/// use stm32f429zi::pll;
/// ```
/// To run all the available tests, add this line before **kernel::process::load_processes()**:
///
/// ```rust,ignore
/// pll::tests::run_all(&peripherals.stm32f4.clocks.pll);
/// ```
///
/// If everything works as expected, the following message should be printed on the kernel console:
///
/// ```text
/// ===============================================
/// Testing PLL...
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Testing PLL configuration...
/// Finished testing PLL configuration.
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Testing PLL struct...
/// Finished testing PLL struct.
/// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
/// Finished testing PLL. Everything is alright!
/// ===============================================
/// ```
///
/// There is also the possibility to run a part of the test suite. Check the functions present in
/// this module for more details.
///
/// # Errors
///
/// If there are any errors, open an issue ticket at <https://github.com/tock/tock>. Please provide the
/// output of the test execution.
pub mod tests {

    use super::*;

    fn compute_output_freq(
        input_freq: usize,
        input_divider: usize,
        multiplier: usize,
        output_divider: usize,
    ) -> usize {
        let vco_input_frequency = input_freq / input_divider;
        if !(1_000_000..=2_000_000).contains(&vco_input_frequency) {
            panic!(
                "vco input frequency {} out of range (1..=2MHz)",
                vco_input_frequency
            );
        }
        if !(50..=432).contains(&multiplier) {
            panic!("multiplier {} out of range (50..=432)", multiplier);
        }
        let vco_output_frequency = vco_input_frequency * multiplier;
        if !(100_000_000..432_000_001).contains(&vco_output_frequency) {
            panic!(
                "vco output frequency {} out of range (100..=432MHz)",
                vco_output_frequency
            );
        }
        ((input_freq / input_divider) * multiplier) / output_divider
    }

    /// Test if the configuration parameters are correctly computed for a given frequency.
    ///
    /// # Usage
    ///
    /// ```rust,ignore
    /// use stm32f429zi::pll; // Import the pll module
    /// /* Code goes here */
    /// pll::test::test_pll_config(&peripherals.stm32f4.pll); // Run the tests
    /// ```
    pub fn test_pll_config() {
        debug!("Testing PLL configuration...");

        /*  CASE 1. 24MHz --> minimum value */
        let desired_output = 24_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(16_000_000, desired_output);

        let computed_output = compute_output_freq(16_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );

        // the input_frequency is 16MHz, ideal PLL input is 2MHz => ideal divider is 8
        assert_eq!(pllm, 8);

        /* CASE 2. 25MHz --> minimum required value for Ethernet devices */
        let desired_output = 25_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(16_000_000, desired_output);

        let computed_output = compute_output_freq(16_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );

        // the input_frequency is 16MHz, ideal PLL input is 2MHz => ideal divider is 8
        assert_eq!(pllm, 8);

        /* CASE 3. 54MHz */
        let desired_output = 54_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(16_000_000, desired_output);

        let computed_output = compute_output_freq(16_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );

        // the input_frequency is 16MHz, ideal PLL input is 2MHz => ideal divider is 8
        assert_eq!(pllm, 8);

        /* CASE 4. 108MHz */
        let desired_output = 108_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(16_000_000, desired_output);

        let computed_output = compute_output_freq(16_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );

        // the input_frequency is 16MHz, ideal PLL input is 2MHz => ideal divider is 8
        assert_eq!(pllm, 8);

        /* CASE 5. 180MHz --> Max frequency for the CPU */
        let desired_output = 180_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(16_000_000, desired_output);

        let computed_output = compute_output_freq(16_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );

        // the input_frequency is 16MHz, ideal PLL input is 2MHz => ideal divider is 8
        assert_eq!(pllm, 8);

        /* CASE 6. 108MHz from 20MHz input clock*/
        let desired_output = 108_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(20_000_000, desired_output);

        let computed_output = compute_output_freq(20_000_000, pllm, plln, pllp.into());
        assert_eq!(
            computed_output,
            desired_output,
            "resulting dividers {} {} {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
        );
        // the input_frequency is 20MHz, ideal PLL input is 2MHz => ideal divider is 10
        assert_eq!(pllm, 10);

        /* CASE 7. 108MHz from 17MHz input clock*/
        let desired_output = 108_000_000;
        let (pllm, pllp, plln, pllq, vco_output_frequency) =
            Pll::compute_dividers(17_000_000, desired_output);

        let computed_output = compute_output_freq(17_000_000, pllm, plln, pllp.into());
        assert!(
            (107_000_000..=109_000_000).contains(&computed_output),
            "resulting dividers {} {} {}: resulting frequency {}",
            pllm,
            plln,
            Into::<usize>::into(pllp),
            computed_output
        );

        debug!("Finished testing PLL configuration.");
    }

    /// Check if the PLL works as expected.
    ///
    /// **NOTE:** it is highly recommended to call [test_pll_config]
    /// first to check whether the configuration parameters are correctly computed.
    ///
    /// # Usage
    ///
    /// ```rust,ignore
    /// use stm32f429zi::pll; // Import the PLL module
    /// /* Code goes here */
    /// pll::test::test_pll_struct(&peripherals.stm32f4.pll); // Run the tests
    /// ```
    pub fn test_pll_struct<'a>(pll: &'a Pll<'a>) {
        debug!("Testing PLL struct...");
        // Make sure the PLL clock is disabled
        assert_eq!(Ok(()), pll.disable());
        assert_eq!(false, pll.is_enabled());

        // Attempting to configure the PLL with either too high or too low frequency
        assert_eq!(
            Err(ErrorCode::INVAL),
            pll.prepare_frequnecy(12_000_000, HSI_FREQUENCY, PllSource::HSI)
        );
        assert_eq!(
            Err(ErrorCode::INVAL),
            pll.prepare_frequnecy(217_000_000, HSI_FREQUENCY, PllSource::HSI)
        );

        // prepare the PLL with a configuration similar to the default configuration
        assert_eq!(
            Ok(()),
            pll.prepare_frequnecy(96_000_000, HSI_FREQUENCY, PllSource::HSI)
        );

        // Start the PLL with the default configuration.
        assert_eq!(Ok(()), pll.enable());

        // Make sure the PLL is enabled.
        assert_eq!(true, pll.is_enabled());

        // By default, the PLL clock is set to 96MHz
        assert_eq!(Some(96_000_000), pll.get_frequency());

        // By default, the PLL48 clock is correctly calibrated
        assert_eq!(true, pll.is_pll48_calibrated());

        // Impossible to configure the PLL clock once it is enabled.
        assert_eq!(
            Err(ErrorCode::FAIL),
            pll.prepare_frequnecy(50_000_000, HSI_FREQUENCY, PllSource::HSI)
        );

        // Stop the PLL in order to reconfigure it.
        assert_eq!(Ok(()), pll.disable());

        // Configure the PLL clock to run at 25MHz
        assert_eq!(
            Ok(()),
            pll.prepare_frequnecy(25_000_000, HSI_FREQUENCY, PllSource::HSI)
        );

        // Start the PLL with the new configuration
        assert_eq!(Ok(()), pll.enable());

        // get_frequency() method should reflect the new change
        assert_eq!(Some(25_000_000), pll.get_frequency());

        // Since 25 is not a multiple of 48, the PLL48 clock is not correctly calibrated
        assert_eq!(false, pll.is_pll48_calibrated());

        // The expected PLL48 clock value in this case should be approximately 40 MHz.
        // It is actually exactly 40MHz in this particular case.
        assert_eq!(Some(40_000_000), pll.get_frequency_pll48());

        // Stop the PLL clock
        assert_eq!(Ok(()), pll.disable());

        // Attempting to get the frequency of the PLL clock when it is disabled should return None.
        assert_eq!(None, pll.get_frequency());
        // Same for PLL48 clock
        assert_eq!(None, pll.get_frequency_pll48());

        // Attempting to configure the PLL clock with a frequency multiple of 48MHz
        assert_eq!(
            Ok(()),
            pll.prepare_frequnecy(144_000_000, HSI_FREQUENCY, PllSource::HSI)
        );
        assert_eq!(Ok(()), pll.enable());
        assert_eq!(Some(144_000_000), pll.get_frequency());

        // PLL48 clock output should be correctly calibrated
        assert_eq!(true, pll.is_pll48_calibrated());
        assert_eq!(Some(48_000_000), pll.get_frequency_pll48());

        // Reconfigure the clock for 100MHz
        assert_eq!(Ok(()), pll.disable());
        assert_eq!(
            Ok(()),
            pll.prepare_frequnecy(100_000_000, HSI_FREQUENCY, PllSource::HSI)
        );
        assert_eq!(Ok(()), pll.enable());
        assert_eq!(Some(100_000_000), pll.get_frequency());

        // In this case, the PLL48 clock is not correctly calibrated. Its frequency is
        // approximately 44MHz.
        assert_eq!(false, pll.is_pll48_calibrated());
        assert!((43_000_000..=45_000_000).contains(&pll.get_frequency_pll48().unwrap()));

        // Configure the clock to 72MHz = 48MHz * 1.5
        assert_eq!(Ok(()), pll.disable());
        assert_eq!(
            Ok(()),
            pll.prepare_frequnecy(72_000_000, HSI_FREQUENCY, PllSource::HSI)
        );
        assert_eq!(Ok(()), pll.enable());
        assert_eq!(Some(72_000_000), pll.get_frequency());

        // In this case, the PLL48 clock is correctly calibrated
        assert_eq!(true, pll.is_pll48_calibrated());
        assert_eq!(Some(48_000_000), pll.get_frequency_pll48());

        // Turn off the PLL clock
        assert_eq!(Ok(()), pll.disable());
        assert_eq!(false, pll.is_enabled());

        debug!("Finished testing PLL struct.");
    }

    /// Run the entire test suite.
    ///
    /// # Usage
    ///
    /// ```rust,ignore
    /// use stm32f429zi::pll; // Import the PLL module
    /// /* Code goes here */
    /// pll::test::run(&peripherals.stm32f4.pll); // Run the tests
    /// ```
    pub fn run<'a>(pll: &'a Pll<'a>) {
        debug!("");
        debug!("===============================================");
        debug!("Testing PLL...");
        debug!("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        test_pll_config();
        debug!("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        test_pll_struct(pll);
        debug!("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
        debug!("Finished testing PLL. Everything is alright!");
        debug!("===============================================");
        debug!("");
    }
}
