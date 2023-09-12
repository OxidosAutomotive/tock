// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive SRL.
//
// Author: Ioan-Cristian CÎRSTEA <ioan.cirstea@oxidos.io>

#![deny(missing_docs)]
#![deny(dead_code)]

//! This module contains all chip-specific code.
//!
//! Some models in the STM32F4 family may have additional features, while others not. Or they can
//! operate internally in different ways for the same feature. This crate provides all the
//! chip-specific crate to be used by others modules in this crate.

/// Clock-related constants for specific chips
pub mod clock_constants {
    // Internal constants prefixed by double underscores are created to avoid the need to duplicate
    // documentation comments

    /// PLL-related constants for specific chips
    pub mod pll_constants {
        // STM32F401 supports frequency down to 24MHz. All other chips in the F4 family down to
        // 13MHz.
        #[cfg(not(feature = "stm32f401"))]
        const __PLL_MIN_FREQ_MHZ: usize = 13_000_000;
        #[cfg(feature = "stm32f401")]
        const __PLL_MIN_FREQ_MHZ: usize = 24_000_000;

        /// Minimum PLL frequency in MHz
        pub const PLL_MIN_FREQ: usize = __PLL_MIN_FREQ_MHZ;
    }

    #[cfg(any(feature = "stm32f412"))]
    const __APB1_FREQUENCY_LIMIT_MHZ: usize = 50_000_000;
    #[cfg(any(feature = "stm32f429", feature = "stm32f446"))]
    #[cfg(not(feature = "cargo-clippy"))]
    const __APB1_FREQUENCY_LIMIT_MHZ: usize = 45_000_000;
    #[cfg(any(feature = "stm32f401"))]
    #[cfg(not(feature = "cargo-clippy"))]
    const __APB1_FREQUENCY_LIMIT_MHZ: usize = 42_000_000;

    /// Maximum allowed APB1 frequency in MHz
    pub const APB1_FREQUENCY_LIMIT_MHZ: usize = __APB1_FREQUENCY_LIMIT_MHZ;

    /// Maximum allowed APB2 frequency in MHz
    // APB2 frequency limit is twice the APB1 frequency limit
    pub const APB2_FREQUENCY_LIMIT_MHZ: usize = APB1_FREQUENCY_LIMIT_MHZ << 1;

    #[cfg(any(feature = "stm32f412"))]
    const __SYS_CLOCK_FREQUENCY_LIMIT_MHZ: usize = 100_000_000;
    #[cfg(any(feature = "stm32f429", feature = "stm32f446"))]
    #[cfg(not(feature = "cargo-clippy"))]
    const __SYS_CLOCK_FREQUENCY_LIMIT_MHZ: usize = 180_000_000;
    #[cfg(any(feature = "stm32f401"))]
    #[cfg(not(feature = "cargo-clippy"))]
    const __SYS_CLOCK_FREQUENCY_LIMIT_MHZ: usize = 84_000_000;

    /// Maximum allowed system clock frequency in MHz
    pub const SYS_CLOCK_FREQUENCY_LIMIT_MHZ: usize = __SYS_CLOCK_FREQUENCY_LIMIT_MHZ;
}

/// Chip-specific flash code
pub mod flash_specific {
    #[cfg(any(
        feature = "stm32f401",
        feature = "stm32f412",
        feature = "stm32f429",
        feature = "stm32f446"
    ))]
    #[derive(Copy, Clone, PartialEq, Debug)]
    /// Enum representing all the possible values for the flash latency
    pub(crate) enum FlashLatency {
        /// 0 wait cycles
        Latency0,
        /// 1 wait cycle
        Latency1,
        /// 2 wait cycles
        Latency2,
        /// 3 wait cycles
        Latency3,
        /// 4 wait cycles
        Latency4,
        /// 5 wait cycles
        Latency5,
        /// 6 wait cycles
        Latency6,
        /// 7 wait cycles
        Latency7,
        /// 8 wait cycles
        Latency8,
        /// 9 wait cycles
        Latency9,
        /// 10 wait cycles
        Latency10,
        /// 11 wait cycles
        Latency11,
        /// 12 wait cycles
        Latency12,
        /// 13 wait cycles
        Latency13,
        /// 14 wait cycles
        Latency14,
        /// 15 wait cycles
        Latency15,
    }

    // The number of wait cycles depends on two factors: system clock frequency and the supply
    // voltage. Currently, this method assumes 2.7-3.6V voltage supply (default value).
    // TODO: Take into the account the power supply
    //
    // The number of wait states varies from chip to chip.
    pub(crate) fn get_number_wait_cycles_based_on_frequency(frequency: usize) -> FlashLatency {
        // feature = "stm32f401"
        // feature = "stm32f429"
        // feature = "stm32f446"
        #[cfg(not(feature = "stm32f412"))]
        {
            match frequency {
                0_000_000..=30_000_000 => FlashLatency::Latency0,
                31_000_000..=60_000_000 => FlashLatency::Latency1,
                61_000_000..=90_000_000 => FlashLatency::Latency2,
                91_000_000..=120_000_000 => FlashLatency::Latency3,
                121_000_000..=150_000_000 => FlashLatency::Latency4,
                _ => FlashLatency::Latency5,
            }
        }
        #[cfg(any(feature = "stm32f412"))]
        {
            match frequency {
                0_000_000..=30_000_000 => FlashLatency::Latency0,
                30_000_001..=64_000_000 => FlashLatency::Latency1,
                64_000_001..=90_000_000 => FlashLatency::Latency2,
                _ => FlashLatency::Latency3,
            }
        }
    }

    pub(crate) fn convert_register_to_enum(flash_latency_register: u32) -> FlashLatency {
        #[cfg(any(
            feature = "stm32f401",
            feature = "stm32f412",
            feature = "stm32f429",
            feature = "stm32f446"
        ))]
        match flash_latency_register {
            0 => FlashLatency::Latency0,
            1 => FlashLatency::Latency1,
            2 => FlashLatency::Latency2,
            3 => FlashLatency::Latency3,
            4 => FlashLatency::Latency4,
            5 => FlashLatency::Latency5,
            6 => FlashLatency::Latency6,
            7 => FlashLatency::Latency7,
            8 => FlashLatency::Latency8,
            9 => FlashLatency::Latency9,
            10 => FlashLatency::Latency10,
            11 => FlashLatency::Latency11,
            12 => FlashLatency::Latency12,
            13 => FlashLatency::Latency13,
            14 => FlashLatency::Latency14,
            // The hardware allows 4-bit latency values
            _ => FlashLatency::Latency15,
        }
    }
}

/// Chip-specific flash code
pub mod pwr_specific {
    /// PWR::VOS voltage scale settings used to adjust the internal voltage regulator
    pub enum VoltageScaling {
        /// voltage scale setting 1
        Scale1,
        /// voltage scale setting 2
        Scale2,
        /// voltage scale setting 3
        Scale3,
    }

    /// The frequency of the system can be increased only if the voltage scaling register and voltage overdrive register are modified
    /// Documented in Ref Man 3.5.1. Overdrive mode increases power consumption and is not available below 2.1V.
    /// To change overdrive register the HSI or HSE needs to be selected as system clock, to change votlage scaling the PLL needs to be disabled.
    pub(crate) fn get_vos_overdrive_basedon_on_frequency(
        frequency: usize,
    ) -> Option<(VoltageScaling, bool)> {
        match frequency {
            0..=120_000_000 => Some((VoltageScaling::Scale1, false)),
            120_000_001..=144_000_000 => Some((VoltageScaling::Scale2, false)),
            144_000_001..=168_000_000 => Some((VoltageScaling::Scale3, false)),
            168_000_001..=180_000_000 => Some((VoltageScaling::Scale3, true)),
            _ => None,
        }
    }
}
