// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Component for initializing HMAC instances.
//!
//! Usage
//! -----
//! ```rust
//! let hash_mutex = components::driver_mutex::DriverMutexComponent::new(hash).finalize(
//!     components::driver_mutex_component_static!(stm32u545::hash::hash::Hash<'static>),
//! );
//!
//! let hmac = components::crypto::hmac::HmacComponent::new(
//!     board_kernel,
//!     capsules_crypto::hmac::DRIVER_NUM,
//!     hash_mutex,
//!     create_capability!(capabilities::MemoryAllocationCapability),
//! )
//! .finalize(components::hmac_crypto_component_static!(
//!     stm32u545::hash::hash::Hash<'static>,
//! ));
//! ```

use capsules_core::driver_mutex::DriverMutex;
use capsules_crypto::hmac::Hmac;
use core::mem::MaybeUninit;
use kernel::capabilities::MemoryAllocationCapability;
use kernel::component::Component;
use kernel::hil::crypto::digest;

/// Statically allocates the storage needed to finalize a [`HmacComponent`].
///
/// `$H` is the concrete type of HMAC implementer.
#[macro_export]
// TODO: Rename it into `hmac_component_static!`
macro_rules! hmac_crypto_component_static {
    ($H:ty $(,)?) => {{ kernel::static_buf!(capsules_crypto::hmac::Hmac<$H>) }};
}

pub struct HmacComponent<H: 'static + digest::Hmac, CAP: MemoryAllocationCapability + 'static> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    hmac: &'static DriverMutex<H>,
    mem_cap: CAP,
}

/// Component helper interface used to initialize a [`Hmac`] instance.
impl<H: 'static + digest::Hmac, CAP: MemoryAllocationCapability + 'static> HmacComponent<H, CAP> {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        hmac: &'static DriverMutex<H>,
        mem_cap: CAP,
    ) -> Self {
        Self {
            board_kernel,
            driver_num,
            hmac,
            mem_cap,
        }
    }
}

impl<H: 'static + digest::Hmac, CAP: MemoryAllocationCapability + 'static> Component
    for HmacComponent<H, CAP>
{
    type StaticInput = &'static mut MaybeUninit<Hmac<H>>;

    type Output = &'static Hmac<H>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        s.write(Hmac::new(
            self.hmac,
            self.board_kernel
                .create_grant(self.driver_num, &self.mem_cap),
        ))
    }
}
