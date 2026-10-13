// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Component for initializing Hash instances.
//!
//! Usage
//! -----
//! ```rust
//! let hash_mutex = components::driver_mutex::DriverMutexComponent::new(hash).finalize(
//!     components::driver_mutex_component_static!(stm32u545::hash::hash::Hash<'static>),
//! );
//!
//! let hash = components::crypto::hash::HashComponent::new(
//!     board_kernel,
//!     capsules_crypto::hash::DRIVER_NUM,
//!     hash_mutex,
//!     create_capability!(capabilities::MemoryAllocationCapability),
//! )
//! .finalize(components::hash_crypto_component_static!(
//!     stm32u545::hash::hash::Hash<'static>,
//! ));
//! ```

use capsules_core::driver_mutex::DriverMutex;
use capsules_crypto::hash::Hash;
use core::mem::MaybeUninit;
use kernel::capabilities::MemoryAllocationCapability;
use kernel::component::Component;
use kernel::hil::crypto::digest;

/// Statically allocates the storage needed to finalize a [`HashComponent`].
///
/// `$H` is the concrete type of Digest implementer.
#[macro_export]
// TODO: Rename it into `hash_component_static!`
macro_rules! hash_crypto_component_static {
    ($H:ty $(,)?) => {{ kernel::static_buf!(capsules_crypto::hash::Hash<$H>) }};
}

/// Component helper interface used to initialize a [`Hash`] instance.
pub struct HashComponent<H: 'static + digest::Digest, CAP: MemoryAllocationCapability + 'static> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    hash: &'static DriverMutex<H>,
    mem_cap: CAP,
}

impl<H: 'static + digest::Digest, CAP: MemoryAllocationCapability + 'static> HashComponent<H, CAP> {
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        hash: &'static DriverMutex<H>,
        mem_cap: CAP,
    ) -> Self {
        Self {
            board_kernel,
            driver_num,
            hash,
            mem_cap,
        }
    }
}

impl<H: 'static + digest::Digest, CAP: MemoryAllocationCapability + 'static> Component
    for HashComponent<H, CAP>
{
    type StaticInput = &'static mut MaybeUninit<Hash<H>>;

    type Output = &'static Hash<H>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        s.write(Hash::new(
            self.hash,
            self.board_kernel
                .create_grant(self.driver_num, &self.mem_cap),
        ))
    }
}
