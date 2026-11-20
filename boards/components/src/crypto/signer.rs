// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Component for initializing Signer instances.
//!
//! Usage
//! -----
//! ```rust
//! let signer_mutex = components::driver_mutex::DriverMutexComponent::new(ecdsa).finalize(
//!     components::driver_mutex_component_static!(ecdsa::hash::hash::Hash<'static>),
//! );
//!
//! let signer = components::crypto::signer::SignerComponent::new(
//!     board_kernel,
//!     capsules_crypto::signer::DRIVER_NUM,
//!     signer_mutex,
//!     create_capability!(capabilities::MemoryAllocationCapability),
//! )
//! .finalize(components::signer_crypto_component_static!(
//!     ecdsa_sw::p256_signer::EcdsaP256SignatureSigner<'static>,
//!     32,
//!     64,
//!     32,
//!     64
//! ));
//! ```

use capsules_core::driver_mutex::DriverMutex;
use capsules_crypto::public_key::GetPublicKey;
use capsules_crypto::sign::Signer;
use core::mem::MaybeUninit;
use kernel::capabilities::MemoryAllocationCapability;
use kernel::component::Component;
use kernel::hil::public_key_crypto::keys::SetKeyBySlice;
use kernel::hil::public_key_crypto::signature::SignatureSign;

/// Statically allocates the storage needed to finalize a [`SignerComponent`].
///
/// `$S` is the concrete type of Signer implementer.
#[macro_export]
macro_rules! signer_crypto_component_static {
    ($S:ty, $HL:expr, $SL:expr, $KL:expr, $PL:expr $(,)?) => {{
        let hash_buf = kernel::static_buf!([u8; $HL]);
        let signature_buf = kernel::static_buf!([u8; $SL]);
        let private_key_buf = kernel::static_buf!([u8; $KL]);
        let public_key_buf = kernel::static_buf!([u8; $PL]);
        let signer = kernel::static_buf!(capsules_crypto::sign::Signer<$S, $HL, $SL, $KL, $PL>);
        (signer, hash_buf, signature_buf, private_key_buf, public_key_buf)
    }};
}

/// Component helper interface used to initialize a [`Signer`] instance.
pub struct SignerComponent<
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
    CAP: MemoryAllocationCapability + 'static,
    const HL: usize,
    const SL: usize,
    const KL: usize,
    const PL: usize,
> {
    board_kernel: &'static kernel::Kernel,
    driver_num: usize,
    signer: &'static DriverMutex<S>,
    mem_cap: CAP,
}

impl<
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
    CAP: MemoryAllocationCapability + 'static,
    const HL: usize,
    const SL: usize,
    const KL: usize,
    const PL: usize,
> SignerComponent<S, CAP, HL, SL, KL, PL>
{
    pub fn new(
        board_kernel: &'static kernel::Kernel,
        driver_num: usize,
        signer: &'static DriverMutex<S>,
        mem_cap: CAP,
    ) -> Self {
        Self {
            board_kernel,
            driver_num,
            signer,
            mem_cap,
        }
    }
}

impl<
    S: SignatureSign<'static, HL, SL>
        + SetKeyBySlice<'static, KL>
        + GetPublicKey<'static, PL>
        + 'static,
    CAP: MemoryAllocationCapability + 'static,
    const HL: usize,
    const SL: usize,
    const KL: usize,
    const PL: usize,
> Component for SignerComponent<S, CAP, HL, SL, KL, PL>
{
    type StaticInput = (
        &'static mut MaybeUninit<Signer<S, HL, SL, KL, PL>>,
        &'static mut MaybeUninit<[u8; HL]>,
        &'static mut MaybeUninit<[u8; SL]>,
        &'static mut MaybeUninit<[u8; KL]>,
        &'static mut MaybeUninit<[u8; PL]>,
    );

    type Output = &'static Signer<S, HL, SL, KL, PL>;

    fn finalize(self, s: Self::StaticInput) -> Self::Output {
        let hash_buf = s.1.write([0u8; HL]);
        let signature_buf = s.2.write([0u8; SL]);
        let private_key_buf = s.3.write([0u8; KL]);
        let public_key_buf = s.4.write([0u8; PL]);
        s.0.write(Signer::new(
            self.signer,
            hash_buf,
            signature_buf,
            private_key_buf,
            public_key_buf,
            self.board_kernel
                .create_grant(self.driver_num, &self.mem_cap),
        ))
    }
}
