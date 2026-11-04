// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2025.

//! ECDSA Signer for P256 signatures.
//!
//! Software implementation using the RustCrypto `p256` crate.
//! All operations are computed synchronously and the result is delivered
//! through a deferred call.

use core::cell::Cell;

use p256::ecdsa;
use p256::ecdsa::signature::hazmat::PrehashSigner;

use capsules_crypto::public_key::{GetPublicKey, GetPublicKeyClient};
use kernel::ErrorCode;
use kernel::hil;
use kernel::hil::public_key_crypto::keys::SetKeyBySliceClient;
use kernel::utilities::cells::{OptionalCell, TakeCell};

/// Length of the hash to sign.
pub const HASH_LEN: usize = 32;
/// Length of the `r || s` signature.
pub const SIGNATURE_LEN: usize = 64;
/// Length of the private key scalar `d`.
pub const PRIVATE_KEY_LEN: usize = 32;
/// Length of the uncompressed public key `X || Y`.
pub const PUBLIC_KEY_LEN: usize = 64;

#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    Signing(Result<(), ErrorCode>),
    ChangingKey,
    GettingPublicKey(Result<(), ErrorCode>),
}

pub struct EcdsaP256SignatureSigner<'a> {
    client: OptionalCell<&'a dyn hil::public_key_crypto::signature::ClientSign<32, 64>>,
    client_key_set: OptionalCell<&'a dyn SetKeyBySliceClient<32>>,
    client_pub_key: OptionalCell<&'a dyn GetPublicKeyClient<64>>,
    signing_key: TakeCell<'static, [u8; PRIVATE_KEY_LEN]>,
    new_key: TakeCell<'static, [u8; PRIVATE_KEY_LEN]>,
    hash_storage: TakeCell<'static, [u8; HASH_LEN]>,
    signature_storage: TakeCell<'static, [u8; SIGNATURE_LEN]>,
    public_key_storage: TakeCell<'static, [u8; PUBLIC_KEY_LEN]>,
    deferred_call: kernel::deferred_call::DeferredCall,
    state: Cell<State>,
}

impl EcdsaP256SignatureSigner<'_> {
    pub fn new(signing_key: &'static mut [u8; PRIVATE_KEY_LEN]) -> Self {
        Self {
            client: OptionalCell::empty(),
            client_key_set: OptionalCell::empty(),
            client_pub_key: OptionalCell::empty(),
            signing_key: TakeCell::new(signing_key),
            new_key: TakeCell::empty(),
            hash_storage: TakeCell::empty(),
            signature_storage: TakeCell::empty(),
            public_key_storage: TakeCell::empty(),
            deferred_call: kernel::deferred_call::DeferredCall::new(),
            state: Cell::new(State::Idle),
        }
    }

    /// Build the `p256` signing key from the stored private key.
    fn with_signing_key<R>(
        &self,
        f: impl FnOnce(&ecdsa::SigningKey) -> Result<R, ErrorCode>,
    ) -> Result<R, ErrorCode> {
        self.signing_key
            .map(|skey| {
                let skey: &[u8; PRIVATE_KEY_LEN] = skey;
                ecdsa::SigningKey::from_bytes(skey.into())
                    .map_err(|_| ErrorCode::INVAL)
                    .and_then(|key| f(&key))
            })
            .unwrap_or(Err(ErrorCode::OFF))
    }
}

impl<'a> hil::public_key_crypto::signature::SignatureSign<'a, HASH_LEN, SIGNATURE_LEN>
    for EcdsaP256SignatureSigner<'a>
{
    fn set_sign_client(
        &self,
        client: &'a dyn hil::public_key_crypto::signature::ClientSign<HASH_LEN, SIGNATURE_LEN>,
    ) {
        self.client.replace(client);
    }

    fn sign(
        &self,
        hash: &'static mut [u8; HASH_LEN],
        signature: &'static mut [u8; SIGNATURE_LEN],
    ) -> Result<
        (),
        (
            ErrorCode,
            &'static mut [u8; HASH_LEN],
            &'static mut [u8; SIGNATURE_LEN],
        ),
    > {
        if self.state.get() != State::Idle {
            return Err((ErrorCode::BUSY, hash, signature));
        }

        let result = self.with_signing_key(|key| {
            let sig: ecdsa::Signature = key.sign_prehash(hash).map_err(|_| ErrorCode::FAIL)?;
            signature.copy_from_slice(&sig.to_bytes());
            Ok(())
        });

        match result {
            Ok(()) => {
                self.hash_storage.replace(hash);
                self.signature_storage.replace(signature);
                self.state.set(State::Signing(Ok(())));
                self.deferred_call.set();
                Ok(())
            }
            Err(e) => Err((e, hash, signature)),
        }
    }
}

impl<'a> hil::public_key_crypto::keys::SetKeyBySlice<'a, PRIVATE_KEY_LEN>
    for EcdsaP256SignatureSigner<'a>
{
    /// Load a new private key.
    ///
    /// The key is validated immediately; it is copied into the signer in the
    /// deferred call, and `key` is then returned to the client.
    fn set_key(
        &self,
        key: &'static mut [u8; PRIVATE_KEY_LEN],
    ) -> Result<(), (ErrorCode, &'static mut [u8; PRIVATE_KEY_LEN])> {
        if self.state.get() != State::Idle {
            return Err((ErrorCode::BUSY, key));
        }
        if self.signing_key.is_none() {
            return Err((ErrorCode::FAIL, key));
        }
        {
            let k: &[u8; PRIVATE_KEY_LEN] = key;
            // Reject zero or scalars >= the curve order.
            if ecdsa::SigningKey::from_bytes(k.into()).is_err() {
                return Err((ErrorCode::INVAL, key));
            }
        }

        self.new_key.replace(key);
        self.state.set(State::ChangingKey);
        self.deferred_call.set();
        Ok(())
    }

    fn set_client(&self, client: &'a dyn SetKeyBySliceClient<PRIVATE_KEY_LEN>) {
        self.client_key_set.replace(client);
    }
}

impl<'a> GetPublicKey<'a, PUBLIC_KEY_LEN> for EcdsaP256SignatureSigner<'a> {
    /// Compute the uncompressed public key `Q = d·G` as `X || Y`.
    fn get_public_key(
        &self,
        public_key: &'static mut [u8; PUBLIC_KEY_LEN],
    ) -> Result<(), (ErrorCode, &'static mut [u8; PUBLIC_KEY_LEN])> {
        if self.state.get() != State::Idle {
            return Err((ErrorCode::BUSY, public_key));
        }

        let result = self.with_signing_key(|key| {
            let point = key.verifying_key().to_encoded_point(false);
            // Uncompressed SEC1 encoding is `0x04 || X || Y`.
            match point.as_bytes().split_first() {
                Some((0x04, xy)) if xy.len() == PUBLIC_KEY_LEN => {
                    public_key.copy_from_slice(xy);
                    Ok(())
                }
                _ => Err(ErrorCode::FAIL),
            }
        });

        match result {
            Ok(()) => {
                self.public_key_storage.replace(public_key);
                self.state.set(State::GettingPublicKey(Ok(())));
                self.deferred_call.set();
                Ok(())
            }
            Err(e) => Err((e, public_key)),
        }
    }

    fn set_public_key_client(&self, client: &'a dyn GetPublicKeyClient<PUBLIC_KEY_LEN>) {
        self.client_pub_key.replace(client);
    }
}

impl kernel::deferred_call::DeferredCallClient for EcdsaP256SignatureSigner<'_> {
    fn handle_deferred_call(&self) {
        match self.state.replace(State::Idle) {
            State::Idle => {}
            State::Signing(result) => {
                if let Some(h) = self.hash_storage.take()
                    && let Some(s) = self.signature_storage.take()
                {
                    self.client.map(|client| client.signing_done(result, h, s));
                }
            }
            State::ChangingKey => {
                if let Some(key) = self.new_key.take() {
                    self.signing_key.map(|skey| skey.copy_from_slice(key));
                    self.client_key_set
                        .map(|client| client.set_key_done(key, Ok(())));
                }
            }
            State::GettingPublicKey(result) => {
                if let Some(pk) = self.public_key_storage.take() {
                    self.client_pub_key
                        .map(|client| client.get_public_key_done(result, pk));
                }
            }
        }
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
