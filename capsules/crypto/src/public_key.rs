// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026

//! Interface for retrieving the public key of a signer's private key.

use kernel::ErrorCode;

/// Client for retrieving the public key that matches the active private key.
pub trait GetPublicKeyClient<const PL: usize> {
    /// Called when the public key has been computed.
    ///
    /// Returns the buffer passed to
    /// [`GetPublicKey::get_public_key()`], holding the public key on success.
    ///
    /// ### `result`:
    ///
    /// - `Ok(())`: `public_key` holds the public key.
    /// - `Err(())`: The public key could not be computed.
    ///   - `ErrorCode::FAIL`: The underlying hardware failed.
    fn get_public_key_done(&self, result: Result<(), ErrorCode>, public_key: &'static mut [u8; PL]);
}

/// Interface for retrieving the public key of the active private key.
///
/// The implementer may derive the public key from the private key (for
/// example `Q = d·G` for elliptic curves) or read it from storage. The
/// private key itself is never exposed.
///
/// `PL` is the length of the public key.
pub trait GetPublicKey<'a, const PL: usize> {
    /// Write the public key into `public_key`.
    ///
    /// This operation is asynchronous and its completion is signaled by
    /// [`GetPublicKeyClient::get_public_key_done()`].
    ///
    /// ## Return
    ///
    /// `Ok()` if the operation was accepted. Otherwise:
    /// - `Err(ErrorCode::BUSY)` if another operation is in progress.
    /// - `Err(ErrorCode::OFF)` if no private key is available.
    fn get_public_key(
        &self,
        public_key: &'static mut [u8; PL],
    ) -> Result<(), (ErrorCode, &'static mut [u8; PL])>;

    fn set_public_key_client(&self, client: &'a dyn GetPublicKeyClient<PL>);
}
