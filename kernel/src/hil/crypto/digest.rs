// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2026.

//! Interfaces for digests / cryptgraphic hashes.
//!
//! This module defines algorithm enum and HIL traits for message digest calculation.
//! The algorithm enum supplies block and digest sizes.

use crate::ErrorCode;

/// Algorithms performing digest calculation
#[derive(Clone, Copy)]
pub enum Algorithm {
    Md5,
    Sha1,
    Sha224,
    Sha256,
    Sha384,
    Sha512_224,
    Sha512_256,
    Sha512,
}

impl Algorithm {
    /// Returns length of digest in accordance with passed algorithm in bytes.
    pub const fn get_digest_len(&self) -> usize {
        match self {
            Algorithm::Md5 => 16,
            Algorithm::Sha1 => 20,
            Algorithm::Sha224 | Algorithm::Sha512_224 => 28,
            Algorithm::Sha256 | Algorithm::Sha512_256 => 32,
            Algorithm::Sha384 => 48,
            Algorithm::Sha512 => 64,
        }
    }

    /// Returns block size in accordance with passed algorithm in bytes.
    ///
    /// Usually used in key calculation at HMAC mode.
    pub const fn get_block_size(&self) -> usize {
        match self {
            Algorithm::Md5 | Algorithm::Sha1 | Algorithm::Sha224 | Algorithm::Sha256 => 512 >> 3,
            Algorithm::Sha384
            | Algorithm::Sha512_224
            | Algorithm::Sha512_256
            | Algorithm::Sha512 => 1024 >> 3,
        }
    }
}

/// Hash trait
pub trait Digest {
    /// Initiate a hashing operation over `len` bytes.
    ///
    /// `len` must be any but greater than zero. The input
    /// is retrieved through [`Client`] callbacks. The driver returns exactly [`Algorithm::get_digest_len`] bytes of
    /// output, then issues exactly one [`Client::hash_done`] callback.
    /// Callbacks must not occur before this method returns.
    ///
    /// Returns [`ErrorCode::BUSY`] if an operation is in progress, [`ErrorCode::INVAL`] if `len`
    /// is nothing but zero, or [`ErrorCode::NOSUPPORT`] if the selected algorithm is unsupported.
    /// On `Ok(())`, a completion callback will occur. On `Err`, no callbacks will occur for this
    /// request.
    fn hash(&self, algorithm: Algorithm, len: usize) -> Result<(), ErrorCode>;

    fn clear_data(&self);

    /// Set the client that provides inputs and receives outputs.
    fn set_client(&self, client: &'static dyn Client);
}
/// Client callbacks for [`Digest`] operations.
pub trait Client {
    /// Retrieve input.
    ///
    /// The driver may issue this callback multiple times, but must not request more than the `len`
    /// passed to [`Digest::hash`] in total.
    fn read_input(&self, input: &mut [u8]) -> Result<usize, ErrorCode>;
    /// Return digest output.
    ///
    /// Across all calls, the driver must return exactly [`Algorithm::get_digest_len`] bytes in input order.
    fn write_output(&self, output: &[u8]) -> Result<(), ErrorCode>;

    /// Signal completion of a hashing operation.
    ///
    /// This callback occurs exactly once after a request accepted by [`Digest::hash`]. If a data
    /// callback returns an error, the driver must abort and report that error through `result`.
    fn hash_done(&self, result: Result<(), ErrorCode>);
}

/// HMAC trait
pub trait Hmac: Digest {
    /// Initiate an HMAC operation over `input_len` bytes with `key_len` bytes as a key.
    ///
    /// `input_len` must be nothing but greater than zero. The input
    /// is retrieved through [`Client`] callbacks. The driver returns exactly [`Algorithm::get_digest_len`] bytes of
    /// output, then issues exactly one [`Client::hash_done`] callback.
    ///
    /// `key_len` must be anything but greater than [`Algorithm::get_block_size`] bytes.
    ///
    /// Callbacks must not occur before this method returns.
    ///
    /// Returns [`ErrorCode::BUSY`] if an operation is in progress, [`ErrorCode::INVAL`] if `input_len`
    /// is nothing but zero OR `key_len` is anything but greater than [`Algorithm::get_block_size`] bytes,
    /// or [`ErrorCode::NOSUPPORT`] if the selected algorithm is unsupported.
    /// On `Ok(())`, a completion callback will occur. On `Err`, no callbacks will occur for this
    /// request.
    fn authenticate(
        &self,
        algorithm: Algorithm,
        input_len: usize,
        key_len: usize,
    ) -> Result<(), ErrorCode>;

    fn set_hmac_client(&self, client: &'static dyn HmacClient);
}

/// Client callbacks for [`Hmac`] operation.
pub trait HmacClient: Client {
    /// Retrieve the cipher key.
    ///
    /// The driver must provide less or equal to [`Algorithm::get_block_size`] bytes.
    fn read_key(&self, key: &mut [u8]) -> Result<usize, ErrorCode>;
}
