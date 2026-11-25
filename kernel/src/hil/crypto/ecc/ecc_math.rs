// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026

//! Interface for Math Operations on Elliptic Curves
//!
//! # Data format
//!
//! All integers (scalars, coordinates, moduli, operands and results) are
//! big-endian byte strings: the most significant byte is at index 0.
//! The same convention applies to data the driver reads from the client and
//! data it writes back.
//!
//! Field elements and scalars are fixed-width: exactly `P_SIZE` bytes,
//! left-padded with zeros. Leading zero bytes must not be stripped.
//!
//! Elliptic curve points are read in uncompressed form as a single stream
//! of `2 * P_SIZE` bytes, `x || y`: the x coordinate in the first `P_SIZE`
//! bytes, followed by the y coordinate. Resulting points are written as two
//! separate `P_SIZE` arrays, `x` and `y`.
//!
//! # Data movement
//!
//! Input and output follow the
//! [crypto data movement contract](super#callback-oriented-data-movement).
//! The client owns all buffers. To supply input, the client passes its
//! unconsumed bytes to a driver-provided `sink`, which absorbs a prefix and
//! returns how many bytes it took. To receive output, the client calls a
//! driver-provided `fill` closure with storage that the driver overwrites.

use crate::{
    ErrorCode, hil::crypto::ecc::ecc_constants::Curve, processbuffer::ReadableProcessSlice,
};

/// Client callbacks for [`EccCrypto`] and [`VerifyEccPoint`] operations.
pub trait EccClient<const P_SIZE: usize, C: Curve<P_SIZE>> {
    /// Supply the scalar for [`EccCrypto::scalar_multiplication`].
    ///
    /// The client passes the unconsumed bytes of the scalar to `sink`, which absorbs a
    /// portion of the slice and returns the number of bytes consumed. A driver may issue this
    /// callback multiple times. The total number of bytes consumed must equal `P_SIZE`.
    /// Each call must consume between one byte and the length of the slice passed to `sink`;
    /// a zero or oversized count terminates the operation with [`ErrorCode::SIZE`].
    fn read_scalar(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Supply the (first) input point, as `x || y`.
    ///
    /// Same rules as [`EccClient::read_scalar`], except that the total number of bytes
    /// consumed must equal `2 * P_SIZE`.
    fn read_point(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Supply the second input point for [`EccCrypto::point_addition`], as `x || y`.
    ///
    /// Same rules as [`EccClient::read_point`]. The driver may interleave calls to this
    /// callback with calls to [`EccClient::read_point`]; each stream is consumed in order.
    fn read_second_point(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Return the resulting point.
    ///
    /// The driver must call this exactly once, after it has taken all input and before
    /// signaling successful completion. The client calls `fill` exactly once with storage for
    /// the x and y coordinates. `fill` overwrites both arrays completely.
    fn write_point(
        &self,
        fill: &mut dyn FnMut(&mut [u8; P_SIZE], &mut [u8; P_SIZE]),
    ) -> Result<(), ErrorCode>;

    /// Signal completion of an elliptic curve operation.
    ///
    /// This callback occurs exactly once after a request accepted by [`EccCrypto`] or
    /// [`VerifyEccPoint`]. If a data callback returns an error, the driver must immediately
    /// abort the operation and report that error through `result`.
    fn operation_done(&self, result: Result<(), ErrorCode>);
}

/// Elliptic curve point arithmetic on curve `C`.
pub trait EccCrypto<'a, const P_SIZE: usize, C: Curve<P_SIZE>> {
    /// Set the client that will receive callbacks for operations.
    fn set_client(&self, client: &'a dyn EccClient<P_SIZE, C>);

    /// Initiate a point addition operation.
    ///
    /// The operands are retrieved through [`EccClient::read_point`] and
    /// [`EccClient::read_second_point`]. The sum is returned through
    /// [`EccClient::write_point`], followed by exactly one [`EccClient::operation_done`]
    /// callback. Callbacks must not occur before this method returns.
    ///
    /// Returns [`ErrorCode::BUSY`] if an operation is already in progress, or
    /// [`ErrorCode::RESERVE`] if no client is registered. On `Ok(())`, a completion callback
    /// will occur. On `Err`, no callbacks will occur for this request.
    fn point_addition(&self) -> Result<(), ErrorCode>;

    /// Initiate a scalar multiplication operation.
    ///
    /// The operands are retrieved through [`EccClient::read_point`] and
    /// [`EccClient::read_scalar`]. The product is returned through
    /// [`EccClient::write_point`], followed by exactly one [`EccClient::operation_done`]
    /// callback. Callbacks must not occur before this method returns.
    ///
    /// Returns [`ErrorCode::BUSY`] if an operation is already in progress, or
    /// [`ErrorCode::RESERVE`] if no client is registered. On `Ok(())`, a completion callback
    /// will occur. On `Err`, no callbacks will occur for this request.
    fn scalar_multiplication(&self) -> Result<(), ErrorCode>;
}

/// Checks whether a point lies on curve `C`.
pub trait VerifyEccPoint<'a, const P_SIZE: usize, C: Curve<P_SIZE>>:
    EccCrypto<'a, P_SIZE, C>
{
    /// Initiate a point verification operation.
    ///
    /// The point is retrieved through [`EccClient::read_point`]. No output is written:
    /// [`EccClient::write_point`] is not called. The driver then issues exactly one
    /// [`EccClient::operation_done`] callback, with `Ok(())` if the point is on the curve
    /// or [`ErrorCode::INVAL`] if it is not. Callbacks must not occur before this method
    /// returns.
    ///
    /// Returns [`ErrorCode::BUSY`] if an operation is already in progress, or
    /// [`ErrorCode::RESERVE`] if no client is registered. On `Ok(())`, a completion callback
    /// will occur. On `Err`, no callbacks will occur for this request.
    fn verify_point(&self) -> Result<(), ErrorCode>;
}
