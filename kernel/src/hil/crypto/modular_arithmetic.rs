// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026.

//! Interface for hardware-accelerated modular arithmetic.
//!
//! A client starts an operation with [`ModularArithmetic::start_computation`],
//! supplies the modulus and operands through the [`MathClient`] read callbacks,
//! and receives the result through [`MathClient::write_number`], followed by
//! exactly one [`MathClient::computation_completed`] callback.
//!
//! # Data format
//!
//! All numbers (the modulus, the operands and the result) are unsigned
//! big-endian integers: the most significant byte comes first.
//!
//! Every number is exactly `modulus_len` bytes long, left-padded with zeros.
//! Leading zero bytes must not be stripped, including from the modulus.
//!
//! # Data movement
//!
//! The client owns all buffers; the driver never lends the client a buffer.
//!
//! - **Input.** In [`read_modulus`](MathClient::read_modulus),
//!   [`read_number`](MathClient::read_number) and
//!   [`read_second_number`](MathClient::read_second_number), the driver passes
//!   a `sink`. The client calls `sink` exactly once with *all* bytes of that
//!   number it has not yet supplied. `sink` absorbs a prefix of the slice and
//!   returns how many bytes it took.
//! - **Output.** In [`write_number`](MathClient::write_number), the driver
//!   passes a `source`. The client calls `source` exactly once with *all* of
//!   its result storage that has not yet been written. `source` writes a prefix
//!   of the slice and returns how many bytes it wrote.
//!
//! In both directions the client advances its position for that number by the
//! returned count. If the closure returns an error, the client returns that
//! same error from the callback. If the count is zero or larger than the slice
//! it was given, the client returns [`ErrorCode::SIZE`]. A callback that
//! returns `Ok(())` without calling its closure is treated by the driver as
//! [`ErrorCode::FAIL`].
//!
//! Because the client always offers every remaining byte, a driver may consume
//! in units that suit its hardware (for example whole 32-bit words) and take
//! the remainder in the last call.
//!
//! # Chunked transfer
//!
//! The driver chooses how many bytes each call moves, so it never needs a
//! buffer large enough to hold a whole number. For each number:
//!
//! - bytes are transferred in order, starting from the most significant end,
//!   so concatenating the chunks yields the full big-endian number;
//! - every byte is transferred exactly once, so the chunks of one number add up
//!   to exactly `modulus_len` bytes.
//!
//! Transfers of different numbers may be interleaved, so a client must track a
//! separate position for the modulus, each operand and the result.
//!
//! # Operations
//!
//! | Operation      | Operands read | Result            |
//! |----------------|---------------|-------------------|
//! | Addition       | `a`, `b`      | `(a + b) mod m`   |
//! | Subtraction    | `a`, `b`      | `(a - b) mod m`   |
//! | Multiplication | `a`, `b`      | `(a * b) mod m`   |
//! | Division       | `a`, `b`      | `(a * b⁻¹) mod m` |
//! | Inverse        | `a`           | `a⁻¹ mod m`       |
//! | Modulo         | `a`           | `a mod m`         |
//!
//! `m` is the modulus, read with [`MathClient::read_modulus`]; `a` is read with
//! [`MathClient::read_number`] and `b` with [`MathClient::read_second_number`].
//! Every result is fully reduced, in the range `0 <= r < m`.
//!
//! For the modulo operation, `a` may be any `modulus_len`-byte value. For all
//! other operations, operands must be less than `m`; a driver may reject larger
//! operands with [`ErrorCode::INVAL`] or produce an unspecified result. Inverse
//! and division complete with [`ErrorCode::INVAL`] if the value to invert has
//! no inverse modulo `m`.

use crate::{
    ErrorCode,
    processbuffer::{ReadableProcessSlice, WriteableProcessSlice},
};

/// Constructor for the addition operation of a driver-defined operation type.
///
/// The operation traits let generic code name an operation without knowing the
/// driver's operation type. A driver's type implements the traits for the
/// operations it supports.
pub trait OpAddition {
    fn addition() -> Self;
}

/// Constructor for the subtraction operation. See [`OpAddition`].
pub trait OpSubtraction {
    fn subtraction() -> Self;
}

/// Constructor for the multiplication operation. See [`OpAddition`].
pub trait OpMultiplication {
    fn multiplication() -> Self;
}

/// Constructor for the division operation. See [`OpAddition`].
pub trait OpDivision {
    fn division() -> Self;
}

/// Constructor for the inverse operation. See [`OpAddition`].
pub trait OpInverse {
    fn inverse() -> Self;
}

/// Constructor for the modulo (reduction) operation. See [`OpAddition`].
pub trait OpModulo {
    fn modulo() -> Self;
}

/// Client callbacks for [`ModularArithmetic`] operations.
///
/// See the [data movement](self#data-movement) rules, which apply to every
/// data callback below.
pub trait MathClient<Op> {
    /// Supply the next bytes of the modulus `m`.
    ///
    /// The driver may call this several times; in total it consumes exactly
    /// `modulus_len` bytes.
    fn read_modulus(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Supply the next bytes of the first operand `a`.
    ///
    /// The driver may call this several times; in total it consumes exactly
    /// `modulus_len` bytes.
    fn read_number(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Supply the next bytes of the second operand `b`.
    ///
    /// Only called for operations that take two operands. The driver may call
    /// this several times; in total it consumes exactly `modulus_len` bytes.
    fn read_second_number(
        &self,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Receive the next bytes of the result.
    ///
    /// Only called if the computation succeeded, after the driver has consumed
    /// all input. The driver may call this several times; in total it writes
    /// exactly `modulus_len` bytes before calling
    /// [`computation_completed`](Self::computation_completed) with `Ok(())`.
    fn write_number(
        &self,
        source: &mut dyn FnMut(&WriteableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode>;

    /// Signal completion of an arithmetic operation.
    ///
    /// Called exactly once for each call to
    /// [`ModularArithmetic::start_computation`] that returned `Ok(())`, after
    /// all data callbacks for that operation.
    ///
    /// `Ok(())` means the complete result has been delivered through
    /// [`write_number`](Self::write_number). On `Err`, any result bytes already
    /// delivered are invalid and must be discarded.
    ///
    /// If any data callback returns an error, the driver immediately aborts the
    /// operation, issues no further data callbacks for it, and passes that same
    /// error here.
    fn computation_completed(&self, result: Result<(), ErrorCode>);
}

/// Hardware-accelerated modular arithmetic.
///
/// See the [module documentation](self) for the data format, data movement and
/// the supported operations.
pub trait ModularArithmetic<'a, Op> {
    /// Set the client that will receive callbacks for operations.
    fn set_client(&self, client: &'a dyn MathClient<Op>);

    /// Start an operation on numbers of `modulus_len` bytes.
    ///
    /// The modulus and operands are read through [`MathClient`] callbacks.
    /// Callbacks must not occur before this method returns.
    ///
    /// On `Ok(())`, exactly one [`MathClient::computation_completed`] callback
    /// follows. If the operation succeeds, the result is delivered first as
    /// exactly `modulus_len` bytes through [`MathClient::write_number`].
    ///
    /// # Errors
    ///
    /// - [`ErrorCode::BUSY`]: another operation is in progress.
    /// - [`ErrorCode::RESERVE`]: no client is registered.
    /// - [`ErrorCode::SIZE`]: `modulus_len` is zero or not supported by the
    ///   driver.
    /// - [`ErrorCode::NOSUPPORT`]: `operation` is not supported by the driver.
    ///
    /// On `Err`, no callbacks occur for this request.
    fn start_computation(&self, modulus_len: usize, operation: Op) -> Result<(), ErrorCode>;
}
