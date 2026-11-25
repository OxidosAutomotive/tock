// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026.

use core::cell::Cell;

use kernel::ErrorCode;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::crypto::ecc::ecc_constants::{Curve, NistP256Constants, P_256_P_SIZE};
use kernel::hil::crypto::ecc::ecc_math::{EccClient, EccCrypto, VerifyEccPoint};
use kernel::hil::crypto::modular_arithmetic::{MathClient, ModularArithmetic};
use kernel::hil::public_key_crypto::rsa_math::{Client, RsaCryptoBase};
use kernel::processbuffer::{ReadableProcessSlice, WriteableProcessSlice};
use kernel::utilities::StaticRef;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::FieldValue;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};

use crate::pkc::constants::{
    ARITH_OP1_IDX, ARITH_OP2_IDX, ARITH_RESULT_IDX, CLRFR, CR, ECC_A_ABS_IDX, ECC_A_SIGN_IDX,
    ECC_ADD_OUT_X_IDX, ECC_ADD_OUT_Y_IDX, ECC_ADD_P_IDX, ECC_ADD_PT1_X_IDX, ECC_ADD_PT1_Y_IDX,
    ECC_ADD_PT1_Z_IDX, ECC_ADD_PT2_X_IDX, ECC_ADD_PT2_Y_IDX, ECC_ADD_PT2_Z_IDX, ECC_B_IDX,
    ECC_MUL_IN_X_IDX, ECC_MUL_IN_Y_IDX, ECC_MUL_K_IDX, ECC_N_IDX, ECC_N_LEN_BITS_IDX,
    ECC_OUT_X_IDX, ECC_OUT_Y_IDX, ECC_P_IDX, ECC_P_LEN_BITS_IDX, ECC_P_R2_IDX, ECC_RESULT_OK,
    EXP_LEN_BITS_IDX, FPCHECK_RESULT_IDX, FPCHECK_X_IDX, FPCHECK_Y_IDX, INV_RED_MODULUS_IDX,
    MODEXP_BASE_IDX, MODEXP_EXPONENT_IDX, MODEXP_RESULT_IDX, MODULUS_IDX, MONT_R2_OUT_IDX,
    OPERAND_LEN_BITS_IDX, P256_R2_MOD_P, PkaRegisters, SR, SupportedOp,
};

/// Largest number the math interface accepts, in bytes.
///
/// The PKA handles operands of up to 4160 bits. 520 bytes plus the two zero
/// terminator words fit exactly in each arithmetic RAM slot (for example
/// `ARITH_OP2` at 0xC68 ends where `ARITH_RESULT` at 0xE78 begins).
const MATH_MAX_LEN: usize = 520;

/// A P-256 coordinate or scalar, big-endian.
type P256Bytes = [u8; P_256_P_SIZE];

/// The ECC client type this driver serves.
type P256Client<'a> = dyn EccClient<P_256_P_SIZE, NistP256Constants> + 'a;

#[derive(Copy, Clone, Debug, PartialEq)]
enum State {
    Idle,
    Rsa,
    ScalarMul,
    PointAddition,
    ProjToAffinePass1,
    ProjToAffinePass2,
    ProjToAffinePass3,
    VerifyPoint,
    MathAddition,
    MathInvert,
    MathComputeR2,
    MathComputeAR,
    MathComputeAB,
    MathModulus,
}

/// Which input to request from a client, and through which callback.
#[derive(Copy, Clone)]
enum Input {
    /// [`MathClient::read_modulus`]
    Modulus,
    /// [`MathClient::read_number`]
    First,
    /// [`MathClient::read_second_number`]
    Second,
    /// [`EccClient::read_scalar`]
    Scalar,
    /// [`EccClient::read_point`]
    Point,
    /// [`EccClient::read_second_point`]
    SecondPoint,
}

pub struct Pka<'a> {
    registers: StaticRef<PkaRegisters>,
    deferred_call: DeferredCall,

    rsa_client: OptionalCell<&'a dyn Client<'a>>,
    ecc_client: OptionalCell<&'a P256Client<'a>>,
    math_client: OptionalCell<&'a dyn MathClient<SupportedOp>>,

    modulus: OptionalCell<&'static [u8]>,
    exponent: OptionalCell<&'static [u8]>,

    message: TakeCell<'static, [u8]>,
    result: TakeCell<'static, [u8]>,

    math_len: Cell<usize>,
    /// x coordinate of a point addition, kept between projective-to-affine passes.
    ecc_x: Cell<P256Bytes>,

    state: Cell<State>,
}

impl<'a> Pka<'a> {
    pub fn new(registers: StaticRef<PkaRegisters>) -> Pka<'a> {
        Pka {
            registers,
            deferred_call: DeferredCall::new(),

            rsa_client: OptionalCell::empty(),
            ecc_client: OptionalCell::empty(),
            math_client: OptionalCell::empty(),

            modulus: OptionalCell::empty(),
            exponent: OptionalCell::empty(),

            message: TakeCell::empty(),
            result: TakeCell::empty(),

            math_len: Cell::new(0),
            ecc_x: Cell::new([0; P_256_P_SIZE]),

            state: Cell::new(State::Idle),
        }
    }

    fn write_slice(&self, idx: usize, data: &[u8]) {
        for (i, chunk) in data.rchunks(4).enumerate() {
            let Some(cell) = self.registers.ram.get(idx + i) else {
                break;
            };
            let mut word = [0u8; 4];
            word[4 - chunk.len()..].copy_from_slice(chunk);
            cell.set(u32::from_be_bytes(word));
        }
    }

    fn read_slice(&self, idx: usize, buffer: &mut [u8]) {
        for (i, chunk) in buffer.rchunks_mut(4).enumerate() {
            let Some(cell) = self.registers.ram.get(idx + i) else {
                break;
            };
            let bytes = cell.get().to_be_bytes();
            chunk.copy_from_slice(&bytes[4 - chunk.len()..]);
        }
    }

    fn set_len(&self, idx: usize, value: u32) {
        self.registers.ram[idx].set(value);
        self.registers.ram[idx + 1].set(0);
    }

    fn clear_ram(&self) {
        for cell in self.registers.ram.iter() {
            cell.set(0);
        }
    }

    fn wait_init_ok(&self) {
        while !self.registers.sr.is_set(SR::INITOK) {}
    }

    fn enable_peripheral(&self) -> Result<(), ErrorCode> {
        // The state check also covers the window between accepting a request
        // and starting the hardware (the deferred call), when SR::BUSY is clear.
        if self.state.get() != State::Idle || self.registers.sr.is_set(SR::BUSY) {
            return Err(ErrorCode::BUSY);
        }
        self.registers.cr.modify(CR::EN::SET);
        self.wait_init_ok();
        Ok(())
    }

    fn start_operation(&self, mode: FieldValue<u32, CR::Register>) {
        self.registers.cr.modify(
            mode + CR::PROCENDIE::SET
                + CR::ADDERRIE::SET
                + CR::RAMERRIE::SET
                + CR::OPERRIE::SET
                + CR::EN::SET,
        );
        self.registers.cr.modify(CR::START::SET);
    }

    /// Starts `mode` if all inputs loaded, otherwise ends the operation with
    /// their error through `abort` (`math_done` or `ecc_done`).
    fn start_or_abort(
        &self,
        loaded: Result<(), ErrorCode>,
        mode: FieldValue<u32, CR::Register>,
        abort: fn(&Self, Result<(), ErrorCode>),
    ) {
        match loaded {
            Ok(()) => self.start_operation(mode),
            Err(e) => abort(self, Err(e)),
        }
    }

    // ---- Client data movement -----------------------------------------------
    //
    // PKA RAM holds numbers least significant word first, each word a native
    // u32. The HILs stream numbers big-endian, most significant byte first. A
    // byte at position `pos` of a `len`-byte number has significance
    // `s = len - 1 - pos`, so it lives in word `slot + s / 4` at bit offset
    // `8 * (s % 4)`. Bytes go straight between the client's slice and PKA RAM,
    // with no staging buffer, for any `len`.

    /// Zeroes the RAM slot for a `len`-byte number at `idx`, including the two
    /// zero words the PKA expects after every operand.
    fn clear_operand(&self, idx: usize, len: usize) {
        for cell in &self.registers.ram[idx..idx + len.div_ceil(4) + 2] {
            cell.set(0);
        }
    }

    /// ORs `bits` into a RAM word of a slot cleared by `clear_operand`.
    fn or_ram_word(&self, idx: usize, bits: u32) {
        let cell = &self.registers.ram[idx];
        cell.set(cell.get() | bits);
    }

    /// Calls the client callback that supplies `input`.
    fn request_input(
        &self,
        input: Input,
        sink: &mut dyn FnMut(&ReadableProcessSlice) -> Result<usize, ErrorCode>,
    ) -> Result<(), ErrorCode> {
        match input {
            Input::Modulus => self
                .math_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_modulus(sink),
            Input::First => self
                .math_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_number(sink),
            Input::Second => self
                .math_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_second_number(sink),
            Input::Scalar => self
                .ecc_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_scalar(sink),
            Input::Point => self
                .ecc_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_point(sink),
            Input::SecondPoint => self
                .ecc_client
                .get()
                .ok_or(ErrorCode::FAIL)?
                .read_second_point(sink),
        }
    }

    /// Streams one client input into PKA RAM.
    ///
    /// The stream is split evenly over `slots`, each holding a `len`-byte
    /// big-endian number: a scalar or math operand uses one slot, a point
    /// `x || y` uses two. Bytes are assembled into words, so each RAM word is
    /// read and written once per callback rather than once per byte.
    ///
    /// The client offers all of its remaining bytes, so a conforming client is
    /// fully read in one callback. The loop still accepts an input split over
    /// several callbacks. A callback that makes no progress (for example one
    /// that returns `Ok(())` without calling `sink`) aborts with `FAIL`.
    fn load_input(&self, input: Input, slots: &[usize], len: usize) -> Result<(), ErrorCode> {
        for &slot in slots {
            self.clear_operand(slot, len);
        }
        let total = slots.len() * len;

        let mut pos = 0;
        while pos < total {
            let before = pos;
            let mut sink = |slice: &ReadableProcessSlice| -> Result<usize, ErrorCode> {
                let n = slice.len().min(total - pos);
                if n == 0 {
                    return Err(ErrorCode::SIZE);
                }
                // (RAM word index, bits gathered for it so far)
                let mut pending: Option<(usize, u32)> = None;
                for byte in slice.iter().take(n) {
                    let sig = len - 1 - pos % len;
                    let word = slots[pos / len] + sig / 4;
                    let bits = u32::from(byte.get()) << (8 * (sig % 4));
                    pending = match pending {
                        Some((w, acc)) if w == word => Some((w, acc | bits)),
                        Some((w, acc)) => {
                            self.or_ram_word(w, acc);
                            Some((word, bits))
                        }
                        None => Some((word, bits)),
                    };
                    pos += 1;
                }
                if let Some((w, acc)) = pending {
                    self.or_ram_word(w, acc);
                }
                Ok(n)
            };
            self.request_input(input, &mut sink)?;
            if pos == before {
                return Err(ErrorCode::FAIL);
            }
        }
        Ok(())
    }

    // ---- Elliptic curves ----------------------------------------------------

    fn load_p256_parameters(&self) {
        let bits = (P_256_P_SIZE as u32) << 3;
        self.set_len(ECC_N_LEN_BITS_IDX, bits);
        self.set_len(ECC_P_LEN_BITS_IDX, bits);
        self.set_len(ECC_A_SIGN_IDX, 0);

        self.write_slice(ECC_A_ABS_IDX, &NistP256Constants::EQ_PARAMS.0);
        self.write_slice(ECC_B_IDX, &NistP256Constants::EQ_PARAMS.1);
        self.write_slice(ECC_N_IDX, &NistP256Constants::N);
        self.write_slice(ECC_P_R2_IDX, &P256_R2_MOD_P);
        self.write_slice(ECC_P_IDX, &NistP256Constants::P);
        self.write_slice(ECC_ADD_P_IDX, &NistP256Constants::P);
    }

    /// Streams a point (`x || y`) from the ECC client into two RAM slots.
    fn load_point(&self, input: Input, x_idx: usize, y_idx: usize) -> Result<(), ErrorCode> {
        self.load_input(input, &[x_idx, y_idx], P_256_P_SIZE)
    }

    /// Accepts an ECC request: inputs are read from the deferred call.
    fn ecc_start(&self, state: State) -> Result<(), ErrorCode> {
        if self.ecc_client.is_none() {
            return Err(ErrorCode::RESERVE);
        }
        self.enable_peripheral()?;
        self.state.set(state);
        self.load_p256_parameters();
        self.deferred_call.set();
        Ok(())
    }

    fn start_projective_to_affine(&self) {
        self.start_operation(CR::MODE::ECCProjectiveToAffine);
    }

    fn feed_affine_to_projective(&self) -> (P256Bytes, P256Bytes) {
        let mut x = [0u8; P_256_P_SIZE];
        let mut y = [0u8; P_256_P_SIZE];

        self.read_slice(ECC_OUT_X_IDX, &mut x);
        self.read_slice(ECC_OUT_Y_IDX, &mut y);
        self.write_slice(ECC_ADD_OUT_X_IDX, &x);
        self.write_slice(ECC_ADD_OUT_Y_IDX, &y);

        (x, y)
    }

    /// Delivers the resulting point through [`EccClient::write_point`].
    ///
    /// `produce` writes the x and y coordinates into the client's storage.
    fn deliver_point(
        &self,
        produce: &dyn Fn(&mut P256Bytes, &mut P256Bytes),
    ) -> Result<(), ErrorCode> {
        let client = self.ecc_client.get().ok_or(ErrorCode::FAIL)?;
        let mut filled = false;
        client.write_point(&mut |x: &mut P256Bytes, y: &mut P256Bytes| {
            produce(x, y);
            filled = true;
        })?;
        if filled { Ok(()) } else { Err(ErrorCode::FAIL) }
    }

    /// Ends the current ECC operation with exactly one completion callback.
    fn ecc_done(&self, result: Result<(), ErrorCode>) {
        self.state.set(State::Idle);
        self.ecc_client.map(|client| client.operation_done(result));
    }

    // ---- Modular arithmetic -------------------------------------------------

    /// Streams one math operand into the RAM slot at `idx`.
    fn load_operand(&self, idx: usize, input: Input) -> Result<(), ErrorCode> {
        self.load_input(input, &[idx], self.math_len.get())
    }

    /// Copies a `math_len`-byte number between RAM slots and terminates it.
    fn copy_operand(&self, from: usize, to: usize) {
        let words = self.math_len.get().div_ceil(4);
        for i in 0..words {
            let word = self.registers.ram[from + i].get();
            self.registers.ram[to + i].set(word);
        }
        self.registers.ram[to + words].set(0);
        self.registers.ram[to + words + 1].set(0);
    }

    /// Streams the result from `ARITH_RESULT` to the math client.
    fn deliver_result(&self) -> Result<(), ErrorCode> {
        let client = self.math_client.get().ok_or(ErrorCode::FAIL)?;
        let len = self.math_len.get();

        let mut pos = 0;
        while pos < len {
            let before = pos;
            client.write_number(&mut |slice: &WriteableProcessSlice| {
                let n = slice.len().min(len - pos);
                if n == 0 {
                    return Err(ErrorCode::SIZE);
                }
                // (RAM word index, its value): each word is read once.
                let mut cached: Option<(usize, u32)> = None;
                for byte in slice.iter().take(n) {
                    let sig = len - 1 - pos;
                    let word = ARITH_RESULT_IDX + sig / 4;
                    let value = match cached {
                        Some((w, v)) if w == word => v,
                        _ => {
                            let v = self.registers.ram[word].get();
                            cached = Some((word, v));
                            v
                        }
                    };
                    byte.set((value >> (8 * (sig % 4))) as u8);
                    pos += 1;
                }
                Ok(n)
            })?;
            if pos == before {
                return Err(ErrorCode::FAIL);
            }
        }
        Ok(())
    }

    /// Ends the current math operation with exactly one completion callback.
    fn math_done(&self, result: Result<(), ErrorCode>) {
        self.state.set(State::Idle);
        self.math_client
            .map(|client| client.computation_completed(result));
    }

    pub fn handle_interrupt(&self) {
        let sr = self.registers.sr.extract();
        if sr.is_set(SR::OPERRF) {
            self.registers.clrfr.write(CLRFR::OPERRFC::SET);
        }
        if sr.is_set(SR::ADDRERRF) {
            self.registers.clrfr.write(CLRFR::ADDERRFC::SET);
        }
        if sr.is_set(SR::RAMERRF) {
            self.registers.clrfr.write(CLRFR::RAMERRFC::SET);
        }
        let success = sr.is_set(SR::PROCENDF);
        if success {
            self.registers.clrfr.write(CLRFR::PROCENDFC::SET);
        }

        // In every arm that delivers a result, the state stays busy until
        // `ecc_done`/`math_done`, so a client that starts a new operation from
        // a data callback gets BUSY. The state is freed before the completion
        // callback, so a new operation can start from there.
        match self.state.get() {
            State::Idle => {}

            State::Rsa => {
                let modulus = self.modulus.take().unwrap();
                let exponent = self.exponent.take().unwrap();
                let message = self.message.take().unwrap();
                let result = self.result.take().unwrap();
                self.state.set(State::Idle);

                let outcome = if success {
                    self.read_slice(MODEXP_RESULT_IDX, result);
                    Ok(true)
                } else {
                    Err(ErrorCode::FAIL)
                };
                self.rsa_client.map(|client| {
                    client.mod_exponent_done(outcome, message, modulus, exponent, result)
                });
            }

            State::ScalarMul => {
                let result = if success {
                    self.deliver_point(&|x: &mut P256Bytes, y: &mut P256Bytes| {
                        self.read_slice(ECC_OUT_X_IDX, x);
                        self.read_slice(ECC_OUT_Y_IDX, y);
                    })
                } else {
                    Err(ErrorCode::FAIL)
                };
                self.ecc_done(result);
            }

            State::PointAddition => {
                if !success {
                    self.ecc_done(Err(ErrorCode::FAIL));
                    return;
                }
                self.state.set(State::ProjToAffinePass1);
                self.start_projective_to_affine();
            }

            State::ProjToAffinePass1 => {
                if !success {
                    self.ecc_done(Err(ErrorCode::FAIL));
                    return;
                }
                self.state.set(State::ProjToAffinePass2);
                self.feed_affine_to_projective();
                self.start_projective_to_affine();
            }

            State::ProjToAffinePass2 => {
                if !success {
                    self.ecc_done(Err(ErrorCode::FAIL));
                    return;
                }
                self.state.set(State::ProjToAffinePass3);
                let (x_out, _) = self.feed_affine_to_projective();
                self.ecc_x.set(x_out);
                self.start_projective_to_affine();
            }

            State::ProjToAffinePass3 => {
                let result = if success {
                    self.deliver_point(&|x: &mut P256Bytes, y: &mut P256Bytes| {
                        *x = self.ecc_x.get();
                        self.read_slice(ECC_OUT_Y_IDX, y);
                    })
                } else {
                    Err(ErrorCode::FAIL)
                };
                self.ecc_done(result);
            }

            State::VerifyPoint => {
                let result = if !success {
                    Err(ErrorCode::FAIL)
                } else if self.registers.ram[FPCHECK_RESULT_IDX].get() == ECC_RESULT_OK {
                    Ok(())
                } else {
                    Err(ErrorCode::INVAL)
                };
                self.ecc_done(result);
            }

            State::MathAddition | State::MathModulus | State::MathInvert | State::MathComputeAB => {
                let result = if success {
                    self.deliver_result()
                } else {
                    Err(ErrorCode::FAIL)
                };
                self.math_done(result);
            }

            State::MathComputeR2 => {
                if !success {
                    self.math_done(Err(ErrorCode::FAIL));
                    return;
                }
                // R^2 mod n becomes the first operand; the client's A the second.
                self.copy_operand(MONT_R2_OUT_IDX, ARITH_OP1_IDX);
                let loaded = self.load_operand(ARITH_OP2_IDX, Input::First);
                self.state.set(State::MathComputeAR);
                self.start_or_abort(loaded, CR::MODE::MontgomeryMultiplication, Self::math_done);
            }

            State::MathComputeAR => {
                if !success {
                    self.math_done(Err(ErrorCode::FAIL));
                    return;
                }
                // A*R becomes the second operand; the client's B the first.
                self.copy_operand(ARITH_RESULT_IDX, ARITH_OP2_IDX);
                let loaded = self.load_operand(ARITH_OP1_IDX, Input::Second);
                self.state.set(State::MathComputeAB);
                self.start_or_abort(loaded, CR::MODE::MontgomeryMultiplication, Self::math_done);
            }
        }
    }
}

fn get_bitlen(data: &[u8]) -> u32 {
    data.iter().position(|&b| b != 0).map_or(0, |i| {
        (8 - data[i].leading_zeros()) + ((data.len() - 1 - i) as u32) * 8
    })
}

impl<'a> RsaCryptoBase<'a> for Pka<'a> {
    fn set_client(&'a self, client: &'a dyn Client<'a>) {
        self.rsa_client.set(client);
    }

    fn clear_data(&self) {
        self.clear_ram();
    }

    fn mod_exponent(
        &self,
        message: &'static mut [u8],
        modulus: &'static [u8],
        exponent: &'static [u8],
        result: &'static mut [u8],
    ) -> Result<
        (),
        (
            ErrorCode,
            &'static mut [u8],
            &'static [u8],
            &'static [u8],
            &'static mut [u8],
        ),
    > {
        if self.registers.sr.is_set(SR::BUSY) || self.state.get() != State::Idle {
            return Err((ErrorCode::BUSY, message, modulus, exponent, result));
        }

        if result.len() < modulus.len() || exponent.is_empty() || message.is_empty() {
            return Err((ErrorCode::SIZE, message, modulus, exponent, result));
        }

        let exp_bits = get_bitlen(exponent);
        let op_bits = get_bitlen(modulus);

        if exp_bits == 0 || op_bits == 0 {
            return Err((ErrorCode::INVAL, message, modulus, exponent, result));
        }

        self.registers.cr.modify(CR::EN::SET);
        self.wait_init_ok();

        self.state.set(State::Rsa);
        self.clear_ram();

        self.set_len(EXP_LEN_BITS_IDX, exp_bits);
        self.set_len(OPERAND_LEN_BITS_IDX, op_bits);

        self.write_slice(MODEXP_EXPONENT_IDX, exponent);
        self.write_slice(MODULUS_IDX, modulus);
        self.write_slice(MODEXP_BASE_IDX, message);

        self.message.replace(message);
        self.modulus.set(modulus);
        self.exponent.set(exponent);
        self.result.replace(result);

        self.start_operation(CR::MODE::MontgomeryModularExp);
        Ok(())
    }
}

impl<'a> EccCrypto<'a, P_256_P_SIZE, NistP256Constants> for Pka<'a> {
    fn set_client(&self, client: &'a P256Client<'a>) {
        self.ecc_client.replace(client);
    }

    fn point_addition(&self) -> Result<(), ErrorCode> {
        self.ecc_start(State::PointAddition)
    }

    fn scalar_multiplication(&self) -> Result<(), ErrorCode> {
        self.ecc_start(State::ScalarMul)
    }
}

impl<'a> VerifyEccPoint<'a, P_256_P_SIZE, NistP256Constants> for Pka<'a> {
    fn verify_point(&self) -> Result<(), ErrorCode> {
        self.ecc_start(State::VerifyPoint)
    }
}

impl<'a> ModularArithmetic<'a, SupportedOp> for Pka<'a> {
    fn set_client(&self, client: &'a dyn MathClient<SupportedOp>) {
        self.math_client.replace(client);
    }

    fn start_computation(
        &self,
        modulus_len: usize,
        operation: SupportedOp,
    ) -> Result<(), ErrorCode> {
        if self.math_client.is_none() {
            return Err(ErrorCode::RESERVE);
        }
        if modulus_len == 0 || modulus_len > MATH_MAX_LEN {
            return Err(ErrorCode::SIZE);
        }
        self.enable_peripheral()?;

        let state = match operation {
            SupportedOp::Addition => State::MathAddition,
            SupportedOp::Multiplication => State::MathComputeR2,
            SupportedOp::Inverse => State::MathInvert,
            SupportedOp::Modulus => State::MathModulus,
        };
        self.math_len.set(modulus_len);
        self.state.set(state);
        self.set_len(OPERAND_LEN_BITS_IDX, (modulus_len as u32) * 8);
        // Operands are read from the deferred call, never from within this call.
        self.deferred_call.set();
        Ok(())
    }
}

impl DeferredCallClient for Pka<'_> {
    fn handle_deferred_call(&self) {
        match self.state.get() {
            State::ScalarMul => {
                let loaded = self
                    .load_input(Input::Scalar, &[ECC_MUL_K_IDX], P_256_P_SIZE)
                    .and_then(|()| {
                        self.load_point(Input::Point, ECC_MUL_IN_X_IDX, ECC_MUL_IN_Y_IDX)
                    });
                self.start_or_abort(loaded, CR::MODE::MontgomeryECC, Self::ecc_done);
            }
            State::PointAddition => {
                // Both inputs are affine; the PKA adds projective points with Z = 1.
                let mut z_coord = [0u8; P_256_P_SIZE];
                z_coord[P_256_P_SIZE - 1] = 1;
                let loaded = self
                    .load_point(Input::Point, ECC_ADD_PT1_X_IDX, ECC_ADD_PT1_Y_IDX)
                    .and_then(|()| {
                        self.load_point(Input::SecondPoint, ECC_ADD_PT2_X_IDX, ECC_ADD_PT2_Y_IDX)
                    });
                self.write_slice(ECC_ADD_PT1_Z_IDX, &z_coord);
                self.write_slice(ECC_ADD_PT2_Z_IDX, &z_coord);
                self.start_or_abort(loaded, CR::MODE::ECCCompleteAddition, Self::ecc_done);
            }
            State::VerifyPoint => {
                let loaded = self.load_point(Input::Point, FPCHECK_X_IDX, FPCHECK_Y_IDX);
                self.start_or_abort(loaded, CR::MODE::FpCheck, Self::ecc_done);
            }
            State::MathAddition => {
                let loaded = self
                    .load_operand(MODULUS_IDX, Input::Modulus)
                    .and_then(|()| self.load_operand(ARITH_OP1_IDX, Input::First))
                    .and_then(|()| self.load_operand(ARITH_OP2_IDX, Input::Second));
                self.start_or_abort(loaded, CR::MODE::ModularAddition, Self::math_done);
            }
            State::MathInvert => {
                let loaded = self
                    .load_operand(INV_RED_MODULUS_IDX, Input::Modulus)
                    .and_then(|()| self.load_operand(ARITH_OP1_IDX, Input::First));
                self.start_or_abort(loaded, CR::MODE::ModularInversion, Self::math_done);
            }
            State::MathComputeR2 => {
                let loaded = self.load_operand(MODULUS_IDX, Input::Modulus);
                self.start_or_abort(loaded, CR::MODE::MontgomeryOnly, Self::math_done);
            }
            State::MathModulus => {
                self.set_len(EXP_LEN_BITS_IDX, (self.math_len.get() as u32) * 8);
                let loaded = self
                    .load_operand(INV_RED_MODULUS_IDX, Input::Modulus)
                    .and_then(|()| self.load_operand(ARITH_OP1_IDX, Input::First));
                self.start_or_abort(loaded, CR::MODE::ModularReduction, Self::math_done);
            }
            _ => {}
        }
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
