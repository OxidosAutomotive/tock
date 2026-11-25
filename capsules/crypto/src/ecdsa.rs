// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026

//! ECDSA Signer for P256 signatures using Hardware Accelerators.
//!
//! This implements deterministic ECDSA signing per RFC 6979
//! combined with SEC1 signature computation, using
//! hardware-accelerated HMAC-SHA256, elliptic-curve scalar multiplication,
//! and big-integer modular arithmetic peripherals.
//!
//! # Algorithm outline
//!
//! Given a private key `d`, message hash `z`  and curve order `n`:
//!
//! 1. **RFC 6979 nonce derivation** (states `HmacDerivingK1` through
//!    `HmacGeneratingK`): derive a deterministic nonce `k` from `d` and
//!    `z` via repeated HMAC-SHA256, seeded with `K = 0x00..00`,
//!    `V = 0x01..01`:
//!      - `K1 = HMAC(K, V || 0x00 || d || z)`, `V1 = HMAC(K1, V)`
//!      - `K2 = HMAC(K1, V1 || 0x01 || d || z)`, `V2 = HMAC(K2, V1)`
//!      - `T = HMAC(K2, V2)`; if `0 < T < n`, `k = T`; otherwise loop
//!        (`HmacGetNewK` / `HmacDerivingV2`) generating more `T` bytes.
//! 2. **R computation** (`EccCalculatingR`): compute the EC point `k*G`
//!    via the hardware ECC accelerator; `r = (k*G).x`.
//!    If `r == 0`, the nonce is rejected and step 1's retry loop
//!    (`InvalidR` / `HmacGetNewK`) runs again to derive a new `k`.
//! 3. **R reduction** (`MathModR`): `r = r mod n`.
//! 4. **S computation**, done in three hardware modular-arithmetic
//!    passes chained through `computation_completed`:
//!      - `MathRDaMul`: `t = r * d mod n`
//!      - `MathResHAdd`: `t = t + z mod n`  (stored back into `s_val`)
//!      - `MathResKDiv`: `s = t / k mod n`  (i.e. `s = k^-1 * (z + r*d) mod n`)
//!    If `s == 0`, the nonce is rejected and control returns to the
//!    RFC 6979 retry loop (`HmacGetNewK`) to derive a new `k`, exactly
//!    as in the `r == 0` case.
//! 5. Signature is `(r, s)`.
//!
//! # Threat model note on timing
//!
//! `check_t`/`check_r`/`check_s` are written to avoid secret-dependent
//! *branches* on individual bytes of `t`/`r`/`s` (all bytes are always
//! scanned, comparisons are accumulated with bitwise operators rather
//! than short-circuiting `if`s). This hides the byte-level memory access
//! pattern of the comparison itself. It does **not** make the surrounding
//! control flow constant-time: the number of HMAC/ECC/math peripheral
//! operations issued, and which state-machine branch runs next, still
//! depends on whether a candidate nonce was rejected. Closing that
//! coarser-grained timing channel (peripheral op counts, request timing)
//! is out of scope for this module; these routines only protect against
//! leaking the comparison result through data-dependent branching within
//! `check_t`/`check_r`/`check_s` themselves.

use capsules_core::driver_mutex::{
    DriverMutex, DriverMutexClient, DriverMutexHandle, DriverMutexRef,
};
use core::cell::Cell;
use core::marker::PhantomData;
use kernel::ErrorCode;
use kernel::hil;
use kernel::hil::crypto::modular_arithmetic::{
    MathClient, ModularArithmetic, OpAddition, OpDivision, OpModulo, OpMultiplication,
};
use kernel::hil::crypto::{
    digest::{Algorithm, Hmac},
    elliptic_curves::{
        ecc_constants::{Curve, NistP256Constants},
        ecc_math::{EccClient, EccCrypto},
    },
};
use kernel::hil::public_key_crypto::keys::SetKeyBySliceClient;
use kernel::hil::public_key_crypto::signature::ClientSign;
use kernel::utilities::cells::MapCell;
use kernel::utilities::cells::{OptionalCell, TakeCell};

const P_LEN: usize = 32;
const SIG_LEN: usize = P_LEN * 2;

#[derive(Clone, Copy, PartialEq)]
enum Operand {
    First,
    Second,
}

#[derive(Clone, Copy, PartialEq)]
enum State {
    /// No signing operation in progress.
    Idle,
    /// Reducing the input hash `z` mod `n` (only needed if `z >= n`).
    ModHash,
    /// RFC 6979: `K1 = HMAC(K, V || 0x00 || d || z)`.
    HmacDerivingK1,
    /// RFC 6979: `V1 = HMAC(K1, V)`.
    HmacDerivingV1,
    /// RFC 6979: `K2 = HMAC(K1, V1 || 0x01 || d || z)`.
    HmacDerivingK2,
    /// RFC 6979: `V2 = HMAC(K2, V1)`.
    HmacDerivingV2,
    /// RFC 6979: `T = HMAC(K2, V2)`; candidate nonce `k`.
    HmacGeneratingK,
    /// RFC 6979 retry loop: candidate `k` was rejected (`r == 0` or
    /// `s == 0`); re-derive `K`/`V` before generating a fresh `T`.
    HmacGetNewK,
    /// SEC1: compute `R = k * G` on the hardware ECC accelerator.
    EccCalculatingR,
    /// `r = R.x == 0`: candidate nonce rejected, re-enter the RFC 6979
    /// retry loop to derive a new `k`.
    InvalidR,
    /// SEC1: `r = r mod n`.
    MathModR,
    /// SEC1: `t = r * d mod n`.
    MathRDaMul,
    /// SEC1: `t = t + z mod n`.
    MathResHAdd,
    /// SEC1: `s = t / k mod n` (i.e. `k^-1 * (z + r*d) mod n`).
    MathResKDiv,
    /// Asynchronously installing a new signing key via a deferred call.
    ChangingKey,
}

pub struct EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants> + 'static,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op> + 'static,
    H: Hmac + 'static,
{
    // Clients
    client: OptionalCell<&'a dyn ClientSign<P_LEN, 64>>,
    client_key_set: OptionalCell<&'a dyn SetKeyBySliceClient<32>>,

    // Hardware support
    ecc_mutex: &'a DriverMutex<E>,
    ecc: MapCell<DriverMutexRef<E>>,
    ecc_handle: OptionalCell<DriverMutexHandle>,
    math_mutex: &'a DriverMutex<M>,
    math: MapCell<DriverMutexRef<M>>,
    math_handle: OptionalCell<DriverMutexHandle>,
    hmac_mutex: &'a DriverMutex<H>,
    hmac: MapCell<DriverMutexRef<H>>,
    hmac_handle: OptionalCell<DriverMutexHandle>,

    // Cryptographic storage
    signing_key: TakeCell<'static, [u8; P_LEN]>,
    hash_storage: TakeCell<'static, [u8; P_LEN]>,
    signature_storage: TakeCell<'static, [u8; 64]>,

    // Internal variables
    k_val: Cell<[u8; P_LEN]>,
    v_val: Cell<[u8; P_LEN]>,
    r_val: Cell<[u8; P_LEN]>,
    s_val: Cell<[u8; P_LEN]>,
    t_val: Cell<[u8; P_LEN]>,
    input_counter: Cell<usize>,
    output_counter: Cell<usize>,
    key_counter: Cell<usize>,

    // State and state switching
    state: Cell<State>,
    current_operand: Cell<Operand>,
    deferred_call: kernel::deferred_call::DeferredCall,
    new_key_buffer: TakeCell<'static, [u8; P_LEN]>,
    _phantom: PhantomData<Op>,
}

impl<'a, E, Op, M, H> EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    pub fn new(
        signing_key: &'static mut [u8; P_LEN],
        ecc_mutex: &'a DriverMutex<E>,
        math_mutex: &'a DriverMutex<M>,
        hmac_mutex: &'a DriverMutex<H>,
    ) -> Self {
        Self {
            client: OptionalCell::empty(),
            client_key_set: OptionalCell::empty(),
            ecc_mutex,
            ecc: MapCell::empty(),
            ecc_handle: OptionalCell::empty(),
            math_mutex,
            math: MapCell::empty(),
            math_handle: OptionalCell::empty(),
            hmac_mutex,
            hmac: MapCell::empty(),
            hmac_handle: OptionalCell::empty(),
            signing_key: TakeCell::new(signing_key),
            hash_storage: TakeCell::empty(),
            signature_storage: TakeCell::empty(),
            k_val: Cell::new([0; P_LEN]),
            v_val: Cell::new([0; P_LEN]),
            r_val: Cell::new([0; P_LEN]),
            s_val: Cell::new([0; P_LEN]),
            t_val: Cell::new([0; P_LEN]),
            input_counter: Cell::new(0),
            output_counter: Cell::new(0),
            key_counter: Cell::new(0),
            state: Cell::new(State::Idle),
            current_operand: Cell::new(Operand::First),
            deferred_call: kernel::deferred_call::DeferredCall::new(),
            new_key_buffer: TakeCell::empty(),
            _phantom: PhantomData::<Op>,
        }
    }

    pub fn register_hmac(&'static self) -> Result<(), ErrorCode> {
        if self.hmac_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let hmac_handle = self.hmac_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.hmac_handle.set(hmac_handle);
        Ok(())
    }

    pub fn register_ecc(&'static self) -> Result<(), ErrorCode> {
        if self.ecc_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let ecc_handle = self.ecc_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.ecc_handle.set(ecc_handle);
        Ok(())
    }

    pub fn register_math(&'static self) -> Result<(), ErrorCode> {
        if self.math_handle.is_some() {
            return Err(ErrorCode::ALREADY);
        }

        let math_handle = self.math_mutex.add_client(self).ok_or(ErrorCode::NOMEM)?;
        self.math_handle.set(math_handle);
        Ok(())
    }

    fn request_hmac(&self, size: usize) {
        self.hmac
            .map(|hmac| hmac.authenticate(Algorithm::Sha256, size, P_LEN));
    }

    fn complete_signature(&self, result: Result<(), ErrorCode>) {
        self.state.set(State::Idle);
        self.ecc.take();
        self.math.take();
        self.hmac.take();

        if let Some(client) = self.client.get() {
            if let (Some(h), Some(s)) = (self.hash_storage.take(), self.signature_storage.take()) {
                s[0..P_LEN].copy_from_slice(&self.r_val.get());
                s[P_LEN..SIG_LEN].copy_from_slice(&self.s_val.get());
                client.signing_done(result, h, s);
            }
        }

        self.k_val.set([0; P_LEN]);
        self.v_val.set([0; P_LEN]);
        self.r_val.set([0; P_LEN]);
        self.s_val.set([0; P_LEN]);
        self.t_val.set([0; P_LEN]);
        self.input_counter.set(0);
        self.output_counter.set(0);
        self.key_counter.set(0);
    }

    fn update_var_from_buf(&self, var: &Cell<[u8; P_LEN]>, index: usize, buf: &[u8]) -> usize {
        let mut var_buf = var.get();
        let len = core::cmp::min(buf.len(), P_LEN - index);
        var_buf[index..index + len].copy_from_slice(&buf[..len]);
        var.set(var_buf);
        len
    }

    fn read_var_to_buf(&self, var: &Cell<[u8; P_LEN]>, index: usize, buf: &mut [u8]) -> usize {
        let var_buf = var.get();
        let len = core::cmp::min(buf.len(), P_LEN - index);
        buf[..len].copy_from_slice(&var_buf[index..index + len]);
        len
    }

    /// Returns whether the candidate nonce `t` is a valid `k`: nonzero
    /// and strictly less than the curve order `n` (RFC 6979 step 3.2.h,
    /// SEC1 nonce validity requirement). Every byte of `t` is always
    /// scanned and compared, and the running results are combined with
    /// bitwise `|=`/`&=` rather than early-exiting `if`s, so the byte at
    /// which `t` and `n` first differ is not revealed through the
    /// control-flow/memory-access pattern of this function. See the
    /// module-level "Threat model note on timing" for what this does
    /// and does not protect against.
    fn check_t(&self) -> bool {
        let t = self.t_val.get();
        let mut non_zero = false;
        let mut t_less_than_n = false;
        let mut exactly_equal_so_far = true;

        t.iter()
            .zip(NistP256Constants::N.iter())
            .for_each(|(&key_byte, &order_byte)| {
                non_zero |= key_byte != 0;
                let byte_less = key_byte < order_byte;
                let byte_equal = key_byte == order_byte;
                t_less_than_n |= byte_less & exactly_equal_so_far;
                exactly_equal_so_far &= byte_equal;
            });
        t_less_than_n & non_zero
    }

    /// Returns whether `r` (the x-coordinate of `k*G`, mod `n`) is
    /// nonzero, i.e. an acceptable signature component per SEC1. See
    /// `check_t` for the constant-time-comparison rationale.
    fn check_r(&self) -> bool {
        let r = self.r_val.get();
        let mut non_zero = false;
        r.iter().for_each(|&r_byte| {
            non_zero |= r_byte != 0;
        });
        non_zero
    }

    /// Returns whether `s = k^-1 * (z + r*d) mod n` is nonzero, i.e. an
    /// acceptable signature component per SEC1. See `check_t` for the
    /// constant-time-comparison rationale.
    fn check_s(&self) -> bool {
        let s = self.s_val.get();
        let mut non_zero = false;
        s.iter().for_each(|&s_byte| {
            non_zero |= s_byte != 0;
        });
        non_zero
    }
}

impl<'a, E, Op, M, H> hil::public_key_crypto::signature::SignatureSign<'a, P_LEN, 64>
    for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn set_sign_client(
        &self,
        client: &'a dyn hil::public_key_crypto::signature::ClientSign<P_LEN, 64>,
    ) {
        self.client.replace(client);
    }

    fn sign(
        &self,
        hash: &'static mut [u8; P_LEN],
        signature: &'static mut [u8; 64],
    ) -> Result<(), (ErrorCode, &'static mut [u8; P_LEN], &'static mut [u8; 64])> {
        if self.state.get() != State::Idle || self.signing_key.is_none() {
            return Err((ErrorCode::BUSY, hash, signature));
        }
        self.v_val.set([0x01; P_LEN]);
        self.k_val.set([0x00; P_LEN]);

        self.input_counter.set(0);
        self.output_counter.set(0);
        self.key_counter.set(0);

        self.hash_storage.replace(hash);
        self.signature_storage.replace(signature);

        if self
            .hash_storage
            .map_or(false, |h| *h >= NistP256Constants::N)
        {
            self.state.set(State::ModHash);
            if let Some(handle) = self.math_handle.get() {
                let _ = self.math_mutex.request(handle);
            }
        } else {
            self.state.set(State::HmacDerivingK1);
            if let Some(handle) = self.hmac_handle.get() {
                let _ = self.hmac_mutex.request(handle);
            }
        }

        Ok(())
    }
}

impl<'a, E, Op, M, H> DriverMutexClient for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn ready(&'static self, resource: capsules_core::driver_mutex::DriverMutexAny) {
        match self.state.get() {
            State::ModHash => {
                if let Ok(math) = resource.downcast::<M>() {
                    math.set_client(self);
                    self.math.put(math);
                    self.math
                        .map(|math| math.start_computation(P_LEN, OpModulo::modulo()));
                }
            }
            State::HmacDerivingK1 => {
                if let Ok(hmac) = resource.downcast::<H>() {
                    hmac.set_hmac_client(self);
                    self.hmac.put(hmac);
                    self.request_hmac(P_LEN + 1 + P_LEN + P_LEN);
                }
            }
            State::InvalidR => {
                if let Ok(hmac) = resource.downcast::<H>() {
                    hmac.set_hmac_client(self);
                    self.hmac.put(hmac);
                    self.state.set(State::HmacGetNewK);
                    self.request_hmac(P_LEN + 1);
                }
            }
            // Reached when `s == 0` was detected in `computation_completed`
            // (`MathResKDiv` arm) and the HMAC mutex was re-requested to
            // derive a fresh nonce candidate. Without this arm the mutex
            // grant was silently dropped here, the retry never issued a
            // new HMAC operation, and the driver stayed stuck outside
            // `Idle` forever (all future `sign()` calls returning `BUSY`).
            // Mirrors the `InvalidR` arm above, which handles the
            // equivalent `r == 0` retry.
            State::HmacGetNewK => {
                if let Ok(hmac) = resource.downcast::<H>() {
                    hmac.set_hmac_client(self);
                    self.hmac.put(hmac);
                    self.request_hmac(P_LEN + 1);
                }
            }
            State::EccCalculatingR => {
                if let Ok(ecc) = resource.downcast::<E>() {
                    ecc.set_client(self);
                    self.ecc.put(ecc);
                    self.ecc.map(|ecc| ecc.scalar_multiplication(true));
                }
            }
            State::MathModR => {
                if let Ok(math) = resource.downcast::<M>() {
                    math.set_client(self);
                    self.math.put(math);
                    self.math
                        .map(|math| math.start_computation(P_LEN, OpModulo::modulo()));
                }
            }
            State::MathRDaMul => {
                if let Ok(math) = resource.downcast::<M>() {
                    math.set_client(self);
                    self.math.put(math);
                    self.current_operand.set(Operand::First);
                    self.math.map(|math| {
                        math.start_computation(P_LEN, OpMultiplication::multiplication())
                    });
                }
            }
            _ => {}
        }
    }
}

impl<'a, E, Op, M, H> kernel::hil::crypto::digest::Client
    for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn read_input(&self, input: &mut [u8]) -> Result<usize, ErrorCode> {
        let state = self.state.get();
        let index = self.input_counter.get();

        match state {
            State::HmacDerivingK2 | State::HmacDerivingK1 => {
                let v = self.v_val.get();
                let single_byte = if matches!(state, State::HmacDerivingK1) {
                    0x00
                } else {
                    0x01
                };
                let mut copied = 0;

                if let Some(d_a) = self.signing_key.take() {
                    if let Some(hash) = self.hash_storage.take() {
                        let mut combined = [0u8; 97]; // P_LEN * 3 + 1
                        combined[0..P_LEN].copy_from_slice(&v);
                        combined[P_LEN] = single_byte;
                        combined[P_LEN + 1..P_LEN * 2 + 1].copy_from_slice(d_a);
                        combined[P_LEN * 2 + 1..P_LEN * 3 + 1].copy_from_slice(hash);

                        let total_len = P_LEN * 3 + 1;
                        let len = core::cmp::min(input.len(), total_len - index);
                        input[..len].copy_from_slice(&combined[index..index + len]);
                        copied = len;

                        self.hash_storage.replace(hash);
                    }
                    self.signing_key.replace(d_a);
                }
                self.input_counter.set(index + copied);
                Ok(copied)
            }
            State::HmacDerivingV1 | State::HmacDerivingV2 | State::HmacGeneratingK => {
                let counter = self.read_var_to_buf(&self.v_val, index, input);
                self.input_counter.set(index + counter);
                Ok(counter)
            }
            State::HmacGetNewK => {
                let mut counter = self.read_var_to_buf(&self.v_val, index, input);
                if index + counter == P_LEN && counter < input.len() {
                    input[counter] = 0;
                    counter += 1;
                }
                self.input_counter.set(index + counter);
                Ok(counter)
            }
            _ => Err(ErrorCode::FAIL),
        }
    }

    fn write_output(&self, output: &[u8]) -> Result<(), ErrorCode> {
        let index = self.output_counter.get();
        match self.state.get() {
            State::HmacDerivingK1 | State::HmacDerivingK2 | State::HmacGetNewK => {
                self.output_counter
                    .set(index + self.update_var_from_buf(&self.k_val, index, output));
            }
            State::HmacDerivingV1 | State::HmacDerivingV2 => {
                self.output_counter
                    .set(index + self.update_var_from_buf(&self.v_val, index, output));
            }
            State::HmacGeneratingK => {
                self.output_counter
                    .set(index + self.update_var_from_buf(&self.t_val, index, output));
                let t_copy = self.t_val.get();
                self.update_var_from_buf(&self.v_val, index, &t_copy);
            }
            _ => return Err(ErrorCode::FAIL),
        }
        Ok(())
    }

    fn hash_done(&self, result: Result<(), ErrorCode>) {
        self.key_counter.set(0);
        self.input_counter.set(0);
        self.output_counter.set(0);
        if result.is_err() {
            self.complete_signature(result);
            return;
        }

        match self.state.get() {
            State::HmacDerivingK1 => {
                self.request_hmac(P_LEN);
                self.state.set(State::HmacDerivingV1);
            }
            State::HmacDerivingV1 => {
                self.request_hmac(P_LEN + 1 + P_LEN * 2);
                self.state.set(State::HmacDerivingK2);
            }
            State::HmacDerivingK2 => {
                self.request_hmac(P_LEN);
                self.state.set(State::HmacDerivingV2);
            }
            State::HmacDerivingV2 => {
                self.request_hmac(P_LEN);
                self.state.set(State::HmacGeneratingK);
            }
            State::HmacGeneratingK => {
                if self.check_t() {
                    let temp_t = self.t_val.get();
                    self.k_val.set(temp_t);
                    self.t_val.set([0; P_LEN]);
                    self.hmac.take();
                    self.state.set(State::EccCalculatingR);
                    if let Some(handle) = self.ecc_handle.get() {
                        let _ = self.ecc_mutex.request(handle);
                    }
                } else {
                    self.request_hmac(P_LEN + 1);
                    self.state.set(State::HmacGetNewK);
                }
            }
            State::HmacGetNewK => {
                self.request_hmac(P_LEN);
                self.state.set(State::HmacDerivingV2);
            }
            _ => {
                self.complete_signature(Err(ErrorCode::FAIL));
            }
        }
    }
}

impl<'a, E, Op, M, H> kernel::hil::crypto::digest::HmacClient
    for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn read_key(&self, key: &mut [u8]) -> Result<usize, ErrorCode> {
        // `k_val` is the current HMAC key `K` throughout the whole RFC 6979
        // derivation (every state from `HmacDerivingK1` through
        // `HmacGetNewK` uses `K` as the HMAC key at some point), so unlike
        // `read_scalar`/`write_point` below there is no single state to
        // guard against; restrict to `Idle`/`ChangingKey`, the only states
        // in which `k_val` is not meaningful HMAC key material.
        if matches!(self.state.get(), State::Idle | State::ChangingKey) {
            return Err(ErrorCode::FAIL);
        }
        let index = self.key_counter.get();
        let counter = self.read_var_to_buf(&self.k_val, index, key);
        self.key_counter.set(counter + index);
        Ok(counter)
    }
}

impl<'a, E, Op, M, H> EccClient for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn read_scalar(&self, scalar: &mut [u8]) -> Result<(), ErrorCode> {
        if self.state.get() == State::EccCalculatingR {
            let index = self.input_counter.get();
            let counter = self.read_var_to_buf(&self.k_val, index, scalar);
            self.input_counter.set(index + counter);
            Ok(())
        } else {
            Err(ErrorCode::FAIL)
        }
    }

    fn read_point(&self, _point: &mut [u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::INVAL)
    }

    fn read_second_point(&self, _point: &mut [u8]) -> Result<(), ErrorCode> {
        Err(ErrorCode::INVAL)
    }

    fn write_point(&self, point: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get() == State::EccCalculatingR {
            let index = self.output_counter.get();
            let counter = self.update_var_from_buf(&self.r_val, index, point);
            self.output_counter.set(index + counter);
            Ok(())
        } else {
            Err(ErrorCode::FAIL)
        }
    }

    fn operation_done(&self, result: Result<(), ErrorCode>) {
        self.ecc.take();
        self.input_counter.set(0);
        self.output_counter.set(0);
        if result.is_err() {
            self.complete_signature(result);
            return;
        }

        if !self.check_r() {
            self.state.set(State::InvalidR);
            if let Some(handle) = self.hmac_handle.get() {
                let _ = self.hmac_mutex.request(handle);
            }
            return;
        }

        self.state.set(State::MathModR);
        if let Some(handle) = self.math_handle.get() {
            let _ = self.math_mutex.request(handle);
        }
    }
}

impl<'a, E, Op, M, H> MathClient<Op> for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn read_modulus(&self, modulus: &mut [u8]) -> Result<(), ErrorCode> {
        let index = self.key_counter.get();
        let end_index = (index + modulus.len()).min(P_LEN);
        modulus.copy_from_slice(&NistP256Constants::N[index..end_index]);
        self.key_counter.set(end_index);
        Ok(())
    }

    fn read_number(&self, num: &mut [u8]) {
        let index = self.input_counter.get();
        match self.state.get() {
            State::ModHash => {
                if let Some(hash) = self.hash_storage.take() {
                    let len = core::cmp::min(num.len(), P_LEN - index);
                    num[..len].copy_from_slice(&hash[index..index + len]);
                    self.hash_storage.replace(hash);

                    if len + index < P_LEN {
                        self.input_counter.set(len + index);
                    } else {
                        self.input_counter.set(0);
                    }
                }
            }
            State::MathModR => {
                let counter = self.read_var_to_buf(&self.r_val, index, num);
                if counter + index < P_LEN {
                    self.input_counter.set(counter + index);
                } else {
                    self.input_counter.set(0);
                }
            }
            State::MathRDaMul => match self.current_operand.get() {
                Operand::First => {
                    let counter = self.read_var_to_buf(&self.r_val, index, num);
                    if counter + index < P_LEN {
                        self.input_counter.set(counter + index);
                    } else {
                        self.current_operand.set(Operand::Second);
                        self.input_counter.set(0);
                    }
                }
                Operand::Second => {
                    if let Some(key) = self.signing_key.take() {
                        let len = core::cmp::min(num.len(), P_LEN - index);
                        num[..len].copy_from_slice(&key[index..index + len]);
                        self.signing_key.replace(key);

                        if len + index < P_LEN {
                            self.input_counter.set(len + index);
                        } else {
                            self.input_counter.set(0);
                        }
                    }
                }
            },
            State::MathResHAdd | State::MathResKDiv => match self.current_operand.get() {
                Operand::First => {
                    let counter = self.read_var_to_buf(&self.s_val, index, num);
                    if counter + index < P_LEN {
                        self.input_counter.set(counter + index);
                    } else {
                        self.current_operand.set(Operand::Second);
                        self.input_counter.set(0);
                    }
                }
                Operand::Second => {
                    if matches!(self.state.get(), State::MathResHAdd) {
                        if let Some(hash) = self.hash_storage.take() {
                            let len = core::cmp::min(num.len(), P_LEN - index);
                            num[..len].copy_from_slice(&hash[index..index + len]);
                            self.hash_storage.replace(hash);

                            if len + index < P_LEN {
                                self.input_counter.set(len + index);
                            } else {
                                self.input_counter.set(0);
                            }
                        }
                    } else {
                        let counter = self.read_var_to_buf(&self.k_val, index, num);
                        if counter + index < P_LEN {
                            self.input_counter.set(counter + index);
                        } else {
                            self.input_counter.set(0);
                        }
                    }
                }
            },
            _ => {}
        }
    }

    fn write_number(&self, num: &[u8]) -> Result<(), ErrorCode> {
        match self.state.get() {
            State::ModHash => {
                let index = self.output_counter.get();
                let end_index = (index + num.len()).min(P_LEN);
                if let Some(hash) = self.hash_storage.take() {
                    hash[index..end_index].copy_from_slice(num);
                    self.hash_storage.replace(hash);
                }
                self.output_counter.set(end_index);
            }
            State::MathModR => {
                let index = self.output_counter.get();
                let counter = self.update_var_from_buf(&self.r_val, index, num);
                self.output_counter.set(index + counter);
            }
            State::MathRDaMul | State::MathResHAdd | State::MathResKDiv => {
                let index = self.output_counter.get();
                let counter = self.update_var_from_buf(&self.s_val, index, num);
                self.output_counter.set(index + counter);
            }
            _ => {}
        }
        Ok(())
    }

    fn computation_completed(&self, result: Result<(), ErrorCode>) {
        if result.is_err() {
            self.math.take();
            self.complete_signature(result);
            return;
        }

        match self.state.get() {
            State::ModHash => {
                self.math.take();
                self.state.set(State::HmacDerivingK1);
                self.input_counter.set(0);
                self.output_counter.set(0);
                self.key_counter.set(0);
                if let Some(handle) = self.hmac_handle.get() {
                    let _ = self.hmac_mutex.request(handle);
                }
            }
            State::MathModR => {
                self.state.set(State::MathRDaMul);
                self.current_operand.set(Operand::First);
                self.math
                    .map(|math| math.start_computation(P_LEN, OpMultiplication::multiplication()));
            }
            State::MathRDaMul => {
                self.state.set(State::MathResHAdd);
                self.current_operand.set(Operand::First);
                self.math
                    .map(|math| math.start_computation(P_LEN, OpAddition::addition()));
            }
            State::MathResHAdd => {
                self.state.set(State::MathResKDiv);
                self.current_operand.set(Operand::First);
                self.math
                    .map(|math| math.start_computation(P_LEN, OpDivision::division()));
            }
            State::MathResKDiv => {
                self.math.take();
                if !self.check_s() {
                    // s == 0: reject this nonce candidate and re-enter the
                    // RFC 6979 retry loop, same as the r == 0 case in
                    // `EccClient::operation_done` above. See the
                    // `State::HmacGetNewK` arm of `ready()` for the mutex
                    // hand-back this depends on.
                    self.state.set(State::HmacGetNewK);
                    if let Some(handle) = self.hmac_handle.get() {
                        let _ = self.hmac_mutex.request(handle);
                    }
                } else {
                    self.complete_signature(result);
                }
            }
            _ => {
                self.math.take();
                self.complete_signature(Err(ErrorCode::FAIL));
            }
        }
    }

    fn read_second_number(&self, num: &mut [u8]) {
        todo!()
    }
}

impl<'a, E, Op, M, H> hil::public_key_crypto::keys::SetKeyBySlice<'a, 32>
    for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn set_key(
        &self,
        key: &'static mut [u8; 32],
    ) -> Result<(), (ErrorCode, &'static mut [u8; 32])> {
        if !matches!(self.state.get(), State::Idle) {
            return Err((ErrorCode::BUSY, key));
        }
        self.state.set(State::ChangingKey);
        self.new_key_buffer.replace(key);
        self.deferred_call.set();
        Ok(())
    }

    fn set_client(&self, client: &'a dyn SetKeyBySliceClient<32>) {
        self.client_key_set.replace(client);
    }
}

impl<'a, E, Op, M, H> kernel::deferred_call::DeferredCallClient
    for EcdsaP256SignatureSigner<'a, E, Op, M, H>
where
    E: EccCrypto<'a, 32, NistP256Constants>,
    Op: OpAddition + OpMultiplication + OpDivision + OpModulo,
    M: ModularArithmetic<'a, Op>,
    H: Hmac,
{
    fn handle_deferred_call(&self) {
        match self.state.get() {
            State::ChangingKey => {
                if let Some(key) = self.new_key_buffer.take() {
                    if let Some(skey) = self.signing_key.take() {
                        skey.copy_from_slice(key);
                        self.signing_key.replace(skey);
                    }
                    if let Some(client) = self.client_key_set.get() {
                        client.set_key_done(key, Ok(()));
                    }
                }
                self.state.set(State::Idle);
            }
            _ => {}
        }
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
