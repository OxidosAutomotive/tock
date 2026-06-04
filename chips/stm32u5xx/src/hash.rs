// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright OxidOS Automotive 2026.

use core::cell::Cell;
use core::cmp::min;
use core::ops::Index;

use crate::dma::{ChannelId, Dma};

use cortexm33::dma_fence::CortexMDmaFence;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::digest::{
    Bit16Data, Bit1Data, Bit32Data, Bit8Data, Client, Digest, DigestData, DigestHash, DigestVerify,
    HmacMd5, HmacSha1, HmacSha224, HmacSha256, HmacSha384, HmacSha512, Md5, Sha1, Sha224, Sha256,
    Sha384, Sha512,
};
use kernel::utilities::cells::{MapCell, OptionalCell};
use kernel::utilities::dma_slice::{DmaSubSlice, DmaSubSliceMut, DmaSubSliceMutImmut};
use kernel::utilities::leasable_buffer::SubSliceMutImmut;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use kernel::debug;

register_structs! {
    /// Hash processor
    pub HashRegisters {
        /// control register
        (0x000 => cr: ReadWrite<u32, CR::Register>),
        /// data input register
        (0x004 => din: WriteOnly<u32>),
        /// start register
        (0x008 => str: ReadWrite<u32, STR::Register>),
        /// HASH aliased digest register 0
        /// 0x00C - 0x01C
        (0x00C => hra: [ReadOnly<u32>; 5]),
        /// interrupt enable register
        (0x020 => imr: ReadWrite<u32, IMR::Register>),
        /// status register
        (0x024 => sr: ReadWrite<u32, SR::Register>),
        (0x028 => _reserved0),
        /// context swap registers
        /// 0x0F8 - 0x1CC
        (0x0F8 => csr: [ReadWrite<u32>; 54]),
        (0x1D0 => _reserved1),
        /// digest register 0
        /// 0x310 - 0x32C
        (0x310 => hr: [ReadOnly<u32>; 8]),
        (0x330 => @END),
    }
}
register_bitfields![u32,
CR [
    /// Initialize message digest calculation
    INIT OFFSET(2) NUMBITS(1) [],
    /// DMA enable
    DMAE OFFSET(3) NUMBITS(1) [],
    /// Data type selection
    DATATYPE OFFSET(4) NUMBITS(2) [
        /// The data written into HASH_DIN are directly used by the HASH processing,
        /// without reordering.
        _32bitData = 0,
        /// Half-word. The data written into HASH_DIN are considered as two half-
        /// words, and are swapped before being used by the HASH processing.
        _16bitData = 1,
        /// Bytes. The data written into HASH_DIN are considered as four bytes, and
        /// are swapped before being used by the HASH processing.
        _8bitData = 2,
        /// Bit-string. The data written into HASH_DIN are considered as 32 bits (1st bit of
        /// the string at position 0), and are swapped before being used by the HASH processing (1st bit
        /// of the string at position 31).
        _1bitData = 3,
    ],
    /// Mode selection
    MODE OFFSET(6) NUMBITS(1) [],
    /// Algorithm selection
    ALGO OFFSET(17) NUMBITS(2) [
        /// SHA-1
        SHA_1 = 0,
        /// MD5
        MD5 = 1,
        /// SHA2-224
        SHA2_224 = 2,
        /// SHA2-256
        SHA2_256 = 3,
    ],
    /// Number of words already pushed
    NBW OFFSET(8) NUMBITS(4) [],
    /// DIN not empty
    DINNE OFFSET(12) NUMBITS(1) [],
    /// Multiple DMA Transfers
    MDMAT OFFSET(13) NUMBITS(1) [],
    /// Long key selection
    LKEY OFFSET(16) NUMBITS(1) []
],
DIN [
    /// Data input
    DATAIN OFFSET(0) NUMBITS(32) []
],
STR [
    /// Digest calculation
    DCAL OFFSET(8) NUMBITS(1) [],
    /// Number of valid bits in the last word of the message
    NBLW OFFSET(0) NUMBITS(5) []
],
HRA [
    /// H0
    H OFFSET(0) NUMBITS(32) []
],
HR [
    /// H0
    H OFFSET(0) NUMBITS(32) []
],
IMR [
    /// Digest calculation completion interrupt enable
    DCIE OFFSET(1) NUMBITS(1) [],
    /// Data input interrupt enable
    DINIE OFFSET(0) NUMBITS(1) []
],
SR [
    /// Busy bit
    BUSY OFFSET(3) NUMBITS(1) [],
    /// DMA Status
    DMAS OFFSET(2) NUMBITS(1) [],
    /// Digest calculation completion interrupt status
    DCIS OFFSET(1) NUMBITS(1) [],
    /// Data input interrupt status
    DINIS OFFSET(0) NUMBITS(1) [],
    /// Number of words expected
    NBWE OFFSET(16) NUMBITS(5) [],
    /// DIN not empty
    DINNE OFFSET(15) NUMBITS(1) [],
    /// Number of words already pushed
    NBWP OFFSET(9) NUMBITS(5) []
],
CSR [
    /// CS0
    CS OFFSET(0) NUMBITS(32) []
]
];
pub const HASH_BASE: StaticRef<HashRegisters> =
    unsafe { StaticRef::new(0x420C0400 as *const HashRegisters) };

// SHA2-256 has the largest digest.
const MAX_DIGEST_LEN: usize = 32;
const LONG_HMAC_KEY_LEN: usize = 64;
// It is an artificial limitation for compatibility with capsules.
// Taken from boards::components::hmac
const MAX_HMAC_KEY_LEN: usize = 512 / 8;

#[derive(Clone, Copy)]
enum Mode {
    MD5,
    SHA1,
    SHA2_224,
    SHA2_256,
}

impl Mode {
    fn get_digest_len(&self) -> usize {
        match self {
            Mode::MD5 => 4,
            Mode::SHA1 => 5,
            Mode::SHA2_224 => 7,
            Mode::SHA2_256 => 8,
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum State {
    Add,
    Run,
    HmacInit,
    HmacPreAuth,
    HmacPostAuth,
    HmacFinalize,
}

pub struct Leftover {
    buffer: Cell<Option<u32>>,
    // it is used if the FIFO is full and we cannot write the leftover we collected.
    // unfortunately, it is not possible to extend the subslice, I could have written the leftover on top of it.
    secondary_buffer: Cell<Option<u32>>,
    index: Cell<usize>,
}

impl Leftover {
    pub fn new() -> Self {
        Leftover {
            buffer: Cell::new(None),
            secondary_buffer: Cell::new(None),
            index: Cell::new(0),
        }
    }

    fn add_byte(buf: Option<u32>, byte: u8) -> Option<u32> {
        match buf {
            Some(b) => Some(b >> 8 | (byte as u32).rotate_right(8)),
            None => Some((byte as u32).rotate_right(8)),
        }
    }

    pub fn add(&self, byte: u8) {
        if !self.is_full() {
            self.buffer.update(|buf| Leftover::add_byte(buf, byte));
        } else {
            self.secondary_buffer
                .update(|buf| Leftover::add_byte(buf, byte));
        }

        self.index.update(|index| (index + 1) % 8);
    }

    pub fn to_le(&self) -> u32 {
        let res = match self.buffer.take() {
            Some(b) => {
                let value = b >> (8 * self.bytes_left());
                self.index.update(|idx| idx.saturating_sub(4));
                if self.secondary_buffer.get().is_some() {
                    self.buffer.set(self.secondary_buffer.take());
                }
                value
            }
            None => 0,
        };

        res
    }

    pub fn bytes_left(&self) -> usize {
        let rem = self.index.get() % 4;
        if rem == 0 {
            0
        } else {
            4 - rem
        }
    }

    pub fn is_full(&self) -> bool {
        self.index.get() >= 4 && self.buffer.get().is_some()
    }

    pub fn is_empty(&self) -> bool {
        self.buffer.get().is_none()
    }
}

pub struct HmacKey {
    key: MapCell<[u8; MAX_HMAC_KEY_LEN]>,
    index: Cell<usize>,
    len: Cell<usize>,
}

impl HmacKey {
    pub fn new() -> Self {
        Self {
            key: MapCell::empty(),
            index: Cell::new(0),
            len: Cell::new(0),
        }
    }

    pub fn set(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.key.is_none() {
            self.key.put([0u8; MAX_HMAC_KEY_LEN]);
        }
        match self.key.map(|buf| {
            if buf.len() >= key.len() {
                buf[..key.len()].copy_from_slice(key);
                self.len.set(key.len());
                Ok(())
            } else {
                Err(ErrorCode::SIZE)
            }
        }) {
            // Ok(Some(_)) => Ok(()),
            // Ok(None) => Err(kernel::ErrorCode::NOMEM),
            // Err(e) => Err(e)
            Some(r) => r,
            None => Err(ErrorCode::NOMEM),
        }
    }

    pub fn is_stored(&self) -> bool {
        self.key.is_some()
    }

    pub fn len(&self) -> usize {
        self.len.get()
    }

    pub fn is_loaded(&self) -> bool {
        self.index.get() == self.len.get()
    }

    pub fn left_to_load(&self) -> usize {
        self.len.get().saturating_sub(self.index.get())
    }

    pub fn reset(&self) {
        if self.key.is_some() {
            self.index.take();
        }
    }

    pub fn clear(&self) {
        self.key.take();
        self.len.take();
        self.index.take();
    }
}

pub struct Hash<'a> {
    regs: StaticRef<HashRegisters>,
    dma: OptionalCell<&'a Dma>,
    dma_channel: Cell<Option<ChannelId>>,
    dma_buffer: MapCell<DmaSubSliceMutImmut<'static, u8>>,
    mode: Cell<Option<Mode>>,
    state: Cell<Option<State>>,
    hmac_key: HmacKey,
    data: Cell<Option<SubSliceMutImmut<'static, u8>>>,
    leftover: Leftover,
    verify: Cell<bool>,
    cancelled: Cell<bool>,
    // How to be more flexible in this field?
    // THIS IS CRITICAL
    client: OptionalCell<&'a dyn Client<MAX_DIGEST_LEN>>,
    digest: OptionalCell<&'static mut [u8; MAX_DIGEST_LEN]>, // Maximum length that can be used
    deferred_call: DeferredCall,
}

impl<'a> Hash<'a> {
    // Associates a DMA controller and channels with the HASH driver
    pub fn set_dma(hash: &'static Self, dma: &'a Dma, channel: ChannelId) {
        hash.dma.set(dma);
        hash.dma_channel.set(Some(channel));
        dma.set_client(channel, hash);
    }

    // TODO(frihetselsker): Think about the return type
    fn start_dma_transfer(&self, dma: &'a Dma) -> Result<(), ()> {
        if let Some(mut data) = self.data.take() {
            let mut can_start = true;
            if !self.leftover.is_empty() {
                let (count, start) = self.trim_subslice(&data, data.len());
                data.slice(count..);
                if let Some(s) = start {
                    can_start = s;
                }
            }
            // Truncate
            let count = self.truncate_subslice(&data, data.len());
            data.slice(..data.len() - count);
            if data.len() == 0 {
                if can_start {
                    self.deferred_call.set();
                }
                return Ok(());
            }

            // Trigger HASH
            if let Some(ch) = self.dma_channel.get() {
                let regs = self.regs;
                // Hardware fence
                // Load data only if we have a channel
                // Otherwise, it is meaningless
                let fence = unsafe { CortexMDmaFence::new() };
                // Convert subslice into DmaSlice
                let (dma_slice, ptr, len) = match data {
                    SubSliceMutImmut::Immutable(d) => {
                        let dma_slice = DmaSubSlice::new(d, fence);
                        // Extract the ph()ysical pointer and length for MMIO
                        let ptr = dma_slice.as_ptr() as u32;
                        let len = dma_slice.len() as u32;
                        (DmaSubSliceMutImmut::Immutable(dma_slice), ptr, len)
                    }
                    SubSliceMutImmut::Mutable(d) => {
                        let dma_slice = unsafe { DmaSubSliceMut::new(d, fence) };
                        // Extract the physical pointer and length for MMIO
                        let ptr = dma_slice.as_mut_ptr() as u32;
                        let len = dma_slice.len() as u32;
                        (DmaSubSliceMutImmut::Mutable(dma_slice), ptr, len)
                    }
                };
                // Save DmaSlice in the struct
                self.dma_buffer.replace(dma_slice);
                dma.setup(ch, crate::dma::DmaPeripheral::Hash, ptr, len);

                regs.imr.modify(IMR::DINIE::SET);

                if can_start {
                    regs.cr.modify(CR::DMAE::SET);
                }

                Ok(())
            } else {
                Err(())
            }
        } else {
            // No data found
            Err(())
        }
    }
}

impl Hash<'_> {
    pub fn new(base: StaticRef<HashRegisters>) -> Self {
        Self {
            regs: base,
            dma: OptionalCell::empty(),
            dma_channel: Cell::new(None),
            dma_buffer: MapCell::empty(),
            mode: Cell::new(None),
            state: Cell::new(None),
            data: Cell::new(None),
            hmac_key: HmacKey::new(),
            verify: Cell::new(false),
            cancelled: Cell::new(false),
            leftover: Leftover::new(),
            digest: OptionalCell::empty(),
            client: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
        }
    }

    pub fn handle_interupts(&self) {
        let regs = self.regs;

        // Disable all the interrupts
        regs.imr.modify(IMR::DCIE::CLEAR + IMR::DINIE::CLEAR);

        if let Some(state) = self.state.get() {
            // Digest Calculation completed?
            if regs.sr.read(SR::DCIS) != 0 {
                // If key is present, then we need to process it accordingly
                match state {
                    State::HmacFinalize | State::Run => {
                        // it is time to return data
                        self.return_data();
                    }
                    _ => (),
                }
            } else if regs.sr.read(SR::DINIS) != 0 {
                match (state, self.cancelled.get()) {
                    (State::Add | State::HmacInit | State::HmacPreAuth, true) => {
                        self.cancelled.set(false);
                        regs.cr.modify(CR::INIT::SET);
                        self.client.map(|client| {
                            self.data.take().map(|buf| match buf {
                                SubSliceMutImmut::Immutable(mut b) => {
                                    b.reset();
                                    client.add_data_done(Err(kernel::ErrorCode::CANCEL), b)
                                }
                                SubSliceMutImmut::Mutable(mut b) => {
                                    b.reset();
                                    client.add_mut_data_done(Err(kernel::ErrorCode::CANCEL), b)
                                }
                            })
                        });
                    }
                    (State::Run | State::HmacPostAuth | State::HmacFinalize, true) => {
                        self.cancelled.set(false);
                        regs.cr.modify(CR::INIT::SET);
                        self.client.map(|client| {
                            if let Some(digest) = self.digest.take() {
                                if self.verify.get() {
                                    client.verification_done(Err(kernel::ErrorCode::CANCEL), digest)
                                } else {
                                    client.hash_done(Err(kernel::ErrorCode::CANCEL), digest);
                                }
                            }
                        });
                    }
                    (State::Add, false) => {
                        if self.dma.is_some() {
                            // Theoretically, there can be a situation
                            // when the FIFO is already filled, but we do need to write leftovers
                            // The DMA transfer cannot be used until all the previous data is loaded
                            // to the peripheral so that calculated hash is not corrupted.
                            if self.leftover.is_full() {
                                regs.din.set(self.leftover.to_le());
                                // It is the appropriate moment to start the DMA transfer
                                regs.cr.modify(CR::DMAE::SET);
                            }
                        } else {
                            if !self.data_progress() {
                                self.deferred_call.set();
                            } else {
                                regs.imr.modify(IMR::DINIE::SET);
                            }
                        }
                    }

                    (State::HmacInit | State::HmacPostAuth, false) => {
                        self.load_key();
                    }
                    (State::Run, false) => {
                        self.state.set(Some(State::HmacPostAuth));
                        self.load_key();
                    }
                    (State::HmacPreAuth, false) => {
                        self.state.set(Some(State::Add));
                        // self.hmac_key.is_loaded.set(Some(true));

                        if let Some(dma) = self.dma.get() {
                            // TODO(frihetselsker): handle result gracefully
                            let _ = self.start_dma_transfer(dma);
                        } else {
                            if !self.data_progress() {
                                // we added all the data
                                self.deferred_call.set();
                            } else {
                                regs.imr.modify(IMR::DINIE::SET);
                            }
                        }
                    }
                    _ => (),
                }
            }
        }
    }

    pub fn handle_dma_interrupt(&self) {
        // Disable the DMA trigger to release the channel
        let regs = self.regs;
        regs.cr.modify(CR::DMAE::CLEAR);
        if let Some(dma_slice) = self.dma_buffer.take() {
            if let Some(State::Add) = self.state.get() {
                self.state.take();
                match dma_slice {
                    DmaSubSliceMutImmut::Immutable(b) => {
                        let mut subslice = b.as_sub_slice();
                        // ugly line of code
                        subslice.slice(0..0);
                        self.client.map(|client| {
                            client.add_data_done(Ok(()), subslice);
                        });
                    }
                    DmaSubSliceMutImmut::Mutable(b) => {
                        let fence = unsafe { CortexMDmaFence::new() };
                        let mut subslice = unsafe { b.take(fence) };
                        // ugly line of code
                        subslice.slice(0..0);
                        self.client.map(|client| {
                            client.add_mut_data_done(Ok(()), subslice);
                        });
                    }
                }
            }
        }
    }

    fn return_data(&self) {
        self.client.map(|client| {
            let regs = self.regs;
            // TODO: fix this disaster
            // NO UNWRAPS
            let digest = self.digest.take().unwrap();
            // We need to compare the result with the digest received before.
            // If there is no operation, we react in no manner.
            if let Some(mode) = self.mode.get() {
                if self.verify.get() {
                    let mut equal = true;
                    for i in 0..mode.get_digest_len() {
                        let d = regs.hr[i].get().to_be_bytes();

                        debug!(
                            "SHA: Check {} - Data: 0x{:02x}{:02x}{:02x}{:02x}",
                            i, d[0], d[1], d[2], d[3]
                        );

                        let idx = i * 4;

                        if digest[idx + 0] != d[0]
                            || digest[idx + 1] != d[1]
                            || digest[idx + 2] != d[2]
                            || digest[idx + 3] != d[3]
                        {
                            equal = false;
                        }
                    }

                    // This resets the peripheral
                    regs.cr.modify(CR::INIT::SET);
                    self.state.take();
                    if self.hmac_key.is_stored() {
                        self.hmac_key.reset();
                    }
                    self.verify.set(false);
                    client.verification_done(Ok(equal), digest);
                } else {
                    for i in 0..mode.get_digest_len() {
                        let d = regs.hr[i].get().to_be_bytes();

                        let idx = i * 4;

                        debug!(
                            "SHA: Check {} - Data: 0x{:02x}{:02x}{:02x}{:02x}",
                            i, d[0], d[1], d[2], d[3]
                        );

                        digest[idx + 0] = d[0];
                        digest[idx + 1] = d[1];
                        digest[idx + 2] = d[2];
                        digest[idx + 3] = d[3];
                    }
                    // pad digest if needed
                    // TODO: make it better in the sense we don't need to store client with
                    digest[(mode.get_digest_len() * 4)..MAX_DIGEST_LEN].fill(0);

                    regs.cr.modify(CR::INIT::SET);
                    // release the peripheral
                    self.state.take();
                    if self.hmac_key.is_stored() {
                        self.hmac_key.reset();
                    }
                    client.hash_done(Ok(()), digest);
                }
            }
        });
    }

    fn process(&self, data: &dyn Index<usize, Output = u8>, count: usize) -> usize {
        let regs = self.regs;
        let words_num = count / 4;
        for i in 0..words_num {
            if regs.sr.read(SR::NBWE) == 0 {
                // 'buffer is full', peripheral says
                return i * 4;
            }
            let data_idx = i * 4;
            let d = u32::from_le_bytes([
                data[data_idx + 0],
                data[data_idx + 1],
                data[data_idx + 2],
                data[data_idx + 3],
            ]);
            regs.din.set(d);
        }

        if !count.is_multiple_of(4) {
            for i in 0..(count % 4) {
                if regs.sr.read(SR::NBWE) == 0 {
                    // 'buffer is full', peripheral says
                    return i + words_num;
                }
                let data_idx = (count - (count % 4)) + i;
                // d |= (data[data_idx] as u32) << (8 * i);
                self.leftover.add(data[data_idx]);
                // We can add checks here if it is full and constantly poll
                // But will it save clock cycles -> No!
            }
            //
            if self.leftover.is_full() {
                regs.din.set(self.leftover.to_le());
            }
        }

        count
    }

    fn trim_subslice(
        &self,
        data: &dyn Index<usize, Output = u8>,
        count: usize,
    ) -> (usize, Option<bool>) {
        let bytes_written = min(self.leftover.bytes_left(), count % 4);
        let mut is_write_successful = None;
        for data_idx in 0..bytes_written {
            self.leftover.add(data[data_idx]);
        }
        if self.leftover.is_full() {
            let regs = self.regs;
            if regs.sr.read(SR::NBWE) == 0 {
                // we cannot write new data for now
                is_write_successful = Some(false);
            } else {
                regs.din.set(self.leftover.to_le());
                // Leftover was written successfully
                is_write_successful = Some(true);
            }
        }
        (bytes_written, is_write_successful)
    }

    fn truncate_subslice(&self, data: &dyn Index<usize, Output = u8>, count: usize) -> usize {
        let bytes_written = count % 4;
        for i in 0..bytes_written {
            let data_idx = (count - bytes_written) + i;
            // d |= (data[data_idx] as u32) << (8 * i);
            self.leftover.add(data[data_idx]);
        }
        bytes_written
    }

    // Return true if processing more data, false if the buffer
    // is completely processed.
    fn data_progress(&self) -> bool {
        self.data.take().is_some_and(|buf| match buf {
            SubSliceMutImmut::Immutable(mut b) => {
                if b.len() == 0 {
                    self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                    false
                } else {
                    if !self.leftover.is_empty() {
                        let (count, is_write_successful) = self.trim_subslice(&b, b.len());
                        b.slice(count..);
                        if let Some(false) = is_write_successful {
                            self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                            return true;
                        }
                    }
                    let count = self.process(&b, b.len());
                    b.slice(count..);
                    if b.len() == 0 {
                        // Finish
                        self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                        false
                    } else {
                        self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                        true
                    }
                }
            }
            SubSliceMutImmut::Mutable(mut b) => {
                if b.len() == 0 {
                    self.data.set(Some(SubSliceMutImmut::Mutable(b)));
                    false
                } else {
                    if !self.leftover.is_empty() {
                        let (count, is_write_successful) = self.trim_subslice(&b, b.len());
                        b.slice(count..);
                        if let Some(false) = is_write_successful {
                            self.data.set(Some(SubSliceMutImmut::Mutable(b)));
                            return true;
                        }
                    }
                    let count = self.process(&b, b.len());
                    b.slice(count..);
                    if b.len() == 0 {
                        self.data.set(Some(SubSliceMutImmut::Mutable(b)));
                        false
                    } else {
                        self.data.set(Some(SubSliceMutImmut::Mutable(b)));
                        true
                    }
                }
            }
        })
    }

    fn load_key(&self) {
        let regs = self.regs;

        self.hmac_key.key.map(|buf| {
            let count = self.process(buf, self.hmac_key.left_to_load());
            self.hmac_key.index.update(|idx| idx + count);
        });

        // Time to compute digest on it.

        if self.hmac_key.left_to_load() == 0 {
            regs.imr.modify(IMR::DINIE::SET);
            if !self.leftover.is_empty() {
                self.flush_leftover();
            }

            self.state.update(|s| match s {
                Some(State::HmacInit) => Some(State::HmacPreAuth),
                Some(State::HmacPostAuth) => Some(State::HmacFinalize),
                _ => s,
            });
            // start the final digest calculation
            regs.str.modify(STR::DCAL::SET);
        } else {
            // We need to process more
            // self.state.update(|_| {
            //     if is_inner {
            //         Some(State::HmacInit)
            //     } else {
            //         Some(State::HmacPostAuth)
            //     }
            // });
            regs.imr.modify(IMR::DINIE::SET);
        }
    }

    fn flush_leftover(&self) {
        let regs = self.regs;
        regs.str
            .modify(STR::NBLW.val(((4 - self.leftover.bytes_left()) * 8) as u32));
        regs.din.set(self.leftover.to_le());
    }
}

impl<'a> DigestHash<'a, 32> for Hash<'a> {
    fn set_hash_client(&'a self, _client: &'a dyn kernel::hil::digest::ClientHash<32>) {
        unimplemented!();
    }

    fn run(
        &'a self,
        digest: &'static mut [u8; 32],
    ) -> Result<(), (kernel::ErrorCode, &'static mut [u8; 32])> {
        if self.state.get().is_some() {
            return Err((ErrorCode::BUSY, digest));
        }
        // we cannot make any computations without setting a mode.
        if self.mode.get().is_none() {
            return Err((ErrorCode::INVAL, digest));
        }
        let regs = self.regs;
        // set the padding
        // assume that we write bytes, not bit by bit
        if !self.leftover.is_empty() {
            self.flush_leftover();
        } else {
            regs.str.modify(STR::NBLW.val(0));
        }
        // enable the interrupt
        if self.hmac_key.is_stored() {
            self.hmac_key.reset();
            regs.imr.modify(IMR::DINIE::SET + IMR::DCIE::SET);
        } else {
            regs.imr.modify(IMR::DCIE::SET);
        }

        // start the final digest calculation
        regs.str.modify(STR::DCAL::SET);
        self.state.set(Some(State::Run));
        self.digest.set(digest);

        Ok(())
    }
}

impl crate::dma::DmaClient for Hash<'_> {
    fn transfer_done(&self, channel: ChannelId) {
        if let Some(ch) = self.dma_channel.get() {
            if ch == channel {
                self.handle_dma_interrupt();
                return;
            }
        }
    }
}

impl<'a> DigestData<'a, 32> for Hash<'a> {
    fn set_data_client(&'a self, _client: &'a dyn kernel::hil::digest::ClientData<32>) {
        unimplemented!();
    }

    fn add_data(
        &self,
        data: kernel::utilities::leasable_buffer::SubSlice<'static, u8>,
    ) -> Result<
        (),
        (
            kernel::ErrorCode,
            kernel::utilities::leasable_buffer::SubSlice<'static, u8>,
        ),
    > {
        if self.state.get().is_some() {
            return Err((ErrorCode::BUSY, data));
        }
        if self.mode.get().is_none() {
            return Err((ErrorCode::INVAL, data));
        }
        if data.len() == 0 {
            return Err((ErrorCode::SIZE, data));
        }

        // If DMA is available, then do use it.
        if data.len() > self.regs.sr.read(SR::NBWE) as usize
            || (self.hmac_key.left_to_load() > self.regs.sr.read(SR::NBWE) as usize
                && !self.hmac_key.is_loaded())
        {
            self.regs.imr.modify(IMR::DINIE::SET);
        }

        self.data.set(Some(SubSliceMutImmut::Immutable(data)));

        // If we have key and it is not loaded yet, do load it now
        if self.hmac_key.is_stored() && self.hmac_key.left_to_load() > 0 {
            self.state.set(Some(State::HmacInit));
            self.load_key();
        } else {
            // debug!("Set ADD state");
            self.state.set(Some(State::Add));

            if let Some(dma) = self.dma.get() {
                // Move buffer to DMA buffer
                // DMA transfer can send only 32-bit words
                // NOTE: use iter() and chain()
                // If we still have leftovers, take bytes from the end and put them to the leftover buffer
                // NEW IDEA: just append bytes on top of what you got, then take a slice which can be divided by 4, add it to the DMA buffer and pass the leftovers back
                // The subslice has to be discarded

                // Trim from the beginning
                // We must ensure that DMA receives size divisible by 4 (only 32-bit words)
                match self.start_dma_transfer(dma) {
                    Ok(()) => return Ok(()),
                    Err(()) => {
                        if let Some(SubSliceMutImmut::Immutable(data)) = self.data.take() {
                            // Should I change it?
                            return Err((ErrorCode::FAIL, data));
                        }
                    }
                }
            } else {
                // Otherwise, act as usual
                if self.data_progress() {
                    self.regs.imr.modify(IMR::DINIE::SET);
                } else {
                    self.deferred_call.set();
                }
            }
        }

        Ok(())
    }

    fn add_mut_data(
        &self,
        data: kernel::utilities::leasable_buffer::SubSliceMut<'static, u8>,
    ) -> Result<
        (),
        (
            kernel::ErrorCode,
            kernel::utilities::leasable_buffer::SubSliceMut<'static, u8>,
        ),
    > {
        if self.state.get().is_some() {
            return Err((ErrorCode::BUSY, data));
        }
        if self.mode.get().is_none() {
            return Err((ErrorCode::INVAL, data));
        }
        if data.len() == 0 {
            return Err((ErrorCode::SIZE, data));
        }

        // If DMA is available, then do use it.
        if data.len() > self.regs.sr.read(SR::NBWE) as usize
            || (self.hmac_key.left_to_load() > self.regs.sr.read(SR::NBWE) as usize
                && !self.hmac_key.is_loaded())
        {
            self.regs.imr.modify(IMR::DINIE::SET);
        }

        self.data.set(Some(SubSliceMutImmut::Mutable(data)));

        // If we have key and it is not loaded yet, do load it now
        if self.hmac_key.is_stored() && self.hmac_key.left_to_load() > 0 {
            self.state.set(Some(State::HmacInit));
            self.load_key();
        } else {
            self.state.set(Some(State::Add));

            if let Some(dma) = self.dma.get() {
                // Move buffer to DMA buffer
                // DMA transfer can send only 32-bit words
                // NOTE: use iter() and chain()
                // If we still have leftovers, take bytes from the end and put them to the leftover buffer
                // NEW IDEA: just append bytes on top of what you got, then take a slice which can be divided by 4, add it to the DMA buffer and pass the leftovers back
                // The subslice has to be discarded

                // Turn u8 to u32
                // Prepare the buffer with slicing
                // Trim from the beginning
                // We must ensure that DMA receives size divisible by 4 (only 32-bit words)
                match self.start_dma_transfer(dma) {
                    Ok(()) => return Ok(()),
                    Err(()) => {
                        if let Some(SubSliceMutImmut::Mutable(data)) = self.data.take() {
                            // Should I change it?
                            return Err((ErrorCode::FAIL, data));
                        }
                    }
                }
            } else {
                // Otherwise, act as usual
                if self.data_progress() {
                    self.regs.imr.modify(IMR::DINIE::SET);
                } else {
                    self.deferred_call.set();
                }
            }
        }

        Ok(())
    }

    fn clear_data(&self) {
        if self.state.get().is_none() {
            self.regs.cr.modify(CR::INIT::SET);
        } else {
            self.cancelled.set(true);
        }
    }
}

impl<'a> DigestVerify<'a, 32> for Hash<'a> {
    fn set_verify_client(&'a self, _client: &'a dyn kernel::hil::digest::ClientVerify<32>) {
        unimplemented!();
    }

    fn verify(
        &'a self,
        compare: &'static mut [u8; 32],
    ) -> Result<(), (kernel::ErrorCode, &'static mut [u8; 32])> {
        self.verify.set(true);
        self.run(compare)
    }
}

impl<'a> Digest<'a, 32> for Hash<'a> {
    fn set_client(&'a self, client: &'a dyn Client<32>) {
        self.client.set(client);
    }
}

impl Md5 for Hash<'_> {
    fn set_mode_md5(&self) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::MD5));
        self.regs.cr.modify(
            CR::ALGO::MD5
                + CR::MODE::CLEAR
                + CR::MDMAT::SET
                + CR::DATATYPE::_8bitData
                + CR::INIT::SET,
        );
        self.hmac_key.clear();
        Ok(())
    }
}

impl Sha1 for Hash<'_> {
    fn set_mode_sha1(&self) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA1));
        self.regs.cr.modify(
            CR::ALGO::SHA_1
                + CR::MODE::CLEAR
                + CR::MDMAT::SET
                + CR::DATATYPE::_8bitData
                + CR::INIT::SET,
        );
        Ok(())
    }
}

impl Sha224 for Hash<'_> {
    fn set_mode_sha224(&self) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_224));
        self.hmac_key.clear();
        self.regs.cr.modify(
            CR::ALGO::SHA2_224
                + CR::MODE::CLEAR
                + CR::MDMAT::SET
                + CR::DATATYPE::_8bitData
                + CR::INIT::SET,
        );
        Ok(())
    }
}

impl Sha256 for Hash<'_> {
    fn set_mode_sha256(&self) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_256));
        self.hmac_key.clear();
        self.regs.cr.modify(
            CR::ALGO::SHA2_256
                + CR::MODE::CLEAR
                + CR::MDMAT::SET
                + CR::DATATYPE::_8bitData
                + CR::INIT::SET,
        );
        Ok(())
    }
}

impl HmacSha256 for Hash<'_> {
    fn set_mode_hmacsha256(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        let regs = self.regs;
        self.mode.set(Some(Mode::SHA2_256));
        self.hmac_key.set(key)?;
        regs.cr
            .modify(CR::ALGO::SHA2_256 + CR::MDMAT::SET + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > LONG_HMAC_KEY_LEN {
            regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        Ok(())
    }
}

impl Sha384 for Hash<'_> {
    fn set_mode_sha384(&self) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl Sha512 for Hash<'_> {
    fn set_mode_sha512(&self) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl HmacMd5 for Hash<'_> {
    fn set_mode_hmacmd5(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::MD5));
        self.hmac_key.set(key)?;
        self.regs
            .cr
            .modify(CR::ALGO::MD5 + CR::MDMAT::SET + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > LONG_HMAC_KEY_LEN {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }
        Ok(())
    }
}

impl HmacSha1 for Hash<'_> {
    fn set_mode_hmacsha1(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA1));
        self.hmac_key.set(key)?;
        self.regs
            .cr
            .modify(CR::ALGO::SHA_1 + CR::MDMAT::SET + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > LONG_HMAC_KEY_LEN {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }
        Ok(())
    }
}

impl HmacSha224 for Hash<'_> {
    fn set_mode_hmacsha224(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_224));
        self.hmac_key.set(key)?;
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_224 + CR::MDMAT::SET + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > LONG_HMAC_KEY_LEN {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        Ok(())
    }
}

impl HmacSha384 for Hash<'_> {
    fn set_mode_hmacsha384(&self, _key: &[u8]) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl HmacSha512 for Hash<'_> {
    fn set_mode_hmacsha512(&self, _key: &[u8]) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl Bit32Data for Hash<'_> {
    fn set_data_type_32_bit(&self) -> Result<(), ErrorCode> {
        self.regs.cr.modify(CR::DATATYPE::_32bitData);
        Ok(())
    }
}

impl Bit16Data for Hash<'_> {
    fn set_data_type_16_bit(&self) -> Result<(), ErrorCode> {
        self.regs.cr.modify(CR::DATATYPE::_16bitData);
        Ok(())
    }
}

impl Bit8Data for Hash<'_> {
    fn set_data_type_8_bit(&self) -> Result<(), ErrorCode> {
        self.regs.cr.modify(CR::DATATYPE::_8bitData);
        Ok(())
    }
}

impl Bit1Data for Hash<'_> {
    fn set_data_type_1_bit(&self) -> Result<(), ErrorCode> {
        self.regs.cr.modify(CR::DATATYPE::_1bitData);
        Ok(())
    }
}

impl DeferredCallClient for Hash<'_> {
    fn handle_deferred_call(&self) {
        // we call deferred call only if we processed
        // all the data in one cycle without using interrupts
        self.state.take();
        self.client.map(|client| {
            self.data.take().map(|buf| match buf {
                SubSliceMutImmut::Immutable(b) => client.add_data_done(Ok(()), b),
                SubSliceMutImmut::Mutable(b) => client.add_mut_data_done(Ok(()), b),
            })
        });
    }

    fn register(&'static self) {
        self.deferred_call.register(self);
    }
}
