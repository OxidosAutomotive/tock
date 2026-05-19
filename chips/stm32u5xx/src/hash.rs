use core::cell::Cell;
use core::ops::Index;

use crate::dma::{ChannelId, Dma, DmaPeripheral};

use cortexm33::dma_fence::CortexMDmaFence;
use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::digest::{
    Bit16Data, Bit1Data, Bit32Data, Bit8Data, Client, Digest, DigestData, DigestHash, DigestVerify,
    HmacMd5, HmacSha1, HmacSha224, HmacSha256, HmacSha384, HmacSha512, Md5, Sha1, Sha224, Sha256,
    Sha384, Sha512,
};
use kernel::utilities::cells::{MapCell, OptionalCell};
use kernel::utilities::dma_slice::{DmaSubSliceMut, DmaSubSliceMutImmut};
use kernel::utilities::leasable_buffer::{SubSliceMut, SubSliceMutImmut};
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

// In terms of 8-bit words
// const HASH_FIFO_SIZE: usize = 68;
// const DIGEST_BLOCK_SIZE: usize = 64;
// SHA2-256 has the largest digest.
const MAX_DIGEST_LEN: usize = 32;
const HASH_FIFO_SIZE: usize = 17;
const MAX_HMAC_KEY_SIZE: usize = 128;

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
    KeyLoadAdd,
    KeyDigestAdd,
    KeyLoadRun,
    KeyDigestRun,
}

pub struct Hash<'a> {
    regs: StaticRef<HashRegisters>,
    dma: OptionalCell<&'a Dma>,
    dma_channel: Cell<Option<ChannelId>>,
    dma_buffer: MapCell<DmaSubSliceMutImmut<'static, u8>>,
    mode: Cell<Option<Mode>>,
    state: Cell<Option<State>>,
    hmac_key: MapCell<[u8; MAX_HMAC_KEY_SIZE]>,
    hmac_key_index: Cell<usize>,
    hmac_key_len: Cell<usize>,
    data: Cell<Option<SubSliceMutImmut<'static, u8>>>,
    leftover_buffer: Cell<u32>,
    leftover_index: Cell<usize>,
    verify: Cell<bool>,
    // How to be more flexible in this field?
    client: OptionalCell<&'a dyn Client<MAX_DIGEST_LEN>>,
    digest: OptionalCell<&'static mut [u8; MAX_DIGEST_LEN]>, // Maximum length that can be used
    deferred_call: DeferredCall,
}

impl<'a> Hash<'a> {
    // Associates a DMA controller and channels with the USART driver.
    pub fn set_dma(hash: &'static Self, dma: &'a Dma, channel: ChannelId) {
        hash.dma.set(dma);
        hash.dma_channel.set(Some(channel));
        dma.set_client(channel, hash);
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
            hmac_key: MapCell::new([0u8; MAX_HMAC_KEY_SIZE]),
            hmac_key_index: Cell::new(0),
            hmac_key_len: Cell::new(0),
            verify: Cell::new(false),
            leftover_buffer: Cell::new(0),
            leftover_index: Cell::new(0),
            digest: OptionalCell::empty(),
            client: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
        }
    }

    pub fn handle_interupts(&self) {
        let regs = self.regs;

        if let Some(state) = self.state.get() {
            debug!("Current state is {:?}", state);
        }
        // Disable all the interrupts
        regs.imr.modify(IMR::DCIE::CLEAR + IMR::DINIE::CLEAR);

        // Digest Calculation completed?
        if regs.sr.read(SR::DCIS) != 0 {
            debug!("SHA: Entered Digest Calculation interrupt");
            if let Some(state) = self.state.get() {
                // If key is present, then we need to process it accordingly
                match state {
                    State::KeyDigestRun => {
                        // it is time to return data
                        // reset index
                        self.hmac_key_index.take();
                        self.return_data();
                    }
                    State::Run => {
                        if self.hmac_key.is_some() {
                            // We need to load a key as well
                            self.load_key(false);
                        } else {
                            self.return_data();
                        }
                    }
                    _ => (),
                }
            }
            // New data can be written (if we filled the FIFO buffer)
        } else if regs.sr.read(SR::DINIS) != 0 {
            debug!("SHA: Entered Data Input interrupt");
            if let Some(state) = self.state.get() {
                match state {
                    State::Add => {
                        if !self.data_progress() {
                            // self.client.map(|client| {
                            //     self.busy.set(false);
                            //     self.data.take().map(|buf| match buf {
                            //         SubSliceMutImmut::Immutable(b) => client.add_data_done(Ok(()), b),
                            //         SubSliceMutImmut::Mutable(b) => client.add_mut_data_done(Ok(()), b),
                            //     })
                            // });
                            // Should we use call client here directly?
                            // regs.imr.modify(IMR::DINIE::CLEAR);
                            self.deferred_call.set();
                        } else {
                            regs.imr.modify(IMR::DINIE::SET);
                        }
                    }
                    State::KeyLoadAdd => {
                        self.load_key(true);
                    }
                    State::KeyLoadRun | State::Run => {
                        self.load_key(false);
                    }
                    State::KeyDigestAdd => {
                        self.state.set(Some(State::Add));
                        self.hmac_key_index.take();
                        debug!("It is time to write the actual data");
                        if !self.data_progress() {
                            // we added all the data
                            debug!("we added all the data");
                            self.deferred_call.set();
                        } else {
                            debug!("we need more data");
                            regs.imr.modify(IMR::DINIE::SET);
                        }
                    }
                    _ => (),
                }
            }
            // clear interrupt
            // regs.sr.modify(SR::DINIS::CLEAR);

            // Small fix, I don;t like it, it should be optimized
            // But we finished!
            // if regs.cr.read(CR::DINNE) == 0 {
            //     regs.imr.modify(IMR::DINIE::CLEAR);
            //     return;
            // }

            // if false, we are done
        }
    }

    pub fn handle_dma_interrupt(&self) {
        self.dma.map(|dma| {
            if let Some(ch) = self.dma_channel.get() {
                dma.clear_interrupt(ch);
            }
        });
        // Disable the DMA trigger to release the channel
        self.regs.cr.modify(CR::DMAE::CLEAR);
        // TODO(frihetselsker): Add states handling
        if let Some(dma_slice) = self.dma_buffer.take() {
            match dma_slice {
                DmaSubSliceMutImmut::Immutable(b) => {
                    let mut subslice = b.as_sub_slice();
                    subslice.reset();

                    self.client.map(|client| {
                        client.add_data_done(Ok(()), subslice);
                    });
                }
                DmaSubSliceMutImmut::Mutable(b) => {
                    let fence = unsafe { CortexMDmaFence::new() };
                    let mut subslice = unsafe { b.take(fence) };
                    subslice.reset();
                    self.client.map(|client| {
                        client.add_mut_data_done(Ok(()), subslice);
                    });
                }
            }
        }
    }

    fn return_data(&self) {
        self.client.map(|client| {
            let regs = self.regs;
            let digest = self.digest.take().unwrap();
            //debug!("SHA: Entered interrupt related to digest calculation");
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

                    // self.clear_data();
                    // NOTE(frihetselsker): I am not sure that this clears the whole internal FIFO, but we will see.
                    regs.cr.modify(CR::INIT::SET);
                    self.state.take();
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
                    digest[(mode.get_digest_len() * 4)..MAX_DIGEST_LEN].fill(0);

                    // self.clear_data();
                    regs.cr.modify(CR::INIT::SET);
                    // release the peripheral
                    self.state.take();
                    client.hash_done(Ok(()), digest);
                }
            }
        });
    }

    fn process(&self, data: &dyn Index<usize, Output = u8>, count: usize) -> usize {
        let regs = self.regs;
        debug!("SHA: current last_index: {}", regs.sr.read(SR::NBWP));
        let words_num = count / 4;
        for i in 0..words_num {
            if regs.sr.read(SR::NBWE) == 0 {
                debug!("SHA: 'buffer is full', process says");
                return i * 4;
            }

            let data_idx = i * 4;
            // Swap is automatically made on Hash peripheral.
            // let mut d = (data[data_idx + 0] as u32) << 0;
            // d |= (data[data_idx + 1] as u32) << 8;
            // d |= (data[data_idx + 2] as u32) << 16;
            // d |= (data[data_idx + 3] as u32) << 24;

            let d = u32::from_le_bytes([
                data[data_idx + 0],
                data[data_idx + 1],
                data[data_idx + 2],
                data[data_idx + 3],
            ]);
            // debug!("SHA: big word: 0x{:02x}", d);

            debug!("SHA: 32-bit word written {:02x}", d);
            //

            regs.din.set(d);

            // debug!(
            //     "SHA: last_index after insertions: {}",
            //     regs.sr.read(SR::NBWP)
            // );
            // debug!("SHA: words left to add: {}", regs.sr.read(SR::NBWE));
        }

        if !count.is_multiple_of(4) {
            debug!("SHA: Adding 8-bit words as leftovers");

            for i in 0..(count % 4) {
                if regs.sr.read(SR::NBWE) == 0 {
                    debug!("SHA: 'buffer is full', process says");
                    return i + words_num;
                }
                let data_idx = (count - (count % 4)) + i;
                // d |= (data[data_idx] as u32) << (8 * i);
                self.leftover_buffer
                    .update(|buf| buf >> 8 | (data[data_idx] as u32).rotate_right(8));
                debug!(
                    "SHA: leftover right now: 0x{:02x}",
                    self.leftover_buffer.get()
                );
                self.leftover_index.update(|index| (index + 1) % 4);
            }
            debug!("SHA: leftovers 0x{:02x}", self.leftover_buffer.get());
            if self.leftover_index.get() == 0 {
                // It is safe because u32::default() = 0
                regs.din.set(self.leftover_buffer.take());
            }
        }

        count
    }

    fn handle_leftover(&self, data: &dyn Index<usize, Output = u8>) -> usize {
        let bytes_written = 4 - self.leftover_index.take();
        for data_idx in 0..bytes_written {
            self.leftover_buffer
                .update(|buf| buf >> 8 | (data[data_idx] as u32).rotate_right(8));
        }
        self.regs.din.set(self.leftover_buffer.take());
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
                    // NOTE(frihetselsker): Look at leasable_buffer section in the docs.
                    if b.len() > 3 && self.leftover_index.get() != 0 {
                        let count = self.handle_leftover(&b);
                        b.slice(count..);
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
                    if b.len() > 3 && self.leftover_index.get() != 0 {
                        let count = self.handle_leftover(&b);
                        b.slice(count..);
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

    fn load_key(&self, is_inner: bool) {
        let regs = self.regs;
        self.hmac_key.map(|buf| {
            let count = self.process(buf, self.hmac_key_len.get() - self.hmac_key_index.get());
            debug!("Key loader: Sent {} bytes", count);
            self.hmac_key_index.update(|idx| idx + count);
        });
        // Time to compute digest on it.
        debug!(
            "Key loader: key_len {} bytes, index {} bytes",
            self.hmac_key_len.get(),
            self.hmac_key_index.get()
        );
        if self.hmac_key_len.get() == self.hmac_key_index.get() {
            regs.imr.modify(IMR::DINIE::SET);
            if self.leftover_buffer.get() != 0 {
                self.write_final_leftover();
            }

            self.state.update(|_| {
                if is_inner {
                    Some(State::KeyDigestAdd)
                } else {
                    Some(State::KeyDigestRun)
                }
            });
            // start the final digest calculation
            debug!("KEY-SHA: Start computation");
            // debug!(
            //     "SHA: busy {}, pushed {}, left {}",
            //     regs.sr.read(SR::BUSY),
            //     regs.sr.read(SR::NBWP),
            //     regs.sr.read(SR::NBWE)
            // );
            regs.str.modify(STR::DCAL::SET);
        } else {
            // We need to process more
            self.state.update(|_| {
                if is_inner {
                    Some(State::KeyLoadAdd)
                } else {
                    Some(State::KeyLoadRun)
                }
            });
            regs.imr.modify(IMR::DINIE::SET);
        }
    }

    fn write_final_leftover(&self) {
        let regs = self.regs;
        debug!("SHA: bits filled: {}", self.leftover_index.get() * 8);
        debug!(
            "SHA: leftover we need to write: 0x{:02x}",
            self.leftover_buffer.get() >> (8 * (4 - self.leftover_index.get()))
        );
        regs.din
            .set(self.leftover_buffer.take() >> (8 * (4 - self.leftover_index.get())));
        regs.str
            .modify(STR::NBLW.val(self.leftover_index.take() as u32 * 8));
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
        // Do I need to create a separate function?
        if self.leftover_buffer.get() != 0 {
            self.write_final_leftover();
        } else {
            regs.str.modify(STR::NBLW.val(0));
        }
        // enable the interrupt
        //debug!("SHA: Enable the interrupt for digest finish");
        if self.hmac_key.is_some() {
            regs.imr.modify(IMR::DCIE::SET + IMR::DINIE::SET);
        } else {
            regs.imr.modify(IMR::DCIE::SET);
        }
        // start the final digest calculation
        debug!("SHA: Start computation");
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
        debug!("SHA: Entered SHA data adder");
        if data.len() > self.regs.sr.read(SR::NBWE) as usize
            || (self.hmac_key_len.get() > self.regs.sr.read(SR::NBWE) as usize
                && self.hmac_key_len.get() > self.hmac_key_index.get())
        {
            self.regs.imr.modify(IMR::DINIE::SET);
        }

        self.data.set(Some(SubSliceMutImmut::Immutable(data)));

        debug!(
            "HMAC-SHA: key_len {}, index {}",
            self.hmac_key_len.get(),
            self.hmac_key_index.get()
        );
        // If we have key and it is not loaded yet, do load it now
        if self.hmac_key.is_some() && self.hmac_key_len.get() > self.hmac_key_index.get() {
            debug!("HMAC-SHA: We need to load a key");
            self.load_key(true);
        } else {
            self.state.set(Some(State::Add));
            // Otherwise, act as usual
            let ret = self.data_progress();
            // Why do I need this variable here?
            if ret {
                self.regs.imr.modify(IMR::DINIE::SET);
            } else {
                debug!("SHA: No more data to add");
                self.deferred_call.set();
            }
        }

        // let ret = self.data_progress();

        // // debug!("SHA: Data processed");

        // if !ret {
        //     debug!("SHA: No more data to add");
        //     self.deferred_call.set();
        // }

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
        debug!("SHA: Entered SHA data adder");

        // If DMA is available, then do use it.
        if let Some(dma) = self.dma.get() {
            // Move buffer to DMA buffer
            // DMA transfer can send only 32-bit words
            // NOTE: use iter() and chain()
            // If we still have leftovers, take bytes from the end and put them to the leftover buffer
            // NEW IDEA: just append bytes on top of what you got, then take a slice which can be divided by 4, add it to the DMA buffer and pass the leftovers back
            // The subslice has to be discarded

            // Turn u8 to u32
            //

            // Hardware fence
            let fence = unsafe { CortexMDmaFence::new() };
            // Convert subslice into DmaSlice
            let dma_slice = DmaSubSliceMut::new_static(data, fence);

            // Extract the physical pointer and length for MMIO
            let ptr = dma_slice.as_mut_ptr() as u32;
            let len = dma_slice.len() as u32;

            // Save DmaSlice in the struct
            self.dma_buffer
                .replace(DmaSubSliceMutImmut::Mutable(dma_slice));

            // Trigger USART
            if let Some(ch) = self.dma_channel.get() {
                dma.setup(ch, crate::dma::DmaPeripheral::Hash, ptr, len);
                self.regs.cr3.modify(CR3::DMAT::SET);
                Ok(())
            } else {
                self.dma_buffer
                    .take()
                    .map(|s| match s {
                        DmaSubSliceMutImmut::Immutable(b) => {}
                        DmaSubSliceMutImmut::Mutable(b) => {
                            let f = unsafe { CortexMDmaFence::new() };
                            let mut buf = unsafe { b.take(f) };
                            buf.reset();
                            Err((kernel::ErrorCode::RESERVE, buf.take()))
                        }
                    })
                    .unwrap()
            }
        } else {
            // Otherwise, rely on the software loading
            self.data.set(Some(SubSliceMutImmut::Mutable(data)));

            if data.len() > self.regs.sr.read(SR::NBWE) as usize
                || (self.hmac_key_len.get() > self.regs.sr.read(SR::NBWE) as usize
                    && self.hmac_key_len.get() > self.hmac_key_index.get())
            {
                self.regs.imr.modify(IMR::DINIE::SET);
            }

            debug!(
                "HMAC-SHA: key_len {}, index {}",
                self.hmac_key_len.get(),
                self.hmac_key_index.get()
            );
            // If we have key and it is not loaded yet, do load it now
            if self.hmac_key.is_some() && self.hmac_key_len.get() > self.hmac_key_index.get() {
                debug!("HMAC-SHA: We need to load a key");
                self.load_key(true);
            } else {
                self.state.set(Some(State::Add));
                // Otherwise, act as usual
                let ret = self.data_progress();
                // Why do I need this variable here?
                if ret {
                    self.regs.imr.modify(IMR::DINIE::SET);
                } else {
                    debug!("SHA: No more data to add");
                    self.deferred_call.set();
                }
            }

            // let ret = self.data_progress();

            // // debug!("SHA: Data processed");

            // if !ret {
            //     debug!("SHA: No more data to add");
            //     self.deferred_call.set();
            // }

            Ok(())
        }
    }

    fn clear_data(&self) {
        unimplemented!()
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
        self.regs
            .cr
            .modify(CR::ALGO::MD5 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        self.hmac_key.take();
        Ok(())
    }
}

impl Sha1 for Hash<'_> {
    fn set_mode_sha1(&self) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA1));
        self.regs
            .cr
            .modify(CR::ALGO::SHA_1 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        self.hmac_key.take();
        Ok(())
    }
}

impl Sha224 for Hash<'_> {
    fn set_mode_sha224(&self) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_224));
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_224 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        self.hmac_key.take();
        Ok(())
    }
}

impl Sha256 for Hash<'_> {
    fn set_mode_sha256(&self) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_256));
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_256 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        self.hmac_key.take();
        Ok(())
    }
}

impl HmacSha256 for Hash<'_> {
    fn set_mode_hmacsha256(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_256));
        debug!("Set mode: Key length: {}", key.len());
        self.hmac_key
            .map(|buf| {
                debug!("Set mode: Buf length: {}", buf.len());
                if buf.len() >= key.len() {
                    buf[..key.len()].copy_from_slice(key);
                    self.hmac_key_len.set(key.len());
                    Ok(())
                } else {
                    Err(ErrorCode::SIZE)
                }
            })
            .transpose()?;
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_256 + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > (HASH_FIFO_SIZE - 1) * 4 {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        // now it's time to fill the buffer in

        // self.hmac_key.set(key);
        // we need to store this key throughout the whole operation
        // and then load it to data subslice and process it as usually
        //
        // The problem is how to store a reference with anonymous lifetime.
        //
        // self.regs.cr.modify(CR::INIT::SET);
        //
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
        debug!("Set mode: Key length: {}", key.len());
        self.hmac_key
            .map(|buf| {
                debug!("Set mode: Buf length: {}", buf.len());
                if buf.len() >= key.len() {
                    buf[..key.len()].copy_from_slice(key);
                    self.hmac_key_len.set(key.len());
                    Ok(())
                } else {
                    Err(ErrorCode::SIZE)
                }
            })
            .transpose()?;
        self.regs
            .cr
            .modify(CR::ALGO::MD5 + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > (HASH_FIFO_SIZE - 1) * 4 {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        // now it's time to fill the buffer in

        // self.hmac_key.set(key);
        // we need to store this key throughout the whole operation
        // and then load it to data subslice and process it as usually
        //
        // The problem is how to store a reference with anonymous lifetime.
        //
        // self.regs.cr.modify(CR::INIT::SET);
        //
        Ok(())
    }
}

impl HmacSha1 for Hash<'_> {
    fn set_mode_hmacsha1(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA1));
        debug!("Set mode: Key length: {}", key.len());
        self.hmac_key
            .map(|buf| {
                debug!("Set mode: Buf length: {}", buf.len());
                if buf.len() >= key.len() {
                    buf[..key.len()].copy_from_slice(key);
                    self.hmac_key_len.set(key.len());
                    Ok(())
                } else {
                    Err(ErrorCode::SIZE)
                }
            })
            .transpose()?;
        self.regs
            .cr
            .modify(CR::ALGO::SHA_1 + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > (HASH_FIFO_SIZE - 1) * 4 {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        // now it's time to fill the buffer in

        // self.hmac_key.set(key);
        // we need to store this key throughout the whole operation
        // and then load it to data subslice and process it as usually
        //
        // The problem is how to store a reference with anonymous lifetime.
        //
        // self.regs.cr.modify(CR::INIT::SET);
        //
        Ok(())
    }
}

impl HmacSha224 for Hash<'_> {
    fn set_mode_hmacsha224(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_224));
        debug!("Set mode: Key length: {}", key.len());
        self.hmac_key
            .map(|buf| {
                debug!("Set mode: Buf length: {}", buf.len());
                if buf.len() >= key.len() {
                    buf[..key.len()].copy_from_slice(key);
                    self.hmac_key_len.set(key.len());
                    Ok(())
                } else {
                    Err(ErrorCode::SIZE)
                }
            })
            .transpose()?;
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_224 + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > (HASH_FIFO_SIZE - 1) * 4 {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }

        // now it's time to fill the buffer in

        // self.hmac_key.set(key);
        // we need to store this key throughout the whole operation
        // and then load it to data subslice and process it as usually
        //
        // The problem is how to store a reference with anonymous lifetime.
        //
        // self.regs.cr.modify(CR::INIT::SET);
        //
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
        debug!("SHA: entered deferred call handler");
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
