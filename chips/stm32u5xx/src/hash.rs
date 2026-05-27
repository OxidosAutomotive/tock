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
use kernel::utilities::leasable_buffer::{SubSlice, SubSliceMutImmut};
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
        // debug!("Leftover: add new byte");
        if !self.is_full() {
            self.buffer.update(|buf| Leftover::add_byte(buf, byte));
        } else {
            self.secondary_buffer
                .update(|buf| Leftover::add_byte(buf, byte));
        }

        // match self.buffer.get() {
        //     Some(buf) => debug!("Leftover: current buffer: 0x{:02x}", buf),
        //     _ => debug!("Leftover: current buffer not exist"),
        // }

        // match self.secondary_buffer.get() {
        //     Some(buf) => debug!("Leftover: current 2ndary buffer: 0x{:02x}", buf),
        //     _ => debug!("Leftover: current 2ndary buffer not exist"),
        // }

        self.index.update(|index| (index + 1) % 8);
        // debug!("Leftover: current index {}", self.index.get());
    }

    pub fn to_le(&self) -> u32 {
        // debug!("Leftover: entered to_le");
        let res = match self.buffer.take() {
            Some(b) => {
                // debug!("Leftover: value before shifting: 0x{:02x}", b);
                let value = b >> (8 * self.bytes_left());
                // CRITICAL: This is a bug!!!!
                // UPD: Fixed
                // debug!("Leftover: value written to the register: 0x{:02x}", value);
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

pub struct HmacKey<'a> {
    key: Cell<Option<SubSlice<'a, u8>>>,
    is_loaded: Cell<Option<bool>>,
}

impl<'a> HmacKey<'a> {
    pub fn new() -> Self {
        Self {
            key: Cell::new(None),
            is_loaded: Cell::new(None),
        }
    }

    pub fn set(&self, key: &'a [u8]) {
        self.key.set(Some(SubSlice::new(key)));
        self.is_loaded.set(Some(false));
    }

    pub fn is_stored(&self) -> bool {
        self.key.get().is_some()
    }

    pub fn len(&self) -> usize {
        if let Some(key) = self.key.get() {
            key.len()
        } else {
            0
        }
    }

    pub fn is_loaded(&self) -> bool {
        if let Some(loaded) = self.is_loaded.get() {
            loaded
        } else {
            false
        }
    }

    pub fn prepare_key(&self) {
        if let Some(mut key) = self.key.get() {
            key.reset();
        }
    }

    pub fn left_to_load(&self) -> usize {
        if let Some(key) = self.key.get() {
            key.len()
        } else {
            0
        }
    }

    pub fn reset(&self) {
        if let Some(mut key) = self.key.get() {
            self.is_loaded.set(Some(false));
            key.reset();
        }
    }

    pub fn clear(&self) {
        self.is_loaded.set(None);
        self.key.take();
    }
}

pub struct Hash<'a> {
    regs: StaticRef<HashRegisters>,
    dma: OptionalCell<&'a Dma>,
    dma_channel: Cell<Option<ChannelId>>,
    dma_buffer: MapCell<DmaSubSliceMutImmut<'static, u8>>,
    mode: Cell<Option<Mode>>,
    state: Cell<Option<State>>,
    hmac_key: HmacKey<'static>,
    data: Cell<Option<SubSliceMutImmut<'static, u8>>>,
    leftover: Leftover,
    verify: Cell<bool>,
    // How to be more flexible in this field?
    client: OptionalCell<&'a dyn Client<MAX_DIGEST_LEN>>,
    // NOTE(frihetselsker): I don't believe it is the best way to store it without explicitly mentioning the size
    // But without it, we lose flexibility in switching modes during the execution.
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
    fn start_dma_transfer(&self, dma: &'a Dma) -> Result<(), Option<SubSliceMutImmut<'a, u8>>> {
        if let Some(state) = self.state.get() {
            let (data, is_hmac) = match state {
                State::Add => (self.data.take(), false),
                State::HmacInit | State::HmacPostAuth => {
                    if let Some(hmac) = self.hmac_key.key.get() {
                        (Some(SubSliceMutImmut::Immutable(hmac)), true)
                    } else {
                        (None, true)
                    }
                }
                _ => (None, false),
            };

            if let Some(mut data) = data {
                if !is_hmac {
                    if !self.leftover.is_empty() {
                        let count = self.trim_subslice(&data, data.len());
                        debug!("DMA-SHA: Trimmed {} bytes", count);
                        data.slice(count..);
                    }
                    // Truncate
                    let count = self.truncate_subslice(&data, data.len());
                    debug!("DMA-SHA: Truncated {} bytes", count);
                    data.slice(..data.len() - count);
                    if data.len() == 0 {
                        self.deferred_call.set();
                        return Ok(());
                    }
                }

                // Hardware fence
                let fence = unsafe { CortexMDmaFence::new() };
                // Convert subslice into DmaSlice
                // let dma_slice = DmaSubSliceMut::new_static(data, fence);
                let (dma_slice, ptr, len) = match data {
                    SubSliceMutImmut::Immutable(d) => {
                        let dma_slice = DmaSubSlice::new(d, fence);
                        // Extract the physical pointer and length for MMIO
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

                debug!("DMA-SHA: DMA transfer length: {} bytes", len);

                // Trigger HASH
                if let Some(ch) = self.dma_channel.get() {
                    let regs = self.regs;
                    // Save DmaSlice in the struct
                    self.dma_buffer.replace(dma_slice);
                    dma.setup(ch, crate::dma::DmaPeripheral::Hash, ptr, len);
                    match state {
                        State::HmacInit => self.state.set(Some(State::HmacPreAuth)),
                        State::HmacPostAuth => self.state.set(Some(State::HmacFinalize)),
                        _ => {}
                    }
                    if is_hmac {
                        // Or is it because of this?
                        regs.cr.modify(CR::MDMAT::CLEAR);
                        regs.str.write(STR::NBLW.val(8 * (len % 4)));
                    } else {
                        regs.cr.modify(CR::MDMAT::SET);
                    }
                    // regs.cr.modify(CR::MDMAT::SET);
                    // TODO(frihetselsker): I should consider checking the buffer emptiness
                    // In the case of fullness, the interrupt should be set
                    //
                    // SOmething fishy is here, I need to take a look here
                    // TODO(frihetselsker)!@!!!!!!!!!!
                    regs.imr.modify(IMR::DINIE::SET + IMR::DCIE::SET);

                    regs.cr.modify(CR::DMAE::SET);

                    Ok(())
                } else {
                    debug!("DMA-SHA: couldn't take a ch");
                    let f = unsafe { CortexMDmaFence::new() };
                    match dma_slice {
                        DmaSubSliceMutImmut::Immutable(d) => {
                            let mut buf = d.as_sub_slice();
                            buf.reset();
                            Err(Some(SubSliceMutImmut::Immutable(buf)))
                        }
                        DmaSubSliceMutImmut::Mutable(d) => {
                            let mut buf = unsafe { d.take(f) };
                            buf.reset();
                            Err(Some(SubSliceMutImmut::Mutable(buf)))
                        }
                    }
                }
            } else {
                debug!("DMA-SHA: couldn't take data");
                Err(None)
            }
        } else {
            debug!("DMA-SHA: state not found");
            Err(None)
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
            leftover: Leftover::new(),
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
        debug!("Do we have any interrupts?");
        debug!("DEBUG: Status reg: {:02b}", regs.sr.get());
        // Digest Calculation completed?
        if regs.sr.read(SR::DCIS) != 0 {
            debug!("SHA: Entered Digest Calculation interrupt");
            if let Some(state) = self.state.get() {
                // If key is present, then we need to process it accordingly
                match state {
                    State::HmacFinalize | State::Run => {
                        // it is time to return data
                        // reset index
                        // self.hmac_key_index.take();
                        self.return_data();
                    }
                    _ => (),
                }
            }
        } else if regs.sr.read(SR::DINIS) != 0 {
            debug!("SHA: Entered Data Input interrupt");
            if let Some(state) = self.state.get() {
                match state {
                    State::Add => {
                        if self.dma.is_some() {
                            // Theoretically, there can be a situation
                            // when the FIFO is already filled, but we do need to write leftovers
                            // The DMA transfer cannot be used until all the previous data is loaded
                            // to the peripheral.
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
                    State::HmacInit => {
                        // Just in case, but it should not enter it here.
                        // TODO(frihetselsker): think of it
                        if self.dma.get().is_some() {
                            // regs.imr.modify(IMR::DINIE::SET);
                            // regs.str.modify(STR::DCAL::SET);
                            self.state.set(Some(State::HmacPreAuth));
                        } else {
                            self.load_key(true);
                        }
                    }
                    State::HmacPostAuth => {
                        if self.dma.get().is_some() {
                            regs.imr.modify(IMR::DINIE::SET);
                            regs.str.modify(STR::DCAL::SET);
                            self.state.set(Some(State::HmacFinalize));
                        } else {
                            self.load_key(false);
                        }
                    }
                    State::Run => {
                        if let Some(dma) = self.dma.get() {
                            self.state.set(Some(State::HmacPostAuth));
                            self.hmac_key.reset();

                            let _ = self.start_dma_transfer(dma);
                        } else {
                            self.load_key(false);
                        }
                    }
                    State::HmacPreAuth => {
                        self.state.set(Some(State::Add));
                        self.hmac_key.is_loaded.set(Some(true));
                        debug!("It is time to write the actual data");
                        if let Some(dma) = self.dma.get() {
                            let _ = self.start_dma_transfer(dma);
                        } else {
                            if !self.data_progress() {
                                // we added all the data
                                debug!("we added all the data");
                                self.deferred_call.set();
                            } else {
                                debug!("we need more data");
                                regs.imr.modify(IMR::DINIE::SET);
                            }
                        }
                    }
                    _ => (),
                }
            }
        } else {
            debug!("Not supported interrupt");
        }
    }

    pub fn handle_dma_interrupt(&self) {
        // Disable the DMA trigger to release the channel
        let regs = self.regs;
        regs.cr.modify(CR::DMAE::CLEAR);
        debug!("DMA-INT: entered the DMA handler");
        if let Some(dma_slice) = self.dma_buffer.take() {
            if let Some(state) = self.state.get() {
                match state {
                    State::Add => {
                        self.state.take();
                        match dma_slice {
                            DmaSubSliceMutImmut::Immutable(b) => {
                                let mut subslice = b.as_sub_slice();
                                subslice.slice(0..0);
                                self.client.map(|client| {
                                    client.add_data_done(Ok(()), subslice);
                                });
                            }
                            DmaSubSliceMutImmut::Mutable(b) => {
                                let fence = unsafe { CortexMDmaFence::new() };
                                let mut subslice = unsafe { b.take(fence) };
                                subslice.slice(0..0);
                                self.client.map(|client| {
                                    client.add_mut_data_done(Ok(()), subslice);
                                });
                            }
                        }
                    }
                    State::HmacInit => {
                        regs.imr.modify(IMR::DINIE::SET);
                        regs.str.modify(STR::DCAL::SET);
                        self.state.set(Some(State::HmacPreAuth));
                    }
                    State::HmacPostAuth => {
                        regs.imr.modify(IMR::DINIE::SET);
                        regs.str.modify(STR::DCAL::SET);
                        self.state.set(Some(State::HmacFinalize));
                    }
                    _ => (),
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
                    if self.hmac_key.is_stored() {
                        self.hmac_key.is_loaded.set(Some(false));
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
                    digest[(mode.get_digest_len() * 4)..MAX_DIGEST_LEN].fill(0);

                    // self.clear_data();
                    regs.cr.modify(CR::INIT::SET);
                    // release the peripheral
                    self.state.take();
                    if self.hmac_key.is_stored() {
                        self.hmac_key.is_loaded.set(Some(false));
                    }
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
            debug!("SHA: 32-bit word written {:02x}", d);
            regs.din.set(d);
        }

        if !count.is_multiple_of(4) {
            debug!("SHA: Adding 8-bit words as leftovers");

            for i in 0..(count % 4) {
                if regs.sr.read(SR::NBWE) == 0 {
                    return i + words_num;
                }
                let data_idx = (count - (count % 4)) + i;
                // d |= (data[data_idx] as u32) << (8 * i);
                debug!("SHA: Byte 0x{:02x}", data[data_idx]);
                self.leftover.add(data[data_idx]);
            }
            if self.leftover.is_full() {
                let new_word = self.leftover.to_le();
                debug!("SHA: Leftover word written {:02x}", new_word);
                debug!("SHA: Current index: {}", self.leftover.index.get());
                regs.din.set(new_word);
            }
        }

        count
    }

    fn trim_subslice(&self, data: &dyn Index<usize, Output = u8>, count: usize) -> usize {
        let bytes_written = min(self.leftover.bytes_left(), count % 4);
        debug!("Trim_subslice: writing {} bytes", bytes_written);
        for data_idx in 0..bytes_written {
            self.leftover.add(data[data_idx]);
        }
        if self.leftover.is_full() {
            let regs = self.regs;
            // THIS IS DISASTER!!!!!!!!!!!!!!!
            if regs.sr.read(SR::NBWE) == 0 {
                debug!("THIS IS DISASTER!!!!!!");
                regs.imr.modify(IMR::DINIE::SET);
            } else {
                regs.din.set(self.leftover.to_le());
            }
        }
        bytes_written
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
        debug!("data_progress");
        self.data.take().is_some_and(|buf| match buf {
            SubSliceMutImmut::Immutable(mut b) => {
                if b.len() == 0 {
                    self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                    false
                } else {
                    // NOTE(frihetselsker): Look at leasable_buffer section in the docs.
                    debug!("Progress: any_leftover? {}", !self.leftover.is_empty());
                    if !self.leftover.is_empty() {
                        let count = self.trim_subslice(&b, b.len());
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
                    debug!("Progress: any_leftover? {}", !self.leftover.is_empty());
                    if !self.leftover.is_empty() {
                        let count = self.trim_subslice(&b, b.len());
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
        if let Some(mut key) = self.hmac_key.key.get() {
            let count = self.process(&key, key.len());
            key.slice(count..);
            // self.hmac_key.key.set(Some(key));
            debug!("Key loader: Sent {} bytes", count);

            // Time to compute digest on it.
            debug!(
                "Key loader: left to load {} bytes",
                self.hmac_key.left_to_load()
            );

            if key.len() == 0 {
                regs.imr.modify(IMR::DINIE::SET);
                if !self.leftover.is_empty() {
                    self.flush_leftover();
                }

                self.hmac_key.is_loaded.set(Some(true));

                self.state.update(|_| {
                    if is_inner {
                        Some(State::HmacPreAuth)
                    } else {
                        Some(State::HmacFinalize)
                    }
                });
                // start the final digest calculation
                debug!("KEY-SHA: Start computation");
                regs.str.modify(STR::DCAL::SET);
            } else {
                // We need to process more
                self.state.update(|_| {
                    if is_inner {
                        Some(State::HmacInit)
                    } else {
                        Some(State::HmacPostAuth)
                    }
                });
                regs.imr.modify(IMR::DINIE::SET);
            }
        }
    }

    fn flush_leftover(&self) {
        let regs = self.regs;
        debug!("SHA: bits filled: {}", self.leftover.bytes_left() * 8);
        debug!(
            "SHA: this is what we write to NBLW: {}",
            ((4 - self.leftover.bytes_left()) * 8)
        );
        regs.str
            // NOTE(frihetselsker): This is AWFUL
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
        // Do I need to create a separate function?
        debug!(
            "RUN: Do we need to flush leftovers? {}",
            !self.leftover.is_empty()
        );
        debug!("RUN: leftover index {}", self.leftover.index.get());
        if !self.leftover.is_empty() {
            debug!("RUN: We have some leftovers here");
            self.flush_leftover();
        } else {
            regs.str.modify(STR::NBLW.val(0));
        }
        // enable the interrupt
        //debug!("SHA: Enable the interrupt for digest finish");
        if self.hmac_key.is_stored() {
            debug!("RUN: HMAC interrupts");
            self.hmac_key.reset();
            debug!("RUN: Key length: {}", self.hmac_key.len());
            // regs.imr.modify(IMR::DINIE::SET);
            regs.imr.modify(IMR::DCIE::SET);
        } else {
            debug!("RUN: simple interrupts");
            regs.imr.modify(IMR::DCIE::SET);
        }
        // start the final digest calculation
        debug!("SHA: Start computation");
        regs.str.modify(STR::DCAL::SET);
        self.state.set(Some(State::Run));
        self.digest.set(digest);

        debug!("DEBUG: Status reg: {:02b}", regs.sr.get());

        Ok(())
    }
}

impl crate::dma::DmaClient for Hash<'_> {
    fn transfer_done(&self, channel: ChannelId) {
        if let Some(ch) = self.dma_channel.get() {
            debug!("DMA: Transfer done, ch: {:?}", ch);
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
            // Prepare the buffer with slicing
            // Trim from the beginning
            // We must ensure that DMA receives size divisible by 4 (only 32-bit words)
            match self.start_dma_transfer(dma) {
                Ok(()) => Ok(()),
                Err(_) => Err((kernel::ErrorCode::RESERVE, data)),
            }

            // Err((kernel::ErrorCode::RESERVE, buf))
        } else {
            if data.len() > self.regs.sr.read(SR::NBWE) as usize
                || (self.hmac_key.left_to_load() > self.regs.sr.read(SR::NBWE) as usize
                    && !self.hmac_key.is_loaded())
            {
                self.regs.imr.modify(IMR::DINIE::SET);
            }

            self.data.set(Some(SubSliceMutImmut::Immutable(data)));

            // debug!(
            //     "HMAC-SHA: key_len {}, index {}",
            //     self.hmac_key_len.get(),
            //     self.hmac_key_index.get()
            // );
            // If we have key and it is not loaded yet, do load it now
            if self.hmac_key.is_stored() && self.hmac_key.left_to_load() > 0 {
                debug!("HMAC-SHA: We need to load a key");
                self.load_key(true);
            } else {
                debug!("Set ADD state");
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
            //
            // // DmaSubSliceMutImmut::Immutable(b) => {
            //     let mut buf = b.as_sub_slice();
            //     buf.reset();
            //     Err((kernel::ErrorCode::RESERVE, buf.take()))
            // }

            Ok(())
        }
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
        debug!("SHA: Entered SHA data adder");

        // let availvable_places = self.regs.sr.read(SR::NBWE);
        // debug!("SHA: {} bytes can be added", availvable_places);
        let data_len = data.len();
        self.data.set(Some(SubSliceMutImmut::Mutable(data)));

        // If DMA is available, then do use it.
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

            // We need to set the state before entering the DMA transfer
            if self.hmac_key.is_stored() && !self.hmac_key.is_loaded() {
                self.state.set(Some(State::HmacInit));
            } else {
                self.state.set(Some(State::Add));
            }
            match self.start_dma_transfer(dma) {
                Ok(()) => return Ok(()),
                Err(_) => {}
            }
        } else {
            // Otherwise, rely on the software loading
            if data_len > self.regs.sr.read(SR::NBWE) as usize
                || (self.hmac_key.left_to_load() > self.regs.sr.read(SR::NBWE) as usize
                    && !self.hmac_key.is_loaded())
            {
                self.regs.imr.modify(IMR::DINIE::SET);
            }
            // self.data.set(Some(SubSliceMutImmut::Mutable(data)));

            // debug!(
            //     "HMAC-SHA: key_len {}, index {}",
            //     self.hmac_key_len.get(),
            //     self.hmac_key_index.get()
            // );
            // If we have key and it is not loaded yet, do load it now
            if self.hmac_key.is_stored() && !self.hmac_key.is_loaded() {
                debug!("HMAC-SHA: We need to load a key");
                self.state.set(Some(State::HmacInit));
                self.load_key(true);
            } else {
                // self.state.take();
                // Otherwise, act as usual
                debug!("ADD: ADD State");
                self.state.set(Some(State::Add));
                let ret = self.data_progress();
                // Why do I need this variable here?
                if ret {
                    self.regs.imr.modify(IMR::DINIE::SET);
                } else {
                    debug!("SHA: No more data to add");
                    self.deferred_call.set();
                }
            }
            return Ok(());
        }
        Ok(())
    }

    fn clear_data(&self) {
        // NOTE(frihetselsker): you cannot cancel the operation,
        // you can only discard all the changes and reset the peripheral
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
        self.regs.cr.modify(
            CR::ALGO::SHA2_256
                + CR::MODE::CLEAR
                + CR::MDMAT::SET
                + CR::DATATYPE::_8bitData
                + CR::INIT::SET,
        );
        self.hmac_key.clear();
        Ok(())
    }
}

impl HmacSha256<'static> for Hash<'static> {
    fn set_mode_hmacsha256(&self, key: &'static [u8]) -> Result<(), kernel::ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_256));
        debug!("Set mode: Key length: {}", key.len());
        self.hmac_key.set(key);
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_256 + CR::MDMAT::SET + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > LONG_HMAC_KEY_LEN {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
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

impl HmacMd5<'static> for Hash<'static> {
    fn set_mode_hmacmd5(&self, key: &'static [u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::MD5));
        self.hmac_key.set(key);
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

impl HmacSha1<'static> for Hash<'static> {
    fn set_mode_hmacsha1(&self, key: &'static [u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA1));
        self.hmac_key.set(key);
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

impl HmacSha224<'static> for Hash<'static> {
    fn set_mode_hmacsha224(&self, key: &'static [u8]) -> Result<(), ErrorCode> {
        if self.state.get().is_some() {
            return Err(ErrorCode::BUSY);
        }
        self.mode.set(Some(Mode::SHA2_224));
        self.hmac_key.set(key);
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

impl<'a> HmacSha384<'a> for Hash<'a> {
    fn set_mode_hmacsha384(&self, _key: &'a [u8]) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl<'a> HmacSha512<'a> for Hash<'a> {
    fn set_mode_hmacsha512(&self, _key: &'a [u8]) -> Result<(), kernel::ErrorCode> {
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
