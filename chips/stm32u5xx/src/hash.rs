use core::cell::Cell;
use core::ops::Index;

use kernel::deferred_call::{DeferredCall, DeferredCallClient};
use kernel::hil::digest::{
    Bit16Data, Bit1Data, Bit32Data, Bit8Data, Client, Digest, DigestData, DigestHash, DigestVerify,
    HmacSha256, HmacSha384, HmacSha512, Sha224, Sha256, Sha384, Sha512,
};
use kernel::utilities::cells::OptionalCell;
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
    HashRegisters {
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
const HASH_BASE: StaticRef<HashRegisters> =
    unsafe { StaticRef::new(0x420C0400 as *const HashRegisters) };

// In terms of 8-bit words
const HASH_FIFO_SIZE: usize = 68;
const DIGEST_BLOCK_SIZE: usize = 64;

// TODO(frihetselsker): What if we add data byte by byte? We need to store the last 32 bit word as well.
// I don't know how it is handled by the buffer
//
// Also, to save more space, I can use registers

pub struct Hash<'a> {
    regs: StaticRef<HashRegisters>,
    // dma: OptionalCell<&'a Dma>,
    // dma_channel: Cell<usize>,
    _hmac_key: OptionalCell<&'a [u8]>,
    data: Cell<Option<SubSliceMutImmut<'static, u8>>>,
    verify: Cell<bool>,
    leftover_buffer: Cell<u32>,
    leftover_index: Cell<usize>,
    client: OptionalCell<&'a dyn Client<32>>,
    digest: OptionalCell<&'static mut [u8; 32]>, // Maximum length that can be used
    deferred_call: DeferredCall,
    busy: Cell<bool>,
}

impl Hash<'_> {
    pub fn new(base: StaticRef<HashRegisters>) -> Self {
        Self {
            regs: base,
            // dma: OptionalCell::empty(),
            // dma_channel: Cell::new(0),
            data: Cell::new(None),
            _hmac_key: OptionalCell::empty(),
            verify: Cell::new(false),
            leftover_buffer: Cell::new(0),
            leftover_index: Cell::new(0),
            digest: OptionalCell::empty(),
            client: OptionalCell::empty(),
            deferred_call: DeferredCall::new(),
            busy: Cell::new(false),
        }
    }

    pub fn handle_interupts(&self) {
        let regs = self.regs;

        // Digest Calculation completed?
        if regs.sr.read(SR::DCIS) != 0 {
            self.client.map(|client| {
                let digest = self.digest.take().unwrap();
                //debug!("SHA: Entered interrupt related to digest calculation");
                // We need to compare the result with the digest received before.
                if self.verify.get() {
                    let mut equal = true;

                    for i in 0..8 {
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
                    self.busy.set(false);
                    self.verify.set(false);
                    client.verification_done(Ok(equal), digest);
                } else {
                    for i in 0..8 {
                        let d = regs.hr[i].get().to_be_bytes();

                        let idx = i * 4;

                        digest[idx + 0] = d[0];
                        digest[idx + 1] = d[1];
                        digest[idx + 2] = d[2];
                        digest[idx + 3] = d[3];
                    }

                    // self.clear_data();
                    regs.cr.modify(CR::INIT::SET);
                    self.busy.set(false);
                    client.hash_done(Ok(()), digest);
                }
            });
            // New data can be written (if we filled the FIFO buffer)
        } else if regs.sr.read(SR::DINIS) != 0 {
            debug!("SHA: The whole buffer has been processed, we can add more!");
            // clear interrupt
            regs.sr.modify(SR::DINIS::CLEAR);

            // if false, we are done
            if !self.data_progress() {
                // self.client.map(|client| {
                //     self.busy.set(false);
                //     self.data.take().map(|buf| match buf {
                //         SubSliceMutImmut::Immutable(b) => client.add_data_done(Ok(()), b),
                //         SubSliceMutImmut::Mutable(b) => client.add_mut_data_done(Ok(()), b),
                //     })
                // });
                // Should we use call client here directly?
                self.deferred_call.set();
                regs.imr.modify(IMR::DINIE::CLEAR);
            } else {
                regs.imr.modify(IMR::DINIE::SET);
            }
        }
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
            debug!("SHA: big word: 0x{:02x}", d);

            //debug!("SHA: 32-bit word written {:02x}", d);
            //

            regs.din.set(d);

            debug!(
                "SHA: last_index after insertions: {}",
                regs.sr.read(SR::NBWP)
            );
            debug!("SHA: words left to add: {}", regs.sr.read(SR::NBWE));
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

    // Return true if processing more data, false if the buffer
    // is completely processed.
    fn data_progress(&self) -> bool {
        self.data.take().is_some_and(|buf| match buf {
            SubSliceMutImmut::Immutable(mut b) => {
                if b.len() == 0 {
                    self.data.set(Some(SubSliceMutImmut::Immutable(b)));
                    false
                } else {
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
}

impl<'a> DigestHash<'a, 32> for Hash<'a> {
    fn set_hash_client(&'a self, client: &'a dyn kernel::hil::digest::ClientHash<32>) {
        unimplemented!();
    }

    fn run(
        &'a self,
        digest: &'static mut [u8; 32],
    ) -> Result<(), (kernel::ErrorCode, &'static mut [u8; 32])> {
        if self.busy.get() {
            return Err((ErrorCode::BUSY, digest));
        }
        let regs = self.regs;
        // set the padding
        // assume that we write bytes, not bit by bit
        if self.leftover_buffer.get() != 0 {
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
        // enable the interrupt
        //debug!("SHA: Enable the interrupt for digest finish");
        regs.imr.modify(IMR::DCIE::SET);
        // start the final digest calculation
        debug!("SHA: Start computation");
        regs.str.modify(STR::DCAL::SET);
        self.busy.set(true);
        self.digest.set(digest);

        Ok(())
    }
}

impl<'a> DigestData<'a, 32> for Hash<'a> {
    fn set_data_client(&'a self, client: &'a dyn kernel::hil::digest::ClientData<32>) {
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
        if self.busy.get() {
            Err((ErrorCode::BUSY, data))
        } else {
            self.busy.set(true);
            self.data.set(Some(SubSliceMutImmut::Immutable(data)));

            let ret = self.data_progress();

            if ret {
                self.regs.imr.modify(IMR::DINIE::SET);
            } else {
                self.deferred_call.set();
            }

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
        if self.busy.get() {
            Err((ErrorCode::BUSY, data))
        } else {
            debug!("SHA: Entered SHA data adder");
            if data.len() > HASH_FIFO_SIZE {
                self.regs.imr.modify(IMR::DINIE::SET);
            }
            self.busy.set(true);
            self.data.set(Some(SubSliceMutImmut::Mutable(data)));
            debug!("SHA: Set data");

            let ret = self.data_progress();

            debug!("SHA: Data processed");

            if ret {
                debug!("SHA: More data to add");
            } else {
                debug!("SHA: No more data to add");
                // Or do I need to call the client directly?
                // Personally, I don't think so
                self.deferred_call.set();
            }

            Ok(())
        }
    }

    fn clear_data(&self) {
        unimplemented!()
    }
}

impl<'a> DigestVerify<'a, 32> for Hash<'a> {
    fn set_verify_client(&'a self, client: &'a dyn kernel::hil::digest::ClientVerify<32>) {
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

impl Sha224 for Hash<'_> {
    fn set_mode_sha224(&self) -> Result<(), kernel::ErrorCode> {
        if self.busy.get() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_224 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        Ok(())
    }
}

impl Sha256 for Hash<'_> {
    fn set_mode_sha256(&self) -> Result<(), kernel::ErrorCode> {
        if self.busy.get() {
            return Err(kernel::ErrorCode::BUSY);
        }
        let regs = self.regs;
        regs.cr
            .modify(CR::ALGO::SHA2_256 + CR::MODE::CLEAR + CR::DATATYPE::_8bitData + CR::INIT::SET);
        Ok(())
    }
}

impl HmacSha256 for Hash<'_> {
    fn set_mode_hmacsha256(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.busy.get() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.regs
            .cr
            .modify(CR::ALGO::SHA2_256 + CR::DATATYPE::_8bitData + CR::MODE::SET);
        if key.len() > 64 {
            self.regs.cr.modify(CR::LKEY::SET + CR::INIT::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR + CR::INIT::SET);
        }
        // self.hmac_key.set(key);
        // self.regs.cr.modify(CR::INIT::SET);
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

impl HmacSha384 for Hash<'_> {
    fn set_mode_hmacsha384(&self, _key: &[u8]) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }
}

impl HmacSha512 for Hash<'_> {
    fn set_mode_hmacsha512(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
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
        self.busy.set(false);
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

pub unsafe fn init() -> &'static Hash<'static> {
    kernel::static_init!(Hash, Hash::new(HASH_BASE))
}
