use core::cell::Cell;

use kernel::hil::digest::{
    Client, Digest, DigestData, DigestHash, DigestVerify, HmacSha256, HmacSha384, HmacSha512,
    Sha224, Sha256, Sha384, Sha512,
};
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::leasable_buffer::SubSliceMutImmut;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    self, register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

use crate::dma::Dma;

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
        (0x00C => hra0: ReadOnly<u32>),
        /// HASH aliased digest register 1
        (0x010 => hra1: ReadOnly<u32>),
        /// HASH aliased digest register 2
        (0x014 => hra2: ReadOnly<u32>),
        /// HASH aliased digest register 3
        (0x018 => hra3: ReadOnly<u32>),
        /// HASH aliased digest register 4
        (0x01C => hra4: ReadOnly<u32>),
        /// interrupt enable register
        (0x020 => imr: ReadWrite<u32, IMR::Register>),
        /// status register
        (0x024 => sr: ReadWrite<u32, SR::Register>),
        (0x028 => _reserved0),
        /// context swap registers
        (0x0F8 => csr0: ReadWrite<u32>),
        /// context swap registers
        (0x0FC => csr1: ReadWrite<u32>),
        /// context swap registers
        (0x100 => csr2: ReadWrite<u32>),
        /// context swap registers
        (0x104 => csr3: ReadWrite<u32>),
        /// context swap registers
        (0x108 => csr4: ReadWrite<u32>),
        /// context swap registers
        (0x10C => csr5: ReadWrite<u32>),
        /// context swap registers
        (0x110 => csr6: ReadWrite<u32>),
        /// context swap registers
        (0x114 => csr7: ReadWrite<u32>),
        /// context swap registers
        (0x118 => csr8: ReadWrite<u32>),
        /// context swap registers
        (0x11C => csr9: ReadWrite<u32>),
        /// context swap registers
        (0x120 => csr10: ReadWrite<u32>),
        /// context swap registers
        (0x124 => csr11: ReadWrite<u32>),
        /// context swap registers
        (0x128 => csr12: ReadWrite<u32>),
        /// context swap registers
        (0x12C => csr13: ReadWrite<u32>),
        /// context swap registers
        (0x130 => csr14: ReadWrite<u32>),
        /// context swap registers
        (0x134 => csr15: ReadWrite<u32>),
        /// context swap registers
        (0x138 => csr16: ReadWrite<u32>),
        /// context swap registers
        (0x13C => csr17: ReadWrite<u32>),
        /// context swap registers
        (0x140 => csr18: ReadWrite<u32>),
        /// context swap registers
        (0x144 => csr19: ReadWrite<u32>),
        /// context swap registers
        (0x148 => csr20: ReadWrite<u32>),
        /// context swap registers
        (0x14C => csr21: ReadWrite<u32>),
        /// context swap registers
        (0x150 => csr22: ReadWrite<u32>),
        /// context swap registers
        (0x154 => csr23: ReadWrite<u32>),
        /// context swap registers
        (0x158 => csr24: ReadWrite<u32>),
        /// context swap registers
        (0x15C => csr25: ReadWrite<u32>),
        /// context swap registers
        (0x160 => csr26: ReadWrite<u32>),
        /// context swap registers
        (0x164 => csr27: ReadWrite<u32>),
        /// context swap registers
        (0x168 => csr28: ReadWrite<u32>),
        /// context swap registers
        (0x16C => csr29: ReadWrite<u32>),
        /// context swap registers
        (0x170 => csr30: ReadWrite<u32>),
        /// context swap registers
        (0x174 => csr31: ReadWrite<u32>),
        /// context swap registers
        (0x178 => csr32: ReadWrite<u32>),
        /// context swap registers
        (0x17C => csr33: ReadWrite<u32>),
        /// context swap registers
        (0x180 => csr34: ReadWrite<u32>),
        /// context swap registers
        (0x184 => csr35: ReadWrite<u32>),
        /// context swap registers
        (0x188 => csr36: ReadWrite<u32>),
        /// context swap registers
        (0x18C => csr37: ReadWrite<u32>),
        /// context swap registers
        (0x190 => csr38: ReadWrite<u32>),
        /// context swap registers
        (0x194 => csr39: ReadWrite<u32>),
        /// context swap registers
        (0x198 => csr40: ReadWrite<u32>),
        /// context swap registers
        (0x19C => csr41: ReadWrite<u32>),
        /// context swap registers
        (0x1A0 => csr42: ReadWrite<u32>),
        /// context swap registers
        (0x1A4 => csr43: ReadWrite<u32>),
        /// context swap registers
        (0x1A8 => csr44: ReadWrite<u32>),
        /// context swap registers
        (0x1AC => csr45: ReadWrite<u32>),
        /// context swap registers
        (0x1B0 => csr46: ReadWrite<u32>),
        /// context swap registers
        (0x1B4 => csr47: ReadWrite<u32>),
        /// context swap registers
        (0x1B8 => csr48: ReadWrite<u32>),
        /// context swap registers
        (0x1BC => csr49: ReadWrite<u32>),
        /// context swap registers
        (0x1C0 => csr50: ReadWrite<u32>),
        /// context swap registers
        (0x1C4 => csr51: ReadWrite<u32>),
        /// context swap registers
        (0x1C8 => csr52: ReadWrite<u32>),
        /// context swap registers
        (0x1CC => csr53: ReadWrite<u32>),
        (0x1D0 => _reserved1),
        /// digest register 0
        (0x310 => hr0: ReadOnly<u32>),
        /// digest register 1
        (0x314 => hr1: ReadOnly<u32>),
        /// digest register 4
        (0x318 => hr2: ReadOnly<u32>),
        /// digest register 3
        (0x31C => hr3: ReadOnly<u32>),
        /// digest register 4
        (0x320 => hr4: ReadOnly<u32>),
        /// supplementary digest register 5
        (0x324 => hr5: ReadOnly<u32>),
        /// supplementary digest register 6
        (0x328 => hr6: ReadOnly<u32>),
        /// supplementary digest register 7
        (0x32C => hr7: ReadOnly<u32>),
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
    /// Number of valid bits in the last word of               the message
    NBLW OFFSET(0) NUMBITS(5) []
],
HRA0 [
    /// H0
    H0 OFFSET(0) NUMBITS(32) []
],
HRA1 [
    /// H1
    H1 OFFSET(0) NUMBITS(32) []
],
HRA2 [
    /// H2
    H2 OFFSET(0) NUMBITS(32) []
],
HRA3 [
    /// H3
    H3 OFFSET(0) NUMBITS(32) []
],
HRA4 [
    /// H4
    H4 OFFSET(0) NUMBITS(32) []
],
HR0 [
    /// H0
    H0 OFFSET(0) NUMBITS(32) []
],
HR1 [
    /// H1
    H1 OFFSET(0) NUMBITS(32) []
],
HR2 [
    /// H2
    H2 OFFSET(0) NUMBITS(32) []
],
HR3 [
    /// H3
    H3 OFFSET(0) NUMBITS(32) []
],
HR4 [
    /// H4
    H4 OFFSET(0) NUMBITS(32) []
],
HR5 [
    /// H5
    H5 OFFSET(0) NUMBITS(32) []
],
HR6 [
    /// H6
    H6 OFFSET(0) NUMBITS(32) []
],
HR7 [
    /// H7
    H7 OFFSET(0) NUMBITS(32) []
],
IMR [
    /// Digest calculation completion interrupt               enable
    DCIE OFFSET(1) NUMBITS(1) [],
    /// Data input interrupt               enable
    DINIE OFFSET(0) NUMBITS(1) []
],
SR [
    /// Busy bit
    BUSY OFFSET(3) NUMBITS(1) [],
    /// DMA Status
    DMAS OFFSET(2) NUMBITS(1) [],
    /// Digest calculation completion interrupt               status
    DCIS OFFSET(1) NUMBITS(1) [],
    /// Data input interrupt               status
    DINIS OFFSET(0) NUMBITS(1) [],
    /// Number of words expected
    NBWE OFFSET(16) NUMBITS(5) [],
    /// DIN not empty
    DINNE OFFSET(15) NUMBITS(1) [],
    /// Number of words already pushed
    NBWP OFFSET(9) NUMBITS(5) []
],
CSR0 [
    /// CS0
    CS0 OFFSET(0) NUMBITS(32) []
],
CSR1 [
    /// CS1
    CS1 OFFSET(0) NUMBITS(32) []
],
CSR2 [
    /// CS2
    CS2 OFFSET(0) NUMBITS(32) []
],
CSR3 [
    /// CS3
    CS3 OFFSET(0) NUMBITS(32) []
],
CSR4 [
    /// CS4
    CS4 OFFSET(0) NUMBITS(32) []
],
CSR5 [
    /// CS5
    CS5 OFFSET(0) NUMBITS(32) []
],
CSR6 [
    /// CS6
    CS6 OFFSET(0) NUMBITS(32) []
],
CSR7 [
    /// CS7
    CS7 OFFSET(0) NUMBITS(32) []
],
CSR8 [
    /// CS8
    CS8 OFFSET(0) NUMBITS(32) []
],
CSR9 [
    /// CS9
    CS9 OFFSET(0) NUMBITS(32) []
],
CSR10 [
    /// CS10
    CS10 OFFSET(0) NUMBITS(32) []
],
CSR11 [
    /// CS11
    CS11 OFFSET(0) NUMBITS(32) []
],
CSR12 [
    /// CS12
    CS12 OFFSET(0) NUMBITS(32) []
],
CSR13 [
    /// CS13
    CS13 OFFSET(0) NUMBITS(32) []
],
CSR14 [
    /// CS14
    CS14 OFFSET(0) NUMBITS(32) []
],
CSR15 [
    /// CS15
    CS15 OFFSET(0) NUMBITS(32) []
],
CSR16 [
    /// CS16
    CS16 OFFSET(0) NUMBITS(32) []
],
CSR17 [
    /// CS17
    CS17 OFFSET(0) NUMBITS(32) []
],
CSR18 [
    /// CS18
    CS18 OFFSET(0) NUMBITS(32) []
],
CSR19 [
    /// CS19
    CS19 OFFSET(0) NUMBITS(32) []
],
CSR20 [
    /// CS20
    CS20 OFFSET(0) NUMBITS(32) []
],
CSR21 [
    /// CS21
    CS21 OFFSET(0) NUMBITS(32) []
],
CSR22 [
    /// CS22
    CS22 OFFSET(0) NUMBITS(32) []
],
CSR23 [
    /// CS23
    CS23 OFFSET(0) NUMBITS(32) []
],
CSR24 [
    /// CS24
    CS24 OFFSET(0) NUMBITS(32) []
],
CSR25 [
    /// CS25
    CS25 OFFSET(0) NUMBITS(32) []
],
CSR26 [
    /// CS26
    CS26 OFFSET(0) NUMBITS(32) []
],
CSR27 [
    /// CS27
    CS27 OFFSET(0) NUMBITS(32) []
],
CSR28 [
    /// CS28
    CS28 OFFSET(0) NUMBITS(32) []
],
CSR29 [
    /// CS29
    CS29 OFFSET(0) NUMBITS(32) []
],
CSR30 [
    /// CS30
    CS30 OFFSET(0) NUMBITS(32) []
],
CSR31 [
    /// CS31
    CS31 OFFSET(0) NUMBITS(32) []
],
CSR32 [
    /// CS32
    CS32 OFFSET(0) NUMBITS(32) []
],
CSR33 [
    /// CS33
    CS33 OFFSET(0) NUMBITS(32) []
],
CSR34 [
    /// CS34
    CS34 OFFSET(0) NUMBITS(32) []
],
CSR35 [
    /// CS35
    CS35 OFFSET(0) NUMBITS(32) []
],
CSR36 [
    /// CS36
    CS36 OFFSET(0) NUMBITS(32) []
],
CSR37 [
    /// CS37
    CS37 OFFSET(0) NUMBITS(32) []
],
CSR38 [
    /// CS38
    CS38 OFFSET(0) NUMBITS(32) []
],
CSR39 [
    /// CS39
    CS39 OFFSET(0) NUMBITS(32) []
],
CSR40 [
    /// CS40
    CS40 OFFSET(0) NUMBITS(32) []
],
CSR41 [
    /// CS41
    CS41 OFFSET(0) NUMBITS(32) []
],
CSR42 [
    /// CS42
    CS42 OFFSET(0) NUMBITS(32) []
],
CSR43 [
    /// CS43
    CS43 OFFSET(0) NUMBITS(32) []
],
CSR44 [
    /// CS44
    CS44 OFFSET(0) NUMBITS(32) []
],
CSR45 [
    /// CS45
    CS45 OFFSET(0) NUMBITS(32) []
],
CSR46 [
    /// CS46
    CS46 OFFSET(0) NUMBITS(32) []
],
CSR47 [
    /// CS47
    CS47 OFFSET(0) NUMBITS(32) []
],
CSR48 [
    /// CS48
    CS48 OFFSET(0) NUMBITS(32) []
],
CSR49 [
    /// CS49
    CS49 OFFSET(0) NUMBITS(32) []
],
CSR50 [
    /// CS50
    CS50 OFFSET(0) NUMBITS(32) []
],
CSR51 [
    /// CS51
    CS51 OFFSET(0) NUMBITS(32) []
],
CSR52 [
    /// CS52
    CS52 OFFSET(0) NUMBITS(32) []
],
CSR53 [
    /// CS53
    CS53 OFFSET(0) NUMBITS(32) []
]
];
const HASH_BASE: StaticRef<HashRegisters> =
    unsafe { StaticRef::new(0x420C0400 as *const HashRegisters) };

const HASH_BUFFER_LEN: usize = 132;
pub struct Hash<'a> {
    regs: StaticRef<HashRegisters>,
    dma: OptionalCell<&'a Dma>,
    dma_channel: Cell<usize>,
    hmacKey: OptionalCell<&'a [u8]>,
    // mode: Cell<Mode>,
    data: Cell<Option<SubSliceMutImmut<'static, u8>>>,
    verify: Cell<bool>,
    client: OptionalCell<&'a dyn Client<32>>,
    digest: OptionalCell<&'static mut [u8; 32]>, // Maximum length that can be used
    busy: Cell<bool>,
}

impl Hash<'_> {
    pub fn new(base: StaticRef<HashRegisters>) -> Self {
        Self {
            regs: base,
            dma: OptionalCell::empty(),
            dma_channel: Cell::new(0),
            data: Cell::new(None),
            hmacKey: OptionalCell::empty(),
            verify: Cell::new(false),
            digest: OptionalCell::empty(),
            client: OptionalCell::empty(),
            busy: Cell::new(false),
        }
    }

    pub fn handle_interupts(&self) {
        // Digest Calculation completed?
        if self.regs.sr.read(SR::DCIS) != 0 {
            // disable the interrupt
            self.regs.sr.modify(SR::DCIS::CLEAR);
            // copy all the data to the digest
            if let Some(digest_buffer) = self.digest.take() {
                let regs = [
                    self.regs.hr0.get(),
                    self.regs.hr1.get(),
                    self.regs.hr2.get(),
                    self.regs.hr3.get(),
                    self.regs.hr4.get(),
                    self.regs.hr5.get(),
                    self.regs.hr6.get(),
                    self.regs.hr7.get(),
                ];

                for (i, word) in regs.iter().enumerate() {
                    let bytes = word.to_be_bytes();
                    let start = i * 4;
                    // A general condition for digests with different lengths
                    if start + 4 <= digest_buffer.len() {
                        digest_buffer[start..start + 4].copy_from_slice(&bytes);
                    }
                }
                // release hash peripheral
                self.busy.set(false);

                // notify the client that digest has been calculated
                self.client.map(|client| {
                    client.hash_done(Ok(()), digest_buffer);
                });
            }
        }
        // New data can be written
        if self.regs.sr.read(SR::DINIS) != 0 {
            // disable the interrupt
            self.regs.sr.modify(SR::DINIS::CLEAR);
            // notify the client that new data can be written.
            self.client.map(|client| client.add_data_done(result, data))
        }
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
        self.regs.
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
        todo!()
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
        todo!()
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
        self.regs.cr.modify(CR::ALGO::SHA2_224 + CR::MODE::CLEAR);
        Ok(())
    }
}

impl Sha256 for Hash<'_> {
    fn set_mode_sha256(&self) -> Result<(), kernel::ErrorCode> {
        if self.busy.get() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.regs.cr.modify(CR::ALGO::SHA2_256 + CR::MODE::CLEAR);
        Ok(())
    }
}

impl HmacSha256 for Hash<'_> {
    fn set_mode_hmacsha256(&self, key: &[u8]) -> Result<(), kernel::ErrorCode> {
        if self.busy.get() {
            return Err(kernel::ErrorCode::BUSY);
        }
        self.regs.cr.modify(CR::ALGO::SHA2_256 + CR::MODE::SET);
        if key.len() > 64 {
            self.regs.cr.modify(CR::LKEY::SET);
        } else {
            self.regs.cr.modify(CR::LKEY::CLEAR);
        }
        self.hmacKey.set(key);
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
