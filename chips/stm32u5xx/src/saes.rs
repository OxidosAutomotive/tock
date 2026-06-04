use core::cell::Cell;
use core::marker::PhantomData;
use kernel::errorcode::ErrorCode;
use kernel::hil::symmetric_encryption::{
    AESKeySize, AES, AES128_KEY_SIZE, AES_BLOCK_SIZE, AES_IV_SIZE, WrappedKeyClient
};
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{
    self, register_bitfields, register_structs, ReadOnly, ReadWrite, WriteOnly,
};

use kernel::utilities::StaticRef;

register_structs! {
    /// Secure AES coprocessor
    pub SaesRegisters {
        /// control register
        (0x000 => cr: ReadWrite<u32, CR::Register>),
        /// status register
        (0x004 => sr: ReadOnly<u32, SR::Register>),
        /// data input register
        (0x008 => dinr: WriteOnly<u32>),
        /// data output register
        (0x00C => doutr: ReadOnly<u32>),
        /// key registers 0-3
        (0x010 => keyr: [WriteOnly<u32, Data::Register>; 4]),
        /// initialization vector registers
        (0x020 => ivr: [ReadWrite<u32, Data::Register>; 4]),
        /// key register 4-7
        (0x030 => keyr2: [WriteOnly<u32, Data::Register>; 4]),
        (0x040 => _reserved0),
        /// dpa configuration register
        (0x100 => dpacfgr: ReadWrite<u32, DPACFGR::Register>),
        (0x104 => _reserved1),
        /// interrupt enable register
        (0x300 => ier: ReadWrite<u32, IER::Register>),
        /// interrupt status register
        (0x304 => isr: ReadOnly<u32, ISR::Register>),
        /// interrupt clear register
        (0x308 => icr: WriteOnly<u32, ICR::Register>),
        (0x30C => @END),
    }
}
register_bitfields![u32,
CR [
    /// IPRST
    IPRST OFFSET(31) NUMBITS(1) [],
    /// KEYSEL
    KEYSEL OFFSET(28) NUMBITS(3) [
        /// Software key, loaded in key registers SAES_KEYx
        SOFTWARE = 0b000,
        /// Derived hardware unique key
        DHUK = 0b001,
        /// Boot hardware key
        BHK = 0b010,
        /// XOR of DHUK and BHK
        XOR_DHUK_BHK = 0b100,
        /// Test mode, 256 bit (0xA5A5A5...A5A5)
        TEST = 0b111
    ],
    /// KSHAREID
    KSHAREID OFFSET(26) NUMBITS(2) [
        AES = 00,
    ],
    /// KMOD
    KMOD OFFSET(24) NUMBITS(2) [
        NORMAL = 0b00,
        WRAPPED = 0b01,
        SHARED = 0b10,
    ],
    /// KEYPROT
    KEYPROT OFFSET(19) NUMBITS(1) [
        /// application can transfer the ownership of the SAES, with its
        /// loaded key, to an application running in another security context
        ALLOW_TRASNFER = 0,
        /// generates key error flag if the application trying to access any register
        /// has a different security context
        RESTRICT_TRANSFER = 1,
    ],
    /// KEYSIZE
    KEYSIZE OFFSET(18) NUMBITS(1) [
        AES128 = 0,
        AES256 = 1,
    ],
    /// DMAOUTEN
    DMAOUTEN OFFSET(12) NUMBITS(1) [],
    /// DMAINEN
    DMAINEN OFFSET(11) NUMBITS(1) [],
    /// CHMOD
    CHMOD OFFSET(5) NUMBITS(2) [
        ECB = 0b000,
        CBC = 0b001,
    ],
    /// MODE
    MODE OFFSET(3) NUMBITS(2) [
        Encrypt = 0b00,
        KeyDerivation = 0b01,
        Decrypt = 0b10,
    ],
    /// DATATYPE
    DATATYPE OFFSET(1) NUMBITS(2) [
        None = 0b00,       // 32-bit (No swapping)
        HalfWord = 0b01,   // 16-bit (Half-word swapping)
        Byte = 0b10,       // 8-bit (Byte swapping)
        Bit = 0b11         // 1-bit (Bit swapping)
    ],
    /// SAES enable
    EN OFFSET(0) NUMBITS(1) []
],
SR [
    /// Key Valid flag
    KEYVALID OFFSET(7) NUMBITS(1) [],
    /// BUSY
    BUSY OFFSET(3) NUMBITS(1) [],
    /// Write error flag
    WRERR OFFSET(2) NUMBITS(1) [],
    /// Read error flag
    RDERR OFFSET(1) NUMBITS(1) [],
    /// Computation complete flag
    CCF OFFSET(0) NUMBITS(1) []
],
DPACFGR [
    /// CONFIGLOCK
    CONFIGLOCK OFFSET(31) NUMBITS(1) [],
    /// TRIMCFG
    TRIMCFG OFFSET(3) NUMBITS(2) [],
    /// RESEED
    RESEED OFFSET(2) NUMBITS(1) [],
    /// REDCFG
    REDCFG OFFSET(1) NUMBITS(1) []
],
IER [
    /// RNGEIE
    RNGEIE OFFSET(3) NUMBITS(1) [],
    /// Key error interrupt enable
    KEIE OFFSET(2) NUMBITS(1) [],
    /// Read or write error interrupt enable
    RWEIE OFFSET(1) NUMBITS(1) [],
    /// Computation complete flag interrupt enable
    CCFIE OFFSET(0) NUMBITS(1) []
],
ISR [
    /// RNGEIF
    RNGEIF OFFSET(3) NUMBITS(1) [],
    /// Key error interrupt flag
    KEIF OFFSET(2) NUMBITS(1) [],
    /// Read or write error interrupt flag
    RWEIF OFFSET(1) NUMBITS(1) [],
    /// Computation complete flag
    CCF OFFSET(0) NUMBITS(1) []
],
ICR [
    /// RNGEIF
    RNGEIF OFFSET(3) NUMBITS(1) [],
    /// Key error interrupt flag clear
    KEIF OFFSET(2) NUMBITS(1) [],
    /// Read or write error interrupt flag clear
    RWEIF OFFSET(1) NUMBITS(1) [],
    /// Computation complete flag clear
    CCF OFFSET(0) NUMBITS(1) []
],
Data [
    DATA OFFSET(0)   NUMBITS(32) []
]
];
pub const SAES_BASE: StaticRef<SaesRegisters> =
    unsafe { StaticRef::new(0x520C0C00 as *const SaesRegisters) };

enum SAESMode {
    ECB,
    CBC,
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum DeferredOp {
    None,
    WriteIvx,
    Crypt(CryptoContext),
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct CryptoContext {
    start_index: usize,
    stop_index: usize,
    curr_index: usize,
    using_dma: bool,
}

#[derive(Copy, Clone, PartialEq, Debug)]
enum State {
    Idle,
    KeyPreparation(DeferredOp),
    Crypt(CryptoContext),
    KeyWrapping,
}

pub enum WrappedKeyType {
    DHUK,
    BHK,
    XORK,
}

// using both AESKeySize and KEY_LEN, even though AESKeySize has const LENGTH because the compiler
// doesn't recognize the LENGTH field as a const
pub struct SharedKey<K: AESKeySize, const KEY_LEN: usize> {
    key: [u8; KEY_LEN],
    key_type: WrappedKeyType,
    _phantom: PhantomData<K>,
}

impl<K: AESKeySize, const KEY_LEN: usize> SharedKey<K, KEY_LEN> {
    pub fn new(key: &[u8], key_type: WrappedKeyType) -> SharedKey<K, KEY_LEN> {
        assert_eq!(K::LENGTH, KEY_LEN);
        let mut wrapped_key_buf = [0; KEY_LEN];
        wrapped_key_buf.copy_from_slice(key);

        SharedKey {
            key: wrapped_key_buf,
            key_type: key_type,
            _phantom: PhantomData::<K>,
        }
    }

    fn set_lower_bytes(&mut self, key: &[u8]) {
        self.key[0..AES128_KEY_SIZE].copy_from_slice(&key[0..AES128_KEY_SIZE]);
    }

}

pub struct Saes<'a, K: AESKeySize, const KEY_LEN: usize> {
    registers: StaticRef<SaesRegisters>,
    mode: Cell<SAESMode>,
    state: Cell<State>,
    encrypting: Cell<bool>,
    client: OptionalCell<&'a dyn kernel::hil::symmetric_encryption::Client<'a>>,
    key_client: OptionalCell<&'a dyn WrappedKeyClient<K, KEY_LEN>>,
    input: TakeCell<'static, [u8]>,
    output: TakeCell<'static, [u8]>,
    iv: Cell<[u8; AES_IV_SIZE]>,
    wrapped_key: OptionalCell<SharedKey<K, KEY_LEN>>,
}

impl<'a, K: AESKeySize, const KEY_LEN: usize> Saes<'a, K, KEY_LEN> {
    pub fn new(base: StaticRef<SaesRegisters>) -> Saes<'a, K, KEY_LEN> {
        assert_eq!(K::LENGTH, KEY_LEN);
        Saes {
            registers: base,
            mode: Cell::new(SAESMode::ECB),
            encrypting: Cell::new(true),
            state: Cell::new(State::Idle),
            client: OptionalCell::empty(),
            key_client: OptionalCell::empty(),
            input: TakeCell::empty(),
            output: TakeCell::empty(),
            iv: Cell::new([0; AES_IV_SIZE]),
            wrapped_key: OptionalCell::empty(),
        }
    }

    fn apply_crypto_direction(&self, encrypting: bool) {
        self.encrypting.set(encrypting);
    }

    fn enable_interrupts(&self) {
        self.registers
            .ier
            .modify(IER::CCFIE::SET + IER::KEIE::SET + IER::RNGEIE::SET + IER::RWEIE::SET);
    }

    fn disable_interrupts(&self) {
        self.registers
            .ier
            .modify(IER::CCFIE::CLEAR + IER::KEIE::CLEAR + IER::RNGEIE::CLEAR + IER::RWEIE::CLEAR);
    }

    /// Helper to write a 128-bit or 256-bit key into the hardware key registers
    fn write_key_registers(&self, key: &[u8]) {
        // Default to using the first 16 bytes for the lower registers (AES-128 behavior)
        let mut lower_key_chunk = &key[0..16];

        if K::LENGTH == 32 {
            // AES-256: Write KEYR7 down to KEYR4 first
            for (reg, chunk) in self
                .registers
                .keyr2
                .iter()
                .rev()
                .zip(key[0..16].chunks_exact(4))
            {
                let word = u32::from_be_bytes(chunk.try_into().unwrap());
                reg.write(Data::DATA.val(word));
            }

            // Update the slice so the lower registers get the second half of the 256-bit key
            lower_key_chunk = &key[16..32];
        }

        // Write KEYR3 down to KEYR0
        for (reg, chunk) in self
            .registers
            .keyr
            .iter()
            .rev()
            .zip(lower_key_chunk.chunks_exact(4))
        {
            let word = u32::from_be_bytes(chunk.try_into().unwrap());
            reg.write(Data::DATA.val(word));
        }
    }

    /// Function for ECB and CBC decryption modes which goes though Key derivation operation
    fn prepare_decryption_key(&self, key: &[u8]) {
        self.state.set(State::KeyPreparation(DeferredOp::None));
        self.registers.cr.modify(CR::EN::CLEAR);
        self.registers.cr.modify(CR::MODE::KeyDerivation);

        self.write_key_registers(key);

        self.registers.cr.modify(CR::EN::SET);
    }

    /// Helper to write a AES_IV_SIZE-byte IV into the hardware IV registers
    fn write_iv_registers(&self, iv: &[u8; AES_IV_SIZE]) {
        for (reg, chunk) in self.registers.ivr.iter().rev().zip(iv.chunks_exact(4)) {
            let word = u32::from_be_bytes(chunk.try_into().expect("IV chunk len mismatch"));
            reg.write(Data::DATA.val(word));
        }
    }

    /// Writes a slice to the data input register. If the slice is smaller
    /// than AES_BLOCK_SIZE bytes, it automatically pads the remainder with zeros.
    fn write_padded_to_dinr(&self, slice: &[u8]) {
        let mut buf = [0u8; AES_BLOCK_SIZE];
        buf[..slice.len()].copy_from_slice(slice);

        for chunk in buf.chunks_exact(4) {
            let word = u32::from_le_bytes(chunk.try_into().unwrap());
            self.registers.dinr.set(word);
        }
    }

    /// writes AES_BLOCK_SIZE bytes to DINR register starting at ctx.current_idx + ctx.message_start
    /// from dest buffer if operation is in-place, or at ctx.current_idx in the input buffer
    fn write_input(&self, ctx: CryptoContext) {
        let current_idx = ctx.curr_index;
        if self.input.is_some() {
            // Out-of-place
            self.input.map(|buf| {
                self.write_padded_to_dinr(&buf[current_idx..current_idx + AES_BLOCK_SIZE])
            });
        } else {
            // In-place
            let offset = ctx.start_index + current_idx;
            self.output
                .map(|buf| self.write_padded_to_dinr(&buf[offset..offset + AES_BLOCK_SIZE]));
        }
    }

    /// returns AES_BLOCK_SIZE bytes of data from the DOUTR register
    fn get_output(&self) -> [u8; AES_BLOCK_SIZE] {
        let mut block = [0u8; AES_BLOCK_SIZE];
        for chunk in block.chunks_exact_mut(4) {
            let word = self.registers.doutr.get();
            chunk.copy_from_slice(&word.to_le_bytes());
        }
        block
    }

    fn start_crypt(&self, ctx: CryptoContext) {
        self.state.set(State::Crypt(ctx));
        if !self.registers.cr.any_matching_bits_set(CR::EN::SET) {
            self.registers.cr.modify(CR::EN::SET);
        }
        self.write_input(ctx);
    }

    fn computation_complete(&self) {
        match self.state.get() {
            State::KeyPreparation(deferred_op) => {
                self.registers.cr.modify(CR::EN::CLEAR);
                self.registers.cr.modify(CR::MODE::Decrypt);
                match deferred_op {
                    DeferredOp::None => self.state.set(State::Idle),
                    DeferredOp::WriteIvx => {
                        self.write_iv_registers(&self.iv.get());
                        self.iv.set([0; AES_IV_SIZE]);
                        self.state.set(State::Idle);
                    }
                    DeferredOp::Crypt(ctx) => {
                        self.write_iv_registers(&self.iv.get());
                        self.iv.set([0; AES_IV_SIZE]);
                        self.start_crypt(ctx);
                    }
                }
            }
            State::Crypt(mut ctx) => {
                if ctx.using_dma {
                    return;
                }
                let start_idx = ctx.start_index;
                let end_idx = ctx.stop_index;
                let offset = start_idx + ctx.curr_index;

                let block = self.get_output();
                self.output.map(|output| {
                    output[offset..offset + AES_BLOCK_SIZE].copy_from_slice(&block);
                });
                ctx.curr_index += AES_BLOCK_SIZE;

                // if encoding is finished, return the buffer to the client
                if start_idx + ctx.curr_index >= end_idx {
                    self.state.set(State::Idle);
                    self.output.take().map(|output| {
                        self.client
                            .map(move |client| client.crypt_done(self.input.take(), output));
                    });
                } else {
                    self.state.set(State::Crypt(ctx));
                    self.write_input(ctx);
                }
            }
            State::KeyWrapping => {

                match KEY_LEN {
                    16 =>

                }
            }
            _ => {}
        }
    }

    pub fn handle_interrupt(&self) {
        if self.registers.isr.is_set(ISR::CCF) {
            self.registers.icr.write(ICR::CCF::SET);
            self.computation_complete();
        }

        if self.registers.isr.is_set(ISR::RWEIF) {
            self.registers.icr.write(ICR::RWEIF::SET);
        }

        if self.registers.isr.is_set(ISR::KEIF) {
            self.registers.icr.write(ICR::KEIF::SET);
        }

        if self.registers.isr.is_set(ISR::RNGEIF) {
            self.registers.icr.write(ICR::RNGEIF::SET);
        }
    }
}

impl<'a, K: AESKeySize, const KEY_LEN: usize> kernel::hil::symmetric_encryption::AES<'a, K>
    for Saes<'a, K, KEY_LEN>
{
    fn enable(&self) {
        self.registers.cr.modify(CR::IPRST::SET);
        self.registers.cr.write(CR::EN::CLEAR);
        self.registers.cr.modify(CR::DATATYPE::Byte);
        self.state.set(State::Idle);
        self.enable_interrupts();
    }

    fn disable(&self) {
        self.registers.cr.write(CR::EN::CLEAR);
        self.disable_interrupts();
        self.registers.cr.modify(CR::IPRST::SET);
        self.state.set(State::Idle);
    }

    fn set_client(&'a self, client: &'a dyn kernel::hil::symmetric_encryption::Client<'a>) {
        self.client.set(client);
    }

    fn set_key(&self, key: &[u8]) -> Result<(), ErrorCode> {
        if key.len() != K::LENGTH {
            return Err(ErrorCode::INVAL);
        }

        if K::LENGTH == 16 {
            self.registers.cr.modify(CR::KEYSIZE::AES128);
        } else {
            self.registers.cr.modify(CR::KEYSIZE::AES256);
        }

        if self.registers.cr.any_matching_bits_set(CR::EN::SET)
            || self.registers.sr.any_matching_bits_set(SR::BUSY::SET)
        {
            return Err(ErrorCode::BUSY);
        }
        self.registers.cr.modify(CR::KEYSEL::SOFTWARE);
        self.registers.cr.modify(CR::KEYPROT::ALLOW_TRASNFER);
        self.registers.cr.modify(CR::KMOD::NORMAL);

        if self.encrypting.get() {
            self.prepare_decryption_key(key);
        } else {
            self.write_key_registers(key);
        }

        Ok(())
    }

    fn set_iv(&self, iv: &[u8]) -> Result<(), ErrorCode> {
        if iv.len() != AES_IV_SIZE {
            return Err(ErrorCode::INVAL);
        }

        if self.registers.cr.any_matching_bits_set(CR::EN::SET) {
            return Err(ErrorCode::BUSY);
        }

        match self.state.get() {
            State::Idle => self.write_iv_registers(iv.try_into().unwrap()),
            State::KeyPreparation(_) => {
                self.iv.set(iv.try_into().unwrap());
                self.state.set(State::KeyPreparation(DeferredOp::WriteIvx));
            }
            _ => return Err(ErrorCode::BUSY),
        }

        Ok(())
    }

    fn start_message(&self) {}

    fn crypt(
        &self,
        source: Option<&'static mut [u8]>,
        dest: &'static mut [u8],
        start_index: usize,
        stop_index: usize,
    ) -> Option<(
        Result<(), ErrorCode>,
        Option<&'static mut [u8]>,
        &'static mut [u8],
    )> {
        let state = self.state.get();

        //  Hardware busy check
        if self.output.is_some() || !matches!(state, State::Idle | State::KeyPreparation(_)) {
            return Some((Err(ErrorCode::BUSY), source, dest));
        }

        // Bounds checking
        if start_index >= stop_index || !(stop_index - start_index).is_multiple_of(AES_BLOCK_SIZE) {
            return Some((Err(ErrorCode::INVAL), source, dest));
        }

        if let Some(src) = &source {
            if src.len() < stop_index - start_index {
                return Some((Err(ErrorCode::INVAL), source, dest));
            }
        }
        if dest.len() < stop_index {
            return Some((Err(ErrorCode::INVAL), source, dest));
        }

        if let Some(src) = source {
            self.input.replace(src);
        }
        self.output.replace(dest);

        let ctx = CryptoContext {
            start_index,
            stop_index,
            curr_index: 0,
            using_dma: false,
        };

        if let State::KeyPreparation(_) = state {
            self.state
                .set(State::KeyPreparation(DeferredOp::Crypt(ctx)));
            return None;
        }

        self.start_crypt(ctx);

        None
    }
}

impl<K: AESKeySize, const KEY_LEN: usize> kernel::hil::symmetric_encryption::AESECB
    for Saes<'_, K, KEY_LEN>
{
    fn set_mode_aesecb(&self, encrypting: bool) -> Result<(), ErrorCode> {
        self.mode.set(SAESMode::ECB);
        self.registers.cr.modify(CR::CHMOD::ECB);
        self.apply_crypto_direction(encrypting);
        Ok(())
    }
}

impl<K: AESKeySize, const KEY_LEN: usize> kernel::hil::symmetric_encryption::AESCBC
    for Saes<'_, K, KEY_LEN>
{
    fn set_mode_aescbc(&self, encrypting: bool) -> Result<(), ErrorCode> {
        self.mode.set(SAESMode::CBC);
        self.registers.cr.modify(CR::CHMOD::CBC);
        self.apply_crypto_direction(encrypting);
        Ok(())
    }
}

pub fn set_iv() -> Result<(), ErrorCode> {

}

pub fn wrap_key(&self, key: &mut [u8], option_iv: Option<&[u8]>) -> Result<(), ErrorCode> {
    let regs = self.registers;
    if regs.sr.any_matching_bits_set(SR::BUSY::SET)
        || regs.cr.any_matching_bits_set(CR::EN::SET)
    {
        return Err(ErrorCode::BUSY);
    }

    if self.state.get() != State::Idle {
        return Err(ErrorCode::BUSY);
    }

    match K::LENGTH {
        16 => {
            regs.cr.modify(CR::KEYSIZE::AES128);
        }
        32 => {
            regs.cr.modify(CR::KEYSIZE::AES256);
        }
        _ => return Err(ErrorCode::INVAL),
    }

    self.state.set(State::KeyWrapping);
    regs.cr
        .modify(CR::MODE::Encrypt + CR::KMOD::WRAPPED + CR::KEYSEL::DHUK);

    if let Some(iv) = option_iv {
        if iv.len() != AES_IV_SIZE {
            self.state.set(State::Idle);
            return Err(ErrorCode::INVAL);
        }
        regs.cr.modify(CR::CHMOD::CBC);
        self.write_iv_registers(iv.try_into().unwrap());
    } else {
        regs.cr.modify(CR::CHMOD::ECB);
    }
    regs.cr.modify(CR::EN::SET);

    self.write_padded_to_dinr(&key[0..AES128_KEY_SIZE]);

    self.wrapped_key
        .set(SharedKey::new(key, WrappedKeyType::DHUK));

    Ok(())
}

pub fn set_client(&'a self, client: &'a dyn WrappedKeyClient<K, KEY_LEN>) {
    self.key_client.set(client);
}
