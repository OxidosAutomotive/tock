use core::cell::Cell;
use kernel::utilities::cells::{TakeCell, VolatileCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::ErrorCode;

register_structs! {
    /// FLASH
    FlashRegisters {
        /// Flash access control register
        (0x000 => acr: ReadWrite<u32, ACR::Register>),
        /// Flash key register
        (0x004 => keyr: WriteOnly<u32>),
        /// Flash option key register
        (0x008 => optkeyr: WriteOnly<u32>),
        /// Status register
        (0x00C => sr: ReadWrite<u32, SR::Register>),
        /// Control register
        (0x010 => cr: ReadWrite<u32, CR::Register>),
        /// Flash option control register
        (0x014 => optcr: ReadWrite<u32, OPTCR::Register>),
        /// Flash option control register 1
        (0x018 => optcr1: ReadWrite<u32>),
        (0x01C => @END),
    }
}

register_bitfields![u32,
    ACR [
        /// Latency - might be wrong, as the reference
        /// manual shows 4 bits and the SVD file shows 3cd-
        LATENCY OFFSET(0) NUMBITS(4) [],
        /// Prefetch enable
        PRFTEN OFFSET(8) NUMBITS(1) [],
        /// Instruction cache enable
        ICEN OFFSET(9) NUMBITS(1) [],
        /// Data cache enable
        DCEN OFFSET(10) NUMBITS(1) [],
        /// Instruction cache reset
        ICRST OFFSET(11) NUMBITS(1) [],
        /// Data cache reset
        DCRST OFFSET(12) NUMBITS(1) []
    ],
    KEYR [
        /// FPEC key
        KEY OFFSET(0) NUMBITS(32) []
    ],
    OPTKEYR [
        /// Option byte key
        OPTKEY OFFSET(0) NUMBITS(32) []
    ],
    SR [
        /// End of operation
        EOP OFFSET(0) NUMBITS(1) [],
        /// Operation error
        OPERR OFFSET(1) NUMBITS(1) [],
        /// Write protection error
        WRPERR OFFSET(4) NUMBITS(1) [],
        /// Programming alignment error
        PGAERR OFFSET(5) NUMBITS(1) [],
        /// Programming parallelism error
        PGPERR OFFSET(6) NUMBITS(1) [],
        /// Programming sequence error
        PGSERR OFFSET(7) NUMBITS(1) [],
        /// Proprietary readout protection error
        RDERR OFFSET(8) NUMBITS(1) [],
        /// Busy
        BSY OFFSET(16) NUMBITS(1) []
    ],
    CR [
        /// Programming
        PG OFFSET(0) NUMBITS(1) [],
        /// Sector Erase
        SER OFFSET(1) NUMBITS(1) [],
        /// Mass Erase of sectors 0 to 11
        MER OFFSET(2) NUMBITS(1) [],
        /// Sector number
        SNB OFFSET(3) NUMBITS(5) [],
        /// Program size
        PSIZE OFFSET(8) NUMBITS(2) [],
        /// Mass Erase of sectors 12 to 23
        MER1 OFFSET(15) NUMBITS(1) [],
        /// Start
        STRT OFFSET(16) NUMBITS(1) [],
        /// End of operation interrupt enable
        EOPIE OFFSET(24) NUMBITS(1) [],
        /// Error interrupt enable
        ERRIE OFFSET(25) NUMBITS(1) [],
        /// Lock
        LOCK OFFSET(31) NUMBITS(1) []
    ],
    OPTCR [
        /// Option lock
        OPTLOCK OFFSET(0) NUMBITS(1) [],
        /// Option start
        OPTSTRT OFFSET(1) NUMBITS(1) [],
        /// BOR reset Level
        BOR_LEV OFFSET(2) NUMBITS(2) [],
        /// WDG_SW User option bytes
        WDG_SW OFFSET(5) NUMBITS(1) [],
        /// nRST_STOP User option bytes
        nRST_STOP OFFSET(6) NUMBITS(1) [],
        /// nRST_STDBY User option bytes
        nRST_STDBY OFFSET(7) NUMBITS(1) [],
        /// Read protect
        RDP OFFSET(8) NUMBITS(8) [],
        /// Not write protect
        nWRP OFFSET(16) NUMBITS(12) [],
        /// Selection of protection mode for nWPRi bits
        SPRMOD OFFSET(31) NUMBITS(1) [],
    ],
    OPTCR1 [
        /// Not write protect
        nWRP OFFSET(16) NUMBITS(12) []
    ]
];

const FLASH_BASE: StaticRef<FlashRegisters> =
    unsafe { StaticRef::new(0x40023C00 as *const FlashRegisters) };

const SECTOR_MASK: u32 = 0x10;
const UNLOCK_KEY1: u32 = 0x45670123;
const UNLOCK_KEY2: u32 = 0xCDEF89AB;

#[derive(Copy, Clone, Debug)]
pub enum ProgramAccess {
    Byte = 0,
    HalfWord = 1,
    Word = 2,
    DoubleWord = 3,
}

#[derive(Copy, Clone, Debug)]
pub enum FlashInterrupt {
    EndOfOperation,
    ErrorInterrupt,
}

#[derive(Copy, Clone, Debug)]
pub enum State {
    Idle,
    Reading(u32),
    SectorErase(u32),
    BankErase(u32),
    MassErase,
    Programming,
}
pub struct Flash {
    registers: StaticRef<FlashRegisters>,
    flash_state: Cell<State>,
    program_access: Cell<ProgramAccess>,
    program_buffer: TakeCell<'static, [u8; 16384]>,
    program_data: Cell<(usize, usize, usize)>, // (current index, buffer len, address)
}

impl Flash {
    pub fn new() -> Flash {
        Flash {
            registers: FLASH_BASE,
            flash_state: Cell::new(State::Idle),
            program_access: Cell::new(ProgramAccess::Byte),
            program_buffer: TakeCell::empty(),
            program_data: Cell::new((0, 0, 0)),
        }
    }

    // fn wait_for(times: usize, f: impl Fn() -> bool) -> bool {
    //     for _ in 0..times {
    //         if f() {
    //             return true;
    //         }
    //     }

    //     false
    // }

    pub fn init(&self) -> Result<(), ErrorCode> {
        self.enable_interrupts();
        if self.is_locked() {
            if self.unlock_control_register() {
                return Ok(());
            } else {
                return Err(ErrorCode::ALREADY);
            }
        }
        Err(ErrorCode::ALREADY)
    }

    pub fn is_locked(&self) -> bool {
        self.registers.cr.is_set(CR::LOCK)
    }

    pub fn enable_interrupt(&self, interrupt: FlashInterrupt) {
        match interrupt {
            FlashInterrupt::EndOfOperation => self.registers.cr.modify(CR::EOPIE::SET),
            FlashInterrupt::ErrorInterrupt => self.registers.cr.modify(CR::ERRIE::SET),
        }
    }

    pub fn enable_interrupts(&self) {
        self.enable_interrupt(FlashInterrupt::EndOfOperation);
        self.enable_interrupt(FlashInterrupt::ErrorInterrupt);
    }

    pub fn sector_erase(&self, sector: u32) -> Result<(), ErrorCode> {
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err(ErrorCode::BUSY);
            }
        }
        match self.flash_state.get() {
            State::Idle => {
                self.flash_state.set(State::SectorErase(sector));
                if sector >= 12 && sector <= 16 {
                    self.registers.cr.modify(CR::SER::SET);
                    self.registers
                        .cr
                        .modify(CR::SNB.val(SECTOR_MASK | (sector - 12)));

                    self.registers.cr.modify(CR::STRT::SET);
                } else {
                    return Err(ErrorCode::INVAL);
                }
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    pub fn bank_erase(&self, bank: u32) -> Result<(), ErrorCode> {
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err(ErrorCode::BUSY);
            }
        }
        match self.flash_state.get() {
            State::Idle => {
                self.flash_state.set(State::BankErase(bank));
                if bank == 0 {
                    self.registers.cr.modify(CR::MER::SET);
                } else if bank == 1 {
                    self.registers.cr.modify(CR::MER1::SET);
                } else {
                    self.flash_state.set(State::Idle);
                    return Err(ErrorCode::INVAL);
                }
                self.registers.cr.modify(CR::STRT::SET);
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    pub fn mass_erase(&self) -> Result<(), ErrorCode> {
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err(ErrorCode::BUSY);
            }
        }
        match self.flash_state.get() {
            State::Idle => {
                self.flash_state.set(State::MassErase);
                self.registers.cr.modify(CR::MER::SET);
                self.registers.cr.modify(CR::MER1::SET);

                self.registers.cr.modify(CR::STRT::SET);

                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    pub fn program_flash(&self) -> Result<(), ErrorCode> {
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err(ErrorCode::BUSY);
            }
        }
        match self.flash_state.get() {
            State::Idle | State::Programming => {
                self.flash_state.set(State::Programming);
                self.registers.cr.modify(CR::PG::SET);

                self.program_buffer.map(|buffer| {
                    let (index, _, address) = self.program_data.get();
                    match self.program_access.get() {
                        ProgramAccess::Byte => {
                            let byte: u8 = buffer[index];
                            let program_address =
                                unsafe { &*((address + index) as *const VolatileCell<u8>) };
                            program_address.set(byte);
                        }
                        ProgramAccess::HalfWord => {
                            let halfword: u16 =
                                (buffer[index] as u16) | (buffer[index + 1] as u16) << 8;
                            let program_address =
                                unsafe { &*((address + index) as *const VolatileCell<u16>) };
                            program_address.set(halfword);
                        }
                        ProgramAccess::Word => {
                            let word: u32 = (buffer[index] as u32) << 16
                                | (buffer[index + 1] as u32) << 24
                                | (buffer[index + 2] as u32)
                                | (buffer[index + 3] as u32) << 8;
                            let program_address =
                                unsafe { &*((address + index) as *const VolatileCell<u32>) };
                            program_address.set(word);
                        }
                        ProgramAccess::DoubleWord => {
                            let doubleword: u64 = (buffer[index] as u64) << 48
                                | (buffer[index + 1] as u64) << 56
                                | (buffer[index + 2] as u64) << 32
                                | (buffer[index + 3] as u64) << 40
                                | (buffer[index + 4] as u64) << 16
                                | (buffer[index + 5] as u64) << 24
                                | (buffer[index + 6] as u64)
                                | (buffer[index + 7] as u64) << 8;
                            let program_address =
                                unsafe { &*((address + index) as *const VolatileCell<u64>) };
                            program_address.set(doubleword);
                        }
                    }
                });
                Ok(())
            }
            _ => Err(ErrorCode::BUSY),
        }
    }

    pub fn unlock_control_register(&self) -> bool {
        if !self.registers.sr.is_set(SR::BSY) {
            self.registers.keyr.set(UNLOCK_KEY1);
            self.registers.keyr.set(UNLOCK_KEY2);
            return true;
        }

        false
    }

    pub fn handle_interrupt(&self) {
        if self.registers.sr.is_set(SR::EOP) {
            // handle the interrupt
            self.registers.sr.modify(SR::EOP::SET);

            // End of Operation
            match self.flash_state.get() {
                State::Idle => {}
                State::Reading(_) => {}
                State::SectorErase(_sector) => {
                    // todo send to the client confirmation that the sector erase is done
                    if self.registers.cr.is_set(CR::SER) {
                        self.registers.cr.modify(CR::SER::CLEAR);
                    }
                    self.flash_state.set(State::Idle);
                }
                State::BankErase(bank) => {
                    // todo send to the client confirmation that the bank erase is done
                    if bank == 0 {
                        if self.registers.cr.is_set(CR::MER) {
                            self.registers.cr.modify(CR::MER::CLEAR);
                        }
                    }
                    if bank == 1 {
                        if self.registers.cr.is_set(CR::MER1) {
                            self.registers.cr.modify(CR::MER1::CLEAR);
                        }
                    }
                    self.flash_state.set(State::Idle);
                }
                State::MassErase => {
                    // todo send to the client confirmation that the mass erase is done
                    if self.registers.cr.is_set(CR::MER) {
                        self.registers.cr.modify(CR::MER::CLEAR);
                    }
                    if self.registers.cr.is_set(CR::MER1) {
                        self.registers.cr.modify(CR::MER1::CLEAR);
                    }
                    self.flash_state.set(State::Idle);
                }
                State::Programming => {
                    self.program_data.replace((
                        self.program_data.get().0 + 2 ^ (self.program_access.get() as usize),
                        self.program_data.get().1,
                        self.program_data.get().2,
                    ));

                    let (index, len, _) = self.program_data.get();
                    if index == len {
                        self.program_data.replace((0, 0, 0));
                        self.flash_state.set(State::Idle);
                        // todo send upcall that the programming command is done
                    } else {
                        if !self.program_flash().is_ok() {
                            // todo send upcall that something failed
                        }
                    }
                }
            }
        }

        if self.registers.sr.is_set(SR::OPERR) {
            if self.registers.sr.is_set(SR::WRPERR) {
                // write protection error handle interrupt
                self.registers.sr.modify(SR::WRPERR::SET);
            }

            if self.registers.sr.is_set(SR::RDERR) {
                // read protection error
                self.registers.sr.modify(SR::RDERR::SET);
            }

            if self.registers.sr.is_set(SR::PGAERR) {
                // programming alignment error
                self.registers.sr.modify(SR::PGAERR::SET);
            }

            if self.registers.sr.is_set(SR::PGPERR) {
                // programming parallelism error
                self.registers.sr.modify(SR::PGPERR::SET);
            }

            if self.registers.sr.is_set(SR::PGSERR) {
                // programming sequence error
                self.registers.sr.modify(SR::PGSERR::SET);
            }
        }
    }
}
