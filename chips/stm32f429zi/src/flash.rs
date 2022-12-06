use core::cell::Cell;
use kernel::deferred_call::DeferredCall;
use kernel::utilities::cells::{OptionalCell, TakeCell, VolatileCell};
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable, Writeable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadWrite, WriteOnly};
use kernel::utilities::StaticRef;
use kernel::{debug, ErrorCode};
use stm32f4xx::deferred_calls::DeferredCallTask;

use kernel::hil::flash::{Client, Error, Flash, HasClient};

register_structs! {
    /// Flash
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

static DEFERRED_CALL: DeferredCall<DeferredCallTask> =
    unsafe { DeferredCall::new(DeferredCallTask::Flash) };

const SECTOR_PADDING: u32 = 0b10000;
const UNLOCK_KEY1: u32 = 0x45670123;
const UNLOCK_KEY2: u32 = 0xCDEF89AB;

const SECTOR_START: usize = 0x08100000;
pub const SECTOR_SIZE: usize = 0x4000;
// const S: usize = 32;
pub const SECTOR_COUNT: usize = 4;

// pub static mut FLASH_BUFFER: [u8; SECTOR_SIZE] = [0; SECTOR_SIZE];
// pub static mut READ_BUFFER: [u8; SECTOR_SIZE] = [0; SECTOR_SIZE];

#[derive(Debug)]
pub struct Sector(pub [u8; SECTOR_SIZE]);

impl Sector {
    fn len(&self) -> usize {
        self.0.len()
    }
}

impl Default for Sector {
    fn default() -> Self {
        Self {
            0: [0; SECTOR_SIZE],
        }
    }
}

impl AsMut<[u8]> for Sector {
    fn as_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}

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
    Reading(usize),
    SectorErase(usize),
    BankErase(usize),
    MassErase,
    Programming,
}
pub struct FlashDevice {
    registers: StaticRef<FlashRegisters>,
    flash_client: OptionalCell<&'static dyn Client<FlashDevice>>,
    flash_state: Cell<State>,
    program_access: Cell<ProgramAccess>,
    program_buffer: TakeCell<'static, Sector>,
    read_buffer: TakeCell<'static, Sector>,
    program_data: Cell<(usize, usize, usize)>, // (current index, buffer len, page_number)
}

impl FlashDevice {
    pub fn new() -> FlashDevice {
        FlashDevice {
            registers: FLASH_BASE,
            flash_state: Cell::new(State::Idle),
            program_access: Cell::new(ProgramAccess::Word),
            program_buffer: TakeCell::empty(),
            read_buffer: TakeCell::empty(),
            program_data: Cell::new((0, 0, 0)),
            flash_client: OptionalCell::empty(),
        }
    }

    pub fn init(&self) -> Result<(), ErrorCode> {
        self.enable_interrupts();
        if self.is_locked() {
            if self.unlock_control_register() {
                // self.registers.acr.modify(ACR::DCEN::SET);
                // self.registers.acr.modify(ACR::ICEN::SET);
                // self.registers.acr.modify(ACR::PRFTEN::SET);
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

    pub fn read_sector(&self, sector: usize, buf: &'static mut Sector) {
        self.flash_state.set(State::Reading(sector));
        // debug!("TEONA setam state la reading {}", sector);
        let mut byte: *const u8 = (SECTOR_START + sector * SECTOR_SIZE) as *const u8;
        // debug!("TEONA pornim de la adresa {:x} pana la {:x}", (SECTOR_START + sector * SECTOR_SIZE), (SECTOR_START + sector * SECTOR_SIZE) + buf.0.len());
        unsafe {
            for i in 0..buf.len() {
                buf.0[i] = *byte;
                byte = byte.offset(1);
            }
        }

        self.read_buffer.replace(buf);
        DEFERRED_CALL.set();
    }

    pub fn sector_erase(&self, sector: usize) {
        debug!(
            "The value of acr latency is {}",
            self.registers.acr.read(ACR::LATENCY)
        );
        if !self.registers.sr.is_set(SR::BSY) {
            self.flash_state.set(State::SectorErase(sector));
            let val: u32 = SECTOR_PADDING + (sector as u32);
            debug!(
                "[DRIVER] setam state la erase {} si sectorul {}, addresa {:x}, psize este {}",
                sector,
                val,
                SECTOR_START + sector * SECTOR_SIZE,
                self.program_access.get() as u32
            );
            self.registers.cr.modify(
                CR::PSIZE.val(self.program_access.get() as u32) + CR::SNB.val(val) + CR::SER::SET,
            );
            self.registers.cr.modify(CR::STRT::SET);
        }
    }

    pub fn bank_erase(&self, bank: usize) {
        self.flash_state.set(State::BankErase(bank));
        // debug!("TEONA setam state la erase bank {}", bank);
        if bank == 0 {
            self.registers.cr.modify(CR::MER::SET);
        } else {
            self.registers.cr.modify(CR::MER1::SET);
        }
        self.registers.cr.modify(CR::STRT::SET);
    }

    pub fn mass_erase(&self) {
        self.flash_state.set(State::MassErase);
        // debug!("TEONA setam state la mass erase");
        self.registers.cr.modify(CR::MER::SET);
        self.registers.cr.modify(CR::MER1::SET);

        self.registers.cr.modify(CR::STRT::SET);
    }

    pub fn program_flash(&self) {
        self.flash_state.set(State::Programming);
        // debug!("TEONA setam state la programming");
        self.registers
            .cr
            .modify(CR::PSIZE.val(self.program_access.get() as u32));
        self.registers.cr.modify(CR::PG::SET);

        self.program_buffer.map(|buffer| {
            let (index, _, page_number) = self.program_data.get();
            let address = SECTOR_START + page_number * SECTOR_SIZE;
            match self.program_access.get() {
                ProgramAccess::Byte => {
                    let byte: u8 = buffer.0[index];
                    let program_address =
                        unsafe { &*((address + index) as *const VolatileCell<u8>) };
                    program_address.set(byte);
                }
                ProgramAccess::HalfWord => {
                    let halfword: u16 =
                        (buffer.0[index] as u16) | (buffer.0[index + 1] as u16) << 8;
                    let program_address =
                        unsafe { &*((address + index) as *const VolatileCell<u16>) };
                    program_address.set(halfword);
                }
                ProgramAccess::Word => {
                    let word: u32 = (buffer.0[index] as u32) << 16
                        | (buffer.0[index + 1] as u32) << 24
                        | (buffer.0[index + 2] as u32)
                        | (buffer.0[index + 3] as u32) << 8;
                    let program_address =
                        unsafe { &*((address + index) as *const VolatileCell<u32>) };
                    program_address.set(word);
                }
                ProgramAccess::DoubleWord => {
                    let doubleword: u64 = (buffer.0[index] as u64) << 48
                        | (buffer.0[index + 1] as u64) << 56
                        | (buffer.0[index + 2] as u64) << 32
                        | (buffer.0[index + 3] as u64) << 40
                        | (buffer.0[index + 4] as u64) << 16
                        | (buffer.0[index + 5] as u64) << 24
                        | (buffer.0[index + 6] as u64)
                        | (buffer.0[index + 7] as u64) << 8;
                    let program_address =
                        unsafe { &*((address + index) as *const VolatileCell<u64>) };
                    program_address.set(doubleword);
                }
            }
        });
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
        debug!(
            "[DRIVER] handle interrupt chips {:?} {:x} {:x} {:x}",
            self.flash_state.get(),
            self.registers.optcr.get(),
            self.registers.sr.get(),
            self.registers.cr.get()
        );

        if self.registers.sr.is_set(SR::EOP) {
            // debug!("We have end of operation");
            // handle the interrupt
            self.registers.sr.modify(SR::EOP::SET);

            // End of Operation
            match self.flash_state.get() {
                State::SectorErase(_sector) => {
                    // debug!("[DRIVER] sector erased {}", _sector);
                    self.registers
                        .cr
                        .modify(CR::SNB.val(0) + CR::STRT::CLEAR + CR::SER::CLEAR);

                    // debug!("TEONA setam state la idle 1");
                    self.flash_state.set(State::Idle);
                    self.flash_client.map(|client| {
                        client.erase_complete(Error::CommandComplete);
                    });
                }
                State::BankErase(bank) => {
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
                    // debug!("TEONA setam state la idle 2");
                    self.flash_state.set(State::Idle);
                    self.flash_client.map(|client| {
                        client.erase_complete(Error::CommandComplete);
                    });
                }
                State::MassErase => {
                    if self.registers.cr.is_set(CR::MER) {
                        self.registers.cr.modify(CR::MER::CLEAR);
                    }
                    if self.registers.cr.is_set(CR::MER1) {
                        self.registers.cr.modify(CR::MER1::CLEAR);
                    }
                    // debug!("TEONA setam state la idle 3");
                    self.flash_state.set(State::Idle);
                    self.flash_client.map(|client| {
                        client.erase_complete(Error::CommandComplete);
                    });
                }
                State::Programming => {
                    self.program_data.replace((
                        self.program_data.get().0 + 2 ^ (self.program_access.get() as usize),
                        self.program_data.get().1,
                        self.program_data.get().2,
                    ));

                    let (index, len, _) = self.program_data.get();
                    if index == len {
                        self.registers.cr.modify(CR::PG::CLEAR);
                        self.program_data.replace((0, 0, 0));
                        // debug!("TEONA setam state la idle 4");
                        self.flash_state.set(State::Idle);
                        self.flash_client.map(|client| {
                            self.program_buffer.take().map(|buffer| {
                                client.write_complete(buffer, Error::CommandComplete);
                            });
                        });
                    } else {
                        self.program_flash();
                    }
                }
                _ => {}
            }
        }

        if let State::Reading(_sector) = self.flash_state.get() {
            // todo send to the client confirmation that the data was read
            // todo data is in self.read_buffer
            self.flash_state.set(State::Idle);
            // panic!("TEONA setam state la idle 5");
            self.flash_client.map(|client| {
                // panic!("suntem in client");
                self.read_buffer.take().map(|buffer| {
                    // panic!("suntem la read_complete");
                    client.read_complete(buffer, Error::CommandComplete);
                    // panic!("totul ok?");
                });
            });
            // panic!("era reading {}", _sector);
        }

        if self.registers.sr.is_set(SR::OPERR) {
            debug!("[DRIVER] We have an error");
            if self.registers.sr.is_set(SR::WRPERR) {
                // write protection error handle interrupt
                self.registers.sr.modify(SR::WRPERR::SET);
                debug!("[DRIVER] write protection");
            }

            if self.registers.sr.is_set(SR::RDERR) {
                // read protection error
                self.registers.sr.modify(SR::RDERR::SET);
                debug!("[DRIVER] read protection");
            }

            if self.registers.sr.is_set(SR::PGAERR) {
                // programming alignment error
                self.registers.sr.modify(SR::PGAERR::SET);
                debug!("[DRIVER] programming alignment error");
            }

            if self.registers.sr.is_set(SR::PGPERR) {
                // programming parallelism error
                self.registers.sr.modify(SR::PGPERR::SET);
                debug!("[DRIVER] programming parallelism error");
                self.registers.cr.modify(CR::PG::CLEAR);
            }

            if self.registers.sr.is_set(SR::PGSERR) {
                // programming sequence error
                self.registers.sr.modify(SR::PGSERR::SET);
                debug!("[DRIVER] programming sequence error");
            }
        }
    }
}

impl Flash for FlashDevice {
    type Page = Sector;

    fn read_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        // debug!(
        //     "[DRIVER] read_page in chips {} la adresa {:x} cu {}",
        //     page_number,
        //     SECTOR_START + page_number * SECTOR_SIZE,
        //     buf.len()
        // );
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err((ErrorCode::OFF, buf));
            }
        }
        if page_number <= 3 {
            // debug!("TEONA page_number ok");
            // todo what to do if offset + buf.len() exceed the sector and go into the next region - out of region
            self.read_sector(page_number, buf);
            Ok(())
        } else {
            // debug!("TEONA send back error");
            Err((ErrorCode::INVAL, buf))
        }
    }

    fn write_page(
        &self,
        page_number: usize,
        buf: &'static mut Self::Page,
    ) -> Result<(), (ErrorCode, &'static mut Self::Page)> {
        // debug!(
        //     "[DRIVER] write_page in chips {} la adresa {:x}",
        //     page_number,
        //     SECTOR_START + page_number * SECTOR_SIZE
        // );
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err((ErrorCode::OFF, buf));
            }
        }
        match self.flash_state.get() {
            State::Idle => {
                self.program_data.set((0, buf.0.len(), page_number));
                self.program_buffer.replace(buf);

                self.program_flash();

                // debug!("TEONA started flash");
                Ok(())
            }
            _ => {
                // debug!("TEONA send back busy");
                Err((ErrorCode::BUSY, buf))
            }
        }
    }

    fn erase_page(&self, page_number: usize) -> Result<(), ErrorCode> {
        debug!("[DRIVER] erase_page in chips {}", page_number);
        // panic!("erase page {}", page_number);
        if self.is_locked() {
            if !self.unlock_control_register() {
                return Err(ErrorCode::OFF);
            }
        }

        if page_number <= 3 {
            match self.flash_state.get() {
                State::Idle => {
                    self.sector_erase(page_number);
                    debug!("sending ok in erase page");
                    Ok(())
                }
                _ => {
                    debug!("sending busy in erase page");
                    Err(ErrorCode::BUSY)
                }
            }
        } else {
            debug!("sending inval in erase page");
            Err(ErrorCode::INVAL)
        }
    }
}

impl<C: Client<Self>> HasClient<'static, C> for FlashDevice {
    fn set_client(&self, client: &'static C) {
        debug!("[DRIVER] set_client in chips");
        self.flash_client.set(client);
    }
}
