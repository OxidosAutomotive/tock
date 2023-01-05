use kernel::{
    debug,
    hil::{
        screen,
        time::{self, Alarm},
    },
    utilities::cells::{OptionalCell, TakeCell},
    ErrorCode,
};

use crate::{
    bus::{self, Bus, BusWidth},
    screen::Screen,
    st77xx::SendCommand,
};
use core::cell::Cell;

pub const SLAVE_ADDRESS_WRITE: u8 = 0b0111100;
pub const SLAVE_ADDRESS_READ: u8 = 0b0111101;
pub const WIDTH: usize = 128;
pub const HEIGHT: u8 = 64;
pub const I2C_ADDR: usize = 0x3D;

pub const BUFFER_SIZE: usize = 32;
pub const SEQUENCE_BUFFER_SIZE: usize = 32;
#[derive(PartialEq, Copy, Clone)]
pub struct ScreenCommand {
    pub id: CommandId,
    pub parameters: Option<&'static [u8]>,
}

#[repr(u8)]
pub enum MemoryAddressing {
    Page = 0x10,
    Horizontal = 0x00,
    Vertical = 0x01,
}

#[derive(Copy, Clone, PartialEq)]
enum ScreenStatus {
    Idle,
    Init,
    Error(ErrorCode),
    SendCommand(),
    SendCommandId(),
    // SendCommandArguments(usize),
}

#[derive(PartialEq, Copy, Clone)]
pub enum CommandId {
    /* Fundamental Commands */
    // Contrast Control - 2 bytes command
    // 2nd byte: contrast step - 0 <-> 255
    SetContrastControl = 0x81,

    // Entire Display ON
    // 0xA4 - output follows RAM content
    // 0xA5 - output ignores RAM content
    EntireDisplay = 0xA4,

    // Set Normal Display - 0xA6
    // 0/1 in RAM - OFF/ON in display panel
    // Set Inverse Display - 0xA7
    // 0/1 in RAM - ON?OFF in display panel
    SetNormalDisplayOn = 0xA6,
    SetNormalDisplayOff = 0xA7,

    // Set Display Off - 0xAE
    // Set Display On - 0xAF
    SetDisplayOff = 0xAE,
    SetDisplayOn = 0xAF,

    /* Addressing Settings */
    // Set Lower Column - 0x00 <-> 0x0F
    SetLowerColumn = 0x00,

    // Set Higher Column - 0x10 <-> 0x1F
    SetHigherColumn = 0x10,

    // Set Memory Addressing Mode - 2 bytes command
    // 2nd byte: MemoryAddressing enum
    SetMemoryMode = 0x20,

    // Set Column Address - 3 bytes command
    // 2nd byte: column start address (0-127)
    // 3rd byte: column end address (0-127)
    SetColumnAddr = 0x21,

    // Set Page Address - 3 bytes command
    // 2nd byte: page start address (0-7)
    // 3rd byte: page end address (0-7)
    SetPageAddr = 0x22,

    // Set Page Start Address for MemoryAddressing::Page - 0xB0 <-> 0xB7
    SetPageStart = 0xB0,

    /* Hardware Configuration */
    // Set Display Start Line - 0x40 <-> 0x7F
    SetDisplayStart = 0x40,

    // Set Segment Re-map
    // column address 0 -> SEG0 - 0xA0
    // column address 127 -> SEG0 - 0xA1
    SetSegmentRemap0 = 0xA0,
    SetSegmentRemap127 = 0xA1,

    // Set Multiplex Radio - 2 bytes command
    // 2nd byte: mux - 16 <-> 64
    SetMultiplexRadio = 0xA8,

    // Set COM Output Scan Direction
    // from COM0 -> COM[multiplex radio - 1] - 0xC0
    // from COM[multiplex radio - 1] -> COM0 - 0xC8
    SetCOMOutputScanAsc = 0xC0,
    SetCOMOutputScanDes = 0xC8,

    // Set Display Offset - 2 bytes command
    // 2nd byte: the vertical shift - 0 <->63
    SetDisplayOffset = 0xD3,

    // Set COM Pins - 2 bytes command
    // 2nd byte: - bit 4: 0 -> seq COM pin configuration
    //                    1 -> reset + alternative pin configuration
    //           - bit 5: 0 -> reset + disable COM left/right remap
    //                    1 -> enable COM left/right remap
    SetCOMPins = 0xDA,

    /* Timing & Driving Scheme Setting */
    // Set Display Clock Divide Ratio + Oscillator Freq - 2 bytes command
    // 2nd byte: - bits 3:0 -> divide ratio (D) of the display clocks (DCLK)
    //                         D = bits 3:0 + 1
    //           - bits 7:4 -> adds to the oscillator frequency
    SetDisplayClockDivideRatio = 0xD5,

    // Set Pre-charge period - 2 bytes command
    // 2nd byte: - bits 3:0 -> phase 1 period: 1 <-> 15
    //           - bits 7:4 -> phase 2 period: 1 <-> 15
    SetPreChargePeriod = 0xD9,

    // Set Vcomh Deselect Level - 2 bytes command
    // 2nd byte: bits 6:4 - 000b (0,65), 010b (0,77), 020b (0,83)
    SetVcomhDeselect = 0xDB,

    // Nop
    Nop = 0xE3,

    /* Scrolling Commands */
    // Continous Horizontal Scroll Setup - 7 bytes commands
    // 2nd, 6th and 7th bytes: dummy bytes
    // 3rd byte: start page address - 0 <-> 7
    // 4th byte: set time interval between each scroll step in frame freq
    //             000b -> 5 | 001b -> 64 | 010b -> 128 | 011b -> 256
    //             100b -> 3 | 101b -> 4  | 110b -> 25  | 111b -> 2
    // 5th byte: end page address - 0 <-> 7 (>= 3rd byte)
    ContHorizontalScrollRight = 0x26,
    ContHorizontalScrollLeft = 0x27,

    // Continous Horizontal & Vertical Scroll Setup - 6 bytes commands
    // 2nd byte: dummy byte
    // 3rd byte: start page address - 0 <-> 7
    // 4th byte: set time interval between each scroll step in frame freq
    //             000b -> 5 | 001b -> 64 | 010b -> 128 | 011b -> 256
    //             100b -> 3 | 101b -> 4  | 110b -> 25  | 111b -> 2
    // 5th byte: end page address - 0 <-> 7 (>= 3rd byte)
    // 6th byte: vertical scroll offset - 0 <-> 63
    ContVertHorizontalScrollRight = 0x29,
    ContVertHorizontalScrollLeft = 0x2A,

    // Deactivate Scrolling that is configured by one of the last 4 commands
    DeactivateScrolling = 0x2E,

    // Activate Scrolling that is configured by one of the last 4 commands
    // Overwrites the previously configured setup
    ActivateScrolling = 0x2F,

    // Set Vertical Scroll Area - 3 bytes command
    // 2nd byte: number of rows in top fixed area
    // 3rd byte: number of rows in scroll area
    SetVerticalScroll = 0xA3,

    /* Charge Pump Settings */
    // Charge Pump Command - 2 bytes command
    // 2nd byte: - 0x14 - enable (followed by 0xAF - display on)
    //           - 0x10 - disable
    ChargePump = 0x8D,
}

const SSD1306_INIT_SEQ: [ScreenCommand; 17] = [
    ScreenCommand {
        id: CommandId::SetDisplayOff,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::SetDisplayClockDivideRatio,
        parameters: Some(&[0x80]),
    },
    ScreenCommand {
        id: CommandId::SetMultiplexRadio,
        parameters: Some(&[HEIGHT - 1]),
    },
    ScreenCommand {
        id: CommandId::SetDisplayOffset,
        parameters: Some(&[0x00]),
    },
    ScreenCommand {
        id: CommandId::SetDisplayStart,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::ChargePump,
        parameters: Some(&[0x10]),
    },
    ScreenCommand {
        id: CommandId::SetMemoryMode,
        parameters: Some(&[MemoryAddressing::Horizontal as u8]),
    },
    ScreenCommand {
        id: CommandId::SetSegmentRemap127,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::SetCOMOutputScanDes,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::SetCOMPins,
        parameters: Some(&[0x12]),
    },
    ScreenCommand {
        id: CommandId::SetContrastControl,
        parameters: Some(&[0xCF]),
    },
    ScreenCommand {
        id: CommandId::SetPreChargePeriod,
        parameters: Some(&[0xF1]),
    },
    ScreenCommand {
        id: CommandId::SetVcomhDeselect,
        parameters: Some(&[0x40]),
    },
    ScreenCommand {
        id: CommandId::EntireDisplay,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::SetNormalDisplayOn,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::DeactivateScrolling,
        parameters: None,
    },
    ScreenCommand {
        id: CommandId::SetDisplayOn,
        parameters: None,
    },
];

pub struct SSD1306<'a, A: Alarm<'a>, B: Bus<'a>> {
    bus: &'a B,
    alarm: &'a A,
    screen: &'static SSD_Screen,
    status: Cell<ScreenStatus>,

    command_sequence: TakeCell<'static, [ScreenCommand]>,
    sequence_length: Cell<usize>,
    sequence_position: Cell<usize>,
    command_arguments: TakeCell<'static, [u8]>,

    initialization_complete: Cell<bool>,
    screen_client: OptionalCell<&'static dyn screen::ScreenClient>,
}

impl<'a, A: Alarm<'a>, B: Bus<'a>> SSD1306<'a, A, B> {
    pub fn new(
        bus: &'a B,
        alarm: &'a A,
        screen: &'static SSD_Screen,
        command_sequence: &'static mut [ScreenCommand],
        command_arguments: &'static mut [u8],
    ) -> SSD1306<'a, A, B> {
        SSD1306 {
            bus,
            alarm,
            screen,
            status: Cell::new(ScreenStatus::Idle),
            command_sequence: TakeCell::new(command_sequence),
            sequence_length: Cell::new(0),
            sequence_position: Cell::new(0),
            command_arguments: TakeCell::new(command_arguments),
            initialization_complete: Cell::new(false),
            screen_client: OptionalCell::empty(),
        }
    }

    pub fn init(&self) -> Result<(), ErrorCode> {
        debug!("[ssd1306 cap] init");
        if self.status.get() == ScreenStatus::Idle {
            self.status.set(ScreenStatus::Init);
            self.do_next_op();
            Ok(())
        } else {
            Err(ErrorCode::BUSY)
        }
    }

    pub fn do_next_op(&self) {
        match self.status.get() {
            ScreenStatus::Init => {
                debug!("[ssd1306 cap] do next op init");
                self.status.set(ScreenStatus::Idle);
                if let Err(err) = self.prepare_init_sequence() {
                    self.status.set(ScreenStatus::Error(err));
                } else {
                    self.sequence_position.set(0);
                    self.status.set(ScreenStatus::SendCommand());
                }
                self.do_next_op();
            }
            ScreenStatus::Idle => {
                panic!("SSD1306 Screen Idle");
            }
            ScreenStatus::Error(err) => todo!(),
            ScreenStatus::SendCommand() => {
                let position = self.sequence_position.get();
                debug!("[ssd1306 cap] do next op send command {}", position);
                // self.status.set(ScreenStatus::SendCommand(position + 1));
                if position < self.sequence_length.get() {
                    self.command_sequence.map_or_else(
                        || panic!("ssd1306: do next op has no command sequence buffer"),
                        |command_sequence| {
                            self.send_command(command_sequence[position]);
                        },
                    )
                } else {
                    // todo commands done
                    self.status.set(ScreenStatus::Idle);
                    if !self.initialization_complete.get() {
                        self.screen_client.map_or_else(
                            || panic!("ssd1306: do next op has no screen client"),
                            |screen_client| {
                                self.initialization_complete.set(true);
                                screen_client.screen_is_ready();
                            },
                        );
                    } else {
                        // todo
                    }
                }
            }
            ScreenStatus::SendCommandId() => {
                // if arguments {
                //     debug!("[ssd1306 cap] do next op send arguments");
                //     self.send_arguments(len);
                // } else {
                self.sequence_position.set(self.sequence_position.get() + 1);
                debug!("[ssd1306 cap] do next op send command id");
                self.status.set(ScreenStatus::SendCommand());
                self.do_next_op();
                // }
            } // ScreenStatus::SendCommandArguments(_) => {
              //     self.sequence_position.set(self.sequence_position.get() + 1);
              //     self.status.set(ScreenStatus::SendCommand());
              //     self.do_next_op();
              // }
        }
    }

    fn send_arguments(&self, len: usize) {
        // self.status.set(ScreenStatus::SendCommandArguments(len));
        if len > 0 {
            self.command_arguments.take().map_or_else(
                || panic!("ssd1306: send argument has no command arguments buffer"),
                |arguments| {
                    debug!("[ssd1306 cap] send arguments {:?}", arguments);
                    let _ = self.bus.write(BusWidth::Bits8, arguments, len);
                },
            );
        }
    }

    fn send_command(&self, cmd: ScreenCommand) {
        // let (parameters_exist, len) = if let Some(params) = cmd.parameters {
        //     self.populate_arguments_buffer(cmd);
        //     debug!("[ssd1306 cap] send command with {} args", params.len());
        //     (true, params.len())
        // } else {
        //     (false, 0)
        // };
        self.status.set(ScreenStatus::SendCommandId());
        debug!("[ssd1306 cap] send command with id {:x}", cmd.id as usize);
        let _ = self.bus.set_addr(BusWidth::Bits8, cmd.id as usize);
        // if let Some(params) = cmd.parameters {
        //     debug!("[ssd1306 cap] send command with {} args", params.len());
        //     self.populate_arguments_buffer(cmd);
        //     self.send_arguments(params.len());
        // }
    }

    fn populate_arguments_buffer(&self, cmd: ScreenCommand) {
        self.command_arguments.map_or_else(
            || panic!("ssd1306 populate arguments has no command arguments buffer"),
            |command_buffer| {
                if let Some(parameters) = cmd.parameters {
                    for (i, param) in parameters.iter().enumerate() {
                        command_buffer[i] = *param;
                    }
                }
            },
        )
    }

    fn prepare_init_sequence(&self) -> Result<(), ErrorCode> {
        if self.status.get() == ScreenStatus::Idle {
            self.command_sequence.map_or_else(
                || panic!("ssd1306: init sequence has no command sequence buffer"),
                |command_sequence| {
                    if SSD1306_INIT_SEQ.len() <= command_sequence.len() {
                        self.sequence_length.set(SSD1306_INIT_SEQ.len());
                        for (i, cmd) in SSD1306_INIT_SEQ.iter().enumerate() {
                            command_sequence[i] = *cmd;
                        }
                        Ok(())
                    } else {
                        Err(ErrorCode::NOMEM)
                    }
                },
            )
        } else {
            Err(ErrorCode::BUSY)
        }
    }
}

impl<'a, A: Alarm<'a>, B: Bus<'a>> time::AlarmClient for SSD1306<'a, A, B> {
    fn alarm(&self) {
        todo!()
    }
}

impl<'a, A: Alarm<'a>, B: Bus<'a>> bus::Client for SSD1306<'a, A, B> {
    fn command_complete(
        &self,
        buffer: Option<&'static mut [u8]>,
        len: usize,
        status: Result<(), kernel::ErrorCode>,
    ) {
        if let Some(command_arguments) = buffer {
            self.command_arguments.replace(command_arguments);
        }

        if let Err(err) = status {
            self.status.set(ScreenStatus::Error(err));
        }

        self.do_next_op();
    }
}

impl<'a, A: Alarm<'a>, B: Bus<'a>> screen::Screen for SSD1306<'a, A, B> {
    fn get_resolution(&self) -> (usize, usize) {
        (WIDTH, HEIGHT as usize)
    }

    fn get_pixel_format(&self) -> screen::ScreenPixelFormat {
        screen::ScreenPixelFormat::Mono
    }

    fn get_rotation(&self) -> screen::ScreenRotation {
        todo!()
    }

    fn set_write_frame(
        &self,
        x: usize,
        y: usize,
        width: usize,
        height: usize,
    ) -> Result<(), kernel::ErrorCode> {
        todo!()
    }

    fn write(&self, buffer: &'static mut [u8], len: usize) -> Result<(), kernel::ErrorCode> {
        todo!()
    }

    fn write_continue(
        &self,
        buffer: &'static mut [u8],
        len: usize,
    ) -> Result<(), kernel::ErrorCode> {
        todo!()
    }

    fn set_client(&self, client: Option<&'static dyn screen::ScreenClient>) {
        if let Some(client) = client {
            self.screen_client.set(client);
        } else {
            self.screen_client.clear();
        }
    }

    fn set_brightness(&self, brightness: usize) -> Result<(), kernel::ErrorCode> {
        todo!()
    }

    fn set_power(&self, enabled: bool) -> Result<(), kernel::ErrorCode> {
        todo!()
    }

    fn set_invert(&self, enabled: bool) -> Result<(), kernel::ErrorCode> {
        todo!()
    }
}

pub struct SSD_Screen {
    init_sequence: &'static [ScreenCommand],
    default_width: usize,
    default_height: usize,
    inverted: bool,
}

pub const SSD1306_Screen: SSD_Screen = SSD_Screen {
    init_sequence: &SSD1306_INIT_SEQ,
    default_height: 64,
    default_width: 128,
    inverted: false,
};
