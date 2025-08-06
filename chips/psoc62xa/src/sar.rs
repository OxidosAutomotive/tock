use kernel::hil;
use kernel::utilities::cells::OptionalCell;
use kernel::utilities::registers::interfaces::{ReadWriteable, Readable};
use kernel::utilities::registers::{register_bitfields, register_structs, ReadOnly, ReadWrite};
use kernel::utilities::StaticRef;

register_structs! {
    /// SAR ADC with Sequencer
    SarRegisters {
        /// Analog control register.
        (0x000 => ctrl: ReadWrite<u32, CTRL::Register>),
        /// Sample control register.
        (0x004 => sample_ctrl: ReadWrite<u32, SAMPLE_CTRL::Register>),
        (0x008 => _reserved0),
        /// Sample time specification ST0 and ST1
        (0x010 => sample_time01: ReadWrite<u32, SAMPLE_TIME01::Register>),
        /// Sample time specification ST2 and ST3
        (0x014 => sample_time23: ReadWrite<u32, SAMPLE_TIME23::Register>),
        /// Global range detect threshold register.
        (0x018 => range_thres: ReadWrite<u32, RANGE_THRES::Register>),
        /// Global range detect mode register.
        (0x01C => range_cond: ReadWrite<u32, RANGE_COND::Register>),
        /// Enable bits for the channels
        (0x020 => chan_en: ReadWrite<u32, CHAN_EN::Register>),
        /// Start control register (firmware trigger).
        (0x024 => start_ctrl: ReadWrite<u32, START_CTRL::Register>),
        (0x028 => _reserved1),
        /// Channel configuration register.
        (0x080 => chan_config: [ReadWrite<u32, CHAN_CONFIG::Register>; 16]),
        (0x0C0 => _reserved2),
        /// Channel working data register
        (0x100 => chan_work: [ReadOnly<u32, CHAN_WORK::Register>; 16]),
        (0x140 => _reserved3),
        /// Channel result data register
        (0x180 => chan_result: [ReadOnly<u32, CHAN_RESULT::Register>; 16]),
        (0x1C0 => _reserved4),
        /// Channel working data register 'updated' bits
        (0x200 => chan_work_updated: ReadOnly<u32>),
        /// Channel result data register 'updated' bits
        (0x204 => chan_result_updated: ReadOnly<u32>),
        /// Channel working data register 'new value' bits
        (0x208 => chan_work_newvalue: ReadOnly<u32>),
        /// Channel result data register 'new value' bits
        (0x20C => chan_result_newvalue: ReadOnly<u32>),
        /// Interrupt request register.
        (0x210 => intr: ReadWrite<u32, INTR::Register>),
        /// Interrupt set request register
        (0x214 => intr_set: ReadWrite<u32, INTR_SET::Register>),
        /// Interrupt mask register.
        (0x218 => intr_mask: ReadWrite<u32, INTR_MASK::Register>),
        /// Interrupt masked request register
        (0x21C => intr_masked: ReadOnly<u32, INTR_MASKED::Register>),
        /// Saturate interrupt request register.
        (0x220 => saturate_intr: ReadWrite<u32>),
        /// Saturate interrupt set request register
        (0x224 => saturate_intr_set: ReadWrite<u32>),
        /// Saturate interrupt mask register.
        (0x228 => saturate_intr_mask: ReadWrite<u32>),
        /// Saturate interrupt masked request register
        (0x22C => saturate_intr_masked: ReadOnly<u32>),
        /// Range detect interrupt request register.
        (0x230 => range_intr: ReadWrite<u32>),
        /// Range detect interrupt set request register
        (0x234 => range_intr_set: ReadWrite<u32>),
        /// Range detect interrupt mask register.
        (0x238 => range_intr_mask: ReadWrite<u32>),
        /// Range interrupt masked request register
        (0x23C => range_intr_masked: ReadOnly<u32>),
        /// Interrupt cause register
        (0x240 => intr_cause: ReadOnly<u32, INTR_CAUSE::Register>),
        (0x244 => _reserved5),
        /// Injection channel configuration register.
        (0x280 => inj_chan_config: ReadWrite<u32, INJ_CHAN_CONFIG::Register>),
        (0x284 => _reserved6),
        /// Injection channel result register
        (0x290 => inj_result: ReadOnly<u32, INJ_RESULT::Register>),
        (0x294 => _reserved7),
        /// Current status of internal SAR registers (mostly for debug)
        (0x2A0 => status: ReadOnly<u32, STATUS::Register>),
        /// Current averaging status (for debug)
        (0x2A4 => avg_stat: ReadOnly<u32, AVG_STAT::Register>),
        (0x2A8 => _reserved8),
        /// SARMUX Firmware switch controls
        (0x300 => mux_switch0: ReadWrite<u32, MUX_SWITCH0::Register>),
        /// SARMUX Firmware switch control clear
        (0x304 => mux_switch_clear0: ReadWrite<u32, MUX_SWITCH_CLEAR0::Register>),
        (0x308 => _reserved9),
        /// SARMUX switch DSI control
        (0x340 => mux_switch_ds_ctrl: ReadWrite<u32, MUX_SWITCH_DS_CTRL::Register>),
        /// SARMUX switch Sar Sequencer control
        (0x344 => mux_switch_sq_ctrl: ReadWrite<u32, MUX_SWITCH_SQ_CTRL::Register>),
        /// SARMUX switch status
        (0x348 => mux_switch_status: ReadOnly<u32, MUX_SWITCH_STATUS::Register>),
        (0x34C => _reserved10),
        /// Analog trim register.
        (0xF00 => ana_trim0: ReadWrite<u32, ANA_TRIM0::Register>),
        /// Analog trim register.
        (0xF04 => ana_trim1: ReadWrite<u32>),
        (0xF08 => @END),
    }
}
register_bitfields![u32,
CTRL [
    /// VREF buffer low power mode.
    PWR_CTRL_VREF OFFSET(0) NUMBITS(3) [
        /// full power  (100 percent) (default), bypass cap, max clk_sar is 18MHz.
        FullPower100PercentDefaultBypassCapMaxClk_sarIs18MHz = 0,
        /// 80 percent power
        _80PercentPower = 1,
        /// 60 percent power
        _60PercentPower = 2,
        /// 50 percent power
        _50PercentPower = 3,
        /// 40 percent power
        _40PercentPower = 4,
        /// 30 percent power
        _30PercentPower = 5,
        /// 20 percent power
        _20PercentPower = 6,
        /// 10 percent power
        _10PercentPower = 7
    ],
    /// SARADC internal VREF selection.
    VREF_SEL OFFSET(4) NUMBITS(3) [
        /// VREF0 from PRB (VREF buffer on)
        VREF0FromPRBVREFBufferOn = 0,
        /// VREF1 from PRB (VREF buffer on)
        VREF1FromPRBVREFBufferOn = 1,
        /// VREF2 from PRB (VREF buffer on)
        VREF2FromPRBVREFBufferOn = 2,
        /// VREF from AROUTE (VREF buffer on)
        VREFFromAROUTEVREFBufferOn = 3,
        /// 1.024V from BandGap (VREF buffer on)
        _1024VFromBandGapVREFBufferOn = 4,
        /// External precision Vref direct from a pin (low impedance path).
        ExternalPrecisionVrefDirectFromAPinLowImpedancePath = 5,
        /// Vdda/2  (VREF buffer on)
        Vdda2VREFBufferOn = 6,
        /// Vdda.
        Vdda = 7
    ],
    /// VREF bypass cap enable for when VREF buffer is on
    VREF_BYP_CAP_EN OFFSET(7) NUMBITS(1) [],
    /// SARADC internal NEG selection for Single ended conversion
    NEG_SEL OFFSET(9) NUMBITS(3) [
        /// NEG input of SARADC is connected to 'vssa_kelvin', gives more precision around z
        VSSA_KELVIN = 0,
        /// NEG input of SARADC is connected to VSSA in AROUTE close to the SARADC
        NEGInputOfSARADCIsConnectedToVSSAInAROUTECloseToTheSARADC = 1,
        /// NEG input of SARADC is connected to P1 pin of SARMUX
        NEGInputOfSARADCIsConnectedToP1PinOfSARMUX = 2,
        /// NEG input of SARADC is connected to P3 pin of SARMUX
        NEGInputOfSARADCIsConnectedToP3PinOfSARMUX = 3,
        /// NEG input of SARADC is connected to P5 pin of SARMUX
        NEGInputOfSARADCIsConnectedToP5PinOfSARMUX = 4,
        /// NEG input of SARADC is connected to P7 pin of SARMUX
        NEGInputOfSARADCIsConnectedToP7PinOfSARMUX = 5,
        /// NEG input of SARADC is connected to an ACORE in AROUTE
        NEGInputOfSARADCIsConnectedToAnACOREInAROUTE = 6,
        /// NEG input of SARADC is shorted with VREF input of SARADC.
        NEGInputOfSARADCIsShortedWithVREFInputOfSARADC = 7
    ],
    /// Hardware control: 0=only firmware control, 1=hardware control masked by firmware
    SAR_HW_CTRL_NEGVREF OFFSET(13) NUMBITS(1) [],
    /// Set the comparator latch delay in accordance with SAR conversion rate
    COMP_DLY OFFSET(14) NUMBITS(2) [
        /// 2.5ns delay, use this for 2.5Msps
        _25nsDelayUseThisFor25Msps = 0,
        /// 4.0ns delay, use this for 2.0Msps
        _40nsDelayUseThisFor20Msps = 1,
        /// 10ns delay, use this for 1.5Msps
        _10nsDelayUseThisFor15Msps = 2,
        /// 12ns delay, use this for 1.0Msps or less
        _12nsDelayUseThisFor10MspsOrLess = 3
    ],
    /// Spare controls, not yet designated, for late changes done with an ECO
    SPARE OFFSET(16) NUMBITS(4) [],
    /// deprecated
    BOOSTPUMP_EN OFFSET(20) NUMBITS(1) [],
    /// For normal ADC operation this bit must be set, for all reference choices - inter
/// Setting this bit is critical to proper function of switches inside SARREF block.
    REFBUF_EN OFFSET(21) NUMBITS(1) [],
    /// Comparator power mode.
    COMP_PWR OFFSET(24) NUMBITS(3) [
        /// Power = 100 percent, Use this for SAR Clock Frequency greater than 18MHz
        Power100PercentUseThisForSARClockFrequencyGreaterThan18MHz = 0,
        /// N/A
        NA = 1,
        /// Power = 60 percent, Use this for SAR Clock Frequency greater than 1.8MHz up to 1
        Power60PercentUseThisForSARClockFrequencyGreaterThan18MHzUpTo18MHz = 2,
        /// Power = 20 percent, Use this for SAR Clock Frequency less than or equal to 1.8MH
        Power20PercentUseThisForSARClockFrequencyLessThanOrEqualTo18MHz = 6
    ],
    /// - 0: SARMUX IP disabled off during DeepSleep power mode
/// - 1: SARMUX IP remains enabled during DeepSleep power mode (if ENABLED=1)
    DEEPSLEEP_ON OFFSET(27) NUMBITS(1) [],
    /// - 0: bypass clock domain synchronization of the DSI config signals.
/// - 1: synchronize the DSI config signals to peripheral clock domain.
    DSI_SYNC_CONFIG OFFSET(28) NUMBITS(1) [],
    /// SAR sequencer takes configuration from DSI signals (note this also has the same
/// - 0: Normal mode, SAR sequencer operates according to CHAN_EN enables and CHAN_C
/// - 1: CHAN_EN, INJ_START_EN and channel configurations in CHAN_CONFIG and INJ_CHA
    DSI_MODE OFFSET(29) NUMBITS(1) [],
    /// Disable SAR sequencer from enabling routing switches (note DSI and firmware can
/// - 0: Normal mode, SAR sequencer changes switches according to pin address in cha
/// - 1: Switches disabled, SAR sequencer does not enable any switches, it is the re
    SWITCH_DISABLE OFFSET(30) NUMBITS(1) [],
    /// - 0: SAR IP disabled (put analog in power down and stop clocks), also can clear
/// - 1: SAR IP enabled.
    ENABLED OFFSET(31) NUMBITS(1) []
],
SAMPLE_CTRL [
    /// Left align data in data[15:0], default data is right aligned in data[11:0], with
    LEFT_ALIGN OFFSET(1) NUMBITS(1) [],
    /// Output data from a single ended conversion as a signed value
///
/// If AVG_MODE = 1 (Interleaved averaging), then SINGLE_ENDED_SIGNED must be config
    SINGLE_ENDED_SIGNED OFFSET(2) NUMBITS(1) [
        /// Default: result data is unsigned (zero extended if needed)
        DefaultResultDataIsUnsignedZeroExtendedIfNeeded = 0,
        /// result data is signed (sign extended if needed)
        ResultDataIsSignedSignExtendedIfNeeded = 1
    ],
    /// Output data from a differential conversion as a signed value when DIFFERENTIAL_E
///
/// If AVG_MODE = 1 (Interleaved averaging), then DIFFERENTIAL_SIGNED must be config
    DIFFERENTIAL_SIGNED OFFSET(3) NUMBITS(1) [
        /// result data is unsigned (zero extended if needed)
        ResultDataIsUnsignedZeroExtendedIfNeeded = 0,
        /// Default: result data is signed (sign extended if needed)
        DefaultResultDataIsSignedSignExtendedIfNeeded = 1
    ],
    /// Averaging Count for channels that have averaging enabled (AVG_EN). A channel wil
/// - In ACCUNDUMP mode  (1st order accumulate and dump filter) a channel will be sa
/// - In INTERLEAVED mode one sample is taken per triggered scan, only in the scan w
    AVG_CNT OFFSET(4) NUMBITS(3) [],
    /// Averaging shifting: after averaging the result is shifted right to fit in 12 bit
    AVG_SHIFT OFFSET(7) NUMBITS(1) [],
    /// Averaging mode,  in DSI mode this bit is ignored and only AccuNDump mode is avai
    AVG_MODE OFFSET(8) NUMBITS(1) [
        /// Accumulate and Dump (1st order accumulate and dump filter): a channel will be sa
        ACCUNDUMP = 0,
        /// Interleaved: Each scan (trigger) one sample is taken per channel and averaged ov
        InterleavedEachScanTriggerOneSampleIsTakenPerChannelAndAveragedOverSeveralScans = 1
    ],
    /// - 0: Wait for next FW_TRIGGER (one shot) or hardware trigger (e.g. from TPWM for
/// - 1: Continuously scan enabled channels, ignore triggers.
    CONTINUOUS OFFSET(16) NUMBITS(1) [],
    /// - 0: firmware trigger only: disable hardware trigger tr_sar_in.
/// - 1: enable hardware trigger tr_sar_in (e.g. from TCPWM, GPIO or UDB).
    DSI_TRIGGER_EN OFFSET(17) NUMBITS(1) [],
    /// - 0: trigger signal is a pulse input, a positive edge detected on the trigger si
/// - 1: trigger signal is a level input, as long as the trigger signal remains high
    DSI_TRIGGER_LEVEL OFFSET(18) NUMBITS(1) [],
    /// - 0: bypass clock domain synchronization of the trigger signal.
/// - 1: synchronize the trigger signal to the SAR clock domain, if needed an edge d
    DSI_SYNC_TRIGGER OFFSET(19) NUMBITS(1) [],
    /// Select whether UABs are scheduled or unscheduled. When no UAB is scanned this se
    UAB_SCAN_MODE OFFSET(22) NUMBITS(1) [
        /// Unscheduled UABs: one or more of the UABs scanned by the SAR is not scheduled, f
        UNSCHEDULED = 0,
        /// Scheduled UABs: All UABs scanned by the SAR are assumed to be properly scheduled
/// This mode requires that the SAR scans strictly periodically, i.e. the SAR has to
        SCHEDULED = 1
    ],
    /// For unscheduled UAB_SCAN_MODE only, do the following if an invalid sample is rec
/// - 0: use the last known valid sample for that channel and clear the NEWVALUE fla
/// - 1: repeat the conversions until a valid sample is received (caveat: could be n
    REPEAT_INVALID OFFSET(23) NUMBITS(1) [],
    /// Static UAB Valid select
/// 0=UAB0 half 0 Valid output
/// 1=UAB0 half 1 Valid output
/// 2=UAB1 half 0 Valid output
/// 3=UAB1 half 1 Valid output
/// 4=UAB2 half 0 Valid output
/// 5=UAB2 half 1 Valid output
/// 6=UAB3 half 0 Valid output
/// 7=UAB3 half 1 Valid output
    VALID_SEL OFFSET(24) NUMBITS(3) [],
    /// Enable static UAB Valid selection (override Hardware)
    VALID_SEL_EN OFFSET(27) NUMBITS(1) [],
    /// Ignore UAB valid signal, including the dynamic/Hardware from AROUTE and the stat
    VALID_IGNORE OFFSET(28) NUMBITS(1) [],
    /// SAR output trigger enable (used for UAB synchronization). To ensure multiple UAB
    TRIGGER_OUT_EN OFFSET(30) NUMBITS(1) [],
    /// Enable to output EOS_INTR to DSI. When enabled each time EOS_INTR is set by the
    EOS_DSI_OUT_EN OFFSET(31) NUMBITS(1) []
],
SAMPLE_TIME01 [
    /// Sample time0 (aperture) in ADC clock cycles. Note that actual sample time is one
    SAMPLE_TIME0 OFFSET(0) NUMBITS(10) [],
    /// Sample time1
    SAMPLE_TIME1 OFFSET(16) NUMBITS(10) []
],
SAMPLE_TIME23 [
    /// Sample time2
    SAMPLE_TIME2 OFFSET(0) NUMBITS(10) [],
    /// Sample time3
    SAMPLE_TIME3 OFFSET(16) NUMBITS(10) []
],
RANGE_THRES [
    /// Low threshold for range detect.
    RANGE_LOW OFFSET(0) NUMBITS(16) [],
    /// High threshold for range detect.
    RANGE_HIGH OFFSET(16) NUMBITS(16) []
],
RANGE_COND [
    /// Range condition select.
    RANGE_COND OFFSET(30) NUMBITS(2) [
        /// result < RANGE_LOW
        ResultRANGE_LOW = 0,
        /// RANGE_LOW <= result < RANGE_HIGH
        RANGE_LOWResultRANGE_HIGH = 1,
        /// RANGE_HIGH <= result
        RANGE_HIGHResult = 2,
        /// result < RANGE_LOW || RANGE_HIGH <= result
        ResultRANGE_LOWRANGE_HIGHResult = 3
    ]
],
CHAN_EN [
    /// Channel enable.
/// - 0: the corresponding channel is disabled.
/// - 1: the corresponding channel is enabled, it will be included in the next scan.
    CHAN_EN OFFSET(0) NUMBITS(16) []
],
START_CTRL [
    /// When firmware writes a 1 here it will trigger the next scan of enabled channels,
    FW_TRIGGER OFFSET(0) NUMBITS(1) []
],
CHAN_WORK_UPDATED [
    /// If set the corresponding WORK register was updated, i.e. was already sampled dur
    CHAN_WORK_UPDATED OFFSET(0) NUMBITS(16) []
],
CHAN_RESULT_UPDATED [
    /// If set the corresponding RESULT register was updated, i.e. was sampled during th
    CHAN_RESULT_UPDATED OFFSET(0) NUMBITS(16) []
],
CHAN_WORK_NEWVALUE [
    /// If set the corresponding WORK data received a new value, i.e. was already sample
/// In case of  a UAB this New Value bit reflects the value of UAB.valid output, for
/// In case of averaging this New Value bit is an OR of all the valid bits received
    CHAN_WORK_NEWVALUE OFFSET(0) NUMBITS(16) []
],
CHAN_RESULT_NEWVALUE [
    /// If set the corresponding RESULT data received a new value, i.e. was sampled duri
/// In case of  a UAB this New Value bit reflects the value of UAB.valid output, for
/// In case of averaging this New Value bit is an OR of all the valid bits received
    CHAN_RESULT_NEWVALUE OFFSET(0) NUMBITS(16) []
],
INTR [
    /// End Of Scan Interrupt: hardware sets this interrupt after completing a scan of a
    EOS_INTR OFFSET(0) NUMBITS(1) [],
    /// Overflow Interrupt: hardware sets this interrupt when it sets a new EOS_INTR whi
    OVERFLOW_INTR OFFSET(1) NUMBITS(1) [],
    /// Firmware Collision Interrupt: hardware sets this interrupt when FW_TRIGGER is as
    FW_COLLISION_INTR OFFSET(2) NUMBITS(1) [],
    /// DSI Collision Interrupt: hardware sets this interrupt when the DSI trigger signa
    DSI_COLLISION_INTR OFFSET(3) NUMBITS(1) [],
    /// Injection End of Conversion Interrupt: hardware sets this interrupt after comple
    INJ_EOC_INTR OFFSET(4) NUMBITS(1) [],
    /// Injection Saturation Interrupt: hardware sets this interrupt if an injection con
    INJ_SATURATE_INTR OFFSET(5) NUMBITS(1) [],
    /// Injection Range detect Interrupt: hardware sets this interrupt if the injection
    INJ_RANGE_INTR OFFSET(6) NUMBITS(1) [],
    /// Injection Collision Interrupt: hardware sets this interrupt when the injection t
    INJ_COLLISION_INTR OFFSET(7) NUMBITS(1) []
],
INTR_SET [
    /// Write with '1' to set corresponding bit in interrupt request register.
    EOS_SET OFFSET(0) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    OVERFLOW_SET OFFSET(1) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    FW_COLLISION_SET OFFSET(2) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    DSI_COLLISION_SET OFFSET(3) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    INJ_EOC_SET OFFSET(4) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    INJ_SATURATE_SET OFFSET(5) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    INJ_RANGE_SET OFFSET(6) NUMBITS(1) [],
    /// Write with '1' to set corresponding bit in interrupt request register.
    INJ_COLLISION_SET OFFSET(7) NUMBITS(1) []
],
INTR_MASK [
    /// Mask bit for corresponding bit in interrupt request register.
    EOS_MASK OFFSET(0) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    OVERFLOW_MASK OFFSET(1) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    FW_COLLISION_MASK OFFSET(2) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    DSI_COLLISION_MASK OFFSET(3) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    INJ_EOC_MASK OFFSET(4) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    INJ_SATURATE_MASK OFFSET(5) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    INJ_RANGE_MASK OFFSET(6) NUMBITS(1) [],
    /// Mask bit for corresponding bit in interrupt request register.
    INJ_COLLISION_MASK OFFSET(7) NUMBITS(1) []
],
INTR_MASKED [
    /// Logical and of corresponding request and mask bits.
    EOS_MASKED OFFSET(0) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    OVERFLOW_MASKED OFFSET(1) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    FW_COLLISION_MASKED OFFSET(2) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    DSI_COLLISION_MASKED OFFSET(3) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    INJ_EOC_MASKED OFFSET(4) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    INJ_SATURATE_MASKED OFFSET(5) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    INJ_RANGE_MASKED OFFSET(6) NUMBITS(1) [],
    /// Logical and of corresponding request and mask bits.
    INJ_COLLISION_MASKED OFFSET(7) NUMBITS(1) []
],
SATURATE_INTR [
    /// Saturate Interrupt: hardware sets this interrupt for each channel if a conversio
    SATURATE_INTR OFFSET(0) NUMBITS(16) []
],
SATURATE_INTR_SET [
    /// Write with '1' to set corresponding bit in interrupt request register.
    SATURATE_SET OFFSET(0) NUMBITS(16) []
],
SATURATE_INTR_MASK [
    /// Mask bit for corresponding bit in interrupt request register.
    SATURATE_MASK OFFSET(0) NUMBITS(16) []
],
SATURATE_INTR_MASKED [
    /// Logical and of corresponding request and mask bits.
    SATURATE_MASKED OFFSET(0) NUMBITS(16) []
],
RANGE_INTR [
    /// Range detect Interrupt: hardware sets this interrupt for each channel if the con
    RANGE_INTR OFFSET(0) NUMBITS(16) []
],
RANGE_INTR_SET [
    /// Write with '1' to set corresponding bit in interrupt request register.
    RANGE_SET OFFSET(0) NUMBITS(16) []
],
RANGE_INTR_MASK [
    /// Mask bit for corresponding bit in interrupt request register.
    RANGE_MASK OFFSET(0) NUMBITS(16) []
],
RANGE_INTR_MASKED [
    /// Logical and of corresponding request and mask bits.
    RANGE_MASKED OFFSET(0) NUMBITS(16) []
],
INTR_CAUSE [
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    EOS_MASKED_MIR OFFSET(0) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    OVERFLOW_MASKED_MIR OFFSET(1) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    FW_COLLISION_MASKED_MIR OFFSET(2) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    DSI_COLLISION_MASKED_MIR OFFSET(3) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    INJ_EOC_MASKED_MIR OFFSET(4) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    INJ_SATURATE_MASKED_MIR OFFSET(5) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    INJ_RANGE_MASKED_MIR OFFSET(6) NUMBITS(1) [],
    /// Mirror copy of corresponding bit in SAR_INTR_MASKED
    INJ_COLLISION_MASKED_MIR OFFSET(7) NUMBITS(1) [],
    /// Reduction OR of all SAR_SATURATION_INTR_MASKED bits
    SATURATE_MASKED_RED OFFSET(30) NUMBITS(1) [],
    /// Reduction OR of all SAR_RANGE_INTR_MASKED bits
    RANGE_MASKED_RED OFFSET(31) NUMBITS(1) []
],
INJ_CHAN_CONFIG [
    /// Address of the pin to be sampled by this injection channel. If differential is e
    INJ_PIN_ADDR OFFSET(0) NUMBITS(3) [],
    /// Address of the port that contains the pin to be sampled by this channel.
    INJ_PORT_ADDR OFFSET(4) NUMBITS(3) [
        /// SARMUX pins.
        SARMUXPins = 0,
        /// CTB0
        CTB0 = 1,
        /// CTB1
        CTB1 = 2,
        /// CTB2
        CTB2 = 3,
        /// CTB3
        CTB3 = 4,
        /// AROUTE virtual port
        AROUTEVirtualPort = 6,
        /// SARMUX virtual port
        SARMUXVirtualPort = 7
    ],
    /// Differential enable for this channel.
/// - 0: The voltage on the addressed pin is measured (Single-ended) and the resulti
/// - 1: The differential voltage on the addressed pin pair is measured and the resu
    INJ_DIFFERENTIAL_EN OFFSET(8) NUMBITS(1) [],
    /// Averaging enable for this channel. If set the AVG_CNT and AVG_SHIFT settings are
    INJ_AVG_EN OFFSET(10) NUMBITS(1) [],
    /// Injection sample time select: select which of the 4 global sample times to use f
    INJ_SAMPLE_TIME_SEL OFFSET(12) NUMBITS(2) [],
    /// Injection channel tailgating.
/// - 0: no tailgating for this channel, SAR is immediately triggered when the INJ_S
/// - 1: injection channel tailgating. The addressed pin is sampled after the next t
    INJ_TAILGATING OFFSET(30) NUMBITS(1) [],
    /// Set by firmware to enable the injection channel. If INJ_TAILGATING is not set th
    INJ_START_EN OFFSET(31) NUMBITS(1) []
],
INJ_RESULT [
    /// SAR conversion result of the channel.
    INJ_RESULT OFFSET(0) NUMBITS(16) [],
    /// The data in this register received a new value (only relevant for UAB, this bit
    INJ_NEWVALUE OFFSET(27) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_INTR register
    INJ_COLLISION_INTR_MIR OFFSET(28) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_INTR register
    INJ_SATURATE_INTR_MIR OFFSET(29) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_INTR register
    INJ_RANGE_INTR_MIR OFFSET(30) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_INTR register
    INJ_EOC_INTR_MIR OFFSET(31) NUMBITS(1) []
],
STATUS [
    /// current channel being sampled (channel 16 indicates the injection channel), only
    CUR_CHAN OFFSET(0) NUMBITS(5) [],
    /// the current switch status, including DSI and sequencer controls, of the switch i
    SW_VREF_NEG OFFSET(30) NUMBITS(1) [],
    /// If high then the SAR is busy with a conversion. This bit is always high when CON
    BUSY OFFSET(31) NUMBITS(1) []
],
AVG_STAT [
    /// the current value of the averaging accumulator
    CUR_AVG_ACCU OFFSET(0) NUMBITS(20) [],
    /// If high then the SAR is in the middle of Interleaved averaging spanning several
/// This bit can be cleared by changing the averaging mode to ACCUNDUMP or by disabl
    INTRLV_BUSY OFFSET(23) NUMBITS(1) [],
    /// the current value of the averaging counter. Note that the value shown is updated
    CUR_AVG_CNT OFFSET(24) NUMBITS(8) []
],
MUX_SWITCH0 [
    /// Firmware control: 0=open, 1=close switch between pin P0 and vplus signal. Write
    MUX_FW_P0_VPLUS OFFSET(0) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P1 and vplus signal. Write
    MUX_FW_P1_VPLUS OFFSET(1) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P2 and vplus signal. Write
    MUX_FW_P2_VPLUS OFFSET(2) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P3 and vplus signal. Write
    MUX_FW_P3_VPLUS OFFSET(3) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P4 and vplus signal. Write
    MUX_FW_P4_VPLUS OFFSET(4) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P5 and vplus signal. Write
    MUX_FW_P5_VPLUS OFFSET(5) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P6 and vplus signal. Write
    MUX_FW_P6_VPLUS OFFSET(6) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P7 and vplus signal. Write
    MUX_FW_P7_VPLUS OFFSET(7) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P0 and vminus signal. Write
    MUX_FW_P0_VMINUS OFFSET(8) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P1 and vminus signal. Write
    MUX_FW_P1_VMINUS OFFSET(9) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P2 and vminus signal. Write
    MUX_FW_P2_VMINUS OFFSET(10) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P3 and vminus signal. Write
    MUX_FW_P3_VMINUS OFFSET(11) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P4 and vminus signal. Write
    MUX_FW_P4_VMINUS OFFSET(12) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P5 and vminus signal. Write
    MUX_FW_P5_VMINUS OFFSET(13) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P6 and vminus signal. Write
    MUX_FW_P6_VMINUS OFFSET(14) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between pin P7 and vminus signal. Write
    MUX_FW_P7_VMINUS OFFSET(15) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between vssa_kelvin and vminus signal.
    MUX_FW_VSSA_VMINUS OFFSET(16) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between temperature sensor and vplus si
    MUX_FW_TEMP_VPLUS OFFSET(17) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between amuxbusa and vplus signal. Writ
    MUX_FW_AMUXBUSA_VPLUS OFFSET(18) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between amuxbusb and vplus signal. Writ
    MUX_FW_AMUXBUSB_VPLUS OFFSET(19) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between amuxbusa and vminus signal. Wri
    MUX_FW_AMUXBUSA_VMINUS OFFSET(20) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between amuxbusb and vminus signal. Wri
    MUX_FW_AMUXBUSB_VMINUS OFFSET(21) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between sarbus0 and vplus signal. Write
    MUX_FW_SARBUS0_VPLUS OFFSET(22) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between sarbus1 and vplus signal. Write
    MUX_FW_SARBUS1_VPLUS OFFSET(23) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between sarbus0 and vminus signal. Writ
    MUX_FW_SARBUS0_VMINUS OFFSET(24) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between sarbus1 and vminus signal. Writ
    MUX_FW_SARBUS1_VMINUS OFFSET(25) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between P4 and coreio0 signal. Write wi
    MUX_FW_P4_COREIO0 OFFSET(26) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between P5 and coreio1 signal. Write wi
    MUX_FW_P5_COREIO1 OFFSET(27) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between P6 and coreio2 signal. Write wi
    MUX_FW_P6_COREIO2 OFFSET(28) NUMBITS(1) [],
    /// Firmware control: 0=open, 1=close switch between P7 and coreio3 signal. Write wi
    MUX_FW_P7_COREIO3 OFFSET(29) NUMBITS(1) []
],
MUX_SWITCH_CLEAR0 [
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P0_VPLUS OFFSET(0) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P1_VPLUS OFFSET(1) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P2_VPLUS OFFSET(2) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P3_VPLUS OFFSET(3) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P4_VPLUS OFFSET(4) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P5_VPLUS OFFSET(5) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P6_VPLUS OFFSET(6) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P7_VPLUS OFFSET(7) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P0_VMINUS OFFSET(8) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P1_VMINUS OFFSET(9) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P2_VMINUS OFFSET(10) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P3_VMINUS OFFSET(11) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P4_VMINUS OFFSET(12) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P5_VMINUS OFFSET(13) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P6_VMINUS OFFSET(14) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P7_VMINUS OFFSET(15) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_VSSA_VMINUS OFFSET(16) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_TEMP_VPLUS OFFSET(17) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSA_VPLUS OFFSET(18) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSB_VPLUS OFFSET(19) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSA_VMINUS OFFSET(20) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSB_VMINUS OFFSET(21) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS0_VPLUS OFFSET(22) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS1_VPLUS OFFSET(23) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS0_VMINUS OFFSET(24) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS1_VMINUS OFFSET(25) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P4_COREIO0 OFFSET(26) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P5_COREIO1 OFFSET(27) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P6_COREIO2 OFFSET(28) NUMBITS(1) [],
    /// Write '1' to clear corresponding bit in MUX_SWITCH0
    MUX_FW_P7_COREIO3 OFFSET(29) NUMBITS(1) []
],
MUX_SWITCH_DS_CTRL [
    /// for P0 switches
    MUX_DS_CTRL_P0 OFFSET(0) NUMBITS(1) [],
    /// for P1 switches
    MUX_DS_CTRL_P1 OFFSET(1) NUMBITS(1) [],
    /// for P2 switches
    MUX_DS_CTRL_P2 OFFSET(2) NUMBITS(1) [],
    /// for P3 switches
    MUX_DS_CTRL_P3 OFFSET(3) NUMBITS(1) [],
    /// for P4 switches
    MUX_DS_CTRL_P4 OFFSET(4) NUMBITS(1) [],
    /// for P5 switches
    MUX_DS_CTRL_P5 OFFSET(5) NUMBITS(1) [],
    /// for P6 switches
    MUX_DS_CTRL_P6 OFFSET(6) NUMBITS(1) [],
    /// for P7 switches
    MUX_DS_CTRL_P7 OFFSET(7) NUMBITS(1) [],
    /// for vssa switch
    MUX_DS_CTRL_VSSA OFFSET(16) NUMBITS(1) [],
    /// for temp switch
    MUX_DS_CTRL_TEMP OFFSET(17) NUMBITS(1) [],
    /// for amuxbusa switch
    MUX_DS_CTRL_AMUXBUSA OFFSET(18) NUMBITS(1) [],
    /// for amuxbusb switches
    MUX_DS_CTRL_AMUXBUSB OFFSET(19) NUMBITS(1) [],
    /// for sarbus0 switch
    MUX_DS_CTRL_SARBUS0 OFFSET(22) NUMBITS(1) [],
    /// for sarbus1 switch
    MUX_DS_CTRL_SARBUS1 OFFSET(23) NUMBITS(1) []
],
MUX_SWITCH_SQ_CTRL [
    /// for P0 switches
    MUX_SQ_CTRL_P0 OFFSET(0) NUMBITS(1) [],
    /// for P1 switches
    MUX_SQ_CTRL_P1 OFFSET(1) NUMBITS(1) [],
    /// for P2 switches
    MUX_SQ_CTRL_P2 OFFSET(2) NUMBITS(1) [],
    /// for P3 switches
    MUX_SQ_CTRL_P3 OFFSET(3) NUMBITS(1) [],
    /// for P4 switches
    MUX_SQ_CTRL_P4 OFFSET(4) NUMBITS(1) [],
    /// for P5 switches
    MUX_SQ_CTRL_P5 OFFSET(5) NUMBITS(1) [],
    /// for P6 switches
    MUX_SQ_CTRL_P6 OFFSET(6) NUMBITS(1) [],
    /// for P7 switches
    MUX_SQ_CTRL_P7 OFFSET(7) NUMBITS(1) [],
    /// for vssa switch
    MUX_SQ_CTRL_VSSA OFFSET(16) NUMBITS(1) [],
    /// for temp switch
    MUX_SQ_CTRL_TEMP OFFSET(17) NUMBITS(1) [],
    /// for amuxbusa switch
    MUX_SQ_CTRL_AMUXBUSA OFFSET(18) NUMBITS(1) [],
    /// for amuxbusb switches
    MUX_SQ_CTRL_AMUXBUSB OFFSET(19) NUMBITS(1) [],
    /// for sarbus0 switch
    MUX_SQ_CTRL_SARBUS0 OFFSET(22) NUMBITS(1) [],
    /// for sarbus1 switch
    MUX_SQ_CTRL_SARBUS1 OFFSET(23) NUMBITS(1) []
],
MUX_SWITCH_STATUS [
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P0_VPLUS OFFSET(0) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P1_VPLUS OFFSET(1) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P2_VPLUS OFFSET(2) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P3_VPLUS OFFSET(3) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P4_VPLUS OFFSET(4) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P5_VPLUS OFFSET(5) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P6_VPLUS OFFSET(6) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P7_VPLUS OFFSET(7) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P0_VMINUS OFFSET(8) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P1_VMINUS OFFSET(9) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P2_VMINUS OFFSET(10) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P3_VMINUS OFFSET(11) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P4_VMINUS OFFSET(12) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P5_VMINUS OFFSET(13) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P6_VMINUS OFFSET(14) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_P7_VMINUS OFFSET(15) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_VSSA_VMINUS OFFSET(16) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_TEMP_VPLUS OFFSET(17) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSA_VPLUS OFFSET(18) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSB_VPLUS OFFSET(19) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSA_VMINUS OFFSET(20) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_AMUXBUSB_VMINUS OFFSET(21) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS0_VPLUS OFFSET(22) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS1_VPLUS OFFSET(23) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS0_VMINUS OFFSET(24) NUMBITS(1) [],
    /// switch status of corresponding bit in MUX_SWITCH0
    MUX_FW_SARBUS1_VMINUS OFFSET(25) NUMBITS(1) []
],
ANA_TRIM0 [
    /// Attenuation cap trimming
    CAP_TRIM OFFSET(0) NUMBITS(5) [],
    /// Attenuation cap trimming
    TRIMUNIT OFFSET(5) NUMBITS(1) []
],
ANA_TRIM1 [
    /// SAR Reference buffer trim
    SAR_REF_BUF_TRIM OFFSET(0) NUMBITS(6) []
],
CHAN_CONFIG [
    /// Address of the pin to be sampled by this channel (connected to Vplus)
    POS_PIN_ADDR OFFSET(0) NUMBITS(3) [],
    /// Address of the port that contains the pin to be sampled by this channel (connect
    POS_PORT_ADDR OFFSET(4) NUMBITS(3) [
        /// SARMUX pins.
        SARMUXPins = 0,
        /// CTB0
        CTB0 = 1,
        /// CTB1
        CTB1 = 2,
        /// CTB2
        CTB2 = 3,
        /// CTB3
        CTB3 = 4,
        /// AROUTE virtual port2 (VPORT2)
        AROUTEVirtualPort2VPORT2 = 5,
        /// AROUTE virtual port1 (VPORT1)
        AROUTEVirtualPort1VPORT1 = 6,
        /// SARMUX virtual port (VPORT0)
        SARMUXVirtualPortVPORT0 = 7
    ],
    /// Differential enable for this channel.
/// If NEG_ADDR_EN=0 and this bit is 1 then POS_PIN_ADDR[0] is ignored and considere
/// - 0: The voltage on the addressed pin is measured (Single-ended) and the resulti
/// - 1: The differential voltage on the addressed pin pair is measured and the resu
    DIFFERENTIAL_EN OFFSET(8) NUMBITS(1) [],
    /// Averaging enable for this channel. If set the AVG_CNT and AVG_SHIFT settings are
    AVG_EN OFFSET(10) NUMBITS(1) [],
    /// Sample time select: select which of the 4 global sample times to use for this ch
    SAMPLE_TIME_SEL OFFSET(12) NUMBITS(2) [],
    /// Address of the neg pin to be sampled by this channel.
    NEG_PIN_ADDR OFFSET(16) NUMBITS(3) [],
    /// Address of the neg port that contains the pin to be sampled by this channel.
    NEG_PORT_ADDR OFFSET(20) NUMBITS(3) [
        /// SARMUX pins.
        SARMUXPins = 0,
        /// AROUTE virtual port2 (VPORT2)
        AROUTEVirtualPort2VPORT2 = 5,
        /// AROUTE virtual port1 (VPORT1)
        AROUTEVirtualPort1VPORT1 = 6,
        /// SARMUX virtual port (VPORT0)
        SARMUXVirtualPortVPORT0 = 7
    ],
    /// 1 - The NEG_PIN_ADDR and NEG_PORT_ADDR determines what drives the Vminus pin. Th
    NEG_ADDR_EN OFFSET(24) NUMBITS(1) [],
    /// DSI data output enable for this channel.
/// - 0: the conversion result for this channel is only stored in the channel data r
/// - 1: the conversion result for this channel is stored in the channel data regist
    DSI_OUT_EN OFFSET(31) NUMBITS(1) []
],
CHAN_WORK [
    /// SAR conversion working data of the channel. The data is written here right after
    WORK OFFSET(0) NUMBITS(16) [],
    /// mirror bit of corresponding bit in SAR_CHAN_WORK_NEWVALUE register
    CHAN_WORK_NEWVALUE_MIR OFFSET(27) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_CHAN_WORK_UPDATED register
    CHAN_WORK_UPDATED_MIR OFFSET(31) NUMBITS(1) []
],
CHAN_RESULT [
    /// SAR conversion result of the channel. The data is copied here from the WORK fiel
    RESULT OFFSET(0) NUMBITS(16) [],
    /// mirror bit of corresponding bit in SAR_CHAN_RESULT_NEWVALUE register
    CHAN_RESULT_NEWVALUE_MIR OFFSET(27) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_SATURATE_INTR register
    SATURATE_INTR_MIR OFFSET(29) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_RANGE_INTR register
    RANGE_INTR_MIR OFFSET(30) NUMBITS(1) [],
    /// mirror bit of corresponding bit in SAR_CHAN_RESULT_UPDATED register
    CHAN_RESULT_UPDATED_MIR OFFSET(31) NUMBITS(1) []
],
];

const SAR_BASE: StaticRef<SarRegisters> =
    unsafe { StaticRef::new(0x409D0000 as *const SarRegisters) };

register_structs! {
    /// PASS top-level MMIO (DSABv2, INTR)
    PassRegisters {
        /// Interrupt cause register
        (0x000 => aref_ctrl: ReadWrite<u32, AREF_CTRL::Register>),
        (0x004 => @END),
    }
}
register_bitfields![u32,
AREF_CTRL [
    ///
    AREF_MODE OFFSET(0) NUMBITS(1) [],
    ///
    AREF_BIAS_SCALE OFFSET(2) NUMBITS(2) [],
    ///
    AREF_RMB OFFSET(4) NUMBITS(3) [],
    ///
    CTB_IPTAT_SCALE OFFSET(7) NUMBITS(1) [],
    ///
    CTB_IPTAT_REDIRECT OFFSET(8) NUMBITS(8) [],
    ///
    IZTAT_SEL OFFSET(16) NUMBITS(1) [],
    ///
    CLOCK_PUMP_PERI_SEL OFFSET(19) NUMBITS(1) [],
    ///
    VREF_SEL OFFSET(20) NUMBITS(2) [],
    ///
    DEEPSLEEP_MODE OFFSET(28) NUMBITS(2) [],
    ///
    DEEPSLEEP_ON OFFSET(30) NUMBITS(1) [],
    ///
    ENABLED OFFSET(31) NUMBITS(1) []
],
];
const PASS_BASE: StaticRef<PassRegisters> =
    unsafe { StaticRef::new(0x409F0E00 as *const PassRegisters) };

pub struct Adc<'a> {
    registers: StaticRef<SarRegisters>,
    pass: StaticRef<PassRegisters>,
    channel: OptionalCell<AdcChannel>,
    adc_client: OptionalCell<&'a dyn hil::adc::Client>,
}

#[derive(PartialEq, Debug, Clone, Copy)]
pub enum AdcChannel {
    Channel0 = 0,
    Channel1 = 1,
    Unsupported,
}

impl<'a> Adc<'a> {
    pub fn new() -> Self {
        Adc {
            registers: SAR_BASE,
            pass: PASS_BASE,
            adc_client: OptionalCell::empty(),
            channel: OptionalCell::empty(),
        }
    }

    pub fn init(&self) {
        self.registers.ctrl.modify(CTRL::ENABLED::CLEAR);

        self.pass.aref_ctrl.modify(AREF_CTRL::ENABLED::SET);

        self.registers.sample_ctrl.modify(
            SAMPLE_CTRL::DIFFERENTIAL_SIGNED::SET
                + SAMPLE_CTRL::AVG_MODE::ACCUNDUMP
                + SAMPLE_CTRL::AVG_SHIFT::SET
                + SAMPLE_CTRL::AVG_CNT.val(7),
        );

        self.registers
            .sample_time01
            .modify(SAMPLE_TIME01::SAMPLE_TIME0.val(4) + SAMPLE_TIME01::SAMPLE_TIME1.val(4));
        self.registers
            .sample_time23
            .modify(SAMPLE_TIME23::SAMPLE_TIME2.val(4) + SAMPLE_TIME23::SAMPLE_TIME3.val(4));

        self.registers.chan_config[0].modify(
            CHAN_CONFIG::DIFFERENTIAL_EN::SET
                + CHAN_CONFIG::POS_PORT_ADDR.val(0)
                + CHAN_CONFIG::POS_PIN_ADDR.val(0)
                + CHAN_CONFIG::NEG_ADDR_EN::SET
                + CHAN_CONFIG::NEG_PORT_ADDR.val(0)
                + CHAN_CONFIG::NEG_PIN_ADDR.val(1)
                + CHAN_CONFIG::AVG_EN::SET,
        );

        self.registers.chan_config[1].modify(
            CHAN_CONFIG::DIFFERENTIAL_EN::SET
                + CHAN_CONFIG::POS_PORT_ADDR.val(0)
                + CHAN_CONFIG::POS_PIN_ADDR.val(2)
                + CHAN_CONFIG::NEG_ADDR_EN::SET
                + CHAN_CONFIG::NEG_PORT_ADDR.val(0)
                + CHAN_CONFIG::NEG_PIN_ADDR.val(3)
                + CHAN_CONFIG::AVG_EN::SET,
        );

        self.registers
            .range_thres
            .modify(RANGE_THRES::RANGE_LOW.val(0x800) + RANGE_THRES::RANGE_HIGH.val(0x7FF));

        self.registers.intr_mask.modify(INTR_MASK::EOS_MASK.val(1));

        self.registers.mux_switch0.modify(
            MUX_SWITCH0::MUX_FW_P0_VPLUS::SET
                + MUX_SWITCH0::MUX_FW_P2_VPLUS::SET
                + MUX_SWITCH0::MUX_FW_P1_VMINUS::SET
                + MUX_SWITCH0::MUX_FW_P3_VMINUS::SET
                + MUX_SWITCH0::MUX_FW_VSSA_VMINUS::SET,
        );

        self.registers.mux_switch_sq_ctrl.modify(
            MUX_SWITCH_SQ_CTRL::MUX_SQ_CTRL_P0::SET
                + MUX_SWITCH_SQ_CTRL::MUX_SQ_CTRL_P1::SET
                + MUX_SWITCH_SQ_CTRL::MUX_SQ_CTRL_P2::SET
                + MUX_SWITCH_SQ_CTRL::MUX_SQ_CTRL_P3::SET
                + MUX_SWITCH_SQ_CTRL::MUX_SQ_CTRL_VSSA::SET,
        );

        self.registers.ctrl.modify(
            CTRL::ENABLED::SET
                + CTRL::REFBUF_EN::SET
                + CTRL::VREF_SEL::Vdda
                + CTRL::NEG_SEL::VSSA_KELVIN
                + CTRL::VREF_BYP_CAP_EN::SET
                + CTRL::SAR_HW_CTRL_NEGVREF::SET
                + CTRL::COMP_DLY::_25nsDelayUseThisFor25Msps,
        );
    }

    fn get_sample(&self, channel: AdcChannel) -> u32 {
        self.registers.chan_result[channel as usize].read(CHAN_RESULT::RESULT)
    }

    fn start_sample(&self) {
        self.registers
            .start_ctrl
            .modify(START_CTRL::FW_TRIGGER::SET);
    }

    pub fn handle_interrupt(&self) {
        if self.registers.intr.is_set(INTR::EOS_INTR) {
            self.registers.intr.modify(INTR::EOS_INTR::SET);

            let Some(channel) = self.channel.take() else {
                return;
            };

            let sample = self.get_sample(channel);

            self.adc_client
                .map(|client| client.sample_ready(sample as u16));
        }
    }
}

impl<'a> hil::adc::Adc<'a> for Adc<'a> {
    type Channel = AdcChannel;

    fn sample(&self, channel: &Self::Channel) -> Result<(), kernel::ErrorCode> {
        if self.channel.is_some() {
            return Err(kernel::ErrorCode::BUSY);
        }

        match channel {
            AdcChannel::Unsupported => Err(kernel::ErrorCode::NOSUPPORT),
            _ => {
                self.channel.set(*channel);
                self.registers
                    .chan_en
                    .modify(CHAN_EN::CHAN_EN.val(1 << (*channel as u32)));
                self.start_sample();
                Ok(())
            }
        }
    }

    fn sample_continuous(
        &self,
        _channel: &Self::Channel,
        _frequency: u32,
    ) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }

    fn stop_sampling(&self) -> Result<(), kernel::ErrorCode> {
        Err(kernel::ErrorCode::NOSUPPORT)
    }

    fn get_resolution_bits(&self) -> usize {
        12
    }

    fn get_voltage_reference_mv(&self) -> Option<usize> {
        None
    }

    fn set_client(&self, client: &'a dyn hil::adc::Client) {
        self.adc_client.set(client);
    }
}
