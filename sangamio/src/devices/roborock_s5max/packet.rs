//! MCU report framing for the Roborock S5 Max.
//!
//! Captured reports use `0xaa` as the sync byte. Bytes after the three-byte
//! header are escaped (`a9 00` = `a9`, `a9 01` = `aa`). The length byte is the
//! unescaped payload length; one trailing byte stores the CRC.

pub const SYNC: u8 = 0xaa;

// Report identifiers observed from the S5 Max MCU and corroborated by the
// stock RoboController's MCU event vocabulary. Keep these as wire constants;
// field offsets are intentionally not guessed here.
pub const REPORT_STATE_MESSAGE: u8 = 0x07;
pub const REPORT_STATE_COMMAND: u8 = 0x40;
pub const REPORT_EVENT_MESSAGE: u8 = 0x0b;
pub const REPORT_EVENT_COMMAND: u8 = 0x08;
pub const SYSTEM_MODE_REPORT_MESSAGE: u8 = 0x02;
pub const SYSTEM_MODE_REPORT_LENGTH: u8 = 0x08;
pub const INTERNAL_ERROR_REPORT_MESSAGE: u8 = 0x04;
pub const INTERNAL_ERROR_REPORT_LENGTH: u8 = 0x04;
pub const BLACK_BOX_REPORT_MESSAGE: u8 = 0x0b;
pub const BLACK_BOX_REPORT_LENGTH: u8 = 0x08;
pub const DEVICE_INFO_REPORT_MESSAGE: u8 = 0x0d;
pub const DEVICE_INFO_REPORT_LENGTH: u8 = 0x10;
pub const DOCK_VOLTAGE_REPORT_MESSAGE: u8 = 0x0f;
pub const DOCK_VOLTAGE_REPORT_LENGTH: u8 = 0x02;
pub const BATTERY_CAPACITY_REPORT_MESSAGE: u8 = 0x16;
pub const BATTERY_CAPACITY_REPORT_LENGTH: u8 = 0x02;
/// Compact BMS report: voltage, current, state of charge, and reserved bytes.
pub const BATTERY_REPORT_MESSAGE: u8 = 0x08;
pub const BATTERY_REPORT_LENGTH: u8 = 0x0a;
/// Stock MCU wheel report produced by `FUN_0800e2dc`.
pub const WHEEL_REPORT_MESSAGE: u8 = 0x51;
pub const WHEEL_REPORT_LENGTH: u8 = 0x08;
pub const FAN_REPORT_MESSAGE: u8 = 0x50;
pub const FAN_REPORT_LENGTH: u8 = 0x06;
pub const BRUSH_REPORT_MESSAGE: u8 = 0x52;
pub const BRUSH_REPORT_LENGTH: u8 = 0x06;
pub const SWEEP_REPORT_MESSAGE: u8 = 0x53;
pub const SWEEP_REPORT_LENGTH: u8 = 0x06;
/// Raw wall-sensor ADC report produced from the MCU ADC/DMA buffer.
pub const WALL_SENSOR_REPORT_MESSAGE: u8 = 0x42;
pub const WALL_SENSOR_REPORT_LENGTH: u8 = 0x02;
/// Gyro-communication token echoed by the MCU to the application processor.
pub const GYRO_ECHO_REPORT_MESSAGE: u8 = 0xd1;
pub const GYRO_ECHO_REPORT_LENGTH: u8 = 0x01;
pub const MCU_TIME_REPORT_MESSAGE: u8 = 0xd2;
pub const MCU_TIME_REPORT_LENGTH: u8 = 0x08;
pub const CALIBRATION_REPORT_MESSAGE: u8 = 0xd5;
pub const CALIBRATION_REPORT_LENGTH: u8 = 0x1c;
pub const MCU_IDENTITY_REPORT_MESSAGE: u8 = 0xf1;
pub const MCU_IDENTITY_REPORT_LENGTH: u8 = 0x10;
pub const STATUS_STRING_REPORT_MESSAGE: u8 = 0xf6;
/// Rolling dock-IR code masks from the left and right receivers.
pub const DOCK_IR_REPORT_MESSAGE: u8 = 0x13;
pub const DOCK_IR_REPORT_LENGTH: u8 = 0x04;
pub const REPORT_FOOTER_MESSAGE: u8 = 0xd0;
pub const REPORT_FOOTER_LENGTH: u8 = 0x02;
pub const LOG_ESCAPE_OVERHEAD: u8 = 0x00;
pub const LOG_MESSAGE: u8 = 0xf9;
pub const LOG_SUBTYPE_DEBUG: u8 = 0x80;
pub const LOG_SUBTYPE_NOTICE: u8 = 0x81;
pub const STATE_BASE_LENGTH: usize = 66;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub raw: Vec<u8>,
    pub payload: Vec<u8>,
    /// Normal framing stores one plus the number of A9/AA escape expansions.
    /// Text-log framing stores zero here and uses a trailing sentinel instead.
    pub escape_overhead: u8,
    pub message: u8,
    pub command: u8,
    pub crc: u8,
    pub crc_valid: bool,
    pub is_log: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuState {
    /// Three little-endian IEEE-754 values at payload offsets 2..14.
    pub acceleration: [f32; 3],
    /// Three little-endian IEEE-754 values at payload offsets 14..26.
    pub angular_rate: [f32; 3],
    /// Four little-endian IEEE-754 values at payload offsets 26..42.
    pub quaternion: [f32; 4],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateReport {
    pub imu: ImuState,
    /// Signed cumulative encoder counters at payload offsets 42 and 46.
    pub left_odometry_ticks: i32,
    pub right_odometry_ticks: i32,
    /// Filtered controller-domain forward-motion estimate at offset 50.
    /// It closely tracks mean encoder ticks per 20 ms, but no physical-unit
    /// scale is assigned yet.
    pub forward_motion: f32,
    /// Payload offset 54; zero in every available capture.
    pub reserved: u32,
    /// Monotonic MCU time in milliseconds at payload offset 58.
    pub mcu_timestamp_ms: u64,
}

/// Eight-byte wheel report emitted by the stock MCU firmware.
///
/// The producer function converts two wheel-current ADC channels to mA,
/// reads and signs the active forward/reverse PWM compare level, then appends
/// fault-state bytes. Encoder velocity is reported through separate counters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WheelReport {
    pub bytes: [u8; 8],
}

impl WheelReport {
    pub const fn left_current_ma(self) -> u16 {
        u16::from_le_bytes([self.bytes[0], self.bytes[1]])
    }

    pub const fn right_current_ma(self) -> u16 {
        u16::from_le_bytes([self.bytes[2], self.bytes[3]])
    }

    /// Signed wheel drive level reconstructed from the active timer compare.
    /// This is actuator demand, not measured encoder velocity.
    pub const fn left_drive_level(self) -> i8 {
        self.bytes[4] as i8
    }

    pub const fn right_drive_level(self) -> i8 {
        self.bytes[5] as i8
    }

    /// Compatibility alias retained for callers of the initial passive probe.
    #[deprecated(note = "use left_drive_level(); this field is drive demand, not wheel speed")]
    pub const fn left_speed(self) -> i8 {
        self.left_drive_level()
    }

    /// Compatibility alias retained for callers of the initial passive probe.
    #[deprecated(note = "use right_drive_level(); this field is drive demand, not wheel speed")]
    pub const fn right_speed(self) -> i8 {
        self.right_drive_level()
    }

    /// Fault/result code. Zero is normal in available captures.
    pub const fn left_fault(self) -> u8 {
        self.bytes[6]
    }

    pub const fn right_fault(self) -> u8 {
        self.bytes[7]
    }
}

/// Shared six-byte report layout used by fan, main brush, and side brush.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MotorReport {
    pub sentinel: u16,
    pub current_ma: u16,
    pub fault: u8,
    pub speed: u8,
}

/// Ten-byte BMS summary copied into report `0x08` by the stock firmware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryReport {
    pub voltage_mv: u16,
    /// Current magnitude reported by the BMS. Direction is represented by
    /// charging state elsewhere in the stock protocol.
    pub current_ma: u16,
    pub state_of_charge_percent: u8,
    pub reserved: [u8; 5],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WallSensorReport {
    /// Raw ADC-domain proximity value; no calibrated distance scale is known.
    pub raw_adc: u16,
}

/// One-byte token copied from an AP command and returned as report `0xd1`.
///
/// Firmware does not derive this byte from an IMU register, despite `0xd1`
/// also being the BMI160 chip ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct GyroEchoReport {
    pub token: u8,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SystemMode {
    Normal,
    Idle,
    Factory,
    ManualBuiltInTest,
    AutomaticBuiltInTest,
    MobilityTest,
    Shutdown,
    ReservedRejected,
    EnergyEfficiency,
    RetreadingFactory,
    StartKeyOnlyBoot,
    WatchdogResetNoDock,
    PinResetNoDock,
    SoftwareResetNoDock,
    DockOnlyBoot,
    WatchdogResetDocked,
    PinResetDocked,
    SoftwareResetDocked,
    ApPowerOnAfterCommunicationFailure,
    Unknown(u8),
}

impl SystemMode {
    pub const fn from_wire(value: u8) -> Self {
        match value {
            0x00 => Self::Normal,
            0x01 => Self::Idle,
            0x02 => Self::Factory,
            0x03 => Self::ManualBuiltInTest,
            0x04 => Self::AutomaticBuiltInTest,
            0x05 => Self::MobilityTest,
            0x06 => Self::Shutdown,
            0x07 => Self::ReservedRejected,
            0x08 => Self::EnergyEfficiency,
            0x09 => Self::RetreadingFactory,
            0x20 => Self::StartKeyOnlyBoot,
            0x21 => Self::WatchdogResetNoDock,
            0x22 => Self::PinResetNoDock,
            0x23 => Self::SoftwareResetNoDock,
            0x40 => Self::DockOnlyBoot,
            0x41 => Self::WatchdogResetDocked,
            0x42 => Self::PinResetDocked,
            0x43 => Self::SoftwareResetDocked,
            0x44 => Self::ApPowerOnAfterCommunicationFailure,
            value => Self::Unknown(value),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SystemModeReport {
    pub mode: SystemMode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct InternalErrorReport {
    pub raw: u32,
}

impl InternalErrorReport {
    pub const fn test_info_invalid(self) -> bool {
        self.raw & 0x01 != 0
    }

    pub const fn gyro_probe_failed(self) -> bool {
        self.raw & 0x02 != 0
    }

    pub const fn bms_communication_failed(self) -> bool {
        self.raw & 0x04 != 0
    }

    /// Compatibility bits consumed by Linux but not set by this MCU image.
    pub const fn compatibility_bits(self) -> u8 {
        ((self.raw >> 3) & 0x07) as u8
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BlackBoxReport {
    /// Zero selects Linux internal event 0x16; nonzero selects event 0x17.
    /// Their user-facing meanings are not present in the local binaries.
    pub selector: u8,
    pub reserved: [u8; 3],
    pub value: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceInfoReport {
    pub bytes: [u8; 16],
}

impl DeviceInfoReport {
    pub const fn record_type(self) -> u8 {
        self.bytes[0]
    }

    pub const fn separately_published_value(self) -> u8 {
        self.bytes[4]
    }

    pub const fn subtype_event_value(self) -> u8 {
        self.bytes[5]
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DockVoltageReport {
    pub millivolts: u16,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BatteryCapacityReport {
    pub raw_units_of_100: u8,
    pub ignored_by_stock_linux: u8,
}

impl BatteryCapacityReport {
    /// Stock Linux publishes byte zero multiplied by 100 mAh. The resulting
    /// 5200 mAh matches Roborock's official S5 Max battery specification.
    pub const fn stock_scaled_value(self) -> u16 {
        self.raw_units_of_100 as u16 * 100
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuTimeReport {
    pub timestamp: u64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CalibrationReport {
    pub cliff_thresholds: [u16; 4],
    pub wall_sensor_one_point: [u8; 4],
    pub bmi160_sensitivity: f32,
    pub bmi160_accel_offsets: [i32; 3],
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct McuIdentityReport {
    pub jtag_device_id: u32,
    pub flash_size_register: u32,
    pub chip_signature: u32,
    pub reserved: u32,
}

/// Decoded dock-beacon pulse classes accumulated by one IR receiver.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DockIrCodeMask(pub u8);

impl DockIrCodeMask {
    /// Firmware pulse code `0x81`, labeled `L` in receiver diagnostics.
    pub const fn beacon_left(self) -> bool {
        self.0 & 0x01 != 0
    }

    /// Firmware pulse code `0x82`, labeled `R` in receiver diagnostics.
    pub const fn beacon_right(self) -> bool {
        self.0 & 0x02 != 0
    }

    /// Firmware pulse code `0x84`; physical beacon meaning is unresolved.
    pub const fn code_84(self) -> bool {
        self.0 & 0x04 != 0
    }

    /// Firmware pulse code `0x88`; physical beacon meaning is unresolved.
    pub const fn code_88(self) -> bool {
        self.0 & 0x08 != 0
    }

    /// Non-one-hot majority-pattern class; physical meaning is unresolved.
    pub const fn majority_pattern(self) -> bool {
        self.0 & 0x10 != 0
    }
}

/// Four-byte report `0x13`, built from a rolling six-entry OR history.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DockIrReport {
    pub left_receiver: DockIrCodeMask,
    pub right_receiver: DockIrCodeMask,
    pub reserved_byte_1: u8,
    pub reserved_byte_3: u8,
}

impl MotorReport {
    fn parse(bytes: &[u8]) -> Option<Self> {
        Some(Self {
            sentinel: u16::from_le_bytes(bytes.get(0..2)?.try_into().ok()?),
            current_ma: u16::from_le_bytes(bytes.get(2..4)?.try_into().ok()?),
            fault: *bytes.get(4)?,
            speed: *bytes.get(5)?,
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ReportFooter {
    pub reports_pending: bool,
    /// Increments from 1 through 255 and skips zero.
    pub sequence: u8,
}

/// One length-delimited report appended to the fixed periodic-state body.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EmbeddedReport<'a> {
    pub message: u8,
    pub data: &'a [u8],
}

/// Iterator over the exact `[message, length, data...]` stream following the
/// 66-byte periodic-state body. A truncated final report terminates iteration.
pub struct EmbeddedReports<'a> {
    remaining: &'a [u8],
}

impl<'a> Iterator for EmbeddedReports<'a> {
    type Item = EmbeddedReport<'a>;

    fn next(&mut self) -> Option<Self::Item> {
        let (&message, rest) = self.remaining.split_first()?;
        let (&length, rest) = rest.split_first()?;
        let length = length as usize;
        let (data, remaining) = rest.split_at_checked(length)?;
        self.remaining = remaining;
        Some(EmbeddedReport { message, data })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DockState {
    Connect,
    Supply,
    Charge,
    Discharge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WarningCounters {
    pub wheel: Option<[u32; 5]>,
    pub brush: Option<[u32; 3]>,
}

/// Stock wheel-control diagnostic emitted by the MCU logger.
///
/// The values are controller-domain integers; their physical units are not
/// assumed here. They are useful for correlating the embedded 0x51 report.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WheelPidDiagnostic {
    pub left: i32,
    pub right: i32,
    pub count: i32,
}

/// Compact sensor summary emitted by the MCU logger.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorSummary {
    pub force: u32,
    pub drop: u32,
    pub bumper: u32,
    pub dock: u32,
    pub cliff: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BmsDiagnostic {
    BatteryStatus {
        soc_percent: i32,
        voltage_mv: i32,
        current_ma: i32,
    },
    AdapterVoltageMv(u32),
    BatteryTemperatureC(i32),
    ModeTransition {
        from: u8,
        to: u8,
    },
}

impl DockState {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Connect => "connect",
            Self::Supply => "supply",
            Self::Charge => "charge",
            Self::Discharge => "discharge",
        }
    }

    fn from_log_text(text: &[u8]) -> Option<Self> {
        let text = text.strip_suffix(b"\r\n").unwrap_or(text);
        match text {
            b"DockSta:1,CONNECT" => Some(Self::Connect),
            b"DockSta:2,SUPPLY" => Some(Self::Supply),
            b"DockSta:3,CHARGE" => Some(Self::Charge),
            b"DockSta:4,DISCHARGE" => Some(Self::Discharge),
            _ => None,
        }
    }
}

impl Frame {
    pub fn escape_overhead_valid(&self) -> bool {
        self.is_log
            || self.raw.len().checked_sub(self.payload.len() + 3)
                == Some(self.escape_overhead as usize)
    }

    pub fn log_subtype(&self) -> Option<u8> {
        self.is_log.then_some(self.command)
    }

    pub fn is_state_report(&self) -> bool {
        !self.is_log && self.message == REPORT_STATE_MESSAGE && self.command == REPORT_STATE_COMMAND
    }

    pub fn is_event_report(&self) -> bool {
        !self.is_log && self.message == REPORT_EVENT_MESSAGE && self.command == REPORT_EVENT_COMMAND
    }

    pub fn is_wheel_report(&self) -> bool {
        self.message == WHEEL_REPORT_MESSAGE && self.command == WHEEL_REPORT_LENGTH
    }

    pub fn wheel_report(&self) -> Option<WheelReport> {
        if !self.is_wheel_report() || self.payload.len() < 2 + WHEEL_REPORT_LENGTH as usize {
            return None;
        }
        Some(WheelReport {
            bytes: self.payload[2..10].try_into().ok()?,
        })
    }

    /// Extracts the embedded wheel subreport from a periodic state packet.
    ///
    /// The stock firmware commonly appends reports as `[message, length,
    /// bytes...]` after the main state payload. In particular, wheel data is
    /// emitted as `51 08` followed by eight opaque bytes.
    pub fn embedded_wheel_report(&self) -> Option<WheelReport> {
        Some(WheelReport {
            bytes: self
                .embedded_report(WHEEL_REPORT_MESSAGE, WHEEL_REPORT_LENGTH)?
                .try_into()
                .ok()?,
        })
    }

    /// Returns a raw TLV report from a binary frame.
    ///
    /// Periodic state packets append reports after their fixed body; standalone
    /// event packets begin directly with a TLV. Unknown reports can be queried
    /// without assigning guessed semantics.
    pub fn embedded_report(&self, message: u8, length: u8) -> Option<&[u8]> {
        self.reports()
            .find(|report| report.message == message && report.data.len() == length as usize)
            .map(|report| report.data)
    }

    /// Returns the first report with this message ID regardless of length.
    pub fn report(&self, message: u8) -> Option<EmbeddedReport<'_>> {
        self.reports().find(|report| report.message == message)
    }

    /// Iterates the TLV reports in any normal binary frame. Periodic state
    /// frames have a fixed 66-byte prefix; all other observed binary frames
    /// begin directly with their first TLV.
    pub fn reports(&self) -> EmbeddedReports<'_> {
        let start = if self.is_state_report() {
            STATE_BASE_LENGTH
        } else {
            0
        };
        let remaining = if !self.is_log && self.payload.len() >= start {
            &self.payload[start..]
        } else {
            &[]
        };
        EmbeddedReports { remaining }
    }

    /// Backward-compatible, state-specific name for [`Self::reports`].
    pub fn embedded_reports(&self) -> EmbeddedReports<'_> {
        let remaining = if self.is_state_report() && self.payload.len() >= STATE_BASE_LENGTH {
            &self.payload[STATE_BASE_LENGTH..]
        } else {
            &[]
        };
        EmbeddedReports { remaining }
    }

    pub fn embedded_motor_report(&self, message: u8) -> Option<MotorReport> {
        let length = match message {
            FAN_REPORT_MESSAGE => FAN_REPORT_LENGTH,
            BRUSH_REPORT_MESSAGE => BRUSH_REPORT_LENGTH,
            SWEEP_REPORT_MESSAGE => SWEEP_REPORT_LENGTH,
            _ => return None,
        };
        MotorReport::parse(self.embedded_report(message, length)?)
    }

    pub fn battery_report(&self) -> Option<BatteryReport> {
        let bytes = self.embedded_report(BATTERY_REPORT_MESSAGE, BATTERY_REPORT_LENGTH)?;
        Some(BatteryReport {
            voltage_mv: u16::from_le_bytes(bytes[0..2].try_into().ok()?),
            current_ma: u16::from_le_bytes(bytes[2..4].try_into().ok()?),
            state_of_charge_percent: bytes[4],
            reserved: bytes[5..10].try_into().ok()?,
        })
    }

    pub fn wall_sensor_report(&self) -> Option<WallSensorReport> {
        let bytes = self.embedded_report(WALL_SENSOR_REPORT_MESSAGE, WALL_SENSOR_REPORT_LENGTH)?;
        Some(WallSensorReport {
            raw_adc: u16::from_le_bytes(bytes.try_into().ok()?),
        })
    }

    pub fn gyro_echo_report(&self) -> Option<GyroEchoReport> {
        let bytes = self.embedded_report(GYRO_ECHO_REPORT_MESSAGE, GYRO_ECHO_REPORT_LENGTH)?;
        Some(GyroEchoReport { token: bytes[0] })
    }

    pub fn system_mode_report(&self) -> Option<SystemModeReport> {
        let bytes = self.embedded_report(SYSTEM_MODE_REPORT_MESSAGE, SYSTEM_MODE_REPORT_LENGTH)?;
        (bytes[..7] == *b"sys_md\0").then(|| SystemModeReport {
            mode: SystemMode::from_wire(bytes[7]),
        })
    }

    pub fn internal_error_report(&self) -> Option<InternalErrorReport> {
        let bytes =
            self.embedded_report(INTERNAL_ERROR_REPORT_MESSAGE, INTERNAL_ERROR_REPORT_LENGTH)?;
        Some(InternalErrorReport {
            raw: u32::from_le_bytes(bytes.try_into().ok()?),
        })
    }

    pub fn black_box_report(&self) -> Option<BlackBoxReport> {
        let bytes = self.embedded_report(BLACK_BOX_REPORT_MESSAGE, BLACK_BOX_REPORT_LENGTH)?;
        Some(BlackBoxReport {
            selector: bytes[0],
            reserved: bytes[1..4].try_into().ok()?,
            value: u32::from_le_bytes(bytes[4..8].try_into().ok()?),
        })
    }

    pub fn device_info_report(&self) -> Option<DeviceInfoReport> {
        let bytes = self.embedded_report(DEVICE_INFO_REPORT_MESSAGE, DEVICE_INFO_REPORT_LENGTH)?;
        Some(DeviceInfoReport {
            bytes: bytes.try_into().ok()?,
        })
    }

    pub fn dock_voltage_report(&self) -> Option<DockVoltageReport> {
        let bytes =
            self.embedded_report(DOCK_VOLTAGE_REPORT_MESSAGE, DOCK_VOLTAGE_REPORT_LENGTH)?;
        Some(DockVoltageReport {
            millivolts: u16::from_le_bytes(bytes.try_into().ok()?),
        })
    }

    pub fn battery_capacity_report(&self) -> Option<BatteryCapacityReport> {
        let bytes = self.embedded_report(
            BATTERY_CAPACITY_REPORT_MESSAGE,
            BATTERY_CAPACITY_REPORT_LENGTH,
        )?;
        Some(BatteryCapacityReport {
            raw_units_of_100: bytes[0],
            ignored_by_stock_linux: bytes[1],
        })
    }

    pub fn mcu_time_report(&self) -> Option<McuTimeReport> {
        let bytes = self.embedded_report(MCU_TIME_REPORT_MESSAGE, MCU_TIME_REPORT_LENGTH)?;
        Some(McuTimeReport {
            timestamp: u64::from_le_bytes(bytes.try_into().ok()?),
        })
    }

    pub fn calibration_report(&self) -> Option<CalibrationReport> {
        let bytes = self.embedded_report(CALIBRATION_REPORT_MESSAGE, CALIBRATION_REPORT_LENGTH)?;
        Some(CalibrationReport {
            cliff_thresholds: [
                u16::from_le_bytes(bytes[0..2].try_into().ok()?),
                u16::from_le_bytes(bytes[2..4].try_into().ok()?),
                u16::from_le_bytes(bytes[4..6].try_into().ok()?),
                u16::from_le_bytes(bytes[6..8].try_into().ok()?),
            ],
            wall_sensor_one_point: bytes[8..12].try_into().ok()?,
            bmi160_sensitivity: f32::from_le_bytes(bytes[12..16].try_into().ok()?),
            bmi160_accel_offsets: [
                i32::from_le_bytes(bytes[16..20].try_into().ok()?),
                i32::from_le_bytes(bytes[20..24].try_into().ok()?),
                i32::from_le_bytes(bytes[24..28].try_into().ok()?),
            ],
        })
    }

    pub fn mcu_identity_report(&self) -> Option<McuIdentityReport> {
        let bytes =
            self.embedded_report(MCU_IDENTITY_REPORT_MESSAGE, MCU_IDENTITY_REPORT_LENGTH)?;
        Some(McuIdentityReport {
            jtag_device_id: u32::from_le_bytes(bytes[0..4].try_into().ok()?),
            flash_size_register: u32::from_le_bytes(bytes[4..8].try_into().ok()?),
            chip_signature: u32::from_le_bytes(bytes[8..12].try_into().ok()?),
            reserved: u32::from_le_bytes(bytes[12..16].try_into().ok()?),
        })
    }

    /// Returns the text body of variable-length report `0xf6` up to its NUL.
    pub fn status_string_report(&self) -> Option<&[u8]> {
        let bytes = self.report(STATUS_STRING_REPORT_MESSAGE)?.data;
        let nul = bytes.iter().position(|byte| *byte == 0)?;
        Some(&bytes[..nul])
    }

    pub fn dock_ir_report(&self) -> Option<DockIrReport> {
        let bytes = self.embedded_report(DOCK_IR_REPORT_MESSAGE, DOCK_IR_REPORT_LENGTH)?;
        Some(DockIrReport {
            left_receiver: DockIrCodeMask(bytes[0]),
            reserved_byte_1: bytes[1],
            right_receiver: DockIrCodeMask(bytes[2]),
            reserved_byte_3: bytes[3],
        })
    }

    pub fn report_footer(&self) -> Option<ReportFooter> {
        let bytes = self.embedded_report(REPORT_FOOTER_MESSAGE, REPORT_FOOTER_LENGTH)?;
        Some(ReportFooter {
            reports_pending: bytes[0] != 0,
            sequence: bytes[1],
        })
    }

    /// Returns the printable body of a stock MCU diagnostic packet.
    ///
    /// These packets use the final `0x5e` as a text terminator and do not use
    /// the normal report CRC convention, so callers should only use this for
    /// frames where `is_log` is true.
    pub fn log_text(&self) -> Option<&[u8]> {
        if !self.is_log || self.payload.len() < 2 {
            return None;
        }
        let text = &self.payload[2..];
        Some(text.strip_suffix(&[0x5e]).unwrap_or(text))
    }

    pub fn dock_state(&self) -> Option<DockState> {
        DockState::from_log_text(self.log_text()?)
    }

    pub fn warning_counters(&self) -> Option<WarningCounters> {
        let text = self.log_text()?;
        let text = text
            .strip_suffix(b"\r\n")
            .or_else(|| text.strip_suffix(b"\n"))
            .unwrap_or(text);
        if let Some(values) = parse_csv::<5>(text, b"WheelWarnSum:") {
            return Some(WarningCounters {
                wheel: Some(values),
                brush: None,
            });
        }
        if let Some(values) = parse_csv::<3>(text, b"BrushWarnSum:") {
            return Some(WarningCounters {
                wheel: None,
                brush: Some(values),
            });
        }
        None
    }

    pub fn wheel_pid_diagnostic(&self) -> Option<WheelPidDiagnostic> {
        let text = self
            .log_text()?
            .strip_suffix(b"\r\n")
            .unwrap_or(self.log_text()?);
        let text = text.strip_prefix(b"[PID_wheeloc,L,")?;
        let (left, text) = split_number(text, b",R,")?;
        let (right, text) = split_number(text, b",cnt,")?;
        let count = parse_number(text.strip_suffix(b",")?)?;
        Some(WheelPidDiagnostic { left, right, count })
    }

    pub fn sensor_summary(&self) -> Option<SensorSummary> {
        let text = self
            .log_text()?
            .strip_suffix(b"\r\n")
            .unwrap_or(self.log_text()?);
        let text = text.strip_prefix(b"Force:")?;
        let (force, text) = split_number::<u32>(text, b" Drop:")?;
        let (drop, text) = split_number::<u32>(text, b" Bmpr:")?;
        let (bumper, text) = split_number::<u32>(text, b" Dock:")?;
        let (dock, text) = split_number::<u32>(text, b" Clf:")?;
        let cliff: u32 = parse_number(text)?;
        Some(SensorSummary {
            force,
            drop,
            bumper,
            dock,
            cliff,
        })
    }

    pub fn bms_diagnostic(&self) -> Option<BmsDiagnostic> {
        let text = self.log_text()?;
        let text = text
            .strip_suffix(b"\r\n")
            .or_else(|| text.strip_suffix(b"\n"))
            .unwrap_or(text);

        if let Some((soc_percent, voltage_mv, current_ma)) = parse_battery_status(text) {
            return Some(BmsDiagnostic::BatteryStatus {
                soc_percent,
                voltage_mv,
                current_ma,
            });
        }

        if let Some(value) = parse_decimal_suffix(text, b"bms:rept RPT Vadpt(on) = ", b"mV!") {
            return Some(BmsDiagnostic::AdapterVoltageMv(value));
        }
        if let Some(value) = parse_decimal_suffix(text, b"bms:chkt, bat temp: ", b"c") {
            return Some(BmsDiagnostic::BatteryTemperatureC(value));
        }
        let mode = text.strip_prefix(b"bms:stam mode[")?.strip_suffix(b"]")?;
        let separator = mode.windows(2).position(|pair| pair == b"->")?;
        let from = std::str::from_utf8(&mode[..separator]).ok()?.parse().ok()?;
        let to = std::str::from_utf8(&mode[separator + 2..])
            .ok()?
            .parse()
            .ok()?;
        Some(BmsDiagnostic::ModeTransition { from, to })
    }

    pub fn imu_state(&self) -> Option<ImuState> {
        if !self.is_state_report() || self.payload.len() < 42 {
            return None;
        }

        let mut values = [0.0; 10];
        for (index, value) in values.iter_mut().enumerate() {
            let offset = 2 + index * 4;
            *value = f32::from_le_bytes(self.payload[offset..offset + 4].try_into().ok()?);
        }
        Some(ImuState {
            acceleration: values[0..3].try_into().ok()?,
            angular_rate: values[3..6].try_into().ok()?,
            quaternion: values[6..10].try_into().ok()?,
        })
    }

    pub fn state_report(&self) -> Option<StateReport> {
        if !self.is_state_report() || self.payload.len() < STATE_BASE_LENGTH {
            return None;
        }
        let imu = self.imu_state()?;
        Some(StateReport {
            imu,
            left_odometry_ticks: i32::from_le_bytes(self.payload[42..46].try_into().ok()?),
            right_odometry_ticks: i32::from_le_bytes(self.payload[46..50].try_into().ok()?),
            forward_motion: f32::from_le_bytes(self.payload[50..54].try_into().ok()?),
            reserved: u32::from_le_bytes(self.payload[54..58].try_into().ok()?),
            mcu_timestamp_ms: u64::from_le_bytes(self.payload[58..66].try_into().ok()?),
        })
    }
}

fn parse_csv<const N: usize>(text: &[u8], prefix: &[u8]) -> Option<[u32; N]> {
    let values = text
        .strip_prefix(prefix)?
        .split(|byte| *byte == b',')
        .map(|value| std::str::from_utf8(value).ok()?.parse::<u32>().ok())
        .collect::<Option<Vec<_>>>()?;
    values.try_into().ok()
}

fn parse_number<T>(value: &[u8]) -> Option<T>
where
    T: std::str::FromStr,
{
    std::str::from_utf8(value).ok()?.parse().ok()
}

fn split_number<'a, T>(text: &'a [u8], separator: &[u8]) -> Option<(T, &'a [u8])>
where
    T: std::str::FromStr,
{
    let position = text
        .windows(separator.len())
        .position(|window| window == separator)?;
    Some((
        parse_number(&text[..position])?,
        &text[position + separator.len()..],
    ))
}

fn parse_decimal_suffix<T>(text: &[u8], prefix: &[u8], suffix: &[u8]) -> Option<T>
where
    T: std::str::FromStr,
{
    let value = text.strip_prefix(prefix)?.strip_suffix(suffix)?;
    std::str::from_utf8(value).ok()?.parse().ok()
}

fn parse_battery_status(text: &[u8]) -> Option<(i32, i32, i32)> {
    let text = std::str::from_utf8(text).ok()?;
    let text = text.strip_prefix("bms:rept soc=")?;
    let (soc, text) = text.split_once(",V=")?;
    let (voltage, current) = text.split_once(",I=")?;
    Some((
        soc.parse().ok()?,
        voltage.parse().ok()?,
        current.parse().ok()?,
    ))
}

#[derive(Debug, Default)]
pub struct FrameDecoder {
    buffer: Vec<u8>,
}

impl FrameDecoder {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, bytes: &[u8]) -> Vec<Frame> {
        self.buffer.extend_from_slice(bytes);
        let mut frames = Vec::new();

        loop {
            let Some(sync) = self.buffer.iter().position(|byte| *byte == SYNC) else {
                self.buffer.clear();
                break;
            };
            if sync != 0 {
                self.buffer.drain(..sync);
            }
            if self.buffer.len() < 2 {
                break;
            }

            let decoded_len = self.buffer[1] as usize + 1; // payload + CRC
            if !(4..=1024).contains(&decoded_len) {
                self.buffer.drain(..1);
                continue;
            }

            let mut cursor = 3;
            let mut decoded = Vec::with_capacity(decoded_len);
            while cursor < self.buffer.len() && decoded.len() < decoded_len {
                let byte = self.buffer[cursor];
                cursor += 1;
                if byte == 0xa9 {
                    if cursor == self.buffer.len() {
                        // Wait for the escaped byte in the next FIFO chunk.
                        cursor -= 1;
                        break;
                    }
                    let escaped = self.buffer[cursor];
                    cursor += 1;
                    match escaped {
                        0x00 => decoded.push(0xa9),
                        0x01 => decoded.push(0xaa),
                        _ => {
                            self.buffer.drain(..1);
                            decoded.clear();
                            cursor = 0;
                            break;
                        }
                    }
                } else {
                    decoded.push(byte);
                }
            }
            if cursor == 0 {
                continue;
            }
            if decoded.len() < decoded_len {
                break;
            }

            let raw: Vec<u8> = self.buffer.drain(..cursor).collect();
            let crc = decoded.pop().expect("validated decoded length");
            let calculated = crc8(&decoded, 0);
            let escape_overhead = raw[2];
            let message = decoded[0];
            let command = decoded[1];
            let is_log = escape_overhead == LOG_ESCAPE_OVERHEAD
                && message == LOG_MESSAGE
                && (LOG_SUBTYPE_DEBUG..=0x87).contains(&command);
            frames.push(Frame {
                escape_overhead,
                message,
                command,
                payload: decoded,
                crc,
                crc_valid: calculated == crc,
                is_log,
                raw,
            });
        }

        frames
    }
}

/// The MCU library uses the reflected polynomial 0x8c and an initial value
/// supplied by the caller. Captured report packets use initial value zero.
pub fn crc8(bytes: &[u8], initial: u8) -> u8 {
    let mut crc = initial;
    for byte in bytes {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 1 != 0 {
                (crc >> 1) ^ 0x8c
            } else {
                crc >> 1
            };
        }
    }
    crc
}

#[cfg(test)]
mod tests {
    use super::{Frame, FrameDecoder, crc8};

    const IDLE_REPORT: &str = "aa46020740d753733e9f305cbf91601f4158a08bbad00c8cba56260236a5d217bd57f3e5bca901d8bf3e000d6d3f811d0700885d07000100000000000000a820e40200000000d00200d69b";

    fn bytes(hex: &str) -> Vec<u8> {
        (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).unwrap())
            .collect()
    }

    #[test]
    fn decodes_split_idle_report() {
        let input = bytes(IDLE_REPORT);
        let mut decoder = FrameDecoder::new();
        assert!(decoder.push(&input[..17]).is_empty());
        let frames = decoder.push(&input[17..]);
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].raw.len(), 75);
        assert_eq!(frames[0].payload.len(), 70);
        assert_eq!(
            (
                frames[0].escape_overhead,
                frames[0].message,
                frames[0].command
            ),
            (2, 7, 0x40)
        );
        assert!(frames[0].escape_overhead_valid());
        assert!(frames[0].crc_valid);
    }

    #[test]
    fn crc_matches_captured_report() {
        let input = bytes(IDLE_REPORT);
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert_eq!(crc8(&frame.payload, 0), frame.crc);
        assert!(frame.crc_valid);
    }

    #[test]
    fn recovers_after_a_corrupt_frame() {
        let valid = bytes(IDLE_REPORT);
        let mut corrupt = valid.clone();
        let last = corrupt.len() - 1;
        corrupt[last] ^= 1;
        corrupt.extend_from_slice(&valid);

        let frames = FrameDecoder::new().push(&corrupt);
        assert_eq!(frames.len(), 2);
        assert!(!frames[0].crc_valid);
        assert!(frames[1].crc_valid);
        assert_eq!(frames[1].raw, valid);
    }

    #[test]
    fn classifies_bms_log_separately() {
        let input = bytes(
            "aa3100f981626d733a6364304420303832342d3420363930372d343634322d39362031363339302d3734302d34332d34320d0a5eaa",
        );
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert!(frame.is_log);
        assert!(!frame.crc_valid);
        assert!(!frame.is_state_report());
    }

    #[test]
    fn classifies_debug_subtype_as_text_log() {
        let input = bytes(
            "aa2400f9807069645f736574706172616d3a31382e30302c313333302e30302c312e30300d0a5eaa",
        );
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert!(frame.is_log);
        assert_eq!(frame.log_subtype(), Some(super::LOG_SUBTYPE_DEBUG));
        assert_eq!(
            frame.log_text(),
            Some(&b"pid_setparam:18.00,1330.00,1.00\r\n"[..])
        );
    }

    #[test]
    fn decodes_dock_state_log() {
        let input = bytes("aa1500f981446f636b5374613a332c4348415247450d0a5eaa");
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert_eq!(frame.dock_state(), Some(super::DockState::Charge));
    }

    #[test]
    fn decodes_warning_counters() {
        let input = bytes("aa1b00f981576865656c5761726e53756d3a382c302c302c302c300d0a5eaa");
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert_eq!(
            frame.warning_counters(),
            Some(super::WarningCounters {
                wheel: Some([8, 0, 0, 0, 0]),
                brush: None,
            })
        );
    }

    #[test]
    fn decodes_bms_voltage_and_mode() {
        let voltage = bytes(
            "aa2500f981626d733a7265707420525054205661647074286f6e29203d2033363138316d56210a5eaa",
        );
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&voltage).pop().unwrap();
        assert_eq!(
            frame.bms_diagnostic(),
            Some(super::BmsDiagnostic::AdapterVoltageMv(36_181))
        );

        let mode = bytes("aa1800f981626d733a7374616d206d6f64655b322d3e335d0d0a5eaa");
        let frame = decoder.push(&mode).pop().unwrap();
        assert_eq!(
            frame.bms_diagnostic(),
            Some(super::BmsDiagnostic::ModeTransition { from: 2, to: 3 })
        );

        let battery =
            bytes("aa2300f981626d733a7265707420736f633d39392c563d31363336392c493d2d3132350d0a5eaa");
        let frame = decoder.push(&battery).pop().unwrap();
        assert_eq!(
            frame.bms_diagnostic(),
            Some(super::BmsDiagnostic::BatteryStatus {
                soc_percent: 99,
                voltage_mv: 16_369,
                current_ma: -125,
            })
        );
    }

    #[test]
    fn decodes_wheel_pid_and_sensor_summary_logs() {
        let wheel = Frame {
            raw: Vec::new(),
            payload: b"\xf9\x81[PID_wheeloc,L,7765,R,6055,cnt,3,\x5e".to_vec(),
            escape_overhead: 0,
            message: 0xf9,
            command: 0x81,
            crc: 0,
            crc_valid: false,
            is_log: true,
        };
        assert_eq!(
            wheel.wheel_pid_diagnostic(),
            Some(super::WheelPidDiagnostic {
                left: 7765,
                right: 6055,
                count: 3,
            })
        );

        let sensors = Frame {
            raw: Vec::new(),
            payload: b"\xf9\x81Force:0 Drop:0 Bmpr:256 Dock:0 Clf:0\x5e".to_vec(),
            escape_overhead: 0,
            message: 0xf9,
            command: 0x81,
            crc: 0,
            crc_valid: false,
            is_log: true,
        };
        assert_eq!(
            sensors.sensor_summary(),
            Some(super::SensorSummary {
                force: 0,
                drop: 0,
                bumper: 256,
                dock: 0,
                cliff: 0,
            })
        );
    }

    #[test]
    fn preserves_raw_wheel_report() {
        let frame = Frame {
            raw: Vec::new(),
            payload: vec![0x51, 0x08, 1, 2, 3, 4, 5, 6, 7, 8],
            escape_overhead: 1,
            message: 0x51,
            command: 0x08,
            crc: 0,
            crc_valid: true,
            is_log: false,
        };
        assert!(frame.is_wheel_report());
        assert_eq!(
            frame.wheel_report().unwrap().bytes,
            [1, 2, 3, 4, 5, 6, 7, 8]
        );
        let wheel = frame.wheel_report().unwrap();
        assert_eq!(wheel.left_current_ma(), 0x0201);
        assert_eq!(wheel.right_current_ma(), 0x0403);
        assert_eq!(wheel.left_drive_level(), 5);
        assert_eq!(wheel.right_drive_level(), 6);
        assert_eq!(wheel.left_fault(), 7);
        assert_eq!(wheel.right_fault(), 8);

        let mut state_payload = vec![0x07, 0x40];
        state_payload.resize(super::STATE_BASE_LENGTH, 0);
        state_payload.extend_from_slice(&[0x51, 0x08, 8, 7, 6, 5, 4, 3, 2, 1]);
        state_payload.extend_from_slice(&[0x52, 0x06, 0xff, 0xff, 0x34, 0x12, 0, 75]);
        state_payload.extend_from_slice(&[0xd0, 0x02, 0, 9]);
        let state = Frame {
            payload: state_payload,
            message: 0x07,
            command: 0x40,
            ..frame
        };
        assert_eq!(
            state.embedded_wheel_report().unwrap().bytes,
            [8, 7, 6, 5, 4, 3, 2, 1]
        );
        assert_eq!(
            state.embedded_report(super::FAN_REPORT_MESSAGE, super::FAN_REPORT_LENGTH),
            None
        );
        assert_eq!(
            state.embedded_motor_report(super::BRUSH_REPORT_MESSAGE),
            Some(super::MotorReport {
                sentinel: 0xffff,
                current_ma: 0x1234,
                fault: 0,
                speed: 75,
            })
        );
        assert_eq!(
            state.report_footer(),
            Some(super::ReportFooter {
                reports_pending: false,
                sequence: 9,
            })
        );
        let reports: Vec<_> = state.embedded_reports().collect();
        assert_eq!(reports.len(), 3);
        assert_eq!(reports[0].message, super::WHEEL_REPORT_MESSAGE);
        assert_eq!(reports[1].message, super::BRUSH_REPORT_MESSAGE);
        assert_eq!(reports[2].message, super::REPORT_FOOTER_MESSAGE);
    }

    #[test]
    fn decodes_confirmed_fields_from_a_moving_capture() {
        let input = bytes(
            "aa5c010740989d853d755710bf326c22416889f43bfd6b943cf8b18bbc865f753cf07d95bcf88ca2bded1e7fbfc4000000f0000000c3cfe14000000000e06e130200000000510888007400433f00005206ffff6501004b42026d00d0020015b4",
        );
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert!(frame.crc_valid);
        assert!(frame.escape_overhead_valid());

        let state = frame.state_report().unwrap();
        assert_eq!(state.left_odometry_ticks, 196);
        assert_eq!(state.right_odometry_ticks, 240);
        assert_eq!(state.forward_motion.to_bits(), 0x40e1_cfc3);
        assert_eq!(state.reserved, 0);
        assert_eq!(state.mcu_timestamp_ms, 34_828_000);

        let wheel = frame.embedded_wheel_report().unwrap();
        assert_eq!(wheel.left_current_ma(), 136);
        assert_eq!(wheel.right_current_ma(), 116);
        assert_eq!(wheel.left_drive_level(), 67);
        assert_eq!(wheel.right_drive_level(), 63);
        assert_eq!((wheel.left_fault(), wheel.right_fault()), (0, 0));

        let brush = frame
            .embedded_motor_report(super::BRUSH_REPORT_MESSAGE)
            .unwrap();
        assert_eq!(brush.sentinel, 0xffff);
        assert_eq!(brush.current_ma, 357);
        assert_eq!(brush.fault, 0);
        assert_eq!(brush.speed, 75);
        assert_eq!(
            frame.wall_sensor_report(),
            Some(super::WallSensorReport { raw_adc: 109 })
        );
        assert_eq!(
            frame.report_footer(),
            Some(super::ReportFooter {
                reports_pending: false,
                sequence: 0x15,
            })
        );
    }

    #[test]
    fn decodes_battery_report_from_captured_bms_values() {
        let input = bytes("aa1001080a06407300610000000000d002012776");
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert!(frame.crc_valid);
        assert_eq!(
            frame.battery_report(),
            Some(super::BatteryReport {
                voltage_mv: 16_390,
                current_ma: 115,
                state_of_charge_percent: 97,
                reserved: [0; 5],
            })
        );
    }

    #[test]
    fn decodes_dock_ir_receiver_masks_from_clocked_capture() {
        let input = bytes(
            "aa62010740bff0f33e6ce4ffbe83071b416889f43b0f5b07bdd8ba2fbfcd649fbcaedd803c484274bf0fbe983ec77c0400470505007de5c73e000000007437920300000000510817000300190400005206ffffb001004a42021b00130418001000d00200d077",
        );
        let mut decoder = FrameDecoder::new();
        let frame = decoder.push(&input).pop().unwrap();
        assert!(frame.crc_valid);
        let report = frame.dock_ir_report().unwrap();
        assert_eq!(report.left_receiver, super::DockIrCodeMask(0x18));
        assert_eq!(report.right_receiver, super::DockIrCodeMask(0x10));
        assert_eq!((report.reserved_byte_1, report.reserved_byte_3), (0, 0));
        assert!(report.left_receiver.code_88());
        assert!(report.left_receiver.majority_pattern());
        assert!(!report.left_receiver.beacon_left());
        assert!(!report.left_receiver.beacon_right());
    }

    #[test]
    fn decodes_gyro_echo_token_and_escaped_token() {
        let mut decoder = FrameDecoder::new();
        let frame = decoder
            .push(&bytes("aa0701d1016ad002001603"))
            .pop()
            .unwrap();
        assert_eq!(
            frame.gyro_echo_report(),
            Some(super::GyroEchoReport { token: 0x6a })
        );

        let frame = decoder
            .push(&bytes("aa0702d101a900d002005a8a"))
            .pop()
            .unwrap();
        assert!(frame.crc_valid);
        assert_eq!(
            frame.gyro_echo_report(),
            Some(super::GyroEchoReport { token: 0xa9 })
        );
    }

    #[test]
    fn decodes_static_analysis_reports_without_guessing_opaque_fields() {
        let mut payload = Vec::new();
        payload.extend_from_slice(&[0x02, 8, b's', b'y', b's', b'_', b'm', b'd', 0, 0x08]);
        payload.extend_from_slice(&[0x04, 4, 0x07, 0, 0, 0]);
        payload.extend_from_slice(&[0x0b, 8, 1, 2, 3, 4, 0x78, 0x56, 0x34, 0x12]);
        payload.extend_from_slice(&[
            0x0d, 16, 3, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
        ]);
        payload.extend_from_slice(&[0x0f, 2, 0x34, 0x12]);
        payload.extend_from_slice(&[0x16, 2, 52, 0xaa]);
        payload.extend_from_slice(&[0xd2, 8, 1, 2, 3, 4, 5, 6, 7, 8]);

        let mut calibration = vec![0xd5, 28];
        calibration.extend_from_slice(&[1, 0, 2, 0, 3, 0, 4, 0]);
        calibration.extend_from_slice(&[5, 6, 7, 8]);
        calibration.extend_from_slice(&1.5_f32.to_le_bytes());
        calibration.extend_from_slice(&(-1_i32).to_le_bytes());
        calibration.extend_from_slice(&2_i32.to_le_bytes());
        calibration.extend_from_slice(&(-3_i32).to_le_bytes());
        payload.extend_from_slice(&calibration);

        payload.extend_from_slice(&[0xf1, 16, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0, 0]);
        payload.extend_from_slice(&[0xf6, 4, b'o', b'k', 0, 0]);

        let frame = Frame {
            raw: Vec::new(),
            message: 0x02,
            command: 8,
            payload,
            escape_overhead: 1,
            crc: 0,
            crc_valid: true,
            is_log: false,
        };

        assert_eq!(
            frame.system_mode_report(),
            Some(super::SystemModeReport {
                mode: super::SystemMode::EnergyEfficiency,
            })
        );
        let error = frame.internal_error_report().unwrap();
        assert!(error.test_info_invalid());
        assert!(error.gyro_probe_failed());
        assert!(error.bms_communication_failed());
        assert_eq!(error.compatibility_bits(), 0);
        assert_eq!(frame.black_box_report().unwrap().value, 0x1234_5678);
        assert_eq!(frame.device_info_report().unwrap().record_type(), 3);
        assert_eq!(frame.dock_voltage_report().unwrap().millivolts, 0x1234);
        assert_eq!(
            frame
                .battery_capacity_report()
                .unwrap()
                .stock_scaled_value(),
            5200
        );
        assert_eq!(
            frame.mcu_time_report().unwrap().timestamp,
            0x0807_0605_0403_0201
        );
        assert_eq!(
            frame.calibration_report(),
            Some(super::CalibrationReport {
                cliff_thresholds: [1, 2, 3, 4],
                wall_sensor_one_point: [5, 6, 7, 8],
                bmi160_sensitivity: 1.5,
                bmi160_accel_offsets: [-1, 2, -3],
            })
        );
        assert_eq!(frame.mcu_identity_report().unwrap().chip_signature, 3);
        assert_eq!(frame.status_string_report(), Some(&b"ok"[..]));
    }
}
