//! Pure AP-to-MCU command encoders for the Roborock S5 Max.
//!
//! These functions only build wire bytes. They never open a device or transmit
//! a command, which keeps active hardware behavior behind an explicit owner.

use super::frame::encode_binary_frame;

pub const QUERY_MCU_VERSION: u8 = 0x80;
pub const SYNC_SENSOR_STATUS: u8 = 0x82;
pub const QUERY_DEVICE_INFO: u8 = 0x88;
pub const QUERY_CURRENT_ERRORS: u8 = 0x8c;
pub const QUERY_BATTERY_CAPACITY: u8 = 0x8f;
pub const SYSTEM_MODE_COMMAND: u8 = 0xb0;
pub const SUBSYSTEM_STATE_COMMAND: u8 = 0xb1;
pub const CHARGER_COMMAND: u8 = 0xb7;
pub const WATER_PUMP_COMMAND: u8 = 0xbf;
pub const WHEEL_COMMAND: u8 = 0xc0;
pub const FAN_COMMAND: u8 = 0xc1;
pub const MAIN_BRUSH_COMMAND: u8 = 0xc3;
pub const SIDE_BRUSH_COMMAND: u8 = 0xc4;
pub const ACK_REPORT: u8 = 0xd1;
pub const HEARTBEAT: u8 = 0xd3;

pub const MAIN_BRUSH_SUBSYSTEM: u32 = 0x00080;
pub const SIDE_BRUSH_SUBSYSTEM: u32 = 0x00100;
pub const FAN_SUBSYSTEM: u32 = 0x00200;
pub const WHEEL_ODOMETRY_SUBSYSTEM: u32 = 0x00400;
pub const OPERATIONAL_SUBSYSTEMS: &[u32] = &[
    0x00002, // bumper
    0x00004, // compatibility hook used by the stock operational transition
    0x00008, // cliff sensors
    0x00020, // drop/lift sensors
    0x00040, // dustbin sensor
    MAIN_BRUSH_SUBSYSTEM,
    SIDE_BRUSH_SUBSYSTEM,
    FAN_SUBSYSTEM,
    WHEEL_ODOMETRY_SUBSYSTEM,
    0x00800, // gyro
    0x02000, // dock IR receivers
    0x04000, // water-box sensor
    0x08000, // water pump
    0x20000, // light-touch/wall sensor
];

const AP_ENVELOPE_MESSAGE: u8 = 0xd0;
const AP_ENVELOPE_COMMAND: u8 = 0x02;

/// Increment the AP transmit counter while preserving zero as a reserved value.
///
/// A takeover must continue the stock counter. Replaying a stale sequence is
/// silently ignored by the MCU.
pub const fn next_tx_sequence(previous: u8) -> u8 {
    if previous == u8::MAX { 1 } else { previous + 1 }
}

/// Encode a generic AP command envelope.
pub fn command_frame(
    sequence: u8,
    command: u8,
    request_ack: bool,
    data: &[u8],
) -> Result<Vec<u8>, String> {
    if sequence == 0 {
        return Err("MCU envelope sequence zero is reserved".to_string());
    }
    let data_length = u8::try_from(data.len()).map_err(|_| {
        format!(
            "command 0x{command:02x} data is too long: {} bytes",
            data.len()
        )
    })?;
    let mut payload = Vec::with_capacity(6 + data.len());
    payload.extend_from_slice(&[
        AP_ENVELOPE_MESSAGE,
        AP_ENVELOPE_COMMAND,
        u8::from(request_ack),
        sequence,
        command,
        data_length,
    ]);
    payload.extend_from_slice(data);
    encode_binary_frame(&payload)
}

/// Encode a command with no data body.
pub fn query_frame(sequence: u8, command: u8, request_ack: bool) -> Result<Vec<u8>, String> {
    command_frame(sequence, command, request_ack, &[])
}

pub fn mcu_version_query_frame(sequence: u8) -> Result<Vec<u8>, String> {
    query_frame(sequence, QUERY_MCU_VERSION, false)
}

pub fn device_info_query_frame(sequence: u8) -> Result<Vec<u8>, String> {
    query_frame(sequence, QUERY_DEVICE_INFO, false)
}

pub fn battery_capacity_query_frame(sequence: u8) -> Result<Vec<u8>, String> {
    query_frame(sequence, QUERY_BATTERY_CAPACITY, false)
}

pub fn current_errors_query_frame(sequence: u8) -> Result<Vec<u8>, String> {
    query_frame(sequence, QUERY_CURRENT_ERRORS, true)
}

/// Encode the periodically refreshed water-pump interval command.
pub fn water_pump_command_frame(
    sequence: u8,
    on_interval: u8,
    off_interval: u8,
) -> Result<Vec<u8>, String> {
    command_frame(
        sequence,
        WATER_PUMP_COMMAND,
        false,
        &[on_interval, off_interval],
    )
}

pub fn sensor_sync_query_frame(sequence: u8) -> Result<Vec<u8>, String> {
    query_frame(sequence, SYNC_SENSOR_STATUS, true)
}

/// Request an MCU system/power mode transition.
///
/// Mode zero is the stock normal/awake state. The MCU confirms this command
/// with a `0x02` system-mode report, which the session acknowledges through
/// the normal incoming-report path.
pub fn system_mode_command_frame(sequence: u8, mode: u8) -> Result<Vec<u8>, String> {
    let mut data = *b"sys_md\0\0";
    data[7] = mode;
    command_frame(sequence, SYSTEM_MODE_COMMAND, false, &data)
}

/// Enable or disable one MCU subsystem using the stock `b1/len4` bit selector.
pub fn subsystem_state_frame(
    sequence: u8,
    selector: u32,
    enabled: bool,
) -> Result<Vec<u8>, String> {
    if selector == 0 || !selector.is_power_of_two() || selector & 1 != 0 {
        return Err(format!("invalid MCU subsystem selector 0x{selector:08x}"));
    }
    let state = selector | u32::from(enabled);
    command_frame(
        sequence,
        SUBSYSTEM_STATE_COMMAND,
        true,
        &state.to_le_bytes(),
    )
}

/// Enable or disable the dock charging path.
pub fn charger_command_frame(sequence: u8, enabled: bool) -> Result<Vec<u8>, String> {
    command_frame(sequence, CHARGER_COMMAND, true, &[u8::from(enabled)])
}

/// Encode the stock `c0/len12` differential-drive target.
pub fn wheel_command_frame(
    sequence: u8,
    linear_target: f32,
    angular_target: f32,
    forced: bool,
) -> Result<Vec<u8>, String> {
    if !linear_target.is_finite() || !angular_target.is_finite() {
        return Err("wheel targets must be finite".to_string());
    }
    let mut data = [0_u8; 12];
    data[0..4].copy_from_slice(&linear_target.to_le_bytes());
    data[4..8].copy_from_slice(&angular_target.to_le_bytes());
    data[8..12].copy_from_slice(&(if forced { 1_u32 } else { 0_u32 }).to_le_bytes());
    command_frame(sequence, WHEEL_COMMAND, false, &data)
}

/// Encode the stock `c1/len2` fan target.
pub fn fan_command_frame(sequence: u8, target: u8) -> Result<Vec<u8>, String> {
    command_frame(sequence, FAN_COMMAND, true, &[target, 0])
}

/// Encode the stock `c3/len2` main-brush target.
pub fn main_brush_command_frame(
    sequence: u8,
    target: u8,
    force_start: bool,
) -> Result<Vec<u8>, String> {
    command_frame(
        sequence,
        MAIN_BRUSH_COMMAND,
        true,
        &[target, u8::from(force_start)],
    )
}

/// Encode the stock `c4/len2` side-brush target.
///
/// This MCU ignores the second byte, but stock sometimes sets it while
/// stopping, so it remains explicit for wire compatibility.
pub fn side_brush_command_frame(
    sequence: u8,
    target: u8,
    compatibility_flag: bool,
) -> Result<Vec<u8>, String> {
    command_frame(
        sequence,
        SIDE_BRUSH_COMMAND,
        true,
        &[target, u8::from(compatibility_flag)],
    )
}

/// Acknowledge an MCU frame that requested an AP response.
pub fn acknowledgement_frame(sequence: u8, received_sequence: u8) -> Result<Vec<u8>, String> {
    command_frame(sequence, ACK_REPORT, false, &[received_sequence])
}

/// Encode the stock zero-data heartbeat body.
pub fn heartbeat_frame(sequence: u8) -> Result<Vec<u8>, String> {
    command_frame(sequence, HEARTBEAT, false, &[0, 0, 0, 0])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sequence_skips_reserved_zero() {
        assert_eq!(next_tx_sequence(0x7e), 0x7f);
        assert_eq!(next_tx_sequence(0xff), 0x01);
    }

    #[test]
    fn encodes_captured_queries() {
        assert_eq!(
            mcu_version_query_frame(0x7f).unwrap(),
            [0xaa, 0x06, 0x01, 0xd0, 0x02, 0x00, 0x7f, 0x80, 0x00, 0xd8]
        );
        assert_eq!(
            sensor_sync_query_frame(0x13).unwrap(),
            [0xaa, 0x06, 0x01, 0xd0, 0x02, 0x01, 0x13, 0x82, 0x00, 0xd8]
        );
    }

    #[test]
    fn encodes_captured_ack_and_heartbeat() {
        assert_eq!(
            acknowledgement_frame(0x15, 0xd1).unwrap(),
            [
                0xaa, 0x07, 0x01, 0xd0, 0x02, 0x00, 0x15, 0xd1, 0x01, 0xd1, 0x03
            ]
        );
        assert_eq!(
            heartbeat_frame(0x6a).unwrap(),
            [
                0xaa, 0x0a, 0x01, 0xd0, 0x02, 0x00, 0x6a, 0xd3, 0x04, 0x00, 0x00, 0x00, 0x00, 0x47,
            ]
        );
    }

    #[test]
    fn encodes_captured_actuator_frames() {
        assert_eq!(
            wheel_command_frame(0xe5, 0.0, 0.0, true).unwrap(),
            [
                0xaa, 0x12, 0x01, 0xd0, 0x02, 0x00, 0xe5, 0xc0, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00,
                0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0c,
            ]
        );
        assert_eq!(
            fan_command_frame(0xe6, 0).unwrap(),
            [
                0xaa, 0x08, 0x01, 0xd0, 0x02, 0x01, 0xe6, 0xc1, 0x02, 0x00, 0x00, 0x7d
            ]
        );
        assert_eq!(
            main_brush_command_frame(0x39, 0, true).unwrap(),
            [
                0xaa, 0x08, 0x01, 0xd0, 0x02, 0x01, 0x39, 0xc3, 0x02, 0x00, 0x01, 0x15
            ]
        );
        assert_eq!(
            side_brush_command_frame(0x3b, 0, true).unwrap(),
            [
                0xaa, 0x08, 0x01, 0xd0, 0x02, 0x01, 0x3b, 0xc4, 0x02, 0x00, 0x01, 0x10
            ]
        );
    }

    #[test]
    fn rejects_invalid_envelopes_and_wheel_values() {
        assert!(fan_command_frame(0, 0).is_err());
        assert!(wheel_command_frame(1, f32::NAN, 0.0, true).is_err());
        assert!(wheel_command_frame(1, 0.0, f32::INFINITY, true).is_err());
        assert!(command_frame(1, 0x80, false, &[0; 256]).is_err());
    }

    #[test]
    fn encodes_zero_water_pump_stop() {
        let frame = water_pump_command_frame(0x24, 0, 0).unwrap();
        assert_eq!(
            &frame[3..11],
            &[0xd0, 0x02, 0x00, 0x24, 0xbf, 0x02, 0x00, 0x00]
        );
    }

    #[test]
    fn encodes_captured_charger_transitions() {
        let disabled = charger_command_frame(0x20, false).unwrap();
        let enabled = charger_command_frame(0x21, true).unwrap();
        assert_eq!(
            &disabled[3..10],
            &[0xd0, 0x02, 0x01, 0x20, 0xb7, 0x01, 0x00]
        );
        assert_eq!(&enabled[3..10], &[0xd0, 0x02, 0x01, 0x21, 0xb7, 0x01, 0x01]);
    }

    #[test]
    fn encodes_captured_normal_system_mode_transition() {
        let frame = system_mode_command_frame(0x20, 0).unwrap();
        assert_eq!(
            &frame[3..17],
            &[
                0xd0, 0x02, 0x00, 0x20, 0xb0, 0x08, b's', b'y', b's', b'_', b'm', b'd', 0, 0,
            ]
        );
    }

    #[test]
    fn encodes_captured_motor_subsystem_enables() {
        assert_eq!(
            subsystem_state_frame(0x98, FAN_SUBSYSTEM, true).unwrap(),
            [
                0xaa, 0x0a, 0x01, 0xd0, 0x02, 0x01, 0x98, 0xb1, 0x04, 0x01, 0x02, 0x00, 0x00, 0x57,
            ]
        );
        assert_eq!(
            &subsystem_state_frame(0x96, MAIN_BRUSH_SUBSYSTEM, true).unwrap()[9..13],
            &[0x81, 0x00, 0x00, 0x00]
        );
        assert_eq!(
            &subsystem_state_frame(0x97, SIDE_BRUSH_SUBSYSTEM, true).unwrap()[9..13],
            &[0x01, 0x01, 0x00, 0x00]
        );
        assert_eq!(
            subsystem_state_frame(0x9a, WHEEL_ODOMETRY_SUBSYSTEM, true).unwrap(),
            [
                0xaa, 0x0a, 0x01, 0xd0, 0x02, 0x01, 0x9a, 0xb1, 0x04, 0x01, 0x04, 0x00, 0x00, 0xfc,
            ]
        );
        assert!(subsystem_state_frame(1, 0, true).is_err());
        assert!(subsystem_state_frame(1, 0x300, true).is_err());
    }
}
