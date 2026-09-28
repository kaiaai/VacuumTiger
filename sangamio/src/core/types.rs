//! Core data types for sensors, commands, and device communication.
//!
//! Key types for device implementers:
//! - [`SensorGroupData`]: Container for sensor values, updated by driver threads
//! - [`Command`]: Inbound commands from TCP clients (mainly [`Command::ComponentControl`])
//! - [`SensorValue`]: Typed sensor values for the `values` HashMap

use crossbeam_channel::{Receiver, Sender};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// Runtime sensor values
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorValue {
    Bool(bool),
    U8(u8),
    U16(u16),
    U32(u32),
    U64(u64),
    I8(i8),
    I16(i16),
    I32(i32),
    I64(i64),
    F32(f32),
    F64(f64),
    String(String),
    Bytes(Vec<u8>),
    Vector3([f32; 3]),
    PointCloud2D(Vec<(f32, f32, u8)>), // (angle_rad, distance_m, quality)
}

/// Runtime sensor group data (shared between threads)
#[derive(Debug, Clone)]
pub struct SensorGroupData {
    pub group_id: String,
    pub timestamp_us: u64,
    /// Monotonically increasing sequence number for each update.
    /// Used by TCP publisher to detect new data even when timestamps
    /// appear equal (avoids message coalescing at high rates).
    pub sequence_number: u64,
    pub values: HashMap<String, SensorValue>,
}

impl SensorGroupData {
    /// Create a new empty SensorGroupData
    pub fn new(group_id: &str) -> Self {
        Self {
            group_id: group_id.to_string(),
            timestamp_us: 0,
            sequence_number: 0,
            values: HashMap::new(),
        }
    }

    /// Set a value (create or update in-place)
    #[inline]
    pub fn set(&mut self, key: &str, value: SensorValue) {
        if let Some(v) = self.values.get_mut(key) {
            *v = value;
        } else {
            self.values.insert(key.to_string(), value);
        }
    }

    /// Update timestamp to current time and increment sequence number.
    ///
    /// The sequence number ensures each update is detected by the TCP publisher
    /// even when timestamps appear equal (e.g., at high update rates like 110Hz).
    #[inline]
    pub fn touch(&mut self) {
        self.timestamp_us = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_micros() as u64)
            .unwrap_or(0);
        self.sequence_number = self.sequence_number.wrapping_add(1);
    }
}

/// Static drive and IMU geometry, published by every driver as the
/// `kinematics` sensor group.
///
/// Clients (e.g. vacuum_ros2_bridge) turn the raw `wheel_left`/`wheel_right`
/// ticks and `gyro_*`/`accel_*` counts in `sensor_status` into SI units with
/// these values, so one client works for every robot without per-model
/// constants. All values are SI.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Kinematics {
    /// Wheel travel per encoder tick, in metres.
    pub wheel_m_per_tick: f32,
    /// Differential-drive wheel track (left-to-right wheel distance), in metres.
    pub wheel_track_m: f32,
    /// Width of the `wheel_left`/`wheel_right` counters; they wrap at `2^bits`.
    pub wheel_tick_bits: u8,
    /// Angular rate per `gyro_x/y/z` count, in rad/s.
    pub gyro_rad_per_lsb: f32,
    /// Acceleration per `accel_x/y/z` count, in m/s². `None` when the robot
    /// does not publish scaled accelerometer counts.
    pub accel_mps2_per_lsb: Option<f32>,
}

impl Kinematics {
    /// Build the (touched, so it gets published) `kinematics` sensor group.
    pub fn to_sensor_group(&self) -> SensorGroupData {
        let mut data = SensorGroupData::new("kinematics");
        data.set("wheel_m_per_tick", SensorValue::F32(self.wheel_m_per_tick));
        data.set("wheel_track_m", SensorValue::F32(self.wheel_track_m));
        data.set("wheel_tick_bits", SensorValue::U8(self.wheel_tick_bits));
        data.set("gyro_rad_per_lsb", SensorValue::F32(self.gyro_rad_per_lsb));
        if let Some(scale) = self.accel_mps2_per_lsb {
            data.set("accel_mps2_per_lsb", SensorValue::F32(scale));
        }
        data.touch();
        data
    }
}

/// Number of angle bins in a packed LiDAR scan.
pub const PACKED_SCAN_BINS: usize = 360;

/// Pack a LiDAR scan into the compact `scan_packed` blob.
///
/// Instead of inflating each point to protobuf floats (~14 bytes), the scan is
/// binned to a fixed grid and packed as raw native values. Wire layout
/// (little-endian, ~1082 bytes for 360 bins -- under one MTU, so a scan is
/// never IP-fragmented):
///   u16 num_bins
///   per bin: u16 distance (0.25mm units, 0 = no return), u8 quality
/// Bin i is angle `i * 2pi / num_bins` (ROS CCW, 0 = forward). Input points are
/// `(angle_rad, distance_m, quality)` with the angle already in that frame.
/// The client converts distance to meters (x0.00025).
pub fn pack_scan(points: &[(f32, f32, u8)]) -> Vec<u8> {
    use std::f32::consts::TAU;

    let mut dist = [0u16; PACKED_SCAN_BINS];
    let mut qual = [0u8; PACKED_SCAN_BINS];
    for &(angle, distance_m, quality) in points {
        let bin = (angle / TAU * PACKED_SCAN_BINS as f32) as usize % PACKED_SCAN_BINS;
        // meters -> raw 0.25mm units (the lidar's native resolution; no real loss)
        let raw = (distance_m * 4000.0).round().clamp(0.0, u16::MAX as f32) as u16;
        dist[bin] = raw;
        qual[bin] = quality;
    }

    let mut buf = Vec::with_capacity(2 + PACKED_SCAN_BINS * 3);
    buf.extend_from_slice(&(PACKED_SCAN_BINS as u16).to_le_bytes());
    for i in 0..PACKED_SCAN_BINS {
        buf.extend_from_slice(&dist[i].to_le_bytes());
        buf.push(qual[i]);
    }
    buf
}

/// Actions that can be performed on components (sensors and actuators)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ComponentAction {
    /// Turn on/activate the component with optional config (e.g., mode)
    Enable {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Turn off/deactivate the component with optional config
    Disable {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Reset to factory defaults / trigger calibration with optional config
    Reset {
        #[serde(default)]
        config: Option<HashMap<String, SensorValue>>,
    },
    /// Configure component parameters (speed, velocity - continuous updates)
    Configure {
        config: HashMap<String, SensorValue>,
    },
}

/// Commands to device
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Command {
    // Unified Component Control
    /// Control any component (sensor or actuator) with standard actions.
    ///
    /// Component IDs are defined in [`crate::devices::crl200s::gd32::commands`] module.
    /// See also `proto/sangamio.proto` for the protobuf schema.
    ///
    /// # Motion Control (id: "drive")
    /// - `Configure { linear: F32, angular: F32 }` - velocity mode (m/s, rad/s)
    /// - `Configure { left: F32, right: F32 }` - tank drive mode (m/s)
    /// - `Enable { mode: U8 }` - enter motor mode (default 0x02 = navigation)
    /// - `Disable` - stop (zero velocity) and exit motor mode
    /// - `Reset` - emergency stop (immediate halt, all actuators off)
    ///
    /// # Actuators (speed 0-100%)
    /// - `vacuum`: Suction motor - Enable/Disable/Configure(speed)
    /// - `main_brush`: Main brush roller - Enable/Disable/Configure(speed)
    /// - `side_brush`: Side brush spinner - Enable/Disable/Configure(speed)
    /// - `water_pump`: Mop water pump - Enable/Disable/Configure(speed)
    /// - `led`: Status LED - Configure(state: 0-18)
    /// - `lidar`: Lidar motor - Enable(pwm)/Disable/Configure(pwm)
    ///
    /// # Sensors
    /// - `imu`: Enable (query calibration state), Reset (factory calibrate)
    /// - `compass`: Enable (query calibration state), Reset (start calibration)
    /// - `cliff_ir`: Enable/Disable IR emitters, Configure(direction)
    ///
    /// # Power Management
    /// - `main_board`: A33 power - Enable/Disable/Reset (WARNING: terminates daemon!)
    /// - `charger`: Charger rail - Enable/Disable
    /// - `mcu`: GD32 MCU - Enable (wake ack), Disable (sleep), Reset (clear errors)
    ComponentControl {
        /// Component identifier (e.g., "drive", "vacuum", "imu", "cliff_ir").
        /// See module docs for complete list of valid IDs.
        id: String,
        /// Action to perform on the component
        action: ComponentAction,
    },

    // Protocol Commands
    /// Protocol sync - first command to wake GD32 and synchronize protocol
    ///
    /// This is a fire-and-forget command. GD32 echoes it back after ~270ms.
    /// Typically sent once at boot before any other commands.
    ProtocolSync,

    // System Lifecycle
    /// Graceful daemon shutdown
    Shutdown,
}

/// Bounded channel capacity for streaming sensor data.
/// At 110Hz with ~150 bytes per message, 1000 messages ≈ 150KB buffer.
/// This allows ~9 seconds of buffering before dropping messages.
pub const STREAM_CHANNEL_CAPACITY: usize = 1000;

/// Streaming channel for sensor data.
///
/// Used to stream sensor updates from driver threads to TCP publisher
/// without data loss (unlike shared mutex which can lose intermediate updates).
pub type StreamSender = Sender<SensorGroupData>;
pub type StreamReceiver = Receiver<SensorGroupData>;

/// Create a new bounded streaming channel pair.
///
/// Returns (sender, receiver) where:
/// - Sender is given to the driver thread to push updates
/// - Receiver is given to the TCP publisher to consume updates
///
/// The channel is bounded to prevent unbounded memory growth.
/// If the publisher falls behind, oldest messages are dropped.
pub fn create_stream_channel() -> (StreamSender, StreamReceiver) {
    crossbeam_channel::bounded(STREAM_CHANNEL_CAPACITY)
}

#[cfg(test)]
mod tests {
    use super::{Kinematics, PACKED_SCAN_BINS, SensorValue, pack_scan};
    use std::f32::consts::PI;

    #[test]
    fn pack_scan_bins_points_in_quarter_millimetres() {
        let blob = pack_scan(&[(0.0, 1.0, 7), (PI / 2.0 + 0.001, 0.25, 9)]);
        assert_eq!(blob.len(), 2 + PACKED_SCAN_BINS * 3);
        assert_eq!(u16::from_le_bytes([blob[0], blob[1]]), 360);
        let bin = |i: usize| {
            let o = 2 + i * 3;
            (u16::from_le_bytes([blob[o], blob[o + 1]]), blob[o + 2])
        };
        assert_eq!(bin(0), (4000, 7));
        assert_eq!(bin(90), (1000, 9));
        assert_eq!(bin(1), (0, 0));
    }

    #[test]
    fn kinematics_group_is_touched_and_omits_unknown_accel() {
        let group = Kinematics {
            wheel_m_per_tick: 0.000798,
            wheel_track_m: 0.229,
            wheel_tick_bits: 16,
            gyro_rad_per_lsb: 0.001,
            accel_mps2_per_lsb: None,
        }
        .to_sensor_group();
        assert_eq!(group.group_id, "kinematics");
        assert_ne!(group.sequence_number, 0, "must be touched to be published");
        assert!(matches!(
            group.values["wheel_tick_bits"],
            SensorValue::U8(16)
        ));
        assert!(!group.values.contains_key("accel_mps2_per_lsb"));
    }
}
