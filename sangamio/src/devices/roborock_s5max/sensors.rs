//! Translation from S5 Max wire reports to VacuumTiger sensor groups.

use super::packet::{BRUSH_REPORT_MESSAGE, FAN_REPORT_MESSAGE, Frame, SWEEP_REPORT_MESSAGE};
use crate::core::driver::DriverInitResult;
use crate::core::types::{SensorGroupData, SensorValue, StreamSender, create_stream_channel};
use std::collections::HashMap;
use std::sync::atomic::{AtomicU16, Ordering};
use std::sync::{Arc, Mutex};

const KEY_REPORT: u8 = 0x06;
const PUMP_ERROR_REPORT: u8 = 0x18;
const DOCK_REPORT: u8 = 0x20;
const BUMPER_REPORT: u8 = 0x21;
const DROP_REPORT: u8 = 0x22;
const CLIFF_REPORT: u8 = 0x23;
const DUSTBIN_REPORT: u8 = 0x24;
const LDS_COVER_REPORT: u8 = 0x25;
const WATER_BOX_REPORT: u8 = 0x26;
const MCU_VERSION_REPORT: u8 = 0x01;

fn group(name: &str) -> Arc<Mutex<SensorGroupData>> {
    Arc::new(Mutex::new(SensorGroupData::new(name)))
}

fn little_u16(bytes: &[u8]) -> Option<u16> {
    Some(u16::from_le_bytes(bytes.get(..2)?.try_into().ok()?))
}

fn update_group(
    group: &Arc<Mutex<SensorGroupData>>,
    update: impl FnOnce(&mut SensorGroupData),
) -> Option<SensorGroupData> {
    let mut data = match group.lock() {
        Ok(data) => data,
        Err(_) => {
            log::error!("S5 Max sensor-group mutex poisoned");
            return None;
        }
    };
    update(&mut data);
    data.touch();
    Some(data.clone())
}

#[derive(Clone)]
pub(crate) struct SensorGroups {
    pub sensor_status: Arc<Mutex<SensorGroupData>>,
    pub power_status: Arc<Mutex<SensorGroupData>>,
    pub attachments: Arc<Mutex<SensorGroupData>>,
    pub motor_status: Arc<Mutex<SensorGroupData>>,
    pub errors: Arc<Mutex<SensorGroupData>>,
    pub dock_ir: Arc<Mutex<SensorGroupData>>,
    pub device_version: Arc<Mutex<SensorGroupData>>,
    pub lidar: Arc<Mutex<SensorGroupData>>,
    dock_state: Arc<AtomicU16>,
    stream_tx: StreamSender,
    wheel_metres_per_tick: f32,
}

impl SensorGroups {
    pub(crate) fn create(
        wheel_mm_per_tick: f32,
        wheel_track_m: f32,
        lds_forward_angle_deg: f32,
    ) -> (Self, DriverInitResult) {
        let sensor_status = group("sensor_status");
        let power_status = group("power_status");
        let attachments = group("attachments");
        let motor_status = group("motor_status");
        let errors = group("errors");
        let dock_ir = group("dock_ir");
        let device_version = group("device_version");
        let lidar = group("lidar");
        let dock_state = Arc::new(AtomicU16::new(u16::MAX));
        let (stream_tx, stream_rx) = create_stream_channel();

        update_group(&device_version, |data| {
            data.set("wheel_mm_per_tick", SensorValue::F32(wheel_mm_per_tick));
            data.set("wheel_track_m", SensorValue::F32(wheel_track_m));
            data.set(
                "lds_forward_angle_deg",
                SensorValue::F32(lds_forward_angle_deg),
            );
        });

        let sensor_data = [
            ("sensor_status", Arc::clone(&sensor_status)),
            ("power_status", Arc::clone(&power_status)),
            ("attachments", Arc::clone(&attachments)),
            ("motor_status", Arc::clone(&motor_status)),
            ("errors", Arc::clone(&errors)),
            ("dock_ir", Arc::clone(&dock_ir)),
            ("device_version", Arc::clone(&device_version)),
            ("lidar", Arc::clone(&lidar)),
        ]
        .into_iter()
        .map(|(name, data)| (name.to_string(), data))
        .collect();
        let stream_receivers = HashMap::from([("sensor_status".to_string(), stream_rx)]);

        (
            Self {
                sensor_status,
                power_status,
                attachments,
                motor_status,
                errors,
                dock_ir,
                device_version,
                lidar,
                dock_state,
                stream_tx,
                wheel_metres_per_tick: wheel_mm_per_tick / 1000.0,
            },
            DriverInitResult {
                sensor_data,
                stream_receivers,
            },
        )
    }

    pub(crate) fn process_mcu_frame(&self, frame: &Frame) {
        if let Some(state) = frame.state_report()
            && let Some(snapshot) = update_group(&self.sensor_status, |data| {
                data.set("acceleration", SensorValue::Vector3(state.imu.acceleration));
                data.set(
                    "angular_velocity",
                    SensorValue::Vector3(state.imu.angular_rate),
                );
                data.set("orientation_x", SensorValue::F32(state.imu.quaternion[0]));
                data.set("orientation_y", SensorValue::F32(state.imu.quaternion[1]));
                data.set("orientation_z", SensorValue::F32(state.imu.quaternion[2]));
                data.set("orientation_w", SensorValue::F32(state.imu.quaternion[3]));
                data.set(
                    "wheel_left_ticks",
                    SensorValue::I32(state.left_odometry_ticks),
                );
                data.set(
                    "wheel_right_ticks",
                    SensorValue::I32(state.right_odometry_ticks),
                );
                data.set(
                    "wheel_left_m",
                    SensorValue::F32(state.left_odometry_ticks as f32 * self.wheel_metres_per_tick),
                );
                data.set(
                    "wheel_right_m",
                    SensorValue::F32(
                        state.right_odometry_ticks as f32 * self.wheel_metres_per_tick,
                    ),
                );
                data.set("forward_motion_raw", SensorValue::F32(state.forward_motion));
                data.set("mcu_timestamp_ms", SensorValue::U64(state.mcu_timestamp_ms));
            })
        {
            let _ = self.stream_tx.try_send(snapshot);
        }

        self.process_discrete_sensors(frame);
        self.process_power(frame);
        self.process_motors(frame);
        self.process_errors(frame);
        self.process_dock_ir(frame);
        self.process_version(frame);
    }

    pub(crate) fn dock_state(&self) -> Arc<AtomicU16> {
        Arc::clone(&self.dock_state)
    }

    fn process_discrete_sensors(&self, frame: &Frame) {
        let reports = [
            (KEY_REPORT, "buttons_raw"),
            (BUMPER_REPORT, "bumper_raw"),
            (DROP_REPORT, "drop_raw"),
            (CLIFF_REPORT, "cliff_raw"),
            (LDS_COVER_REPORT, "lds_cover_bumper_raw"),
        ];
        let values = reports
            .into_iter()
            .filter_map(|(message, name)| {
                frame.report(message).and_then(|report| {
                    little_u16(report.data).map(|value| (name, SensorValue::U16(value)))
                })
            })
            .collect::<Vec<_>>();
        if !values.is_empty() {
            update_group(&self.sensor_status, |data| {
                for (name, value) in values {
                    data.set(name, value);
                }
            });
        }

        let dustbin = frame
            .report(DUSTBIN_REPORT)
            .and_then(|report| little_u16(report.data));
        let water_box = frame
            .report(WATER_BOX_REPORT)
            .and_then(|report| little_u16(report.data));
        if dustbin.is_some() || water_box.is_some() {
            update_group(&self.attachments, |data| {
                if let Some(value) = dustbin {
                    data.set("dustbin_raw", SensorValue::U16(value));
                    data.set("dustbin_attached", SensorValue::Bool(value & 1 == 0));
                }
                if let Some(value) = water_box {
                    data.set("water_box_raw", SensorValue::U16(value));
                    data.set("water_box_attached", SensorValue::Bool(value & 2 == 0));
                    data.set(
                        "water_box_presence_bit",
                        SensorValue::U8(((value >> 1) & 1) as u8),
                    );
                    // The S5 Max stock handler uses only bit 1 for the water
                    // box. It has no independent electronic mop-cloth sensor.
                    data.set("mop_detection_supported", SensorValue::Bool(false));
                }
            });
        }
    }

    fn process_power(&self, frame: &Frame) {
        let battery = frame.battery_report();
        let dock = frame
            .report(DOCK_REPORT)
            .and_then(|report| little_u16(report.data));
        let dock_voltage = frame.dock_voltage_report();
        let capacity = frame.battery_capacity_report();
        if battery.is_none() && dock.is_none() && dock_voltage.is_none() && capacity.is_none() {
            return;
        }
        if let Some(value) = dock {
            self.dock_state.store(value, Ordering::Release);
        }
        update_group(&self.power_status, |data| {
            if let Some(battery) = battery {
                data.set(
                    "battery_voltage_v",
                    SensorValue::F32(battery.voltage_mv as f32 / 1000.0),
                );
                data.set(
                    "battery_current_magnitude_a",
                    SensorValue::F32(battery.current_ma as f32 / 1000.0),
                );
                data.set(
                    "battery_level_percent",
                    SensorValue::U8(battery.state_of_charge_percent),
                );
            }
            if let Some(value) = dock {
                data.set("dock_state_raw", SensorValue::U16(value));
                data.set("dock_connected", SensorValue::Bool(value != 0));
                data.set("is_charging", SensorValue::Bool(value == 1));
            }
            if let Some(report) = dock_voltage {
                data.set(
                    "dock_voltage_v",
                    SensorValue::F32(report.millivolts as f32 / 1000.0),
                );
            }
            if let Some(report) = capacity {
                data.set(
                    "battery_capacity_mah",
                    SensorValue::U16(report.stock_scaled_value()),
                );
            }
        });
    }

    fn process_motors(&self, frame: &Frame) {
        let wheel = frame
            .wheel_report()
            .or_else(|| frame.embedded_wheel_report());
        let fan = frame.embedded_motor_report(FAN_REPORT_MESSAGE);
        let main_brush = frame.embedded_motor_report(BRUSH_REPORT_MESSAGE);
        let side_brush = frame.embedded_motor_report(SWEEP_REPORT_MESSAGE);
        if wheel.is_none() && fan.is_none() && main_brush.is_none() && side_brush.is_none() {
            return;
        }
        update_group(&self.motor_status, |data| {
            if let Some(wheel) = wheel {
                data.set(
                    "wheel_left_current_ma",
                    SensorValue::U16(wheel.left_current_ma()),
                );
                data.set(
                    "wheel_right_current_ma",
                    SensorValue::U16(wheel.right_current_ma()),
                );
                data.set(
                    "wheel_left_drive_level",
                    SensorValue::I8(wheel.left_drive_level()),
                );
                data.set(
                    "wheel_right_drive_level",
                    SensorValue::I8(wheel.right_drive_level()),
                );
                data.set("wheel_left_fault", SensorValue::U8(wheel.left_fault()));
                data.set("wheel_right_fault", SensorValue::U8(wheel.right_fault()));
            }
            for (prefix, report) in [
                ("fan", fan),
                ("main_brush", main_brush),
                ("side_brush", side_brush),
            ] {
                if let Some(report) = report {
                    data.set(
                        &format!("{prefix}_current_ma"),
                        SensorValue::U16(report.current_ma),
                    );
                    data.set(
                        &format!("{prefix}_speed_raw"),
                        SensorValue::U8(report.speed),
                    );
                    data.set(&format!("{prefix}_fault"), SensorValue::U8(report.fault));
                }
            }
        });
    }

    fn process_errors(&self, frame: &Frame) {
        let internal = frame.internal_error_report();
        let pump = frame.report(PUMP_ERROR_REPORT).map(|report| report.data);
        if internal.is_none() && pump.is_none() {
            return;
        }
        update_group(&self.errors, |data| {
            if let Some(report) = internal {
                data.set("internal_raw", SensorValue::U32(report.raw));
                data.set(
                    "test_info_invalid",
                    SensorValue::Bool(report.test_info_invalid()),
                );
                data.set(
                    "gyro_probe_failed",
                    SensorValue::Bool(report.gyro_probe_failed()),
                );
                data.set(
                    "bms_communication_failed",
                    SensorValue::Bool(report.bms_communication_failed()),
                );
            }
            if let Some(bytes) = pump {
                data.set("water_pump_error_raw", SensorValue::Bytes(bytes.to_vec()));
            }
        });
    }

    fn process_dock_ir(&self, frame: &Frame) {
        let Some(report) = frame.dock_ir_report() else {
            return;
        };
        update_group(&self.dock_ir, |data| {
            data.set("left_mask", SensorValue::U8(report.left_receiver.0));
            data.set("right_mask", SensorValue::U8(report.right_receiver.0));
            data.set(
                "left_far_left",
                SensorValue::Bool(report.left_receiver.beacon_left()),
            );
            data.set(
                "left_far_right",
                SensorValue::Bool(report.left_receiver.beacon_right()),
            );
            data.set(
                "left_close_left",
                SensorValue::Bool(report.left_receiver.code_84()),
            );
            data.set(
                "left_close_right",
                SensorValue::Bool(report.left_receiver.code_88()),
            );
            data.set(
                "left_common",
                SensorValue::Bool(report.left_receiver.majority_pattern()),
            );
            data.set(
                "right_far_left",
                SensorValue::Bool(report.right_receiver.beacon_left()),
            );
            data.set(
                "right_far_right",
                SensorValue::Bool(report.right_receiver.beacon_right()),
            );
            data.set(
                "right_close_left",
                SensorValue::Bool(report.right_receiver.code_84()),
            );
            data.set(
                "right_close_right",
                SensorValue::Bool(report.right_receiver.code_88()),
            );
            data.set(
                "right_common",
                SensorValue::Bool(report.right_receiver.majority_pattern()),
            );
        });
    }

    fn process_version(&self, frame: &Frame) {
        let version = frame.report(MCU_VERSION_REPORT).and_then(|report| {
            let end = report
                .data
                .iter()
                .position(|byte| *byte == 0)
                .unwrap_or(report.data.len());
            std::str::from_utf8(&report.data[..end]).ok()
        });
        let mode = frame.system_mode_report();
        if version.is_none() && mode.is_none() {
            return;
        }
        update_group(&self.device_version, |data| {
            if let Some(version) = version {
                data.set("mcu_version", SensorValue::String(version.to_string()));
            }
            if let Some(mode) = mode {
                data.set(
                    "system_mode",
                    SensorValue::String(format!("{:?}", mode.mode)),
                );
            }
        });
    }
}

#[cfg(test)]
mod tests {
    use super::SensorGroups;
    use crate::core::types::SensorValue;
    use crate::devices::roborock_s5max::packet::{Frame, FrameDecoder};

    fn report_frame(payload: Vec<u8>) -> Frame {
        Frame {
            raw: Vec::new(),
            payload,
            escape_overhead: 0,
            message: 0x0b,
            command: 0x08,
            crc: 0,
            crc_valid: true,
            is_log: false,
        }
    }

    fn bool_value(value: Option<&SensorValue>) -> bool {
        let Some(SensorValue::Bool(value)) = value else {
            panic!("expected boolean sensor value");
        };
        *value
    }

    #[test]
    fn publishes_calibration_metadata() {
        let (groups, _) = SensorGroups::create(0.798, 0.229, 261.2);
        let data = groups.device_version.lock().unwrap();
        assert!(matches!(
            data.values.get("wheel_mm_per_tick"),
            Some(SensorValue::F32(value)) if (*value - 0.798).abs() < f32::EPSILON
        ));
        assert!(matches!(
            data.values.get("wheel_track_m"),
            Some(SensorValue::F32(value)) if (*value - 0.229).abs() < f32::EPSILON
        ));
        assert!(matches!(
            data.values.get("lds_forward_angle_deg"),
            Some(SensorValue::F32(value)) if (*value - 261.2).abs() < f32::EPSILON
        ));
    }

    #[test]
    fn maps_stock_attachment_presence_bits_without_inventing_mop_detection() {
        let (groups, _) = SensorGroups::create(0.798, 0.229, 261.2);
        groups.process_mcu_frame(&report_frame(vec![
            0x24, 0x02, 0x00, 0x00, // dustbin present: bit 0 clear
            0x26, 0x02, 0x00, 0x00, // water box present: bit 1 clear
        ]));
        {
            let data = groups.attachments.lock().unwrap();
            assert!(bool_value(data.values.get("dustbin_attached")));
            assert!(bool_value(data.values.get("water_box_attached")));
            assert!(!bool_value(data.values.get("mop_detection_supported")));
            assert!(!data.values.contains_key("mop_attached"));
        }

        groups.process_mcu_frame(&report_frame(vec![
            0x24, 0x02, 0x01, 0x00, // dustbin absent: bit 0 set
            0x26, 0x02, 0x02, 0x00, // water box absent: bit 1 set
        ]));
        let data = groups.attachments.lock().unwrap();
        assert!(!bool_value(data.values.get("dustbin_attached")));
        assert!(!bool_value(data.values.get("water_box_attached")));
    }

    #[test]
    fn labels_bms_current_as_unsigned_magnitude() {
        let input = (0.."aa1001080a06407300610000000000d002012776".len())
            .step_by(2)
            .map(|index| {
                u8::from_str_radix(
                    &"aa1001080a06407300610000000000d002012776"[index..index + 2],
                    16,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        let frame = FrameDecoder::new().push(&input).pop().unwrap();
        let (groups, _) = SensorGroups::create(0.798, 0.229, 261.2);
        groups.process_mcu_frame(&frame);
        let data = groups.power_status.lock().unwrap();
        assert!(matches!(
            data.values.get("battery_current_magnitude_a"),
            Some(SensorValue::F32(value)) if (*value - 0.115).abs() < f32::EPSILON
        ));
        assert!(!data.values.contains_key("battery_current_a"));
    }
}
