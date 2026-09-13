//! Native Roborock S5 Max hardware driver.
//!
//! The driver exclusively owns the MCU and LDS UARTs, LDS motor controller,
//! and hardware watchdog. Actuation is gated by configuration, fresh safety
//! telemetry, bounded command leases, and ordered shutdown cleanup.

pub mod commands;
pub mod frame;
pub mod lds;
mod lds_motor;
mod lds_reader;
mod lifecycle;
mod mcu;
pub mod packet;
mod safety;
mod sensors;
mod sys;
mod tty;
mod watchdog;

use crate::config::DeviceConfig;
use crate::core::driver::{DeviceDriver, DriverInitResult};
use crate::core::types::{Command, ComponentAction, SensorValue};
use crate::error::{Error, Result};
use commands::OPERATIONAL_SUBSYSTEMS;
use lds_motor::LdsMotorGuard;
use lds_reader::LdsReader;
use lifecycle::{DriverState, Lifecycle};
use mcu::{ActuatorCommand, McuDriver};
use safety::SafetyController;
use sensors::SensorGroups;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use std::thread;
use std::time::{Duration, Instant};
use tty::ensure_devices_unowned;
use watchdog::HardwareWatchdog;

/// Read-oriented production driver for the Roborock S5 Max.
pub struct RoborockS5MaxDriver {
    config: DeviceConfig,
    shutdown: Arc<AtomicBool>,
    mcu: Option<McuDriver>,
    lds: Option<LdsReader>,
    lds_motor: Option<LdsMotorGuard>,
    watchdog: Option<HardwareWatchdog>,
    lifecycle: Lifecycle,
    safety: SafetyController,
    dock_state: Option<Arc<AtomicU16>>,
    restore_charging_on_shutdown: bool,
    initialized: bool,
}

impl RoborockS5MaxDriver {
    pub fn new(config: DeviceConfig) -> Result<Self> {
        if config.roborock_s5max.is_none() {
            return Err(Error::Config(
                "roborock_s5max device requires [device.roborock_s5max] section".to_string(),
            ));
        }
        let lifecycle = Lifecycle::new();
        let safety = SafetyController::new(lifecycle.clone());
        Ok(Self {
            config,
            shutdown: Arc::new(AtomicBool::new(false)),
            mcu: None,
            lds: None,
            lds_motor: None,
            watchdog: None,
            lifecycle,
            safety,
            dock_state: None,
            restore_charging_on_shutdown: false,
            initialized: false,
        })
    }

    fn shutdown_all(&mut self) -> Result<()> {
        let faulted = self.lifecycle.state() == DriverState::Fault;
        let mut first_error = None;

        if self.initialized {
            if let Some(mcu) = self.mcu.as_ref()
                && let Err(error) = mcu.execute(ActuatorCommand::EmergencyStop)
            {
                first_error.get_or_insert(error);
            }
            if let Some(motor) = self.lds_motor.as_ref()
                && let Err(error) = motor.stop()
            {
                first_error.get_or_insert(error);
            }
            self.safety.set_lds_running(false).ok();
            if self.restore_charging_on_shutdown && !faulted {
                let restore_result = self
                    .mcu
                    .as_ref()
                    .ok_or_else(|| Error::Other("S5 Max MCU is unavailable".to_string()))
                    .and_then(|mcu| mcu.execute(ActuatorCommand::Charger(true)))
                    .and_then(|()| self.wait_for_charging());
                if let Err(error) = restore_result {
                    first_error.get_or_insert(error);
                } else {
                    self.restore_charging_on_shutdown = false;
                }
            }
        }
        if let Err(error) = self.lifecycle.begin_stopping() {
            self.lifecycle.fault(error.to_string());
        }
        self.shutdown.store(true, Ordering::Release);

        if let Some(mut lds) = self.lds.take()
            && let Err(error) = lds.join()
        {
            first_error.get_or_insert(error);
        }
        if let Some(mut mcu) = self.mcu.take()
            && let Err(error) = mcu.join()
        {
            first_error.get_or_insert(error);
        }
        if let Some(mut lds_motor) = self.lds_motor.take()
            && let Err(error) = lds_motor.stop_and_disarm()
        {
            first_error.get_or_insert(error);
        }
        if let Some(mut watchdog) = self.watchdog.take()
            && let Err(error) = watchdog.join()
        {
            first_error.get_or_insert(error);
        }
        self.initialized = false;
        self.safety.reset_targets();

        if let Some(error) = first_error {
            self.lifecycle.fault(format!("shutdown: {error}"));
            Err(error)
        } else {
            if !faulted {
                self.lifecycle.mark_stopped()?;
            }
            Ok(())
        }
    }

    fn execute_actuator(&self, command: ActuatorCommand, nonzero: bool) -> Result<()> {
        let hardware = self.config.roborock_s5max.as_ref().ok_or_else(|| {
            Error::Config("missing Roborock S5 Max hardware configuration".to_string())
        })?;
        if nonzero {
            self.safety
                .nonzero_allowed(hardware.allow_actuation, std::time::Instant::now())?;
        }
        self.mcu
            .as_ref()
            .ok_or_else(|| Error::Other("S5 Max MCU is not initialized".to_string()))?
            .execute(command)
    }

    fn speed(action: &ComponentAction, default: u8) -> Result<u8> {
        match action {
            ComponentAction::Disable { .. } => Ok(0),
            ComponentAction::Enable { config } => config
                .as_ref()
                .and_then(|values| values.get("speed"))
                .map(Self::speed_value)
                .transpose()
                .map(|value| value.unwrap_or(default)),
            ComponentAction::Configure { config } => config
                .get("speed")
                .ok_or_else(|| Error::InvalidParameter("speed is required".to_string()))
                .and_then(Self::speed_value),
            ComponentAction::Reset { .. } => Err(Error::InvalidParameter(
                "Reset is only supported for drive emergency stop".to_string(),
            )),
        }
    }

    fn speed_value(value: &SensorValue) -> Result<u8> {
        let speed = match value {
            SensorValue::U8(value) => u32::from(*value),
            SensorValue::U16(value) => u32::from(*value),
            SensorValue::U32(value) => *value,
            _ => {
                return Err(Error::InvalidParameter(
                    "speed must be an unsigned integer from 0 to 100".to_string(),
                ));
            }
        };
        u8::try_from(speed)
            .ok()
            .filter(|speed| *speed <= 100)
            .ok_or_else(|| Error::InvalidParameter("speed must be from 0 to 100".to_string()))
    }

    fn drive_value(
        config: &std::collections::HashMap<String, SensorValue>,
        key: &str,
    ) -> Result<f32> {
        match config.get(key) {
            Some(SensorValue::F32(value)) => Ok(*value),
            Some(SensorValue::F64(value)) => Ok(*value as f32),
            _ => Err(Error::InvalidParameter(format!(
                "drive Configure requires numeric {key}"
            ))),
        }
    }

    fn wait_for_charging(&self) -> Result<()> {
        let dock_state = self
            .dock_state
            .as_ref()
            .ok_or_else(|| Error::Other("S5 Max dock state is unavailable".to_string()))?;
        let deadline = Instant::now() + Duration::from_secs(2);
        while Instant::now() < deadline {
            let state = dock_state.load(Ordering::Acquire);
            if state == 1 {
                return Ok(());
            }
            thread::sleep(Duration::from_millis(20));
        }
        Err(Error::Other(format!(
            "S5 Max charging transition was not confirmed; dock_state={}",
            dock_state.load(Ordering::Acquire)
        )))
    }

    fn set_charging(&mut self, enabled: bool) -> Result<()> {
        let was_docked = self
            .dock_state
            .as_ref()
            .is_some_and(|state| state.load(Ordering::Acquire) != 0);
        self.execute_actuator(ActuatorCommand::Charger(enabled), true)?;
        if !enabled {
            // b7=0 controls the charger, not physical dock contact. On the
            // dock the MCU may report a very short non-charging transition
            // and then return to state 1 while the contacts remain engaged.
            // The acknowledged command is therefore the only synchronous
            // completion condition for disable.
            self.restore_charging_on_shutdown |= was_docked;
            return Ok(());
        }
        self.wait_for_charging()?;
        self.restore_charging_on_shutdown = false;
        Ok(())
    }

    fn set_lidar(&mut self, enabled: bool) -> Result<()> {
        if !enabled {
            let stop_result = self.lds_motor.as_ref().map_or(Ok(()), LdsMotorGuard::stop);
            let safety_result = self.safety.set_lds_running(false);
            return stop_result.and(safety_result);
        }

        let hardware = self.config.roborock_s5max.as_ref().unwrap();
        self.safety
            .nonzero_allowed(hardware.allow_actuation, Instant::now())?;
        if self
            .lds_motor
            .as_ref()
            .is_some_and(LdsMotorGuard::is_running)
        {
            return Ok(());
        }
        let dock_state = self
            .dock_state
            .as_ref()
            .map_or(u16::MAX, |state| state.load(Ordering::Acquire));
        if dock_state != 0 || self.restore_charging_on_shutdown {
            return Err(Error::Other(format!(
                "S5 Max LDS start requires physical undock; dock_state={dock_state}"
            )));
        }
        // A permanent replacement can take ownership before the stock stack
        // has converted an undocked boot into the normal/discharge state. MCU
        // telemetry remains available there, but the LDS power domain does not
        // start. Reproduce the stock b7=0 discharge transition followed by its
        // b0 `sys_md\0\x00` wake before touching the motor controller. The b7
        // command is acknowledged synchronously; incoming mode confirmation is
        // handled and acknowledged by the MCU session worker.
        self.execute_actuator(ActuatorCommand::Charger(false), false)?;
        self.execute_actuator(ActuatorCommand::WakeMcu, false)?;
        self.execute_actuator(ActuatorCommand::StopDrive, false)?;
        // Submit each acknowledged transition separately so the MCU worker can
        // consume its ACK before the next subsystem record is sent.
        for &subsystem in OPERATIONAL_SUBSYSTEMS {
            self.execute_actuator(ActuatorCommand::EnableSubsystem(subsystem), false)?;
        }
        // Stock waits for the acknowledged operational subsystem transition
        // before its laser worker starts the separate Linux motor controller.
        thread::sleep(Duration::from_millis(250));
        let start_result = self
            .lds_motor
            .as_mut()
            .ok_or_else(|| Error::Other("S5 Max LDS motor is unavailable".to_string()))?
            .configure_and_start();
        if let Err(error) = start_result {
            if let Some(motor) = self.lds_motor.as_ref() {
                let _ = motor.stop();
            }
            return Err(error);
        }
        if let Err(error) = self.safety.set_lds_running(true) {
            let _ = self.set_lidar(false);
            return Err(error);
        }
        Ok(())
    }

    fn component_control(&mut self, id: &str, action: &ComponentAction) -> Result<()> {
        match id {
            "drive" => match action {
                ComponentAction::Disable { .. } => {
                    self.execute_actuator(ActuatorCommand::StopDrive, false)
                }
                ComponentAction::Reset { .. } => {
                    self.execute_actuator(ActuatorCommand::EmergencyStop, false)
                }
                ComponentAction::Configure { config } => {
                    let hardware = self.config.roborock_s5max.as_ref().unwrap();
                    let linear = Self::drive_value(config, "linear")?;
                    let angular = Self::drive_value(config, "angular")?;
                    if !linear.is_finite()
                        || !angular.is_finite()
                        || linear.abs() > 0.30
                        || angular.abs() > 1.60
                    {
                        return Err(Error::InvalidParameter(
                            "drive exceeds S5 Max limits (0.30 m/s, 1.60 rad/s)".to_string(),
                        ));
                    }
                    // Stock c0 captures use encoder ticks/20 ms for the linear
                    // field and rad/s for yaw: a commanded pi/2 turn settles
                    // near 1.57 on the MCU's yaw-rate report.
                    let ticks_per_20ms = linear * 20.0 / hardware.wheel_mm_per_tick;
                    self.execute_actuator(
                        ActuatorCommand::Drive {
                            linear: ticks_per_20ms,
                            angular,
                        },
                        linear != 0.0 || angular != 0.0,
                    )
                }
                ComponentAction::Enable { .. } => Ok(()),
            },
            "vacuum" => {
                let target = Self::speed(action, 100)?;
                self.execute_actuator(ActuatorCommand::Fan(target), target != 0)
            }
            "main_brush" => {
                let target = Self::speed(action, 71)?;
                self.execute_actuator(ActuatorCommand::MainBrush(target), target != 0)
            }
            "side_brush" => {
                let target = Self::speed(action, 30)?;
                self.execute_actuator(ActuatorCommand::SideBrush(target), target != 0)
            }
            "water_pump" => {
                let target = Self::speed(action, 100)?;
                if target != 0 {
                    return Err(Error::NotImplemented(
                        "S5 Max water-pump duty scheduling is not implemented".to_string(),
                    ));
                }
                self.execute_actuator(
                    ActuatorCommand::WaterPump {
                        on_interval: 0,
                        off_interval: 0xff,
                    },
                    false,
                )
            }
            "lidar" => match action {
                ComponentAction::Enable { .. } => self.set_lidar(true),
                ComponentAction::Disable { .. } => self.set_lidar(false),
                _ => Err(Error::InvalidParameter(
                    "S5 Max lidar supports only Enable and Disable".to_string(),
                )),
            },
            "charger" => match action {
                ComponentAction::Enable { .. } => self.set_charging(true),
                ComponentAction::Disable { .. } => self.set_charging(false),
                _ => Err(Error::InvalidParameter(
                    "S5 Max charger supports only Enable and Disable".to_string(),
                )),
            },
            _ => Err(Error::InvalidParameter(format!(
                "unsupported S5 Max component {id:?}"
            ))),
        }
    }
}

impl DeviceDriver for RoborockS5MaxDriver {
    fn initialize(&mut self) -> Result<DriverInitResult> {
        if self.initialized {
            return Err(Error::Other(
                "Roborock S5 Max driver is already initialized".to_string(),
            ));
        }
        let hardware = self.config.roborock_s5max.clone().ok_or_else(|| {
            Error::Config(
                "roborock_s5max device requires [device.roborock_s5max] section".to_string(),
            )
        })?;
        let last_stock_sequence = hardware.last_stock_sequence.ok_or_else(|| {
            Error::Config(
                "device.roborock_s5max.last_stock_sequence is required at runtime; use 0 only for ownership from MCU reset"
                    .to_string(),
            )
        })?;

        let selected_devices = [
            hardware.mcu_port.as_str(),
            "/dev/uart_mcu",
            hardware.lds_port.as_str(),
            "/dev/uart_lds",
            hardware.lds_motor.as_str(),
            hardware.watchdog.as_str(),
        ];
        ensure_devices_unowned(&selected_devices)?;
        self.lifecycle.begin_synchronizing()?;

        self.shutdown.store(false, Ordering::Release);
        let (sensors, mut result) = SensorGroups::create(
            hardware.wheel_mm_per_tick,
            hardware.wheel_track_m,
            hardware.lds_forward_angle_deg,
        );
        self.dock_state = Some(sensors.dock_state());
        result
            .sensor_data
            .insert("driver_status".to_string(), self.lifecycle.sensor_data());

        let watchdog = match HardwareWatchdog::start(
            &hardware.watchdog,
            self.lifecycle.clone(),
            Arc::clone(&self.shutdown),
        ) {
            Ok(watchdog) => watchdog,
            Err(error) => {
                self.lifecycle.fault(format!("watchdog startup: {error}"));
                let _ = self.shutdown_all();
                return Err(error);
            }
        };
        self.watchdog = Some(watchdog);

        let lds_motor = match LdsMotorGuard::open(&hardware.lds_motor) {
            Ok(motor) => motor,
            Err(error) => {
                self.lifecycle
                    .fault(format!("LDS motor ownership: {error}"));
                let _ = self.shutdown_all();
                return Err(error);
            }
        };
        self.lds_motor = Some(lds_motor);

        let mcu = match McuDriver::start(
            &hardware.mcu_port,
            last_stock_sequence,
            sensors.clone(),
            self.lifecycle.clone(),
            self.safety.clone(),
            Arc::clone(&self.shutdown),
        ) {
            Ok(mcu) => mcu,
            Err(error) => {
                self.lifecycle.fault(format!("MCU startup: {error}"));
                let _ = self.shutdown_all();
                return Err(error);
            }
        };
        self.mcu = Some(mcu);
        if let Err(error) = self.lifecycle.mark_read_only_ready() {
            self.lifecycle.fault(error.to_string());
            let _ = self.shutdown_all();
            return Err(error);
        }

        let feedback_motor = match self
            .lds_motor
            .as_ref()
            .expect("LDS motor guard initialized above")
            .feedback_file()
        {
            Ok(file) => file,
            Err(error) => {
                self.lifecycle
                    .fault(format!("LDS feedback ownership: {error}"));
                let _ = self.shutdown_all();
                return Err(error);
            }
        };
        let motor_running = self
            .lds_motor
            .as_ref()
            .expect("LDS motor guard initialized above")
            .running_flag();
        let lds = match LdsReader::start(
            &hardware.lds_port,
            hardware.lds_forward_angle_deg,
            Arc::clone(&sensors.lidar),
            self.lifecycle.clone(),
            Arc::clone(&self.shutdown),
            feedback_motor,
            motor_running,
        ) {
            Ok(lds) => lds,
            Err(error) => {
                self.lifecycle.fault(format!("LDS startup: {error}"));
                let _ = self.shutdown_all();
                return Err(error);
            }
        };
        self.lds = Some(lds);
        if let Err(error) = self.lifecycle.mark_operational() {
            self.lifecycle.fault(error.to_string());
            let _ = self.shutdown_all();
            return Err(error);
        }
        self.initialized = true;
        log::info!("Roborock S5 Max driver initialized: {}", self.config.name);
        Ok(result)
    }

    fn send_command(&mut self, cmd: Command) -> Result<()> {
        match cmd {
            Command::Shutdown => self.shutdown_all(),
            Command::ComponentControl { id, action } => self.component_control(&id, &action),
            _ => Err(Error::NotImplemented(
                "unsupported Roborock S5 Max command".to_string(),
            )),
        }
    }
}

impl Drop for RoborockS5MaxDriver {
    fn drop(&mut self) {
        if let Err(error) = self.shutdown_all() {
            log::error!("Roborock S5 Max driver shutdown failed: {error}");
        }
    }
}
