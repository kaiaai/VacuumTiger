//! Single-owner MCU session with framed reports and guarded command dispatch.

use super::commands::{
    ACK_REPORT, FAN_SUBSYSTEM, MAIN_BRUSH_SUBSYSTEM, SIDE_BRUSH_SUBSYSTEM,
    WHEEL_ODOMETRY_SUBSYSTEM, acknowledgement_frame, charger_command_frame, fan_command_frame,
    heartbeat_frame, main_brush_command_frame, next_tx_sequence, sensor_sync_query_frame,
    side_brush_command_frame, subsystem_state_frame, system_mode_command_frame,
    water_pump_command_frame, wheel_command_frame,
};
use super::lifecycle::{Lifecycle, SAFETY_REPORT_TIMEOUT};
use super::packet::{FrameDecoder, ReportFooter};
use super::safety::SafetyController;
use super::sensors::SensorGroups;
use super::tty::ExclusiveTty;
use crate::error::{Error, Result};
use crossbeam_channel::{Receiver, Sender, bounded};
use std::cell::Cell;
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Condvar, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const MCU_BAUD: libc::speed_t = libc::B1152000;
const MCU_BAUD_NUMBER: u32 = 1_152_000;
const HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);
const ACK_TIMEOUT: Duration = Duration::from_millis(23);
const MAX_SYNC_ATTEMPTS: u8 = 3;
const WORKER_STARTUP_TIMEOUT: Duration = Duration::from_millis(1_500);
const STARTUP_WAIT_TIMEOUT: Duration = Duration::from_secs(2);
const ACK_LENGTH: u8 = 1;
const ACK_DEDUP_WINDOW_FRAMES: u32 = 128;
const COMMAND_QUEUE_CAPACITY: usize = 8;
const COMMAND_TIMEOUT: Duration = Duration::from_millis(200);

#[derive(Debug, Clone, Copy)]
pub(crate) enum ActuatorCommand {
    WakeMcu,
    EnableSubsystem(u32),
    Drive { linear: f32, angular: f32 },
    StopDrive,
    Fan(u8),
    MainBrush(u8),
    SideBrush(u8),
    WaterPump { on_interval: u8, off_interval: u8 },
    Charger(bool),
    EmergencyStop,
}

struct CommandRequest {
    command: ActuatorCommand,
    reply: Sender<Result<Vec<u8>>>,
}

struct SessionChannels {
    ready: Sender<Result<()>>,
    commands: Receiver<CommandRequest>,
    command_acks: Arc<CommandAckTracker>,
}

const DOCK_STATUS_REPORT: u8 = 0x20;
const BATTERY_STATUS_REPORT: u8 = 0x08;
const DUSTBIN_STATUS_REPORT: u8 = 0x24;
const DOCK_VOLTAGE_STATUS_REPORT: u8 = 0x0f;

#[derive(Debug)]
struct RecentAckRequest {
    frame_index: u32,
    sequence: u8,
    payload: Vec<u8>,
}

#[derive(Debug, Default)]
struct RecentAckRequests {
    entries: Vec<RecentAckRequest>,
}

impl RecentAckRequests {
    fn is_duplicate(&mut self, frame_index: u32, sequence: u8, payload: &[u8]) -> bool {
        self.entries.retain(|entry| {
            frame_index.saturating_sub(entry.frame_index) < ACK_DEDUP_WINDOW_FRAMES
        });
        if self
            .entries
            .iter()
            .any(|entry| entry.sequence == sequence && entry.payload == payload)
        {
            return true;
        }
        self.entries.push(RecentAckRequest {
            frame_index,
            sequence,
            payload: payload.to_vec(),
        });
        false
    }
}

#[derive(Debug)]
struct AckRetryWindow {
    attempts: u8,
    next_retry: Instant,
}

#[derive(Debug, Default)]
struct CommandAckTracker {
    states: Mutex<HashMap<u8, CommandAckState>>,
    changed: Condvar,
}

#[derive(Debug)]
struct CommandAckState {
    acknowledged: bool,
    frame: Vec<u8>,
    attempts: u8,
    next_retry: Instant,
}

impl CommandAckTracker {
    fn expect(&self, sequence: u8, frame: &[u8], now: Instant) -> Result<()> {
        self.states
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max command ACK tracker".to_string()))?
            .insert(
                sequence,
                CommandAckState {
                    acknowledged: false,
                    frame: frame.to_vec(),
                    attempts: 1,
                    next_retry: now + ACK_TIMEOUT,
                },
            );
        Ok(())
    }

    fn cancel(&self, sequence: u8) {
        if let Ok(mut states) = self.states.lock() {
            states.remove(&sequence);
        }
    }

    fn acknowledge(&self, sequence: u8) {
        if let Ok(mut states) = self.states.lock()
            && let Some(state) = states.get_mut(&sequence)
        {
            state.acknowledged = true;
            self.changed.notify_all();
        }
    }

    fn take_due_retries(&self, now: Instant) -> Result<Vec<(u8, Vec<u8>, u8)>> {
        let mut states = self
            .states
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max command ACK tracker".to_string()))?;
        let mut retries = Vec::new();
        for (&sequence, state) in states.iter_mut() {
            if !state.acknowledged && state.attempts < MAX_SYNC_ATTEMPTS && now >= state.next_retry
            {
                state.attempts += 1;
                state.next_retry = now + ACK_TIMEOUT;
                retries.push((sequence, state.frame.clone(), state.attempts));
            }
        }
        Ok(retries)
    }

    fn wait_for_all(&self, sequences: &[u8], timeout: Duration) -> Result<()> {
        if sequences.is_empty() {
            return Ok(());
        }
        let states = self
            .states
            .lock()
            .map_err(|_| Error::MutexPoisoned("S5 Max command ACK tracker".to_string()))?;
        let (mut states, _) = self
            .changed
            .wait_timeout_while(states, timeout, |states| {
                sequences
                    .iter()
                    .any(|sequence| !states.get(sequence).is_some_and(|state| state.acknowledged))
            })
            .map_err(|_| Error::MutexPoisoned("S5 Max command ACK tracker".to_string()))?;
        let missing = sequences
            .iter()
            .copied()
            .filter(|sequence| !states.get(sequence).is_some_and(|state| state.acknowledged))
            .collect::<Vec<_>>();
        for sequence in sequences {
            states.remove(sequence);
        }
        if missing.is_empty() {
            Ok(())
        } else {
            Err(Error::Other(format!(
                "MCU command ACK timeout; missing sequences {missing:02x?}"
            )))
        }
    }
}

impl AckRetryWindow {
    fn after_initial_send(now: Instant) -> Self {
        Self {
            attempts: 1,
            next_retry: now + ACK_TIMEOUT,
        }
    }

    fn take_retry(&mut self, now: Instant) -> bool {
        if now < self.next_retry || self.attempts >= MAX_SYNC_ATTEMPTS {
            return false;
        }
        self.attempts += 1;
        self.next_retry = now + ACK_TIMEOUT;
        true
    }

    fn exhausted(&self, now: Instant) -> bool {
        self.attempts == MAX_SYNC_ATTEMPTS && now >= self.next_retry
    }
}

pub(crate) struct McuDriver {
    handle: Option<JoinHandle<Result<()>>>,
    command_tx: Sender<CommandRequest>,
    command_acks: Arc<CommandAckTracker>,
}

impl McuDriver {
    pub(crate) fn start(
        path: &str,
        last_stock_sequence: u8,
        sensors: SensorGroups,
        lifecycle: Lifecycle,
        safety: SafetyController,
        shutdown: Arc<AtomicBool>,
    ) -> Result<Self> {
        let tty = ExclusiveTty::open(path, MCU_BAUD, MCU_BAUD_NUMBER)?;
        let (ready_tx, ready_rx) = bounded(1);
        let (command_tx, command_rx) = bounded(COMMAND_QUEUE_CAPACITY);
        let command_acks = Arc::new(CommandAckTracker::default());
        let worker_command_acks = Arc::clone(&command_acks);
        let path = path.to_string();
        let thread_shutdown = Arc::clone(&shutdown);
        let thread_lifecycle = lifecycle.clone();
        let handle = thread::Builder::new()
            .name("s5max-mcu".to_string())
            .spawn(move || {
                let result = session_loop(
                    tty,
                    last_stock_sequence,
                    sensors,
                    lifecycle,
                    safety,
                    &thread_shutdown,
                    SessionChannels {
                        ready: ready_tx,
                        commands: command_rx,
                        command_acks: worker_command_acks,
                    },
                );
                if let Err(error) = &result {
                    log::error!("S5 Max MCU worker failed on {path}: {error}");
                    thread_lifecycle.fault(format!("MCU worker: {error}"));
                    thread_shutdown.store(true, Ordering::Release);
                }
                result
            })
            .map_err(|error| Error::Other(format!("spawn S5 Max MCU worker: {error}")))?;
        let mut driver = Self {
            handle: Some(handle),
            command_tx,
            command_acks,
        };

        match ready_rx.recv_timeout(STARTUP_WAIT_TIMEOUT) {
            Ok(Ok(())) => Ok(driver),
            Ok(Err(error)) => {
                shutdown.store(true, Ordering::Release);
                let _ = driver.join();
                Err(error)
            }
            Err(error) => {
                shutdown.store(true, Ordering::Release);
                let _ = driver.join();
                Err(Error::Other(format!(
                    "S5 Max MCU startup did not complete within {}ms: {error}",
                    STARTUP_WAIT_TIMEOUT.as_millis()
                )))
            }
        }
    }

    pub(crate) fn execute(&self, command: ActuatorCommand) -> Result<()> {
        let (reply_tx, reply_rx) = bounded(1);
        self.command_tx
            .send_timeout(
                CommandRequest {
                    command,
                    reply: reply_tx,
                },
                COMMAND_TIMEOUT,
            )
            .map_err(|error| Error::Other(format!("S5 Max MCU command queue: {error}")))?;
        let acknowledged_sequences = reply_rx
            .recv_timeout(COMMAND_TIMEOUT)
            .map_err(|error| Error::Other(format!("S5 Max MCU command timeout: {error}")))??;
        self.command_acks
            .wait_for_all(&acknowledged_sequences, COMMAND_TIMEOUT)
    }

    pub(crate) fn join(&mut self) -> Result<()> {
        let Some(handle) = self.handle.take() else {
            return Ok(());
        };
        handle.join().map_err(|_| Error::ThreadPanic)?
    }
}

struct McuSafetyGuard<'a> {
    tty: &'a ExclusiveTty,
    next_sequence: Cell<u8>,
    wheel_subsystem_enabled: Cell<bool>,
    safety: SafetyController,
    command_acks: Arc<CommandAckTracker>,
    armed: bool,
}

impl<'a> McuSafetyGuard<'a> {
    fn new(
        tty: &'a ExclusiveTty,
        first_sequence: u8,
        safety: SafetyController,
        command_acks: Arc<CommandAckTracker>,
    ) -> Self {
        Self {
            tty,
            next_sequence: Cell::new(first_sequence),
            wheel_subsystem_enabled: Cell::new(false),
            safety,
            command_acks,
            armed: true,
        }
    }

    fn take_sequence(&self) -> u8 {
        let sequence = self.next_sequence.get();
        self.next_sequence.set(next_tx_sequence(sequence));
        sequence
    }

    fn send(&self, frame: std::result::Result<Vec<u8>, String>) -> Result<()> {
        self.tty.write_complete(&frame.map_err(Error::Other)?)
    }

    fn send_requiring_ack(
        &self,
        sequence: u8,
        frame: std::result::Result<Vec<u8>, String>,
    ) -> Result<u8> {
        let frame = frame.map_err(Error::Other)?;
        self.command_acks.expect(sequence, &frame, Instant::now())?;
        if let Err(error) = self.tty.write_complete(&frame) {
            self.command_acks.cancel(sequence);
            return Err(error);
        }
        Ok(sequence)
    }

    fn forced_wheel_stop(&self) -> Result<()> {
        self.send(wheel_command_frame(self.take_sequence(), 0.0, 0.0, true))
    }

    fn execute(&self, command: ActuatorCommand) -> Result<Vec<u8>> {
        let mut acknowledgements = Vec::new();
        match command {
            ActuatorCommand::WakeMcu => {
                self.send(system_mode_command_frame(self.take_sequence(), 0))?;
            }
            ActuatorCommand::EnableSubsystem(subsystem) => {
                let sequence = self.take_sequence();
                acknowledgements.push(self.send_requiring_ack(
                    sequence,
                    subsystem_state_frame(sequence, subsystem, true),
                )?);
                if subsystem == WHEEL_ODOMETRY_SUBSYSTEM {
                    self.wheel_subsystem_enabled.set(true);
                }
            }
            ActuatorCommand::Drive { linear, angular } => {
                if (linear != 0.0 || angular != 0.0) && !self.wheel_subsystem_enabled.get() {
                    let sequence = self.take_sequence();
                    acknowledgements.push(self.send_requiring_ack(
                        sequence,
                        subsystem_state_frame(sequence, WHEEL_ODOMETRY_SUBSYSTEM, true),
                    )?);
                    // The b1 selector enables the subsystem for the MCU session;
                    // periodic c0 velocity refreshes must not resend it. Doing so
                    // floods the acknowledgement window and eventually wraps the
                    // one-byte transmit sequence.
                    self.wheel_subsystem_enabled.set(true);
                }
                self.send(wheel_command_frame(
                    self.take_sequence(),
                    linear,
                    angular,
                    false,
                ))?;
                self.safety
                    .arm_drive_lease(linear, angular, Instant::now())?;
            }
            ActuatorCommand::StopDrive => {
                self.forced_wheel_stop()?;
                self.safety.clear_drive_target()?;
            }
            ActuatorCommand::Fan(target) => {
                if target != 0 {
                    let sequence = self.take_sequence();
                    acknowledgements.push(self.send_requiring_ack(
                        sequence,
                        subsystem_state_frame(sequence, FAN_SUBSYSTEM, true),
                    )?);
                }
                let sequence = self.take_sequence();
                acknowledgements
                    .push(self.send_requiring_ack(sequence, fan_command_frame(sequence, target))?);
                self.safety.set_fan(target)?;
            }
            ActuatorCommand::MainBrush(target) => {
                if target != 0 {
                    let sequence = self.take_sequence();
                    acknowledgements.push(self.send_requiring_ack(
                        sequence,
                        subsystem_state_frame(sequence, MAIN_BRUSH_SUBSYSTEM, true),
                    )?);
                }
                let sequence = self.take_sequence();
                acknowledgements.push(self.send_requiring_ack(
                    sequence,
                    main_brush_command_frame(sequence, target, false),
                )?);
                self.safety.set_main_brush(target)?;
            }
            ActuatorCommand::SideBrush(target) => {
                if target != 0 {
                    let sequence = self.take_sequence();
                    acknowledgements.push(self.send_requiring_ack(
                        sequence,
                        subsystem_state_frame(sequence, SIDE_BRUSH_SUBSYSTEM, true),
                    )?);
                }
                let sequence = self.take_sequence();
                acknowledgements.push(self.send_requiring_ack(
                    sequence,
                    side_brush_command_frame(sequence, target, false),
                )?);
                self.safety.set_side_brush(target)?;
            }
            ActuatorCommand::WaterPump {
                on_interval,
                off_interval,
            } => {
                self.send(water_pump_command_frame(
                    self.take_sequence(),
                    on_interval,
                    off_interval,
                ))?;
                self.safety.set_water_pump(on_interval)?;
            }
            ActuatorCommand::Charger(enabled) => {
                let sequence = self.take_sequence();
                acknowledgements.push(
                    self.send_requiring_ack(sequence, charger_command_frame(sequence, enabled))?,
                );
            }
            ActuatorCommand::EmergencyStop => self.emit_stop_set()?,
        }
        Ok(acknowledgements)
    }

    fn emit_stop_set(&self) -> Result<()> {
        let mut failures = Vec::new();
        let (records, next_sequence) = ordered_stop_frames(self.next_sequence.get());
        self.next_sequence.set(next_sequence);
        for (description, frame) in records {
            if let Err(error) = self.send(frame) {
                failures.push(format!("{description}: {error}"));
            }
        }
        self.safety.reset_targets();
        if failures.is_empty() {
            Ok(())
        } else {
            Err(Error::Other(failures.join("; ")))
        }
    }

    fn stop_and_disarm(&mut self) -> Result<()> {
        self.emit_stop_set()?;
        self.armed = false;
        Ok(())
    }
}

type StopRecord = (&'static str, std::result::Result<Vec<u8>, String>);

fn ordered_stop_frames(first_sequence: u8) -> ([StopRecord; 5], u8) {
    let wheel_sequence = first_sequence;
    let fan_sequence = next_tx_sequence(wheel_sequence);
    let side_brush_sequence = next_tx_sequence(fan_sequence);
    let main_brush_sequence = next_tx_sequence(side_brush_sequence);
    let water_pump_sequence = next_tx_sequence(main_brush_sequence);
    let next_sequence = next_tx_sequence(water_pump_sequence);
    (
        [
            (
                "forced zero-wheel stop",
                wheel_command_frame(wheel_sequence, 0.0, 0.0, true),
            ),
            ("hard fan stop", fan_command_frame(fan_sequence, 0)),
            (
                "side-brush stop",
                side_brush_command_frame(side_brush_sequence, 0, false),
            ),
            (
                "main-brush stop",
                main_brush_command_frame(main_brush_sequence, 0, false),
            ),
            (
                "water-pump stop",
                water_pump_command_frame(water_pump_sequence, 0, 0),
            ),
        ],
        next_sequence,
    )
}

impl Drop for McuSafetyGuard<'_> {
    fn drop(&mut self) {
        if self.armed
            && let Err(error) = self.emit_stop_set()
        {
            log::error!("S5 Max emergency stop retry failed: {error}");
        }
    }
}

fn contains_sensor_snapshot(frame: &super::packet::Frame) -> bool {
    let shape = frame
        .reports()
        .map(|report| (report.message, report.data.len()))
        .collect::<Vec<_>>();
    [
        (DOCK_STATUS_REPORT, 2),
        (BATTERY_STATUS_REPORT, 10),
        (DUSTBIN_STATUS_REPORT, 2),
        (DOCK_VOLTAGE_STATUS_REPORT, 2),
    ]
    .into_iter()
    .all(|required| shape.contains(&required))
}

fn report_requests_ack(footer: Option<ReportFooter>) -> bool {
    footer.is_some_and(|footer| footer.reports_pending)
}

fn notify_startup(sender: &mut Option<Sender<Result<()>>>, result: Result<()>) {
    if let Some(sender) = sender.take() {
        let _ = sender.send(result);
    }
}

fn session_loop(
    tty: ExclusiveTty,
    last_stock_sequence: u8,
    sensors: SensorGroups,
    lifecycle: Lifecycle,
    safety: SafetyController,
    shutdown: &AtomicBool,
    channels: SessionChannels,
) -> Result<()> {
    let first_sequence = next_tx_sequence(last_stock_sequence);
    let mut guard = McuSafetyGuard::new(
        &tty,
        first_sequence,
        safety,
        Arc::clone(&channels.command_acks),
    );
    let session_result = guarded_session_loop(&guard, sensors, lifecycle, shutdown, channels);
    let stop_result = guard.stop_and_disarm();
    match (session_result, stop_result) {
        (Err(session), Err(stop)) => Err(Error::Other(format!(
            "{session}; ordered safety stop also failed: {stop}"
        ))),
        (Err(error), Ok(())) | (Ok(()), Err(error)) => Err(error),
        (Ok(()), Ok(())) => Ok(()),
    }
}

fn guarded_session_loop(
    guard: &McuSafetyGuard<'_>,
    sensors: SensorGroups,
    lifecycle: Lifecycle,
    shutdown: &AtomicBool,
    channels: SessionChannels,
) -> Result<()> {
    let SessionChannels {
        ready,
        commands: command_rx,
        command_acks: _,
    } = channels;
    let mut startup_sender = Some(ready);
    let startup_deadline = Instant::now() + WORKER_STARTUP_TIMEOUT;

    guard.send(heartbeat_frame(guard.take_sequence()))?;
    let mut next_heartbeat = Instant::now() + HEARTBEAT_INTERVAL;

    let sync_sequence = guard.take_sequence();
    let sync_frame = sensor_sync_query_frame(sync_sequence).map_err(Error::Other)?;
    guard.tty.write_complete(&sync_frame)?;
    let mut sync_retry = AckRetryWindow::after_initial_send(Instant::now());
    let mut sync_acknowledged = false;
    let mut snapshot_received = false;
    let mut safety_report_deadline = None;

    let mut decoder = FrameDecoder::new();
    let mut recent_ack_requests = RecentAckRequests::default();
    let mut binary_frame_index = 0u32;
    let mut buffer = [0u8; 1024];

    while !shutdown.load(Ordering::Acquire) {
        let now = Instant::now();
        while let Ok(request) = command_rx.try_recv() {
            let result = guard.execute(request.command);
            let failed = result.is_err();
            let _ = request.reply.send(result);
            if failed {
                return Err(Error::Other("S5 Max actuator command failed".to_string()));
            }
        }
        for (sequence, frame, attempt) in guard.command_acks.take_due_retries(now)? {
            log::warn!("Retrying S5 Max MCU command sequence 0x{sequence:02x}, attempt {attempt}");
            guard.tty.write_complete(&frame)?;
        }
        if guard.safety.take_expired_drive_lease(now) {
            log::warn!("S5 Max drive lease expired; sending forced zero-wheel stop");
            guard.forced_wheel_stop()?;
        }
        if safety_report_deadline.is_some_and(|deadline| now >= deadline) {
            return Err(Error::Other(format!(
                "S5 Max safety reports stale for more than {}ms",
                SAFETY_REPORT_TIMEOUT.as_millis()
            )));
        }
        if now >= next_heartbeat {
            guard.send(heartbeat_frame(guard.take_sequence()))?;
            next_heartbeat = now + HEARTBEAT_INTERVAL;
        }

        if !sync_acknowledged && sync_retry.take_retry(now) {
            guard.tty.write_complete(&sync_frame)?;
        }

        if startup_sender.is_some() && !sync_acknowledged && sync_retry.exhausted(now) {
            let error = Error::Other(format!(
                "MCU did not acknowledge sensor sync after {} attempts",
                sync_retry.attempts
            ));
            notify_startup(&mut startup_sender, Err(error));
            return Err(Error::Other(
                "MCU sensor-sync acknowledgement exhausted".to_string(),
            ));
        }
        if startup_sender.is_some() && now >= startup_deadline {
            let missing = match (sync_acknowledged, snapshot_received) {
                (false, false) => "sync acknowledgement and sensor snapshot",
                (false, true) => "sync acknowledgement",
                (true, false) => "sensor snapshot",
                (true, true) => unreachable!(),
            };
            notify_startup(
                &mut startup_sender,
                Err(Error::Other(format!(
                    "MCU startup timed out waiting for {missing}"
                ))),
            );
            return Err(Error::Other(format!(
                "MCU startup timed out waiting for {missing}"
            )));
        }

        if !guard.tty.poll_readable(Duration::from_millis(20))? {
            continue;
        }
        let count = guard.tty.read_available(&mut buffer)?;
        for frame in decoder.push(&buffer[..count]) {
            if frame.is_log {
                continue;
            }
            if !frame.crc_valid || !frame.escape_overhead_valid() {
                log::warn!("Discarding invalid S5 Max MCU frame");
                continue;
            }
            binary_frame_index = binary_frame_index.wrapping_add(1);

            let footer = frame.report_footer();
            let duplicate = footer.is_some_and(|footer| {
                footer.reports_pending
                    && recent_ack_requests.is_duplicate(
                        binary_frame_index,
                        footer.sequence,
                        &frame.payload,
                    )
            });

            if frame.message == ACK_REPORT
                && frame.command == ACK_LENGTH
                && let Some(sequence) = frame.payload.get(2).copied()
            {
                guard.command_acks.acknowledge(sequence);
                if sequence == sync_sequence {
                    sync_acknowledged = true;
                }
            }
            if contains_sensor_snapshot(&frame) && !duplicate {
                snapshot_received = true;
            }
            if !duplicate {
                sensors.process_mcu_frame(&frame);
                if frame.is_state_report() {
                    lifecycle.mark_safety_report(now);
                    safety_report_deadline = Some(now + SAFETY_REPORT_TIMEOUT);
                }
            }

            if report_requests_ack(footer) {
                let footer = footer.expect("ACK request requires footer");
                guard.send(acknowledgement_frame(
                    guard.take_sequence(),
                    footer.sequence,
                ))?;
            }

            if startup_sender.is_some() && sync_acknowledged && snapshot_received {
                notify_startup(&mut startup_sender, Ok(()));
                safety_report_deadline = Some(now + SAFETY_REPORT_TIMEOUT);
                log::info!(
                    "S5 Max MCU synchronized after {} attempt(s)",
                    sync_retry.attempts
                );
            }
        }
    }

    if startup_sender.is_some() {
        notify_startup(
            &mut startup_sender,
            Err(Error::Other("MCU shutdown during startup".to_string())),
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{
        ACK_TIMEOUT, AckRetryWindow, CommandAckTracker, RecentAckRequests,
        contains_sensor_snapshot, notify_startup, ordered_stop_frames,
    };
    use crate::devices::roborock_s5max::commands::{
        fan_command_frame, main_brush_command_frame, side_brush_command_frame,
        water_pump_command_frame, wheel_command_frame,
    };
    use crate::devices::roborock_s5max::packet::Frame;
    use crossbeam_channel::{TryRecvError, bounded};
    use std::time::{Duration, Instant};

    #[test]
    fn matches_command_acknowledgements_by_sequence() {
        let tracker = CommandAckTracker::default();
        let now = Instant::now();
        tracker.expect(0x42, &[0x42], now).unwrap();
        tracker.expect(0x43, &[0x43], now).unwrap();
        tracker.acknowledge(0x43);
        tracker.acknowledge(0x42);
        tracker.wait_for_all(&[0x42, 0x43], Duration::ZERO).unwrap();
    }

    #[test]
    fn rejects_a_missing_command_acknowledgement() {
        let tracker = CommandAckTracker::default();
        tracker.expect(0x42, &[0x42], Instant::now()).unwrap();
        let error = tracker.wait_for_all(&[0x42], Duration::ZERO).unwrap_err();
        assert!(error.to_string().contains("42"));
    }

    #[test]
    fn retries_an_unacknowledged_command_twice() {
        let tracker = CommandAckTracker::default();
        let start = Instant::now();
        tracker.expect(0x42, &[0xaa, 0x42], start).unwrap();
        assert!(tracker.take_due_retries(start).unwrap().is_empty());
        assert_eq!(
            tracker.take_due_retries(start + ACK_TIMEOUT).unwrap(),
            vec![(0x42, vec![0xaa, 0x42], 2)]
        );
        assert_eq!(
            tracker.take_due_retries(start + ACK_TIMEOUT * 2).unwrap(),
            vec![(0x42, vec![0xaa, 0x42], 3)]
        );
        assert!(
            tracker
                .take_due_retries(start + ACK_TIMEOUT * 3)
                .unwrap()
                .is_empty()
        );
    }

    #[test]
    fn deduplicates_only_exact_recent_ack_requests() {
        let mut tracker = RecentAckRequests::default();
        assert!(!tracker.is_duplicate(10, 0x44, &[0x20, 0x02, 0x01, 0x00]));
        assert!(tracker.is_duplicate(12, 0x44, &[0x20, 0x02, 0x01, 0x00]));
        assert!(!tracker.is_duplicate(13, 0x44, &[0x24, 0x02, 0x01, 0x00]));
        assert!(!tracker.is_duplicate(140, 0x44, &[0x20, 0x02, 0x01, 0x00]));
    }

    #[test]
    fn requires_the_complete_startup_snapshot_shape() {
        let frame = |payload| Frame {
            raw: Vec::new(),
            payload,
            escape_overhead: 0,
            message: 0x0b,
            command: 0x08,
            crc: 0,
            crc_valid: true,
            is_log: false,
        };
        let complete = frame(vec![
            0x20, 2, 1, 0, 0x08, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x24, 2, 0, 0, 0x0f, 2, 0, 0,
        ]);
        assert!(contains_sensor_snapshot(&complete));

        let incomplete = frame(vec![
            0x20, 2, 1, 0, 0x08, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x24, 2, 0, 0,
        ]);
        assert!(!contains_sensor_snapshot(&incomplete));
    }

    #[test]
    fn ordered_stop_set_attempts_every_actuator_and_advances_sequence() {
        let (records, next_sequence) = ordered_stop_frames(0xfd);
        assert_eq!(next_sequence, 0x03);
        assert_eq!(records[0].0, "forced zero-wheel stop");
        assert_eq!(
            records[0].1.as_ref().unwrap(),
            &wheel_command_frame(0xfd, 0.0, 0.0, true).unwrap()
        );
        assert_eq!(
            records[1].1.as_ref().unwrap(),
            &fan_command_frame(0xfe, 0).unwrap()
        );
        assert_eq!(
            records[2].1.as_ref().unwrap(),
            &side_brush_command_frame(0xff, 0, false).unwrap()
        );
        assert_eq!(
            records[3].1.as_ref().unwrap(),
            &main_brush_command_frame(0x01, 0, false).unwrap()
        );
        assert_eq!(
            records[4].1.as_ref().unwrap(),
            &water_pump_command_frame(0x02, 0, 0).unwrap()
        );
    }

    #[test]
    fn exhausts_three_attempts_when_an_ack_is_lost() {
        let start = Instant::now();
        let mut retry = AckRetryWindow::after_initial_send(start);
        assert!(!retry.take_retry(start + ACK_TIMEOUT - std::time::Duration::from_millis(1)));
        assert!(retry.take_retry(start + ACK_TIMEOUT));
        assert!(retry.take_retry(start + ACK_TIMEOUT * 2));
        assert!(!retry.exhausted(start + ACK_TIMEOUT * 3 - std::time::Duration::from_millis(1)));
        assert!(retry.exhausted(start + ACK_TIMEOUT * 3));
        assert!(!retry.take_retry(start + ACK_TIMEOUT * 4));
        assert_eq!(retry.attempts, 3);
    }

    #[test]
    fn startup_shutdown_notification_is_delivered_only_once() {
        let (sender, receiver) = bounded(1);
        let mut startup = Some(sender);
        notify_startup(&mut startup, Ok(()));
        notify_startup(&mut startup, Ok(()));
        assert!(receiver.recv().unwrap().is_ok());
        assert!(matches!(
            receiver.try_recv(),
            Err(TryRecvError::Disconnected)
        ));
    }
}
