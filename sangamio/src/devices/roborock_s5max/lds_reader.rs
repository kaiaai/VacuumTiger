//! Read-only LDS UART worker.

use super::lds::{LdsDecoder, LdsRevolution, LdsRevolutionAssembler};
use super::lds_motor::LDS_MOTOR_SET_CURRENT_SPEED;
use super::lifecycle::Lifecycle;
use super::tty::ExclusiveTty;
use crate::core::types::{SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use std::f32::consts::{PI, TAU};
use std::fs::File;
use std::os::fd::AsRawFd;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const LDS_BAUD: libc::speed_t = libc::B115200;
const LDS_BAUD_NUMBER: u32 = 115_200;
const ZERO_FEEDBACK_INTERVAL: Duration = Duration::from_millis(50);
// A cold rotor cannot report speed immediately. Preserve the stock abnormal-
// speed window before sending zero feedback to the kernel motor controller.
const COLD_START_FEEDBACK_GRACE: Duration = Duration::from_secs(3);
// Publish a usable scan through occasional UART loss while keeping absent
// samples explicit for downstream filtering.
const MIN_PUBLISHABLE_PACKETS: usize = 75;

const fn revolution_publishable(packet_count: usize) -> bool {
    packet_count >= MIN_PUBLISHABLE_PACKETS
}

fn zero_feedback_due(
    motor_running: bool,
    packet_seen: bool,
    motor_started: Option<Instant>,
    last_feedback: Instant,
    now: Instant,
) -> bool {
    motor_running
        && now.duration_since(last_feedback) >= ZERO_FEEDBACK_INTERVAL
        && (packet_seen
            || motor_started
                .is_some_and(|started| now.duration_since(started) >= COLD_START_FEEDBACK_GRACE))
}

pub(crate) fn speed_feedback_rpm_x100(speed_raw: u16) -> libc::c_int {
    ((u32::from(speed_raw) * 100 + 32) / 64) as libc::c_int
}

fn feed_speed(file: &File, speed: libc::c_int) -> Result<()> {
    let mut argument = speed;
    if unsafe { libc::ioctl(file.as_raw_fd(), LDS_MOTOR_SET_CURRENT_SPEED, &mut argument) } < 0 {
        return Err(std::io::Error::last_os_error().into());
    }
    Ok(())
}

pub(crate) struct LdsReader {
    handle: Option<JoinHandle<Result<()>>>,
}

impl LdsReader {
    pub(crate) fn start(
        path: &str,
        forward_angle_degrees: f32,
        sensor_data: Arc<Mutex<SensorGroupData>>,
        lifecycle: Lifecycle,
        shutdown: Arc<AtomicBool>,
        feedback_motor: File,
        motor_running: Arc<AtomicBool>,
    ) -> Result<Self> {
        let tty = ExclusiveTty::open(path, LDS_BAUD, LDS_BAUD_NUMBER)?;
        let path = path.to_string();
        let thread_lifecycle = lifecycle;
        let handle = thread::Builder::new()
            .name("s5max-lds".to_string())
            .spawn(move || {
                let result = reader_loop(
                    tty,
                    forward_angle_degrees,
                    sensor_data,
                    &shutdown,
                    feedback_motor,
                    motor_running,
                );
                if let Err(error) = &result {
                    log::error!("S5 Max LDS worker failed on {path}: {error}");
                    thread_lifecycle.fault(format!("LDS worker: {error}"));
                    shutdown.store(true, Ordering::Release);
                }
                result
            })
            .map_err(|error| Error::Other(format!("spawn S5 Max LDS worker: {error}")))?;
        Ok(Self {
            handle: Some(handle),
        })
    }

    pub(crate) fn join(&mut self) -> Result<()> {
        let Some(handle) = self.handle.take() else {
            return Ok(());
        };
        handle.join().map_err(|_| Error::ThreadPanic)?
    }
}

fn reader_loop(
    tty: ExclusiveTty,
    forward_angle_degrees: f32,
    sensor_data: Arc<Mutex<SensorGroupData>>,
    shutdown: &AtomicBool,
    feedback_motor: File,
    motor_running: Arc<AtomicBool>,
) -> Result<()> {
    let mut decoder = LdsDecoder::new();
    let mut assembler = LdsRevolutionAssembler::new();
    let forward_radians = forward_angle_degrees.to_radians();
    let mut buffer = [0u8; 1024];
    let mut last_feedback = Instant::now();
    let mut motor_started = None;
    let mut packet_seen = false;

    while !shutdown.load(Ordering::Acquire) {
        let now = Instant::now();
        let running = motor_running.load(Ordering::Acquire);
        match (running, motor_started) {
            (true, None) => motor_started = Some(now),
            (false, Some(_)) => {
                motor_started = None;
                packet_seen = false;
            }
            _ => {}
        }
        if !tty.poll_readable(Duration::from_millis(50))? {
            if zero_feedback_due(running, packet_seen, motor_started, last_feedback, now) {
                feed_speed(&feedback_motor, 0)?;
                last_feedback = Instant::now();
            }
            continue;
        }
        let count = tty.read_available(&mut buffer)?;
        for packet in decoder.push(&buffer[..count]) {
            packet_seen = true;
            if running {
                feed_speed(&feedback_motor, speed_feedback_rpm_x100(packet.speed_raw))?;
                last_feedback = Instant::now();
            }
            if let Some(revolution) = assembler.push(packet)
                && revolution_publishable(revolution.packet_count)
            {
                publish_revolution(&sensor_data, &revolution, forward_radians);
            }
        }
    }
    Ok(())
}

fn publish_revolution(
    sensor_data: &Arc<Mutex<SensorGroupData>>,
    revolution: &LdsRevolution,
    forward_radians: f32,
) {
    let points = revolution
        .samples
        .iter()
        .enumerate()
        .filter_map(|(raw_angle, sample)| {
            let sample = sample.as_ref()?;
            if sample.invalid() || sample.distance_mm() == 0 {
                return None;
            }
            let raw_radians = raw_angle as f32 * PI / 180.0;
            let body_bearing = (forward_radians - raw_radians).rem_euclid(TAU);
            Some((
                body_bearing,
                sample.distance_mm() as f32 / 1000.0,
                sample.signal_strength.min(u8::MAX as u16) as u8,
            ))
        })
        .collect::<Vec<_>>();

    let Ok(mut data) = sensor_data.lock() else {
        log::error!("S5 Max lidar sensor-group mutex poisoned");
        return;
    };
    data.set("scan", SensorValue::PointCloud2D(points));
    data.set(
        "rotation_speed_rpm",
        SensorValue::F32(revolution.mean_speed_rpm()),
    );
    data.set(
        "invalid_samples",
        SensorValue::U16(revolution.invalid_sample_count() as u16),
    );
    data.touch();
}

#[cfg(test)]
mod tests {
    use super::{
        COLD_START_FEEDBACK_GRACE, publish_revolution, revolution_publishable,
        speed_feedback_rpm_x100, zero_feedback_due,
    };
    use crate::core::types::{SensorGroupData, SensorValue};
    use crate::devices::roborock_s5max::lds::{
        INDEX_MIN, LdsDecoder, LdsRevolutionAssembler, PACKET_LENGTH,
    };
    use std::sync::{Arc, Mutex};
    use std::time::{Duration, Instant};

    #[test]
    fn applies_the_measured_clockwise_to_body_transform() {
        // Use the parser's captured packet fixture and close its partial scan
        // with a wrapped index. This test checks transformation independently
        // of UART ownership.
        let raw: [u8; PACKET_LENGTH] = [
            0xfa, 0xbb, 0xc2, 0x4a, 0x7f, 0x0f, 0x78, 0x00, 0x64, 0x0f, 0x8e, 0x01, 0x27, 0x0f,
            0x6d, 0x02, 0xf4, 0x0e, 0xf7, 0x02, 0x45, 0x40,
        ];
        let mut decoder = LdsDecoder::new();
        let packet = decoder.push(&raw).pop().unwrap();
        let mut assembler = LdsRevolutionAssembler::new();
        assert!(assembler.push(packet).is_none());

        // `finish` returns the partial revolution; publication itself accepts
        // partial fixtures although the production loop publishes only complete scans.
        let revolution = assembler.finish().unwrap();
        let data = Arc::new(Mutex::new(SensorGroupData::new("lidar")));
        publish_revolution(&data, &revolution, 261.2_f32.to_radians());
        let data = data.lock().unwrap();
        let SensorValue::PointCloud2D(points) = data.values.get("scan").unwrap() else {
            panic!("scan has unexpected type");
        };
        let expected = (261.2_f32 - (packet.index - INDEX_MIN) as f32 * 4.0).to_radians();
        assert!((points[0].0 - expected).abs() < 1e-5);
        assert!((points[0].1 - 3.967).abs() < 1e-6);
    }

    #[test]
    fn converts_packet_q10_6_speed_to_kernel_feedback() {
        assert_eq!(speed_feedback_rpm_x100(0), 0);
        assert_eq!(speed_feedback_rpm_x100(300 * 64), 30_000);
        assert_eq!(speed_feedback_rpm_x100(0x4ac2), 29_903);
    }

    #[test]
    fn accepts_only_sufficiently_complete_revolutions() {
        assert!(!revolution_publishable(74));
        assert!(revolution_publishable(75));
        assert!(revolution_publishable(90));
    }

    #[test]
    fn cold_rotor_gets_a_bounded_feedback_grace_period() {
        let now = Instant::now();
        let last_feedback = now - Duration::from_millis(100);
        assert!(!zero_feedback_due(
            true,
            false,
            Some(now),
            last_feedback,
            now
        ));
        assert!(zero_feedback_due(true, true, Some(now), last_feedback, now));
        let after_grace = now + COLD_START_FEEDBACK_GRACE;
        assert!(zero_feedback_due(
            true,
            false,
            Some(now),
            last_feedback,
            after_grace
        ));
        assert!(!zero_feedback_due(
            false,
            true,
            Some(now),
            last_feedback,
            after_grace
        ));
    }
}
