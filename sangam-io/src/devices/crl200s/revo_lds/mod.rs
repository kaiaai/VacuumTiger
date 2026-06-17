//! Revo LDS / Roborock LDS Lidar driver
//!
//! # Protocol
//!
//! This driver supports the Revo LDS (LDS08RR) and compatible LiDARs that use
//! the 0xFA sync byte protocol (same as Neato XV11, Roborock LDS).
//!
//! Packet format: 22 bytes total
//! - Sync: 0xFA
//! - Index: 0xA0 to 0xF9 (90 packets per revolution)
//! - Speed: 2 bytes little-endian (RPM × 64)
//! - Data: 4 samples × 4 bytes each
//! - Checksum: 2 bytes
//!
//! # Coordinate Frame (Output)
//!
//! After applying the configurable `AffineTransform1D` from `sangamio.toml`,
//! output data follows **ROS REP-103** convention:
//!
//! - **0° = forward**: Robot's front direction (+X axis)
//! - **Rotation**: Counter-clockwise (CCW) positive when viewed from above
//! - **90° = left**: Robot's left side (+Y axis)
//! - **Angle range**: 0.0 to 2π radians
//! - **Distance units**: Meters (m)
//!
//! # Scan Accumulation
//!
//! The driver accumulates packets until a complete 360° scan is collected,
//! then publishes the full scan. Scan completion is detected when the index
//! wraps from 0xF9 back to 0xA0.

pub mod protocol;

use crate::config::{AffineTransform1D, LidarMountingConfig};
use crate::core::types::{SensorGroupData, SensorValue};
use crate::error::{Error, Result};
use protocol::{RevoLdsPacketReader, ParseResult};
use serialport::SerialPort;
use std::io::Write;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const LIDAR_BAUD_RATE: u32 = 115200;
const LIDAR_READ_TIMEOUT_MS: u64 = 100;

/// Minimum points required before accepting a scan as complete
const MIN_SCAN_POINTS: usize = 50;

/// Maximum time to wait for a complete scan before publishing partial results
const SCAN_TIMEOUT_SECS: f32 = 2.0;

/// Expected number of packets per revolution
const PACKETS_PER_REVOLUTION: usize = 90;

/// Revo LDS lidar driver
pub struct RevoLdsDriver {
    port_path: String,
    shutdown: Arc<AtomicBool>,
    reader_handle: Option<JoinHandle<()>>,
    packet_count: Arc<AtomicU64>,
    error_count: Arc<AtomicU64>,
    /// Angle transform for coordinate frame conversion
    angle_transform: AffineTransform1D,
    /// Lidar mounting configuration for robot-center transformation
    lidar_mounting: LidarMountingConfig,
}

impl RevoLdsDriver {
    /// Create a new Revo LDS driver
    ///
    /// # Arguments
    /// - `port_path`: Serial port path (e.g., "/dev/ttyS1")
    /// - `angle_transform`: Transform applied to all lidar angles
    /// - `lidar_mounting`: Mounting configuration for robot-center transformation
    pub fn new(
        port_path: &str,
        angle_transform: AffineTransform1D,
        lidar_mounting: LidarMountingConfig,
    ) -> Self {
        Self {
            port_path: port_path.to_string(),
            shutdown: Arc::new(AtomicBool::new(false)),
            reader_handle: None,
            packet_count: Arc::new(AtomicU64::new(0)),
            error_count: Arc::new(AtomicU64::new(0)),
            angle_transform,
            lidar_mounting,
        }
    }

    /// Start the lidar reader thread
    pub fn start(&mut self, sensor_data: Arc<Mutex<SensorGroupData>>) -> Result<()> {
        // Open serial port
        let port = serialport::new(&self.port_path, LIDAR_BAUD_RATE)
            .timeout(Duration::from_millis(LIDAR_READ_TIMEOUT_MS))
            .open()
            .map_err(Error::Serial)?;

        // Flush serial buffer to clear stale data
        if let Err(e) = port.clear(serialport::ClearBuffer::Input) {
            log::warn!("Failed to clear lidar serial input buffer: {}", e);
        } else {
            log::debug!("Cleared lidar serial input buffer");
        }

        let shutdown = Arc::clone(&self.shutdown);
        let packet_count = Arc::clone(&self.packet_count);
        let error_count = Arc::clone(&self.error_count);
        let angle_transform = self.angle_transform;
        let lidar_mounting = self.lidar_mounting.clone();

        self.reader_handle = Some(
            thread::Builder::new()
                .name("revo-lds-reader".to_string())
                .spawn(move || {
                    Self::reader_loop(
                        port,
                        shutdown,
                        sensor_data,
                        packet_count,
                        error_count,
                        angle_transform,
                        lidar_mounting,
                    );
                })
                .map_err(|e| Error::Other(format!("Failed to spawn lidar thread: {}", e)))?,
        );

        log::info!("Revo LDS lidar driver started on {}", self.port_path);
        Ok(())
    }

    /// Reader loop - reads packets and accumulates full scans
    fn reader_loop(
        mut port: Box<dyn SerialPort>,
        shutdown: Arc<AtomicBool>,
        sensor_data: Arc<Mutex<SensorGroupData>>,
        packet_count: Arc<AtomicU64>,
        error_count: Arc<AtomicU64>,
        angle_transform: AffineTransform1D,
        lidar_mounting: LidarMountingConfig,
    ) {
        let mut reader = RevoLdsPacketReader::with_transform(angle_transform);
        let mut accumulated_points: Vec<(f32, f32, u8)> = Vec::with_capacity(400);
        let mut last_index: Option<u8> = None;
        let mut last_scan_time = Instant::now();
        let mut last_diagnostic_time = Instant::now();
        let mut packets_in_scan = 0;
        let mut last_rpm: f32 = 0.0;

        // Raw dump: capture first 64KB of serial data for protocol analysis
        const RAW_DUMP_SIZE: usize = 65536;
        let mut raw_dump_buf: Vec<u8> = Vec::with_capacity(RAW_DUMP_SIZE);
        let mut raw_dump_done = false;

        // Clear reader buffer on startup
        reader.clear();

        while !shutdown.load(Ordering::Relaxed) {
            // Read raw bytes directly from serial port for dump
            if !raw_dump_done {
                let mut temp = [0u8; 512];
                match std::io::Read::read(&mut port, &mut temp) {
                    Ok(0) => {}
                    Ok(n) => {
                        raw_dump_buf.extend_from_slice(&temp[..n]);
                        log::info!("Revo LDS raw dump: {} / {} bytes", raw_dump_buf.len(), RAW_DUMP_SIZE);
                        if raw_dump_buf.len() >= RAW_DUMP_SIZE {
                            raw_dump_done = true;
                            match std::fs::File::create("/tmp/lidar_raw.bin") {
                                Ok(mut f) => {
                                    let _ = f.write_all(&raw_dump_buf);
                                    log::info!("Revo LDS raw dump saved to /tmp/lidar_raw.bin ({} bytes)", raw_dump_buf.len());
                                }
                                Err(e) => log::error!("Failed to write raw dump: {}", e),
                            }
                            // Feed captured data into the parser buffer
                            reader.feed_bytes(&raw_dump_buf);
                            raw_dump_buf = Vec::new(); // free memory
                        }
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::TimedOut => {}
                    Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => {}
                    Err(e) => log::error!("Raw dump read error: {}", e),
                }
                if !raw_dump_done {
                    thread::sleep(Duration::from_millis(1));
                    continue;
                }
            }

            // Step 1: Read bytes from serial port into buffer
            match reader.read_bytes(&mut port) {
                Ok(_bytes_read) => {
                    // Step 2: Drain ALL complete packets from buffer
                    loop {
                        match reader.parse_next() {
                            Ok(ParseResult::Points(points, rpm)) => {
                                let count = packet_count.fetch_add(1, Ordering::Relaxed) + 1;
                                last_rpm = rpm;
                                packets_in_scan += 1;

                                // Get current packet index from angle
                                // First point's angle tells us which packet this is
                                let current_index = if let Some(first_point) = points.first() {
                                    // Convert angle back to index
                                    let angle_deg = first_point.angle.to_degrees();
                                    let packet_num = (angle_deg / 4.0) as u8;
                                    0xA0 + packet_num
                                } else {
                                    continue;
                                };

                                // Detect scan completion: index wrapped from high to low
                                // (e.g., 0xF9 → 0xA0)
                                let scan_complete = if let Some(prev_index) = last_index {
                                    current_index < prev_index && packets_in_scan >= PACKETS_PER_REVOLUTION - 5
                                } else {
                                    false
                                };

                                if scan_complete {
                                    // Publish complete scan
                                    if accumulated_points.len() >= MIN_SCAN_POINTS {
                                        Self::publish_scan(&sensor_data, &accumulated_points, last_rpm);
                                    }
                                    accumulated_points.clear();
                                    packets_in_scan = 0;
                                    last_scan_time = Instant::now();
                                }

                                // Add points to accumulator
                                for point in &points {
                                    // Transform point from lidar frame to robot center frame
                                    let (new_angle, new_distance) = lidar_mounting
                                        .transform_to_robot_center(point.angle, point.distance);

                                    // Convert signal strength to quality (0-255)
                                    let quality = (point.signal_strength.min(255) as u8).max(1);

                                    accumulated_points.push((new_angle, new_distance, quality));
                                }

                                last_index = Some(current_index);

                                // Log statistics periodically
                                if count.is_multiple_of(100) {
                                    log::debug!(
                                        "Revo LDS: {} packets, {} points accumulated, {:.1} RPM",
                                        count,
                                        accumulated_points.len(),
                                        last_rpm
                                    );
                                }

                                // Log diagnostics every 60 seconds
                                if last_diagnostic_time.elapsed().as_secs() >= 60 {
                                    let errors = error_count.load(Ordering::Relaxed);
                                    let (bytes_discarded, checksum_failures, buffer_size) =
                                        reader.diagnostics();
                                    log::info!(
                                        "Revo LDS stats: {} packets, {} errors, {} bytes discarded, {} checksum failures, {} bytes buffered, {:.1} RPM",
                                        count,
                                        errors,
                                        bytes_discarded,
                                        checksum_failures,
                                        buffer_size,
                                        last_rpm
                                    );
                                    last_diagnostic_time = Instant::now();
                                }
                            }
                            Ok(ParseResult::None) => {
                                // No more complete packets in buffer
                                break;
                            }
                            Err(e) => {
                                error_count.fetch_add(1, Ordering::Relaxed);
                                log::error!("Revo LDS parse error: {}", e);
                                break;
                            }
                        }
                    }

                    // Check for scan timeout
                    let elapsed = last_scan_time.elapsed().as_secs_f32();
                    if elapsed > SCAN_TIMEOUT_SECS && accumulated_points.len() > MIN_SCAN_POINTS {
                        log::warn!(
                            "Revo LDS scan timeout ({:.1}s), publishing {} points",
                            elapsed,
                            accumulated_points.len()
                        );
                        Self::publish_scan(&sensor_data, &accumulated_points, last_rpm);
                        accumulated_points.clear();
                        packets_in_scan = 0;
                        last_scan_time = Instant::now();
                    }

                    // Small sleep to avoid busy-waiting
                    thread::sleep(Duration::from_millis(1));
                }
                Err(e) => {
                    error_count.fetch_add(1, Ordering::Relaxed);
                    log::error!("Revo LDS read error: {}", e);
                    thread::sleep(Duration::from_millis(10));
                }
            }
        }

        log::info!("Revo LDS reader thread exiting");
    }

    /// Publish accumulated scan to sensor data
    fn publish_scan(
        sensor_data: &Arc<Mutex<SensorGroupData>>,
        points: &[(f32, f32, u8)],
        rpm: f32,
    ) {
        let Ok(mut data) = sensor_data.lock() else {
            log::error!("Failed to lock sensor data for lidar scan");
            return;
        };
        data.touch();
        data.set("scan", SensorValue::PointCloud2D(points.to_vec()));
        data.set("rpm", SensorValue::F32(rpm));

        log::trace!("Published Revo LDS scan with {} points at {:.1} RPM", points.len(), rpm);
    }

    /// Shutdown the driver
    pub fn shutdown(&mut self) -> Result<()> {
        log::info!("Shutting down Revo LDS driver...");
        self.shutdown.store(true, Ordering::Relaxed);

        if let Some(handle) = self.reader_handle.take() {
            handle.join().map_err(|_| Error::ThreadPanic)?;
        }

        log::info!("Revo LDS driver shutdown complete");
        Ok(())
    }
}

impl Drop for RevoLdsDriver {
    fn drop(&mut self) {
        let _ = self.shutdown();
    }
}
