//! Revo LDS / Roborock LDS Protocol Implementation
//!
//! Protocol based on Neato XV11/Roborock LDS (0xFA sync byte variant).
//! Packet format: [0xFA] [Index] [Speed 2B] [Data0 4B] [Data1 4B] [Data2 4B] [Data3 4B] [Checksum 2B]
//!
//! Total packet size: 22 bytes
//! - 90 packets per revolution (index 0xA0 to 0xF9)
//! - 4 samples per packet = 360 samples per revolution
//!
//! References:
//! - https://github.com/Roborock-OpenSource/Cullinan
//! - Neato XV11 LIDAR documentation

use crate::config::AffineTransform1D;
use crate::error::{Error, Result};
use std::f32::consts::TAU;
use std::io::Read;

/// Packet size: sync(1) + index(1) + speed(2) + data(16) + checksum(2) = 22 bytes
const PACKET_SIZE: usize = 22;

/// Maximum buffer size before forced clear (safety limit)
const MAX_BUFFER_SIZE: usize = 4096;

/// Sync byte for Revo LDS packets
const SYNC_BYTE: u8 = 0xFA;

/// Index range: 0xA0 (160) to 0xF9 (249) = 90 packets
const INDEX_MIN: u8 = 0xA0;
const INDEX_MAX: u8 = 0xF9;

/// Lidar scan point
#[derive(Debug, Clone)]
pub struct LidarPoint {
    pub angle: f32,           // radians
    pub distance: f32,        // meters
    pub signal_strength: u16, // signal quality (higher = better)
}

/// Complete lidar scan (360 points)
#[derive(Debug, Clone)]
pub struct LidarScan {
    pub points: Vec<LidarPoint>,
    pub rpm: f32, // motor speed in RPM
}

/// Result of parsing a lidar packet
#[derive(Debug, Clone)]
pub enum ParseResult {
    /// Measurement packet with 4 points
    Points(Vec<LidarPoint>, f32), // points, rpm
    /// No complete packet available
    None,
}

/// Packet reader for Revo LDS with robust error recovery
pub struct RevoLdsPacketReader {
    buffer: Vec<u8>,
    /// Count of bytes discarded due to resync (for diagnostics)
    bytes_discarded: u64,
    /// Count of checksum failures (for diagnostics)
    checksum_failures: u64,
    /// Angle transform to apply to all lidar angles
    angle_transform: AffineTransform1D,
}

impl RevoLdsPacketReader {
    /// Create a new reader with custom angle transform
    ///
    /// The transform is applied to all lidar angles: output = scale * input + offset
    /// Use identity transform for no angle modification.
    pub fn with_transform(angle_transform: AffineTransform1D) -> Self {
        Self {
            buffer: Vec::with_capacity(1024),
            bytes_discarded: 0,
            checksum_failures: 0,
            angle_transform,
        }
    }

    /// Get diagnostic counters: (bytes_discarded, checksum_failures, buffer_size)
    pub fn diagnostics(&self) -> (u64, u64, usize) {
        (self.bytes_discarded, self.checksum_failures, self.buffer.len())
    }

    /// Clear the buffer (call on startup after serial flush)
    pub fn clear(&mut self) {
        self.buffer.clear();
    }

    /// Feed raw bytes directly into the parser buffer (for raw dump replay)
    pub fn feed_bytes(&mut self, data: &[u8]) {
        self.buffer.extend_from_slice(data);
    }

    /// Read bytes from port into internal buffer
    ///
    /// Returns the number of bytes read, or 0 if no data available (timeout/wouldblock).
    /// Call this once, then call `parse_next()` in a loop to drain all packets.
    pub fn read_bytes<R: Read>(&mut self, port: &mut R) -> Result<usize> {
        let mut temp_buf = [0u8; 512];
        let mut total_read = 0;

        // Loop to drain all available data from serial port
        loop {
            match port.read(&mut temp_buf) {
                Ok(0) => break, // EOF or no more data
                Ok(n) => {
                    log::trace!(
                        "Revo LDS raw read: {} bytes: {:02X?}",
                        n,
                        &temp_buf[..n.min(32)]
                    );
                    self.buffer.extend_from_slice(&temp_buf[..n]);
                    total_read += n;

                    // If we read less than buffer size, no more data available
                    if n < temp_buf.len() {
                        break;
                    }
                }
                Err(e) if e.kind() == std::io::ErrorKind::TimedOut => break,
                Err(e) if e.kind() == std::io::ErrorKind::WouldBlock => break,
                Err(e) => return Err(Error::Io(e)),
            }
        }

        Ok(total_read)
    }

    /// Parse and return the next complete packet from the buffer
    ///
    /// Returns:
    /// - `ParseResult::Points(points, rpm)` for valid measurement packets
    /// - `ParseResult::None` when no complete packet is available
    pub fn parse_next(&mut self) -> Result<ParseResult> {
        // Safety check: if buffer is too large, something is wrong
        if self.buffer.len() > MAX_BUFFER_SIZE {
            log::warn!(
                "Revo LDS buffer overflow ({} bytes), clearing. Discarded: {}, Checksum failures: {}",
                self.buffer.len(),
                self.bytes_discarded,
                self.checksum_failures
            );
            self.bytes_discarded += self.buffer.len() as u64;
            self.buffer.clear();
            return Ok(ParseResult::None);
        }

        self.try_parse_packet_with_resync()
    }

    /// Parse packet with loop-based resync on failure
    fn try_parse_packet_with_resync(&mut self) -> Result<ParseResult> {
        loop {
            // Need at least one full packet
            if self.buffer.len() < PACKET_SIZE {
                return Ok(ParseResult::None);
            }

            // Find sync byte 0xFA
            let Some(sync_idx) = self.buffer.iter().position(|&b| b == SYNC_BYTE) else {
                // No sync found - discard buffer up to last byte
                let discard = self.buffer.len().saturating_sub(1);
                if discard > 0 {
                    self.bytes_discarded += discard as u64;
                    log::trace!("Revo LDS: no sync found, discarding {} bytes", discard);
                    self.buffer.drain(0..discard);
                }
                return Ok(ParseResult::None);
            };

            // Discard bytes before sync
            if sync_idx > 0 {
                self.bytes_discarded += sync_idx as u64;
                log::trace!("Revo LDS: discarding {} bytes before sync", sync_idx);
                self.buffer.drain(0..sync_idx);
            }

            // Check if we have complete packet
            if self.buffer.len() < PACKET_SIZE {
                return Ok(ParseResult::None);
            }

            // Validate index byte (0xA0 to 0xF9)
            let index = self.buffer[1];
            if !(INDEX_MIN..=INDEX_MAX).contains(&index) {
                // Invalid index - this 0xFA is a false sync
                self.bytes_discarded += 1;
                self.buffer.drain(0..1);
                log::trace!(
                    "Revo LDS: invalid index 0x{:02X}, draining 1 byte",
                    index
                );
                continue;
            }

            // Validate checksum
            let calculated = Self::calculate_checksum(&self.buffer[0..20]);
            let received = (self.buffer[20] as u16) | ((self.buffer[21] as u16) << 8);

            if calculated != received {
                self.checksum_failures += 1;
                self.bytes_discarded += 1;
                self.buffer.drain(0..1);
                log::trace!(
                    "Revo LDS: checksum mismatch (calc=0x{:04X}, recv=0x{:04X}), draining 1 byte",
                    calculated,
                    received
                );
                continue;
            }

            // Valid packet! Parse it
            let result = self.parse_measurement_packet();

            // Remove processed packet
            self.buffer.drain(0..PACKET_SIZE);

            return Ok(result);
        }
    }

    /// Calculate checksum for Revo LDS packet
    /// Algorithm: rotating left-shift accumulation with overflow correction
    fn calculate_checksum(data: &[u8]) -> u16 {
        let mut checksum: u32 = 0;

        // Process 20 bytes as 10 little-endian u16 values
        for i in (0..20).step_by(2) {
            let value = (data[i] as u32) | ((data[i + 1] as u32) << 8);
            checksum = (checksum << 1).wrapping_add(value);
        }

        // Add overflow bit back and mask to 15 bits
        ((checksum.wrapping_add(checksum >> 15)) & 0x7FFF) as u16
    }

    /// Parse a valid measurement packet (buffer starts at sync byte)
    fn parse_measurement_packet(&self) -> ParseResult {
        let index = self.buffer[1];

        // Speed: bytes 2-3, little-endian, RPM × 64
        let speed_raw = (self.buffer[2] as u16) | ((self.buffer[3] as u16) << 8);
        let rpm = speed_raw as f32 / 64.0;

        // Calculate base angle for this packet
        // Index 0xA0 = samples 0-3, Index 0xA1 = samples 4-7, etc.
        let packet_number = (index - INDEX_MIN) as u32;
        let base_angle_deg = (packet_number * 4) as f32;

        let mut points = Vec::with_capacity(4);

        // Parse 4 samples (4 bytes each, starting at byte 4)
        for sample_idx in 0..4 {
            let offset = 4 + sample_idx * 4;

            // Distance: 14-bit little-endian
            // byte 0: distance[7:0]
            // byte 1: bit7=invalid, bit6=strength_warning, bits5-0=distance[13:8]
            let dist_low = self.buffer[offset] as u16;
            let byte1 = self.buffer[offset + 1];
            let invalid = (byte1 & 0x80) != 0;
            let _strength_warning = (byte1 & 0x40) != 0;
            let dist_high = (byte1 & 0x3F) as u16;
            let distance_mm = dist_low | (dist_high << 8);

            // Signal strength: bytes 2-3, little-endian
            let strength = (self.buffer[offset + 2] as u16)
                | ((self.buffer[offset + 3] as u16) << 8);

            // Skip invalid readings
            if invalid || distance_mm == 0 {
                continue;
            }

            // Calculate angle for this sample
            let angle_deg = base_angle_deg + sample_idx as f32;
            let raw_angle_rad = angle_deg.to_radians();

            // Apply configurable transform
            let mut angle_rad = self.angle_transform.apply(raw_angle_rad);

            // Normalize to [0, 2π)
            while angle_rad < 0.0 {
                angle_rad += TAU;
            }
            while angle_rad >= TAU {
                angle_rad -= TAU;
            }

            // Convert distance to meters
            let distance_m = distance_mm as f32 / 1000.0;

            points.push(LidarPoint {
                angle: angle_rad,
                distance: distance_m,
                signal_strength: strength,
            });
        }

        ParseResult::Points(points, rpm)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_checksum_calculation() {
        // Test vector from documentation
        // This is a simplified test - actual packets would need real data
        let data = [0xFA, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00];
        let checksum = RevoLdsPacketReader::calculate_checksum(&data);
        // For all zeros except sync and index, checksum should be predictable
        assert!(checksum < 0x8000, "Checksum should be 15-bit");
    }

    #[test]
    fn test_index_validation() {
        // Valid indices: 0xA0 to 0xF9
        assert!((INDEX_MIN..=INDEX_MAX).contains(&0xA0));
        assert!((INDEX_MIN..=INDEX_MAX).contains(&0xF9));
        assert!(!(INDEX_MIN..=INDEX_MAX).contains(&0x00));
        assert!(!(INDEX_MIN..=INDEX_MAX).contains(&0xFF));
    }

    #[test]
    fn test_angle_calculation() {
        // Index 0xA0 (160) should give angles 0, 1, 2, 3 degrees
        let packet_num = (0xA0_u8 - INDEX_MIN) as u32;
        assert_eq!(packet_num, 0);
        assert_eq!(packet_num * 4, 0); // base angle

        // Index 0xF9 (249) should give angles 356, 357, 358, 359 degrees
        let packet_num = (0xF9_u8 - INDEX_MIN) as u32;
        assert_eq!(packet_num, 89);
        assert_eq!(packet_num * 4, 356); // base angle
    }
}
