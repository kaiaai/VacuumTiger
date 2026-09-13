//! Passive Roborock LDS packet framing.
//!
//! Captured S5 Max traffic uses the 22-byte XV11 packet layout:
//! `fa index speed_le sample[4] checksum_le`.
//! Each packet contains four consecutive one-degree samples. Index `0xa0`
//! starts at 0 degrees and index `0xf9` ends at 359 degrees.
//!
//! Synchronized scan/IMU registration shows that raw angles increase clockwise
//! when viewed from above. Fixed-wall synchronization measured robot-forward
//! at raw 261.2 degrees. The reader keeps this mounting angle configurable and
//! transforms samples with `body = wrap(forward_raw - raw_angle)`.

pub const SYNC: u8 = 0xfa;
pub const PACKET_LENGTH: usize = 22;
pub const INDEX_MIN: u8 = 0xa0;
pub const INDEX_MAX: u8 = 0xf9;
pub const PACKETS_PER_REVOLUTION: usize = 90;
pub const SAMPLES_PER_REVOLUTION: usize = 360;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LdsSample {
    pub distance_raw: u16,
    pub signal_strength: u16,
}

impl LdsSample {
    /// Bit 15 marks a sample for which no valid distance was returned.
    pub const fn invalid(self) -> bool {
        self.distance_raw & 0x8000 != 0
    }

    /// Bit 14 marks a sample whose signal strength is below the sensor limit.
    pub const fn strength_warning(self) -> bool {
        self.distance_raw & 0x4000 != 0
    }

    /// Valid distance bits, in millimetres.
    pub const fn distance_mm(self) -> u16 {
        self.distance_raw & 0x3fff
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LdsPacket {
    pub index: u8,
    pub speed_raw: u16,
    pub samples: [LdsSample; 4],
    pub checksum: u16,
}

impl LdsPacket {
    pub fn parse(bytes: &[u8; PACKET_LENGTH]) -> Option<Self> {
        if bytes[0] != SYNC || !(INDEX_MIN..=INDEX_MAX).contains(&bytes[1]) {
            return None;
        }
        let sample = |offset| LdsSample {
            distance_raw: u16::from_le_bytes([bytes[offset], bytes[offset + 1]]),
            signal_strength: u16::from_le_bytes([bytes[offset + 2], bytes[offset + 3]]),
        };
        Some(Self {
            index: bytes[1],
            speed_raw: u16::from_le_bytes([bytes[2], bytes[3]]),
            samples: [sample(4), sample(8), sample(12), sample(16)],
            checksum: u16::from_le_bytes([bytes[20], bytes[21]]),
        })
    }

    pub const fn start_angle_degrees(self) -> u16 {
        (self.index - INDEX_MIN) as u16 * 4
    }

    pub fn speed_rpm(self) -> f32 {
        self.speed_raw as f32 / 64.0
    }

    pub fn checksum_valid(self, bytes: &[u8; PACKET_LENGTH]) -> bool {
        self.checksum == packet_checksum(bytes)
    }
}

/// XV11's 15-bit rolling checksum over the first ten little-endian words.
pub fn packet_checksum(bytes: &[u8; PACKET_LENGTH]) -> u16 {
    let mut accumulator = 0_u32;
    for word in bytes[..20].chunks(2) {
        accumulator = (accumulator << 1) + u16::from_le_bytes([word[0], word[1]]) as u32;
    }
    (((accumulator & 0x7fff) + (accumulator >> 15)) & 0x7fff) as u16
}

#[derive(Debug, Default)]
pub struct LdsDecoder {
    buffer: Vec<u8>,
    discarded: u64,
}

impl LdsDecoder {
    pub fn new() -> Self {
        Self {
            buffer: Vec::with_capacity(PACKET_LENGTH * 4),
            discarded: 0,
        }
    }

    pub const fn discarded(&self) -> u64 {
        self.discarded
    }

    pub fn push(&mut self, bytes: &[u8]) -> Vec<LdsPacket> {
        self.buffer.extend_from_slice(bytes);
        let mut packets = Vec::new();
        loop {
            let Some(start) = self.buffer.iter().position(|byte| *byte == SYNC) else {
                self.discarded += self.buffer.len() as u64;
                self.buffer.clear();
                break;
            };
            if start != 0 {
                self.discarded += start as u64;
                self.buffer.drain(..start);
            }
            if self.buffer.len() < PACKET_LENGTH {
                break;
            }
            let raw: [u8; PACKET_LENGTH] = self.buffer[..PACKET_LENGTH].try_into().unwrap();
            if let Some(packet) =
                LdsPacket::parse(&raw).filter(|packet| packet.checksum_valid(&raw))
            {
                packets.push(packet);
                self.buffer.drain(..PACKET_LENGTH);
            } else {
                self.discarded += 1;
                self.buffer.drain(..1);
            }
        }
        packets
    }
}

/// Passive reconstruction of one LDS revolution for capture diagnostics.
///
/// This intentionally has no actuator or UART-write functionality. A scan can
/// be partial when observation starts in the middle of a revolution or bytes
/// were lost before reaching this decoder.
#[derive(Debug, Clone)]
pub struct LdsRevolution {
    pub samples: [Option<LdsSample>; SAMPLES_PER_REVOLUTION],
    pub packet_count: usize,
    pub duplicate_packets: usize,
    pub min_speed_raw: u16,
    pub max_speed_raw: u16,
    speed_raw_sum: u64,
}

impl LdsRevolution {
    pub const fn missing_packets(&self) -> usize {
        PACKETS_PER_REVOLUTION - self.packet_count
    }

    pub const fn complete(&self) -> bool {
        self.packet_count == PACKETS_PER_REVOLUTION
    }

    pub fn mean_speed_rpm(&self) -> f32 {
        if self.packet_count == 0 {
            return 0.0;
        }
        self.speed_raw_sum as f32 / self.packet_count as f32 / 64.0
    }

    pub fn min_speed_rpm(&self) -> f32 {
        self.min_speed_raw as f32 / 64.0
    }

    pub fn max_speed_rpm(&self) -> f32 {
        self.max_speed_raw as f32 / 64.0
    }

    pub fn valid_sample_count(&self) -> usize {
        self.samples
            .iter()
            .flatten()
            .filter(|sample| !sample.invalid())
            .count()
    }

    pub fn invalid_sample_count(&self) -> usize {
        self.samples
            .iter()
            .flatten()
            .filter(|sample| sample.invalid())
            .count()
    }

    pub fn strength_warning_count(&self) -> usize {
        self.samples
            .iter()
            .flatten()
            .filter(|sample| sample.strength_warning())
            .count()
    }

    pub fn observed_sample_count(&self) -> usize {
        self.samples.iter().flatten().count()
    }
}

#[derive(Debug)]
struct RevolutionBuilder {
    revolution: LdsRevolution,
    seen_packets: [bool; PACKETS_PER_REVOLUTION],
    last_index: Option<u8>,
}

impl RevolutionBuilder {
    fn new() -> Self {
        Self {
            revolution: LdsRevolution {
                samples: [None; SAMPLES_PER_REVOLUTION],
                packet_count: 0,
                duplicate_packets: 0,
                min_speed_raw: u16::MAX,
                max_speed_raw: 0,
                speed_raw_sum: 0,
            },
            seen_packets: [false; PACKETS_PER_REVOLUTION],
            last_index: None,
        }
    }

    fn insert(&mut self, packet: LdsPacket) {
        let packet_index = (packet.index - INDEX_MIN) as usize;
        if self.seen_packets[packet_index] {
            self.revolution.duplicate_packets += 1;
            self.last_index = Some(packet.index);
            return;
        }
        self.seen_packets[packet_index] = true;
        self.revolution.packet_count += 1;
        self.revolution.min_speed_raw = self.revolution.min_speed_raw.min(packet.speed_raw);
        self.revolution.max_speed_raw = self.revolution.max_speed_raw.max(packet.speed_raw);
        self.revolution.speed_raw_sum += u64::from(packet.speed_raw);
        let first_angle = packet.start_angle_degrees() as usize;
        for (offset, sample) in packet.samples.into_iter().enumerate() {
            self.revolution.samples[first_angle + offset] = Some(sample);
        }
        self.last_index = Some(packet.index);
    }

    fn finish(mut self) -> LdsRevolution {
        if self.revolution.packet_count == 0 {
            self.revolution.min_speed_raw = 0;
        }
        self.revolution
    }
}

/// Groups ordered, checksum-valid LDS packets at each index wrap.
#[derive(Debug, Default)]
pub struct LdsRevolutionAssembler {
    current: Option<RevolutionBuilder>,
}

impl LdsRevolutionAssembler {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn push(&mut self, packet: LdsPacket) -> Option<LdsRevolution> {
        let wrapped = self
            .current
            .as_ref()
            .and_then(|current| current.last_index)
            .is_some_and(|last_index| packet.index < last_index);
        let completed = wrapped.then(|| {
            self.current
                .take()
                .expect("wrap requires a current scan")
                .finish()
        });
        self.current
            .get_or_insert_with(RevolutionBuilder::new)
            .insert(packet);
        completed
    }

    pub fn finish(mut self) -> Option<LdsRevolution> {
        self.current.take().map(RevolutionBuilder::finish)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_captured_packet_shape() {
        let raw = [
            0xfa, 0xbb, 0xc2, 0x4a, 0x7f, 0x0f, 0x78, 0x00, 0x64, 0x0f, 0x8e, 0x01, 0x27, 0x0f,
            0x6d, 0x02, 0xf4, 0x0e, 0xf7, 0x02, 0x45, 0x40,
        ];
        let packet = LdsPacket::parse(&raw).unwrap();
        assert_eq!(packet.index, 0xbb);
        assert_eq!(packet.start_angle_degrees(), 108);
        assert_eq!(packet.speed_raw, 0x4ac2);
        assert_eq!(packet.speed_rpm(), 299.03125);
        assert_eq!(packet.samples[0].distance_raw, 0x0f7f);
        assert_eq!(packet.samples[0].distance_mm(), 0x0f7f);
        assert_eq!(packet.samples[0].signal_strength, 0x0078);
        assert_eq!(packet.checksum, 0x4045);
        assert!(packet.checksum_valid(&raw));
    }

    #[test]
    fn identifies_no_return_sample() {
        let raw = [
            0xfa, 0xb9, 0xc2, 0x4a, 0x20, 0x80, 0, 0, 0x20, 0x80, 0, 0, 0x20, 0x80, 0, 0, 0x20,
            0x80, 0, 0, 0x67, 0x4f,
        ];
        let packet = LdsPacket::parse(&raw).unwrap();
        assert!(packet.samples.iter().all(|sample| sample.invalid()));
        assert_eq!(packet.samples[0].distance_mm(), 0x20);
        assert!(packet.checksum_valid(&raw));
    }

    #[test]
    fn resynchronizes_across_chunks() {
        let raw = [
            0xfa, 0xbb, 0xc2, 0x4a, 0x7f, 0x0f, 0x78, 0x00, 0x64, 0x0f, 0x8e, 0x01, 0x27, 0x0f,
            0x6d, 0x02, 0xf4, 0x0e, 0xf7, 0x02, 0x45, 0x40,
        ];
        let mut decoder = LdsDecoder::new();
        assert!(decoder.push(&raw[..7]).is_empty());
        let packets = decoder.push(&raw[7..]);
        assert_eq!(packets.len(), 1);
        assert_eq!(packets[0].index, 0xbb);
    }

    #[test]
    fn rejects_a_corrupt_packet_while_resynchronizing() {
        let mut raw = [
            0xfa, 0xbb, 0xc2, 0x4a, 0x7f, 0x0f, 0x78, 0x00, 0x64, 0x0f, 0x8e, 0x01, 0x27, 0x0f,
            0x6d, 0x02, 0xf4, 0x0e, 0xf7, 0x02, 0x45, 0x40,
        ];
        raw[4] ^= 1;
        let mut decoder = LdsDecoder::new();
        assert!(decoder.push(&raw).is_empty());
        assert!(decoder.discarded() > 0);
    }

    fn synthetic_packet(index: u8, speed_raw: u16, invalid: bool) -> LdsPacket {
        let distance_raw = if invalid { 0x8000 } else { 1_000 };
        LdsPacket {
            index,
            speed_raw,
            samples: [LdsSample {
                distance_raw,
                signal_strength: 10,
            }; 4],
            checksum: 0,
        }
    }

    #[test]
    fn reconstructs_a_complete_revolution() {
        let mut assembler = LdsRevolutionAssembler::new();
        for index in INDEX_MIN..=INDEX_MAX {
            assert!(
                assembler
                    .push(synthetic_packet(index, 19_200, false))
                    .is_none()
            );
        }
        let scan = assembler
            .push(synthetic_packet(INDEX_MIN, 19_264, true))
            .unwrap();
        assert!(scan.complete());
        assert_eq!(scan.packet_count, 90);
        assert_eq!(scan.missing_packets(), 0);
        assert_eq!(scan.observed_sample_count(), 360);
        assert_eq!(scan.valid_sample_count(), 360);
        assert_eq!(scan.invalid_sample_count(), 0);
        assert_eq!(scan.mean_speed_rpm(), 300.0);

        let partial = assembler.finish().unwrap();
        assert_eq!(partial.packet_count, 1);
        assert_eq!(partial.invalid_sample_count(), 4);
    }

    #[test]
    fn reports_partial_revolutions_and_duplicates() {
        let mut assembler = LdsRevolutionAssembler::new();
        assert!(
            assembler
                .push(synthetic_packet(0xb0, 19_000, false))
                .is_none()
        );
        assert!(
            assembler
                .push(synthetic_packet(0xb0, 19_000, false))
                .is_none()
        );
        assert!(
            assembler
                .push(synthetic_packet(0xb2, 19_100, true))
                .is_none()
        );
        let scan = assembler
            .push(synthetic_packet(INDEX_MIN, 19_200, false))
            .unwrap();
        assert!(!scan.complete());
        assert_eq!(scan.packet_count, 2);
        assert_eq!(scan.duplicate_packets, 1);
        assert_eq!(scan.missing_packets(), 88);
        assert_eq!(scan.observed_sample_count(), 8);
        assert_eq!(scan.valid_sample_count(), 4);
        assert_eq!(scan.invalid_sample_count(), 4);
    }
}
