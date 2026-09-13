//! Side-effect-free MCU binary frame encoding.
//!
//! The decoder remains in [`super::packet`]. Keeping the encoder independent
//! from UART ownership lets tests and the production worker share exactly the
//! same wire representation.

use super::packet::{SYNC, crc8};

/// Encode one MCU payload, append its CRC, and escape reserved bytes.
pub fn encode_binary_frame(payload: &[u8]) -> Result<Vec<u8>, String> {
    let payload_length = u8::try_from(payload.len())
        .map_err(|_| format!("payload too large: {} bytes", payload.len()))?;
    let crc = crc8(payload, 0);
    let mut escaped = Vec::with_capacity(payload.len() + 1);
    let mut expansions = 0u8;

    for byte in payload.iter().copied().chain(std::iter::once(crc)) {
        match byte {
            0xa9 => {
                escaped.extend_from_slice(&[0xa9, 0x00]);
                expansions = expansions
                    .checked_add(1)
                    .ok_or_else(|| "escape overhead overflow".to_string())?;
            }
            0xaa => {
                escaped.extend_from_slice(&[0xa9, 0x01]);
                expansions = expansions
                    .checked_add(1)
                    .ok_or_else(|| "escape overhead overflow".to_string())?;
            }
            byte => escaped.push(byte),
        }
    }

    let escape_overhead = expansions
        .checked_add(1)
        .ok_or_else(|| "escape overhead overflow".to_string())?;
    let mut raw = Vec::with_capacity(3 + escaped.len());
    raw.extend_from_slice(&[SYNC, payload_length, escape_overhead]);
    raw.extend_from_slice(&escaped);
    Ok(raw)
}

#[cfg(test)]
mod tests {
    use super::encode_binary_frame;

    #[test]
    fn escapes_reserved_payload_bytes() {
        assert_eq!(
            encode_binary_frame(&[0xa9, 0xaa]).unwrap(),
            [0xaa, 0x02, 0x03, 0xa9, 0x00, 0xa9, 0x01, 0x8d]
        );
    }

    #[test]
    fn rejects_payloads_larger_than_the_wire_length_field() {
        assert!(encode_binary_frame(&[0; 256]).is_err());
    }
}
