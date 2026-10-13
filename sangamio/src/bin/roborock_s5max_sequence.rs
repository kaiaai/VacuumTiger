//! Read the stock S5 Max AP transmit sequence from a frozen `rr_loader`.
//!
//! This is deliberately a separate handoff utility: the robot's 32-bit `dd`
//! cannot seek to `/proc/PID/mem` addresses above `i32::MAX`.

use std::env;
use std::fs;
use std::fs::File;
use std::io::{Read, Seek, SeekFrom};
use std::process::ExitCode;

const LIBRARY_NAME: &str = "libuart_api";
// Offset of g_stRuaUartInfo's transmit sequence in the libuart_api build from
// Roborock firmware 4.1.2_1668 (Build ID
// 8d639ff160b8766b7bb87e848e9b0a9c0ff12a08). Re-derive this offset before
// using the handoff helper with any other firmware/library build.
const SEQUENCE_OFFSET: u64 = 0x2ff4d;

fn run() -> Result<u8, String> {
    let mut arguments = env::args().skip(1);
    let pid = arguments
        .next()
        .ok_or_else(|| "usage: roborock_s5max_sequence <rr_loader-pid>".to_string())?;
    if arguments.next().is_some() || pid.parse::<u32>().is_err() {
        return Err("usage: roborock_s5max_sequence <rr_loader-pid>".to_string());
    }

    let maps_path = format!("/proc/{pid}/maps");
    let maps =
        fs::read_to_string(&maps_path).map_err(|error| format!("read {maps_path}: {error}"))?;
    let load_bias = maps
        .lines()
        .find_map(|line| {
            let mut fields = line.split_whitespace();
            let range = fields.next()?;
            fields.next()?;
            let file_offset = fields.next()?;
            if !line.contains(LIBRARY_NAME) {
                return None;
            }
            let start = range.split_once('-')?.0;
            let start = u64::from_str_radix(start, 16).ok()?;
            let file_offset = u64::from_str_radix(file_offset, 16).ok()?;
            start.checked_sub(file_offset)
        })
        .ok_or_else(|| format!("cannot find {LIBRARY_NAME} mapping in {maps_path}"))?;

    let memory_path = format!("/proc/{pid}/mem");
    let mut memory =
        File::open(&memory_path).map_err(|error| format!("open {memory_path}: {error}"))?;
    let address = load_bias
        .checked_add(SEQUENCE_OFFSET)
        .ok_or_else(|| "sequence address overflow".to_string())?;
    memory
        .seek(SeekFrom::Start(address))
        .map_err(|error| format!("seek {memory_path} to 0x{address:x}: {error}"))?;

    let mut sequence = [0_u8; 1];
    memory
        .read_exact(&mut sequence)
        .map_err(|error| format!("read sequence at 0x{address:x}: {error}"))?;
    Ok(sequence[0])
}

fn main() -> ExitCode {
    match run() {
        Ok(sequence) => {
            println!("{sequence}");
            ExitCode::SUCCESS
        }
        Err(error) => {
            eprintln!("{error}");
            ExitCode::FAILURE
        }
    }
}
