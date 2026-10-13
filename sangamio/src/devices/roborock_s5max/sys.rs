//! Platform shims so the driver type-checks on non-Linux development hosts.
//!
//! The driver only runs on Linux. These aliases exist so the crate, and the
//! driver's pure unit tests (framing, decoding, lifecycle, safety), build on
//! macOS too.

/// Request-argument type for `libc::ioctl`.
#[cfg(target_os = "linux")]
pub(crate) type IoctlRequest = libc::Ioctl;
#[cfg(not(target_os = "linux"))]
pub(crate) type IoctlRequest = libc::c_ulong;

/// MCU UART speed. Linux exposes a `B1152000` symbol; Apple termios takes the
/// literal rate as `speed_t`.
#[cfg(target_os = "linux")]
pub(crate) const MCU_BAUD: libc::speed_t = libc::B1152000;
#[cfg(not(target_os = "linux"))]
pub(crate) const MCU_BAUD: libc::speed_t = 1_152_000;
