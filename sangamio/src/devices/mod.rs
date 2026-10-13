//! Device driver implementations and factory.
//!
//! # Adding a New Device
//!
//! 1. **Create module**: `src/devices/my_device/mod.rs`
//! 2. **Implement trait**: `impl DeviceDriver for MyDeviceDriver`
//! 3. **Register here**: Add match arm in [`create_device`]
//!
//! ## Module Structure
//! ```text
//! devices/my_device/
//! ├── mod.rs          # MyDeviceDriver struct + DeviceDriver impl
//! ├── protocol.rs     # Packet parsing (if serial)
//! └── commands.rs     # Command handlers (optional)
//! ```
//!
//! ## Key Patterns (from CRL200S)
//! - Use `Arc<Mutex<>>` for serial port shared across threads
//! - Use `Arc<AtomicBool>` for shutdown signal
//! - Implement `Drop` to join spawned threads
//! - Return sensor groups from `initialize()` for TCP streaming
//!
//! See [`crl200s::CRL200SDriver`] for a complete example.

pub mod crl200s;
pub mod roborock_s5max;

#[cfg(feature = "mock")]
pub mod mock;

use crate::config::Config;
use crate::core::driver::DeviceDriver;
use crate::error::{Error, Result};
use crl200s::CRL200SDriver;
use roborock_s5max::RoborockS5MaxDriver;

#[cfg(feature = "mock")]
use mock::MockDriver;

/// Device factory - creates driver based on `device.type` in config.
///
/// Supported device types:
/// - `crl200s`: Real CRL-200S robot hardware
/// - `roborock_s5max`: Roborock S5 Max hardware
/// - `mock`: Simulated robot for algorithm testing (requires `mock` feature)
///
/// To add a new device type, add a match arm here:
/// ```ignore
/// "my_device" => Ok(Box::new(MyDeviceDriver::new(config.device.clone())?)),
/// ```
pub fn create_device(config: &Config) -> Result<Box<dyn DeviceDriver>> {
    match config.device.device_type.as_str() {
        "crl200s" => {
            let driver = CRL200SDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        "roborock_s5max" => {
            let driver = RoborockS5MaxDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        #[cfg(feature = "mock")]
        "mock" => {
            let driver = MockDriver::new(config.device.clone())?;
            Ok(Box::new(driver))
        }
        #[cfg(not(feature = "mock"))]
        "mock" => Err(Error::Config(
            "Mock device not available: rebuild with --features mock".to_string(),
        )),
        _ => Err(Error::UnknownDevice(config.device.device_type.clone())),
    }
}

#[cfg(test)]
mod tests {
    use super::create_device;
    use crate::config::Config;

    #[test]
    fn factory_constructs_s5max_without_touching_hardware() {
        let config: Config = basic_toml::from_str(
            r#"
[device]
type = "roborock_s5max"
name = "S5 Max test"

[device.roborock_s5max]

[network]
bind_address = "127.0.0.1:5555"
"#,
        )
        .unwrap();

        let mut driver = create_device(&config).unwrap();
        let error = match driver.initialize() {
            Ok(_) => panic!("driver unexpectedly initialized without a sequence handoff"),
            Err(error) => error,
        };
        assert!(error.to_string().contains("last_stock_sequence"));
    }
}
