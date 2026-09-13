//! Stop-only LDS motor ownership used by the safety lifecycle.

use crate::error::{Error, Result};
use std::fs::{File, OpenOptions};
use std::io;
use std::os::fd::AsRawFd;
use std::os::unix::fs::OpenOptionsExt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread;
use std::time::Duration;

const LDS_MOTOR_SET_SPEED: libc::Ioctl = 0x4004_f810u32 as libc::Ioctl;
pub(crate) const LDS_MOTOR_SET_CURRENT_SPEED: libc::Ioctl = 0x4004_f812u32 as libc::Ioctl;
const LDS_MOTOR_START: libc::Ioctl = 0x4004_f813u32 as libc::Ioctl;
const LDS_MOTOR_STOP: libc::Ioctl = 0x4004_f814u32 as libc::Ioctl;
const LDS_MOTOR_SET_PRODUCT_ID: libc::Ioctl = 0x4004_f826u32 as libc::Ioctl;
const TARGET_RPM_X100: libc::c_int = 30_000;
const PRODUCT_ID: libc::c_int = 1;
const START_VALUE: libc::c_int = 20_000;
const STOP_SETTLE: Duration = Duration::from_millis(50);

pub(crate) struct LdsMotorGuard {
    file: File,
    path: String,
    armed: bool,
    running: Arc<AtomicBool>,
}

impl LdsMotorGuard {
    pub(crate) fn open(path: &str) -> Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_CLOEXEC)
            .open(path)
            .map_err(|error| Error::Other(format!("open S5 Max LDS motor {path}: {error}")))?;
        Ok(Self {
            file,
            path: path.to_string(),
            armed: true,
            running: Arc::new(AtomicBool::new(false)),
        })
    }

    fn ioctl_value(&self, request: libc::Ioctl, value: libc::c_int, operation: &str) -> Result<()> {
        let mut argument = value;
        if unsafe { libc::ioctl(self.file.as_raw_fd(), request, &mut argument) } < 0 {
            return Err(Error::Other(format!(
                "{operation} S5 Max LDS motor {}: {}",
                self.path,
                io::Error::last_os_error()
            )));
        }
        Ok(())
    }

    pub(crate) fn configure_and_start(&mut self) -> Result<()> {
        self.ioctl_value(LDS_MOTOR_SET_SPEED, TARGET_RPM_X100, "set target speed")?;
        self.ioctl_value(LDS_MOTOR_SET_PRODUCT_ID, PRODUCT_ID, "set product ID")?;
        self.ioctl_value(LDS_MOTOR_START, START_VALUE, "start")?;
        self.running.store(true, Ordering::Release);
        log::info!("S5 Max LDS motor started at 300 RPM");
        Ok(())
    }

    pub(crate) fn feedback_file(&self) -> Result<File> {
        self.file.try_clone().map_err(Error::from)
    }

    pub(crate) fn running_flag(&self) -> Arc<AtomicBool> {
        Arc::clone(&self.running)
    }

    pub(crate) fn is_running(&self) -> bool {
        self.running.load(Ordering::Acquire)
    }

    pub(crate) fn stop(&self) -> Result<()> {
        self.running.store(false, Ordering::Release);
        self.ioctl_value(LDS_MOTOR_STOP, 0, "stop")?;
        thread::sleep(STOP_SETTLE);
        Ok(())
    }

    pub(crate) fn stop_and_disarm(&mut self) -> Result<()> {
        self.stop()?;
        self.armed = false;
        Ok(())
    }
}

impl Drop for LdsMotorGuard {
    fn drop(&mut self) {
        if self.armed
            && let Err(error) = self.stop()
        {
            log::error!("S5 Max LDS motor drop-stop failed: {error}");
        }
    }
}
