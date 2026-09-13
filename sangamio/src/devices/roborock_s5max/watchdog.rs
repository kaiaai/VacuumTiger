//! Hardware watchdog ownership matching the stock WatchDoge contract.

use super::lifecycle::Lifecycle;
use crate::error::{Error, Result};
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

const TIMEOUT_SECONDS: libc::c_int = 16;
const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(2);
const WDIOC_KEEPALIVE: libc::Ioctl = 0x8004_5705u32 as libc::Ioctl;
const WDIOC_SETTIMEOUT: libc::Ioctl = 0xc004_5706u32 as libc::Ioctl;

struct MagicCloseWatchdog(File);

impl MagicCloseWatchdog {
    fn open(path: &str) -> Result<Self> {
        OpenOptions::new()
            .write(true)
            .custom_flags(libc::O_CLOEXEC)
            .open(path)
            .map(Self)
            .map_err(Error::Io)
    }
}

impl AsRawFd for MagicCloseWatchdog {
    fn as_raw_fd(&self) -> RawFd {
        self.0.as_raw_fd()
    }
}

impl Drop for MagicCloseWatchdog {
    fn drop(&mut self) {
        // The stock WatchDoge process uses Linux's magic-close contract too.
        // Writing V disarms the watchdog when the descriptor closes cleanly.
        let _ = self.0.write_all(b"V");
    }
}

pub(crate) struct HardwareWatchdog {
    handle: Option<JoinHandle<Result<()>>>,
    shutdown: Arc<AtomicBool>,
}

impl HardwareWatchdog {
    pub(crate) fn start(
        path: &str,
        lifecycle: Lifecycle,
        shutdown: Arc<AtomicBool>,
    ) -> Result<Self> {
        let file = MagicCloseWatchdog::open(path).map_err(|error| {
            Error::Other(format!(
                "open S5 Max watchdog {path}: {error}; stop WatchDoge before takeover"
            ))
        })?;
        let fd = file.as_raw_fd();
        let mut timeout = TIMEOUT_SECONDS;
        if unsafe { libc::ioctl(fd, WDIOC_SETTIMEOUT, &mut timeout) } < 0 {
            return Err(std::io::Error::last_os_error().into());
        }
        if unsafe { libc::ioctl(fd, WDIOC_KEEPALIVE, 0) } < 0 {
            return Err(std::io::Error::last_os_error().into());
        }

        let path = path.to_string();
        let thread_shutdown = Arc::clone(&shutdown);
        let thread_lifecycle = lifecycle;
        let handle = thread::Builder::new()
            .name("s5max-watchdog".to_string())
            .spawn(move || {
                let file = file;
                let mut next_keepalive = Instant::now() + KEEPALIVE_INTERVAL;
                while !thread_shutdown.load(Ordering::Acquire) {
                    let now = Instant::now();
                    if now >= next_keepalive {
                        if unsafe { libc::ioctl(file.as_raw_fd(), WDIOC_KEEPALIVE, 0) } < 0 {
                            let reason = format!(
                                "watchdog keepalive {path}: {}",
                                std::io::Error::last_os_error()
                            );
                            thread_lifecycle.fault(reason.clone());
                            thread_shutdown.store(true, Ordering::Release);
                            return Err(Error::Other(reason));
                        }
                        next_keepalive = now + KEEPALIVE_INTERVAL;
                    }
                    thread::sleep(Duration::from_millis(20));
                }
                drop(file);
                Ok(())
            })
            .map_err(|error| Error::Other(format!("spawn S5 Max watchdog worker: {error}")))?;

        log::info!(
            "S5 Max watchdog acquired: {}s timeout, {}ms keepalive",
            timeout,
            KEEPALIVE_INTERVAL.as_millis()
        );
        Ok(Self {
            handle: Some(handle),
            shutdown,
        })
    }

    pub(crate) fn join(&mut self) -> Result<()> {
        let Some(handle) = self.handle.take() else {
            return Ok(());
        };
        handle.join().map_err(|_| Error::ThreadPanic)?
    }
}

impl Drop for HardwareWatchdog {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::Release);
        if let Err(error) = self.join() {
            log::error!("S5 Max watchdog shutdown failed: {error}");
        }
    }
}
