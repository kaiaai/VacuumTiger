//! Exclusive raw-UART ownership for the S5 Max Linux target.

use super::sys::IoctlRequest;
use crate::error::{Error, Result};
use std::fs::{self, File, OpenOptions};
use std::io;
use std::os::fd::{AsRawFd, RawFd};
use std::os::unix::fs::OpenOptionsExt;
use std::path::Path;
use std::time::{Duration, Instant};

const WRITE_TIMEOUT: Duration = Duration::from_millis(100);

#[derive(Debug, Clone, PartialEq, Eq)]
pub(crate) struct DeviceOwner {
    pub pid: i32,
    pub fd: i32,
    pub process: String,
    pub device: String,
}

fn process_name(proc_root: &Path, pid: i32) -> String {
    fs::read_to_string(proc_root.join(pid.to_string()).join("comm"))
        .map(|name| name.trim().to_string())
        .unwrap_or_else(|_| "unknown".to_string())
}

pub(crate) fn discover_device_owners_at(
    proc_root: &Path,
    selected_devices: &[&str],
) -> Result<Vec<DeviceOwner>> {
    let entries = fs::read_dir(proc_root)?;
    let mut owners = Vec::new();

    for entry in entries.flatten() {
        let Some(pid) = entry
            .file_name()
            .to_str()
            .and_then(|name| name.parse::<i32>().ok())
        else {
            continue;
        };
        let Ok(fds) = fs::read_dir(entry.path().join("fd")) else {
            continue;
        };
        for fd_entry in fds.flatten() {
            let Some(fd) = fd_entry
                .file_name()
                .to_str()
                .and_then(|name| name.parse::<i32>().ok())
            else {
                continue;
            };
            let Ok(target) = fs::read_link(fd_entry.path()) else {
                continue;
            };
            let device = target.to_string_lossy();
            if selected_devices.iter().any(|selected| device == *selected) {
                owners.push(DeviceOwner {
                    pid,
                    fd,
                    process: process_name(proc_root, pid),
                    device: device.into_owned(),
                });
            }
        }
    }

    owners.sort_by_key(|owner| (owner.pid, owner.fd));
    Ok(owners)
}

pub(crate) fn ensure_devices_unowned(selected_devices: &[&str]) -> Result<()> {
    let owners = discover_device_owners_at(Path::new("/proc"), selected_devices)?;
    if owners.is_empty() {
        return Ok(());
    }
    let owners = owners
        .iter()
        .map(|owner| {
            format!(
                "pid={} process={} fd={} device={}",
                owner.pid, owner.process, owner.fd, owner.device
            )
        })
        .collect::<Vec<_>>()
        .join("; ");
    Err(Error::Other(format!(
        "S5 Max hardware is already owned: {owners}"
    )))
}

pub(crate) struct ExclusiveTty {
    file: Option<File>,
    original: libc::termios,
    path: String,
}

impl ExclusiveTty {
    pub(crate) fn open(path: &str, baud: libc::speed_t, baud_number: u32) -> Result<Self> {
        let file = OpenOptions::new()
            .read(true)
            .write(true)
            .custom_flags(libc::O_NOCTTY | libc::O_NONBLOCK | libc::O_CLOEXEC)
            .open(path)?;
        let fd = file.as_raw_fd();

        if unsafe { libc::ioctl(fd, libc::TIOCEXCL as IoctlRequest) } < 0 {
            return Err(Error::Other(format!(
                "TIOCEXCL {path}: {}",
                io::Error::last_os_error()
            )));
        }

        let mut original = std::mem::MaybeUninit::<libc::termios>::uninit();
        if unsafe { libc::tcgetattr(fd, original.as_mut_ptr()) } < 0 {
            return Err(Error::Other(format!(
                "tcgetattr {path}: {}",
                io::Error::last_os_error()
            )));
        }
        let original = unsafe { original.assume_init() };
        let mut configured = original;
        unsafe { libc::cfmakeraw(&mut configured) };
        configured.c_cflag &= !(libc::CSIZE | libc::PARENB | libc::CSTOPB | libc::CRTSCTS);
        configured.c_cflag |= libc::CS8 | libc::CLOCAL | libc::CREAD;
        configured.c_cc[libc::VMIN] = 0;
        configured.c_cc[libc::VTIME] = 1;

        if unsafe { libc::cfsetispeed(&mut configured, baud) } < 0
            || unsafe { libc::cfsetospeed(&mut configured, baud) } < 0
        {
            return Err(Error::Other(format!(
                "configure {baud_number} baud for {path}: {}",
                io::Error::last_os_error()
            )));
        }
        if unsafe { libc::tcsetattr(fd, libc::TCSANOW, &configured) } < 0 {
            return Err(Error::Other(format!(
                "tcsetattr {path}: {}",
                io::Error::last_os_error()
            )));
        }

        Ok(Self {
            file: Some(file),
            original,
            path: path.to_string(),
        })
    }

    pub(crate) fn fd(&self) -> RawFd {
        self.file
            .as_ref()
            .expect("exclusive tty is open")
            .as_raw_fd()
    }

    pub(crate) fn read_available(&self, buffer: &mut [u8]) -> Result<usize> {
        let count = unsafe {
            libc::read(
                self.fd(),
                buffer.as_mut_ptr().cast::<libc::c_void>(),
                buffer.len(),
            )
        };
        if count >= 0 {
            return Ok(count as usize);
        }
        let error = io::Error::last_os_error();
        if matches!(
            error.kind(),
            io::ErrorKind::Interrupted | io::ErrorKind::WouldBlock
        ) {
            Ok(0)
        } else {
            Err(error.into())
        }
    }

    pub(crate) fn poll_readable(&self, timeout: Duration) -> Result<bool> {
        let mut poll_fd = libc::pollfd {
            fd: self.fd(),
            events: libc::POLLIN,
            revents: 0,
        };
        let timeout_ms = timeout.as_millis().min(i32::MAX as u128) as i32;
        let result = unsafe { libc::poll(&mut poll_fd, 1, timeout_ms) };
        if result < 0 {
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                return Ok(false);
            }
            return Err(error.into());
        }
        if poll_fd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
            return Err(Error::Other(format!(
                "poll {} terminal revents=0x{:x}",
                self.path, poll_fd.revents
            )));
        }
        Ok(result > 0 && poll_fd.revents & libc::POLLIN != 0)
    }

    pub(crate) fn write_complete(&self, frame: &[u8]) -> Result<()> {
        let mut written = 0;
        let deadline = Instant::now() + WRITE_TIMEOUT;
        while written < frame.len() {
            let count = unsafe {
                libc::write(
                    self.fd(),
                    frame[written..].as_ptr().cast::<libc::c_void>(),
                    frame.len() - written,
                )
            };
            if count > 0 {
                written += count as usize;
                continue;
            }
            if count == 0 {
                return Err(Error::Other(format!(
                    "write {} returned zero after {written} bytes",
                    self.path
                )));
            }
            let error = io::Error::last_os_error();
            if error.kind() == io::ErrorKind::Interrupted {
                continue;
            }
            if error.kind() == io::ErrorKind::WouldBlock {
                if Instant::now() >= deadline {
                    return Err(Error::Other(format!(
                        "write {} timed out after {}ms ({written}/{} bytes)",
                        self.path,
                        WRITE_TIMEOUT.as_millis(),
                        frame.len()
                    )));
                }
                let mut poll_fd = libc::pollfd {
                    fd: self.fd(),
                    events: libc::POLLOUT,
                    revents: 0,
                };
                let result = unsafe { libc::poll(&mut poll_fd, 1, 20) };
                if result < 0 && io::Error::last_os_error().kind() == io::ErrorKind::Interrupted {
                    continue;
                }
                if result < 0 {
                    return Err(io::Error::last_os_error().into());
                }
                if poll_fd.revents & (libc::POLLERR | libc::POLLHUP | libc::POLLNVAL) != 0 {
                    return Err(Error::Other(format!(
                        "poll {} while writing returned terminal revents=0x{:x}",
                        self.path, poll_fd.revents
                    )));
                }
                continue;
            }
            return Err(error.into());
        }
        if unsafe { libc::tcdrain(self.fd()) } < 0 {
            return Err(io::Error::last_os_error().into());
        }
        Ok(())
    }
}

impl Drop for ExclusiveTty {
    fn drop(&mut self) {
        if let Some(file) = &self.file {
            let fd = file.as_raw_fd();
            let _ = unsafe { libc::tcsetattr(fd, libc::TCSANOW, &self.original) };
            let _ = unsafe { libc::ioctl(fd, libc::TIOCNXCL as IoctlRequest) };
        }
    }
}

#[cfg(test)]
mod tests {
    use super::discover_device_owners_at;
    use std::fs;
    use std::os::unix::fs::symlink;
    use std::time::{SystemTime, UNIX_EPOCH};

    #[test]
    fn reports_all_selected_device_owners() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let root = std::env::temp_dir().join(format!("s5max-driver-proc-{unique}"));
        fs::create_dir_all(root.join("490/fd")).unwrap();
        fs::write(root.join("490/comm"), "rr_loader\n").unwrap();
        symlink("/dev/ttyS2", root.join("490/fd/58")).unwrap();
        symlink("/dev/uart_mcu", root.join("490/fd/59")).unwrap();
        symlink("/dev/ttyS1", root.join("490/fd/60")).unwrap();

        let owners = discover_device_owners_at(&root, &["/dev/ttyS2", "/dev/uart_mcu"]).unwrap();
        assert_eq!(owners.len(), 2);
        assert_eq!(owners[0].process, "rr_loader");
        assert_eq!(owners[0].fd, 58);
        assert_eq!(owners[1].device, "/dev/uart_mcu");

        fs::remove_dir_all(root).unwrap();
    }
}
