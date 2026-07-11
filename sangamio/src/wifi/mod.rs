//! WiFi provisioning: captive-portal setup access point.
//!
//! # Purpose
//!
//! Out of the box the Proscenic M6 Pro has no way to join a home network without
//! rooting it over ADB and hand-editing `wpa_supplicant.conf`. This module lets an
//! unprovisioned robot bring up its own temporary WiFi access point with a small
//! web page, so the owner can pick their network and type the password from a
//! phone -- the same "connect to `Robot-Setup`, fill in a form" flow every
//! consumer smart device uses.
//!
//! # Why this works on the CRL-200S
//!
//! The stock firmware already ships every native piece we need and even uses AP
//! mode itself for its factory pairing:
//! - **Realtek RTL8189ETV** radio (`nl80211`) -- supports AP mode reliably.
//! - **hostapd** -- runs the access point.
//! - **dnsmasq** -- hands out DHCP leases and (with a wildcard) drives the
//!   captive-portal popup.
//! - **wpa_supplicant / udhcpc** -- join the chosen home network afterwards.
//!
//! We drive those existing binaries; we do not reimplement WiFi.
//!
//! # Mode switch, never concurrent
//!
//! The radio is either an access point (provisioning) or a station (normal
//! operation). We never try AP+STA at once -- that is the one genuinely fragile
//! thing on Realtek parts. When the user submits credentials we tear the AP down
//! and bring the station up.
//!
//! # Safety / trigger
//!
//! [`provision_if_needed`] only opens the portal when **both**:
//! 1. `[wifi] enabled = true` in the config, and
//! 2. `wpa_supplicant.conf` contains no `network={ ... }` block.
//!
//! So an already-provisioned robot (including every robot set up the old ADB way)
//! is never disturbed, and a robot that is actually online can't be stranded in AP
//! mode by a flaky live-connectivity probe -- the trigger is the static config
//! file, not a ping.

mod portal;

use crate::config::WifiConfig;
use crate::error::{Error, Result};
use std::fs;
use std::process::Command;
use std::thread;
use std::time::{Duration, Instant};

/// Temporary files written while the setup AP is up.
const HOSTAPD_CONF: &str = "/tmp/sangamio_hostapd.conf";
const DNSMASQ_CONF: &str = "/tmp/sangamio_dnsmasq.conf";
const DNSMASQ_PID: &str = "/tmp/sangamio_dnsmasq.pid";
/// Directory where `wpa_supplicant` keeps its control sockets (matches the stock
/// firmware's `ctrl_interface`). We clear the stale socket before restarting.
const WPA_CTRL_DIR: &str = "/etc/wifi/sockets";

/// Entry point: if provisioning is enabled and the robot has no configured
/// network, run the setup portal and block until the user's network is joined.
///
/// Returns `Ok(())` when the robot is either already provisioned (nothing to do)
/// or has just been provisioned successfully. Errors are non-fatal to the daemon
/// -- `main` logs them and continues, so a provisioning hiccup never blocks the
/// rest of SangamIO from starting.
pub fn provision_if_needed(cfg: &WifiConfig) -> Result<()> {
    if !cfg.enabled {
        return Ok(());
    }

    if !cfg.force_portal && has_configured_network(cfg) {
        log::info!(
            "WiFi: a network is already configured in {}, skipping setup portal",
            cfg.wpa_conf_path
        );
        return Ok(());
    }

    log::info!(
        "WiFi: no network configured -- starting setup access point \"{}\" on {} ({})",
        ap_ssid(cfg),
        cfg.interface,
        cfg.ap_address
    );

    ap_up(cfg)?;
    // Serve the portal until the user submits credentials and we join their
    // network. On success the AP is already down (connect_station tore it down).
    let result = portal::serve(cfg);
    if result.is_err() {
        // Best-effort cleanup so we don't leave the radio wedged in AP mode.
        ap_down(cfg);
    }
    result
}

/// True if `wpa_supplicant.conf` already defines at least one network. Missing or
/// unreadable file counts as "no network" (so provisioning can proceed).
fn has_configured_network(cfg: &WifiConfig) -> bool {
    match fs::read_to_string(&cfg.wpa_conf_path) {
        Ok(contents) => contents
            .lines()
            .map(|l| l.trim())
            .filter(|l| !l.starts_with('#'))
            .any(|l| l.replace(' ', "").starts_with("network={")),
        Err(_) => false,
    }
}

/// The setup AP's SSID, with a short MAC-derived suffix appended when available.
fn ap_ssid(cfg: &WifiConfig) -> String {
    match mac_suffix(&cfg.interface) {
        Some(suffix) => format!("{}-{}", cfg.ap_ssid, suffix),
        None => cfg.ap_ssid.clone(),
    }
}

/// Last two octets of the interface MAC as uppercase hex (e.g. "9F3A"), read from
/// `/sys/class/net/<iface>/address`. `None` if unreadable.
fn mac_suffix(iface: &str) -> Option<String> {
    let addr = fs::read_to_string(format!("/sys/class/net/{}/address", iface)).ok()?;
    let hex: String = addr.trim().split(':').collect();
    if hex.len() >= 4 {
        Some(hex[hex.len() - 4..].to_uppercase())
    } else {
        None
    }
}

// ============================================================================
// Access-point (provisioning) mode
// ============================================================================

/// Bring the interface up as an access point: static IP, hostapd, dnsmasq.
fn ap_up(cfg: &WifiConfig) -> Result<()> {
    // Make sure nothing else owns the radio. On the CRL-200S the stock
    // `wifiManager` (started by the procd service `robotWifiManager`) owns wlan0
    // and *spawns* wpa_supplicant, so it must be stopped first or it just
    // respawns the supplicant and fights us for the interface. Its init script
    // sets no `respawn`, so a plain kill sticks. All best-effort / no-ops if
    // absent.
    let _ = run_quiet("killall", &["wifiManager"]);
    let _ = run_quiet("killall", &["wpa_supplicant"]);
    let _ = run_quiet("killall", &["hostapd"]);
    let _ = run_quiet("killall", &["dnsmasq"]);
    // Give the driver a moment to release the interface.
    thread::sleep(Duration::from_millis(300));

    // Static IP for the AP subnet.
    run_ok(
        "ifconfig",
        &[
            &cfg.interface,
            &cfg.ap_address,
            "netmask",
            "255.255.255.0",
            "up",
        ],
    )?;

    // hostapd runs the AP itself.
    fs::write(HOSTAPD_CONF, hostapd_conf(cfg))
        .map_err(|e| Error::Other(format!("write hostapd conf: {}", e)))?;
    run_ok("hostapd", &["-B", HOSTAPD_CONF])?;

    // dnsmasq: DHCP for clients + wildcard DNS so the phone's captive-portal
    // detector resolves everything to us and pops the sign-in sheet. Best-effort:
    // if it fails the user can still browse to the AP IP manually.
    if let Err(e) = fs::write(DNSMASQ_CONF, dnsmasq_conf(cfg))
        .map_err(|e| Error::Other(format!("write dnsmasq conf: {}", e)))
        .and_then(|_| run_ok("dnsmasq", &["-C", DNSMASQ_CONF, "-x", DNSMASQ_PID]))
    {
        log::warn!(
            "WiFi: dnsmasq failed to start ({}); clients must open http://{} manually",
            e,
            cfg.ap_address
        );
    }

    Ok(())
}

/// Tear down the access point (hostapd + our dnsmasq instance).
fn ap_down(cfg: &WifiConfig) {
    let _ = run_quiet("killall", &["hostapd"]);
    let _ = run_quiet("killall", &["dnsmasq"]);
    let _ = fs::remove_file(HOSTAPD_CONF);
    let _ = fs::remove_file(DNSMASQ_CONF);
    let _ = fs::remove_file(DNSMASQ_PID);
    let _ = cfg; // reserved for future per-interface teardown
}

fn hostapd_conf(cfg: &WifiConfig) -> String {
    let mut c = format!(
        "interface={iface}\n\
         driver=nl80211\n\
         ssid={ssid}\n\
         hw_mode=g\n\
         channel={chan}\n\
         ieee80211n=1\n\
         ignore_broadcast_ssid=0\n",
        iface = cfg.interface,
        ssid = ap_ssid(cfg),
        chan = cfg.ap_channel,
    );
    // Optional WPA2 on the setup AP.
    if cfg.ap_password.len() >= 8 && cfg.ap_password.len() <= 63 {
        c.push_str(&format!(
            "wpa=2\n\
             wpa_key_mgmt=WPA-PSK\n\
             rsn_pairwise=CCMP\n\
             wpa_passphrase={}\n",
            cfg.ap_password
        ));
    }
    c
}

fn dnsmasq_conf(cfg: &WifiConfig) -> String {
    format!(
        "interface={iface}\n\
         bind-interfaces\n\
         except-interface=lo\n\
         no-resolv\n\
         dhcp-range=192.168.5.50,192.168.5.150,255.255.255.0,12h\n\
         dhcp-option=3,{ip}\n\
         dhcp-option=6,{ip}\n\
         address=/#/{ip}\n",
        iface = cfg.interface,
        ip = cfg.ap_address,
    )
}

// ============================================================================
// Station (normal) mode
// ============================================================================

/// Write the submitted credentials, switch the radio from AP to station mode, and
/// wait for an association + DHCP lease. Returns the acquired IPv4 on success.
///
/// The caller (the portal handler) must have already sent its HTTP response,
/// because tearing down the AP drops the phone that submitted the form.
pub fn connect_station(cfg: &WifiConfig, ssid: &str, psk: &str) -> Result<String> {
    validate_credentials(ssid, psk)?;

    fs::write(&cfg.wpa_conf_path, wpa_supplicant_conf(ssid, psk))
        .map_err(|e| Error::Other(format!("write {}: {}", cfg.wpa_conf_path, e)))?;
    log::info!("WiFi: wrote credentials for SSID \"{}\", joining network", ssid);

    // Leave AP mode and clear any stale supplicant state.
    ap_down(cfg);
    let _ = run_quiet("killall", &["wpa_supplicant"]);
    let _ = fs::remove_file(format!("{}/{}", WPA_CTRL_DIR, cfg.interface));
    thread::sleep(Duration::from_millis(300));

    // Start the supplicant (backgrounded) against the config we just wrote.
    run_ok(
        "wpa_supplicant",
        &["-B", "-i", &cfg.interface, "-c", &cfg.wpa_conf_path],
    )?;
    thread::sleep(Duration::from_millis(500));

    // Poll for a lease. udhcpc is bounded (`-n` exit if no lease, `-q` quit once
    // obtained) so each attempt returns promptly.
    let deadline = Instant::now() + Duration::from_secs(cfg.connect_timeout_secs);
    loop {
        let _ = run_quiet(
            "udhcpc",
            &["-i", &cfg.interface, "-n", "-q", "-t", "3"],
        );
        if let Some(ip) = station_ip(cfg) {
            log::info!("WiFi: joined \"{}\" -- robot is now at {}", ssid, ip);
            return Ok(ip);
        }
        if Instant::now() >= deadline {
            return Err(Error::Other(format!(
                "could not join \"{}\" within {}s",
                ssid, cfg.connect_timeout_secs
            )));
        }
        thread::sleep(Duration::from_secs(1));
    }
}

/// The interface's current station IPv4, if any -- i.e. a real lease that is not
/// the AP address and not a link-local `169.254.x` self-assignment.
fn station_ip(cfg: &WifiConfig) -> Option<String> {
    let out = Command::new("ip")
        .args(["addr", "show", &cfg.interface])
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stdout);
    for line in text.lines() {
        let line = line.trim();
        if let Some(rest) = line.strip_prefix("inet ") {
            let ip = rest.split('/').next().unwrap_or("").trim();
            if !ip.is_empty()
                && ip != cfg.ap_address
                && !ip.starts_with("169.254.")
                && !ip.starts_with("127.")
            {
                return Some(ip.to_string());
            }
        }
    }
    None
}

/// Build a `wpa_supplicant.conf`. Matches the format the existing ADB tutorial
/// uses. An empty passphrase yields an open-network block.
fn wpa_supplicant_conf(ssid: &str, psk: &str) -> String {
    let mut c = String::from("update_config=1\nctrl_interface=/etc/wifi/sockets\n\nnetwork={\n");
    c.push_str(&format!("\tssid=\"{}\"\n", escape_conf(ssid)));
    if psk.is_empty() {
        c.push_str("\tkey_mgmt=NONE\n");
    } else {
        c.push_str(&format!("\tpsk=\"{}\"\n", escape_conf(psk)));
        c.push_str("\tkey_mgmt=WPA-PSK\n");
    }
    c.push_str("}\n");
    c
}

/// Reject anything that could break out of the quoted `wpa_supplicant.conf`
/// string or that the WPA spec forbids.
fn validate_credentials(ssid: &str, psk: &str) -> Result<()> {
    if ssid.is_empty() || ssid.len() > 32 {
        return Err(Error::InvalidParameter(
            "SSID must be 1-32 characters".to_string(),
        ));
    }
    if ssid.chars().any(|c| (c as u32) < 0x20) {
        return Err(Error::InvalidParameter(
            "SSID contains control characters".to_string(),
        ));
    }
    if !psk.is_empty() && (psk.len() < 8 || psk.len() > 63) {
        return Err(Error::InvalidParameter(
            "WPA password must be 8-63 characters (or empty for an open network)".to_string(),
        ));
    }
    if psk.chars().any(|c| (c as u32) < 0x20) {
        return Err(Error::InvalidParameter(
            "password contains control characters".to_string(),
        ));
    }
    Ok(())
}

/// Escape backslashes and double quotes for a quoted config string.
fn escape_conf(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

// ============================================================================
// Command helpers
// ============================================================================

/// Run a command, returning an error if it can't be spawned or exits non-zero.
fn run_ok(cmd: &str, args: &[&str]) -> Result<()> {
    let status = Command::new(cmd)
        .args(args)
        .status()
        .map_err(|e| Error::Other(format!("spawn {}: {}", cmd, e)))?;
    if status.success() {
        Ok(())
    } else {
        Err(Error::Other(format!(
            "{} exited with {}",
            cmd,
            status.code().map(|c| c.to_string()).unwrap_or_else(|| "signal".to_string())
        )))
    }
}

/// Run a command, discarding output and ignoring failure (e.g. `killall` when the
/// target isn't running). Returns whether it exited successfully.
fn run_quiet(cmd: &str, args: &[&str]) -> bool {
    Command::new(cmd)
        .args(args)
        .output()
        .map(|o| o.status.success())
        .unwrap_or(false)
}
