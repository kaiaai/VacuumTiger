# WiFi Provisioning (captive-portal setup AP)

Lets an **unprovisioned** Proscenic M6 Pro join a home network without ADB. When
enabled and the robot has no configured network, SangamIO brings up its own
temporary WiFi access point serving a small web page; the owner connects a phone,
picks their network, and types the password. This replaces the manual
`adb shell` → edit `wpa_supplicant.conf` flow documented in the
[connect tutorial](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/).

## Why it works on this hardware

The stock firmware already ships (and itself uses) every native piece:

| Piece | Role |
|-------|------|
| Realtek **RTL8189ETV** (`nl80211`) | radio; supports AP mode reliably |
| **hostapd** | runs the setup access point |
| **dnsmasq** | DHCP + wildcard DNS (drives the captive-portal popup) |
| **wpa_supplicant** / **udhcpc** | join the chosen home network |

SangamIO drives these existing binaries — it does not reimplement WiFi.

## Enabling

Add to `/etc/sangamio.toml` (all fields except `enabled` are optional):

```toml
[wifi]
enabled = true
interface = "wlan0"
ap_ssid = "Proscenic-Setup"   # a MAC suffix is appended: Proscenic-Setup-9F3A
ap_password = ""              # 8–63 chars = WPA2 setup AP; empty = open setup AP
ap_channel = 6
ap_address = "192.168.5.1"
wpa_conf_path = "/etc/wifi/wpa_supplicant.conf"
connect_timeout_secs = 25
```

If the `[wifi]` section is absent, WiFi is left entirely to the OS (previous
behavior).

## Trigger & safety model

The portal opens only when **both** are true:

1. `[wifi] enabled = true`, and
2. `wpa_conf_path` has **no** `network={ … }` block.

So a robot that already has credentials (including every robot set up the old ADB
way) is never disturbed, and a robot that is actually online cannot be stranded in
AP mode by a flaky connectivity probe — the decision is the static config file,
not a ping. Set `force_portal = true` only for testing.

## What happens

1. **AP up** — kill any supplicant/hostapd/dnsmasq, `ifconfig wlan0 192.168.5.1`,
   start `hostapd`, start `dnsmasq` (DHCP `192.168.5.50–150`, `address=/#/192.168.5.1`).
2. **Portal** — zero-dep HTTP server on `:80`. Serves the form at `/`, answers the
   OS captive probes (`/generate_204`, `/hotspot-detect.html`, `/connecttest.txt`, …)
   with a 302 to the form so the phone's "Sign in to network" sheet appears.
3. **Submit** (`POST /connect`) — validate SSID (1–32) and password (empty or
   8–63); reject control characters; escape before writing config. The page
   responds **before** the radio switches (joining drops the phone off the AP).
4. **Station up** — write `wpa_supplicant.conf`, tear down the AP, clear the stale
   `wpa_supplicant` control socket, start `wpa_supplicant -B` + bounded `udhcpc`,
   then poll `ip addr show wlan0` for a real (non-AP, non-link-local) lease up to
   `connect_timeout_secs`.
5. **Success** → provisioning returns and the daemon continues normally, now on the
   home network. **Failure** → the setup AP comes back so the user can retry.

It is always a **mode switch** — AP for setup, station for normal use, never both
at once (Realtek AP+STA coexistence is the fragile path we avoid).

## Testing on hardware (untested as of this commit)

Power the robot **on** (solid LED — the LiDAR/motor gate doesn't matter here, but
Linux must be running), then:

1. Deploy the new `sangamio` binary and this config with `enabled = true`.
2. Ensure the robot has **no** network configured (fresh, or temporarily move the
   `network={…}` block aside / set `force_portal = true`).
3. Run SangamIO. Look for the log line
   `setup portal ready -- connect a phone to "Proscenic-Setup-XXXX"`.
4. On a phone, join that SSID → the captive sign-in sheet should appear (or browse
   to `http://192.168.5.1/`). Enter your home SSID/password, submit.
5. Expect `WiFi: joined "<ssid>" -- robot is now at <ip>`; confirm the robot is
   reachable at that IP and the setup SSID is gone.

### If something's off
- **No `hostapd`/`dnsmasq`/`iw`** — confirm they still exist on the image
  (`which hostapd dnsmasq wpa_supplicant udhcpc`). They're in the stock ROM.
- **AP never appears** — the stock `robotWifiManager` init script may be fighting
  for `wlan0`; disable it (as with `AuxCtrl`).
- **Joins but no lease** — raise `connect_timeout_secs`; check `udhcpc` flags match
  the on-device BusyBox (`-n -q -t`).
