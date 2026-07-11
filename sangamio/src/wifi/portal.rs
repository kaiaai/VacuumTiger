//! Minimal captive-portal HTTP server for WiFi setup.
//!
//! Deliberately zero-dependency: a single-threaded HTTP/1.1 handler over
//! `std::net::TcpListener`, in keeping with SangamIO's small-binary ethos. It
//! serves exactly one form, answers the OS captive-portal probes so the phone
//! pops its "Sign in to network" sheet, and on submit joins the chosen network.
//!
//! Flow note: joining the home network drops the phone off our AP, so `/connect`
//! sends its "applying settings" page *before* [`super::connect_station`] switches
//! the radio. If the join fails we bring the AP back up and keep serving, so the
//! user can reconnect to the setup network and retry.

use crate::config::WifiConfig;
use crate::error::{Error, Result};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{TcpListener, TcpStream};
use std::time::Duration;

/// Serve the portal until the robot successfully joins the user's network.
///
/// Blocks. Returns `Ok(())` once connected; a returned error means the listener
/// itself failed (individual request errors are logged and the loop continues).
pub fn serve(cfg: &WifiConfig) -> Result<()> {
    let listener = TcpListener::bind("0.0.0.0:80")
        .map_err(|e| Error::Other(format!("bind captive portal :80: {}", e)))?;
    log::info!(
        "WiFi: setup portal ready -- connect a phone to \"{}\" and open http://{}/",
        super::ap_ssid(cfg),
        cfg.ap_address
    );

    for stream in listener.incoming() {
        match stream {
            Ok(stream) => {
                let _ = stream.set_read_timeout(Some(Duration::from_secs(10)));
                let _ = stream.set_write_timeout(Some(Duration::from_secs(10)));
                match handle(stream, cfg) {
                    Ok(true) => return Ok(()), // connected -- provisioning done
                    Ok(false) => {}
                    Err(e) => log::warn!("WiFi portal request error: {}", e),
                }
            }
            Err(e) => log::warn!("WiFi portal accept error: {}", e),
        }
    }
    Ok(())
}

/// Handle one request. Returns `Ok(true)` if the robot just joined the user's
/// network (caller should stop serving), `Ok(false)` otherwise.
fn handle(mut stream: TcpStream, cfg: &WifiConfig) -> Result<bool> {
    let (method, path, body) = read_request(&mut stream)?;
    log::debug!("WiFi portal: {} {}", method, path);

    // OS captive-portal probes -> redirect to the form so the sign-in sheet opens.
    // (Android /generate_204, Apple /hotspot-detect.html, Windows /connecttest.txt, ...)
    if is_captive_probe(&path) {
        redirect_to_portal(&mut stream, cfg)?;
        return Ok(false);
    }

    if method == "POST" && path == "/connect" {
        return handle_connect(stream, cfg, &body);
    }

    if path == "/" {
        respond(&mut stream, "200 OK", "text/html", &form_page(cfg, None))?;
        return Ok(false);
    }

    // Anything else: send everyone to the form.
    redirect_to_portal(&mut stream, cfg)?;
    Ok(false)
}

/// Process a submitted setup form.
fn handle_connect(mut stream: TcpStream, cfg: &WifiConfig, body: &str) -> Result<bool> {
    let params = parse_form(body);
    let ssid = params
        .iter()
        .find(|(k, _)| k == "ssid")
        .map(|(_, v)| v.clone())
        .unwrap_or_default();
    let psk = params
        .iter()
        .find(|(k, _)| k == "psk")
        .map(|(_, v)| v.clone())
        .unwrap_or_default();

    // Validate before we respond, so obvious mistakes re-show the form without
    // dropping the phone off the AP.
    if let Err(e) = super::validate_credentials(&ssid, &psk) {
        let msg = format!("{}", e);
        respond(
            &mut stream,
            "200 OK",
            "text/html",
            &form_page(cfg, Some(&msg)),
        )?;
        return Ok(false);
    }

    // Respond FIRST -- switching to station mode will drop this phone's link to
    // the AP, so we can't reliably send a page afterwards.
    respond(
        &mut stream,
        "200 OK",
        "text/html",
        &connecting_page(&ssid),
    )?;
    let _ = stream.flush();
    drop(stream);

    match super::connect_station(cfg, &ssid, &psk) {
        Ok(ip) => {
            log::info!("WiFi: provisioning complete, robot at {}", ip);
            Ok(true)
        }
        Err(e) => {
            log::warn!("WiFi: join failed ({}); restoring setup AP for retry", e);
            // Bring the setup AP back so the user can reconnect and try again.
            super::ap_up(cfg)?;
            Ok(false)
        }
    }
}

// ============================================================================
// HTTP request parsing (minimal HTTP/1.1)
// ============================================================================

/// Read one request: returns (method, path, body). Reads headers up to the blank
/// line, then exactly `Content-Length` bytes of body.
fn read_request(stream: &mut TcpStream) -> Result<(String, String, String)> {
    let mut reader = BufReader::new(stream);

    let mut request_line = String::new();
    reader
        .read_line(&mut request_line)
        .map_err(|e| Error::Other(format!("read request line: {}", e)))?;
    let mut parts = request_line.split_whitespace();
    let method = parts.next().unwrap_or("").to_string();
    let path = parts.next().unwrap_or("/").to_string();

    let mut content_length = 0usize;
    loop {
        let mut line = String::new();
        let n = reader
            .read_line(&mut line)
            .map_err(|e| Error::Other(format!("read header: {}", e)))?;
        if n == 0 || line == "\r\n" || line == "\n" {
            break;
        }
        if let Some(v) = header_value(&line, "content-length") {
            content_length = v.trim().parse().unwrap_or(0);
        }
    }

    let mut body = String::new();
    if content_length > 0 {
        // Cap the body: the form is tiny, so this also bounds a hostile client.
        let capped = content_length.min(4096);
        let mut buf = vec![0u8; capped];
        reader
            .read_exact(&mut buf)
            .map_err(|e| Error::Other(format!("read body: {}", e)))?;
        body = String::from_utf8_lossy(&buf).into_owned();
    }

    Ok((method, path, body))
}

/// Case-insensitive `Header-Name: value` extraction for a single line.
fn header_value<'a>(line: &'a str, name: &str) -> Option<&'a str> {
    let (k, v) = line.split_once(':')?;
    if k.trim().eq_ignore_ascii_case(name) {
        Some(v)
    } else {
        None
    }
}

fn is_captive_probe(path: &str) -> bool {
    matches!(
        path,
        "/generate_204"
            | "/gen_204"
            | "/hotspot-detect.html"
            | "/library/test/success.html"
            | "/ncsi.txt"
            | "/connecttest.txt"
            | "/redirect"
            | "/success.txt"
            | "/canonical.html"
    )
}

// ============================================================================
// HTTP responses
// ============================================================================

fn respond(stream: &mut TcpStream, status: &str, content_type: &str, body: &str) -> Result<()> {
    let response = format!(
        "HTTP/1.1 {status}\r\n\
         Content-Type: {ct}\r\n\
         Content-Length: {len}\r\n\
         Cache-Control: no-store\r\n\
         Connection: close\r\n\
         \r\n\
         {body}",
        status = status,
        ct = content_type,
        len = body.len(),
        body = body,
    );
    stream
        .write_all(response.as_bytes())
        .map_err(|e| Error::Other(format!("write response: {}", e)))?;
    Ok(())
}

fn redirect_to_portal(stream: &mut TcpStream, cfg: &WifiConfig) -> Result<()> {
    let response = format!(
        "HTTP/1.1 302 Found\r\n\
         Location: http://{ip}/\r\n\
         Cache-Control: no-store\r\n\
         Content-Length: 0\r\n\
         Connection: close\r\n\
         \r\n",
        ip = cfg.ap_address,
    );
    stream
        .write_all(response.as_bytes())
        .map_err(|e| Error::Other(format!("write redirect: {}", e)))?;
    Ok(())
}

// ============================================================================
// x-www-form-urlencoded parsing
// ============================================================================

/// Parse `a=b&c=d` into (key, value) pairs, percent- and `+`-decoded.
fn parse_form(body: &str) -> Vec<(String, String)> {
    body.split('&')
        .filter(|s| !s.is_empty())
        .map(|pair| {
            let (k, v) = pair.split_once('=').unwrap_or((pair, ""));
            (url_decode(k), url_decode(v))
        })
        .collect()
}

/// Percent-decode a form field (`+` -> space, `%XX` -> byte).
fn url_decode(s: &str) -> String {
    let bytes = s.as_bytes();
    let mut out: Vec<u8> = Vec::with_capacity(bytes.len());
    let mut i = 0;
    while i < bytes.len() {
        match bytes[i] {
            b'+' => out.push(b' '),
            b'%' if i + 2 < bytes.len() => {
                let hi = hex_val(bytes[i + 1]);
                let lo = hex_val(bytes[i + 2]);
                match (hi, lo) {
                    (Some(h), Some(l)) => {
                        out.push((h << 4) | l);
                        i += 2;
                    }
                    _ => out.push(b'%'),
                }
            }
            b => out.push(b),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn hex_val(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Escape text for safe interpolation into HTML.
fn html_escape(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&#39;")
}

// ============================================================================
// HTML pages (self-contained, mobile-first)
// ============================================================================

const STYLE: &str = "body{font-family:-apple-system,Segoe UI,Roboto,sans-serif;\
background:#0f172a;color:#e2e8f0;margin:0;padding:24px;display:flex;\
justify-content:center}\
.card{max-width:420px;width:100%;background:#1e293b;border-radius:16px;\
padding:28px 24px;box-shadow:0 10px 30px rgba(0,0,0,.4)}\
h1{font-size:20px;margin:0 0 4px}p{color:#94a3b8;font-size:14px;line-height:1.5}\
label{display:block;margin:18px 0 6px;font-size:13px;color:#cbd5e1}\
input{width:100%;box-sizing:border-box;padding:12px;border-radius:10px;\
border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:16px}\
button{margin-top:24px;width:100%;padding:13px;border:0;border-radius:10px;\
background:#38bdf8;color:#082f49;font-size:16px;font-weight:600}\
.err{background:#7f1d1d;color:#fecaca;padding:10px 12px;border-radius:8px;\
font-size:13px;margin-top:12px}";

fn form_page(cfg: &WifiConfig, error: Option<&str>) -> String {
    let err_html = match error {
        Some(e) => format!("<div class=\"err\">{}</div>", html_escape(e)),
        None => String::new(),
    };
    format!(
        "<!doctype html><html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<title>Robot WiFi Setup</title><style>{style}</style></head><body>\
<div class=\"card\">\
<h1>Connect your robot to WiFi</h1>\
<p>Enter your home network name and password. The robot ({ssid}) will join it \
and this setup network will disappear.</p>\
{err}\
<form method=\"POST\" action=\"/connect\">\
<label for=\"ssid\">Network name (SSID)</label>\
<input id=\"ssid\" name=\"ssid\" autocapitalize=\"none\" autocorrect=\"off\" required>\
<label for=\"psk\">Password</label>\
<input id=\"psk\" name=\"psk\" type=\"password\" \
placeholder=\"leave blank for an open network\">\
<button type=\"submit\">Connect</button>\
</form></div></body></html>",
        style = STYLE,
        ssid = html_escape(&super::ap_ssid(cfg)),
        err = err_html,
    )
}

fn connecting_page(ssid: &str) -> String {
    format!(
        "<!doctype html><html><head><meta charset=\"utf-8\">\
<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">\
<title>Connecting…</title><style>{style}</style></head><body>\
<div class=\"card\">\
<h1>Applying settings…</h1>\
<p>The robot is joining <b>{ssid}</b>. This setup network will now disappear — \
that's expected.</p>\
<p>Reconnect your phone to <b>{ssid}</b>. If the robot doesn't come online within \
a minute, it will bring the setup network back so you can try again.</p>\
</div></body></html>",
        style = STYLE,
        ssid = html_escape(ssid),
    )
}
