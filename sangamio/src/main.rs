//! SangamIO - Hardware abstraction daemon for robot vacuum
//!
//! ## Protocol Architecture
//!
//! - **UDP Unicast (port 5556)**: Sensor streaming to registered clients (fire-and-forget)
//! - **TCP (port 5555)**: Commands only (reliable, bidirectional)
//!
//! When a TCP client connects for commands, their IP is automatically registered
//! for UDP sensor streaming. This eliminates network flooding from broadcasts.

use sangam_io::config::Config;
use sangam_io::core::types::Command;
use sangam_io::devices::create_device;
use sangam_io::error::{Error, Result};
use sangam_io::streaming::{
    TcpReceiver, Transport, UdpClientRegistry, UdpPublisher, create_serializer,
};
use std::env;
use std::net::{SocketAddr, TcpListener, TcpStream, UdpSocket};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::thread;

/// Parse config path from command line arguments.
///
/// Supports:
/// - `sangam-io <path>` (positional)
/// - `sangam-io --config <path>` (flag-based)
/// - `sangam-io -c <path>` (short flag)
///
/// Defaults to `/etc/sangamio.toml` if not specified.
fn parse_config_path() -> String {
    let args: Vec<String> = env::args().collect();

    // Look for --config or -c flag
    for i in 1..args.len() {
        if (args[i] == "--config" || args[i] == "-c") && i + 1 < args.len() {
            return args[i + 1].clone();
        }
    }

    // Fall back to first positional argument (if it doesn't start with -)
    if args.len() > 1 && !args[1].starts_with('-') {
        return args[1].clone();
    }

    // Default path
    "/etc/sangamio.toml".to_string()
}

fn main() -> Result<()> {
    // Initialize logger
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    log::info!("SangamIO v0.3.0 starting...");

    // Get config path from args or default
    // Supports: sangam-io <path> OR sangam-io --config <path>
    let config_path = parse_config_path();

    // Load configuration
    log::info!("Using config: {}", config_path);
    let config = Config::load(&config_path)?;

    log::info!(
        "Device: {} ({})",
        config.device.name,
        config.device.device_type
    );

    // Create device driver
    let mut driver = create_device(&config)?;

    // Initialize driver and get sensor data + streaming channels
    let init_result = driver.initialize()?;
    let sensor_data = init_result.sensor_data;
    let stream_receivers = init_result.stream_receivers;
    log::info!(
        "Initialized {} sensor groups ({} with streaming channels)",
        sensor_data.len(),
        stream_receivers.len()
    );

    let driver = Arc::new(Mutex::new(driver));

    // Set up shutdown signal handler
    let running = Arc::new(AtomicBool::new(true));
    let r = Arc::clone(&running);

    ctrlc::set_handler(move || {
        log::info!("Received shutdown signal");
        r.store(false, Ordering::Relaxed);
    })
    .map_err(|e| Error::Other(format!("Error setting Ctrl-C handler: {}", e)))?;

    log::debug!("Wire format: Protobuf");

    // =========================================================================
    // UDP Unicast Setup (sensor streaming to registered clients)
    // =========================================================================
    // UDP streaming port (may be different from TCP for localhost testing)
    let udp_streaming_port = config.network.udp_port();

    // Bind UDP socket to any available port (we only send, not receive)
    let udp_socket = UdpSocket::bind("0.0.0.0:0")
        .map_err(|e| Error::Other(format!("Failed to create UDP socket: {}", e)))?;

    if udp_streaming_port == config.network.port() {
        log::debug!(
            "UDP unicast streaming enabled (port {} - same as TCP)",
            udp_streaming_port
        );
    } else {
        log::debug!(
            "UDP unicast streaming enabled (port {} - different from TCP:{})",
            udp_streaming_port,
            config.network.port()
        );
    }

    // Client registry: tracks which IP to send UDP sensor data to
    // Updated when TCP clients connect/disconnect
    let udp_client_registry: UdpClientRegistry = Arc::new(Mutex::new(None));

    // Tracks the alive-flag of the currently-active TCP connection, so a new
    // connection can signal the previous (possibly stale) one to stop and take over.
    let current_conn: Arc<Mutex<Option<Arc<AtomicBool>>>> = Arc::new(Mutex::new(None));

    // Telemetry transport for the current client (UDP default), plus a write-clone
    // of its TCP socket. Lets a NAT'd/remote client fall back from UDP to TCP
    // telemetry: the bridge requests it via the reserved "telemetry" command, the
    // TcpReceiver flips this flag, and the UdpPublisher writes frames into the socket.
    let telemetry_transport: Arc<Mutex<Transport>> = Arc::new(Mutex::new(Transport::Udp));
    let tcp_writer: Arc<Mutex<Option<TcpStream>>> = Arc::new(Mutex::new(None));

    // Spawn single UDP publisher thread (unicast to registered client)
    let udp_serializer = create_serializer();
    let udp_sensor_data = sensor_data.clone();
    let udp_running = Arc::clone(&running);
    let udp_registry_clone = Arc::clone(&udp_client_registry);
    let udp_transport = Arc::clone(&telemetry_transport);
    let udp_tcp_writer = Arc::clone(&tcp_writer);
    let _udp_handle = thread::Builder::new()
        .name("udp-publisher".to_string())
        .spawn(move || {
            let mut publisher = UdpPublisher::new(
                udp_socket,
                udp_serializer,
                udp_sensor_data,
                stream_receivers, // UDP gets ownership of streaming channels
                udp_running,
                udp_registry_clone,
                udp_transport,
                udp_tcp_writer,
            );
            if let Err(e) = publisher.run() {
                log::error!("UDP publisher error: {}", e);
            }
        })
        .map_err(|e| Error::Other(format!("Failed to spawn UDP publisher: {}", e)))?;

    // =========================================================================
    // TCP Server Setup (commands only)
    // =========================================================================
    let bind_addr = &config.network.bind_address;
    let listener = TcpListener::bind(bind_addr)
        .map_err(|e| Error::Other(format!("Failed to bind to {}: {}", bind_addr, e)))?;
    if let Err(e) = listener.set_nonblocking(true) {
        log::warn!("Failed to set nonblocking mode: {}", e);
    }

    log::info!("TCP server listening on {} (commands only)", bind_addr);
    log::info!("SangamIO running. Press Ctrl-C to stop.");

    // Main loop - accept TCP connections for commands
    // NOTE: TCP connect also registers client for UDP sensor streaming.
    // Only one client at a time - prevents conflicting commands.
    while running.load(Ordering::Relaxed) {
        match listener.accept() {
            Ok((stream, addr)) => {
                let udp_addr = SocketAddr::new(addr.ip(), udp_streaming_port);

                // Last-connection-wins: instead of rejecting a new client when one
                // is already registered, drop the previous (possibly stale) client
                // and let this new connection take over. A stale half-open client
                // would otherwise block every reconnect until it timed out.
                let conn_alive = Arc::new(AtomicBool::new(true));
                {
                    let mut cur = current_conn.lock().unwrap_or_else(|e| e.into_inner());
                    if let Some(old) = cur.take() {
                        old.store(false, Ordering::Relaxed); // signal old receiver to stop
                        log::info!(
                            "Replacing previous client with new connection from {}",
                            addr
                        );
                    }
                    *cur = Some(Arc::clone(&conn_alive));
                }
                {
                    let mut registry = udp_client_registry
                        .lock()
                        .unwrap_or_else(|e| e.into_inner());
                    *registry = Some(udp_addr);
                }
                log::info!(
                    "TCP client connected: {} (UDP streaming -> {})",
                    addr,
                    udp_addr
                );

                // Set socket to blocking mode for reliable command handling
                if let Err(e) = stream.set_nonblocking(false) {
                    log::error!("Failed to set socket to blocking mode: {}", e);
                    conn_alive.store(false, Ordering::Relaxed);
                    {
                        let mut cur = current_conn.lock().unwrap_or_else(|e| e.into_inner());
                        if cur.as_ref().map(|c| Arc::ptr_eq(c, &conn_alive)).unwrap_or(false) {
                            *cur = None;
                        }
                    }
                    if let Ok(mut guard) = udp_client_registry.lock() {
                        *guard = None;
                    }
                    continue;
                }

                // Stash a write-clone of the socket so the publisher can stream
                // telemetry over TCP if this client falls back from UDP. The new
                // client always starts on UDP (the low-latency fast path). The write
                // timeout keeps a wedged remote from freezing the 110Hz publisher.
                match stream.try_clone() {
                    Ok(w) => {
                        let _ = w.set_write_timeout(Some(std::time::Duration::from_millis(200)));
                        *tcp_writer.lock().unwrap_or_else(|e| e.into_inner()) = Some(w);
                    }
                    Err(e) => log::warn!("Failed to clone stream for TCP telemetry: {}", e),
                }
                *telemetry_transport
                    .lock()
                    .unwrap_or_else(|e| e.into_inner()) = Transport::Udp;

                // Clone resources for receiver thread
                let driver_clone = Arc::clone(&driver);
                let recv_serializer = create_serializer();
                let recv_running = Arc::clone(&running);
                let registry_clone = Arc::clone(&udp_client_registry);
                let current_conn_clone = Arc::clone(&current_conn);
                let conn_alive_recv = Arc::clone(&conn_alive);
                let recv_transport = Arc::clone(&telemetry_transport);
                let cleanup_tcp_writer = Arc::clone(&tcp_writer);
                let cleanup_transport = Arc::clone(&telemetry_transport);

                // Spawn receiver thread (commands only - no publisher needed)
                let _recv_handle = thread::Builder::new()
                    .name("tcp-receiver".to_string())
                    .spawn(move || {
                        let mut receiver = TcpReceiver::new(
                            recv_serializer,
                            driver_clone,
                            recv_running,
                            conn_alive_recv,
                            recv_transport,
                        );
                        if let Err(e) = receiver.run(stream) {
                            log::error!("TCP receiver error: {}", e);
                        }
                        log::info!("TCP client disconnected: {}", addr);

                        // Only unregister if this is still the active connection
                        // (a newer connection may have already taken over).
                        let still_current = {
                            let mut cur = current_conn_clone
                                .lock()
                                .unwrap_or_else(|e| e.into_inner());
                            if cur.as_ref().map(|c| Arc::ptr_eq(c, &conn_alive)).unwrap_or(false) {
                                *cur = None;
                                true
                            } else {
                                false
                            }
                        };
                        if still_current {
                            if let Ok(mut guard) = registry_clone.lock() {
                                *guard = None;
                            }
                            // Stop streaming telemetry into the now-dead socket and
                            // reset for the next client (which starts on UDP).
                            *cleanup_tcp_writer.lock().unwrap_or_else(|e| e.into_inner()) = None;
                            *cleanup_transport.lock().unwrap_or_else(|e| e.into_inner()) =
                                Transport::Udp;
                        }
                    });
            }
            Err(ref e) if e.kind() == std::io::ErrorKind::WouldBlock => {
                // No connection pending, sleep briefly (10ms for responsive connection acceptance)
                std::thread::sleep(std::time::Duration::from_millis(10));
            }
            Err(e) => {
                log::error!("Accept error: {}", e);
            }
        }
    }

    // Shutdown
    log::info!("Shutting down...");
    {
        let mut driver = driver
            .lock()
            .map_err(|e| Error::MutexPoisoned(format!("driver mutex: {}", e)))?;
        driver.send_command(Command::Shutdown)?;
    }

    log::info!("SangamIO stopped");
    Ok(())
}
