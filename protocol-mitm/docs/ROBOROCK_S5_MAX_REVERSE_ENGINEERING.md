# Roborock S5 Max reverse-engineering method

This document gives the method used to study the Roborock S5 Max hardware protocol.
It applies to firmware `4.1.2_1668` and the MCU image named in the scope section.

The method starts with passive observation. Active operations occur only after the passive evidence defines the frame format and the safety behavior.

## Scope

The study used this robot software and firmware:

- Linux firmware `4.1.2_1668`.
- Linux kernel `3.4.39` on ARMv7.
- Ubuntu `14.04.3 LTS` with glibc `2.19`.
- MCU image `RR_RUBYSLITE_MP_STM32.A3.G1_M4.0.1_RELEASE_20230207-210523_FULL.bin`.
- Stock process host `rr_loader`.
- Stock controller processes `RoboController` and `AppProxy`.
- Stock UART library `libuart_api.so.4.0.1`.
- Stock chassis plug-in `librr_plugin_chassis.so`.
- Stock laser plug-in `librr_plugin_laser.so`.

Do not apply an address, offset, or command from this document to a different firmware without new evidence.
Binary offsets and behavior can change between firmware versions.

## Evidence classes

Use one evidence class for each result:

- Confirmed: firmware code and a live capture give the same result.
- Correlated: a controlled physical change gives a repeatable protocol change.
- Observed: the frame and bytes are known, but their purpose is not known.

Do not give a semantic name to an observed field without more evidence.
Keep unknown data available in the decoder.

## Safety rules

Do passive work before active work. A passive operation does not write to a robot device or change a shared FIFO cursor.

Use these rules for all live work:

1. Put the robot on a clear, level surface.
2. Keep the robot in reach of the operator.
3. Use a fixed time limit for each operation.
4. Prepare the stop operation before the active operation.
5. Keep wheel and cleaning-motor targets at zero during early work.
6. Stop the stock owner before another process opens an exclusive UART.
7. Do not let two processes own `/dev/ttyS2`, `/dev/ttyS1`, `/dev/lds_motor`, or `/dev/watchdog`.
8. Restore the stock stack with a reboot after each ownership experiment.
9. Make sure that Valetudo and the stock configuration remain available for recovery.

The hardware watchdog can reset the application processor. A replacement process must feed or disarm it during full takeover.

## Hardware inventory

Start with a read-only inventory of the robot. Record the kernel, firmware, mounts, partitions, processes, sockets, and device nodes.

The S5 Max uses these device paths:

| Path             | Function                             |
|------------------|--------------------------------------|
| `/dev/ttyS2`     | Main MCU UART at 1,152,000 baud      |
| `/dev/uart_mcu`  | Kernel receive FIFO for the main MCU |
| `/dev/ttyS1`     | LDS UART at 115,200 baud             |
| `/dev/uart_lds`  | Kernel receive FIFO for the LDS      |
| `/dev/lds_motor` | LDS motor controller                 |
| `/dev/watchdog`  | Hardware watchdog                    |

The stock process `rr_loader` owns the UART and LDS motor devices. `WatchDoge` owns the hardware watchdog during stock operation.

Use `/proc/<pid>/fd` and `/proc/<pid>/maps` to identify device owners and loaded libraries.
Do not stop a process during this inventory.

## Copy the stock artifacts

Copy the binaries before static analysis. The robot uses a Dropbear server without an SFTP server.

Use legacy SCP mode:

```sh
scp -O root@ROBOT:/opt/rockrobo/cleaner/bin/RoboController .
scp -O root@ROBOT:/opt/rockrobo/cleaner/lib/libuart_api.so.4.0.1 .
scp -O root@ROBOT:/opt/rockrobo/firmware/bin/MCU_FIRMWARE_MATCH .
```

Record a hash for each copied file. An address is valid only for the matching binary.

## Find the transport boundary

Examine process file descriptors before disassembly. This step identifies named pipes, UARTs, shared memory, and kernel FIFOs.

The stock controller uses `libuart_api.so.4.0.1`. Useful exported names include `init_uart`, `read_data`, `sent_cmd_sigle_thread`, and `rua_keep_mcu_heartbeat`.

Search strings and symbols for these subjects:

- UART paths and baud rates.
- Frame assembly and escaping.
- ACK and retry operations.
- Motor and sensor names.
- LDS motor operations.
- Watchdog operations.

Use Ghidra or radare2 for static analysis. Use ARM little-endian language settings for the Linux binaries.

The MCU image uses Thumb code. Load it at image base `0x08000000` when the vector table and references support that address.

## Observe receive traffic

The kernel receive FIFOs expose ring buffers through `mmap`. A ring buffer is a fixed memory area with wrapping indexes.

Map the FIFO with `PROT_READ`. Keep a private read position and do not change the stock tail value.

The observed FIFO metadata contains five `u32` values:

- Monotonic head.
- Monotonic tail.
- Ring mask.
- Element size.
- Initialization flag.

Use the difference between the current head and the private read position to copy new bytes.
Handle ring wrap before frame decoding.

## Recover frame boundaries

Find repeated sync bytes and stable length fields. Preserve incomplete data between reads because one frame can span several reads.

For the MCU stream, distinguish binary frames from firmware text logs. They use different termination rules.

For the LDS stream, search for `FA` followed by an index from `A0` through `F9`.
Use the XV11 checksum before you accept a packet.

Add captured byte sequences as unit-test fixtures. Include split frames, bad checksums, escape bytes, and unrelated leading bytes.

## Correlate data with controlled states

Change one physical state at a time. Keep all other conditions stable.

Use these passive states:

- Docked and idle.
- Undocked and idle.
- Straight cleaning.
- Left turn.
- Right turn.
- Pause and resume.
- Return to dock.
- Final dock contact.
- Dustbin removal and insertion.
- Water-box removal and insertion.
- Button press and release.
- Front raised while stationary.
- Left side raised while stationary.

Record the physical action and the capture time. Compare only reports that occur inside that interval.

Use wheel counters and quaternion yaw as independent measurements. A result is stronger when both measurements agree.

## Align MCU and LDS time

Capture both receive FIFOs in one process. Add one host monotonic timestamp to every MCU frame and LDS packet.

Fit MCU time to host time. Reject large scheduling outliers before geometric calibration.

Use full LDS revolutions for moving calibration. Partial revolutions are valid for a stationary fixed-wall test.

The S5 Max study used these calibrations:

- Ordinary cleaning supplied wheel-scale and track data.
- A stationary flat wall supplied the LDS forward angle.
- Controlled tilts supplied the IMU axis names.
- Controlled dock positions supplied the dock-beacon classes.

Keep physical values in the device configuration. Motor wear and manufacturing variation can change them.

## Observe transmit traffic

Observe stock AP-to-MCU writes before you send a command. AP means application processor.

The stock image does not include `strace` or `gdb`. The study used `ptrace` at the `write` system-call boundary.

The observer used these limits:

1. Attach to the threads of the stock UART owner.
2. Filter ARM system call 4 for the `/dev/ttyS2` file descriptor.
3. Copy at most 4,096 bytes from each stopped write buffer.
4. Do not change registers or process memory.
5. Resume each thread immediately after the copy.
6. Stop after a fixed capture interval.
7. Detach every traced thread during cleanup.

Trace docked idle first. System-call observation can delay the stock process for a short time.

Use one host clock for transmit writes and receive FIFO frames. This permits ACK and response correlation.

## Capture the boot sequence

Cold boot contains queries and state changes that do not occur during normal idle operation.

The writable startup hook is `/mnt/reserve/_root.sh`. The read-only root file system does not require modification.

Use a one-shot marker and a detached helper. Make the helper fail open so stock boot continues after an observer error.

Make a byte-for-byte backup of `_root.sh` before a change. Use `bash -n` on the candidate script before installation.

After the capture, restore the backup and remove all markers. Make sure that no observer remains in the boot path.

## Do exclusive-ownership tests

Stop the stock hardware owner before an exclusive UART operation. Prevent `WatchDoge` from restarting that owner during the operation.

First, do a zero-I/O ownership test:

1. Open `/dev/ttyS2` with `O_RDWR`, `O_NOCTTY`, `O_NONBLOCK`, and `O_CLOEXEC`.
2. Apply `TIOCEXCL`.
3. Apply raw 1,152,000 8N1 termios.
4. Make sure that the process is the only UART owner.
5. Hold ownership for a fixed interval.
6. Do not read or write the UART.
7. Restore termios and close the descriptor.
8. Reboot and examine the stock owners.

Do not continue if exclusive ownership or stock recovery fails.

## Add non-actuating queries

Continue the AP sequence from the last stock value. The sequence skips zero and wraps from `0xff` to `0x01`.

Add one query at a time. Use one write and a fixed response limit during the first operation.

The study used this order:

1. Query MCU version with `0x80`.
2. Query device information with `0x88`.
3. Query battery capacity with `0x8f`.
4. Query current errors with `0x8c`.
5. Synchronize sensors with `0x82`.

Implement acknowledgements in both directions before a continuous session. ACK means acknowledgement.

The stock retry rule uses three attempts and a 23 ms interval. A retry uses the same frame and sequence.

## Add a bounded replacement session

The continuous MCU session must own one transmit sequence and one UART writer.
Other threads submit requests through channels.

The session must do these operations:

- Send the `d3` heartbeat once each second.
- Send `d1` for each MCU frame that requests an ACK.
- Send the same ACK for a duplicate frame.
- Suppress duplicate delivery to the consumer.
- Keep MCU footer order separate from AP transmit order.
- Enter a fault state after lost reports or exhausted retries.

Do not reject a receive frame because its footer sequence is not consecutive. Concurrent MCU producers can change the observed order.

## Take ownership of the watchdog

The hardware watchdog uses a 16-second timeout. Feed it every two seconds during the bounded replacement session.

Use the watchdog magic-close operation on each normal and error exit. Restore the stock process after the experiment.

Do not stop `WatchDoge` without another watchdog owner. The application processor can reset after the timeout.

## Take ownership of the LDS motor

UART ownership does not keep the LDS rotor active. The stock laser plug-in drives `/dev/lds_motor`.

Use the recovered stock sequence:

1. Set the target to `30000`, which means 300.00 RPM.
2. Set product ID `1` for the tested S5 Max.
3. Start the motor with value `20000`.
4. Send measured speed through `0x4004f812`.
5. Send zero feedback every 50 ms until packets arrive.
6. Stop the motor with `0x4004f814` on each exit path.

The docked power state prevents normal LDS rotation on the tested robot. Start the LDS only after the robot enters the operational discharge state.

## Add actuator operations

Do not send a nonzero actuator command before the stop set and deadman timer exist. A deadman timer stops motion when command updates stop.

Use this order for active acceptance work:

1. Send forced stationary `c0(v=0,w=0,forced=1)`.
2. Make sure that wheel counters do not change.
3. Send the full stop set.
4. Operate one cleaning motor for a fixed interval.
5. Make sure that feedback becomes nonzero.
6. Stop that motor and make sure that feedback returns to zero.
7. Operate the wheels with a short lease and `forced=0`.
8. Stop lease updates and make sure that the deadman emits a forced stop.

The ordered stop set is:

1. Forced zero wheels.
2. Hard fan stop.
3. Side-brush stop.
4. Main-brush stop.
5. Pump stop.
6. LDS motor stop.

Normal wheel movement must use `forced=0`. A nonzero forced value bypasses MCU bumper, drop, and dock interlocks.

## Preserve test evidence

Store the smallest useful byte fixtures in the repository. Do not store private addresses, passwords, or unrelated robot data.

Each protocol result must state:

- The firmware version.
- The source binary hash.
- The physical state.
- The capture method.
- The expected frame bytes.
- The evidence class.
- The known limits.

Keep large raw captures outside the source tree when the unit tests contain sufficient fixtures.

## Stop conditions

Stop the work and restore the stock stack after one of these events:

- An unexpected wheel command.
- A lost heartbeat.
- An exhausted ACK retry.
- A stale sensor report.
- A UART read or write error.
- A watchdog ownership error.
- An LDS stop error.
- A process that does not detach.
- A stock process that does not return after reboot.

Do not continue an active sequence after a stop condition.

## Results from the tested unit

The method produced these working values for the tested robot:

| Quantity                   |           Value |
|----------------------------|----------------:|
| Wheel distance             | `0.798 mm/tick` |
| Wheel track                |       `0.229 m` |
| LDS forward raw angle      |     `261.2 deg` |
| LDS target speed           |       `300 RPM` |
| MCU UART                   | `1,152,000 8N1` |
| LDS UART                   |   `115,200 8N1` |
| MCU state interval         |         `20 ms` |
| MCU command retry interval |         `23 ms` |
| Drive lease                |        `250 ms` |
| State-report limit         |        `500 ms` |
| Hardware watchdog timeout  |          `16 s` |
| Battery capacity wire unit |       `100 mAh` |

These values are device defaults. Keep wheel and LDS geometry values adjustable for another unit.
