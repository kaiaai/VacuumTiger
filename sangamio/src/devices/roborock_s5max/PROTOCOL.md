# Roborock S5 Max hardware protocol

This document describes the protocol used by the native SangamIO driver for the Roborock S5 Max.
The reference applies to firmware `4.1.2_1668` and the tested MCU image.

## Hardware interfaces

| Interface             | Device           | Parameters            |
|-----------------------|------------------|-----------------------|
| Main MCU UART         | `/dev/ttyS2`     | 1,152,000 baud, 8N1   |
| Main MCU receive FIFO | `/dev/uart_mcu`  | 16 KiB ring           |
| LDS UART              | `/dev/ttyS1`     | 115,200 baud, 8N1     |
| LDS receive FIFO      | `/dev/uart_lds`  | 16 KiB ring           |
| LDS motor             | `/dev/lds_motor` | Linux ioctl interface |
| Hardware watchdog     | `/dev/watchdog`  | 16-second timeout     |

One worker owns the main MCU UART. It controls frame writes, sequence allocation, ACK processing, retries, and heartbeats.

Do not operate the stock hardware stack and SangamIO at the same time. They use the same device nodes.

## MCU binary frame

The MCU binary frame uses this wire form:

```text
AA  payload_length  escape_overhead  escaped(payload || crc8)
```

`payload_length` is the unescaped payload length without the CRC byte.
`escape_overhead` is one plus the number of escape expansions.

Use these escape pairs:

```text
A9 00 -> A9
A9 01 -> AA
```

The CRC uses the Dallas/Maxim CRC-8 algorithm. It uses reflected polynomial `0x8c` and initial value zero.

Calculate the CRC over the unescaped payload. Append the CRC before escape encoding.

## MCU text frame

Firmware logs use a different wire form:

```text
AA  payload_length  00  F9  subtype  text...  5E  AA
```

The final `AA` is a sentinel. It is not a binary-frame CRC.

Observed subtype `0x80` contains high-rate motor and IMU diagnostics. Observed subtype `0x81` contains state, battery, dock, and warning messages.

## Payload records

A normal payload contains type-length-value records:

```text
record_id  record_length  record_data...
```

The payload can contain more than one record. Periodic state record `0x07/0x40` starts with a fixed 64-byte body.

Most payloads end with footer record `0xd0/len2`:

```text
d0 02 acknowledgement_request sequence
```

The transmit sequence wraps from `0xff` to `0x01`. The sequence never uses zero.

If `acknowledgement_request` is one, the receiver sends `d1/len1` with the same sequence value.

Footer sequences from the MCU are ACK tokens. They are not one global receive order because MCU producers operate concurrently.

## Retry and duplicate rules

An ACK-required command uses a 23 ms response limit. The sender makes no more than three attempts.

A retry uses the same encoded frame and sequence. Do not allocate a new sequence for that retry.

Identify a duplicate MCU report with its footer sequence and exact unescaped payload. Send the ACK again, but deliver the report once.

## Heartbeat

The AP sends this normal heartbeat approximately once each second:

```text
d3 04 00 00 00 00
```

A nonzero `d3` value sets the MCU communication timeout. Value `0xffffffff` disables that timeout.

The MCU can also send `d3`. The stock AP ignores that reverse-direction record.

## Periodic state record

Record `0x07/len0x40` uses this layout. Offsets include the two-byte record header.

| Offset |     Size | Type     | Meaning                                   |
|-------:|---------:|----------|-------------------------------------------|
|      0 |        1 | `u8`     | Record ID `0x07`                          |
|      1 |        1 | `u8`     | Body length `0x40`                        |
|      2 |       12 | `f32[3]` | Acceleration                              |
|     14 |       12 | `f32[3]` | Angular rate                              |
|     26 |       16 | `f32[4]` | Quaternion `(x, y, z, w)`                 |
|     42 |        4 | `i32`    | Left cumulative wheel ticks               |
|     46 |        4 | `i32`    | Right cumulative wheel ticks              |
|     50 |        4 | `f32`    | Filtered forward value in ticks per 20 ms |
|     54 |        4 | `u32`    | Reserved, observed as zero                |
|     58 |        8 | `u64`    | MCU monotonic time in milliseconds        |
|     66 | variable | TLV      | Additional reports                        |

All multibyte values are little-endian.

The IMU uses X forward, Y left, and Z up. Positive yaw is left or counterclockwise.

## MCU reports

|            ID |   Length | Meaning                                         |
|--------------:|---------:|-------------------------------------------------|
|        `0x01` |       64 | MCU version string                              |
|        `0x02` |        8 | System-mode tag and mode                        |
|        `0x04` |        4 | Startup and internal-error word                 |
|        `0x06` |        2 | Button report                                   |
|        `0x08` |       10 | Battery summary                                 |
|        `0x0a` |        8 | Factory and test data                           |
|        `0x0b` |        8 | Black-box event                                 |
|        `0x0c` |       32 | Product or part number                          |
|        `0x0d` |       16 | Typed device-information block                  |
|        `0x0e` |        4 | RTC value                                       |
|        `0x0f` |        2 | Dock voltage in millivolts                      |
|        `0x10` |       16 | Navigation-position cache                       |
|        `0x11` | variable | AP cache slice                                  |
|        `0x12` |        2 | Charging-cycle value                            |
|        `0x13` |        4 | Dock IR masks                                   |
|        `0x16` |        2 | Battery total capacity                          |
|        `0x18` |        3 | Water-pump hard fault                           |
|        `0x20` |        2 | Dock state                                      |
|        `0x21` |        2 | Bumper state                                    |
|        `0x22` |        2 | Drop or lift state                              |
|        `0x23` |        2 | Cliff state                                     |
|        `0x24` |        2 | Dustbin state                                   |
|        `0x25` |        2 | LDS cover bumper state                          |
|        `0x26` |        2 | Water-box state                                 |
|        `0x42` |        2 | Wall-sensor ADC value                           |
|        `0x50` |        6 | Fan motor feedback                              |
|        `0x51` |        8 | Wheel motor feedback                            |
|        `0x52` |        6 | Main-brush feedback                             |
|        `0x53` |        6 | Side-brush feedback                             |
|        `0x54` | variable | Mop-motor record, logged only by stock software |
|        `0xd0` |        2 | Frame footer                                    |
|        `0xd1` |        1 | ACK token                                       |
|        `0xd2` |        8 | MCU time synchronization value                  |
|        `0xd3` |        4 | Heartbeat or heartbeat configuration            |
|        `0xd5` |       28 | Persisted sensor calibration data               |
|        `0xf1` |       16 | MCU identity tuple                              |
|        `0xf5` |        8 | MCU upgrade status                              |
|        `0xf6` | variable | NUL-terminated status string                    |
| `0xf7`-`0xfa` | variable | MCU log records                                 |

Unknown records remain available through the generic report iterator.

## Battery report

Record `0x08/len10` uses this layout:

| Bytes | Type  | Meaning                           |
|------:|-------|-----------------------------------|
|  0..1 | `u16` | Voltage in millivolts             |
|  2..3 | `u16` | Current magnitude in milliamperes |
|     4 | `u8`  | State of charge in percent        |
|  5..9 | bytes | Reserved, observed as zero        |

Record `0x16/len2` gives total battery capacity. The stock AP multiplies byte zero by 100 mAh and ignores byte one.

## Dock state

Record `0x20/len2` is a little-endian dock state:

| Value | Meaning                              |
|------:|--------------------------------------|
|   `0` | Disconnected                         |
|   `1` | Dock supply and charging established |
|   `2` | Not observed                         |
|   `3` | Physical dock contact                |

These values differ from internal MCU diagnostic names.

## Dock IR report

Record `0x13/len4` uses this layout:

| Byte | Meaning                              |
|-----:|--------------------------------------|
|    0 | Rolling mask from the left receiver  |
|    1 | Reserved                             |
|    2 | Rolling mask from the right receiver |
|    3 | Reserved                             |

Use these mask bits:

|    Bit | Meaning                    |
|-------:|----------------------------|
| `0x01` | Far-left beacon class      |
| `0x02` | Far-right beacon class     |
| `0x04` | Close-guidance left class  |
| `0x08` | Close-guidance right class |
| `0x10` | Common or center class     |

## Attachment reports

Record `0x24` gives dustbin state. Value `00 00` means that the dustbin is present on the tested unit.

Record `0x26` gives water-box state. Value `00 00` means that the water box is present on the tested unit.

The tested hardware does not provide an independent electronic mop-cloth presence report.

## Motor feedback

Record `0x51/len8` uses this layout:

| Bytes | Meaning                             |
|------:|-------------------------------------|
|  0..1 | Left motor current in milliamperes  |
|  2..3 | Right motor current in milliamperes |
|     4 | Signed left drive-demand level      |
|     5 | Signed right drive-demand level     |
|     6 | Left fault or result code           |
|     7 | Right fault or result code          |

Records `0x50`, `0x52`, and `0x53` use one common six-byte layout:

| Bytes | Meaning                       |
|------:|-------------------------------|
|  0..1 | Sentinel `0xffff`             |
|  2..3 | Motor current in milliamperes |
|     4 | Fault or result code          |
|     5 | Controller speed              |

## Error word

Record `0x04/len4` is a little-endian `u32`:

|        Field | Meaning on this MCU image              |    Stock result |
|-------------:|----------------------------------------|----------------:|
|        Bit 0 | Invalid persistent test data           | Diagnostic only |
|        Bit 1 | Gyro probe or identification failure   |       Error 101 |
|        Bit 2 | Repeated BMS communication failure     |       Error 111 |
|        Bit 3 | Compatibility field with no MCU setter |       Error 112 |
|        Bit 4 | Compatibility field value 1            |       Error 120 |
|        Bit 5 | Compatibility field value 2            |       Error 121 |
| Bits 4 and 5 | Compatibility field value 3            |       Error 122 |

Treat bits 4 and 5 as one two-bit field. Do not report them as two independent errors.

## System modes

Record `0x02/len8` contains this value:

```text
73 79 73 5f 6d 64 00 MM
 s  y  s  _  m  d  \0 mode
```

|   Mode | Meaning                                   |
|-------:|-------------------------------------------|
| `0x00` | Normal or awake                           |
| `0x01` | Idle                                      |
| `0x02` | Factory mode                              |
| `0x03` | Manual built-in test                      |
| `0x04` | Automatic built-in test                   |
| `0x05` | Mobility test                             |
| `0x06` | Shutdown                                  |
| `0x07` | Reserved and rejected                     |
| `0x08` | Energy-efficiency mode                    |
| `0x09` | Retreading factory mode                   |
| `0x20` | Start-key-only boot                       |
| `0x21` | Watchdog reset without dock power         |
| `0x22` | Reset-pin boot without dock power         |
| `0x23` | Other reset without dock power            |
| `0x40` | Dock-only boot                            |
| `0x41` | Watchdog reset with dock power            |
| `0x42` | Reset-pin boot with dock power            |
| `0x43` | Other reset with dock power               |
| `0x44` | AP power-on after a communication failure |

These modes describe MCU power and factory states. They do not describe cleaning or navigation states.

## AP commands

Each AP command follows the `d0` footer inside the payload. The command can request an ACK through footer byte zero.

|     ID | Length | Meaning                            |
|-------:|-------:|------------------------------------|
| `0x80` |      0 | Query MCU version                  |
| `0x82` |      0 | Synchronize sensor state           |
| `0x88` |      0 | Query device information           |
| `0x8c` |      0 | Query current errors               |
| `0x8f` |      0 | Query battery total capacity       |
| `0xb0` |      8 | Set MCU system mode                |
| `0xb1` |      4 | Enable or disable one subsystem    |
| `0xb3` |     16 | Set four LED channel tuples        |
| `0xb7` |      1 | Control charging                   |
| `0xbb` |      1 | Set maximum LED brightness         |
| `0xbc` |     16 | Store navigation-position cache    |
| `0xbd` |      6 | Store robot-state cache            |
| `0xbe` |      2 | Set motor threshold                |
| `0xbf` |      2 | Set water-pump lease               |
| `0xc0` |     12 | Set wheel and yaw targets          |
| `0xc1` |      2 | Set fan target                     |
| `0xc3` |      2 | Set main-brush target              |
| `0xc4` |      2 | Set side-brush target              |
| `0xd1` |      1 | Send an ACK                        |
| `0xd3` |      4 | Send heartbeat or set its interval |

## Subsystem command

Command `0xb1/len4` is a little-endian selector. Bit zero is the enable value.

| Selector without bit zero | Subsystem                |
|--------------------------:|--------------------------|
|                 `0x00002` | Bumper                   |
|                 `0x00004` | Empty compatibility hook |
|                 `0x00008` | Cliff sensors            |
|                 `0x00010` | Reserved                 |
|                 `0x00020` | Drop or lift sensors     |
|                 `0x00040` | Dustbin sensor           |
|                 `0x00080` | Main brush               |
|                 `0x00100` | Side brush               |
|                 `0x00200` | Fan                      |
|                 `0x00400` | Wheel odometry           |
|                 `0x00800` | Gyro                     |
|                 `0x02000` | Dock IR receivers        |
|                 `0x04000` | Water-box sensor         |
|                 `0x08000` | Water pump               |
|                 `0x20000` | Wall sensor              |

For example, value `0x00000002` disables the bumper. Value `0x00000003` enables the bumper.

## Wheel command

Command `0xc0/len12` uses this layout:

| Offset | Type  | Meaning                                      |
|-------:|-------|----------------------------------------------|
|      0 | `f32` | Mean wheel target in encoder ticks per 20 ms |
|      4 | `f32` | Body yaw target in radians per second        |
|      8 | `u32` | Forced safety override                       |

Positive yaw turns left or counterclockwise. Negative yaw turns right or clockwise.

Use `forced=0` for normal movement. A nonzero value bypasses MCU interlocks for the bumper, drop sensors, and dock.

Enable wheel odometry with acknowledged `b1=0x00000401` before a nonzero wheel command.

Use a 250 ms drive lease. Send forced zero wheels when that lease expires.

## Cleaning-motor commands

Command `0xc1/len2` sets the fan target in byte zero. The MCU ignores byte one.

The user-visible fan presets use these values during active cleaning:

| User choice | Request |
|-------------|--------:|
| Off         |      30 |
| Minimum     |      38 |
| Medium-low  |      55 |
| Medium-high |      75 |
| High        |     100 |

User-visible Off is not a hard stop. Use `c1 = 00 00` for a physical fan stop.

Command `0xc3/len2` sets the main-brush target in byte zero. Byte one is a force-start flag.

Command `0xc4/len2` sets the side-brush target in byte zero. The tested MCU ignores byte one.

Normal cleaning uses main-brush request 71 and side-brush request 30. Return-home operation uses requests 45 and 20.

## Water-pump command

Command `0xbf/len2` is a timed lease. The MCU multiplies both bytes by ten for its internal counters.

Use `0b 00` for a 1.1-second ON lease. Use `00 ff` for the OFF lease.

The AP creates the water levels with ON-pulse scheduling:

| Level  | Approximate ON-to-ON period |
|--------|----------------------------:|
| Low    |                     31.67 s |
| Medium |                      6.51 s |
| High   |                      4.52 s |
| Off    |        No periodic ON pulse |

The MCU has no separate Low, Medium, or High command value.

## LED command

Command `0xb3/len16` contains four channel tuples:

```text
mode:u8  parameter:u8  argument:u16_le
```

The binary structure is known. Some mode and argument values remain unknown.

## LDS packet

The LDS uses a 22-byte XV11 packet:

```text
FA  index(A0..F9)  speed_le  sample[4]  checksum_le
```

Index `A0` starts at zero degrees. Index `F9` starts at 356 degrees.

Each packet contains four one-degree samples. Ninety packets form one 360-sample revolution.

The speed value is `u16 / 64` RPM. The target speed is 300 RPM.

Each sample contains a distance and a signal value:

| Field               | Meaning                 |
|---------------------|-------------------------|
| Distance bits 0..13 | Distance in millimeters |
| Distance bit 14     | Signal-strength warning |
| Distance bit 15     | Invalid or no return    |
| Signal `u16`        | Return strength         |

Use the XV11 15-bit rolling checksum over the first ten little-endian words.

Raw LDS angles increase clockwise. Convert them to the robot frame with this expression:

```text
body_bearing = wrap(261.2 deg - raw_angle)
```

Robot-frame bearings start at the front and increase counterclockwise.

## LDS motor interface

The LDS motor uses `/dev/lds_motor`:

| Operation           |        ioctl |               Argument |
|---------------------|-------------:|-----------------------:|
| Set target speed    | `0x4004f810` |                `30000` |
| Send measured speed | `0x4004f812` |          RPM times 100 |
| Start motor         | `0x4004f813` |                `20000` |
| Stop motor          | `0x4004f814` |                    `0` |
| Set PID P           | `0x4004f815` |       Controller value |
| Set PID I           | `0x4004f816` |       Controller value |
| Set PID D           | `0x4004f817` |       Controller value |
| Set duty            | `0x4004f824` |       Controller value |
| Set product type    | `0x4004f826` | `1` on the tested unit |

Send zero measured speed every 50 ms until a valid LDS packet arrives. Use a three-second cold-start grace period.

Stop the motor on disable, startup error, shutdown, and object destruction.

## Geometry and units

The tested unit uses these measured values:

| Quantity              |           Value |
|-----------------------|----------------:|
| Wheel distance        | `0.798 mm/tick` |
| Wheel track           |       `0.229 m` |
| LDS forward raw angle |     `261.2 deg` |
| Battery capacity unit |       `100 mAh` |

Use these odometry expressions:

```text
delta_distance = 0.000798 * (delta_left + delta_right) / 2
delta_yaw = 0.000798 * (delta_right - delta_left) / 0.229
```

Keep the wheel and LDS geometry values in the device configuration. Another unit can require different values.

## Lifecycle and safety

The driver uses this lifecycle:

```text
Created -> Synchronizing -> ReadOnlyReady -> Operational -> Stopping -> Stopped
                              |                 |
                              +--------------> Fault
```

Use these safety limits:

| Limit                  |    Value |
|------------------------|---------:|
| State-report freshness | `500 ms` |
| Drive lease            | `250 ms` |
| ACK response           |  `23 ms` |
| Command attempts       |      `3` |
| MCU heartbeat interval |    `1 s` |
| Watchdog feed interval |    `2 s` |
| Watchdog timeout       |   `16 s` |

The driver rejects nonzero commands before synchronization and fresh safety reports.

The ordered stop set is forced zero wheels, hard fan stop, side-brush stop, main-brush stop, pump stop, and LDS stop.

Attempt all stop operations after one stop operation fails. Enter the Fault state after stale reports, UART errors, or exhausted ACK retries.

## Dock and LDS lifecycle

The physical dock state and the charging command are separate. Disabling charging does not remove physical dock contact.

The tested robot does not rotate the LDS normally while it remains docked. Start the LDS after the robot is physically undocked.

For an undocked start:

1. Synchronize the MCU.
2. Hold forced zero wheels.
3. Make sure that dock state is zero.
4. Set LDS product ID `1`.
5. Set the target speed to 300 RPM.
6. Start the LDS motor.
7. Send measured-speed feedback.
8. Stop the LDS motor on each exit path.

## Build and deployment limits

Firmware `4.1.2_1668` uses glibc `2.19`. A binary linked to a new host glibc will not start on the robot.

Build the robot binary as static ARM musl:

```sh
cargo build --release --target armv7-unknown-linux-musleabihf
```

Stop `rr_loader` and `WatchDoge` only through a process-handoff procedure. Preserve Valetudo for recovery.

Use the takeover and recovery procedure in `sangam-io/README.md`.

## Known limits

These fields remain incomplete:

- The names of the two `0x0b` black-box selector states.
- Most bytes in device-information report `0x0d`.
- Physical units for parts of calibration report `0xd5`.
- The application grammar of status report `0xf6`.
- Exact meanings for all LED tuple values.
- The physical meaning of motor threshold 450.
- Factory, maintenance, and firmware-upgrade operations.
- LDS position relative to the wheel axle.

The driver keeps unknown records and raw fields available. It does not assign names without evidence.

## Related documents

- `protocol-mitm/docs/ROBOROCK_S5_MAX_REVERSE_ENGINEERING.md` gives the research method.
- `sangam-io/README.md` gives the takeover and recovery procedure.
- `docs/roborock-s5max-suite-deployment.md` gives the full suite deployment procedure.
- `sangam-io/docs/roborock-s5max-driver-plan.md` gives the driver development plan.
- `sangam-io/docs/roborock-s5max-reverse-engineering.md` gives the detailed research record.
- `sangam-io/src/devices/crl200s/COMMANDS.md` gives the CRL-200S command reference.
- `sangam-io/src/devices/crl200s/SENSORSTATUS.md` gives the CRL-200S status reference.
