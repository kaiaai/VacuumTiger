# Changelog

## 2026-10-12

### SangamIO

- **Roborock S5 Max support.** New `roborock_s5max` driver by
  [@gkfabs](https://github.com/gkfabs) (Fabien Carrion). See
  `sangamio/roborock-s5max.toml` and "Roborock S5 Max live takeover" in
  `sangamio/README.md`.
- **`kinematics` sensor group.** Every driver now publishes its wheel scale,
  wheel track, tick-counter width and IMU scales in SI units, resent every 2 s.
  [vacuum_ros2_bridge](https://github.com/remakeai/vacuum_ros2_bridge) uses it,
  so one bridge serves every robot.
- **CRL-200S gyro fix.** The gyro reports 0.01025 °/s per count, not 0.061.
  The Proscenic M6 Pro configs now map yaw as `z = [0, 1]`, so CCW is positive.
  Verified with an in-place spin on an M6 Pro.
- The S5 Max LiDAR uses the packed `scan_packed` format, like the CRL-200S.
- New `[device.hardware]` options `wheel_mm_per_tick` and `wheel_track_m`
  (defaults unchanged).
- `sangamio/bin/sangamio` rebuilt from this release.
- WiFi captive-portal provisioning moved to the `feat/wifi-captive-portal`
  branch until it is tested.

### Upgrading a Proscenic M6 Pro

1. Copy the new `sangamio/bin/sangamio` to the robot, as in
   [Part 1 of the tutorial](https://makerspet.com/blog/tutorial-connect-robot-vacuum-cleaner-to-ros-2-proscenic-m6-pro/).
2. If you copied `sangamio.toml` to `/etc/sangamio.toml`, change the yaw line
   under `[device.hardware.frame_transforms.imu_gyro]` to `z = [0, 1]`.

Old and new SangamIO builds work with old and new bridges in any combination.
