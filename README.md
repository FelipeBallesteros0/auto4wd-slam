# auto4wd — a 4WD robot that maps with an RPLIDAR

*[Leer en español](README.es.md)*

A small skid-steer robot that you drive with a DualShock 4 while it builds a 2D map of the room in real time.
An Arduino Due runs the four motors, reads the four quadrature encoders and the gyro, and a Raspberry Pi 5
running ROS 2 Jazzy turns that into odometry, feeds it to `slam_toolbox` together with an RPLIDAR A1 and
streams everything to [Foxglove](https://foxglove.dev) on a PC.

It is a hobby project, not a product: there is no autonomous navigation, you drive it and the map draws itself.

## Hardware

| Part | What it does |
| --- | --- |
| 4WD chassis, 4 DC gear motors with incremental quadrature encoders | 65 mm wheels, 994 counts per wheel turn (x4 decoding), 215 mm track |
| 4× BTS7960 H-bridges | one per wheel; IN1/IN2 both driven with PWM (direction = which pin carries the PWM) |
| 300 W step-down converter | battery → 12 V for the motors |
| Step-down converter with USB-C output | battery → 5 V for the Raspberry Pi |
| 4× 18650 Li-ion cells (Samsung ICR18650-26J, 2600 mAh) | the battery pack |
| Arduino Due | motors, encoders (8 interrupt-capable pins — the reason for picking the Due) and the IMU |
| MPU-9250 IMU on I²C | gyro Z for rotation (skid-steer wheels slip when turning, encoders lie) |
| Raspberry Pi 5 (8 GB), Ubuntu 24.04 + ROS 2 Jazzy | odometry node, lidar driver, SLAM, Foxglove bridge |
| RPLIDAR A1M8 (via its CP2102 USB-serial adapter) | 360° laser scans at ~8 Hz |
| DualShock 4 over Bluetooth | driving |

**Golden rule:** motors and computer never share a supply rail. With a common rail, the inrush of four motors
starting at once browns out the Pi and it reboots the moment you accelerate.

## Wiring

Four diagrams, one per subsystem (PNG + Fritzing `.fzz` in [`docs/cableado/`](docs/cableado/)):

| | |
| --- | --- |
| ![Power](docs/cableado/energia.png) | **Power** — 4×18650 pack → 300 W step-down (12 V) → the four BTS7960 → motors; a second step-down with USB-C output feeds the Pi |
| ![Motors](docs/cableado/motores.png) | **Motors** — two PWM pins per wheel (forward / reverse) from the Due to each BTS7960; 5 V, GND and both enables |
| ![Encoders](docs/cableado/encoders.png) | **Encoders** — channels A/B of the four motors on Due pins 22–29 |
| ![Sensors](docs/cableado/sensores.png) | **Sensors** — MPU-9250 on I²C (pins 20/21); the Due and the lidar go to the Pi over USB |

Pin assignment (from `teleop_due/teleop_due.ino`):

| Wheel | PWM {reverse, forward} | Encoder (A, B) |
| --- | --- | --- |
| front left | 12, 11 | 22, 23 |
| rear left | 8, 7 | 26, 27 |
| front right | 9, 10 | 24, 25 |
| rear right | 5, 6 | 28, 29 |

In the diagrams the «forward» pin goes to RPWM and «reverse» to LPWM. Which way is «forward» depends on how the motor leads are screwed into the bridge: if a wheel spins backwards, swap those two motor leads (or the two PWM pins).

## Software

```
DS4 --BT--> Raspberry Pi 5 (teleop_ros.py: gamepad + odometry) --USB serial--> Arduino Due (teleop_due.ino)
RPLIDAR A1 --USB--> Raspberry Pi 5 (rplidar_ros) --> slam_toolbox --> /map
foxglove_bridge (ws://<robot>.local:8765) --> Foxglove on the PC
```

| Path | Contents |
| --- | --- |
| `teleop_due/` | Due firmware: PWM per wheel, encoder decoding by **sampling at 20 kHz** from a timer, gyro over **software I²C** (bit-bang), 300 ms watchdog, telemetry `T fl rl fr rr yaw_mrad gz_mrad_s` at 10 Hz |
| `ros/teleop_ros.py` | ROS 2 node: gamepad → `V <left> <right>` to the Due; odometry (`/odom`, TF) with distance from encoders and rotation from the gyro; `/ruedas` diagnostic topic |
| `ros/mapper.sh`, `ros/mapper.service` | systemd launcher: lidar (with retries and a watchdog), odometry node, `slam_toolbox` (lifecycle activation with retries), `foxglove_bridge` |
| `ros/slam_params.yaml` | `slam_toolbox` parameters (5 cm cells, map update every 0.5 s) |
| `ros/layout_robot.json` | Foxglove layout: 3D panel with `/map`, `/scan` and `/odom`, camera following the robot |
| `ros/50-cloud-init.yaml`, `ros/network-config` | netplan example with home Wi-Fi + phone hotspot (placeholders) |
| `rpi/` (rest) | diagnostics: map dump to PGM, scan/map overlay, lidar orientation test, serial console, straight-line test |
| `mapeo_encoders/`, `i2c_*`, `imu_diag/`, `test_blink/` | bring-up sketches: wheel↔encoder mapping, I²C scanners, IMU diagnostics |
| `analiza2m.py`, `MAPEO_RESULTADO.md` | odometry calibration analysis and the wheel/encoder pin table |

### Build and flash the Due

The Pi can't compile for the Due, so build on a PC and flash from the Pi:

```bash
arduino-cli compile --fqbn arduino:sam:arduino_due_x_dbg teleop_due --output-dir build_teleop
# on the Pi (stop the service first: sudo systemctl stop mapper)
stty -F /dev/ttyACM0 1200; sleep 2.5
~/BOSSA/bin/bossac --port=ttyACM0 -U false -e -w -v -b teleop_due.ino.bin -R
```

Use `bossac` **1.6.1-arduino** (`git clone -b arduino https://github.com/arduino/BOSSA && make bin/bossac`).
The `bossac` 1.9 from the Ubuntu repositories reports "Verify successful" but the Due never starts the program.

### Raspberry Pi

```bash
sudo apt install ros-jazzy-ros-base ros-jazzy-slam-toolbox ros-jazzy-rplidar-ros \
                 ros-jazzy-foxglove-bridge ros-jazzy-rmw-cyclonedds-cpp
mkdir -p ~/teleop && cp ros/{mapper.sh,teleop_ros.py,slam_params.yaml} ~/teleop/
sudo cp ros/mapper.service /etc/systemd/system/ && sudo systemctl enable --now mapper
```

Then open Foxglove → *Open connection* → *Foxglove WebSocket* → `ws://<robot>.local:8765` and import
`ros/layout_robot.json`. Keep the robot still for ~3 s after power-on (gyro bias calibration).

**Controls (DS4):** R2 accelerate · L2 reverse · left stick steer (pivots in place if not accelerating) ·
Circle brake · D-pad ↑/↓ speed limit (starts at 50 %). Drive slowly when mapping: the A1 sweeps 360° in ~120 ms.

## Lessons learned (the bugs worth knowing about)

- **The lidar is mounted rotated 180°.** Its zero points backwards. With `yaw = 0` in the `base_link → laser`
  transform the map came out as a star even with a perfect gyro. Fix: `LASER_XY_YAW = (0, 0, 0.10, math.pi)`.
- **The lidar motor knocks the IMU off the I²C bus** (0 drops in 45 s with the motor stopped, 24 with it spinning).
  The SAM3X hardware TWI then hangs for ~112 ms per transaction, so the firmware talks to the IMU with
  software I²C, recovers the bus with 10 SCL pulses + STOP, and the node holds the heading during short gaps.
- **Encoders under-counted by ~7 %** in three layers: slow `digitalRead` in the ISR, the Due's
  `attachInterrupt` dispatcher dropping edges (→ timer sampling at 20 kHz), and an empirical scale of
  1.0773 measured with a tape over 3 runs of 2.10 m.
- **Skid-steer encoders lie when turning** — rotation comes from the gyro, distance from the encoders.
- **"Phantom topics" in ROS 2:** with `ROS_LOCALHOST_ONLY=1`, FastDDS did not match late-joining
  subscribers over loopback (`lo` also lacked the `MULTICAST` flag). Switching to CycloneDDS fixed it; to talk
  to the graph over SSH, export the same `RMW_IMPLEMENTATION` and `CYCLONEDDS_URI` as `mapper.sh`.
- **`serial.write()` from pyserial can block forever** when the Due's CDC-ACM stalls: the node writes with
  `O_NONBLOCK` + `os.write` and reopens the port after repeated `EAGAIN`.
- **`slam_toolbox` is a lifecycle node:** started with `ros2 run` it stays `unconfigured` until
  `configure` + `activate`.
- **Always use `/dev/serial/by-id/` paths:** after a USB hiccup the lidar came back as `ttyUSB1`.
- **A phone hotspot's SSID is the phone's name** — in this case with a trailing space. Wi-Fi SSIDs are compared
  byte by byte.

## License

[MIT](LICENSE). Wiring illustrations use parts from the Fritzing library; see [NOTICE.md](NOTICE.md).
