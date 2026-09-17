# lidar-bot

A differential-drive indoor robot that maps a house with a 2D LiDAR and drives
itself to a commanded coordinate. Raspberry Pi 4 running ROS 2 for the thinking,
an ESP32 driver board for the real-time wheel control, powered by an 18 V
power-tool battery.

**Status:** mapping works. Autonomous frontier exploration works in simulation and
on the bench. Nav2 waypoint navigation is not done yet.

![status](https://img.shields.io/badge/ROS_2-Lyrical-blue) ![status](https://img.shields.io/badge/stage-mapping-green)

---

## What it does today

- Builds a live occupancy map of a house with slam_toolbox and a 2D LiDAR
- Streams map, scan, odometry, TF and camera to Foxglove over WiFi
- Drives from a gamepad, with a closed-loop heading hold that keeps it straight
- Explores autonomously: finds the boundary of the known world, plans a route
  through free space, and drives there
- Has a browser dashboard on the Pi for driving, telemetry and drivetrain
  calibration

## The split, and why

The Pi does everything that needs to *think*. The ESP32 does everything that
needs to be *on time*. A 1 GB Pi cannot service ~1600 encoder interrupts a
second while running SLAM, so the ESP32 owns the wheels and reports back over
serial as JSON.

```
MacBook ── WiFi ──► Raspberry Pi 4 ── USB serial ──► ESP32 driver board ──► motors
Foxglove              ROS 2 Lyrical                  encoders, PID, IMU
                      lidar, SLAM, explorer          power telemetry
                          ▲
                          └── USB ── LD19/LD20 LiDAR
```

Navigation runs entirely on-board. WiFi carries commands in and telemetry out —
a dropout freezes the operator's view, not the robot.

---

## Repository layout

```
ros2/lidarbot/        ROS 2 package: lidar driver, base bridge, explorer, launch
pi/robot_server.py    Standalone Pi web dashboard: drive, telemetry, calibration
pi/tools/             Bench scripts: calibration, lidar decode, camera probe
firmware/             Patch against the upstream Waveshare ESP32 firmware
tools/                Diagnostics and the offline test harness
docs/                 Build order, setup, calibration, mapping, gotchas
```

The ROS 2 stack and `robot_server.py` **cannot run at the same time** — they both
want the ESP32 serial port, and it has exactly one owner. Use the dashboard for
setup and calibration, then stop it and launch ROS for mapping.

---

## Quick start

Assumes the robot is built and the firmware is flashed. If not, start at
[docs/01-build-order.md](docs/01-build-order.md).

```bash
# on the Pi, once
cd ~/robot_ws && colcon build --symlink-install
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc

# terminal 1 — hardware
ros2 launch lidarbot bringup.launch.py \
    gyro_scale:=0.999 gyro_bias:=0.324 gyro_invert:=true

# terminal 2 — mapping
ros2 launch lidarbot slam.launch.py

# terminal 3 — autonomous exploration (optional)
ros2 run lidarbot explorer --ros-args -p dry_run:=true
```

Then point Foxglove at `ws://<pi-hostname>.local:8765` and add `/map`, `/scan`,
`/explore/path` and the robot model to a 3D panel.

The gyro numbers come out of your own calibration — see
[docs/03-calibration.md](docs/03-calibration.md). Do not copy mine.

---

## Hardware

| Part | Model | Notes |
|---|---|---|
| Compute | Raspberry Pi 4, 1 GB | Headless Ubuntu Server 26.04. RAM is the binding constraint |
| Driver board | Waveshare General Driver for Robots (WS-23730) | ESP32-WROOM-32, 2× TB6612FNG, 9-axis IMU, INA219, powers the Pi over the 40-pin header |
| Motors | 2× JGB37-520, 12 V 333 RPM | Hall encoders, ~1:18.8 gearbox, 6 mm D-shaft |
| LiDAR | LDROBOT LD19 / LD20 / D300 | 360° DToF, UART 230400 8N1 over the bundled USB adapter |
| Battery | Ozito PXC 18 V | 5S, 15–21 V. Treat as unprotected at the terminals |
| Camera | USB webcam | Streaming only, not used for navigation |

Full bill of materials, power chain and printed parts:
[docs/02-hardware.md](docs/02-hardware.md).

### Power chain

```
18 V pack → rocker switch → 10 A fuse → INA219 (high-side) → XL4016 buck @ 12.0 V
    → Waveshare board → 5 V over the 40-pin header → Pi → LiDAR (USB), camera (USB)
    → PH2.0 ×2 → motors + encoders
```

One buck converter only; the driver board's regulator feeds the Pi. **Never** plug
USB-C into the Pi while it is mounted and fed from the board.

---

## ROS 2 nodes

| Node | Does |
|---|---|
| `ldlidar_node` | LD19/LD20 packets → `/scan`. Pure Python, CRC-checked |
| `base_bridge` | `/cmd_vel` ↔ ESP32 JSON, publishes `/odom`, `/imu`, `/battery_state`, `odom→base_footprint` |
| `explorer` | Frontier exploration: picks a target, plans a route, drives it |
| `camera_node` | USB webcam → `/camera/image_raw/compressed` |
| `pad_teleop` | Gamepad → `TwistStamped`, with a deadman |
| `odom_check` | Bench tool: drive a square, print closure and scale error |

`planning.py` holds the grid planner (wavefront distance, path extraction, pure
pursuit) as plain numpy with no ROS imports, so it can be tested off the robot.

### Topics worth watching

```
/scan                     6 Hz    RELIABLE — slam_toolbox will not accept BEST_EFFORT
/odom                    25 Hz    encoder distance + gyro heading
/map                      1 Hz    slam_toolbox occupancy grid
/explore/path                     the planned route (nav_msgs/Path)
/explore/marker                   current target, green sphere
/explore/candidates               every frontier considered, amber points
```

---

## Testing without the robot

The planner and the exploration controller run headless against a simulated
house — stubbed ROS, raycast LiDAR, diff-drive model:

```bash
cd tools/tests
python3 test_planning.py          # planner unit tests
python3 sim_explore.py            # full closed-loop run through a doorway
DOORW=0.76 python3 sim_explore.py # a realistic interior door
```

This is how the doorway navigation was debugged — see
[docs/06-exploration.md](docs/06-exploration.md).

---

## Documentation

| Doc | Contents |
|---|---|
| [00-pi-setup.md](docs/00-pi-setup.md) | Headless Ubuntu, WiFi that stays up, ROS 2 on 1 GB |
| [01-build-order.md](docs/01-build-order.md) | The order it was actually built in, and what each stage proves |
| [02-hardware.md](docs/02-hardware.md) | Bill of materials, power chain, printed parts |
| [03-calibration.md](docs/03-calibration.md) | Drivetrain calibration — the step that made it drive straight |
| [04-mapping.md](docs/04-mapping.md) | Bringup, SLAM, Foxglove, saving a map |
| [05-firmware.md](docs/05-firmware.md) | ESP32 changes and how to flash them |
| [06-exploration.md](docs/06-exploration.md) | How autonomous exploration works |
| [07-gotchas.md](docs/07-gotchas.md) | Every trap that cost a day, and how to spot it |

---

## Credits

ESP32 firmware is a fork of
[waveshareteam/ugv_base_general](https://github.com/waveshareteam/ugv_base_general)
(see `firmware/`). Mapping is
[slam_toolbox](https://github.com/SteveMacenski/slam_toolbox). Everything else
here is original.

MIT licensed.
