# Build order

The order matters. Each stage proves something the next stage depends on, and
skipping ahead means debugging two unknowns at once. This is the order it was
actually built in, including the parts that had to be redone.

| # | Stage | Proves |
|---|---|---|
| 1 | Pi provisioning | Headless Ubuntu, WiFi that stays up, ROS 2 talker/listener |
| 2 | Simulation rehearsal | You can map and navigate *something* before hardware exists |
| 3 | Mechanical | Chassis, drivetrain, LiDAR mast |
| 4 | Power | 12.0 V measured at the board before anything is connected to it |
| 5 | Firmware | Wheels turn the right way, encoders count the right way |
| 6 | Odometry | Push 1 m → `/odom` reads 1 m. Spin 360° → heading comes back |
| 7 | Calibration | It drives *straight*. See 03-calibration.md |
| 8 | Perception | `/scan` at a steady rate, and the scan looks like the room |
| 9 | Mapping | slam_toolbox publishes `/map` and `map → odom` |
| 10 | Exploration | It picks its own targets and drives to them |
| 11 | Navigation | Nav2 + AMCL on a saved map — **not done yet** |

## 1. Pi provisioning

Ubuntu Server 26.04 LTS 64-bit, headless. ROS 2 Lyrical, `ros-lyrical-ros-base`
— never the desktop variant on 1 GB. `zram-tools` with zstd at 100% is not
optional on this much RAM.

Two things will waste a day if you do not know them: WiFi credentials set in the
Imager are applied by cloud-init only on what it believes is first boot, and
Broadcom WiFi power-save silently drops inbound connections. Both are written up
in [07-gotchas.md](07-gotchas.md).

## 2. Simulation rehearsal

Worth doing while waiting for parts. TurtleBot3 in Gazebo on the laptop, all the
way through map → save → navigate. When the real robot later publishes nothing,
you already know what "working" looks like.

## 3–4. Mechanical and power

Set the buck converter to 12.0 V with a multimeter **before** connecting the
driver board. Check polarity twice. The board feeds the Pi over the 40-pin
header, so a mistake here costs both.

A caster is a third contact point, and it has a personality: see
[03-calibration.md](03-calibration.md) for what it does to a straight line.

## 5. Firmware

Flash `ugv_base_general`, then verify signs before anything else:

- commanded forward → both wheels forward
- commanded forward → both encoder counts increase
- commanded left → the robot turns left, and the gyro reads positive

Any of these being backwards is a five-minute fix now and a week of confusing
SLAM output later. The dashboard has a `/board?fix=1` endpoint that swaps the
motor channels if they are wired the other way round.

## 6. Odometry

Before any mapping. Push the robot forward exactly 1 m by hand and read `/odom`.
Spin it 360° and check the heading returns. Then drive a 1 m square with
`odom_check`:

```bash
ros2 run lidarbot odom_check --ros-args -p loop:=4.0
```

It prints closure error, heading error and scale error at the end. Mine came out
at 1.7% closure and 0.5% heading error, which is good enough for SLAM to correct.
A map can only ever be as good as the odometry under it.

## 7. Calibration

The step that turned a robot which curved into a robot which drives straight.
Full procedure in [03-calibration.md](03-calibration.md).

## 8. Perception

`ros2 topic hz /scan` should be rock steady. If the rate and the reported spin
speed disagree — 360 rpm is 6 rev/s, so anything other than ~6 scans/s is wrong
— the driver is mis-framing packets, not the LiDAR misbehaving.

## 9. Mapping

[04-mapping.md](04-mapping.md).

## 10. Exploration

[06-exploration.md](06-exploration.md).
