# Drivetrain calibration

The problem this solves: after turning right, the robot would drift heavily right
on the next straight run. A 90° pivot followed by 1.3 m of "straight" ended up
**91.7° off line**. After calibration and heading hold, the same manoeuvre ends
4.0° off.

Two separate causes, and they need different fixes.

## Why a caster makes you drift

A swivelling caster behaves like a supermarket trolley wheel. After a turn it is
still pointing the way you were turning, and it *steers* the robot in that
direction until it swings round. Open-loop "equal power to both wheels" is not
straight — it is straight plus whatever the caster is doing.

No amount of wheel calibration fixes this, because it is not a wheel problem. It
needs closed-loop heading: measure the heading with the gyro and correct.

## Why skid-steer geometry lies

Track width is not the number you measure with a ruler. The wheels scrub
sideways when the robot turns, so the *effective* track width is larger than the
geometric one. Mine measured 175 mm and calibrated to 246.7 mm — a factor of
1.41. Using the ruler figure means every commanded turn rate is wrong by 40%.

## The procedure

Run the dashboard on the Pi (stop any ROS launch first — one serial port, one
owner):

```bash
python3 pi/robot_server.py
```

Open `http://<pi>:8080/calibrate` and work through the steps in order. Each one
depends on the last.

### Step 0 — gyro sign and axes

Auto-detects whether the gyro counts the way ROS expects (positive =
anticlockwise) and which physical axis is yaw. If the IMU is bolted in rotated,
rolling the robot will show up as pitch in the visualiser — that is the symptom
this step fixes.

### Step 0b — gyro bias

Leave the robot completely still. It measures the resting drift rate, in deg/s.
Mine was 0.324 deg/s. Unbiased, that alone is 20° of heading error a minute.

### Step 1 — wheel direction

Confirms left and right are not swapped, and that "right" in the UI matches
"right" in reality. If the motor channels are wired the other way round,
`/board?fix=1` swaps them in the ESP32 rather than making you re-crimp.

### Step 3 — wheel diameter

Drive a measured straight line, enter the true distance, and it back-solves the
effective wheel diameter from the encoder counts.

### Step 4 — track width and gyro scale, by laps

This is the step that is easy to get wrong. Spinning until the gyro *says* 360°
is circular: the gyro always reports the target back at you, because that is what
stopped the spin.

Instead the robot spins for a **fixed time** and you count laps by eye, pressing
PASSED THE MARK each time the robot comes back round to a reference point. Ground
truth is `laps × 360°`, which is independent of the sensor being calibrated.

Seven laps gives 2520° of truth. Against 2492.4° of measured gyro that is a scale
factor of 0.99. Bench-tested against a known plant, three laps recovered a
174.2 mm track width where the truth was 175 mm, and a gyro scale of 1.732 where
the truth was 1.724 — both within 0.5%.

### Step 5 — heading hold

Turn on `hold` and drive: pivot 90°, then straight. Compare the final heading
error with hold off and on. Mine: **91.7° off → 4.0° off**.

The gains are in deg/s-of-yaw units, converted to a wheel-speed differential
using the calibrated track width. Defaults that work:

```
hold_kp 2.00   hold_ki 1.20   hold_kd 0.25   hold_max 70.0
rate_kp 0.60   rate_ki 1.50   rate_max 90.0
```

## Getting the numbers into ROS

The calibration lives in `~/.lidarbot_drive.json` on the Pi. Three values have to
be passed to the ROS launch by hand, because the bridge cannot read the
dashboard's settings file:

```bash
ros2 launch lidarbot bringup.launch.py \
    gyro_scale:=0.999 gyro_bias:=0.324 gyro_invert:=true
```

`gyro_invert` must match `hdg_inv` in the JSON. If it is wrong, odometry heading
runs backwards, and SLAM cannot build anything at all.

Wheel diameter, pulses-per-rev and track width do not need passing: `base_bridge`
asks the ESP32 for them at startup (`T:701`), so the board is the single source
of truth for geometry.

## What "good" looks like

| Measurement | Mine |
|---|---|
| Gyro rate | 25.1 Hz |
| Gyro bias | 0.324 deg/s |
| Gyro scale | 0.9892 |
| Effective track width | 246.7 mm (1.41× the measured 175 mm) |
| Wheel diameter | 67.9 mm |
| Encoder PPR | 660 |
| Heading error, hold off | 91.7° |
| Heading error, hold on | 4.0° |
| Square-drive closure error | 1.7% |
