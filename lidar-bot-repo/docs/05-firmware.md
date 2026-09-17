# ESP32 firmware

The driver board runs a fork of
[waveshareteam/ugv_base_general](https://github.com/waveshareteam/ugv_base_general),
the open-source Waveshare firmware for this exact board. Not `ugv_base_ros`,
which targets their other board.

Upstream gives you encoder reading, per-wheel PID, 20 kHz PWM, a JSON command
protocol over serial, INA219 telemetry, OLED output and a WiFi control panel.

## Applying the local changes

This repo carries the changes as a patch rather than a vendored copy.

```bash
git clone https://github.com/waveshareteam/ugv_base_general
cd ugv_base_general
git checkout d308df9
git apply /path/to/lidar-bot/firmware/lidarbot-changes.patch
```

Then open `General_Driver/General_Driver.ino` in the Arduino IDE with the ESP32
core installed, select **ESP32 Dev Module**, and flash.

## What changed

| Area | Change |
|---|---|
| Geometry | Wheel diameter, track width and encoder counts-per-rev for the JGB37-520 |
| Telemetry | `gz` (yaw rate) added to the feedback stream |
| Power | Battery-side INA219 at `0x41`, low-voltage buzzer at ~15.5 V |
| Safety | Command watchdog — wheels stop if no command arrives within ~0.5 s |
| Wiring | Motor channel swap, so reversed wiring is fixed in software |
| Geometry query | `T:701` returns wheel diameter, PPR and track width |
| Web page | Extended control panel |

## Why `gz` matters

Without yaw rate in the feedback stream, heading has to be derived from wheel
odometry — and on a skid-steer that is badly wrong, because the wheels scrub
through every turn. The gyro does not care.

Early on the board only sent `gz` in the 1 Hz `T:710` reply, which is far too
slow for a heading loop. After reflashing it arrives at 25.1 Hz with the rest of
the telemetry. If you see the bridge warn *"no 'gz' in telemetry"*, your firmware
predates this change.

## Before you flash

The patch sets an OTA password of `lidarbot` (it is marked `change me` in the
diff). Change it. Anything else you add — WiFi credentials especially — keep out
of the committed source.

## The serial port has one owner

`robot_server.py` and the ROS `base_bridge` both open the ESP32 port. Only one
may run at a time. `serial read failed: ... multiple access on port?` means two
things have it open — usually a stale process from a previous launch.
