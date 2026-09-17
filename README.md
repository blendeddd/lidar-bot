
# lidar-bot26
## Basic lidar tracking platform for development

---

## Features

- **LiDAR**: basic 360 lidar scanning module for room location
- **Battery**: tool battery for simple battery managements and swaps
- **Expansion Bays**: USB, Ethernet and 12V+ XT60 communication extensions for computer vision or external modules
- **ROS2 on Pi4**: advanced development and future proof compatibility
- **Manual Override**: joystick support through ESP32 Motor Control
- **Active Cooling**: motor drivers + heatsinks
- **Upgrades**: compatible with later revisions of pi5 for local llm support on device

## Future of this project

- [ ] **SLAM**: configure and compile ros2 program for mapping
- [ ] **Enclosure Design**: design and print sturdy casing
- [ ] **Connectors**: choose and crimp correct connectors for water
- [ ] **Reliability**: ensure robot is suitable for moderate terrain
- [ ] **Weather-proofing**: general proofing of air vents for dust and rain
- [ ] **OS**: configure software build for easy deployment on upcoming robots
- [ ] **Power-metering**: accurate battery estimates and current readings


> [!NOTE]
> Basic list for whats coming next for this robot, intended to be a learning base for the software and understanding the hardware of the build

---

### Product list

| Hardware               | Quantity         |
| ---------------------- | ------------------- |
| Raspberry Pi 4 1GB   | 1        |
| Waveshare Robot Controller         | 1         |
| JGB37-520 12V 333RPM Motor       | 2     |
| D300 DTOF LiDAR LD19P         | 1        |
| XL4016 12V Buck Converter        | 1   |
| INA219 Current/Volt Sensor | 1  |
| 18V 2.5AH Tool Battery | 1 |
| Various connectors, wires and adapters        | n/a      |


> Not an accurate list of components, subject to change. Most parts are from Aliexpress and Amazon.

---

## Summary

### Hardware:
The core branches from the Raspberry Pi 4, this is run in a headless configuration. The Waveshare General driver running dual motor drivers, imu, esp32 and current sensing sits on top of the pi providing direct contact from motors to software. Setup will be powered by the 18v tool battery > Buck > External INA219 > Driver board with an optional bench power supply inlet.

### Software

**Pi Side**
Ubuntu Server 26.04 LTS (resolute) headless > ROS 2 Lyrical Luth ros-lyrical-ros-base (no desktop) > ldlidar_stl_ros2 for the LiDAR > ros-lyrical-slam-toolbox for mapping > ros-lyrical-navigation2 + nav2-bringup for navigation > teleop-twist-keyboard, joy, teleop-twist-joy for manual driving > zram-tools (zstd, 100%) 

1GB > avahi-daemon and a WiFi power-save systemd unit for networking sanity > plus our own base_bridge node, URDF, launch configs, and systemd bringup service.

**ESP32 Side**

waveshareteam/ugv_base_general — Waveshare's open-source firmware for this exact board (not ugv_base_ros, which is for their other one). Gives encoders, PID, 20kHz PWM, JSON serial protocol, INA219 telemetry, OLED and a WiFi web panel out of the box. Four local mods: wheel diameter and track width, encoder counts-per-rev, battery INA219 at 0x41 plus buzzer at ~15.5V, and a command watchdog that stops the wheels if nothing arrives within ~0.5s.

**Mac**
Foxglove for LiDAR visuals + simulation > Nav2 mapping
Also CAD modelling for enclosure, can be offshored to ipad (shapr3d)


## Progress
### 31/8/26
Parts ordered, raspberry pi flashed with ubuntu and loading most prerequisites for ROS2. Issues with wifi connection through walls but stable signal so far while out of a case. 
### 4/9/26
Draft modelling progressed, aux parts and material arrived. Development on ROS2 > Foxglove made, simulation mapping with SLAM completed.
### 17/9/26
Build complete! basic navigation software built through python and ROS2. Calibration suite through esp32 firmware for motor calib and network config
