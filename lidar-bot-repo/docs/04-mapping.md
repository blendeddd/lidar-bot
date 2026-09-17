# Mapping a house

Stop `robot_server.py` before running any of this. The ESP32 serial port has one
owner.

## 1. Build, once

From the laptop:

```bash
scp -r ros2/lidarbot pi@<pi>.local:~/robot_ws/src/
```

On the Pi:

```bash
sudo apt install -y ros-lyrical-slam-toolbox ros-lyrical-foxglove-bridge \
                    ros-lyrical-robot-state-publisher ros-lyrical-tf2-ros \
                    ros-lyrical-joy ros-lyrical-teleop-twist-joy
cd ~/robot_ws && colcon build --symlink-install
echo "source ~/robot_ws/install/setup.bash" >> ~/.bashrc
source ~/robot_ws/install/setup.bash
```

`--symlink-install` means edited Python takes effect without rebuilding — but a
*new* module or a new console entry point still needs `colcon build`, and a
running node keeps the code it already loaded. Restart the node after editing.

## 2. Bring the hardware up

```bash
ros2 launch lidarbot bringup.launch.py \
    gyro_scale:=0.999 gyro_bias:=0.324 gyro_invert:=true
```

Your own numbers, from [03-calibration.md](03-calibration.md).

Starts: `robot_state_publisher` (URDF), `base_bridge`, `ldlidar_node`,
`camera_node`, gamepad teleop, and the Foxglove bridge on port 8765.

Check before going further:

```bash
ros2 topic hz /scan     # ~6 Hz, steady
ros2 topic hz /odom     # ~25 Hz
ros2 run tf2_ros tf2_echo odom base_footprint
```

If `/scan` runs but its rate disagrees with the reported spin speed (360 rpm =
6 rev/s = 6 scans/s), the driver is mis-framing packets.

## 3. Start SLAM

```bash
ros2 launch lidarbot slam.launch.py
```

You should see the solver plugin load and then:

```
[lidarbot] slam_toolbox configured, activating
```

That line matters. slam_toolbox is a **lifecycle node** — launched as a plain
node it starts unconfigured, creates no publishers, logs nothing, and exits
cleanly on Ctrl-C having done absolutely nothing. `slam.launch.py` emits the
configure and activate transitions explicitly.

Then:

```bash
ros2 topic hz /map                                  # ~1 Hz
ros2 run tf2_ros tf2_echo map base_footprint        # resolves
```

`Invalid frame ID "map"` means slam_toolbox never activated, or never received a
scan it could use.

## 4. Watch it in Foxglove

Connection → Foxglove WebSocket → `ws://<pi>.local:8765`

3D panel settings:

- Display frame: `map`
- Topics: `/map`, `/scan`, the robot model, `/explore/path`
- Mesh up-axis: **Z-up** (otherwise the robot renders on its side)

Camera: add an Image panel on `/camera/image_raw/compressed`.

## 5. Drive and map

Hold the deadman. Drive **slowly**. Straight runs down halls, gentle turns, and
pause for a beat after each turn to let the scan matcher settle. Return to where
you started so the loop closes.

A 2D LiDAR at 6 Hz cannot be hurried. A map built carelessly is worth less than
no map, because you will navigate on it later and not know which walls are lies.

## 6. Save it

Two different saves. You want both.

**The posegraph** — slam_toolbox's internal state, lets you resume mapping later:

```bash
mkdir -p ~/maps
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/pi/maps/house'}"
```

**The occupancy grid** — what AMCL and Nav2 navigate on:

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/pi/maps/house'}}"
```

Both are safe mid-run; they do not disturb mapping. Save partway through a long
session as insurance.

To resume from a posegraph, add to `config/slam_toolbox.yaml`:

```yaml
map_file_name: /home/pi/maps/house
map_start_at_dock: true
```

Park the robot roughly where it was when you saved — odometry restarts at zero,
and the first scan match has to line up with the loaded graph.

## 7. Check the map before you trust it

Open the `.pgm`. Doubled walls or a sheared room mean a loop closure that did not
take. Much easier to fix by re-driving that area while the posegraph is still
live than after building the rest of the house on top of it.

## Diagnostics

`tools/slam_diag.sh` collects everything relevant in one pass: node list, `/scan`
QoS and subscriber count, slam_toolbox's parameters, the full TF chain at the
scan timestamp, scan shape and range statistics, clock skew, and slam's own lines
out of `~/.ros/log/latest/launch.log`.

```bash
scp tools/slam_diag.sh pi@<pi>.local:~/
ssh pi@<pi>.local 'bash ~/slam_diag.sh 2>&1 | tail -120'
```
