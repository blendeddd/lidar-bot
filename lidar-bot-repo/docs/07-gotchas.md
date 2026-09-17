# Gotchas

Every one of these cost real time. Each entry is the symptom first, because the
symptom is what you will have when you come looking.

---

## ROS 2

### slam_toolbox runs, logs nothing, publishes nothing

**Symptom:** the process starts, sits there, and exits cleanly on Ctrl-C.
`ros2 topic hz /map` says *"does not appear to be published yet"*.
`tf2_echo map base_footprint` says `Invalid frame ID "map"`.

**Cause:** slam_toolbox is a lifecycle node. Launched as a plain `Node` it stays
*unconfigured* — no publishers, no subscriptions, no TF broadcaster.

**Tell:** "no publisher at all" rather than "a publisher with no data". The map
publisher is created in the constructor, so if the topic has no publisher, the
node never got that far.

**Fix:** launch it as a `LifecycleNode` and emit the configure and activate
transitions. See `ros2/lidarbot/launch/slam.launch.py`. Running it by hand needs
`ros2 lifecycle set /slam_toolbox configure` then `activate`.

### slam_toolbox receives no scans even though /scan is publishing

**Symptom:** `/scan` at a healthy rate, `/map` never appears.

**Cause:** QoS. A BEST_EFFORT publisher cannot satisfy a RELIABLE subscriber, and
nothing warns you — the data simply never flows.

**Tell:** `ros2 topic info /scan --verbose` is the only place this is visible.

**Fix:** publish `/scan` RELIABLE. Foxglove warning *"Some, but not all,
publishers on topic '/scan' are offering RELIABLE"* means a **stale node is still
running** — `pkill -f ldlidar_node` before relaunching.

### AttributeError: 'RcutilsLogger' object has no attribute 'warn'

**Cause:** the `.warn` alias was removed. Use `.warning()`.

**Why it hurts:** it fires inside error-handling paths, so it only crashes when
something else is already going wrong, and it kills the node.

### /cmd_vel is TwistStamped, not Twist

Newer ROS 2 moved velocity commands to `geometry_msgs/msg/TwistStamped`.
`teleop_twist_keyboard` still defaults to plain `Twist` and silently fails:
topics look connected, publisher runs, robot never moves.

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

### colcon: Unknown CMake command "ament_target_dependencies"

Missing the ament CMake build tooling. Either install it, or sidestep it — the
LiDAR driver here was rewritten in pure Python for exactly this reason.

### Duplicate package names: src/lidarbot, src/~

A literal directory named `~` created by a quoted `scp` destination.

**Remove it with the path quoted:**

```bash
rm -rf "$HOME/robot_ws/src/~"
```

Unquoted, `rm -rf ~` deletes your entire home directory. Quote it.

### Edited a file, nothing changed

`--symlink-install` symlinks Python, but a running process keeps the code it has
already loaded. Restart the node. A *new* module or console entry point also
needs a rebuild.

---

## LiDAR

### The scan is wild, spasmodic, and the rate does not match the spin speed

**Symptom:** 360 rpm reported (= 6 rev/s) but 12 scans/s published, and a wall of
malformed packets.

**Cause:** three separate framing bugs, all of which produce the same mush —
consuming a length from an unvalidated header, no CRC, and detecting a new
revolution from `angle_end < angle_start` (which fires on corrupt packets, cutting
every revolution in half).

**Fix:** one-byte resync on a bad header, CRC8 on every packet, and accumulate the
swept angle rather than watching for a wrap.

**The diagnostic that cracked it:** rpm and scans/s must agree. 360 rpm is 6 rev/s
is 6 scans/s. Anything else is the driver, not the sensor.

### LDROBOT CRC8

Polynomial `0x4D`, init 0, no reflection, no final xor. Not a standard variant —
verify a table against known-good packets before trusting it.

### LD20 when you ordered an LD19

Wire-compatible. 230400 baud, `0x54 | VerLen | speed(2) | start_angle(2) |
N×[dist(2), intensity(1)] | end_angle(2) | timestamp(2) | crc(1)`, where
`N = VerLen & 0x1F`. Treat it as an LD19.

---

## Drivetrain

### It drifts in the direction it last turned

A swivelling caster steers the robot until it swings back round. Not fixable by
wheel calibration. Needs closed-loop heading hold — see
[03-calibration.md](03-calibration.md).

### The gyro says 5 physical turns is 1 turn

Effective track width on a skid-steer is much larger than the measured one
because the wheels scrub. Mine: 175 mm measured, 246.7 mm effective.

### Spinning to a gyro target measures nothing

If the spin stops when the gyro reaches 360°, the gyro will always report 360°.
Spin for a fixed time and count laps by eye instead. Ground truth has to be
independent of the sensor you are calibrating.

### The robot suddenly turns the wrong way and behaves erratically

Gyro sign inverted relative to what the controller expects. Auto-detect it, and
put a runaway guard on the loop that disables it when the error explodes.

### Heading loop oscillates instead of holding

Gains in the wrong units. These are deg/s of yaw converted to a wheel-speed
differential through the calibrated track width; get the conversion wrong and
they are out by the track-width factor.

---

## Exploration

### It plans paths through walls

Straight-line distance ranks a frontier 4 m away through a wall above one 4.7 m
away via the doorway. Rank by route distance through known free space instead —
a wavefront flood from the robot gives both the ranking and the reachability test.

### It shuffles left and right at a doorway and never goes through

Four causes, all of which look identical from outside:

1. The obstacle check measured clearance straight down the robot's x-axis while
   it was steering diagonally, so the wall *beside* the opening braked it.
2. The dodge re-compared left-vs-right clearance every 100 ms, and the answer
   flipped as the robot rotated.
3. A hard "rotate until aligned, then creep" gate produced pivot-creep-drift-pivot.
4. Pure pursuit that picks "the first waypoint further than the lookahead" skips
   the doorway waypoint once it comes within the lookahead, and aims at the next
   one — back through the wall.

Fixes, in order: rotate the lane check to the steering bearing; commit to a dodge
direction for a fixed time; one continuous control law; and walk the path by arc
length so the carrot is always *on* the route.

### It refuses a doorway it would fit through

`clear_width` is the robot's half-width plus a margin, not its planning radius.
0.30 m demands a 600 mm corridor.

### It vetoes a route, replans the same route, and loops

The planner only sees `/map`, which lags and cannot contain anything that
appeared since. Stamp live scan returns into the planner's obstacle mask so the
route goes *round* rather than the controller vetoing it.

---

## Raspberry Pi

### WiFi credentials from the Imager are ignored

`network-config` is applied by cloud-init only on what it believes is first boot.
Editing it on an already-booted card does nothing unless `instance-id` in
`meta-data` changes too.

The long hex string the Imager writes as the password is not corruption — it is
the WPA PSK derived from passphrase + SSID.

**Permanent fix**, bypassing cloud-init — `/etc/netplan/50-wifi.yaml`, `chmod 600`:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "SSID":
          password: "passphrase"
```

```bash
sudo rfkill unblock wifi && sudo iw reg set AU && sudo netplan apply
```

### "Host is down" although the Pi is connected and pinging the internet

Broadcom WiFi power save. The radio sleeps and drops inbound connections.

```bash
sudo iw dev wlan0 set power_save off
```

Make it permanent with a systemd oneshot unit.

### Recovery without a monitor

Direct laptop ↔ Pi ethernet, static both ends. Laptop `192.168.50.1/24` with the
router field empty, Pi `eth0` `192.168.50.2/24`. No DHCP or discovery to go wrong.
Host key mismatch after reflashing is expected: `ssh-keygen -R <ip>`.

### Settings silently not saving

Running under `sudo` makes `~` resolve to `/root`, so the settings file is written
somewhere you are not looking. Print the resolved path at startup, write
atomically, and never swallow the exception.

### Camera opens but never yields a frame

The UVC *metadata* node opens successfully and returns nothing. Probe with the
V4L2 backend and prove a read succeeds before accepting a device.

### Object of type bytes is not JSON serializable

A raw JPEG frame stored in a dict that later gets serialised. Keep binary out of
anything that will be turned into JSON.
