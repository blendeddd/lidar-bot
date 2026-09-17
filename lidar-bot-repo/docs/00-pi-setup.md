# Raspberry Pi setup

Headless Ubuntu Server 26.04, ROS 2 Lyrical, on a 1 GB Pi 4.

Host: user `pi`, hostname of your choosing → `ssh pi@<pi>.local`

## WiFi wouldn't connect (Imager settings ignored)

**Symptom:** WiFi credentials entered in Raspberry Pi Imager, Pi never joined the network.

**Cause:** Ubuntu applies `network-config` through cloud-init, which only runs on what it
believes is the *first* boot. Editing `network-config` on an already-booted card does nothing
unless `instance-id` in `meta-data` (same `system-boot` partition) is also changed.

**Note:** the long hex string Imager writes as the password is not corruption — it is the
WPA PSK derived from passphrase + SSID (PBKDF2-HMAC-SHA1, 4096 iterations). netplan accepts
either that or the plaintext passphrase.

**Fix (permanent, bypasses cloud-init):**
```yaml
# /etc/netplan/50-wifi.yaml   (chmod 600)
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

## Recovery access without a monitor

Direct MacBook ↔ Pi ethernet with static addresses on both ends — no DHCP, no Internet
Sharing, no discovery to go wrong. Mac USB LAN adapter → `192.168.50.1/24`, router field
empty; Pi `eth0` → `192.168.50.2/24`. `ssh pi@192.168.50.2`.

Host key mismatch after reflashing is expected: `ssh-keygen -R <ip>`.

## "Host is down" over WiFi despite the Pi being connected

**Symptom:** Pi associated to the AP, had a DHCP lease, pinged the internet fine — but the
Mac could not reach it and it did not appear in the router's client list.

**Cause:** Broadcom WiFi power save. The radio sleeps and drops inbound connections.

**Fix:**
```bash
sudo apt install -y iw && sudo iw dev wlan0 set power_save off
```
Made permanent with a systemd oneshot unit (`setup/wifi-powersave-off.service`).

## Other

- `dpkg was interrupted` → `sudo dpkg --configure -a`, then `sudo apt --fix-broken install`
- zram on 1 GB: `zram-tools`, `ALGO=zstd`, `PERCENT=100`
- `avahi-daemon` for `.local` name resolution — stops the IP hunting for good

## /cmd_vel is TwistStamped, not Twist (ROS 2 Jazzy+)

Newer ROS 2 releases moved velocity commands from `geometry_msgs/msg/Twist` to
`geometry_msgs/msg/TwistStamped` — the timestamp lets controllers reject stale commands.
`teleop_twist_keyboard` still defaults to plain `Twist`, so it silently fails to drive
anything: topics look connected, publisher runs, robot never moves.

Check with `ros2 topic info /cmd_vel --verbose` — compare the type against what your
publisher sends.

Teleop fix:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

Manual publish:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}}}"
```

**Implication for this project:** the `base_bridge` node must subscribe to `TwistStamped`
(Nav2 on Lyrical publishes it), not `Twist`.

## LD20 is LD19-compatible (verified)

Received an FHL-LD20 instead of the ordered D300/LD19P. No driver officially lists LD20
and the community lidar list has its spec fields blank, but the wire protocol is identical
to LD06/LD19: 230400 baud, packets `0x54 | VerLen | speed(2) | start_angle(2) |
N x [dist(2), intensity(1)] | end_angle(2) | timestamp(2) | crc(1)`, N = `VerLen & 0x1F`.

Confirmed by decoding it directly on the Mac with `lidar_test.py` (standalone, no ROS —
pyserial + matplotlib in a venv). **Launch it in ROS as an LD19:**
`ros2 launch ldlidar_stl_ros2 ld19.launch.py`

macOS note: Homebrew Python blocks `pip install` (PEP 668). Use a venv:
`python3 -m venv ~/robot-venv && source ~/robot-venv/bin/activate && pip install pyserial matplotlib`
