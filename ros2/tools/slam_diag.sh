#!/usr/bin/env bash
# lidar-bot: why is slam_toolbox not publishing /map?
# Run on the Pi with bringup + slam already running.  Nothing here changes
# anything; it only reads.
source /opt/ros/lyrical/setup.bash 2>/dev/null
source "$HOME/robot_ws/install/setup.bash" 2>/dev/null

hr() { echo; echo "===== $* ====="; }

hr "1. nodes alive"
ros2 node list

hr "2. is slam_toolbox installed at all"
ros2 pkg executables slam_toolbox 2>&1 | head

hr "3. /scan publishers and subscribers (QoS + counts)"
ros2 topic info /scan --verbose 2>&1 | head -40

hr "4. what slam_toolbox thinks its parameters are"
for p in scan_topic base_frame odom_frame map_frame mode use_sim_time \
         transform_timeout minimum_time_interval resolution throttle_scans; do
  printf '  %-24s ' "$p"
  timeout 5 ros2 param get /slam_toolbox "$p" 2>&1 | tail -1
done

hr "5. TF chain slam_toolbox must resolve for every scan"
echo "--- odom -> base_footprint (from base_bridge) ---"
timeout 4 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -12
echo "--- base_footprint -> base_laser (from robot_state_publisher) ---"
timeout 4 ros2 run tf2_ros tf2_echo base_footprint base_laser 2>&1 | head -12
echo "--- odom -> base_laser (the full lookup) ---"
timeout 4 ros2 run tf2_ros tf2_echo odom base_laser 2>&1 | head -12

hr "6. one LaserScan, header and shape"
timeout 8 ros2 topic echo /scan --once --field header 2>&1 | head -12
timeout 8 python3 - <<'PY' 2>&1 | tail -20
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
import math, time

rclpy.init()
n = Node("scan_probe")
got = {}
def cb(m):
    if got: return
    finite = [r for r in m.ranges if math.isfinite(r)]
    got.update(dict(
        frame=m.header.frame_id,
        stamp=m.header.stamp.sec + m.header.stamp.nanosec/1e9,
        n=len(m.ranges), finite=len(finite),
        amin=round(m.angle_min,4), amax=round(m.angle_max,4),
        inc=round(m.angle_increment,6),
        rmin=m.range_min, rmax=m.range_max,
        scan_time=round(m.scan_time,4),
        nearest=round(min(finite),3) if finite else None,
        farthest=round(max(finite),3) if finite else None,
        nan=sum(1 for r in m.ranges if r != r),
        zero=sum(1 for r in m.ranges if r == 0.0),
    ))
n.create_subscription(LaserScan, "/scan", cb,
    QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE,
               history=HistoryPolicy.KEEP_LAST))
end = time.time() + 5
while time.time() < end and not got:
    rclpy.spin_once(n, timeout_sec=0.2)
if got:
    now = n.get_clock().now().nanoseconds/1e9
    got["age_at_receive_s"] = round(now - got["stamp"], 4)
    for k, v in got.items():
        print(f"  {k:18} {v}")
else:
    print("  NO SCAN RECEIVED on a RELIABLE subscriber")
n.destroy_node()
rclpy.shutdown()
PY

hr "7. clock sanity (a skewed clock makes every TF lookup fail)"
date -u; echo "uptime: $(cut -d' ' -f1 /proc/uptime)s"

hr "8. slam_toolbox's own output from the launch log"
LOG=$(ls -dt "$HOME"/.ros/log/*/ 2>/dev/null | head -1)
echo "log dir: $LOG"
grep -iE "slam|karto|ceres|scan|transform|odom" "$LOG/launch.log" 2>/dev/null \
  | grep -viE "ldlidar\]: [0-9]" | tail -40
echo "--- any stderr files ---"
for f in "$LOG"*stderr*; do [ -s "$f" ] && { echo "## $f"; tail -20 "$f"; }; done

hr "9. slam services (proof the node finished initialising)"
ros2 service list 2>/dev/null | grep -i slam | head

hr "done"
