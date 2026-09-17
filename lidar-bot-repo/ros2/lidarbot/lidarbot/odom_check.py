#!/usr/bin/env python3
"""
Read /odom and say, in plain numbers, whether the odometry is trustworthy.

Usage:
    ros2 run lidarbot odom_check
    ros2 run lidarbot odom_check --ros-args -p loop:=4.0     # perimeter walked
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Odometry


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OdomCheck(Node):
    def __init__(self):
        super().__init__("odom_check")
        self.declare_parameter("loop", 0.0)   # perimeter you actually walked, m
        self.loop = float(self.get_parameter("loop").value)

        qos = QoSProfile(depth=10,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "odom", self.on_odom, qos)

        self.first = None
        self.last = None
        self.path = 0.0            # distance travelled along the path
        self.turned = 0.0          # total heading change, radians
        self.n = 0
        self.create_timer(0.5, self.show)
        self.get_logger().info(
            "drive a shape that comes back to where it started, then Ctrl-C")

    def on_odom(self, m):
        p = m.pose.pose.position
        th = yaw_of(m.pose.pose.orientation)
        if self.first is None:
            self.first = (p.x, p.y, th)
        else:
            lx, ly, lth = self.last
            self.path += math.hypot(p.x - lx, p.y - ly)
            d = (th - lth + math.pi) % (2 * math.pi) - math.pi
            self.turned += abs(d)
        self.last = (p.x, p.y, th)
        self.n += 1

    def show(self):
        if self.last is None:
            self.get_logger().warning("nothing on /odom yet - is base_bridge running?")
            return
        x, y, th = self.last
        fx, fy, fth = self.first
        gap = math.hypot(x - fx, y - fy)
        print("\r  x %+6.3f  y %+6.3f  heading %+7.1f deg   "
              "path %6.3f m   turned %6.0f deg   back-to-start gap %5.3f m"
              % (x, y, math.degrees(th), self.path, math.degrees(self.turned), gap),
              end="", flush=True)

    def summary(self):
        print()
        if self.last is None or self.n < 5:
            print("  no usable odometry was received")
            return
        x, y, th = self.last
        fx, fy, fth = self.first
        gap = math.hypot(x - fx, y - fy)
        dth = math.degrees((th - fth + math.pi) % (2 * math.pi) - math.pi)
        print("  ---- odometry check ----")
        print("  messages                %d" % self.n)
        print("  path length reported    %.3f m" % self.path)
        print("  total heading change    %.0f deg" % math.degrees(self.turned))
        print("  finished at             x %+.3f  y %+.3f  heading %+.1f deg"
              % (x, y, math.degrees(th)))
        print("  gap back to the start   %.3f m" % gap)
        print("  heading error on return %+.1f deg" % dth)
        if self.path > 0.2:
            print("  closure error           %.1f%% of the path driven"
                  % (100.0 * gap / self.path))
        if self.loop > 0.2:
            print("  scale error             %+.1f%%  (reported %.2f m vs the "
                  "%.2f m you walked)"
                  % (100.0 * (self.path / self.loop - 1.0), self.path, self.loop))
            print("    -> if this is more than a couple of percent, redo the "
                  "wheel diameter step")
        if self.path > 1.0:
            err = 100.0 * gap / self.path
            if err < 3:
                print("  verdict: good enough for SLAM")
            elif err < 8:
                print("  verdict: usable, but check the gyro scale and track width")
            else:
                print("  verdict: too loose - fix odometry before mapping, or "
                      "SLAM will keep tearing the map")
        if abs(dth) > 10:
            print("  the heading did not come back either: that is the gyro "
                  "scale, not the wheels")


def main():
    rclpy.init()
    node = OdomCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        try:
            node.destroy_node()
        except (KeyboardInterrupt, Exception):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, Exception):
            pass


if __name__ == "__main__":
    main()
