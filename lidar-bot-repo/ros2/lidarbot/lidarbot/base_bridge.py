#!/usr/bin/env python3
"""lidar-bot base bridge."""
import json, math, threading, time
import serial

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TwistStamped, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, BatteryState
from tf2_ros import TransformBroadcaster


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class BaseBridge(Node):
    def __init__(self):
        super().__init__("base_bridge")

        self.declare_parameter("port", "/dev/ttyAMA0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("track_width", 0.21)      # metres, effective
        self.declare_parameter("wheel_diameter", 0.070)
        self.declare_parameter("pulses_per_rev", 414.0)
        self.declare_parameter("cmd_scale", 1.0)         # m/s -> command units
        self.declare_parameter("cmd_max", 1.0)           # clamp on each wheel
        self.declare_parameter("use_gyro_yaw", True)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("watchdog", 0.6)          # stop if no cmd_vel
        self.declare_parameter("gyro_bias", 0.0)         # deg/s, from calibrate.py drift
        self.declare_parameter("gyro_scale", 1.0)        # from calibrate.py spin
        self.declare_parameter("gyro_invert", False)

        self.declare_parameter("closed_loop", True)
        self.declare_parameter("hold_kp", 2.0)     # deg/s of correction per deg
        self.declare_parameter("hold_ki", 1.2)
        self.declare_parameter("hold_kd", 0.25)
        self.declare_parameter("hold_max", 70.0)   # deg/s
        self.declare_parameter("rate_kp", 0.6)     # deg/s of trim per deg/s
        self.declare_parameter("rate_ki", 1.5)
        self.declare_parameter("rate_max", 90.0)   # deg/s

        g = lambda n: self.get_parameter(n).value
        self.track = float(g("track_width"))
        self.cmd_scale = float(g("cmd_scale"))
        self.cmd_max = float(g("cmd_max"))
        self.use_gyro = bool(g("use_gyro_yaw"))
        self.pub_tf = bool(g("publish_tf"))
        self.odom_frame = g("odom_frame")
        self.base_frame = g("base_frame")
        self.imu_frame = g("imu_frame")
        self.watchdog = float(g("watchdog"))
        self.gyro_bias = float(g("gyro_bias"))
        self.gyro_scale = float(g("gyro_scale"))
        self.gyro_sign = -1.0 if bool(g("gyro_invert")) else 1.0
        self.closed_loop = bool(g("closed_loop"))
        self.hold_kp = float(g("hold_kp"))
        self.hold_ki = float(g("hold_ki"))
        self.hold_kd = float(g("hold_kd"))
        self.hold_max = float(g("hold_max"))
        self.rate_kp = float(g("rate_kp"))
        self.rate_ki = float(g("rate_ki"))
        self.rate_max = float(g("rate_max"))
        self.gz_now = 0.0        # latest measured yaw rate, deg/s
        self.hold_target = None  # heading being held, radians
        self.hold_i = 0.0
        self.rate_i = 0.0
        self.ctrl_t = 0.0
        self.warned_no_gz = False

        self.m_per_pulse = (math.pi * float(g("wheel_diameter"))) / float(g("pulses_per_rev"))

        self.x = self.y = self.th = 0.0
        self.prev_l = self.prev_r = None
        self.prev_stamp = None
        self.last_cmd = 0.0
        self.lock = threading.Lock()

        self.ser = serial.Serial(g("port"), int(g("baud")), timeout=0.3)
        self.wlock = threading.Lock()
        time.sleep(0.3)
        self.ser.reset_input_buffer()

        self.read_geometry_from_board()

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.odom_pub = self.create_publisher(Odometry, "odom", qos)
        self.imu_pub = self.create_publisher(Imu, "imu", qos)
        self.bat_pub = self.create_publisher(BatteryState, "battery_state", qos)
        self.tfb = TransformBroadcaster(self)

        self.create_subscription(TwistStamped, "cmd_vel", self.on_twist_stamped, qos)
        self.create_subscription(Twist, "cmd_vel_unstamped", self.on_twist, qos)

        self.running = True
        self.rx = threading.Thread(target=self.serial_loop, daemon=True)
        self.rx.start()
        self.create_timer(0.1, self.check_watchdog)

        self.get_logger().info(
            f"bridge up: {self.m_per_pulse*1000:.4f} mm/pulse, track {self.track:.3f} m, "
            f"{'gyro' if self.use_gyro else 'wheel'} heading"
            f"{', gyro sign INVERTED' if self.gyro_sign < 0 else ''}"
            f"{', closed-loop heading ON' if self.closed_loop else ''}")

    def read_geometry_from_board(self):
        """ask the ESP32 for its saved geometry so there is one source of truth"""
        try:
            self.send({"T": 701})
            end = time.time() + 1.5
            while time.time() < end:
                line = self.ser.readline().decode(errors="ignore").strip()
                try:
                    d = json.loads(line)
                except ValueError:
                    continue
                if "ppr" in d and "wd" in d:
                    self.m_per_pulse = (math.pi * float(d["wd"])) / float(d["ppr"])
                    self.track = float(d["tw"])
                    self.get_logger().info(
                        f"geometry from board: wheel {d['wd']:.4f} m, ppr {d['ppr']}, track {d['tw']:.3f} m")
                    return
        except Exception as e:
            self.get_logger().warning(f"could not read geometry from board ({e}); using parameters")

    def send(self, obj):
        try:
            with self.wlock:
                self.ser.write((json.dumps(obj) + "\n").encode())
                self.ser.flush()
        except Exception as e:
            self.get_logger().warning(f"serial write failed: {e}")

    def trim(self, v, w):
        """Extra wheel differential, m/s, so the robot actually turns at w."""
        if not self.closed_loop:
            return 0.0
        now = time.time()
        dt = (now - self.ctrl_t) if self.ctrl_t else 0.05
        self.ctrl_t = now
        dt = min(max(dt, 0.005), 0.2)

        want = math.degrees(w)
        turning = abs(want) > 3.0
        moving = abs(v) > 0.03

        if turning:
            self.hold_i = 0.0
            self.hold_target = self.th
            err = want - self.gz_now
            self.rate_i += err * dt
            lim = self.rate_max / max(self.rate_ki, 1e-6)
            self.rate_i = max(-lim, min(lim, self.rate_i))
            out = self.rate_kp * err + self.rate_ki * self.rate_i
        elif moving:
            self.rate_i = 0.0
            if self.hold_target is None:
                self.hold_target = self.th
            e = math.degrees((self.hold_target - self.th + math.pi)
                             % (2 * math.pi) - math.pi)
            if abs(e) > 60.0:                 # something is badly wrong
                self.hold_target = self.th
                self.hold_i = 0.0
                e = 0.0
            self.hold_i += e * dt
            lim = self.hold_max / max(self.hold_ki, 1e-6)
            self.hold_i = max(-lim, min(lim, self.hold_i))
            out = (self.hold_kp * e + self.hold_ki * self.hold_i
                   - self.hold_kd * self.gz_now)
        else:
            self.hold_target = self.th
            self.hold_i = self.rate_i = 0.0
            return 0.0

        cap = self.rate_max if turning else self.hold_max
        out = max(-cap, min(cap, out))
        return math.radians(out) * self.track / 2.0

    def drive(self, v, w):
        corr = self.trim(v, w)
        l = (v - w * self.track / 2.0 - corr) * self.cmd_scale
        r = (v + w * self.track / 2.0 + corr) * self.cmd_scale
        peak = max(abs(l), abs(r))
        if peak > self.cmd_max:
            l, r = l * self.cmd_max / peak, r * self.cmd_max / peak
        self.send({"T": 1, "L": round(l, 3), "R": round(r, 3)})
        self.last_cmd = time.time()

    def on_twist_stamped(self, msg):
        self.drive(msg.twist.linear.x, msg.twist.angular.z)

    def on_twist(self, msg):
        self.drive(msg.linear.x, msg.angular.z)

    def check_watchdog(self):
        if self.last_cmd and (time.time() - self.last_cmd) > self.watchdog:
            self.send({"T": 1, "L": 0, "R": 0})
            self.last_cmd = 0.0

    def serial_loop(self):
        while self.running and rclpy.ok():
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue
                d = json.loads(line)
            except Exception:
                continue
            if "odl" not in d:
                continue
            if not (self.running and rclpy.ok()):
                break
            try:
                self.handle_feedback(d)
            except Exception as e:
                if self.running:
                    self.get_logger().warning(f"feedback error: {e}")

    def handle_feedback(self, d):
        now = self.get_clock().now()
        stamp = now.to_msg()

        l_cnt, r_cnt = float(d["odl"]), float(d["odr"])
        if self.prev_l is None:
            self.prev_l, self.prev_r, self.prev_stamp = l_cnt, r_cnt, now
            return

        dt = (now - self.prev_stamp).nanoseconds / 1e9
        if dt <= 0.0:
            return

        dl = (l_cnt - self.prev_l) * self.m_per_pulse
        dr = (r_cnt - self.prev_r) * self.m_per_pulse
        self.prev_l, self.prev_r, self.prev_stamp = l_cnt, r_cnt, now

        dist = (dl + dr) / 2.0

        has_gz = "gz" in d
        gz = ((float(d.get("gz", 0.0)) - self.gyro_bias)
              * self.gyro_scale * self.gyro_sign)                           # deg/s

        self.gz_now = gz
        if self.use_gyro and has_gz:
            dth = math.radians(gz) * dt
        else:
            if self.use_gyro and not self.warned_no_gz:
                self.warned_no_gz = True
                self.get_logger().warning(
                    "no 'gz' in telemetry - falling back to wheel heading. "
                    "Flash the firmware that adds gz to the feedback stream; "
                    "wheel heading drifts badly on a skid-steer.")
            dth = (dr - dl) / self.track

        self.th += dth
        self.x += dist * math.cos(self.th - dth / 2.0)
        self.y += dist * math.sin(self.th - dth / 2.0)
        self.th = math.atan2(math.sin(self.th), math.cos(self.th))

        v = dist / dt
        w = dth / dt

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = yaw_to_quat(self.th)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.pose.covariance[0] = 0.02
        odom.pose.covariance[7] = 0.02
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[35] = 0.05
        self.odom_pub.publish(odom)

        if self.pub_tf:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.rotation = yaw_to_quat(self.th)
            self.tfb.sendTransform(t)

        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = self.imu_frame
        imu.orientation = yaw_to_quat(math.radians(float(d.get("y", 0.0))))
        imu.angular_velocity.z = math.radians(gz)
        imu.linear_acceleration.x = float(d.get("ax", 0.0))
        imu.linear_acceleration.y = float(d.get("ay", 0.0))
        imu.linear_acceleration.z = float(d.get("az", 0.0))
        self.imu_pub.publish(imu)

        if "pv" in d:
            b = BatteryState()
            b.header.stamp = stamp
            b.voltage = float(d["pv"])
            b.current = float(d.get("pa", 0.0)) / 1000.0
            b.percentage = float(d.get("pct", -1)) / 100.0
            b.present = True
            self.bat_pub.publish(b)


def main():
    rclpy.init()
    node = BaseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        try:
            node.rx.join(timeout=1.0)
        except Exception:
            pass
        node.send({"T": 1, "L": 0, "R": 0})
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
