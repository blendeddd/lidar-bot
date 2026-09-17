#!/usr/bin/env python3
"""
GameSir pad -> /cmd_vel (TwistStamped), read straight off evdev.

Usage:
    ros2 run lidarbot pad_teleop
    ros2 run lidarbot pad_teleop --ros-args -p max_speed:=0.3 -p deadman:=false
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool


class PadTeleop(Node):
    def __init__(self):
        super().__init__("pad_teleop")
        from evdev import InputDevice, ecodes, list_devices
        self.ecodes = ecodes

        self.declare_parameter("max_speed", 0.25)      # m/s at full trigger
        self.declare_parameter("max_turn", 1.2)        # rad/s at full stick
        self.declare_parameter("boost", 2.0)           # multiplier while RB held
        self.declare_parameter("steer_invert", False)
        self.declare_parameter("deadman", True)        # require LB to move
        self.declare_parameter("rate", 20.0)
        g = lambda n: self.get_parameter(n).value
        self.max_v = float(g("max_speed"))
        self.max_w = float(g("max_turn"))
        self.boost = float(g("boost"))
        self.inv = -1.0 if bool(g("steer_invert")) else 1.0
        self.deadman = bool(g("deadman"))

        self.dev = None
        for path in list_devices():
            d = InputDevice(path)
            caps = d.capabilities()
            if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps and \
               ecodes.ABS_X in [a for a, _ in caps[ecodes.EV_ABS]]:
                self.dev = d
                break
        if self.dev is None:
            raise RuntimeError(
                "no gamepad found. Check it is paired and that you are in the "
                "'input' group (sudo usermod -aG input $USER, then log out)")

        info = {c: i for c, i in self.dev.capabilities()[ecodes.EV_ABS]}
        have = list(info)
        self.ax_rt = ecodes.ABS_GAS if ecodes.ABS_GAS in have else ecodes.ABS_RZ
        self.ax_lt = ecodes.ABS_BRAKE if ecodes.ABS_BRAKE in have else ecodes.ABS_Z
        self.info = info
        self.vals = {ecodes.ABS_X: 0, self.ax_rt: 0, self.ax_lt: 0}
        self.held = set()
        self.dev.grab()

        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.man_pub = self.create_publisher(Bool, "manual_active", 10)
        self.create_timer(1.0 / max(1.0, float(g("rate"))), self.tick)
        self.get_logger().info(
            f"pad: {self.dev.name} | throttle RT/LT, steer left stick, "
            f"{'hold LB to drive' if self.deadman else 'deadman OFF'} | "
            f"{self.max_v:.2f} m/s, {math.degrees(self.max_w):.0f} deg/s")

    def stick(self, raw):
        i = self.info[self.ecodes.ABS_X]
        mid, half = (i.max + i.min) / 2.0, ((i.max - i.min) / 2.0 or 1)
        v = (raw - mid) / half
        if abs(v) < 0.12:
            return 0.0
        v = (abs(v) - 0.12) / 0.88 * (1 if v > 0 else -1)
        return v * self.inv

    def trig(self, code, raw):
        i = self.info[code]
        v = (raw - i.min) / ((i.max - i.min) or 1)
        return 0.0 if v < 0.06 else (v - 0.06) / 0.94

    def tick(self):
        ec = self.ecodes
        try:
            for ev in self.dev.read():
                if ev.type == ec.EV_ABS and ev.code in self.vals:
                    self.vals[ev.code] = ev.value
                elif ev.type == ec.EV_KEY:
                    (self.held.add if ev.value else self.held.discard)(ev.code)
        except BlockingIOError:
            pass
        except OSError:
            self.get_logger().error("pad disconnected")
            self.send(0.0, 0.0)
            return

        live = (not self.deadman) or (ec.BTN_TL in self.held)
        if ec.BTN_EAST in self.held:            # B = stop
            live = False
        b = Bool()
        b.data = bool(live)
        self.man_pub.publish(b)
        if not live:
            return

        scale = self.boost if ec.BTN_TR in self.held else 1.0
        thr = (self.trig(self.ax_rt, self.vals[self.ax_rt])
               - self.trig(self.ax_lt, self.vals[self.ax_lt]))
        steer = self.stick(self.vals[ec.ABS_X])
        self.send(thr * self.max_v * scale, -steer * self.max_w * scale)

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.pub.publish(m)


def main():
    rclpy.init()
    node = None
    try:
        node = PadTeleop()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as e:
        if isinstance(e, RuntimeError):
            print("pad_teleop: %s" % e)
    finally:
        if node is not None:
            try:
                node.send(0.0, 0.0)
                node.dev.ungrab()
            except Exception:
                pass
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
