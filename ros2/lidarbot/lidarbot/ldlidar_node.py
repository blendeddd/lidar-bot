#!/usr/bin/env python3
"""LDROBOT LD19 / LD20 / D300 -> sensor_msgs/LaserScan, in pure Python.

The upstream ldlidar_stl_ros2 package is C++ and needs the full ament build
toolchain, which is a lot of machinery (and compile time on a 1 GB Pi) for what
is a very simple serial protocol. This node reuses the packet parser that has
been reading your lidar through robot_server.py for weeks, so there is nothing
new to get wrong on the hardware side.

Packet, 230400 baud 8N1:
    0x54 | VerLen | speed(2) | start_angle(2) | N x [dist(2) intensity(1)]
         | end_angle(2) | timestamp(2) | crc(1)
N = VerLen & 0x1F, angles in hundredths of a degree, distances in mm.
"""
import math
import struct

# LDROBOT CRC8: poly 0x4D, init 0, no reflection, no final xor. Verified
# against the first sixteen entries of the published table.
_CRC = []
for _i in range(256):
    _c = _i
    for _ in range(8):
        _c = ((_c << 1) ^ 0x4D) & 0xFF if _c & 0x80 else (_c << 1) & 0xFF
    _CRC.append(_c)


def crc8(data):
    c = 0
    for b in data:
        c = _CRC[(c ^ b) & 0xFF]
    return c

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


class LdLidar(Node):
    def __init__(self):
        super().__init__("ldlidar")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 230400)
        self.declare_parameter("frame_id", "base_laser")
        self.declare_parameter("topic", "scan")
        self.declare_parameter("bins", 450)          # samples per revolution
        self.declare_parameter("invert", True)       # lidar counts clockwise,
        #                                              ROS counts anticlockwise
        self.declare_parameter("angle_offset", 0.0)  # degrees, mounting rotation
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 12.0)
        self.declare_parameter("min_intensity", 0)   # drop weaker returns
        self.declare_parameter("best_effort", False) # see the QoS note below
        self.declare_parameter("use_crc", True)      # reject corrupt packets
        self.declare_parameter("rpm_min", 60.0)      # sanity window on the
        self.declare_parameter("rpm_max", 1500.0)    #   reported spin speed

        g = lambda n: self.get_parameter(n).value
        self.port_name = g("port")
        self.baud = int(g("baud"))
        self.frame = g("frame_id")
        self.bins = max(90, int(g("bins")))
        self.sign = -1.0 if g("invert") else 1.0
        self.offset = float(g("angle_offset"))
        self.rmin = float(g("range_min"))
        self.rmax = float(g("range_max"))
        self.min_int = int(g("min_intensity"))
        self.use_crc = bool(g("use_crc"))
        self.rpm_min = float(g("rpm_min"))
        self.rpm_max = float(g("rpm_max"))

        # RELIABLE, deliberately. A best-effort publisher cannot satisfy a
        # reliable subscriber, and slam_toolbox subscribes reliably - so
        # best-effort here means SLAM silently receives nothing while viewers
        # (which adapt) show the scan perfectly. A reliable publisher serves
        # both. At 6 Hz and ~10 kB a scan the cost is nothing.
        qos = QoSProfile(depth=5,
                         reliability=(QoSReliabilityPolicy.BEST_EFFORT
                                      if bool(g("best_effort"))
                                      else QoSReliabilityPolicy.RELIABLE),
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(LaserScan, g("topic"), qos)

        self.inc = 2.0 * math.pi / self.bins
        self._clear()
        self.rpm = 0.0
        self.last_pub = self.get_clock().now()
        self.revs = 0
        self.bad = 0
        self.good = 0
        self.crc_ok = 0
        self.crc_bad = 0
        self.crc_checked = False
        self.sweep = 0.0        # degrees accumulated since the last publish
        self.last_end = None    # end angle of the previous accepted packet

        self.ser = serial.Serial(self.port_name, self.baud, timeout=0.5)
        self.buf = bytearray()
        self.get_logger().info(
            f"publishing {g('topic')} as "
            f"{'BEST_EFFORT' if bool(g('best_effort')) else 'RELIABLE'}")
        self.get_logger().info(
            f"ldlidar on {self.port_name} at {self.baud}, frame '{self.frame}', "
            f"{self.bins} samples/rev, {'clockwise' if self.sign < 0 else 'anticlockwise'} "
            f"input, offset {self.offset:+.1f} deg")

        # drain the port as fast as it fills; 10 Hz of packets is light work
        self.timer = self.create_timer(0.01, self.spin_once)
        self.stats = self.create_timer(5.0, self._stats)

    # ------------------------------------------------------------------ helpers
    def _clear(self):
        self.dist = [0.0] * self.bins
        self.inten = [0.0] * self.bins

    def _stats(self):
        total = self.good + self.bad
        pct = (100.0 * self.bad / total) if total else 0.0
        sps = self.revs / 5.0
        rps = self.rpm / 60.0
        msg = (f"{sps:.1f} scans/s, {self.rpm:.0f} rpm ({rps:.1f} rev/s), "
               f"{self.bad}/{total} packets rejected ({pct:.1f}%)"
               + ("" if self.use_crc else ", CRC OFF"))
        # scans/s must equal rev/s. If it does not, revolutions are being cut
        # short or merged, and the scans going out are partial - which looks
        # like a wildly unstable lidar in a viewer.
        if rps > 0.5 and abs(sps - rps) / rps > 0.2:
            self.get_logger().warning(msg + "  <- scans/s and rev/s disagree, "
                                         "scans are being split or merged")
        elif pct > 25.0:
            self.get_logger().warning(msg + "  <- losing a lot of packets")
        else:
            self.get_logger().info(msg)
        self.revs = 0
        self.bad = 0
        self.good = 0

    def _bin(self, deg):
        a = math.radians(deg * self.sign + self.offset)
        a = (a + math.pi) % (2.0 * math.pi) - math.pi        # wrap to [-pi, pi)
        i = int((a + math.pi) / self.inc)
        return i if 0 <= i < self.bins else None

    # ------------------------------------------------------------------- parser
    def spin_once(self):
        try:
            n = self.ser.in_waiting
            if n:
                self.buf += self.ser.read(n)
        except Exception as e:                       # unplugged mid-run
            self.get_logger().error(f"serial read failed: {e}")
            return

        while True:
            i = self.buf.find(b"\x54")
            if i < 0:
                del self.buf[:]
                return
            if len(self.buf) < i + 2:
                del self.buf[:i]
                return

            verlen = self.buf[i + 1]
            count = verlen & 0x1F
            # The top three bits of VerLen identify the protocol version and are
            # always 001 on these units. Checking them rejects most stray 0x54
            # bytes before we commit to a length.
            if (verlen & 0xE0) != 0x20 or count == 0:
                del self.buf[:i + 1]        # advance ONE byte and resync
                self.bad += 1
                continue

            plen = 11 + 3 * count
            if len(self.buf) < i + plen:
                del self.buf[:i]
                return
            pkt = bytes(self.buf[i:i + plen])

            if not self._accept(pkt, count):
                # Never swallow plen bytes on a bad frame: a single stray 0x54
                # would eat the real packet behind it and the framing would
                # cascade. Step forward one byte instead.
                del self.buf[:i + 1]
                self.bad += 1
                continue

            del self.buf[:i + plen]
            self.good += 1
            self._ingest(pkt, count)

    def _accept(self, pkt, count):
        """Structural and CRC checks. Returns True if this really is a packet."""
        if self.use_crc:
            ok = crc8(pkt[:-1]) == pkt[-1]
            if not self.crc_checked:
                self.crc_ok += 1 if ok else 0
                self.crc_bad += 0 if ok else 1
                n = self.crc_ok + self.crc_bad
                if n >= 300:
                    self.crc_checked = True
                    rate = self.crc_ok / n
                    if rate < 0.2:
                        self.use_crc = False
                        self.get_logger().warning(
                            f"only {rate*100:.0f}% of packets passed CRC, so the "
                            f"check itself is wrong for this unit - disabling it "
                            f"and relying on structural checks")
                    else:
                        self.get_logger().info(
                            f"CRC check good: {rate*100:.1f}% of packets pass")
            if not ok:
                return False

        speed = struct.unpack_from("<H", pkt, 2)[0] / 6.0      # deg/s -> rpm
        if not self.rpm_min <= speed <= self.rpm_max:
            return False
        a0 = struct.unpack_from("<H", pkt, 4)[0] / 100.0
        a1 = struct.unpack_from("<H", pkt, 6 + 3 * count)[0] / 100.0
        if a0 >= 360.0 or a1 >= 360.0:
            return False
        # one packet covers roughly count x 0.8 deg; anything much wider is
        # mis-framed even if it happens to satisfy everything above
        if not 0.0 <= (a1 - a0) % 360.0 <= count * 4.0:
            return False
        return True

    def _ingest(self, pkt, count):
        self.rpm = struct.unpack_from("<H", pkt, 2)[0] / 6.0
        a0 = struct.unpack_from("<H", pkt, 4)[0] / 100.0
        a1 = struct.unpack_from("<H", pkt, 6 + 3 * count)[0] / 100.0

        span = (a1 - a0) % 360.0
        step = span / (count - 1) if count > 1 else 0.0
        for k in range(count):
            mm, q = struct.unpack_from("<HB", pkt, 6 + 3 * k)
            if mm == 0 or q < self.min_int:
                continue
            r = mm / 1000.0
            if not (self.rmin <= r <= self.rmax):
                continue
            b = self._bin((a0 + step * k) % 360.0)
            if b is not None and (self.dist[b] == 0.0 or r < self.dist[b]):
                self.dist[b] = r
                self.inten[b] = float(q)

        # Publish on accumulated sweep, not on "did the end angle wrap". One
        # corrupt packet used to look like a wrap and cut the revolution in
        # half, which is why 360 rpm was producing 12 scans a second.
        if self.last_end is not None:
            adv = (a1 - self.last_end) % 360.0
            if adv < 45.0:                    # ignore an implausible jump
                self.sweep += adv
        self.last_end = a1
        if self.sweep >= 358.0:
            self.sweep = 0.0
            self.publish()

    # ---------------------------------------------------------------- publisher
    def publish(self):
        now = self.get_clock().now()
        scan_time = (now - self.last_pub).nanoseconds / 1e9
        self.last_pub = now
        if not 0.02 < scan_time < 1.0:
            scan_time = 0.1
        self.revs += 1

        m = LaserScan()
        # stamp the START of the sweep - stamping 'now' asks TF to look into the
        # future and slam_toolbox drops the scan
        m.header.stamp = (now - rclpy.duration.Duration(
            seconds=scan_time)).to_msg()
        m.header.frame_id = self.frame
        m.angle_min = -math.pi
        m.angle_max = math.pi - self.inc
        m.angle_increment = self.inc
        m.scan_time = scan_time
        m.time_increment = scan_time / self.bins
        m.range_min = self.rmin
        m.range_max = self.rmax
        m.ranges = [d if d > 0.0 else float("inf") for d in self.dist]
        m.intensities = list(self.inten)
        self.pub.publish(m)
        self._clear()


def main():
    rclpy.init()
    node = None
    try:
        node = LdLidar()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.ser.close()
            except Exception:
                pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
