#!/usr/bin/env python3
"""USB webcam -> sensor_msgs/CompressedImage, so Foxglove can show it.

robot_server.py owns both serial ports and cannot run while ROS does, so its
MJPEG stream is unavailable during mapping. This publishes JPEG frames instead,
which the Foxglove bridge forwards only to clients that actually have an Image
panel open - so it costs no bandwidth until you look at it.

The capture-open logic is the same one that eventually worked on this Pi: force
the V4L2 backend, and prove a node really yields a frame before accepting it,
because a UVC webcam exposes a metadata node alongside the capture node and the
metadata one opens happily and never returns an image.

    ros2 run lidarbot camera_node
    ros2 run lidarbot camera_node --ros-args -p index:=-1 -p fps:=5.0
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage


class Camera(Node):
    def __init__(self):
        super().__init__("camera")
        import cv2
        self.cv2 = cv2

        self.declare_parameter("index", -1)        # -1 = probe every node
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 10.0)        # publish rate
        self.declare_parameter("quality", 50)      # JPEG, 10-95
        self.declare_parameter("flip", False)      # 180 deg, if mounted upside down
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("topic", "camera/image_raw/compressed")
        g = lambda n: self.get_parameter(n).value
        self.index = int(g("index"))
        self.w, self.h = int(g("width")), int(g("height"))
        self.fps = max(1.0, float(g("fps")))
        self.quality = int(g("quality"))
        self.flip = bool(g("flip"))
        self.frame = g("frame_id")

        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.pub = self.create_publisher(CompressedImage, g("topic"), qos)

        self.cap = None
        self.dev = "-"
        self.sent = 0
        self.bytes = 0
        self.create_timer(1.0 / self.fps, self.tick)
        self.create_timer(5.0, self.stats)

    def open(self):
        cv2 = self.cv2
        order = ([self.index] if self.index >= 0 else []) + list(range(0, 10))
        for i in order:
            for src in (i, f"/dev/video{i}"):
                try:
                    cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
                except Exception:
                    continue
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.w)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.h)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                ok, _ = cap.read()
                if ok:
                    self.dev = f"/dev/video{i}"
                    self.get_logger().info(
                        f"camera on {self.dev}, {self.w}x{self.h}, "
                        f"{self.fps:.0f} fps, quality {self.quality}")
                    return cap
                cap.release()
        return None

    def tick(self):
        if self.cap is None:
            self.cap = self.open()
            if self.cap is None:
                self.get_logger().warning(
                    "no working camera node (is the user in the 'video' group?)",
                    throttle_duration_sec=10.0)
                return
        ok, img = self.cap.read()
        if not ok:
            self.get_logger().warning("camera read failed, reopening")
            self.cap.release()
            self.cap = None
            return
        if self.flip:
            img = self.cv2.flip(img, -1)
        ok, buf = self.cv2.imencode(
            ".jpg", img, [int(self.cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return
        m = CompressedImage()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.format = "jpeg"
        m.data = buf.tobytes()
        self.pub.publish(m)
        self.sent += 1
        self.bytes += len(m.data)

    def stats(self):
        if self.sent:
            self.get_logger().info(
                f"{self.sent / 5.0:.1f} fps, {self.bytes / 5120.0:.0f} kB/s "
                f"from {self.dev}")
        self.sent = self.bytes = 0


def main():
    rclpy.init()
    node = None
    try:
        node = Camera()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ImportError:
        print("camera_node needs OpenCV: sudo apt install -y python3-opencv")
    finally:
        if node is not None:
            try:
                if node.cap is not None:
                    node.cap.release()
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
