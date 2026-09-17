#!/usr/bin/env python3
"""
Closed-loop test of the explorer against a synthetic house. No ROS required.

Stubs rclpy so explorer.py imports unchanged, then closes the loop with a
diff-drive model and a raycast LiDAR. Measures the two failure modes that
matter: does it aim through walls, and does it shuffle instead of advancing.

Usage:
    python3 sim_explore.py
    DOORW=0.76 python3 sim_explore.py
    DOORW=0.76 TH0=180 python3 sim_explore.py
    DOORW=0.76 GHOST=4.4,3.8,5.0,4.4 python3 sim_explore.py
    DOORW=0.0 EXPECT=unreachable python3 sim_explore.py
"""
import math
import os
import sys
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "..", "ros2", "lidarbot")

RES = 0.05
DOORW = float(os.environ.get("DOORW", "0.9"))
DOOR0 = float(os.environ.get("DOOR0", "4.6"))
TH0 = float(os.environ.get("TH0", "0.0"))
GHOST = os.environ.get("GHOST", "")
EXPECT = os.environ.get("EXPECT", "arrive")
TRACE = int(os.environ.get("TRACE", "0"))


def cls(name, **fields):
    def init(self):
        for k, v in fields.items():
            setattr(self, k, v() if callable(v) else v)
    return type(name, (), {"__init__": init})


XYZ = lambda: types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
QUAT = lambda: types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)
HDR = lambda: types.SimpleNamespace(stamp=None, frame_id="")
POSE = lambda: types.SimpleNamespace(position=XYZ(), orientation=QUAT())
TW = lambda: types.SimpleNamespace(linear=XYZ(), angular=XYZ())

mods = {}


def mod(name, **attrs):
    m = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(m, k, v)
    mods[name] = m
    return m


class Logger:
    def info(self, m): print("   [info]", m)
    def warning(self, m): print("   [warn]", m)
    def error(self, m): print("   [err ]", m)


class Node:
    def __init__(self, name):
        self._p, self.timers = {}, []

    def declare_parameter(self, n, v): self._p[n] = v
    def get_parameter(self, n): return types.SimpleNamespace(value=self._p[n])
    def create_timer(self, period, cb): self.timers.append((period, cb))
    def create_subscription(self, *a, **k): return None

    def create_publisher(self, *a, **k):
        return types.SimpleNamespace(publish=lambda m: None)

    def get_logger(self): return Logger()

    def get_clock(self):
        return types.SimpleNamespace(
            now=lambda: types.SimpleNamespace(to_msg=lambda: None))

    def destroy_node(self): pass


mod("rclpy", init=lambda *a, **k: None, ok=lambda: True,
    spin=lambda n: None, shutdown=lambda: None)
mod("rclpy.node", Node=Node)
mod("rclpy.time", Time=lambda *a, **k: None)
mod("rclpy.qos",
    QoSProfile=lambda **k: None,
    QoSReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2),
    QoSHistoryPolicy=types.SimpleNamespace(KEEP_LAST=1),
    QoSDurabilityPolicy=types.SimpleNamespace(TRANSIENT_LOCAL=1))
for sub in ("node", "time", "qos"):
    setattr(mods["rclpy"], sub, mods["rclpy." + sub])

Marker = cls("Marker", header=HDR, ns="", id=0, type=0, action=0, pose=POSE,
             scale=XYZ,
             color=lambda: types.SimpleNamespace(r=0.0, g=0.0, b=0.0, a=0.0),
             points=list)
for i, n in enumerate(["ARROW", "CUBE", "SPHERE", "CYLINDER", "LINE_STRIP",
                       "LINE_LIST", "CUBE_LIST", "SPHERE_LIST", "POINTS"]):
    setattr(Marker, n, i)
Marker.ADD, Marker.DELETE = 0, 2

mod("geometry_msgs", msg=None)
mod("geometry_msgs.msg",
    TwistStamped=cls("TwistStamped", header=HDR, twist=TW),
    PointStamped=cls("PointStamped", header=HDR, point=XYZ),
    Point=cls("Point", x=0.0, y=0.0, z=0.0),
    PoseStamped=cls("PoseStamped", header=HDR, pose=POSE))
mod("visualization_msgs", msg=None)
mod("visualization_msgs.msg", Marker=Marker)
mod("nav_msgs", msg=None)
mod("nav_msgs.msg",
    OccupancyGrid=cls("OccupancyGrid", header=HDR, info=None, data=list),
    Path=cls("Path", header=HDR, poses=list))
mod("sensor_msgs", msg=None)
mod("sensor_msgs.msg",
    LaserScan=cls("LaserScan", header=HDR, ranges=list, angle_min=0.0,
                  angle_increment=0.0))
mod("std_msgs", msg=None)
mod("std_msgs.msg", Bool=cls("Bool", data=False))
mod("tf2_ros",
    Buffer=lambda: types.SimpleNamespace(lookup_transform=lambda *a, **k: None),
    TransformListener=lambda *a, **k: None)
for pkg in ("geometry_msgs", "visualization_msgs", "nav_msgs", "sensor_msgs",
            "std_msgs"):
    setattr(mods[pkg], "msg", mods[pkg + ".msg"])
sys.modules.update(mods)

sys.path.insert(0, PKG)
import lidarbot.explorer as EX
from lidarbot.explorer import Explorer
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class Clock:
    """The sim runs 1200 ticks in a fraction of a second of wall clock, so the
    real time.time() would leave escape_hold and the stuck window never firing."""
    t = 1000.0

    def time(self): return Clock.t


EX.time = Clock()


def house():
    W, H = int(14 / RES), int(10 / RES)
    a = np.full((H, W), -1, np.int16)

    def box(x0, y0, x1, y1, v):
        a[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = v

    box(1, 1, 6, 8, 0)
    box(6.2, 1, 11, 8, 0)
    box(6.0, 1, 6.2, 8, 100)
    box(6.0, DOOR0, 6.2, DOOR0 + DOORW, 0)
    box(0.9, 0.9, 11.1, 1.0, 100)
    box(0.9, 8.0, 11.1, 8.1, 100)
    box(0.9, 0.9, 1.0, 8.1, 100)
    box(11.0, 0.9, 11.1, 8.1, 100)
    box(10.4, 1.1, 11.0, 7.9, -1)          # the unexplored strip
    return a, W, H


A, W, H = house()
solid = A >= 65
if GHOST:
    gx0, gy0, gx1, gy1 = [float(v) for v in GHOST.split(",")]
    solid[int(gy0 / RES):int(gy1 / RES), int(gx0 / RES):int(gx1 / RES)] = True


def raycast(x, y, th, n=450, rmax=12.0):
    out = []
    for k in range(n):
        ang = -math.pi + k * (2 * math.pi / n) + th
        dx, dy = math.cos(ang), math.sin(ang)
        r, hit = 0.0, float("inf")
        while r < rmax:
            r += RES * 0.7
            i, j = int((x + dx * r) / RES), int((y + dy * r) / RES)
            if not (0 <= i < W and 0 <= j < H) or solid[j, i]:
                hit = r
                break
        out.append(hit)
    return out


def grid_msg():
    g = OccupancyGrid()
    g.info = types.SimpleNamespace(
        resolution=RES, width=W, height=H,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=0.0, y=0.0)))
    g.data = A.reshape(-1).tolist()
    return g


e = Explorer()
e._p.update(max_speed=0.12, max_turn=0.8, robot_radius=0.22, plan_step=3,
            lookahead=0.35, min_speed=0.05, turn_gate=60.0, escape_hold=1.5)
e.__init__()

state = {"x": 3.0, "y": 3.0, "th": math.radians(TH0)}
e.pose = lambda: (state["x"], state["y"], state["th"])
e.grid = grid_msg()

cmds, traj = [], []


def send(v, w):
    cmds.append((v, w))
    dt = 0.1
    Clock.t += dt
    state["th"] += w * dt
    state["x"] += v * dt * math.cos(state["th"])
    state["y"] += v * dt * math.sin(state["th"])
    traj.append((state["x"], state["y"]))


e.send = send


def refresh_scan():
    s = LaserScan()
    s.angle_min = -math.pi
    s.angle_increment = 2 * math.pi / 450
    s.ranges = raycast(state["x"], state["y"], state["th"])
    e.scan = s


refresh_scan()
e.pick()
tgt = e.target
print(f"\ntarget {tgt}, route {len(e.path)} waypoints")

for step in range(1200):
    refresh_scan()
    if step % 10 == 0:
        e.plan()
    if step % 20 == 0:
        e.pick()
    e.control()
    if e.done:
        break

xs = [p[0] for p in traj] or [state["x"]]
fwd = [v for v, _w in cmds]
spin = sum(1 for v, w in cmds if abs(v) < 1e-6 and abs(w) > 1e-6)
rev, prev = 0, 0
for v, w in cmds:
    if abs(v) < 1e-6 and abs(w) > 1e-6:
        sgn = 1 if w > 0 else -1
        rev += 1 if prev and sgn != prev else 0
        prev = sgn
    else:
        prev = 0
hit = any(solid[int(y / RES), int(x / RES)] for x, y in traj
          if 0 <= int(x / RES) < W and 0 <= int(y / RES) < H)

print(f"ticks {len(cmds)} | mean speed {np.mean(fwd) if fwd else 0:.3f} m/s | "
      f"turn-in-place {spin} ({100 * spin // max(1, len(cmds))}%)")
print(f"reversals while stationary: {rev}")
print(f"final ({state['x']:.2f}, {state['y']:.2f}), max x {max(xs):.2f}")

ok = True


def check(name, cond, extra=""):
    global ok
    print(("  PASS  " if cond else "  FAIL  ") + name + ("  " + extra if extra else ""))
    ok = ok and cond


if EXPECT == "unreachable":
    check("no target is chosen when none is reachable", tgt is None)
    check("it says why instead of grinding at a wall", "reachable" in e.why, e.why)
    check("it never moved", abs(np.mean(fwd)) < 1e-6 if fwd else True)
else:
    check("a target was chosen", tgt is not None)
    check("it reached the right-hand room", max(xs) > 6.5, f"max x {max(xs):.2f}")
    check("it got most of the way to the target",
          tgt is not None and math.hypot(state["x"] - tgt[0],
                                         state["y"] - tgt[1]) < 1.6)
    check("it did not drive through a wall", not hit)
    check("it mostly moves rather than pivoting", spin < 0.45 * len(cmds),
          f"{100 * spin // max(1, len(cmds))}% pivoting")
    check("it does not rock left-right on the spot",
          rev <= (14 if GHOST else 6), f"{rev} reversals")
    check("mean speed is a real speed", np.mean(fwd) > 0.03 if fwd else False)

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
