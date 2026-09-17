#!/usr/bin/env python3
"""
Frontier exploration: drive to the edge of the known world, repeat.

Usage:
    ros2 run lidarbot explorer
    ros2 run lidarbot explorer --ros-args -p max_speed:=0.10 -p dry_run:=true
"""
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy,
                       QoSDurabilityPolicy)
from geometry_msgs.msg import (TwistStamped, PointStamped, Point,
                               PoseStamped)
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener

from . import planning as P


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Explorer(Node):
    def __init__(self):
        super().__init__("explorer")

        self.declare_parameter("max_speed", 0.12)     # m/s. slow is the point
        self.declare_parameter("max_turn", 0.8)       # rad/s
        self.declare_parameter("stop_dist", 0.35)     # hard stop ahead, m
        self.declare_parameter("slow_dist", 0.80)     # start easing off, m
        self.declare_parameter("clear_width", 0.18)   # robot half-width + margin, m
        self.declare_parameter("arrive", 0.40)        # target reached, m
        self.declare_parameter("min_frontier", 6)     # cells, ignore specks
        self.declare_parameter("robot_radius", 0.22)  # keep targets this clear
        self.declare_parameter("stuck_time", 12.0)    # s without progress
        self.declare_parameter("stuck_dist", 0.15)    # m that counts as progress
        self.declare_parameter("settle", 2.0)         # s pause on arrival
        self.declare_parameter("dry_run", False)      # pick targets, don't move
        self.declare_parameter("plan_step", 3)        # coarsen the grid by this
        self.declare_parameter("lookahead", 0.35)     # m along the route
        self.declare_parameter("min_speed", 0.05)     # m/s, do not crawl
        self.declare_parameter("turn_gate", 60.0)     # deg: pivot beyond this
        self.declare_parameter("escape_hold", 1.5)    # s committed to a dodge
        self.declare_parameter("hard_stop", 0.16)     # m, emergency only
        self.declare_parameter("off_route", 0.25)     # m off the path = recover
        g = lambda n: self.get_parameter(n).value
        self.v_max = float(g("max_speed"))
        self.w_max = float(g("max_turn"))
        self.stop_d = float(g("stop_dist"))
        self.slow_d = float(g("slow_dist"))
        self.lane = float(g("clear_width"))
        self.arrive = float(g("arrive"))
        self.min_fr = int(g("min_frontier"))
        self.radius = float(g("robot_radius"))
        self.stuck_t = float(g("stuck_time"))
        self.stuck_d = float(g("stuck_dist"))
        self.settle = float(g("settle"))
        self.dry = bool(g("dry_run"))
        self.pstep = max(1, int(g("plan_step")))
        self.look = float(g("lookahead"))
        self.v_min = float(g("min_speed"))
        self.gate = float(g("turn_gate"))
        self.esc_hold = float(g("escape_hold"))
        self.hard_stop = float(g("hard_stop"))
        self.off_route = float(g("off_route"))

        best = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                          history=QoSHistoryPolicy.KEEP_LAST)
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "map", self.on_map, latched)
        self.create_subscription(LaserScan, "scan", self.on_scan, best)
        self.create_subscription(Bool, "manual_active", self.on_manual, 10)
        self.pub = self.create_publisher(TwistStamped, "cmd_vel", 10)
        self.goal_pub = self.create_publisher(PointStamped, "explore/target", 10)
        self.mark_pub = self.create_publisher(Marker, "explore/marker", latched)
        self.cand_pub = self.create_publisher(Marker, "explore/candidates", latched)
        self.path_pub = self.create_publisher(Path, "explore/path", latched)
        self.cands = ([], [])

        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.grid = None
        self.scan = None
        self.manual = False
        self.manual_since = 0.0
        self.target = None          # (x, y) in the map frame
        self.banned = []            # targets we could not reach
        self.last_pose = None
        self.last_move_t = time.time()
        self.hist = []              # (t, x, y) for the stuck window
        self.arrived_t = 0.0
        self.recover = 0            # >0 = reversing out of trouble
        self.done = False
        self.empty = 0              # consecutive picks that found nothing
        self.why = "starting up"    # what it is waiting on
        self.D = None               # wavefront distance from the robot, cells
        self.Tc = None              # coarse traversable mask it was flooded on
        self.dmeta = None           # (cell size, origin x, origin y, shape)
        self.path = []              # route to the target, map frame
        self.esc_dir = 0            # committed dodge direction, +1 left
        self.esc_until = 0.0
        self.unreach = 0            # consecutive plans that found no route
        self.path_t = 0.0           # when the route was last rebuilt
        self.blocked = 0            # control ticks vetoed by the scan
        self._dgrid = None           # grid the distance field was built on
        self._dt = 0.0
        self._dpose = (0.0, 0.0)

        self.create_timer(0.1, self.control)
        self.create_timer(1.0, self.plan)
        self.create_timer(2.0, self.pick)
        self.create_timer(1.0, self.republish)   # keep the viewer fed
        self.create_timer(5.0, self.status)      # say out loud what it wants
        self.get_logger().info(
            f"explorer up: {self.v_max:.2f} m/s, stop at {self.stop_d:.2f} m. "
            f"{'DRY RUN - no motion.' if self.dry else 'Hold the pad deadman to take over.'}")

    def on_map(self, m):
        self.grid = m

    def on_scan(self, m):
        self.scan = m

    def on_manual(self, m):
        if m.data:
            self.manual_since = time.time()
        self.manual = bool(m.data)

    def pose(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint",
                                         rclpy.time.Time())
        except Exception:
            return None
        p = t.transform.translation
        return p.x, p.y, yaw_of(t.transform.rotation)

    def build_dist(self):
        """Flood known free space outward from the robot."""
        if self.grid is None:
            return False
        me = self.pose()
        if me is None:
            return False
        now = time.time()
        if (self.D is not None and self.grid is self._dgrid
                and (now - self._dt) < 0.8
                and math.hypot(me[0] - self._dpose[0],
                               me[1] - self._dpose[1]) < 0.10):
            return True
        g = self.grid
        res = g.info.resolution
        a = np.asarray(g.data, dtype=np.int16).reshape(g.info.height,
                                                      g.info.width)
        Tc = P.coarsen(P.traversable(a, res, self.radius,
                                     extra=self.scan_obstacles(me, g, a.shape)),
                       self.pstep)
        cres = res * self.pstep
        ox = g.info.origin.position.x
        oy = g.info.origin.position.y
        self.dmeta = (cres, ox, oy, Tc.shape)
        start = P.nearest_seed(Tc, self._cell(me[0], me[1]),
                              rings=max(3, int(round(0.6 / cres))))
        if start is None:
            self.D = self.Tc = None
            self.path = []
            return False
        self.D = P.wavefront(Tc, [start])
        self.Tc = Tc
        self._dgrid, self._dt, self._dpose = self.grid, now, (me[0], me[1])
        return True

    def scan_obstacles(self, me, g, shape):
        """Current scan returns, as a mask on the map grid."""
        s = self.scan
        if s is None:
            return None
        r = np.asarray(s.ranges, dtype=np.float64)
        if r.size == 0:
            return None
        ang = s.angle_min + np.arange(r.size) * s.angle_increment + me[2]
        ok = np.isfinite(r) & (r > 0.05) & (r < 5.0)
        if not ok.any():
            return None
        res = g.info.resolution
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        ii = ((me[0] + r[ok] * np.cos(ang[ok]) - ox) / res).astype(np.int64)
        jj = ((me[1] + r[ok] * np.sin(ang[ok]) - oy) / res).astype(np.int64)
        h, w = shape
        keep = (ii >= 0) & (ii < w) & (jj >= 0) & (jj < h)
        if not keep.any():
            return None
        mask = np.zeros(shape, dtype=bool)
        mask[jj[keep], ii[keep]] = True
        return mask

    def _cell(self, x, y):
        cres, ox, oy, _shape = self.dmeta
        return (int((y - oy) / cres), int((x - ox) / cres))

    def _world(self, j, i):
        cres, ox, oy, _shape = self.dmeta
        return (ox + (i + 0.5) * cres, oy + (j + 0.5) * cres)

    def _snap(self, x, y, reach=0.4):
        """Nearest planning cell to (x, y) that the wavefront reached."""
        if self.D is None:
            return None
        rings = max(2, int(round(reach / self.dmeta[0])))
        c = P.nearest_seed(self.Tc, self._cell(x, y), rings=rings, D=self.D)
        if c is None or self.D[c] >= P.INF:
            return None
        return c

    def route_dist(self, cx, cy):
        """Driving distance in metres to each candidate; inf if unreachable."""
        out = np.full(len(cx), np.inf)
        if self.D is None:
            return out
        cres = self.dmeta[0]
        for n in range(len(cx)):
            c = self._snap(float(cx[n]), float(cy[n]))
            if c is not None:
                out[n] = float(self.D[c]) * cres
        return out

    def plan(self):
        """Refresh the distance field and the route to the current target."""
        if self.done:
            return
        if not self.build_dist():
            self.path = []
            return
        if self.target is None:
            self.path = []
            self.show_path()
            return
        tc = self._snap(self.target[0], self.target[1], reach=0.5)
        if tc is None:
            self.unreach += 1
            if self.unreach >= 3:
                self.get_logger().warning(
                    "no route to the target through known space - "
                    "blacklisting it and picking another")
                self.banned.append(self.target)
                self.target = None
                self.show_target()
                self.unreach = 0
            self.path = []
            self.show_path()
            return
        self.unreach = 0
        pts = [self._world(j, i) for (j, i) in P.descend(self.D, tc)][::-1]
        if pts:
            pts.append((float(self.target[0]), float(self.target[1])))
        self.path = P.simplify(pts, 0.06)
        self.path_t = time.time()
        self.show_path()

    def show_path(self):
        m = Path()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        for px, py in self.path:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.w = 1.0
            m.poses.append(ps)
        self.path_pub.publish(m)

    def pick(self):
        """Choose the nearest worthwhile boundary between known and unknown."""
        if self.done:
            self.why = "finished - no frontiers left"
            return
        if self.grid is None:
            self.why = "no /map yet - is slam_toolbox running? (ros2 topic hz /map)"
            return
        me = self.pose()
        if me is None:
            self.why = ("no map -> base_footprint transform yet "
                        "(ros2 run tf2_ros tf2_echo map base_footprint)")
            return
        if self.target is not None and not self.stale():
            self.why = "driving to the current frontier"
            return

        g = self.grid
        w, h = g.info.width, g.info.height
        res, ox, oy = g.info.resolution, g.info.origin.position.x, \
            g.info.origin.position.y
        a = np.asarray(g.data, dtype=np.int16).reshape(h, w)

        free = (a >= 0) & (a < 25)
        unknown = a < 0
        occupied = a >= 65

        nb = np.zeros_like(unknown)
        nb[1:, :] |= unknown[:-1, :]
        nb[:-1, :] |= unknown[1:, :]
        nb[:, 1:] |= unknown[:, :-1]
        nb[:, :-1] |= unknown[:, 1:]
        frontier = free & nb

        pad = max(1, int(self.radius / res))
        grown = occupied.copy()
        for _ in range(pad):
            g2 = grown.copy()
            g2[1:, :] |= grown[:-1, :]
            g2[:-1, :] |= grown[1:, :]
            g2[:, 1:] |= grown[:, :-1]
            g2[:, :-1] |= grown[:, 1:]
            grown = g2
        frontier &= ~grown

        ys, xs = np.nonzero(frontier)
        n_free = int(free.sum())
        if len(xs) == 0:
            self.stop()
            self.cands = ([], [])
            self.empty += 1
            if n_free < 2000:
                self.why = (f"map has only {n_free} free cells - drive a "
                            f"little to seed it")
                return
            if self.empty < 5:
                self.why = f"no frontiers this pass ({self.empty}/5)"
                return
            self.done = True
            self.why = "finished"
            self.get_logger().info(
                "no frontiers left - the reachable area is fully mapped. "
                "Save it: ros2 run nav2_map_server map_saver_cli -f ~/maps/house")
            return
        self.empty = 0

        wx = ox + (xs + 0.5) * res
        wy = oy + (ys + 0.5) * res
        key = np.stack([np.round(wx / 0.3), np.round(wy / 0.3)], axis=1)
        uniq, inv, counts = np.unique(key, axis=0, return_inverse=True,
                                      return_counts=True)
        cx = np.bincount(inv, weights=wx) / counts
        cy = np.bincount(inv, weights=wy) / counts

        keep = counts >= self.min_fr
        if not keep.any():
            keep = counts == counts.max()
        cx, cy, counts = cx[keep], cy[keep], counts[keep]

        if self.D is None and not self.build_dist():
            self.why = "waiting for a map and a pose to plan on"
            return
        self.cands = (cx, cy)                   # show everything considered
        self.show_candidates(cx, cy)
        d = self.route_dist(cx, cy)
        live = np.isfinite(d)
        if not live.any():
            self.stop()
            self.why = (f"none of {len(cx)} frontiers is reachable through "
                        f"known free space - open a route by hand")
            return
        cx, cy, counts, d = cx[live], cy[live], counts[live], d[live]
        score = d - 0.25 * np.sqrt(counts)      # near, but prefer big openings
        for bx, by in self.banned:
            score[np.hypot(cx - bx, cy - by) < 0.6] = 1e6
        i = int(np.argmin(score))
        if score[i] >= 1e6:
            self.banned.clear()                 # nothing left; forgive and retry
            self.get_logger().info("all frontiers were blacklisted - retrying them")
            return

        self.why = "driving to the current frontier"
        self.target = (float(cx[i]), float(cy[i]))
        self.last_move_t = time.time()
        self.last_pose = me
        self.get_logger().info(
            f"heading for a frontier {d[i]:.1f} m away by road "
            f"({math.hypot(cx[i] - me[0], cy[i] - me[1]):.1f} m as the crow "
            f"flies, {int(counts[i])} cells, {len(cx)} reachable candidates)")
        pt = PointStamped()
        pt.header.frame_id = "map"
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x, pt.point.y = self.target
        self.goal_pub.publish(pt)
        self.show_target()
        self.plan()                             # route to it straight away

    def status(self):
        tgt = "none" if self.target is None else "(%.2f, %.2f)" % self.target
        self.get_logger().info(
            f"status: {self.why} | target {tgt} | {len(self.cands[0])} candidates"
            f" | map {'yes' if self.grid is not None else 'NO'}"
            f" | tf {'yes' if self.pose() is not None else 'NO'}"
            f" | scan {'yes' if self.scan is not None else 'NO'}"
            f" | route {len(self.path)} pts")

    def republish(self):
        """
        Markers are state, not events - send them steadily so a viewer that
        joins mid-run sees the picture straight away.
        """
        self.show_target()
        self.show_candidates(*self.cands)
        self.show_path()

    def _marker(self, mid, kind, size, rgba):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "explore"
        m.id = mid
        m.type = kind
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = size
        m.scale.z = size
        m.color.r, m.color.g, m.color.b, m.color.a = rgba
        return m

    def show_target(self):
        """Where it is going: one fat green sphere."""
        m = self._marker(0, Marker.SPHERE, 0.25, (0.2, 0.9, 0.4, 0.9))
        if self.target is None:
            m.action = Marker.DELETE
        else:
            m.pose.position.x, m.pose.position.y = self.target
            m.pose.position.z = 0.1
        self.mark_pub.publish(m)

    def show_candidates(self, cx, cy):
        """Every frontier it considered: small amber dots."""
        m = self._marker(1, Marker.POINTS, 0.08, (1.0, 0.75, 0.2, 0.8))
        for a, b in zip(cx, cy):
            p = Point()
            p.x, p.y, p.z = float(a), float(b), 0.05
            m.points.append(p)
        self.cand_pub.publish(m)

    def stale(self):
        """Has the current target been swallowed by the map, or gone bad?"""
        if self.grid is None or self.target is None:
            return True
        g = self.grid
        res = g.info.resolution
        i = int((self.target[0] - g.info.origin.position.x) / res)
        j = int((self.target[1] - g.info.origin.position.y) / res)
        if not (0 <= i < g.info.width and 0 <= j < g.info.height):
            return True
        a = np.asarray(g.data, dtype=np.int16).reshape(g.info.height,
                                                       g.info.width)
        j0, j1 = max(0, j - 2), min(g.info.height, j + 3)
        i0, i1 = max(0, i - 2), min(g.info.width, i + 3)
        return not (a[j0:j1, i0:i1] < 0).any()   # no unknown nearby any more

    def sector(self, lo, hi):
        """Closest return between two bearings, radians, robot frame."""
        s = self.scan
        if s is None:
            return float("inf")
        n = len(s.ranges)
        best = float("inf")
        for k in range(n):
            a = s.angle_min + k * s.angle_increment
            if lo <= a <= hi:
                r = s.ranges[k]
                if 0.02 < r < best:
                    best = r
        return best

    def lane_clear(self, bearing=0.0):
        """Distance to the nearest thing inside the lane we are steering down."""
        s = self.scan
        if s is None:
            return float("inf")
        best = float("inf")
        for k in range(len(s.ranges)):
            r = s.ranges[k]
            if not (0.02 < r < 6.0):
                continue
            a = s.angle_min + k * s.angle_increment - bearing
            if abs(a) > math.pi / 2:
                continue
            fwd = r * math.cos(a)
            if fwd > 0 and abs(r * math.sin(a)) < self.lane:
                best = min(best, fwd)
        return best

    def control(self):
        if self.done or self.dry:
            return
        if self.manual or (time.time() - self.manual_since) < 2.0:
            return                                    # you have the controls
        if self.target is None or self.scan is None:
            self.stop()
            return
        me = self.pose()
        if me is None:
            self.stop()
            return

        if time.time() < self.arrived_t:
            self.stop()                               # settling after arrival
            return

        now = time.time()
        x, y, th = me
        dx, dy = self.target[0] - x, self.target[1] - y
        dist = math.hypot(dx, dy)
        if dist < self.arrive or self.stale():
            self.get_logger().info("frontier reached, pausing to let the map settle")
            self.target = None
            self.show_target()
            self.arrived_t = time.time() + self.settle
            self.stop()
            return

        self.hist.append((now, x, y))
        while len(self.hist) > 1 and now - self.hist[0][0] > self.stuck_t:
            self.hist.pop(0)
        moved = (math.hypot(x - self.hist[0][1], y - self.hist[0][2])
                 if self.hist else 0.0)
        windowed = (now - self.hist[0][0]) >= self.stuck_t and moved < self.stuck_d
        if windowed and self.recover == 0:
            self.get_logger().warning("stuck - backing off and trying a different frontier")
            self.banned.append(self.target)
            self.target = None
            self.hist.clear()
            self.recover = 25                          # ~2.5 s of reversing
            return

        if self.recover > 0:
            self.recover -= 1
            self.esc_dir = 0
            self.send(-0.08, 0.6 if (self.recover // 12) % 2 == 0 else -0.6)
            self.last_move_t = time.time()
            return

        routed = bool(self.path) and (now - self.path_t) < 4.0

        if routed:
            cross, near, _ = P.track(self.path, x, y, self.look)
            look = max(0.15, self.look - 1.5 * cross)
            _, _, aim = P.track(self.path, x, y, look)
            if cross > self.off_route:
                aim = near                  # badly off: re-acquire the line
            err = math.atan2(aim[1] - y, aim[0] - x) - th
            err = (err + math.pi) % (2 * math.pi) - math.pi
            ahead = self.lane_clear(max(-0.79, min(0.79, err)))

            if abs(err) > math.radians(self.gate):
                self.send(0.0, math.copysign(self.w_max, err))
                return

            if ahead < self.hard_stop:
                self.blocked += 1
                self.stop()
                self.path = []
                if self.blocked >= 20:                 # ~2 s of no route
                    self.get_logger().warning(
                        "route is blocked by something not in the map - "
                        "dodging by hand")
                    self.blocked = 0
                    self.esc_until = 0.0
                return
            self.blocked = 0

            v = self.v_max * max(0.35, 1.0 - abs(err) / math.radians(90.0))
            if cross > self.off_route * 0.5:
                v *= 0.6                               # ease while recovering
            if ahead < self.slow_d:
                span = max(0.05, self.slow_d - self.hard_stop)
                v *= max(0.4, (ahead - self.hard_stop) / span)
            v = max(v, min(self.v_min, self.v_max))
            self.send(v, max(-self.w_max, min(self.w_max, 1.6 * err)))
            return

        aim = self.target
        err = math.atan2(aim[1] - y, aim[0] - x) - th
        err = (err + math.pi) % (2 * math.pi) - math.pi
        ahead = self.lane_clear(max(-0.79, min(0.79, err)))

        if ahead < self.stop_d:
            if self.esc_dir == 0 or now > self.esc_until:
                left = self.sector(0.35, 1.4)
                right = self.sector(-1.4, -0.35)
                if math.isinf(left) and math.isinf(right):
                    self.esc_dir = 1 if err >= 0.0 else -1
                else:
                    self.esc_dir = 1 if left > right else -1
                self.esc_until = now + self.esc_hold
            back = -0.05 if ahead < self.stop_d * 0.6 else 0.0
            self.send(back, self.w_max * self.esc_dir)
            return
        if self.esc_dir != 0 and (now > self.esc_until
                                  or ahead > self.stop_d * 1.6):
            self.esc_dir = 0
        if self.esc_dir != 0:
            self.send(0.0, self.w_max * self.esc_dir)
            return

        if abs(err) > math.radians(self.gate):
            self.send(0.0, math.copysign(self.w_max, err))
            return
        v = self.v_max * max(0.35, 1.0 - abs(err) / math.radians(90.0))
        if ahead < self.slow_d:
            span = max(0.05, self.slow_d - self.stop_d)
            v *= max(0.45, (ahead - self.stop_d) / span)
        v = max(v, min(self.v_min, self.v_max))
        self.send(v, max(-self.w_max, min(self.w_max, 1.4 * err)))

    def send(self, v, w):
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "base_link"
        m.twist.linear.x = float(v)
        m.twist.angular.z = float(w)
        self.pub.publish(m)

    def stop(self):
        self.send(0.0, 0.0)


def main():
    rclpy.init()
    node = None
    try:
        node = Explorer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
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
