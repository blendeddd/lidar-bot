#!/usr/bin/env python3
"""Unit tests for the grid planner. No ROS required."""
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "ros2", "lidarbot"))
from lidarbot import planning as P

RES = 0.03
ok = True


def check(name, cond, extra=""):
    global ok
    print(("  PASS  " if cond else "  FAIL  ") + name + ("  " + extra if extra else ""))
    ok = ok and cond


def house():
    """20 x 12 m, two rooms, one doorway 4 m down the dividing wall."""
    W, H = int(20 / RES), int(12 / RES)
    a = np.full((H, W), -1, np.int16)

    def box(x0, y0, x1, y1, v):
        a[int(y0 / RES):int(y1 / RES), int(x0 / RES):int(x1 / RES)] = v

    box(1, 1, 6, 8, 0)
    box(6.2, 1, 12, 8, 0)
    box(6.0, 1, 6.2, 8, 100)
    box(6.0, 5.0, 6.2, 5.8, 0)
    box(0.9, 0.9, 12.1, 1.0, 100)
    box(0.9, 8.0, 12.1, 8.1, 100)
    box(0.9, 0.9, 1.0, 8.1, 100)
    box(12.0, 0.9, 12.1, 8.1, 100)
    return a, W, H


a, W, H = house()
me = (3.0, 3.0)
target = (7.0, 3.0)

t0 = time.time()
T = P.traversable(a, RES, 0.22)
Tc = P.coarsen(T, 3)
cres = RES * 3
t_build = time.time() - t0

cell = lambda x, y: (int(y / cres), int(x / cres))
world = lambda j, i: ((i + 0.5) * cres, (j + 0.5) * cres)

t0 = time.time()
D = P.wavefront(Tc, [P.nearest_seed(Tc, cell(*me))])
t_wave = time.time() - t0

tc = P.nearest_seed(Tc, cell(*target), rings=8, D=D)
pts = [world(j, i) for (j, i) in P.descend(D, tc)][::-1]
path = P.simplify(pts, 0.06)

geo = float(D[tc]) * cres
euc = math.hypot(target[0] - me[0], target[1] - me[1])

print(f"grid {W}x{H} = {W * H} cells -> coarse {Tc.shape} = {Tc.size}")
print(f"traversable {t_build * 1000:.0f} ms, wavefront {t_wave * 1000:.0f} ms")
print(f"euclidean {euc:.2f} m, geodesic {geo:.2f} m, "
      f"path {len(pts)} -> {len(path)} points")

# 8-connected unit-cost BFS counts a diagonal as 1 rather than 1.414, so it
# under-reports a diagonal detour by ~10%. Fine for ranking frontiers.
check("the wall costs real distance", geo > euc * 1.3, f"{geo:.2f} vs {euc:.2f}")
check("a route exists", len(path) >= 2)

bad = [(x, y) for (x, y) in pts if a[int(y / RES), int(x / RES)] >= 65]
check("no path point is inside an obstacle", not bad, f"{len(bad)} bad")

through = [(x, y) for (x, y) in pts if 5.9 <= x <= 6.3]
check("the wall is crossed only at the doorway",
      bool(through) and all(4.8 <= y <= 6.0 for (_x, y) in through),
      f"{len(through)} crossings")

sealed = P.nearest_seed(Tc, cell(15.0, 3.0), rings=4, D=D)
check("a target outside the known area is unreachable",
      sealed is None or D[sealed] >= P.INF)

c = P.carrot(path, me[0], me[1], 0.6)
bearing = math.degrees(math.atan2(c[1] - me[1], c[0] - me[0]))
straight = math.degrees(math.atan2(target[1] - me[1], target[0] - me[0]))
check("the carrot aims along the route, not at the wall",
      abs(bearing - straight) > 15.0,
      f"{bearing:.0f} deg vs {straight:.0f} deg")

# ranking: near-through-a-wall must lose to far-but-open
for cx, cy in [(7.0, 3.0), (5.5, 7.0)]:
    cc = P.nearest_seed(Tc, cell(cx, cy), rings=8, D=D)
    g = float(D[cc]) * cres if cc is not None and D[cc] < P.INF else float("inf")
    print(f"    candidate ({cx}, {cy}): euclidean "
          f"{math.hypot(cx - me[0], cy - me[1]):.2f} m, geodesic {g:.2f} m")

print("\nALL PASS" if ok else "\nFAILURES")
sys.exit(0 if ok else 1)
