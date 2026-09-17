#!/usr/bin/env python3
"""
Grid planning helpers for the explorer.

Pure numpy, no ROS - so it can be unit tested off the robot.

The explorer used to score frontiers by straight-line distance, which happily
picked a target two metres away through a wall over one four metres away down
the hall, then drove at it until the lane check stopped it dead.  Everything
here exists to replace "how far away is it" with "how far must I actually
drive to get there", and to hand back a path that goes round the wall.
"""
import math
import numpy as np

INF = np.int32(1 << 29)

# 8-connected, so paths cut corners instead of going in staircases
_SHIFTS = ((-1, 0), (1, 0), (0, -1), (0, 1),
           (-1, -1), (-1, 1), (1, -1), (1, 1))


def classify(a, free_max=25, occ_min=65):
    """OccupancyGrid data -> (free, unknown, occupied) boolean arrays."""
    free = (a >= 0) & (a < free_max)
    unknown = a < 0
    occupied = a >= occ_min
    return free, unknown, occupied


def grow(mask, cells):
    """Dilate a boolean mask by `cells` in the 4-connected sense."""
    out = mask.copy()
    for _ in range(max(0, int(cells))):
        nxt = out.copy()
        nxt[1:, :] |= out[:-1, :]
        nxt[:-1, :] |= out[1:, :]
        nxt[:, 1:] |= out[:, :-1]
        nxt[:, :-1] |= out[:, 1:]
        out = nxt
    return out


def traversable(a, res, radius, free_max=25, occ_min=65, extra=None):
    """Cells the robot's centre may occupy: known free, and far enough from
    anything solid that its body clears.  Unknown is NOT traversable - we do
    not plan through space we have never seen.

    `extra` is an optional mask of things the map does not contain - live scan
    hits, typically.  Without it the planner keeps routing straight through a
    bag on the floor, the controller vetoes it a metre later, and the two
    argue; with it the route simply goes round.
    """
    free, _unknown, occupied = classify(a, free_max, occ_min)
    if extra is not None:
        occupied = occupied | extra
    pad = int(math.ceil(radius / res))
    return free & ~grow(occupied, pad)


def coarsen(T, k):
    """Block-reduce a traversable mask by k, keeping only blocks that are
    entirely traversable.  Planning on 3 cm cells across a house is a lot of
    iterations for a Pi; 9 cm is plenty and stays conservative."""
    k = max(1, int(k))
    if k == 1:
        return T.copy()
    h, w = T.shape
    hh, ww = h // k, w // k
    if hh == 0 or ww == 0:
        return T.copy()
    return T[:hh * k, :ww * k].reshape(hh, k, ww, k).all(axis=(1, 3))


def wavefront(T, seeds, max_iter=100000):
    """Breadth-first distance, in cells, from `seeds` across `T`.

    Vectorised: each iteration dilates the frontier once with numpy rather
    than touching a Python-level queue, so cost is (longest path in cells) x
    (a handful of whole-array ops) instead of one step per cell.
    """
    D = np.full(T.shape, INF, dtype=np.int32)
    cur = np.zeros(T.shape, dtype=bool)
    h, w = T.shape
    for (j, i) in seeds:
        if 0 <= j < h and 0 <= i < w and T[j, i]:
            cur[j, i] = True
    if not cur.any():
        return D
    D[cur] = 0
    d = 0
    while cur.any() and d < max_iter:
        d += 1
        nxt = np.zeros_like(cur)
        for dj, di in _SHIFTS:
            js, je = max(0, dj), h + min(0, dj)
            is_, ie = max(0, di), w + min(0, di)
            nxt[js:je, is_:ie] |= cur[js - dj:je - dj, is_ - di:ie - di]
        nxt &= T & (D == INF)
        if not nxt.any():
            break
        D[nxt] = d
        cur = nxt
    return D


def nearest_seed(T, cell, rings=6, D=None):
    """Snap a cell onto the traversable set (and, if D is given, onto a cell
    the wavefront actually reached).  The robot's own centre is often inside
    the inflated obstacle layer when it is close to a wall, and a frontier
    cell can be shaved off by coarsening - without this, planning gives up
    exactly when it is needed most."""
    h, w = T.shape
    j0, i0 = cell

    def ok(j, i):
        if not (0 <= j < h and 0 <= i < w and T[j, i]):
            return False
        return D is None or D[j, i] < INF

    if ok(j0, i0):
        return (j0, i0)
    for r in range(1, int(rings) + 1):
        ring = []
        for i in range(i0 - r, i0 + r + 1):
            ring.append((j0 - r, i))
            ring.append((j0 + r, i))
        for j in range(j0 - r + 1, j0 + r):
            ring.append((j, i0 - r))
            ring.append((j, i0 + r))
        best, best_d = None, None
        for j, i in ring:
            if not ok(j, i):
                continue
            cost = (j - j0) ** 2 + (i - i0) ** 2
            if D is not None:
                cost += int(D[j, i]) * 4         # prefer genuinely closer
            if best_d is None or cost < best_d:
                best, best_d = (j, i), cost
        if best is not None:
            return best
    return None


def descend(D, start):
    """Walk downhill on a wavefront from `start` to its seed.

    D was flooded from the robot, so descending it from the target yields the
    route target -> robot; the caller reverses it.
    """
    h, w = D.shape
    j, i = start
    if not (0 <= j < h and 0 <= i < w) or D[j, i] >= INF:
        return []
    path = [(j, i)]
    guard = int(D[j, i]) + 4
    while D[j, i] > 0 and guard > 0:
        guard -= 1
        best, best_d = None, D[j, i]
        for dj, di in _SHIFTS:
            nj, ni = j + dj, i + di
            if 0 <= nj < h and 0 <= ni < w and D[nj, ni] < best_d:
                best, best_d = (nj, ni), D[nj, ni]
        if best is None:
            break
        j, i = best
        path.append((j, i))
    return path


def simplify(pts, tol=0.06):
    """Drop points that sit on a straight run (Douglas-Peucker, iterative)."""
    if len(pts) < 3:
        return list(pts)
    keep = [0, len(pts) - 1]
    stack = [(0, len(pts) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        ax, ay = pts[a]
        bx, by = pts[b]
        dx, dy = bx - ax, by - ay
        n = math.hypot(dx, dy)
        worst, wi = 0.0, None
        for k in range(a + 1, b):
            px, py = pts[k]
            if n < 1e-9:
                dev = math.hypot(px - ax, py - ay)
            else:
                dev = abs(dx * (py - ay) - dy * (px - ax)) / n
            if dev > worst:
                worst, wi = dev, k
        if wi is not None and worst > tol:
            keep.append(wi)
            stack.append((a, wi))
            stack.append((wi, b))
    keep.sort()
    return [pts[k] for k in keep]


def carrot(path, x, y, lookahead):
    """Pure pursuit: the point `lookahead` metres along the path from wherever
    the robot currently sits on it.

    Picking "the first waypoint further than lookahead" instead is what broke
    at doorways: as soon as the robot got within lookahead of the doorway
    waypoint, that test skipped it and returned the waypoint beyond, whose
    straight line ran back through the wall.  Walking the polyline by arc
    length cannot skip a corner, because the carrot is always ON the route.
    """
    if not path:
        return None
    if len(path) == 1:
        return tuple(path[0])

    # nearest point on the polyline, as (segment, fraction along it)
    best_d2, seg, frac = float("inf"), 0, 0.0
    for k in range(len(path) - 1):
        ax, ay = path[k]
        bx, by = path[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx +
                                                     (y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2, seg, frac = d2, k, t

    ax, ay = path[seg]
    bx, by = path[seg + 1]
    cx, cy = ax + frac * (bx - ax), ay + frac * (by - ay)
    remain = float(lookahead)
    k = seg
    while k < len(path) - 1:
        bx, by = path[k + 1]
        step = math.hypot(bx - cx, by - cy)
        if step >= remain:
            f = remain / step if step > 1e-12 else 1.0
            return (cx + (bx - cx) * f, cy + (by - cy) * f)
        remain -= step
        cx, cy = bx, by
        k += 1
    return tuple(path[-1])


def track(path, x, y, lookahead):
    """(cross-track distance, nearest point on the path, carrot).

    One pass over the polyline answers both questions the controller needs:
    how far off the route are we, and where should we aim.
    """
    if not path:
        return (0.0, None, None)
    if len(path) == 1:
        p0 = tuple(path[0])
        return (math.hypot(p0[0] - x, p0[1] - y), p0, p0)
    best_d2, seg, frac = float("inf"), 0, 0.0
    for k in range(len(path) - 1):
        ax, ay = path[k]
        bx, by = path[k + 1]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        t = 0.0 if L2 < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx +
                                                     (y - ay) * dy) / L2))
        px, py = ax + t * dx, ay + t * dy
        d2 = (px - x) ** 2 + (py - y) ** 2
        if d2 < best_d2:
            best_d2, seg, frac = d2, k, t
    ax, ay = path[seg]
    bx, by = path[seg + 1]
    near = (ax + frac * (bx - ax), ay + frac * (by - ay))
    return (math.sqrt(best_d2), near, carrot(path, x, y, lookahead))
