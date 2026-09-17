# Autonomous exploration

Frontier exploration without Nav2. Nav2 plus slam_toolbox does not leave much of
a 1 GB Pi, and it is a tuning project in its own right. This reads the occupancy
grid slam_toolbox already publishes, finds the boundary between "free" and "not
yet seen", plans a route through known free space, and follows it.

```bash
ros2 run lidarbot explorer --ros-args -p dry_run:=true
```

Start in `dry_run`. It picks targets and publishes markers but sends no
`/cmd_vel`, so you can check its choices in Foxglove before it is allowed to move.

In the 3D panel add:

- `/explore/path` — the planned route
- `/explore/marker` — current target, green sphere
- `/explore/candidates` — every frontier considered, amber points

All latched and republished at 1 Hz, so a viewer joining mid-run sees them
immediately. A status line prints every 5 s naming what it is waiting on.

## How it picks a target

1. A frontier cell is free and touches unknown.
2. Obstacles are grown by `robot_radius` and frontier cells inside that are
   dropped, so targets are reachable rather than pressed against a wall.
3. Cells are clustered into ~0.3 m buckets; clusters smaller than
   `min_frontier` are noise.
4. A **wavefront floods outward from the robot through known free space only**.
   Each cluster is scored by that route distance, minus a bonus for size. A
   cluster the flood never reached has no route and is skipped entirely.

Step 4 is the one that matters. Straight-line distance picks targets through
walls:

```
candidate A:  straight-line 4.00 m,  by road 5.49 m
candidate B:  straight-line 4.72 m,  by road 3.96 m   <- correctly preferred
```

## How it drives there

The route comes from descending the same wavefront from the target back to the
robot, then simplified with Douglas-Peucker.

**The planner leads, the scan is only a veto.** The planner has already checked
body clearance along the whole path, so the reactive layer exists to catch things
the map does not contain. Letting a reactive dodge override a valid route is what
produces a left-right stalemate at every doorway.

Steering is pure pursuit, walking the polyline by arc length — never "the first
waypoint further than the lookahead", which skips a doorway waypoint the moment it
comes within range and aims at the next one through the wall. The lookahead
shortens as cross-track error grows, which turns corner-cutting into route
recovery.

Live scan returns are stamped into the planner's obstacle mask, so an unmapped
obstacle gets routed *around* instead of vetoed a metre later.

## Parameters

| Param | Default | Raise/lower if |
|---|---|---|
| `max_speed` | 0.12 | m/s. Slower gives the scan matcher more to work with |
| `clear_width` | 0.18 | Robot half-width + margin. Raise if it clips doorframes |
| `robot_radius` | 0.22 | Planning clearance. **Lower** to attempt tighter gaps |
| `lookahead` | 0.35 | m along the route. Longer cuts corners, shorter oscillates |
| `hard_stop` | 0.16 | m. Emergency veto while following a route |
| `min_frontier` | 6 | cells. Raise if it chases noise |
| `dry_run` | false | Plan and publish markers without moving |

## Testing it without a robot

`tools/tests/sim_explore.py` stubs out ROS, builds a synthetic two-room house,
raycasts a LiDAR and closes the loop through a diff-drive model. It measures the
two failure modes directly: does it aim through walls, and does it shuffle instead
of advancing.

```bash
cd tools/tests
python3 test_planning.py                      # planner unit tests
python3 sim_explore.py                        # 0.9 m doorway
DOORW=0.76 python3 sim_explore.py             # realistic interior door
DOORW=0.76 TH0=180 python3 sim_explore.py     # starts facing away
DOORW=0.76 GHOST=4.4,3.8,5.0,4.4 python3 sim_explore.py   # unmapped obstacle
DOORW=0.0 EXPECT=unreachable python3 sim_explore.py       # sealed door
```

Before and after the doorway fixes, same run:

| | before | after |
|---|---|---|
| got through the doorway | no | yes |
| mean speed | 0.026 m/s | 0.080 m/s |
| turning in place | 73% of ticks | 0% |
| left-right reversals | 858 | 0 |

With a box narrowing the doorway to 0.46 m it declines and says *"none of 21
frontiers is reachable through known free space"* rather than wedging itself.
That is `robot_radius` being conservative, and it is the correct default.

## When it stops

`done` needs five consecutive passes finding no frontiers **and** at least 2000
free cells in the map. One empty result at startup — when the map is a single
scan wide — must not be allowed to end the run.
