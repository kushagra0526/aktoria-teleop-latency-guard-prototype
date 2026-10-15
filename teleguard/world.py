"""2D workspace: the end effector moves through gaps between obstacles to a sequence of goals.

Units: metres and seconds. The robot is a disc of radius R with speed and acceleration limits.
Contacts are measured by the physics: an obstacle pushes the effector back out, and the impact
speed is the velocity component into the surface at first touch.
"""
import math
import random

R = 0.02
V_MAX = 0.6
A_MAX = 3.0
HARD = 0.05      # impact speed above this is a hard contact


def rect_dist(p, rc):
    x0, y0, x1, y1 = rc
    dx = max(x0 - p[0], 0.0, p[0] - x1)
    dy = max(y0 - p[1], 0.0, p[1] - y1)
    return math.hypot(dx, dy)


def nearest(p, obstacles):
    """-> (distance from the disc edge to the nearest obstacle, unit vector from p toward it)."""
    best, vec = 1e9, (0.0, 0.0)
    for rc in obstacles:
        x0, y0, x1, y1 = rc
        qx, qy = min(max(p[0], x0), x1), min(max(p[1], y0), y1)
        d = math.hypot(qx - p[0], qy - p[1])
        if d < best:
            best = d
            vec = ((qx - p[0]) / d, (qy - p[1]) / d) if d > 1e-9 else (0.0, 0.0)
    return best - R, vec


def make_layout(rng, n_walls=None, gap=None):
    """Walls across a 1.2 x 0.8 m table with one gap each; goals alternate ends (pick, place, back)."""
    n = n_walls or rng.randint(2, 3)
    g = gap or rng.uniform(0.09, 0.14)
    obs, waypoints = [], []
    xs = [0.25 + i * (0.7 / max(1, n - 1) if n > 1 else 0) for i in range(n)]
    for x in xs:
        gy = rng.uniform(0.18, 0.62)
        obs.append((x - 0.015, 0.0, x + 0.015, gy - g / 2))
        obs.append((x - 0.015, gy + g / 2, x + 0.015, 0.8))
        waypoints.append((x - 0.06, gy))
        waypoints.append((x + 0.06, gy))
    # a few small clutter blocks off the path
    for _ in range(rng.randint(1, 3)):
        cx, cy = rng.uniform(0.1, 1.1), rng.uniform(0.05, 0.75)
        if all(math.hypot(cx - w[0], cy - w[1]) > 0.12 for w in waypoints):
            obs.append((cx - 0.03, cy - 0.03, cx + 0.03, cy + 0.03))
    start = (0.08, rng.uniform(0.3, 0.5))
    goal = (1.12, rng.uniform(0.3, 0.5))
    there = [start] + waypoints + [goal]
    path = there[1:] + list(reversed(there[:-1]))     # out and back: pick at the far end, place at the start
    return {"obstacles": obs, "start": start, "path": path, "goal_idx": [len(there) - 2, len(path) - 1]}
