"""Operator in the loop: video goes down a delayed link, the operator reacts, commands go up a
delayed link, the robot's policy decides what to execute, physics measures the contacts.

Operator model: sees the newest video frame that had arrived `react` seconds ago, steers toward the
next waypoint with a P-controller capped at their usual speed, advances waypoints on what they see.
"""
import heapq
import math
import random

from .net import ROUTES, Link, stalls
from .policies import PERIOD, make
from .world import A_MAX, HARD, R, V_MAX, nearest

DT = 0.004
FRAME = 1 / 30


def run(layout, route, policy_name, seed, op=None, routes=ROUTES, t_max=45.0, record=None):
    rng = random.Random(seed)
    # a trained remote operator: knows the lag (the UI shows it) and keeps blind travel ~lag_budget metres
    op = op or {"speed": 0.35, "gain": 3.0, "react": 0.2, "noise": 0.02, "reach": 0.025, "pass": 0.045, "lag_budget": 0.10}
    win = stalls(routes[route], random.Random(seed * 2 + 3), t_max)
    up = Link(routes[route], random.Random(seed * 2 + 1), win)
    down = Link(routes[route], random.Random(seed * 2 + 2), win)
    pol = make(policy_name)
    obstacles, path = layout["obstacles"], layout["path"]
    pos, vel = list(layout["start"]), [0.0, 0.0]
    hist = []                     # (t, x, y) of the true robot, for frames
    frames = []                   # heap of (arrival, sent_t, x, y)
    seen = None                   # newest frame (arrival, sent_t, x, y) delivered to the operator
    pending = []                  # frames that arrived, waiting to be "noticed" after reaction time
    packets = []                  # heap of (arrival, seq, send_t, vx, vy)
    seq, wp = 0, 0
    dwell_until = 0.0
    lag_avg = None
    next_cmd, next_frame = 0.0, 0.0
    contacts = hard = 0
    peak = 0.0
    in_contact = False
    done_t = None
    goals_reached = 0
    hold = 0.0
    t = 0.0
    while t < t_max:
        # robot sends a video frame of where it is now
        if t >= next_frame:
            d = down.send(t)
            if d is not None:
                heapq.heappush(frames, (t + d, t, pos[0], pos[1]))
            next_frame += FRAME
        while frames and frames[0][0] <= t:
            pending.append(heapq.heappop(frames))
        # the operator acts on what reached their screen `react` seconds ago
        while pending and pending[0][0] <= t - op["react"]:
            f = pending.pop(0)
            if seen is None or f[1] > seen[1]:
                seen = f
        if t >= next_cmd:
            if seen is not None and wp < len(path):
                px, py = seen[2], seen[3]
                tx, ty = path[wp]
                dx, dy = tx - px, ty - py
                dist = math.hypot(dx, dy)
                if t < dwell_until:
                    vx = vy = 0.0                      # picking / placing
                elif dist < (op["reach"] if wp in layout["goal_idx"] else op["pass"]):
                    if wp in layout["goal_idx"]:
                        dwell_until = t + 0.8
                    wp += 1
                    vx = vy = 0.0
                else:
                    # the operator adapts to the lag they are used to (a few-second average), not to a spike
                    inst = 2 * (t - seen[1]) + op["react"]          # video age + command trip (~same) + reaction
                    lag_avg = inst if lag_avg is None else lag_avg + (inst - lag_avg) * PERIOD / 5.0
                    sp = min(op["speed"], op["lag_budget"] / lag_avg, min(op["gain"], 0.7 / lag_avg) * dist)
                    vx = sp * dx / dist + rng.gauss(0, op["noise"])
                    vy = sp * dy / dist + rng.gauss(0, op["noise"])
            else:
                vx = vy = 0.0
            d = up.send(t)
            if d is not None:
                heapq.heappush(packets, (t + d, seq, t, vx, vy))
            seq += 1
            next_cmd += PERIOD
        while packets and packets[0][0] <= t:
            a, s, st, vx, vy = heapq.heappop(packets)
            pol.on_packet((s, st, vx, vy), t)
        # robot control + physics
        cx, cy = pol.command(t, pos, obstacles)
        sp = math.hypot(cx, cy)
        if sp > V_MAX:
            cx, cy = cx * V_MAX / sp, cy * V_MAX / sp
        ax, ay = (cx - vel[0]) / DT, (cy - vel[1]) / DT
        am = math.hypot(ax, ay)
        if am > A_MAX:
            ax, ay = ax * A_MAX / am, ay * A_MAX / am
        vel[0] += ax * DT
        vel[1] += ay * DT
        nx, ny = pos[0] + vel[0] * DT, pos[1] + vel[1] * DT
        clear, u = nearest((nx, ny), obstacles)
        if clear < 0:
            vn = vel[0] * u[0] + vel[1] * u[1]
            if not in_contact:
                contacts += 1
                peak = max(peak, vn)
                if vn > HARD:
                    hard += 1
            in_contact = True
            nx, ny = nx + clear * u[0], ny + clear * u[1]      # pushed back to the surface
            if vn > 0:
                vel[0] -= vn * u[0]
                vel[1] -= vn * u[1]
        elif clear > 0.003:
            in_contact = False
        nx = min(max(nx, R), 1.2 - R)
        ny = min(max(ny, R), 0.8 - R)
        pos = [nx, ny]
        if record is not None and int(t / DT) % 10 == 0:
            record.append((round(t, 3), round(nx, 4), round(ny, 4), round(math.hypot(*vel), 3), in_contact,
                           any(a_ <= t < b_ for a_, b_ in win)))
        # task truth: the robot itself holds each goal (pick, then place) for 0.3 s
        gi = layout["goal_idx"][goals_reached] if goals_reached < len(layout["goal_idx"]) else None
        if gi is not None:
            gx, gy = path[gi]
            if math.hypot(pos[0] - gx, pos[1] - gy) < 0.03 and math.hypot(*vel) < 0.08:
                hold += DT
                if hold >= 0.3:
                    goals_reached += 1
                    hold = 0.0
                    if goals_reached == len(layout["goal_idx"]):
                        done_t = t
                        break
            else:
                hold = 0.0
        t += DT
    if record is not None:
        record.append(("stalls", win))
    return {"route": route, "policy": policy_name, "done": done_t is not None, "time": round(done_t or t_max, 2),
            "contacts": contacts, "hard": hard, "peak_impact": round(peak, 3), "guard_stops": getattr(pol, "stops", 0)}
