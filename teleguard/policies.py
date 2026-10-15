"""What the robot does with command packets that arrive late, out of order, or not at all.

Every packet: (seq, send_time, vx, vy). Robot and operator clocks are NTP-synced (error ~1 ms),
so a packet's age is measurable on the robot.

naive     apply the newest packet to arrive (even if it is older than one already applied)
timeout   newest by sequence number; stop if nothing arrives for 200 ms
buffer80  fixed 80 ms playout buffer, then newest playable by sequence; stop after 200 ms of nothing
guard     age-aware: drop stale (> 300 ms) and out-of-order packets; playout delay = p95 of recent ages;
          dead-man decel when the playing command goes stale; speed toward the nearest obstacle capped
          so the robot can stop inside the clearance at the loop delay it is actually seeing
"""
import math
from collections import deque

from .world import A_MAX, nearest

PERIOD = 0.02


class Naive:
    name = "naive"

    def __init__(self):
        self.v = (0.0, 0.0)

    def on_packet(self, pkt, now):
        self.v = pkt[2:]

    def command(self, now, pos, obstacles):
        return self.v


class Timeout:
    name = "timeout"

    def __init__(self, limit=0.2):
        self.v, self.seq, self.last, self.limit = (0.0, 0.0), -1, 0.0, limit

    def on_packet(self, pkt, now):
        self.last = now
        if pkt[0] > self.seq:
            self.seq, self.v = pkt[0], pkt[2:]

    def command(self, now, pos, obstacles):
        return self.v if now - self.last <= self.limit else (0.0, 0.0)


class Buffer:
    name = "buffer80"

    def __init__(self, delay=0.08, limit=0.2):
        self.q, self.v, self.seq, self.delay, self.limit, self.last_play = [], (0.0, 0.0), -1, delay, limit, 0.0

    def on_packet(self, pkt, now):
        self.q.append(pkt)

    def command(self, now, pos, obstacles):
        ready = [p for p in self.q if p[1] + self.delay <= now and p[0] > self.seq]
        if ready:
            p = max(ready, key=lambda p: p[0])
            self.seq, self.v, self.last_play = p[0], p[2:], now
            self.q = [x for x in self.q if x[0] > self.seq]
        return self.v if now - self.last_play <= self.limit else (0.0, 0.0)


class Guard:
    name = "guard"

    def __init__(self, max_age=0.3, react=0.2, margin=0.01, use=("age", "playout", "deadman", "cap")):
        self.q, self.v, self.seq, self.sent = [], (0.0, 0.0), -1, -1.0
        self.ages = deque(maxlen=100)
        self.max_age, self.react, self.margin, self.use = max_age, react, margin, set(use)
        self.stops = 0
        self._stopped = False

    def playout(self):
        if "playout" not in self.use or len(self.ages) < 5:
            return 0.0
        a = sorted(self.ages)
        return min(0.25, a[int(0.95 * (len(a) - 1))])

    def on_packet(self, pkt, now):
        age = now - pkt[1]
        self.ages.append(age)
        if "age" in self.use and (age > self.max_age or pkt[0] <= self.seq):
            return                                   # stale or out of order: never applied
        self.q.append(pkt)

    def command(self, now, pos, obstacles):
        D = self.playout()
        ready = [p for p in self.q if p[1] + D <= now and p[0] > self.seq]
        if ready:
            p = max(ready, key=lambda p: p[0])
            self.seq, self.v, self.sent = p[0], p[2:], p[1]
            self.q = [x for x in self.q if x[0] > self.seq]
        v = self.v
        if "deadman" in self.use and now - self.sent > D + 3 * PERIOD + 0.02:
            if not self._stopped:
                self.stops += 1
                self._stopped = True
            return (0.0, 0.0)
        self._stopped = False
        if "cap" in self.use:
            # loop delay: playout (uplink) + the same again for video back + operator reaction
            loop = 2 * max(D, (sum(self.ages) / len(self.ages)) if self.ages else 0.0) + self.react
            clear, u = nearest(pos, obstacles)
            toward = v[0] * u[0] + v[1] * u[1]
            if toward > 0:
                room = max(0.0, clear - self.margin)
                # largest speed that can still stop within the room after `loop` seconds of blind travel
                cap = -A_MAX * loop + math.sqrt((A_MAX * loop) ** 2 + 2 * A_MAX * room)
                if toward > cap:
                    v = (v[0] - (toward - cap) * u[0], v[1] - (toward - cap) * u[1])
        return v


def make(name, **kw):
    if name.startswith("guard"):
        use = {"guard": ("age", "playout", "deadman", "cap"), "guard-no-cap": ("age", "playout", "deadman"),
               "guard-no-playout": ("age", "deadman", "cap"), "guard-no-deadman": ("age", "playout", "cap"),
               "guard-cap-only": ("cap",)}[name]
        g = Guard(use=use, **kw)
        g.name = name
        return g
    return {"naive": Naive, "timeout": Timeout, "buffer80": Buffer}[name]()
