import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from teleguard.net import ROUTES, Link  # noqa: E402
from teleguard.policies import Guard, Naive, Timeout  # noqa: E402
from teleguard.sim import run  # noqa: E402
from teleguard.world import A_MAX, R, make_layout, nearest  # noqa: E402


def test_stall_holds_packets_until_it_ends():
    ln = Link(ROUTES["in_sf"], random.Random(1), [(1.0, 1.6)])
    d = ln.send(1.2)
    assert d is not None and 1.2 + d >= 1.6 + ROUTES["in_sf"]["base"]


def test_guard_drops_stale_and_out_of_order():
    g = Guard()
    g.on_packet((5, 0.0, 0.1, 0.0), now=0.5)        # 500 ms old: dropped
    assert g.q == []
    g.on_packet((6, 0.49, 0.1, 0.0), now=0.6)
    g.command(0.7, (0.5, 0.5), [])
    g.on_packet((4, 0.55, -0.3, 0.0), now=0.71)     # older sequence than the one applied: dropped
    assert all(p[0] > 6 for p in g.q)
    n = Naive()
    n.on_packet((6, 0.49, 0.1, 0.0), 0.6)
    n.on_packet((4, 0.40, -0.3, 0.0), 0.61)
    assert n.command(0.62, (0, 0), []) == (-0.3, 0.0)   # naive executes the stale reversal


def test_deadman_and_timeout_stop_when_commands_stop():
    g = Guard(use=("age", "playout", "deadman"))
    g.ages.extend([0.13] * 20)                      # the link has been running at ~130 ms
    g.on_packet((1, 0.0, 0.2, 0.0), now=0.13)
    assert g.command(0.14, (0.5, 0.4), []) != (0.0, 0.0)
    assert g.command(0.30, (0.5, 0.4), []) == (0.0, 0.0)   # nothing newer for > playout + 80 ms
    t = Timeout()
    t.on_packet((1, 0.0, 0.2, 0.0), now=0.13)
    assert t.command(0.30, (0.5, 0.4), []) != (0.0, 0.0) and t.command(0.34, (0.5, 0.4), []) == (0.0, 0.0)


def test_cap_lets_the_robot_stop_inside_the_clearance():
    g = Guard(use=("cap",))
    g.ages.extend([0.15] * 20)
    wall = [(0.6, 0.0, 0.62, 0.8)]
    pos = (0.5, 0.4)
    g.on_packet((1, -0.15, 0.5, 0.0), 0.0)
    v = g.command(0.0, pos, wall)
    clear, _ = nearest(pos, wall)
    loop = 2 * 0.15 + g.react
    assert v[0] * loop + v[0] ** 2 / (2 * A_MAX) <= clear - g.margin + 1e-9
    assert abs(g.command(0.0, pos, [])[0] - 0.5) < 1e-9      # no obstacle ahead: no cap


def test_physics_counts_contacts_and_never_penetrates():
    layout = make_layout(random.Random(19), gap=0.09)
    rec = []
    r = run(layout, "in_sf", "naive", 19, record=rec)
    rec.pop()
    for t, x, y, *_ in rec:
        assert nearest((x, y), layout["obstacles"])[0] > -1e-3
    assert r["contacts"] >= r["hard"]


def test_demo_episode_guard_vs_naive():
    from demo import build
    layout, out = build()
    assert out["naive"][0]["hard"] >= 1
    assert out["guard"][0]["hard"] == 0 and out["guard-cap-only"][0]["hard"] == 0
    assert out["guard-cap-only"][0]["done"]


def test_deterministic():
    layout = make_layout(random.Random(3))
    assert run(layout, "in_sf", "guard", 3) == run(layout, "in_sf", "guard", 3)
    assert R > 0 and math.isfinite(A_MAX)
