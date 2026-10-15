"""Benchmark: remote operator + network route + robot-side policy, contacts measured by the physics.

    python evaluate.py dev        # 60 layouts x routes local / mx_sf / in_sf, operator styles A
    python evaluate.py heldout    # eval/heldout.py: other routes, narrower gaps, bolder operators (frozen in git first)
"""
import json
import random
import sys
import time
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from teleguard.net import ROUTES  # noqa: E402
from teleguard.sim import run  # noqa: E402
from teleguard.world import make_layout  # noqa: E402

POLICIES = ["naive", "timeout", "buffer80", "guard", "guard-no-cap", "guard-no-deadman", "guard-cap-only"]
DEV = {
    "routes": ROUTES, "route_names": ["local", "mx_sf", "in_sf"], "seeds": list(range(1, 61)), "gap": None,
    "operators": [
        {"speed": 0.35, "gain": 3.0, "react": 0.2, "noise": 0.02, "reach": 0.025, "pass": 0.045, "lag_budget": 0.10},
        {"speed": 0.45, "gain": 3.0, "react": 0.2, "noise": 0.02, "reach": 0.025, "pass": 0.045, "lag_budget": 0.16},
    ],
}


def run_split(cfg):
    agg = {r: {p: defaultdict(float) for p in POLICIES} for r in cfg["route_names"]}
    for seed in cfg["seeds"]:
        rng = random.Random(seed)
        layout = make_layout(rng, gap=rng.uniform(*cfg["gap"]) if cfg["gap"] else None)
        op = cfg["operators"][seed % len(cfg["operators"])]
        for route in cfg["route_names"]:
            res = {p: run(layout, route, p, seed, op=op, routes=cfg["routes"]) for p in POLICIES}
            all_done = all(r["done"] for r in res.values())
            for p, r in res.items():
                a = agg[route][p]
                a["episodes"] += 1
                a["hard"] += r["hard"]
                a["contacts"] += r["contacts"]
                a["eps_with_hard"] += r["hard"] > 0
                a["peak_impact"] = max(a["peak_impact"], r["peak_impact"])
                a["done"] += r["done"]
                a["guard_stops"] += r["guard_stops"]
                if all_done:                       # compare time on episodes every policy finished
                    a["time_sum"] += r["time"]
                    a["time_n"] += 1
    out = {}
    for route, pols in agg.items():
        out[route] = {}
        for p, a in pols.items():
            out[route][p] = {k: (round(v, 3) if isinstance(v, float) else v) for k, v in a.items()}
            out[route][p]["mean_time"] = round(a["time_sum"] / a["time_n"], 2) if a["time_n"] else None
    return out


def show(name, res):
    print(f"\n== {name}")
    print(f"{'route':<12}{'policy':<18}{'hard contacts':>14}{'eps w/ hard':>12}{'peak m/s':>10}{'done':>7}{'mean time s':>13}{'guard stops':>12}")
    for route, pols in res.items():
        for p, a in pols.items():
            print(f"{route:<12}{p:<18}{int(a['hard']):>14}{int(a['eps_with_hard']):>12}{a['peak_impact']:>10.2f}"
                  f"{int(a['done'])}/{int(a['episodes']):<5}{str(a['mean_time']):>10}{int(a['guard_stops']):>12}")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    split = sys.argv[1] if len(sys.argv) > 1 else "dev"
    if split == "heldout":
        from eval.heldout import CFG
        cfg = CFG
    else:
        cfg = DEV
    t0 = time.time()
    res = run_split(cfg)
    (HERE / "out").mkdir(exist_ok=True)
    (HERE / f"out/eval_{split}.json").write_text(json.dumps(res, indent=1), encoding="utf-8")
    show(split, res)
    print(f"\n{time.time() - t0:.0f}s")
