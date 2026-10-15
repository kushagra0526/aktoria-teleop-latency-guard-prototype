"""One teleop episode over an India -> SF 4G link with stalls, three robot-side policies -> out/report.html

    python demo.py
"""
import html
import json
import random
import sys
from pathlib import Path

HERE = Path(__file__).parent
sys.path.insert(0, str(HERE))

from eval.heldout import CFG, ROUTES  # noqa: E402
from teleguard.sim import run  # noqa: E402
from teleguard.world import make_layout  # noqa: E402

SEED = 19
POLS = [("naive", "#e15759"), ("guard-cap-only", "#4e9ff5"), ("guard", "#59a14f")]
e = html.escape


def build():
    layout = make_layout(random.Random(SEED), gap=0.09)
    op = CFG["operators"][0]
    out = {}
    for p, _ in POLS:
        rec = []
        r = run(layout, "in_sf_4g", p, SEED, op=op, routes=ROUTES, record=rec)
        stalls = rec.pop()[1]
        out[p] = (r, rec, stalls)
    return layout, out


def svg(layout, out, W=760, H=507):
    sx, sy = W / 1.2, H / 0.8
    g = [f'<rect x="0" y="0" width="{W}" height="{H}" fill="#151a22"/>']
    for x0, y0, x1, y1 in layout["obstacles"]:
        g.append(f'<rect x="{x0 * sx:.1f}" y="{y0 * sy:.1f}" width="{(x1 - x0) * sx:.1f}" height="{(y1 - y0) * sy:.1f}" fill="#7a8496"/>')
    for i in layout["goal_idx"]:
        gx, gy = layout["path"][i]
        g.append(f'<circle cx="{gx * sx:.1f}" cy="{gy * sy:.1f}" r="9" fill="none" stroke="#f2c14e" stroke-width="2.5"/>')
    for p, col in POLS:
        r, rec, _ = out[p]
        pts = " ".join(f"{x * sx:.1f},{y * sy:.1f}" for t, x, y, *_ in rec)
        g.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="2" opacity="0.9"/>')
        prev = False
        for t, x, y, v, c, st in rec:
            if c and not prev:
                g.append(f'<circle cx="{x * sx:.1f}" cy="{y * sy:.1f}" r="7" fill="none" stroke="{col}" stroke-width="3"/>')
            prev = c
    return f'<svg viewBox="0 0 {W} {H}" width="100%">{"".join(g)}</svg>'


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    layout, out = build()
    stalls = out["naive"][2]
    print(f"route in_sf_4g, layout seed {SEED}, stalls: " + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in stalls if a < 40))
    for p, _ in POLS:
        r = out[p][0]
        print(f"  {p:<16} hard contacts {r['hard']}  peak {r['peak_impact']:.2f} m/s  done {r['done']}  time {r['time']}s  dead-man stops {r['guard_stops']}")
    ev = {}
    for k, f in (("dev", "out/eval_dev.json"), ("heldout_first_run", "out/heldout_first_run.json")):
        if (HERE / f).exists():
            ev[k] = json.loads((HERE / f).read_text(encoding="utf-8"))
    (HERE / "out").mkdir(exist_ok=True)
    (HERE / "out/report.html").write_text(render(layout, out, ev), encoding="utf-8")
    print("wrote out/report.html")


def render(layout, out, ev):
    legend = "".join(f'<span style="color:{c}">■ {e(p)}: {out[p][0]["hard"]} hard contacts, {out[p][0]["time"]} s</span> &nbsp; ' for p, c in POLS)
    stalls = out["naive"][2]
    tl = "".join(f'<div style="position:absolute;left:{a / 40 * 100:.2f}%;width:{(b - a) / 40 * 100:.2f}%;top:0;bottom:0;background:#e15759"></div>'
                 for a, b in stalls if a < 40)
    rows = ""
    for k, lab in (("dev", "dev: local / Mexico→SF / India→SF"), ("heldout_first_run", "held-out first run: India→SF / India→SF 4G / Wi-Fi hop")):
        if k not in ev:
            continue
        for route, pols in ev[k].items():
            for p in ("naive", "timeout", "buffer80", "guard", "guard-no-cap", "guard-cap-only"):
                a = pols[p]
                rows += (f"<tr><td>{e(lab) if p == 'naive' else ''}</td><td>{e(route) if p == 'naive' else ''}</td><td>{e(p)}</td>"
                         f"<td><b>{int(a['hard'])}</b></td><td>{int(a['eps_with_hard'])}/{int(a['episodes'])}</td><td>{int(a['done'])}/{int(a['episodes'])}</td>"
                         f"<td>{a['mean_time']}</td></tr>")
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Teleop latency guard</title><style>
:root{{--bg:#0f1217;--fg:#e8ecf2;--mut:#93a0b4;--card:#171c25;--line:#2a3140}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--fg);font:15px/1.5 system-ui,-apple-system,Segoe UI,sans-serif}}
main{{max-width:1100px;margin:0 auto;padding:26px 16px 60px}}h1{{font-size:25px;margin:0 0 4px}}h2{{font-size:17px;margin:24px 0 8px}}.sub{{color:var(--mut)}}
.card{{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:12px}}
table{{width:100%;border-collapse:collapse;background:var(--card);font-size:13px}}td,th{{border:1px solid var(--line);padding:5px 7px;text-align:left}}
</style></head><body><main>
<h1>Operator in India, robot in San Francisco, link over 4G</h1>
<div class="sub">Simulated: delayed video to the operator, delayed commands back, stalls where nothing gets through then everything arrives at once. Same operator, same network trace, three robot-side policies. Contacts are measured by the physics.
Unofficial prototype by Kushagra Pratap Singh, not affiliated with Aktoria Robotics.</div>
<div class="card" style="margin-top:12px">{svg(layout, out)}<div style="margin-top:6px;font-size:14px">{legend} &nbsp; circles = contacts, yellow = pick / place</div>
<div style="margin-top:10px;font-size:13px;color:var(--mut)">link stalls over the first 40 s</div><div style="position:relative;height:14px;background:#232a36;border-radius:4px">{tl}</div></div>
<h2>Benchmark (60 episodes per route; ablations remove one part of the guard)</h2>
<div style="overflow-x:auto"><table><tr><th>set</th><th>route</th><th>robot-side policy</th><th>hard contacts</th><th>episodes with one</th><th>tasks done</th><th>mean time (s)</th></tr>{rows}</table></div>
</main></body></html>"""


if __name__ == "__main__":
    main()
