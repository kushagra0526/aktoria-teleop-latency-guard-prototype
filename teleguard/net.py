"""One-way network delay traces: base + gamma jitter + rare spikes, Gilbert-Elliott burst loss.

Routes are assumptions in the right range for the claims on aktoria.com (~5 ms local,
~130 ms India -> San Francisco); they are not measurements of Aktoria's network.
"""
import random

ROUTES = {
    "local": {"stall_every": 600.0, "stall": (0.2, 0.4), "base": 0.005, "jit_k": 2.0, "jit_s": 0.0008, "spike_p": 0.0005, "spike": (0.02, 0.05), "p_gb": 0.0005, "p_bg": 0.3, "loss_bad": 0.5},
    "mx_sf": {"stall_every": 40.0, "stall": (0.3, 0.8), "base": 0.055, "jit_k": 2.0, "jit_s": 0.004, "spike_p": 0.003, "spike": (0.05, 0.15), "p_gb": 0.002, "p_bg": 0.15, "loss_bad": 0.6},
    "in_sf": {"stall_every": 25.0, "stall": (0.3, 1.0), "base": 0.125, "jit_k": 2.0, "jit_s": 0.006, "spike_p": 0.004, "spike": (0.08, 0.25), "p_gb": 0.003, "p_bg": 0.10, "loss_bad": 0.7},
}


def stalls(route, rng, t_max=60.0):
    """Windows when the path freezes (Wi-Fi/4G handover, bufferbloat): nothing gets through, then
    everything queued is delivered at once. Shared by uplink and downlink of one session."""
    out, t = [], 0.0
    while True:
        t += rng.expovariate(1.0 / route["stall_every"])
        if t > t_max:
            return out
        d = rng.uniform(*route["stall"])
        out.append((t, t + d))
        t += d


class Link:
    """Per-packet delay or None (lost). Packets sent inside a stall wait for its end."""

    def __init__(self, route, rng, stall_windows=()):
        self.r, self.rng, self.bad, self.stalls = route, rng, False, list(stall_windows)

    def send(self, t=None):
        r, rng = self.r, self.rng
        if t is not None:
            for a, b in self.stalls:
                if a <= t < b:
                    return (b - t) + r["base"] + rng.gammavariate(r["jit_k"], r["jit_s"]) + rng.uniform(0, 0.03)
        self.bad = (rng.random() < r["p_gb"]) if not self.bad else (rng.random() >= r["p_bg"])
        if self.bad and rng.random() < r["loss_bad"]:
            return None
        d = r["base"] + rng.gammavariate(r["jit_k"], r["jit_s"])
        if rng.random() < r["spike_p"]:
            d += rng.uniform(*r["spike"])
        return d


def link(route_name, seed, routes=ROUTES):
    return Link(routes[route_name], random.Random(seed))
