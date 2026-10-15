"""Held-out: committed to git before the first run on it; not used for tuning.
Harsher links (India->SF over 4G with frequent stalls, a lossy Wi-Fi hop at the operator end),
narrower gaps, bolder and slower-reacting operators, other layout seeds."""

ROUTES = {
    "in_sf_4g": {"stall_every": 12.0, "stall": (0.4, 1.5), "base": 0.14, "jit_k": 1.5, "jit_s": 0.015, "spike_p": 0.01,
                 "spike": (0.1, 0.4), "p_gb": 0.005, "p_bg": 0.08, "loss_bad": 0.8},
    "wifi_hop": {"stall_every": 30.0, "stall": (0.2, 0.6), "base": 0.07, "jit_k": 1.2, "jit_s": 0.012, "spike_p": 0.006,
                 "spike": (0.05, 0.3), "p_gb": 0.01, "p_bg": 0.05, "loss_bad": 0.9},
    "in_sf": {"stall_every": 25.0, "stall": (0.3, 1.0), "base": 0.125, "jit_k": 2.0, "jit_s": 0.006, "spike_p": 0.004,
              "spike": (0.08, 0.25), "p_gb": 0.003, "p_bg": 0.10, "loss_bad": 0.7},
}

CFG = {
    "routes": ROUTES, "route_names": ["in_sf", "in_sf_4g", "wifi_hop"], "seeds": list(range(3001, 3061)), "gap": (0.075, 0.11),
    "operators": [
        {"speed": 0.5, "gain": 3.5, "react": 0.25, "noise": 0.03, "reach": 0.025, "pass": 0.05, "lag_budget": 0.2},
        {"speed": 0.4, "gain": 2.5, "react": 0.3, "noise": 0.02, "reach": 0.025, "pass": 0.04, "lag_budget": 0.12},
        {"speed": 0.3, "gain": 3.0, "react": 0.18, "noise": 0.015, "reach": 0.02, "pass": 0.04, "lag_budget": 0.08},
    ],
}
