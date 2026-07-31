from __future__ import annotations

import numpy as np

from .config import TS

WEIGHTS = {"IAE": 0.40, "RMSE": 0.25, "peak": 0.20, "effort": 0.15}

def compute(t, e, u, ts=TS):
    e = np.asarray(e, dtype=float)
    u = np.asarray(u, dtype=float)
    n = len(e)
    if n == 0:
        return {}

    m = {
        "RMSE": float(np.sqrt(np.mean(e ** 2))),
        "MAE": float(np.mean(np.abs(e))),
        "peak": float(np.max(np.abs(e))),
        "IAE": float(np.sum(np.abs(e)) * ts),
        "ISE": float(np.sum(e ** 2) * ts),
        "std": float(np.std(e)),
        "effort": float(np.sum(np.abs(np.diff(u))) / (n * ts)) if n > 1 else 0.0,
        "u_rms": float(np.sqrt(np.mean(u ** 2))),
        "u_max": float(np.max(np.abs(u))),
        "in_1deg": float(100.0 * np.mean(np.abs(e) <= 1.0)),
        "in_2deg": float(100.0 * np.mean(np.abs(e) <= 2.0)),
    }
    tail = min(64, n)
    m["sse"] = float(np.mean(e[-tail:]))
    return m

def settling_time(t, e, band, ts=TS):
    out = np.where(np.abs(e) > band)[0]
    if len(out) == 0:
        return 0.0
    if out[-1] >= len(e) - 1:
        return float("nan")
    return float((out[-1] + 1) * ts)

def composite(table: dict, weights=None):
    weights = WEIGHTS if weights is None else weights
    names = list(table)
    scores = {}
    total_w = sum(weights.values())
    for n in names:
        s = 0.0
        for key, w in weights.items():
            vals = np.array([abs(table[m][key]) for m in names], dtype=float)
            best = float(np.min(vals[vals > 0])) if np.any(vals > 0) else 1.0
            s += w * (abs(table[n][key]) / best)
        scores[n] = float(s / total_w)
    return scores

def weight_sensitivity(table: dict, n_draws=20000, seed=7):
    rng = np.random.default_rng(seed)
    names = list(table)
    keys = list(WEIGHTS)
    wins = {n: 0 for n in names}

    ratios = np.empty((len(names), len(keys)))
    for i, n in enumerate(names):
        for j, k in enumerate(keys):
            vals = np.array([abs(table[m][k]) for m in names], dtype=float)
            best = float(np.min(vals[vals > 0])) if np.any(vals > 0) else 1.0
            ratios[i, j] = abs(table[n][k]) / best

    w = rng.dirichlet(np.ones(len(keys)), size=n_draws)
    scores = ratios @ w.T
    winners = np.argmin(scores, axis=0)
    for idx in winners:
        wins[names[idx]] += 1
    return {n: 100.0 * c / n_draws for n, c in wins.items()}
