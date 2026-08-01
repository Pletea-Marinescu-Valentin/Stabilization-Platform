from __future__ import annotations

import numpy as np

from .config import AXES, CONTROLLERS, INPUT_DELAY_SAMPLES, RANDOM_SEED, TS
from .design import controller_response, loop_metrics
from .identification import load_prbs, fit_oe
from .plant import AxisPlant

BLOCK_FRACTION = 0.6

def bootstrap_parameters(axis: str, n_draws=200, seed=RANDOM_SEED, verbose=False):
    t, u, y = load_prbs(axis)
    n = len(y)
    span = int(BLOCK_FRACTION * n)
    rng = np.random.default_rng(seed)

    K, wn, zeta = [], [], []
    for _ in range(n_draws):
        i0 = int(rng.integers(0, n - span))
        sl = slice(i0, i0 + span)
        try:
            m = fit_oe(u[sl], y[sl], 2, 1, INPUT_DELAY_SAMPLES, TS, axis)
            if np.max(np.abs(m.poles())) >= 1.0:
                continue
            w, z = m.continuous_modes()
            if not (0 < w[0] < 200 and 0 < z[0] < 1):
                continue
            K.append(m.dc_gain)
            wn.append(float(w[0]))
            zeta.append(float(z[0]))
        except Exception:
            continue

    out = dict(K=np.array(K), wn=np.array(wn), zeta=np.array(zeta),
               n=len(K), n_draws=n_draws)
    for key in ("K", "wn", "zeta"):
        v = out[key]
        out[f"{key}_mean"] = float(np.mean(v))
        out[f"{key}_cv"] = float(100 * np.std(v) / abs(np.mean(v)))
        out[f"{key}_p5"] = float(np.percentile(v, 5))
        out[f"{key}_p95"] = float(np.percentile(v, 95))
    if verbose:
        print(f"  [{axis}] {out['n']}/{n_draws} usable refits: "
              f"K {out['K_cv']:.1f}%, wn {out['wn_cv']:.1f}%, "
              f"zeta {out['zeta_cv']:.1f}% coefficient of variation")
    return out

def ranking_stability(nominal: AxisPlant, designs, boot, ts=TS):
    resp = {n: controller_response(n, designs) for n in CONTROLLERS}
    order_ref = sorted(CONTROLLERS,
                       key=lambda n: -loop_metrics(nominal, resp[n], ts)[0])

    kept_full, kept_pid_last, unstable = 0, 0, 0
    ms_max, total = [], 0
    for K, wn, zeta in zip(boot["K"], boot["wn"], boot["zeta"]):
        p = AxisPlant(nominal.axis, float(K), float(wn), float(zeta),
                      nominal.delay, ts, noise_std=nominal.noise_std)
        bw, ms = {}, {}
        bad = False
        for n in CONTROLLERS:
            try:
                b, s, _ = loop_metrics(p, resp[n], ts)
            except Exception:
                bad = True
                break
            bw[n], ms[n] = b, s
        if bad:
            continue
        total += 1
        if max(ms.values()) > 2.0:
            unstable += 1
        ms_max.append(max(ms.values()))
        order = sorted(CONTROLLERS, key=lambda n: -bw[n])
        if order == order_ref:
            kept_full += 1
        if min(bw, key=bw.get) == min(CONTROLLERS, key=lambda n: loop_metrics(nominal, resp[n], ts)[0]):
            kept_pid_last += 1

    return dict(order_nominal=order_ref, n=total,
                full_order_pct=100.0 * kept_full / max(total, 1),
                slowest_pct=100.0 * kept_pid_last / max(total, 1),
                ms_median=float(np.median(ms_max)) if ms_max else float("nan"),
                ms_p95=float(np.percentile(ms_max, 95)) if ms_max else float("nan"),
                ms_over2_pct=100.0 * unstable / max(total, 1))

def study(plants, designs, n_draws=200, verbose=True):
    out = {}
    for ax in AXES:
        boot = bootstrap_parameters(ax, n_draws, verbose=verbose)
        rank = ranking_stability(plants[ax], designs[ax], boot)
        out[ax] = dict(bootstrap={k: v for k, v in boot.items()
                                  if not isinstance(v, np.ndarray)},
                       ranking=rank)
        if verbose:
            print(f"  [{ax}] slowest structure unchanged in "
                  f"{rank['slowest_pct']:.0f}% of draws, full order in "
                  f"{rank['full_order_pct']:.0f}%; median worst Ms "
                  f"{rank['ms_median']:.2f}")
    return out
