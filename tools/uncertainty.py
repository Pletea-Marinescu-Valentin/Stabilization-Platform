from __future__ import annotations

import numpy as np

from .config import AXES, CONTROLLERS, RANDOM_SEED, TS
from .design import controller_response, loop_metrics
from .identification import fit_oe, identify_axis, prepare_axis
from .plant import AxisPlant

BLOCK_FRACTION = 0.6

def bootstrap_parameters(axis: str, n_draws=200, seed=RANDOM_SEED, verbose=False):
    t, u, y, ts, _ = prepare_axis(axis)
    ref = identify_axis(axis, verbose=False)
    na, nb, d = ref.na, ref.nb, ref.delay
    n = len(y)
    span = int(BLOCK_FRACTION * n)
    rng = np.random.default_rng(seed)

    K, wn, zeta = [], [], []
    for _ in range(n_draws):
        i0 = int(rng.integers(0, n - span))
        sl = slice(i0, i0 + span)
        try:
            m = fit_oe(u[sl], y[sl], na, nb, d, ts, axis)
            if np.max(np.abs(m.poles())) >= 1.0:
                continue
            w, z = m.dominant_mode()
            if not (0 < w < 200 and 0 < z < 1):
                continue
            K.append(m.dc_gain)
            wn.append(float(w))
            zeta.append(float(z))
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
    bw_ref = {n: loop_metrics(nominal, resp[n], ts)[0] for n in CONTROLLERS}
    order_ref = sorted(CONTROLLERS, key=lambda n: -bw_ref[n])
    fastest_ref, slowest_ref = order_ref[0], order_ref[-1]
    pairs = [(a, b) for i, a in enumerate(order_ref) for b in order_ref[i + 1:]]

    kept_full = kept_slowest = kept_fastest = unstable = 0
    kept_pair = {p: 0 for p in pairs}
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
        if sorted(CONTROLLERS, key=lambda n: -bw[n]) == order_ref:
            kept_full += 1
        if min(bw, key=bw.get) == slowest_ref:
            kept_slowest += 1
        if max(bw, key=bw.get) == fastest_ref:
            kept_fastest += 1
        for a, b in pairs:
            if bw[a] > bw[b]:
                kept_pair[(a, b)] += 1

    d = max(total, 1)
    pct = {f"{a}>{b}": 100.0 * c / d for (a, b), c in kept_pair.items()}
    gap = {f"{a}>{b}": 100.0 * (bw_ref[a] - bw_ref[b]) / bw_ref[b] for a, b in pairs}
    tied = [k for k, v in gap.items() if abs(v) < 0.1]
    return dict(order_nominal=order_ref, n=total, bandwidth=bw_ref,
                full_order_pct=100.0 * kept_full / d,
                slowest_pct=100.0 * kept_slowest / d,
                fastest_pct=100.0 * kept_fastest / d,
                pairwise_pct=pct, nominal_gap_pct=gap, tied=tied,
                resolved=[k for k, v in pct.items() if v >= 95.0],
                unresolved=[k for k, v in pct.items()
                            if v < 95.0 and k not in tied],
                ms_median=float(np.median(ms_max)) if ms_max else float("nan"),
                ms_worst=float(np.max(ms_max)) if ms_max else float("nan"),
                ms_p95=float(np.percentile(ms_max, 95)) if ms_max else float("nan"),
                ms_over2_pct=100.0 * unstable / d)

def study(plants, designs, n_draws=200, verbose=True):
    out = {}
    for ax in AXES:
        boot = bootstrap_parameters(ax, n_draws, verbose=verbose)
        rank = ranking_stability(plants[ax], designs[ax], boot)
        out[ax] = dict(bootstrap={k: v for k, v in boot.items()
                                  if not isinstance(v, np.ndarray)},
                       ranking=rank)
        if verbose:
            print(f"  [{ax}] fastest {rank['fastest_pct']:.0f}%, slowest "
                  f"{rank['slowest_pct']:.0f}%, full order "
                  f"{rank['full_order_pct']:.0f}% of draws; worst Ms "
                  f"{rank['ms_worst']:.2f} (median {rank['ms_median']:.2f})")
            n_cmp = len(rank["pairwise_pct"]) - len(rank["tied"])
            print(f"        {len(rank['resolved'])}/{n_cmp} separable pairs "
                  f"resolved at 95% ({len(rank['tied'])} tied by construction: "
                  f"{', '.join(rank['tied']) or 'none'}); unresolved: " +
                  (", ".join(f"{k} ({rank['pairwise_pct'][k]:.0f}%, nominal gap "
                             f"{rank['nominal_gap_pct'][k]:.1f}%)"
                             for k in rank["unresolved"]) or "none"))
    return out
