from __future__ import annotations

import csv
import json
import time

import numpy as np

from .config import (AXES, CONTROLLER_LABELS, CONTROLLERS, FIGURES, RESULTS,
                     SCENARIOS, TS, T_DIST_START, T_TOTAL, T_TRACK_END, WC, ZETA)
from . import controllers as ctrl_mod
from . import metrics as met
from .design import (MS_TARGET, design_all, design_equal_robustness,
                     reference_bandwidth, robustness_frontier, verify_designs)
from .plant import describe_scenarios, fit_all, payload_variant
from .simulate import simulate
from .ingest import (EXPERIMENTS, collect as collect_experiments, discover,
                     timing_report)

def _ensure_dirs():
    RESULTS.mkdir(parents=True, exist_ok=True)
    FIGURES.mkdir(parents=True, exist_ok=True)

def run_everything(verbose=True):
    _ensure_dirs()
    out = {}

    if verbose:
        print("=" * 70)
        print("1. System identification")
    plants = fit_all(verbose=verbose)
    out["plants"] = {
        ax: dict(K=p.K, wn=p.wn, zeta=p.zeta, delay=p.delay,
                 delay_ms=p.delay_seconds * 1000, fit_valid=p.fit_valid,
                 noise_std=p.noise_std, f_n=p.wn / (2 * np.pi))
        for ax, p in plants.items()}
    out["scenarios"] = describe_scenarios(plants)

    if verbose:
        print(f"\n2. Controller design (equal robustness: Ms = {MS_TARGET})")
    designs, verification, frontier, dmeta = {}, {}, {}, {}
    for ax in AXES:
        designs[ax], dmeta[ax] = design_equal_robustness(plants[ax], MS_TARGET)
        verification[ax] = verify_designs(plants[ax], designs[ax])
        frontier[ax] = robustness_frontier(plants[ax], designs[ax])
        if verbose:
            print(f"  [{ax}]")
            for n, a in verification[ax].items():
                f = frontier[ax][n]
                print(f"     {a.name:5s} BW={a.bandwidth:5.2f}  Ms={a.modulus_margin:4.2f}  "
                      f"GM={a.gain_margin_db:5.1f}dB  PM={a.phase_margin_deg:5.1f}deg  "
                      f"gain tol={f['gain_db']:5.1f}dB  freq tol={f['freq_pct']:5.1f}%")

    if verbose:
        print(f"\n2b. Fixed-performance baseline (wc={WC} rad/s, zeta={ZETA})")
    out["fixed_spec"] = {}
    for ax in AXES:
        fd = design_all(plants[ax])
        fv = verify_designs(plants[ax], fd)
        out["fixed_spec"][ax] = {n: dict(bandwidth=a.bandwidth,
                                         modulus_margin=a.modulus_margin)
                                 for n, a in fv.items()}
        if verbose:
            print(f"  [{ax}] " + "  ".join(
                f"{n}: BW={a.bandwidth:.2f} Ms={a.modulus_margin:.2f}"
                for n, a in fv.items()))

    out["verification"] = {ax: {n: vars(a) for n, a in v.items()}
                           for ax, v in verification.items()}
    out["frontier"] = frontier
    out["design_meta"] = dmeta
    out["design_params"] = _design_params(designs)

    measured = collect_experiments(verbose=verbose)
    out["source"] = "measured" if measured else "model"

    if verbose:
        if measured:
            print(f"\n3. Closed-loop runs (logs from {EXPERIMENTS})")
            tm = _timing_summary()
            out["timing"] = tm
            print(f"  {tm['files']} runs, {tm['samples']} samples each, "
                  f"T={tm['duration']:.1f}s, dt={tm['dt_mean']*1000:.2f}"
                  f"+-{tm['dt_std']*1000:.2f} ms (jitter {tm['jitter_pct']:.1f}%)")
        else:
            print("\n3. Closed-loop runs (evaluated on the identified models)")

    runs, rows = {}, []
    for ax in AXES:
        for sc in SCENARIOS:
            for name in CONTROLLERS:
                if measured and (ax, sc, name) in measured:
                    res = measured[(ax, sc, name)]
                else:
                    plant_sc, slosh = payload_variant(plants[ax], sc)
                    c = ctrl_mod.build(name, designs[ax], plants[ax])
                    res = simulate(plant_sc, slosh, c, ax, sc)
                runs[(ax, sc, name)] = res
                _write_run_csv(res)

                for win, (t0, t1) in (("acq", (1.0, T_TRACK_END)),
                                      ("dist", (T_DIST_START, T_TOTAL))):
                    tw, ew, uw = res.window(t0, t1)
                    m = met.compute(tw, ew, uw)
                    m.update(axis=ax, scenario=sc, controller=name, window=win)
                    if win == "acq":
                        m["settling"] = met.settling_time(tw, ew, 0.5)
                    rows.append(m)
        if verbose:
            print(f"  [{ax}] {len(SCENARIOS) * len(CONTROLLERS)} runs done")

    out["metrics"] = rows

    if verbose:
        print("\n4. Composite scores")
    out["scores"] = _scores(rows, verbose)
    out["cost"] = _cost_table(designs, plants)

    if verbose:
        print("\n5. Slosh-mode stability")
    from .design import slosh_stability
    out["slosh"] = {}
    for ax in AXES:
        for sc in SCENARIOS:
            r = slosh_stability(plants[ax], designs[ax], sc)
            if r is None:
                continue
            out["slosh"][f"{ax}|{sc}"] = r
            if verbose:
                bits = []
                for n in CONTROLLERS:
                    cz = r[n]["critical_zeta"]
                    bits.append(f"{n}: zc={cz:.4f}" if cz else f"{n}: unstable")
                print(f"  {ax:5s} {sc:5s} zeta_s={r['actual']:.4f} | " + "  ".join(bits))

    _write_summary_csv(rows)
    with open(RESULTS / "summary.json", "w") as fh:
        json.dump(_jsonable(out), fh, indent=2)

    from .report import write_all_tables
    write_all_tables(out)

    from .figures import make_all_figures
    make_all_figures(out, runs, plants, designs)

    if verbose:
        print(f"\nWrote results to {RESULTS} and figures to {FIGURES}")
    return out, runs

def _design_params(designs):
    p = {}
    for ax, d in designs.items():
        p[ax] = dict(
            pid=dict(kp=d["pid"].kp, ki=d["pid"].ki, kd=d["pid"].kd, tf=d["pid"].tf),
            rst=dict(R=list(d["rst"].R), S=list(d["rst"].S), T=list(d["rst"].T)),
            lqg=dict(Kx=list(np.ravel(d["lqg"].Kx)), Ki=d["lqg"].Ki,
                     L=list(np.ravel(d["lqg"].L)), q_y=d["lqg"].q_y,
                     q_i=d["lqg"].q_i, meas_var=d["lqg"].meas_var,
                     proc_var=d["lqg"].proc_var),
            lqr=dict(Kx=list(np.ravel(d["lqr"].Kx)), Ki=d["lqr"].Ki,
                     q_y=d["lqr"].q_y, q_i=d["lqr"].q_i),
            mrac=dict(theta0=list(d["mrac"].theta0), gamma=list(d["mrac"].gamma),
                      sigma=d["mrac"].sigma, dead_zone=d["mrac"].dead_zone,
                      theta_lim=list(d["mrac"].theta_lim)),
        )
    return p

def _scores(rows, verbose=True):
    scores = {}
    for ax in AXES:
        for sc in SCENARIOS:
            for win in ("acq", "dist"):
                table = {}
                for r in rows:
                    if r["axis"] == ax and r["scenario"] == sc and r["window"] == win:
                        table[r["controller"]] = r
                if len(table) != len(CONTROLLERS):
                    continue
                s = met.composite(table)
                sens = met.weight_sensitivity(table)
                scores[f"{ax}|{sc}|{win}"] = dict(score=s, win_pct=sens)
                if verbose:
                    best = min(s, key=s.get)
                    print(f"  {ax:5s} {sc:5s} {win:4s}: " +
                          "  ".join(f"{n}={s[n]:.2f}" for n in CONTROLLERS) +
                          f"   best={best} ({sens[best]:.0f}% of weightings)")
    return scores

def _timing_summary():
    reports = [timing_report(e) for e in discover()]
    if not reports:
        return {}
    agg = lambda k: float(np.mean([r[k] for r in reports]))
    return dict(files=len(reports), samples=int(np.min([r["n"] for r in reports])),
                duration=agg("duration"), dt_mean=agg("dt_mean"),
                dt_std=agg("dt_std"), dt_max=float(np.max([r["dt_max"] for r in reports])),
                jitter_pct=agg("jitter_pct"))

def _cost_table(designs, plants):
    cost = {}
    for name in CONTROLLERS:
        c = ctrl_mod.build(name, designs["roll"], plants["roll"])
        cost[name] = dict(flops=int(c.ops),
                          us_at_600mhz=round(c.ops / 600.0, 3))
    return cost

def _write_run_csv(res):
    path = RESULTS / f"run_{res.axis}_{res.controller}_{res.scenario}.csv"
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["time_s", "base_tilt_deg", "angle_deg", "error_deg",
                    "command_deg"])
        for i in range(len(res.t)):
            w.writerow([f"{res.t[i]:.5f}", f"{res.d[i]:.4f}", f"{res.y[i]:.4f}",
                        f"{res.e[i]:.4f}", f"{res.u[i]:.4f}"])

def _write_summary_csv(rows):
    keys = ["axis", "scenario", "controller", "window", "RMSE", "MAE", "peak",
            "IAE", "ISE", "effort", "u_rms", "u_max", "sse", "in_1deg", "in_2deg"]
    with open(RESULTS / "metrics.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(keys)
        for r in rows:
            w.writerow([r.get(k, "") for k in keys])

def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.floating, np.integer)):
        return float(o)
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, float) and not np.isfinite(o):
        return None
    return o

if __name__ == "__main__":
    t0 = time.time()
    run_everything()
    print(f"total {time.time() - t0:.1f}s")
