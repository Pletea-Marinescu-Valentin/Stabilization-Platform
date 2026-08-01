from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .config import (AXES, CONTROLLERS, ROOT, SCENARIOS, TS, T_DIST_START,
                     T_TOTAL, T_TRACK_END)
from .protocol import RunResult, base_tilt

EXPERIMENTS = ROOT / "experiments"

ALIASES = {
    "time_ms": ("time_ms", "t_ms", "millis", "time"),
    "time_s": ("time_s", "t_s", "t"),
    "roll": ("roll_deg", "roll", "roll_angle"),
    "pitch": ("pitch_deg", "pitch", "pitch_angle"),
    "u_roll": ("u_roll", "control_roll", "cmd_roll", "u_roll_deg"),
    "u_pitch": ("u_pitch", "control_pitch", "cmd_pitch", "u_pitch_deg"),
    "e_roll": ("e_roll", "error_roll", "err_roll"),
    "e_pitch": ("e_pitch", "error_pitch", "err_pitch"),
}

REQUIRED = ("roll", "pitch", "u_roll", "u_pitch")

GENERATED_COLUMNS = {"base_tilt_deg", "angle_deg"}

TARGET = {"roll": -175.44, "pitch": -0.20}

def detect_schema(fieldnames):
    names = {n.strip().lower(): n for n in (fieldnames or [])}

    if GENERATED_COLUMNS & set(names):
        raise ValueError(
            "this is a file written by tools/run_all.py, not a hardware log "
            "(columns " + ", ".join(sorted(GENERATED_COLUMNS & set(names))) +
            "). Hardware logs come off the SD card with a time_ms column.")

    schema = {}
    for key, options in ALIASES.items():
        for opt in options:
            if opt in names:
                schema[key] = names[opt]
                break
        else:
            schema[key] = None

    if schema["time_ms"] is None and schema["time_s"] is None:
        raise ValueError("no time column (expected time_ms)")
    missing = [k for k in REQUIRED if schema[k] is None]
    if missing:
        raise ValueError(f"missing required columns: {missing}")
    return schema

def _wrap180(x):
    return np.rad2deg(np.angle(np.exp(1j * np.deg2rad(x))))

@dataclass
class Experiment:
    controller: str
    scenario: str
    path: Path

def discover(folder: Path = EXPERIMENTS):
    if not folder.is_dir():
        return []
    found = []
    pat = re.compile(r"(?:log_)?(?P<ctrl>[a-z]+)[_-](?P<scen>[a-z0-9]+)\.csv$", re.I)
    for f in sorted(folder.glob("*.csv")):
        m = pat.search(f.name.lower())
        if not m:
            continue
        ctrl, scen = m.group("ctrl"), m.group("scen")
        if ctrl == "adaptive":
            ctrl = "mrac"
        if scen in ("threequarters", "34", "full34"):
            scen = "full"
        if ctrl in CONTROLLERS and scen in SCENARIOS:
            found.append(Experiment(ctrl, scen, f))
    return found

def load_run(exp: Experiment, axis: str, ts=TS) -> RunResult:
    with open(exp.path) as fh:
        reader = csv.DictReader(fh)
        schema = detect_schema(reader.fieldnames)
        rows = list(reader)

    def col(key):
        name = schema.get(key)
        if name is None:
            return None
        return np.array([float(r[name]) for r in rows])

    if schema["time_ms"] is not None:
        t = col("time_ms") / 1000.0
    else:
        t = col("time_s")
    t = t - t[0]
    y_raw = col(axis)
    u = col(f"u_{axis}")

    e = col(f"e_{axis}")
    if e is None:
        e = _wrap180(TARGET[axis] - y_raw)

    # Resample onto the uniform analysis grid; hardware logs are close to
    # uniform but not exactly, and every metric here assumes a fixed step.
    n = int(round(min(t[-1], T_TOTAL) / ts))
    grid = np.arange(n) * ts
    e_u = np.interp(grid, t, e)
    u_u = np.interp(grid, t, u)
    y_u = np.interp(grid, t, y_raw)

    d = base_tilt(grid)
    return RunResult(t=grid, r=np.zeros(n), y=y_u, u=u_u, d=d, e=e_u,
                     axis=axis, controller=exp.controller,
                     scenario=exp.scenario)

def timing_report(exp: Experiment):
    with open(exp.path) as fh:
        rows = list(csv.DictReader(fh))
    key = "time_ms" if "time_ms" in {n.strip().lower() for n in rows[0]} else "time_s"
    div = 1000.0 if key == "time_ms" else 1.0
    t = np.array([float(r[k]) for r in rows for k in rows[0] if k.strip().lower() == key]) / div
    dt = np.diff(t)
    return dict(n=len(t), duration=float(t[-1] - t[0]),
                dt_mean=float(dt.mean()), dt_std=float(dt.std()),
                dt_max=float(dt.max()),
                jitter_pct=float(100 * dt.std() / max(dt.mean(), 1e-9)))

def collect(folder: Path = EXPERIMENTS, verbose=True):
    exps = discover(folder)
    if not exps:
        return None
    runs = {}
    for exp in exps:
        for axis in AXES:
            try:
                runs[(axis, exp.scenario, exp.controller)] = load_run(exp, axis)
            except Exception as err:
                if verbose:
                    print(f"  skip {exp.path.name} [{axis}]: {err}")
    if verbose:
        have = sorted({(c, s) for (_, s, c) in runs})
        print(f"  loaded {len(runs)} axis-runs from {len(exps)} files")
        missing = [(c, s) for c in CONTROLLERS for s in SCENARIOS
                   if (c, s) not in have]
        if missing:
            print(f"  missing: {missing}")
    return runs

if __name__ == "__main__":
    exps = discover()
    if not exps:
        print(f"no experiment logs found in {EXPERIMENTS}")
        print("expected files named  <controller>_<scenario>.csv, e.g. pid_half.csv")
        print(f"controllers: {CONTROLLERS}")
        print(f"scenarios:   {list(SCENARIOS)}")
    else:
        bad = 0
        for e in exps:
            try:
                tr = timing_report(e)
            except Exception as err:
                print(f"{e.path.name:30s} REJECTED: {err}")
                bad += 1
                continue
            print(f"{e.path.name:30s} {e.controller:5s} {e.scenario:6s} "
                  f"n={tr['n']:5d} T={tr['duration']:6.1f}s "
                  f"dt={tr['dt_mean']*1000:.2f}+-{tr['dt_std']*1000:.2f} ms "
                  f"(jitter {tr['jitter_pct']:.1f}%)")
        if bad:
            print(f"\n{bad} of {len(exps)} files rejected; nothing will be "
                  f"ingested from them.")
