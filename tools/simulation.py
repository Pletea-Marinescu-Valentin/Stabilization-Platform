from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import lfilter, lfilter_zi

from .config import (IMU_BIAS_WALK, IMU_NOISE_STD, IMU_QUANT, RANDOM_SEED,
                     T_DIST_START, T_TOTAL, T_TRACK_END, TS)
from . import controllers as ctrl_mod

def base_tilt(t: np.ndarray) -> np.ndarray:
    d = np.zeros_like(t)

    d += 6.0 * ((t >= 1.0) & (t < T_TRACK_END))

    def trapezoid(t0, hold, height, rise=0.4):
        seg = np.zeros_like(t)
        up = (t >= t0) & (t < t0 + rise)
        seg[up] = height * (t[up] - t0) / rise
        flat = (t >= t0 + rise) & (t < t0 + rise + hold)
        seg[flat] = height
        dn = (t >= t0 + rise + hold) & (t < t0 + 2 * rise + hold)
        seg[dn] = height * (1.0 - (t[dn] - t0 - rise - hold) / rise)
        return seg

    for t0, h in [(21.0, 5.0), (26.0, -5.0), (31.0, 7.0)]:
        d += trapezoid(t0, 2.0, h)

    rock = (t >= 36.0) & (t < 52.0)
    d[rock] += 4.0 * np.sin(2 * np.pi * 0.25 * (t[rock] - 36.0))

    sw = (t >= 52.0) & (t < 70.0)
    if sw.any():
        tt = t[sw] - 52.0
        f0, f1, T = 0.2, 4.0, 18.0
        phase = 2 * np.pi * (f0 * tt + 0.5 * (f1 - f0) / T * tt ** 2)
        d[sw] += 2.0 * np.sin(phase)

    d += 3.0 * ((t >= 71.0) & (t < T_TOTAL))
    return d

@dataclass
class RunResult:
    t: np.ndarray
    r: np.ndarray
    y: np.ndarray
    y_meas: np.ndarray
    u: np.ndarray
    d: np.ndarray
    e: np.ndarray
    axis: str
    controller: str
    scenario: str
    theta: np.ndarray = None

    def window(self, t0, t1):
        m = (self.t >= t0) & (self.t < t1)
        return self.t[m], self.e[m], self.u[m]

def simulate(plant, slosh, controller, axis, scenario, seed=RANDOM_SEED,
             t_total=T_TOTAL, ts=TS, noise=True):
    n = int(round(t_total / ts))
    t = np.arange(n) * ts
    rng = np.random.default_rng(seed)

    num, den = plant.discrete(slosh)
    num = np.asarray(num, dtype=float)
    den = np.asarray(den, dtype=float)
    order = max(len(num), len(den)) - 1
    zi = np.zeros(order)

    d = base_tilt(t)
    r = np.zeros(n)

    y = np.zeros(n)
    y_meas = np.zeros(n)
    u = np.zeros(n)
    theta_log = []

    bias = 0.0
    controller.reset()

    for k in range(n):
        y_true = y[k - 1] if k else 0.0
        angle = y_true + d[k]
        if noise:
            bias += IMU_BIAS_WALK * rng.standard_normal() * np.sqrt(ts)
            meas = angle + bias + IMU_NOISE_STD * rng.standard_normal()
            meas = np.round(meas / IMU_QUANT) * IMU_QUANT
        else:
            meas = angle
        y_meas[k] = meas

        uk = controller.update(r[k], meas)
        u[k] = uk
        if isinstance(controller, ctrl_mod.MRACController):
            theta_log.append(controller.theta.copy())

        out, zi = lfilter(num, den, [uk], zi=zi)
        y[k] = float(out[0])

    y_total = y + d
    e = r - y_total
    return RunResult(t=t, r=r, y=y_total, y_meas=y_meas, u=u, d=d, e=e,
                     axis=axis, controller=controller.name, scenario=scenario,
                     theta=np.array(theta_log) if theta_log else None)
