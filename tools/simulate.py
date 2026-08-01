from __future__ import annotations

import numpy as np
from scipy.signal import lfilter, lfilter_zi

from .config import (IMU_BIAS_WALK, IMU_NOISE_STD, IMU_QUANT, RANDOM_SEED,
                     T_DIST_START, T_TOTAL, T_TRACK_END, TS)
from . import controllers as ctrl_mod
from .protocol import RunResult, base_tilt

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
