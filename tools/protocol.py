from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .config import T_TOTAL, T_TRACK_END

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
    u: np.ndarray
    d: np.ndarray
    e: np.ndarray
    axis: str
    controller: str
    scenario: str
    y_meas: np.ndarray = None
    theta: np.ndarray = None

    def window(self, t0, t1):
        m = (self.t >= t0) & (self.t < t1)
        return self.t[m], self.e[m], self.u[m]
