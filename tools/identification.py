from __future__ import annotations

import csv
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter

from .config import DATASETS, TS

def load_prbs(axis: str):
    path = DATASETS / f"{axis}_id.csv"
    with open(path) as fh:
        rows = list(csv.DictReader(fh))

    t = np.array([float(r["time_ms"]) for r in rows]) / 1000.0
    u = np.array([float(r["u_deg"]) for r in rows])
    y = np.array([float(r[f"{axis}_deg"]) for r in rows])

    y = np.rad2deg(np.unwrap(np.deg2rad(y)))

    good = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t, u, y = t[good], u[good], y[good]

    keep = t > t[0] + 1.0
    t, u, y = t[keep], u[keep], y[keep]

    ramp = np.vstack([np.ones_like(t), t - t[0]]).T
    y = y - ramp @ np.linalg.lstsq(ramp, y, rcond=None)[0]
    u = u - u.mean()

    return t - t[0], u, y

@dataclass
class AxisModel:

    axis: str
    a: np.ndarray
    b: np.ndarray
    delay: int
    ts: float
    fit_train: float = 0.0
    fit_valid: float = 0.0
    noise_std: float = 0.0
    residual: np.ndarray = field(default=None, repr=False)

    @property
    def den(self) -> np.ndarray:
        return np.concatenate([[1.0], self.a])

    @property
    def num(self) -> np.ndarray:
        return np.concatenate([np.zeros(self.delay), self.b])

    @property
    def dc_gain(self) -> float:
        return float(self.b.sum() / self.den.sum())

    def poles(self) -> np.ndarray:
        return np.roots(self.den)

    def zeros(self) -> np.ndarray:
        return np.roots(self.b) if len(self.b) > 1 else np.array([])

    def continuous_modes(self):
        s = np.log(self.poles().astype(complex)) / self.ts
        wn = np.abs(s)
        zeta = np.where(wn > 0, -np.real(s) / np.maximum(wn, 1e-12), 0.0)
        return wn, zeta

    def simulate(self, u: np.ndarray, y0: np.ndarray | None = None) -> np.ndarray:
        y = lfilter(self.num, self.den, u)
        if y0 is not None:
            start = max(len(self.a), len(self.b) + self.delay)
            y[:start] = y0[:start]
        return y

def _arx_initial(u, y, na, nb, d):
    start = max(na, nb + d)
    phi, tgt = [], []
    for k in range(start, len(y)):
        row = [-y[k - i - 1] for i in range(na)] + [u[k - d - j] for j in range(nb)]
        phi.append(row)
        tgt.append(y[k])
    theta, *_ = np.linalg.lstsq(np.array(phi), np.array(tgt), rcond=None)
    return theta

def _fit_percent(y, yhat):
    denom = np.linalg.norm(y - y.mean())
    if denom == 0:
        return 0.0
    return float(100.0 * (1.0 - np.linalg.norm(y - yhat) / denom))

def fit_oe(u, y, na, nb, d, ts=TS, axis="") -> AxisModel:
    theta0 = _arx_initial(u, y, na, nb, d)

    def residual(p):
        m = AxisModel(axis, p[:na], p[na:], d, ts)
        yhat = m.simulate(u, y)
        if not np.all(np.isfinite(yhat)):
            return np.full(len(y), 1e6)
        return yhat - y

    sol = least_squares(residual, theta0, method="lm", max_nfev=30000)
    model = AxisModel(axis, sol.x[:na], sol.x[na:], d, ts)
    yhat = model.simulate(u, y)
    model.fit_train = _fit_percent(y, yhat)
    model.residual = y - yhat
    model.noise_std = float(np.std(np.diff(model.residual)) / np.sqrt(2.0))
    return model

def select_order(u, y, na_range=(1, 5), nb_range=(1, 4), d_range=(0, 4), ts=TS, axis=""):
    split = int(0.7 * len(y))
    utr, ytr = u[:split], y[:split]
    uva, yva = u[split:], y[split:]

    table = []
    for na in range(na_range[0], na_range[1] + 1):
        for nb in range(nb_range[0], nb_range[1] + 1):
            for d in range(d_range[0], d_range[1] + 1):
                try:
                    m = fit_oe(utr, ytr, na, nb, d, ts, axis)
                    if np.max(np.abs(m.poles())) >= 1.0:
                        continue
                    yhat = m.simulate(uva, yva)
                    fv = _fit_percent(yva, yhat)
                    table.append(dict(na=na, nb=nb, d=d, fit_valid=fv, fit_train=m.fit_train))
                except Exception:
                    continue
    table.sort(key=lambda r: -r["fit_valid"])
    return table

def identify_axis(axis: str, na=2, nb=2, d=None, verbose=True) -> AxisModel:
    from .config import INPUT_DELAY_SAMPLES

    d = INPUT_DELAY_SAMPLES if d is None else d
    t, u, y = load_prbs(axis)

    split = int(0.7 * len(y))
    model = fit_oe(u[:split], y[:split], na, nb, d, TS, axis)
    model.fit_valid = _fit_percent(y[split:], model.simulate(u[split:], y[split:]))

    final = fit_oe(u, y, na, nb, d, TS, axis)
    final.fit_valid = model.fit_valid

    if verbose:
        wn, zeta = final.continuous_modes()
        print(f"[{axis}] OE({na},{nb},d={d})  fit(train)={final.fit_train:5.1f}%  "
              f"fit(valid)={final.fit_valid:5.1f}%")
        print(f"        dc gain = {final.dc_gain:+.3f} deg/deg   "
              f"wn = {np.unique(wn.round(2))} rad/s   zeta = {np.unique(zeta.round(3))}")
        print(f"        noise std = {final.noise_std:.4f} deg")
    return final

def identify_all(verbose=True) -> dict[str, AxisModel]:
    return {ax: identify_axis(ax, verbose=verbose) for ax in ("roll", "pitch")}

if __name__ == "__main__":
    for ax in ("roll", "pitch"):
        t, u, y = load_prbs(ax)
        print(f"\n===== {ax.upper()} order selection =====")
        for row in select_order(u, y, axis=ax)[:6]:
            print("   na={na} nb={nb} d={d}  fit_valid={fit_valid:6.2f}%  "
                  "fit_train={fit_train:6.2f}%".format(**row))
    print("\n===== selected models =====")
    identify_all()
