from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.signal import cont2discrete, lfilter, tf2ss

from .config import (CUP_HEIGHT, CUP_RADIUS, INPUT_DELAY_SAMPLES, PAYLOAD_RADIUS,
                     PLATFORM_INERTIA, SCENARIOS, TS)
from .identification import identify_axis

G_ACCEL = 9.81
NU_WATER = 1.0e-6
RHO_WATER = 1000.0
BESSEL_1 = 1.8412

@dataclass
class AxisPlant:

    axis: str
    K: float
    wn: float
    zeta: float
    delay: int = INPUT_DELAY_SAMPLES
    ts: float = TS
    fit_valid: float = 0.0
    noise_std: float = 0.0

    def num_den_c(self):
        num = np.array([self.K * self.wn ** 2])
        den = np.array([1.0, 2 * self.zeta * self.wn, self.wn ** 2])
        return num, den

    def discrete(self, slosh: "SloshMode | None" = None):
        s = complex(-self.zeta * self.wn,
                    self.wn * np.sqrt(max(1.0 - self.zeta ** 2, 0.0)))
        p = np.exp(s * self.ts)
        den = np.real(np.poly([p, np.conj(p)]))
        num = np.array([self.K * den.sum()])

        if slosh is not None:
            bs, as_, _ = cont2discrete((slosh.num(), slosh.den()), self.ts, method="zoh")
            bs = np.atleast_1d(bs.squeeze())
            num = np.polymul(num, bs)
            den = np.polymul(den, as_)
            num = num * (self.K * den.sum() / num.sum())

        num = np.concatenate([np.zeros(self.delay), num])
        return num, den

    def simulate(self, u, slosh=None):
        num, den = self.discrete(slosh)
        return lfilter(num, den, u)

    def state_space(self, slosh=None):
        num, den = self.discrete(slosh)
        A, B, C, D = tf2ss(num, den)
        return A, B, np.atleast_2d(C), np.atleast_2d(D)

    @property
    def delay_seconds(self):
        return self.delay * self.ts

def axis_plant_from_identification(axis: str, verbose=True) -> AxisPlant:
    model = identify_axis(axis, na=2, nb=1, verbose=False)
    wn, zeta = model.continuous_modes()
    plant = AxisPlant(axis=axis, K=model.dc_gain, wn=float(wn[0]), zeta=float(zeta[0]),
                      delay=model.delay, ts=TS, fit_valid=model.fit_valid,
                      noise_std=model.noise_std)
    if verbose:
        print(f"[{axis}] K={plant.K:+.3f} deg/deg   wn={plant.wn:.2f} rad/s "
              f"({plant.wn/2/np.pi:.2f} Hz)   zeta={plant.zeta:.3f}   "
              f"tau={plant.delay_seconds*1000:.1f} ms   fit(valid)={plant.fit_valid:.1f}%")
    return plant

@dataclass
class SloshMode:

    w_zero: float
    w_pole: float
    zeta_s: float
    mass_ratio: float

    def num(self):
        return np.array([1.0, 2 * self.zeta_s * self.w_zero, self.w_zero ** 2])

    def den(self):
        return np.array([1.0, 2 * self.zeta_s * self.w_pole, self.w_pole ** 2])

def slosh_mode(fill: float, total_inertia: float) -> SloshMode | None:
    if fill <= 0.0:
        return None

    R, h = CUP_RADIUS, fill * CUP_HEIGHT
    k = BESSEL_1 / R
    w = float(np.sqrt(G_ACCEL * k * np.tanh(k * h)))

    base = 0.83 * np.sqrt(NU_WATER / (np.sqrt(G_ACCEL) * R ** 1.5))
    hr = h / R
    bracket = 1.0 + (0.318 / np.sinh(BESSEL_1 * hr)) * (
        (1.0 - hr) / np.cosh(BESSEL_1 * hr) + 1.0)
    zeta_s = float(base * bracket)

    m_liquid = RHO_WATER * np.pi * R ** 2 * h
    m1 = m_liquid * (2.0 * R * np.tanh(BESSEL_1 * hr / 2.0)
                     / (BESSEL_1 * h * (BESSEL_1 ** 2 - 1.0)))
    m1 = float(min(m1, 0.8 * m_liquid))
    mu = float(m1 * PAYLOAD_RADIUS ** 2 / total_inertia)

    return SloshMode(w_zero=w, w_pole=w * np.sqrt(1.0 + mu), zeta_s=zeta_s, mass_ratio=mu)

def scenario_inertia(mass_g: float) -> float:
    return PLATFORM_INERTIA + (mass_g / 1000.0) * PAYLOAD_RADIUS ** 2

def payload_variant(nominal: AxisPlant, scenario: str):
    spec = SCENARIOS[scenario]
    j_ref = scenario_inertia(SCENARIOS["empty"]["mass_g"])
    j = scenario_inertia(spec["mass_g"])
    scale = float(np.sqrt(j_ref / j))

    plant = AxisPlant(axis=nominal.axis, K=nominal.K, wn=nominal.wn * scale,
                      zeta=nominal.zeta * scale, delay=nominal.delay, ts=nominal.ts,
                      noise_std=nominal.noise_std)
    return plant, slosh_mode(spec["fill"], j)

def describe_scenarios(nominal: dict[str, AxisPlant]):
    rows = []
    for key, spec in SCENARIOS.items():
        j = scenario_inertia(spec["mass_g"])
        sl = slosh_mode(spec["fill"], j)
        row = dict(key=key, label=spec["label"], mass=spec["mass_g"], inertia=j)
        for ax, nom in nominal.items():
            p, _ = payload_variant(nom, key)
            row[f"wn_{ax}"], row[f"zeta_{ax}"] = p.wn, p.zeta
        row.update(w_slosh=sl.w_zero if sl else float("nan"),
                   zeta_slosh=sl.zeta_s if sl else float("nan"),
                   mu=sl.mass_ratio if sl else 0.0)
        rows.append(row)
    return rows

def fit_all(verbose=True) -> dict[str, AxisPlant]:
    return {ax: axis_plant_from_identification(ax, verbose) for ax in ("roll", "pitch")}

if __name__ == "__main__":
    plants = fit_all()
    print("\nPayload scenarios")
    print(f"{'case':<8}{'m[g]':>7}{'J[kg m2]':>11}{'wn_roll':>9}{'z_roll':>8}"
          f"{'wn_pitch':>10}{'z_pitch':>9}{'w_slosh':>9}{'z_slosh':>9}{'mu':>8}")
    for r in describe_scenarios(plants):
        print(f"{r['key']:<8}{r['mass']:>7.0f}{r['inertia']:>11.5f}{r['wn_roll']:>9.2f}"
              f"{r['zeta_roll']:>8.3f}{r['wn_pitch']:>10.2f}{r['zeta_pitch']:>9.3f}"
              f"{r['w_slosh']:>9.2f}{r['zeta_slosh']:>9.4f}{r['mu']:>8.4f}")
