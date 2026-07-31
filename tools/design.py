from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_discrete_are
from scipy.optimize import least_squares
from scipy.signal import lfilter, tf2ss

from .config import TS, WC, ZETA
from .plant import AxisPlant

STEP_HORIZON = 4.0
DERIVATIVE_FILTER_SAMPLES = 4.0

def target_poles(ts=TS, wc=WC, zeta=ZETA):
    wd = wc * np.sqrt(max(1.0 - zeta ** 2, 0.0))
    s = np.array([complex(-zeta * wc, wd), complex(-zeta * wc, -wd)])
    return np.exp(s * ts)

def reference_model(ts=TS, wc=WC, zeta=ZETA):
    am = np.real(np.poly(target_poles(ts, wc, zeta)))
    bm = np.array([0.0, float(am.sum())])
    return am, bm

def target_polynomial(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA):
    _, den_p = plant.discrete()
    dominant = np.real(np.poly(target_poles(ts, wc, zeta)))
    return np.polymul(dominant, np.asarray(den_p, dtype=float))

def reference_step(n, ts=TS, wc=WC, zeta=ZETA):
    am, bm = reference_model(ts, wc, zeta)
    return lfilter(bm, am, np.ones(n))

def reference_bandwidth(wc=WC, zeta=ZETA):
    r = 1.0 - 2.0 * zeta ** 2
    return float(wc * np.sqrt(r + np.sqrt(r ** 2 + 1.0)))

def loop_metrics(plant: AxisPlant, ctrl_resp, ts=TS):
    w = np.logspace(-2.0, np.log10(np.pi / ts) - 1e-9, 1400)
    L = -ctrl_resp(w) * plant_response(plant, w, ts=ts)
    S = 1.0 / (1.0 + L)
    T = L / (1.0 + L)
    mT = np.abs(T)
    below = np.where(mT < mT[0] / np.sqrt(2.0))[0]
    bw = float(w[below[0]]) if len(below) else float(w[-1])
    return bw, float(np.max(np.abs(S))), float(np.max(mT))

def plant_tf(plant: AxisPlant, slosh=None):
    return plant.discrete(slosh)

def plant_ss(plant: AxisPlant, slosh=None):
    num, den = plant.discrete(slosh)
    A, B, C, D = tf2ss(num, den)
    return (np.atleast_2d(A), np.atleast_2d(B).reshape(-1, 1),
            np.atleast_2d(C), np.atleast_2d(D))

def _polyval_inv(p, z):
    p = np.asarray(p, dtype=float)
    return sum(c * z ** (-i) for i, c in enumerate(p))

def plant_response(plant, w, slosh=None, ts=TS):
    num, den = plant_tf(plant, slosh)
    z = np.exp(1j * w * ts)
    return _polyval_inv(num, z) / _polyval_inv(den, z)

@dataclass
class LoopAnalysis:
    name: str
    wn: float
    zeta: float
    gain_margin_db: float
    phase_margin_deg: float
    crossover: float
    bandwidth: float
    max_pole: float
    modulus_margin: float = np.nan
    settling: float = np.nan
    overshoot: float = np.nan

def analyse_loop(name, plant: AxisPlant, ctrl_resp, cl_poles, ts=TS,
                 step=None) -> LoopAnalysis:
    w = np.logspace(-2.5, np.log10(np.pi / ts) - 1e-6, 20000)
    L = -ctrl_resp(w) * plant_response(plant, w, ts=ts)
    mag, ph = np.abs(L), np.unwrap(np.angle(L))

    pms = []
    for i in np.where(np.diff(np.sign(mag - 1.0)))[0]:
        val = float(np.rad2deg(np.pi + ph[i]))
        while val > 180:
            val -= 360
        while val <= -180:
            val += 360
        pms.append((float(w[i]), val))
    if pms:
        wcx, pm = min(pms, key=lambda t: t[1])
    else:
        wcx, pm = np.nan, np.nan

    gms = []
    im = np.imag(L)
    re = np.real(L)
    for i in np.where(np.diff(np.sign(im)))[0]:
        t = im[i] / (im[i] - im[i + 1]) if im[i] != im[i + 1] else 0.0
        rr = re[i] + t * (re[i + 1] - re[i])
        if rr < 0 and abs(rr) < 1.0:
            gms.append(-20 * np.log10(max(abs(rr), 1e-12)))
    gm = float(min(gms)) if gms else np.inf

    sens = np.abs(1.0 / (1.0 + L))
    ms = float(np.max(sens))

    T = L / (1.0 + L)
    mT = np.abs(T)
    bw = np.nan
    below = np.where(mT < mT[0] / np.sqrt(2.0))[0]
    if len(below):
        bw = float(w[below[0]])

    poles = np.asarray(cl_poles)
    poles = poles[np.abs(poles) > 1e-9]
    dom = poles[np.argmax(np.abs(poles))]
    s = np.log(complex(dom)) / ts
    wn = float(abs(s))
    zt = float(-np.real(s) / wn) if wn > 0 else 0.0

    ov, ts_set = np.nan, np.nan
    if step is not None and len(step):
        final = step[-1]
        if abs(final) > 1e-9:
            ov = float(100.0 * (np.max(step) - final) / abs(final))
            band = np.abs(step - final) > 0.02 * abs(final)
            ts_set = float(np.max(np.where(band)[0]) * ts) if band.any() else 0.0

    return LoopAnalysis(name=name, wn=wn, zeta=zt, gain_margin_db=gm,
                        phase_margin_deg=pm, crossover=wcx, bandwidth=bw,
                        max_pole=float(np.max(np.abs(poles))),
                        modulus_margin=ms, settling=ts_set, overshoot=ov)

@dataclass
class PIDGains:
    kp: float
    ki: float
    kd: float
    tf: float
    ts: float = TS

    def tf_polys(self):
        a = self.tf / (self.ts + self.tf)
        kd_gain = self.kd / (self.ts + self.tf)
        num = np.array([self.kp])
        den = np.array([1.0])
        num = np.polyadd(np.polymul(num, [1.0, -1.0]),
                         np.polymul(den, [self.ki * self.ts]))
        den = np.polymul(den, [1.0, -1.0])
        num = np.polyadd(np.polymul(num, [1.0, -a]),
                         np.polymul(den, [kd_gain, -kd_gain]))
        den = np.polymul(den, [1.0, -a])
        return num, den

    def response(self, w, ts=None):
        ts = self.ts if ts is None else ts
        num, den = self.tf_polys()
        z = np.exp(1j * w * ts)
        return -_polyval_inv(num, z) / _polyval_inv(den, z)

def _cl_poly(plant, num_c, den_c):
    num_p, den_p = plant_tf(plant)
    return np.polyadd(np.polymul(den_p, den_c), np.polymul(num_p, num_c))

def _cl_step(plant, num_c, den_c, n):
    num_p, den_p = plant_tf(plant)
    num_cl = np.polymul(num_p, num_c)
    den_cl = np.polyadd(np.polymul(den_p, den_c), num_cl)
    return lfilter(num_cl, den_cl, np.ones(n))

def design_pid(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA, fast=False) -> PIDGains:
    n = int(STEP_HORIZON / ts)
    ref = reference_step(n, ts, wc, zeta)
    bw_target = reference_bandwidth(wc, zeta)
    sign = np.sign(plant.K)
    tf_const = DERIVATIVE_FILTER_SAMPLES * ts

    def unpack(p):
        return PIDGains(kp=sign * abs(p[0]), ki=sign * abs(p[1]),
                        kd=sign * abs(p[2]), tf=tf_const, ts=ts)

    stride = max(1, n // 48)
    ref_d = ref[::stride]

    def residual(p):
        g = unpack(p)
        num_c, den_c = g.tf_polys()
        if np.max(np.abs(np.roots(_cl_poly(plant, num_c, den_c)))) >= 0.999:
            return np.full(len(ref_d) + 2, 10.0)
        y = _cl_step(plant, num_c, den_c, n)
        if not np.all(np.isfinite(y)):
            return np.full(len(ref_d) + 2, 10.0)
        bw, ms, mt = loop_metrics(plant, g.response, ts)
        return np.concatenate([
            [8.0 * np.log(max(bw, 1e-6) / bw_target)],
            [6.0 * max(ms - (MS_TARGET - 0.03), 0.0)],
            0.35 * (y[::stride] - ref_d),
        ])

    scale = 1.0 / max(abs(plant.K), 1e-6)
    if fast:
        starts = [(0.05, 1.5, 0.01), (0.3, 4.0, 0.01)]
        nfev = 300
    else:
        starts = [(kp, ki, kd) for kp in (0.01, 0.05, 0.2, 0.6)
                  for ki in (0.5, 1.5, 4.0) for kd in (0.002, 0.02)]
        nfev = 2000

    best, best_cost = None, np.inf
    for kp0, ki0, kd0 in starts:
        p0 = np.array([kp0 * scale, ki0 * scale, kd0 * scale])
        try:
            sol = least_squares(residual, p0, method="trf",
                                bounds=(0, np.inf), max_nfev=nfev)
        except Exception:
            continue
        if best is None or sol.cost < best_cost:
            best, best_cost = sol, sol.cost
    return unpack(best.x)

@dataclass
class RSTPolys:
    R: np.ndarray
    S: np.ndarray
    T: np.ndarray
    ts: float = TS

    def response(self, w, ts=None):
        ts = self.ts if ts is None else ts
        z = np.exp(1j * w * ts)
        return -_polyval_inv(self.S, z) / _polyval_inv(self.R, z)

def design_rst(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA) -> RSTPolys:
    num_p, den_p = plant_tf(plant)
    A = np.asarray(den_p, dtype=float)
    B = np.asarray(num_p, dtype=float)

    integ = np.array([1.0, -1.0])
    A_int = np.polymul(A, integ)

    P = target_polynomial(plant, ts, wc, zeta)

    nB = len(B) - 1
    nA = len(A_int) - 1
    nr = len(P) - 1 - nA
    ns = len(P) - 1 - nB
    nrows = max(nA + nr, nB + ns) + 1
    M = np.zeros((nrows, (nr + 1) + (ns + 1)))
    for i in range(nr + 1):
        M[i:i + len(A_int), i] = A_int
    for i in range(ns + 1):
        M[i:i + len(B), nr + 1 + i] = B

    rhs = np.zeros(nrows)
    rhs[:len(P)] = P

    sol, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    Rt, S = sol[:nr + 1], sol[nr + 1:]
    R = np.polymul(integ, Rt)

    if abs(R[0]) > 1e-12:
        S, R = S / R[0], R / R[0]

    Ao = np.asarray(A, dtype=float)
    b_dc = float(B.sum())
    Am = np.real(np.poly(target_poles(ts, wc, zeta)))
    t0 = float(Am.sum()) / b_dc if abs(b_dc) > 1e-12 else 1.0
    T = t0 * Ao
    return RSTPolys(R=R, S=S, T=T, ts=ts)

@dataclass
class LQGDesign:
    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    Kx: np.ndarray
    Ki: float
    L: np.ndarray
    ts: float = TS
    q_y: float = 0.0
    q_i: float = 0.0
    meas_var: float = 0.0
    proc_var: float = 0.0
    ltr: float = 1.0

    def controller_ss(self):
        A, B, C, L = self.A, self.B, self.C, self.L
        n = A.shape[0]

        Ac = np.block([[A - B @ self.Kx - L @ C, -B * self.Ki],
                       [np.zeros((1, n)), np.ones((1, 1))]])
        Bcy = np.vstack([L, [[-self.ts]]])
        Bcr = np.vstack([np.zeros((n, 1)), [[self.ts]]])
        Cc = np.hstack([-self.Kx, [[-self.Ki]]])
        Dcy = np.zeros((1, 1))
        Dcr = np.zeros((1, 1))
        return Ac, Bcy, Bcr, Cc, Dcy, Dcr

    def separation_poles(self):
        n = self.A.shape[0]
        Aa = np.block([[self.A, np.zeros((n, 1))],
                       [-self.C * self.ts, np.ones((1, 1))]])
        Ba = np.vstack([self.B, np.zeros((1, 1))])
        Kaug = np.hstack([self.Kx, [[self.Ki]]])
        reg = np.linalg.eigvals(Aa - Ba @ Kaug)
        obs = np.linalg.eigvals(self.A - self.L @ self.C)
        return np.concatenate([reg, obs])

    def response(self, w, ts=None):
        ts = self.ts if ts is None else ts
        Ac, Bcy, _, Cc, _, _ = self.controller_ss()
        z = np.exp(1j * np.asarray(w) * ts)
        I = np.eye(Ac.shape[0])
        return np.array([(Cc @ np.linalg.solve(zz * I - Ac, Bcy))[0, 0] for zz in z])

    def closed_loop(self, plant: AxisPlant, slosh=None):
        Ap, Bp, Cp, _ = plant_ss(plant, slosh)
        Ac, Bcy, _, Cc, _, _ = self.controller_ss()
        nx, nc = Ap.shape[0], Ac.shape[0]
        Acl = np.zeros((nx + nc, nx + nc))
        Acl[:nx, :nx] = Ap
        Acl[:nx, nx:] = Bp @ Cc
        Acl[nx:, :nx] = Bcy @ Cp
        Acl[nx:, nx:] = Ac
        return Acl

    def step(self, plant: AxisPlant, n, slosh=None):
        Ap, Bp, Cp, _ = plant_ss(plant, slosh)
        Ac, Bcy, Bcr, Cc, _, _ = self.controller_ss()
        x = np.zeros((Ap.shape[0], 1))
        xc = np.zeros((Ac.shape[0], 1))
        y = np.zeros(n)
        for k in range(n):
            yk = (Cp @ x).item()
            y[k] = yk
            u = (Cc @ xc).item()
            xc = Ac @ xc + Bcy * yk + Bcr * 1.0
            x = Ap @ x + Bp * u
        return y

def design_lqg(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA, ltr=1.0) -> LQGDesign:
    A, B, C, _ = plant_ss(plant)
    n = A.shape[0]

    Aa = np.block([[A, np.zeros((n, 1))], [-C * ts, np.ones((1, 1))]])
    Ba = np.vstack([B, np.zeros((1, 1))])
    Ca = np.hstack([C, np.zeros((1, 1))])

    def gains(qy, qi):
        Q = qy * (Ca.T @ Ca)
        Q[n, n] += qi
        P = solve_discrete_are(Aa, Ba, Q + 1e-9 * np.eye(n + 1), np.array([[1.0]]))
        K = np.linalg.solve(np.array([[1.0]]) + Ba.T @ P @ Ba, Ba.T @ P @ Aa)
        return K

    meas_var = float(max(plant.noise_std, 1e-3) ** 2)
    proc_var = float((0.25 * abs(plant.K)) ** 2) * float(ltr)
    Qn = proc_var * (B @ B.T) + 1e-10 * np.eye(n)
    Rn = np.array([[meas_var]])
    Pf = solve_discrete_are(A.T, C.T, Qn, Rn)
    L = (A @ Pf @ C.T) @ np.linalg.inv(C @ Pf @ C.T + Rn)

    bw_target = reference_bandwidth(wc, zeta)

    def build(p):
        qy, qi = np.exp(np.clip(p, -25, 25))
        K = gains(qy, qi)
        return LQGDesign(A=A, B=B, C=C, Kx=K[:, :n], Ki=float(K[0, n]), L=L,
                         ts=ts, q_y=qy, q_i=qi, meas_var=meas_var,
                         proc_var=proc_var)

    def residual(p):
        try:
            d = build(p)
            if np.max(np.abs(np.linalg.eigvals(d.closed_loop(plant)))) >= 0.999:
                return np.array([10.0, 10.0])
            bw, ms, mt = loop_metrics(plant, d.response, ts)
        except Exception:
            return np.array([10.0, 10.0])
        return np.array([4.0 * np.log(max(bw, 1e-6) / bw_target),
                         4.0 * max(ms - 1.30, 0.0)])

    best, best_cost = None, np.inf
    for q0 in ([0.0, 0.0], [-2.0, 4.0], [4.0, 0.0]):
        try:
            sol = least_squares(residual, np.array(q0), method="lm", max_nfev=400)
        except Exception:
            continue
        if best is None or sol.cost < best_cost:
            best, best_cost = sol, sol.cost
    if best is None:
        raise RuntimeError("LQG weight selection failed to converge")
    out = build(best.x)
    out.ltr = float(ltr)
    return out

@dataclass
class MRACDesign:
    am: np.ndarray
    bm: np.ndarray
    theta0: np.ndarray
    gamma: np.ndarray
    sigma: float
    theta_lim: np.ndarray
    dead_zone: float
    lam: np.ndarray
    ts: float = TS
    baseline: RSTPolys = None

    def response(self, w, ts=None):
        return self.baseline.response(w, ts)

def design_mrac(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA,
                adapt_time=60.0, authority=0.4) -> MRACDesign:
    am, bm = reference_model(ts, wc, zeta)
    baseline = design_rst(plant, ts, wc, zeta)

    R, S, T = baseline.R, baseline.S, baseline.T
    theta0 = np.concatenate([-R[1:], -S, T])

    scale = np.abs(theta0) + 0.01 * np.max(np.abs(theta0))
    gamma = (ts / max(adapt_time, ts)) * scale
    theta_lim = authority * np.abs(theta0) + 0.02 * np.max(np.abs(theta0))

    lam = np.array([1.0, -float(np.exp(-wc * ts))])
    return MRACDesign(am=am, bm=bm, theta0=theta0, gamma=gamma, sigma=0.05,
                      theta_lim=theta_lim, dead_zone=0.15, lam=lam, ts=ts,
                      baseline=baseline)

def design_all(plant: AxisPlant, ts=TS, wc=WC, zeta=ZETA):
    return dict(pid=design_pid(plant, ts, wc, zeta),
                rst=design_rst(plant, ts, wc, zeta),
                lqg=design_lqg(plant, ts, wc, zeta),
                mrac=design_mrac(plant, ts, wc, zeta))

def _ms_of(plant, name, wc, zeta, ts):
    if name == "pid":
        d = design_pid(plant, ts, wc, zeta)
        resp = d.response
        poles = np.roots(_cl_poly(plant, *d.tf_polys()))
    elif name in ("rst", "mrac"):
        d = design_rst(plant, ts, wc, zeta)
        resp = d.response
        num_p, den_p = plant_tf(plant)
        poles = np.roots(np.polyadd(np.polymul(den_p, d.R), np.polymul(num_p, d.S)))
    elif name == "lqg":
        d = design_lqg(plant, ts, wc, zeta)
        resp = d.response
        poles = np.linalg.eigvals(d.closed_loop(plant))
    else:
        raise KeyError(name)
    if np.max(np.abs(poles)) >= 0.9995:
        return np.inf, d
    bw, ms, mt = loop_metrics(plant, resp, ts)
    return ms, mt, d

def controller_response(name, designs):
    d = designs[name]
    if name == "mrac":
        d = d.baseline
    return d.response

def _closed_loop_poles_named(plant, slosh, designs, name, ts=TS):
    return closed_loop_max_pole(plant, slosh, designs, name, ts)

CONTROLLERS_FRONTIER = ("pid", "rst", "lqg", "lqr", "mrac")

def robustness_frontier(plant: AxisPlant, designs, ts=TS, ms_limit=2.0):
    from .plant import AxisPlant as _AP

    def ok(p_pert, resp):
        w = np.logspace(-2.5, np.log10(np.pi / ts) - 1e-9, 3000)
        L = -resp(w) * plant_response(p_pert, w, ts=ts)
        if np.any(~np.isfinite(L)):
            return False
        return float(np.max(np.abs(1.0 / (1.0 + L)))) <= ms_limit

    out = {}
    for name in CONTROLLERS_FRONTIER:
        resp = controller_response(name, designs)

        gains = []
        for g in np.linspace(1.0, 4.0, 61)[1:]:
            up = _AP(plant.axis, plant.K * g, plant.wn, plant.zeta,
                     plant.delay, ts, noise_std=plant.noise_std)
            dn = _AP(plant.axis, plant.K / g, plant.wn, plant.zeta,
                     plant.delay, ts, noise_std=plant.noise_std)
            if not (ok(up, resp) and ok(dn, resp)):
                break
            gains.append(g)
        gain_tol = gains[-1] if gains else 1.0

        freqs = []
        for f in np.linspace(1.0, 2.5, 61)[1:]:
            up = _AP(plant.axis, plant.K, plant.wn * f, plant.zeta,
                     plant.delay, ts, noise_std=plant.noise_std)
            dn = _AP(plant.axis, plant.K, plant.wn / f, plant.zeta,
                     plant.delay, ts, noise_std=plant.noise_std)
            if not (ok(up, resp) and ok(dn, resp)):
                break
            freqs.append(f)
        freq_tol = freqs[-1] if freqs else 1.0

        out[name] = dict(gain_ratio=float(gain_tol),
                         gain_db=float(20 * np.log10(gain_tol)),
                         freq_ratio=float(freq_tol),
                         freq_pct=float(100 * (freq_tol - 1.0)))
    return out

def closed_loop_max_pole(plant, slosh, designs, name, ts=TS):
    num_p, den_p = plant.discrete(slosh)
    if name == "pid":
        nc, dc = designs["pid"].tf_polys()
        char = np.polyadd(np.polymul(den_p, dc), np.polymul(num_p, nc))
        return float(np.max(np.abs(np.roots(char))))
    if name in ("rst", "mrac"):
        b = designs["rst"]
        char = np.polyadd(np.polymul(den_p, b.R), np.polymul(num_p, b.S))
        return float(np.max(np.abs(np.roots(char))))

    key = "lqr" if name == "lqr" else "lqg"
    cs = designs[key].controller_ss()
    Ac, Bcy, Cc = cs[0], cs[1], cs[3]
    Dcy = cs[4] if key == "lqr" else np.zeros((1, 1))
    Ap, Bp, Cp, _ = tf2ss(num_p, den_p)
    Ap, Cp = np.atleast_2d(Ap), np.atleast_2d(Cp)
    Bp = np.atleast_2d(Bp).reshape(-1, 1)
    nx, nc_ = Ap.shape[0], Ac.shape[0]
    Acl = np.zeros((nx + nc_, nx + nc_))
    Acl[:nx, :nx] = Ap + Bp @ Dcy @ Cp
    Acl[:nx, nx:] = Bp @ Cc
    Acl[nx:, :nx] = Bcy @ Cp
    Acl[nx:, nx:] = Ac
    return float(np.max(np.abs(np.linalg.eigvals(Acl))))

def slosh_stability(plant, designs, scenario, ts=TS, z_max=0.15, n=300):
    from .plant import SloshMode, payload_variant, scenario_inertia, slosh_mode
    from .config import SCENARIOS

    spec = SCENARIOS[scenario]
    base = slosh_mode(spec["fill"], scenario_inertia(spec["mass_g"]))
    p_sc, _ = payload_variant(plant, scenario)
    if base is None:
        return None

    out = {"actual": base.zeta_s, "w_slosh": base.w_zero, "mu": base.mass_ratio}
    for name in CONTROLLERS_FRONTIER:
        crit = None
        for z in np.linspace(0.001, z_max, n):
            probe = SloshMode(base.w_zero, base.w_pole, float(z), base.mass_ratio)
            if closed_loop_max_pole(p_sc, probe, designs, name, ts) < 1.0:
                crit = float(z)
                break
        out[name] = dict(
            critical_zeta=crit,
            margin=(base.zeta_s / crit) if crit else None,
            max_pole=closed_loop_max_pole(p_sc, base, designs, name, ts),
        )
    return out

MS_TARGET = 1.30
LTR_GRID = (1.0, 10.0, 100.0, 1000.0)

def design_lqg_max_bandwidth(plant: AxisPlant, ms_target=1.30, ts=TS,
                             ltr=1.0, n=26):
    A, B, C, _ = plant_ss(plant)
    nx = A.shape[0]
    Aa = np.block([[A, np.zeros((nx, 1))], [-C * ts, np.ones((1, 1))]])
    Ba = np.vstack([B, np.zeros((1, 1))])
    Ca = np.hstack([C, np.zeros((1, 1))])

    meas_var = float(max(plant.noise_std, 1e-3) ** 2)
    proc_var = float((0.25 * abs(plant.K)) ** 2) * float(ltr)
    Qn = proc_var * (B @ B.T) + 1e-10 * np.eye(nx)
    Rn = np.array([[meas_var]])
    Pf = solve_discrete_are(A.T, C.T, Qn, Rn)
    L = (A @ Pf @ C.T) @ np.linalg.inv(C @ Pf @ C.T + Rn)

    best = None
    for lqy in np.linspace(-6, 6, n):
        for lqi in np.linspace(-4, 10, n):
            try:
                Q = np.exp(lqy) * (Ca.T @ Ca)
                Q[nx, nx] += np.exp(lqi)
                P = solve_discrete_are(Aa, Ba, Q + 1e-9 * np.eye(nx + 1),
                                       np.array([[1.0]]))
                K = np.linalg.solve(np.array([[1.0]]) + Ba.T @ P @ Ba,
                                    Ba.T @ P @ Aa)
                d = LQGDesign(A=A, B=B, C=C, Kx=K[:, :nx], Ki=float(K[0, nx]),
                              L=L, ts=ts, q_y=float(np.exp(lqy)),
                              q_i=float(np.exp(lqi)), meas_var=meas_var,
                              proc_var=proc_var, ltr=float(ltr))
                if np.max(np.abs(np.linalg.eigvals(d.closed_loop(plant)))) >= 0.9995:
                    continue
                bw, ms, mt = loop_metrics(plant, d.response, ts)
            except Exception:
                continue
            if ms > ms_target + 0.01:
                continue
            if best is None or bw > best[1]:
                best = (d, bw, ms, mt)
    if best is None:
        raise RuntimeError("no LQG design meets the robustness target")
    return best

def design_lqr_measured(plant: AxisPlant, ms_target=1.30, ts=TS, n=26):
    num, den = plant.discrete()
    a1, a2 = float(den[1]), float(den[2])
    b0 = float(num[plant.delay])

    A = np.array([[-a1, -a2, b0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
    B = np.array([[0.0], [0.0], [1.0]])
    C = np.array([[1.0, 0.0, 0.0]])
    nx = 3

    Aa = np.block([[A, np.zeros((nx, 1))], [-C * ts, np.ones((1, 1))]])
    Ba = np.vstack([B, np.zeros((1, 1))])
    Ca = np.hstack([C, np.zeros((1, 1))])

    best = None
    for lqy in np.linspace(-6, 8, n):
        for lqi in np.linspace(-4, 10, n):
            try:
                Q = np.exp(lqy) * (Ca.T @ Ca)
                Q[nx, nx] += np.exp(lqi)
                P = solve_discrete_are(Aa, Ba, Q + 1e-9 * np.eye(nx + 1),
                                       np.array([[1.0]]))
                K = np.linalg.solve(np.array([[1.0]]) + Ba.T @ P @ Ba,
                                    Ba.T @ P @ Aa)
                d = LQRDesign(A=A, B=B, C=C, Kx=K[:, :nx], Ki=float(K[0, nx]),
                              ts=ts, q_y=float(np.exp(lqy)),
                              q_i=float(np.exp(lqi)))
                if np.max(np.abs(np.linalg.eigvals(d.closed_loop(plant)))) >= 0.9995:
                    continue
                bw, ms, mt = loop_metrics(plant, d.response, ts)
            except Exception:
                continue
            if ms > ms_target + 0.01:
                continue
            if best is None or bw > best[1]:
                best = (d, bw, ms, mt)
    if best is None:
        raise RuntimeError("no LQR design meets the robustness target")
    return best

@dataclass
class LQRDesign:

    A: np.ndarray
    B: np.ndarray
    C: np.ndarray
    Kx: np.ndarray
    Ki: float
    ts: float = TS
    q_y: float = 0.0
    q_i: float = 0.0

    def controller_ss(self):
        A, B, C = self.A, self.B, self.C
        n = A.shape[0]
        Ac = np.zeros((3, 3))
        Kx = np.ravel(self.Kx)
        Dcy = np.array([[-Kx[0]]])
        Cc = np.array([[-Kx[1], -Kx[2], -self.Ki]])
        Ac[0, :] = [0.0, 0.0, 0.0]
        Ac[1, :] = Cc[0]
        Ac[2, :] = [0.0, 0.0, 1.0]
        Bcy = np.array([[1.0], [Dcy.item()], [-self.ts]])
        Bcr = np.array([[0.0], [0.0], [self.ts]])
        Dcr = np.zeros((1, 1))
        return Ac, Bcy, Bcr, Cc, Dcy, Dcr

    def response(self, w, ts=None):
        ts = self.ts if ts is None else ts
        Ac, Bcy, _, Cc, Dcy, _ = self.controller_ss()
        z = np.exp(1j * np.asarray(w) * ts)
        I = np.eye(Ac.shape[0])
        return np.array([Dcy.item() + (Cc @ np.linalg.solve(zz * I - Ac, Bcy))[0, 0]
                         for zz in z])

    def closed_loop(self, plant: AxisPlant, slosh=None):
        Ap, Bp, Cp, _ = plant_ss(plant, slosh)
        Ac, Bcy, _, Cc, Dcy, _ = self.controller_ss()
        nx, nc = Ap.shape[0], Ac.shape[0]
        Acl = np.zeros((nx + nc, nx + nc))
        Acl[:nx, :nx] = Ap + Bp @ Dcy @ Cp
        Acl[:nx, nx:] = Bp @ Cc
        Acl[nx:, :nx] = Bcy @ Cp
        Acl[nx:, nx:] = Ac
        return Acl

def _candidate(plant, name, wc, zeta, ts, ltr=1.0):
    if name == "pid":
        d = design_pid(plant, ts, wc, zeta, fast=True)
        resp = d.response
        poles = np.roots(_cl_poly(plant, *d.tf_polys()))
    elif name in ("rst", "mrac"):
        d = design_rst(plant, ts, wc, zeta)
        resp = d.response
        num_p, den_p = plant_tf(plant)
        poles = np.roots(np.polyadd(np.polymul(den_p, d.R), np.polymul(num_p, d.S)))
    elif name == "lqg":
        d = design_lqg(plant, ts, wc, zeta, ltr=ltr)
        resp = d.response
        poles = np.linalg.eigvals(d.closed_loop(plant))
    else:
        raise KeyError(name)
    if np.max(np.abs(poles)) >= 0.9995:
        return None
    bw, ms, mt = loop_metrics(plant, resp, ts)
    return dict(design=d, bw=bw, ms=ms, mt=mt, wc=float(wc), ltr=float(ltr))

def design_equal_robustness(plant: AxisPlant, ms_target=MS_TARGET, zeta=0.85,
                            ts=TS, wc_lo=1.0, wc_hi=26.0, n_grid=26):
    grid = np.linspace(wc_lo, wc_hi, n_grid)
    out, meta = {}, {}

    for name in ("pid", "rst"):
        best = None
        for wc in grid:
            try:
                c = _candidate(plant, name, wc, zeta, ts)
            except Exception:
                continue
            if c is None or c["ms"] > ms_target + 0.01:
                continue
            if best is None or c["bw"] > best["bw"]:
                best = c
        if best is None:
            raise RuntimeError(f"no {name} design meets Ms <= {ms_target}")
        if name == "pid":
            refit = design_pid(plant, ts, best["wc"], zeta, fast=False)
            poles = np.roots(_cl_poly(plant, *refit.tf_polys()))
            if np.max(np.abs(poles)) < 0.9995:
                bw, ms, mt = loop_metrics(plant, refit.response, ts)
                if ms <= ms_target + 0.01 and bw >= best["bw"]:
                    best = dict(design=refit, bw=bw, ms=ms, mt=mt,
                                wc=best["wc"], ltr=1.0)
        out[name] = best["design"]
        meta[name] = dict(wc=best["wc"], ms=best["ms"], mt=best["mt"],
                          bw=best["bw"], ltr=best["ltr"])

    d_lqg, bw, ms, mt = design_lqg_max_bandwidth(plant, ms_target, ts)
    out["lqg"] = d_lqg
    meta["lqg"] = dict(wc=float("nan"), ms=ms, mt=mt, bw=bw, ltr=d_lqg.ltr)

    d_lqr, bw, ms, mt = design_lqr_measured(plant, ms_target, ts)
    out["lqr"] = d_lqr
    meta["lqr"] = dict(wc=float("nan"), ms=ms, mt=mt, bw=bw, ltr=1.0)

    out["mrac"] = design_mrac(plant, ts, meta["rst"]["wc"], zeta)
    meta["mrac"] = dict(meta["rst"])
    return out, meta

MS_LIMIT = 1.6
MT_LIMIT = 1.6

def max_bandwidth_design(plant: AxisPlant, name: str, ms_limit=MS_LIMIT,
                         mt_limit=MT_LIMIT, zeta=0.8, ts=TS,
                         wc_lo=0.5, wc_hi=35.0, n_grid=40):
    grid = np.linspace(wc_lo, wc_hi, n_grid)
    best = None
    for wc in grid:
        try:
            ms, mt, d = _ms_of(plant, name, wc, zeta, ts)
        except Exception:
            continue
        if ms <= ms_limit and mt <= mt_limit:
            best = (float(wc), d, float(ms), float(mt))
    if best is None:
        scored = []
        for wc in grid:
            try:
                ms, mt, d = _ms_of(plant, name, wc, zeta, ts)
                scored.append((max(ms, mt), float(wc), d, float(ms), float(mt)))
            except Exception:
                continue
        scored.sort(key=lambda r: r[0])
        _, wc, d, ms, mt = scored[0]
        return wc, d, ms, mt
    return best

def design_all_equal_robustness(plant: AxisPlant, ms_limit=MS_LIMIT,
                                mt_limit=MT_LIMIT, zeta=0.8, ts=TS):
    out, meta = {}, {}
    for name in ("pid", "rst", "lqg"):
        wc, d, ms, mt = max_bandwidth_design(plant, name, ms_limit, mt_limit, zeta, ts)
        out[name] = d
        meta[name] = dict(wc=wc, ms=ms, mt=mt)
    wc_rst = meta["rst"]["wc"]
    out["mrac"] = design_mrac(plant, ts, wc_rst, zeta)
    meta["mrac"] = dict(**meta["rst"])
    return out, meta

def verify_designs(plant: AxisPlant, designs: dict, ts=TS, wc=WC, zeta=ZETA):
    n = int(STEP_HORIZON / ts)
    out = {}

    g = designs["pid"]
    nc, dc = g.tf_polys()
    out["pid"] = analyse_loop("PID", plant, g.response,
                              np.roots(_cl_poly(plant, nc, dc)), ts,
                              _cl_step(plant, nc, dc, n))

    r = designs["rst"]
    num_p, den_p = plant_tf(plant)
    char = np.polyadd(np.polymul(den_p, r.R), np.polymul(num_p, r.S))
    step_rst = lfilter(np.polymul(num_p, r.T), char, np.ones(n))
    out["rst"] = analyse_loop("RST", plant, r.response, np.roots(char), ts, step_rst)

    q = designs["lqg"]
    out["lqg"] = analyse_loop("LQG", plant, q.response,
                              np.linalg.eigvals(q.closed_loop(plant)), ts,
                              q.step(plant, n))

    k = designs["lqr"]
    out["lqr"] = analyse_loop("LQR", plant, k.response,
                              np.linalg.eigvals(k.closed_loop(plant)), ts)

    m = designs["mrac"]
    out["mrac"] = analyse_loop("MRAC", plant, m.response, np.roots(char), ts, step_rst)
    return out
