from __future__ import annotations

import csv
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import butter, csd, filtfilt, lfilter, welch
from scipy.stats import chi2

from .config import DATASETS, TS

HP_CUTOFF_HZ = 0.10
BURN_IN = 32
CV_FOLDS = 5
SETTLE_FRACTION = 0.6
MIN_SEGMENT = 12
NPERSEG = 256
DC_BAND_HZ = (0.20, 1.50)
DC_TOLERANCE = 0.35
COHERENCE_FLOOR = 0.80
SPECTRAL_MAX = 0.60
SE_CAP = 2.0
NA_RANGE = (2, 4)
NB_RANGE = (1, 3)
D_RANGE = (0, 3)
DESIGN_NA = 2
DESIGN_NB = 1


@dataclass
class Record:
    axis: str
    t: np.ndarray
    u: np.ndarray
    y: np.ndarray
    motor: np.ndarray
    ts_median: float
    ts_mean: float
    ts_std: float
    dt_max: float
    n_gaps: int
    jitter_pct: float
    quant: float
    n: int

    @property
    def duration(self):
        return float(self.t[-1] - self.t[0])


@dataclass
class AxisModel:
    axis: str
    a: np.ndarray
    b: np.ndarray
    delay: int
    ts: float
    fit_train: float = 0.0
    fit_valid: float = 0.0
    cv_mean: float = 0.0
    cv_std: float = 0.0
    noise_std: float = 0.0
    sensor_noise_std: float = 0.0
    residual: np.ndarray = field(default=None, repr=False)
    na: int = 0
    nb: int = 0
    design_form: dict = field(default=None, repr=False)

    @property
    def den(self) -> np.ndarray:
        return np.concatenate([[1.0], self.a])

    @property
    def num(self) -> np.ndarray:
        return np.concatenate([np.zeros(self.delay), self.b])

    @property
    def dc_gain(self) -> float:
        return float(self.b.sum() / self.den.sum())

    @property
    def delay_ms(self) -> float:
        return float(self.delay * self.ts * 1000.0)

    @property
    def n_par(self) -> int:
        return len(self.a) + len(self.b)

    def poles(self) -> np.ndarray:
        return np.roots(self.den)

    def zeros(self) -> np.ndarray:
        return np.roots(self.b) if len(self.b) > 1 else np.array([])

    def continuous_modes(self):
        s = np.log(self.poles().astype(complex)) / self.ts
        wn = np.abs(s)
        zeta = np.where(wn > 0, -np.real(s) / np.maximum(wn, 1e-12), 0.0)
        return wn, zeta

    def dominant_mode(self):
        wn, zeta = self.continuous_modes()
        osc = np.where(zeta < 0.999)[0]
        i = osc[np.argmin(zeta[osc])] if len(osc) else int(np.argmin(zeta))
        return float(wn[i]), float(zeta[i])

    def model_output(self, u: np.ndarray, y0: np.ndarray | None = None) -> np.ndarray:
        return lfilter(self.num, self.den, u)

    def frequency_response(self, w):
        z = np.exp(1j * np.asarray(w, dtype=float) * self.ts)
        n = sum(c * z ** (-i) for i, c in enumerate(self.num))
        d = sum(c * z ** (-i) for i, c in enumerate(self.den))
        return n / d


def load_record(axis: str) -> Record:
    path = DATASETS / f"{axis}_id.csv"
    with open(path) as fh:
        rows = list(csv.DictReader(fh))

    t = np.array([float(r["time_ms"]) for r in rows]) / 1000.0
    u = np.array([float(r["u_deg"]) for r in rows])
    y = np.array([float(r[f"{axis}_deg"]) for r in rows])
    motor = np.array([float(r.get("motor_pos", "nan") or "nan") for r in rows])

    good = np.isfinite(t) & np.isfinite(u) & np.isfinite(y)
    t, u, y, motor = t[good], u[good], y[good], motor[good]

    order = np.argsort(t)
    t, u, y, motor = t[order], u[order], y[order], motor[order]
    y = np.rad2deg(np.unwrap(np.deg2rad(y)))

    dt = np.diff(t)
    med = float(np.median(dt))
    uq = np.unique(y)
    dq = np.abs(np.diff(uq))
    dq = dq[dq > 0]

    return Record(axis=axis, t=t - t[0], u=u, y=y, motor=motor,
                  ts_median=med, ts_mean=float(np.mean(dt)),
                  ts_std=float(np.std(dt)), dt_max=float(np.max(dt)),
                  n_gaps=int(np.sum(dt > 1.5 * med)),
                  jitter_pct=float(100.0 * np.std(dt) / med),
                  quant=float(dq.min()) if len(dq) else 0.0, n=len(t))


def uniform_grid(rec: Record, ts: float | None = None):
    ts = rec.ts_mean if ts is None else ts
    grid = np.arange(0.0, rec.t[-1], ts)
    idx = np.clip(np.searchsorted(rec.t, grid, side="right") - 1, 0, rec.n - 1)
    return grid, rec.u[idx], np.interp(grid, rec.t, rec.y), ts


def high_pass(u, y, ts, cutoff=HP_CUTOFF_HZ):
    b, a = butter(2, cutoff / (0.5 / ts), btype="high")
    return filtfilt(b, a, u), filtfilt(b, a, y)


_PREPARED = {}
_IDENTIFIED = {}


def prepare_axis(axis: str, cutoff=HP_CUTOFF_HZ):
    key = (axis, cutoff)
    if key in _PREPARED:
        return _PREPARED[key]
    rec = load_record(axis)
    t, u, y, ts = uniform_grid(rec)
    start = np.argmax(u != 0.0)
    lead = max(0, start - int(round(1.0 / ts)))
    t, u, y = t[lead:], u[lead:], y[lead:]
    uf, yf = high_pass(u, y, ts, cutoff)
    _PREPARED[key] = (t - t[0], uf, yf, ts, rec)
    return _PREPARED[key]


def segments(u, min_len=MIN_SEGMENT):
    edges = np.where(np.diff(u) != 0)[0] + 1
    out = []
    for s in np.split(np.arange(len(u)), edges):
        if len(s) >= min_len:
            out.append(s)
    return out


def excitation_summary(rec: Record):
    t, u, y, ts = uniform_grid(rec)
    segs = segments(u)
    levels = np.array([u[s[0]] for s in segs])
    dwell = np.array([len(s) * ts for s in segs])
    excited = levels != 0.0
    sign_flips = int(np.sum(np.diff(np.sign(levels[excited])) != 0))
    half = dwell[excited]
    urms = float(np.sqrt(np.mean(u[u != 0] ** 2))) if np.any(u != 0) else 0.0
    return dict(
        kind="staircase square wave" if sign_flips >= 0.8 * (excited.sum() - 1)
             else "irregular",
        n_segments=int(excited.sum()),
        n_levels=int(len(np.unique(np.abs(levels[excited])))),
        amp_min=float(np.min(np.abs(levels[excited]))),
        amp_max=float(np.max(np.abs(levels[excited]))),
        dwell_min=float(half.min()), dwell_max=float(half.max()),
        dwell_mean=float(half.mean()),
        f_fundamental=float(0.5 / half.mean()),
        sign_alternations=sign_flips,
        u_rms=urms,
        crest=float(np.max(np.abs(u)) / urms) if urms else float("nan"),
        quiescent_s=float(np.sum(u == 0.0) * ts))


def static_map(rec: Record):
    t, u, y, ts = uniform_grid(rec)
    settled, level, spread = [], [], []
    for s in segments(u):
        tail = s[int(SETTLE_FRACTION * len(s)):]
        level.append(float(u[s[0]]))
        settled.append(float(np.mean(y[tail])))
        spread.append(float(np.std(y[tail])))
    level = np.array(level)
    settled = np.array(settled)

    pairs, mids, amps = [], [], []
    for a in np.unique(np.abs(level[level != 0])):
        ip = np.where(np.isclose(level, a))[0]
        im = np.where(np.isclose(level, -a))[0]
        if len(ip) and len(im):
            amps.append(float(a))
            pairs.append(float((settled[ip[0]] - settled[im[0]]) / (2 * a)))
            mids.append(float((settled[ip[0]] + settled[im[0]]) / 2))
    amps = np.array(amps)
    gains = np.array(pairs)
    mids = np.array(mids)

    lo = amps <= np.median(amps)
    hi = amps > np.median(amps)
    return dict(levels=level.tolist(), settled=settled.tolist(),
                hold_std=spread, amplitudes=amps.tolist(),
                secant_gain=gains.tolist(), midpoint=mids.tolist(),
                gain_small=float(np.mean(gains[lo])),
                gain_large=float(np.mean(gains[hi])),
                gain_ratio=float(np.mean(gains[hi]) / np.mean(gains[lo])),
                drift_span=float(np.max(mids) - np.min(mids)),
                gain_spread_pct=float(100.0 * (np.max(gains) - np.min(gains))
                                      / abs(np.mean(gains))))


def etfe(u, y, ts, nperseg=NPERSEG):
    fs = 1.0 / ts
    f, puu = welch(u, fs, nperseg=nperseg)
    _, pyy = welch(y, fs, nperseg=nperseg)
    _, pyu = csd(u, y, fs, nperseg=nperseg)
    with np.errstate(divide="ignore", invalid="ignore"):
        g = pyu / puu
        coh = np.abs(pyu) ** 2 / (puu * pyy)
    return dict(f=f, mag=np.abs(g), phase=np.angle(g), coh=np.nan_to_num(coh))


def etfe_dc_gain(sp, band=DC_BAND_HZ, floor=COHERENCE_FLOOR):
    m = (sp["f"] >= band[0]) & (sp["f"] <= band[1]) & (sp["coh"] >= floor)
    if not np.any(m):
        m = (sp["f"] >= band[0]) & (sp["f"] <= band[1])
    return float(np.mean(sp["mag"][m]))


def etfe_peak(sp, fmin=1.0):
    m = sp["f"] >= fmin
    i = np.argmax(sp["mag"] * m)
    return float(sp["f"][i]), float(sp["mag"][i]), float(sp["coh"][i])


def _fit_percent(y, yhat, burn=BURN_IN):
    y, yhat = y[burn:], yhat[burn:]
    denom = np.linalg.norm(y - y.mean())
    if denom == 0:
        return 0.0
    return float(100.0 * (1.0 - np.linalg.norm(y - yhat) / denom))


def _arx_initial(u, y, na, nb, d):
    s = max(na, nb + d)
    cols = [-y[s - i - 1: len(y) - i - 1] for i in range(na)]
    cols += [u[s - d - j: len(y) - d - j] for j in range(nb)]
    phi = np.column_stack(cols)
    theta, *_ = np.linalg.lstsq(phi, y[s:], rcond=None)
    return theta


def fit_arx(u, y, na, nb, d, ts=TS, axis="") -> AxisModel:
    th = _arx_initial(u, y, na, nb, d)
    m = AxisModel(axis, th[:na], th[na:], d, ts, na=na, nb=nb)
    yhat = m.model_output(u)
    m.fit_train = _fit_percent(y, yhat)
    m.residual = y - yhat
    return m


def fit_oe(u, y, na, nb, d, ts=TS, axis="") -> AxisModel:
    theta0 = _arx_initial(u, y, na, nb, d)

    def residual(p):
        m = AxisModel(axis, p[:na], p[na:], d, ts)
        yhat = m.model_output(u)
        if not np.all(np.isfinite(yhat)):
            return np.full(len(y) - BURN_IN, 1e6)
        return (yhat - y)[BURN_IN:]

    sol = least_squares(residual, theta0, method="lm", max_nfev=30000)
    model = AxisModel(axis, sol.x[:na], sol.x[na:], d, ts, na=na, nb=nb)
    yhat = model.model_output(u)
    model.fit_train = _fit_percent(y, yhat)
    model.residual = y - yhat
    model.noise_std = float(np.std(model.residual[BURN_IN:]))
    return model


def blocked_cv(u, y, na, nb, d, ts, axis="", k=CV_FOLDS):
    folds = np.array_split(np.arange(len(y)), k)
    scores = []
    for i in range(k):
        va = folds[i]
        tr = np.concatenate([folds[j] for j in range(k) if j != i])
        try:
            m = fit_oe(u[tr], y[tr], na, nb, d, ts, axis)
            if np.max(np.abs(m.poles())) >= 1.0:
                return None
            scores.append(_fit_percent(y[va], m.model_output(u[va])))
        except Exception:
            return None
    return float(np.mean(scores)), float(np.std(scores))


def spectral_distance(m: AxisModel, sp, floor=COHERENCE_FLOOR):
    mask = (sp["coh"] >= floor) & (sp["f"] > 0)
    if np.sum(mask) < 4:
        return float("nan")
    gm = np.abs(m.frequency_response(2 * np.pi * sp["f"][mask]))
    ge = sp["mag"][mask]
    ok = (gm > 1e-9) & (ge > 1e-9)
    if np.sum(ok) < 4:
        return float("nan")
    return float(np.sqrt(np.mean((np.log(gm[ok]) - np.log(ge[ok])) ** 2)))


def _plausible(m: AxisModel, dc_ref: float, sp=None, cutoff=HP_CUTOFF_HZ):
    p = m.poles()
    if len(p) == 0 or np.max(np.abs(p)) >= 0.9999:
        return False, "unstable", float("nan")
    s = np.log(p.astype(complex)) / m.ts
    if np.any(np.abs(s) < 2 * np.pi * cutoff):
        return False, "pole below prefilter cutoff", float("nan")
    if dc_ref > 0 and abs(abs(m.dc_gain) - dc_ref) / dc_ref > DC_TOLERANCE:
        return False, "dc gain inconsistent with etfe", float("nan")
    wn, zeta = m.dominant_mode()
    if not (0.5 < wn < np.pi / m.ts) or not (0.0 < zeta < 1.0):
        return False, "non physical dominant mode", float("nan")
    sd = spectral_distance(m, sp) if sp is not None else float("nan")
    if np.isfinite(sd) and sd > SPECTRAL_MAX:
        return False, f"frequency response mismatch ({sd:.2f})", sd
    return True, "", sd


def select_structure(u, y, ts, dc_ref, axis="", na_range=NA_RANGE,
                     nb_range=NB_RANGE, d_range=D_RANGE, na_max=None,
                     nb_max=None, sp=None):
    table = []
    for na in range(na_range[0], (na_max or na_range[1]) + 1):
        for nb in range(nb_range[0], (nb_max or nb_range[1]) + 1):
            for d in range(d_range[0], d_range[1] + 1):
                try:
                    m = fit_oe(u, y, na, nb, d, ts, axis)
                except Exception:
                    continue
                ok, why, sd = _plausible(m, dc_ref, sp)
                cv = blocked_cv(u, y, na, nb, d, ts, axis) if ok else None
                wn, zeta = m.dominant_mode() if ok else (float("nan"),) * 2
                table.append(dict(na=na, nb=nb, d=d, n_par=na + nb,
                                  plausible=bool(ok), reject=why,
                                  cv_mean=cv[0] if cv else float("nan"),
                                  cv_std=cv[1] if cv else float("nan"),
                                  fit_train=m.fit_train, dc_gain=m.dc_gain,
                                  spectral=sd, wn=wn, zeta=zeta,
                                  delay_ms=d * ts * 1000.0))
    good = [r for r in table if r["plausible"] and np.isfinite(r["cv_mean"])]
    good.sort(key=lambda r: -r["cv_mean"])
    if not good:
        return table, None
    best = good[0]
    thr = best["cv_mean"] - min(best["cv_std"] / np.sqrt(CV_FOLDS), SE_CAP)
    within = [r for r in good if r["cv_mean"] >= thr]
    within.sort(key=lambda r: (round(r["spectral"], 2)
                               if np.isfinite(r["spectral"]) else 9.0,
                               r["n_par"], r["d"], -r["cv_mean"]))
    return table, within[0]


def design_form_response(K, wn, zeta, d, ts, w):
    s = complex(-zeta * wn, wn * np.sqrt(max(1.0 - zeta ** 2, 0.0)))
    p = np.exp(s * ts)
    den = np.real(np.poly([p, np.conj(p)]))
    z = np.exp(1j * np.asarray(w, dtype=float) * ts)
    num = K * den.sum()
    return (num * z ** (-d)) / (den[0] + den[1] * z ** -1 + den[2] * z ** -2)


def to_design_form(model: AxisModel, ts_design=TS):
    wn, zeta = model.dominant_mode()
    d = int(round(model.delay_ms / (ts_design * 1000.0)))
    return dict(K=model.dc_gain, wn=wn, zeta=zeta, delay=d, ts=ts_design,
                delay_ms=d * ts_design * 1000.0, f_n=wn / (2 * np.pi))


def model_gap(a: AxisModel, b: AxisModel, f_lo=0.20, f_hi_factor=1.5,
              n_grid=400):
    wn, _ = a.dominant_mode()
    f_hi = min(f_hi_factor * wn / (2 * np.pi), 0.40 / a.ts)
    f = np.logspace(np.log10(f_lo), np.log10(f_hi), n_grid)
    w = 2 * np.pi * f
    ga = np.abs(a.frequency_response(w))
    gb = np.abs(b.frequency_response(w))
    dev = np.abs(20.0 * np.log10(ga / gb))
    below = f < 0.5 * wn / (2 * np.pi)
    return dict(rms_db=float(np.sqrt(np.mean(dev ** 2))),
                max_db=float(np.max(dev)),
                f_at_max=float(f[np.argmax(dev)]),
                rms_db_below_resonance=float(np.sqrt(np.mean(dev[below] ** 2))),
                band_hz=(float(f_lo), float(f_hi)))


def sensor_noise(rec: Record):
    t, u, y, ts = uniform_grid(rec)
    quiet = u == 0.0
    if np.sum(quiet) > 32:
        seg = y[quiet]
        return float(np.std(np.diff(seg)) / np.sqrt(2.0))
    return float("nan")


def residual_tests(model: AxisModel, u, y, max_lag=25):
    r = (y - model.model_output(u))[BURN_IN:]
    uu = u[BURN_IN:]
    n = len(r)
    r0 = r - r.mean()
    denom = np.dot(r0, r0)
    acf = np.array([np.dot(r0[k:], r0[:n - k]) / denom for k in range(max_lag + 1)])
    bound = 1.96 / np.sqrt(n)
    lb = n * (n + 2) * np.sum(acf[1:] ** 2 / (n - np.arange(1, max_lag + 1)))
    p_lb = float(1.0 - chi2.cdf(lb, max_lag - model.n_par))

    u0 = uu - uu.mean()
    sc = np.sqrt(np.dot(u0, u0) * denom)
    lags = np.arange(-max_lag, max_lag + 1)
    ccf = np.array([np.dot(r0[k:], u0[:n - k]) / sc if k >= 0
                    else np.dot(r0[:n + k], u0[-k:]) / sc for k in lags])
    return dict(acf=acf.tolist(), ccf=ccf.tolist(), lags=lags.tolist(),
                bound=float(bound),
                acf_inside_pct=float(100.0 * np.mean(np.abs(acf[1:]) < bound)),
                ccf_inside_pct=float(100.0 * np.mean(np.abs(ccf) < bound)),
                ljung_box=float(lb), ljung_p=p_lb)


def amplitude_split(axis, structure, ts, cutoff=HP_CUTOFF_HZ):
    rec = load_record(axis)
    t, u, y, _ = uniform_grid(rec, ts)
    au = np.abs(u)
    thr = np.median(au[au > 0])
    out = {}
    for tag, mask in (("small", (au <= thr) & (au > 0)), ("large", au > thr)):
        idx = np.where(mask)[0]
        if len(idx) < 200:
            continue
        splits = np.split(idx, np.where(np.diff(idx) != 1)[0] + 1)
        run = max(splits, key=len)
        if len(run) < 200:
            continue
        uf, yf = high_pass(u[run], y[run], ts, cutoff)
        try:
            m = fit_oe(uf, yf, structure["na"], structure["nb"], structure["d"],
                       ts, axis)
        except Exception:
            continue
        wn, zeta = m.dominant_mode()
        out[tag] = dict(n=int(len(run)), K=m.dc_gain, wn=wn, zeta=zeta,
                        fit=m.fit_train,
                        amp=float(np.mean(np.abs(u[run]))))
    if "small" in out and "large" in out:
        out["K_ratio"] = out["large"]["K"] / out["small"]["K"]
        out["wn_ratio"] = out["large"]["wn"] / out["small"]["wn"]
    return out


def identify_axis(axis: str, na=None, nb=None, d=None, na_max=DESIGN_NA,
                  nb_max=DESIGN_NB, verbose=True) -> AxisModel:
    key = (axis, na, nb, d, na_max, nb_max)
    if key in _IDENTIFIED:
        return _IDENTIFIED[key]

    t, u, y, ts, rec = prepare_axis(axis)
    sp = etfe(u, y, ts)
    dc_ref = etfe_dc_gain(sp)

    if na is None or nb is None or d is None:
        table, best = select_structure(u, y, ts, dc_ref, axis, na_max=na_max,
                                       nb_max=nb_max, sp=sp)
        if best is None:
            raise RuntimeError(f"no plausible model structure for {axis}")
        na, nb, d = best["na"], best["nb"], best["d"]
        cv_mean, cv_std = best["cv_mean"], best["cv_std"]
    else:
        cv = blocked_cv(u, y, na, nb, d, ts, axis)
        cv_mean, cv_std = cv if cv else (float("nan"), float("nan"))

    model = fit_oe(u, y, na, nb, d, ts, axis)
    model.cv_mean, model.cv_std = cv_mean, cv_std
    model.fit_valid = cv_mean
    model.sensor_noise_std = sensor_noise(rec)
    model.design_form = to_design_form(model)

    if verbose:
        wn, zeta = model.dominant_mode()
        df = model.design_form
        print(f"[{axis}] OE(na={na}, nb={nb}, d={d})  ts={ts*1000:.2f} ms")
        print(f"        K = {model.dc_gain:+.3f} deg/deg  (etfe {dc_ref:.3f})   "
              f"wn = {wn:.2f} rad/s ({wn/2/np.pi:.2f} Hz)   zeta = {zeta:.3f}")
        print(f"        delay = {d} samples = {model.delay_ms:.1f} ms "
              f"({df['delay']} samples at the {TS*1000:.2f} ms control period)")
        print(f"        fit(train) = {model.fit_train:.1f}%   "
              f"fit(cv) = {cv_mean:.1f} +- {cv_std:.1f}%   "
              f"spectral rms = {spectral_distance(model, sp):.3f}")
        print(f"        residual std = {model.noise_std:.3f} deg   "
              f"sensor noise = {model.sensor_noise_std:.4f} deg")
    _IDENTIFIED[key] = model
    return model


def identify_all(verbose=True) -> dict[str, AxisModel]:
    return {ax: identify_axis(ax, verbose=verbose) for ax in ("roll", "pitch")}


def full_report(axis: str, na_max=DESIGN_NA, nb_max=DESIGN_NB):
    t, u, y, ts, rec = prepare_axis(axis)
    sp = etfe(u, y, ts)
    dc_ref = etfe_dc_gain(sp)
    fpk, mpk, cpk = etfe_peak(sp)
    table, best = select_structure(u, y, ts, dc_ref, axis, na_max=na_max,
                                   nb_max=nb_max, sp=sp)
    full, free = select_structure(u, y, ts, dc_ref, axis, na_max=NA_RANGE[1],
                                  nb_max=NB_RANGE[1], sp=sp)
    model = identify_axis(axis, best["na"], best["nb"], best["d"], verbose=False)
    rich = fit_oe(u, y, free["na"], free["nb"], free["d"], ts, axis)
    wn, zeta = model.dominant_mode()

    band = sp["coh"] >= COHERENCE_FLOOR
    return dict(
        axis=axis,
        timing=dict(n=rec.n, duration=rec.duration, ts_median=rec.ts_median,
                    ts_mean=rec.ts_mean, ts_std=rec.ts_std, dt_max=rec.dt_max,
                    n_gaps=rec.n_gaps, jitter_pct=rec.jitter_pct,
                    quant=rec.quant, ts_fit=ts, ts_nominal=TS,
                    clock_error_pct=100.0 * (rec.ts_mean - TS) / TS),
        excitation=excitation_summary(rec),
        static=static_map(rec),
        spectrum=dict(f=sp["f"].tolist(), mag=sp["mag"].tolist(),
                      coh=sp["coh"].tolist(), dc_gain=dc_ref,
                      peak_hz=fpk, peak_mag=mpk, peak_coh=cpk,
                      coherent_to_hz=float(sp["f"][band].max()) if np.any(band) else 0.0,
                      coh_mean=float(np.mean(sp["coh"][sp["f"] < 8.0]))),
        structure=dict(table=table, selected=best, unconstrained=free,
                       full_table=full, na_max=na_max, nb_max=nb_max,
                       cv_cost=float(free["cv_mean"] - best["cv_mean"]),
                       gap=model_gap(model, rich)),
        model=dict(a=model.a.tolist(), b=model.b.tolist(), d=model.delay,
                   ts=ts, K=model.dc_gain, wn=wn, zeta=zeta,
                   f_n=wn / (2 * np.pi), delay_ms=model.delay_ms,
                   fit_train=model.fit_train, cv_mean=model.cv_mean,
                   cv_std=model.cv_std, residual_std=model.noise_std,
                   sensor_noise=model.sensor_noise_std,
                   spectral=spectral_distance(model, sp),
                   zeros=np.abs(model.zeros()).tolist()),
        design_form=to_design_form(model),
        residuals=residual_tests(model, u, y),
        amplitude=amplitude_split(axis, best, ts))


if __name__ == "__main__":
    import json

    for ax in ("roll", "pitch"):
        r = full_report(ax)
        print("=" * 74)
        print(f"{ax.upper()}")
        tm, ex, st = r["timing"], r["excitation"], r["static"]
        print(f"  record   {tm['n']} samples, {tm['duration']:.1f} s, "
              f"ts {tm['ts_mean']*1000:.2f} ms (nominal {tm['ts_nominal']*1000:.2f}, "
              f"{tm['clock_error_pct']:+.2f}%), jitter {tm['jitter_pct']:.1f}%, "
              f"{tm['n_gaps']} gaps, quantisation {tm['quant']:.3f} deg")
        print(f"  input    {ex['kind']}, {ex['n_segments']} holds, "
              f"{ex['n_levels']} levels {ex['amp_min']:.2f}..{ex['amp_max']:.2f} deg, "
              f"dwell {ex['dwell_min']:.2f}..{ex['dwell_max']:.2f} s, "
              f"f0 ~ {ex['f_fundamental']:.2f} Hz, crest {ex['crest']:.2f}")
        print(f"  static   secant gain {st['gain_small']:+.2f} (small) "
              f"{st['gain_large']:+.2f} (large), ratio {st['gain_ratio']:.2f}, "
              f"spread {st['gain_spread_pct']:.0f}%, drift {st['drift_span']:.2f} deg")
        sp = r["spectrum"]
        print(f"  spectrum |G| dc {sp['dc_gain']:.2f}, peak {sp['peak_mag']:.2f} at "
              f"{sp['peak_hz']:.2f} Hz (coh {sp['peak_coh']:.2f}), "
              f"coherent to {sp['coherent_to_hz']:.1f} Hz")
        st = r["structure"]
        sel, free = st["selected"], st["unconstrained"]
        print(f"  order    design class na={sel['na']} nb={sel['nb']} d={sel['d']} "
              f"cv={sel['cv_mean']:.1f}+-{sel['cv_std']:.1f}%  |  "
              f"unrestricted na={free['na']} nb={free['nb']} d={free['d']} "
              f"cv={free['cv_mean']:.1f}%")
        g = st["gap"]
        print(f"           restriction costs {st['cv_cost']:.1f} cv points; "
              f"the two models differ by {g['rms_db']:.2f} dB rms over "
              f"{g['band_hz'][0]:.1f}-{g['band_hz'][1]:.1f} Hz "
              f"({g['rms_db_below_resonance']:.2f} dB below resonance, "
              f"worst {g['max_db']:.1f} dB at {g['f_at_max']:.2f} Hz)")
        m = r["model"]
        print(f"  model    K={m['K']:+.3f}  wn={m['wn']:.2f} rad/s "
              f"({m['f_n']:.2f} Hz)  zeta={m['zeta']:.3f}  "
              f"delay={m['delay_ms']:.1f} ms")
        print(f"           fit train {m['fit_train']:.1f}%, cv {m['cv_mean']:.1f}%, "
              f"spectral rms {m['spectral']:.3f} log-units, "
              f"residual {m['residual_std']:.3f} deg, "
              f"sensor noise {m['sensor_noise']:.4f} deg")
        df = r["design_form"]
        print(f"  design   K={df['K']:+.3f}  wn={df['wn']:.2f} rad/s "
              f"({df['f_n']:.2f} Hz)  zeta={df['zeta']:.3f}  "
              f"delay={df['delay']} samples ({df['delay_ms']:.1f} ms) at "
              f"ts={df['ts']*1000:.2f} ms")
        rr = r["residuals"]
        print(f"  residual acf inside 95%% band {rr['acf_inside_pct']:.0f}%, "
              f"ccf {rr['ccf_inside_pct']:.0f}%, Ljung-Box p={rr['ljung_p']:.3f}")
        am = r["amplitude"]
        if "K_ratio" in am:
            print(f"  amplitude small |u|={am['small']['amp']:.2f}: K={am['small']['K']:+.2f} "
                  f"wn={am['small']['wn']:.1f} fit={am['small']['fit']:.0f}%  |  "
                  f"large |u|={am['large']['amp']:.2f}: K={am['large']['K']:+.2f} "
                  f"wn={am['large']['wn']:.1f} fit={am['large']['fit']:.0f}%  "
                  f"(K ratio {am['K_ratio']:.2f})")
        print("  rejected structures:")
        for row in r["structure"]["table"]:
            if not row["plausible"]:
                print(f"     na={row['na']} nb={row['nb']} d={row['d']}: {row['reject']}")
