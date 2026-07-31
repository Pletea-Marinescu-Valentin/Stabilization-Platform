from __future__ import annotations

import numpy as np

from .config import DU_MAX, TS, U_MAX

class BaseController:

    name = "base"
    ops = 0

    def __init__(self, ts=TS, u_max=U_MAX, du_max=DU_MAX):
        self.ts = ts
        self.u_max = u_max
        self.du_max = du_max
        self.u_prev = 0.0
        self.saturated = False

    def reset(self):
        self.u_prev = 0.0
        self.saturated = False

    def _limit(self, u):
        du = np.clip(u - self.u_prev, -self.du_max, self.du_max)
        u_lim = np.clip(self.u_prev + du, -self.u_max, self.u_max)
        self.saturated = abs(u_lim - u) > 1e-12
        self.u_prev = u_lim
        return u_lim

    def update(self, r, y):
        raise NotImplementedError

class PIDController(BaseController):

    name = "pid"
    ops = 14

    def __init__(self, gains, ts=TS, **kw):
        super().__init__(ts, **kw)
        self.kp, self.ki, self.kd = gains.kp, gains.ki, gains.kd
        self.a = gains.tf / (ts + gains.tf)
        self.kd_gain = gains.kd / (ts + gains.tf)
        self.tt = abs(gains.kp / gains.ki) if gains.ki else 0.0
        self.reset()

    def reset(self):
        super().reset()
        self.integral = 0.0
        self.deriv = 0.0
        self.e_prev = 0.0

    def update(self, r, y):
        e = r - y
        self.deriv = self.a * self.deriv + self.kd_gain * (e - self.e_prev)
        u_unsat = self.kp * e + self.integral + self.deriv
        u = self._limit(u_unsat)
        self.integral += self.ki * self.ts * e
        if self.tt > 0.0:
            self.integral += (self.ts / self.tt) * (u - u_unsat)
        self.e_prev = e
        return u

class RSTController(BaseController):

    name = "rst"
    ops = 12

    def __init__(self, polys, ts=TS, **kw):
        super().__init__(ts, **kw)
        self.R = np.asarray(polys.R, dtype=float)
        self.S = np.asarray(polys.S, dtype=float)
        self.T = np.asarray(polys.T, dtype=float)
        self.ops = 2 * (len(self.R) - 1) + 2 * len(self.S) + 2 * len(self.T)
        self.reset()

    def reset(self):
        super().reset()
        self.u_hist = np.zeros(max(len(self.R) - 1, 1))
        self.y_hist = np.zeros(max(len(self.S), 1))
        self.r_hist = np.zeros(max(len(self.T), 1))

    def update(self, r, y):
        self.y_hist = np.roll(self.y_hist, 1)
        self.y_hist[0] = y
        self.r_hist = np.roll(self.r_hist, 1)
        self.r_hist[0] = r

        acc = float(self.T @ self.r_hist[:len(self.T)])
        acc -= float(self.S @ self.y_hist[:len(self.S)])
        acc -= float(self.R[1:] @ self.u_hist[:len(self.R) - 1])

        u = self._limit(acc)
        self.u_hist = np.roll(self.u_hist, 1)
        self.u_hist[0] = u
        return u

class LQGController(BaseController):

    name = "lqg"
    ops = 26

    def __init__(self, design, ts=TS, **kw):
        super().__init__(ts, **kw)
        self.A = np.asarray(design.A, dtype=float)
        self.B = np.asarray(design.B, dtype=float).reshape(-1, 1)
        self.C = np.asarray(design.C, dtype=float).reshape(1, -1)
        self.Kx = np.asarray(design.Kx, dtype=float).reshape(1, -1)
        self.Ki = float(design.Ki)
        self.L = np.asarray(design.L, dtype=float).reshape(-1, 1)
        n = self.A.shape[0]
        self.ops = 2 * n * n + 6 * n + 6
        self.reset()

    def reset(self):
        super().reset()
        self.xhat = np.zeros((self.A.shape[0], 1))
        self.xint = 0.0

    def update(self, r, y):
        u_unsat = (-self.Kx @ self.xhat).item() - self.Ki * self.xint
        u = self._limit(u_unsat)

        innov = y - (self.C @ self.xhat).item()
        self.xhat = self.A @ self.xhat + self.B * u + self.L * innov
        if not self.saturated:
            self.xint += self.ts * (r - y)
        return u

class LQRController(BaseController):

    name = "lqr"
    ops = 10

    def __init__(self, design, ts=TS, **kw):
        super().__init__(ts, **kw)
        self.Kx = np.ravel(np.asarray(design.Kx, dtype=float))
        self.Ki = float(design.Ki)
        self.ops = 2 * len(self.Kx) + 6
        self.reset()

    def reset(self):
        super().reset()
        self.y_prev = 0.0
        self.xint = 0.0

    def update(self, r, y):
        u_unsat = -(self.Kx[0] * y + self.Kx[1] * self.y_prev
                    + self.Kx[2] * self.u_prev) - self.Ki * self.xint
        u = self._limit(u_unsat)
        if not self.saturated:
            self.xint += self.ts * (r - y)
        self.y_prev = y
        return u

class MRACController(BaseController):

    name = "mrac"
    ops = 34

    def __init__(self, design, plant_sign=-1.0, ts=TS, **kw):
        super().__init__(ts, **kw)
        self.am = np.asarray(design.am, dtype=float)
        self.bm = np.asarray(design.bm, dtype=float)
        self.theta0 = np.asarray(design.theta0, dtype=float).copy()
        self.gamma = np.asarray(design.gamma, dtype=float)
        self.sigma = float(design.sigma)
        self.theta_lim = np.asarray(design.theta_lim, dtype=float)
        self.dead_zone = float(design.dead_zone)
        self.sign = float(np.sign(plant_sign))
        base = design.baseline
        self.n_u = len(base.R) - 1
        self.n_y = len(base.S)
        self.n_r = len(base.T)
        self.ops = 6 * len(self.theta0) + 12
        self.reset()

    def reset(self):
        super().reset()
        self.theta = self.theta0.copy()
        self.u_hist = np.zeros(self.n_u)
        self.y_hist = np.zeros(self.n_y)
        self.rr_hist = np.zeros(self.n_r)
        self.ym_hist = np.zeros(len(self.am))
        self.r_hist = np.zeros(len(self.bm))
        self.ym = 0.0

    def _reference_model(self, r):
        self.r_hist = np.roll(self.r_hist, 1)
        self.r_hist[0] = r
        acc = float(self.bm @ self.r_hist)
        acc -= float(self.am[1:] @ self.ym_hist[:len(self.am) - 1])
        self.ym_hist = np.roll(self.ym_hist, 1)
        self.ym_hist[0] = acc
        self.ym = acc
        return acc

    def update(self, r, y):
        ym = self._reference_model(r)

        self.y_hist = np.roll(self.y_hist, 1)
        self.y_hist[0] = y
        self.rr_hist = np.roll(self.rr_hist, 1)
        self.rr_hist[0] = r
        phi = np.concatenate([self.u_hist, self.y_hist, self.rr_hist])

        u = self._limit(float(self.theta @ phi))

        e1 = y - ym
        if abs(e1) > self.dead_zone and not self.saturated:
            norm = 1.0 + float(phi @ phi)
            self.theta -= self.gamma * self.sign * e1 * phi / norm
        self.theta += self.sigma * self.ts * (self.theta0 - self.theta)
        np.clip(self.theta, self.theta0 - self.theta_lim,
                self.theta0 + self.theta_lim, out=self.theta)

        self.u_hist = np.roll(self.u_hist, 1)
        self.u_hist[0] = u
        return u

def build(name, designs, plant, ts=TS):
    if name == "pid":
        return PIDController(designs["pid"], ts)
    if name == "rst":
        return RSTController(designs["rst"], ts)
    if name == "lqg":
        return LQGController(designs["lqg"], ts)
    if name == "lqr":
        return LQRController(designs["lqr"], ts)
    if name == "mrac":
        return MRACController(designs["mrac"], plant_sign=np.sign(plant.K), ts=ts)
    raise KeyError(name)
