from __future__ import annotations

import numpy as np

from .config import AXES, ROOT, TS, U_MAX, DU_MAX
from .design import MS_TARGET, design_equal_robustness
from .plant import fit_all

HEADER = ROOT / "firmware" / "teensy" / "controller_params.h"

def _arr(name, values, fmt="%.8ff"):
    body = ", ".join(fmt % v for v in np.ravel(values))
    return f"static const float {name}[{len(np.ravel(values))}] = {{{body}}};"

def generate(path=HEADER, verbose=True):
    plants = fit_all(verbose=False)
    designs = {ax: design_equal_robustness(plants[ax], MS_TARGET)[0] for ax in AXES}

    L = []
    L.append("// Controller parameters for both attitude axes.")
    L.append("// Every controller is designed on the same identified model and held")
    L.append(f"// to the same robustness constraint: Ms <= {MS_TARGET}, "
             f"Ts = {TS} s ({1/TS:.2f} Hz),")
    L.append("// with closed-loop bandwidth pushed as far as that constraint allows.")
    L.append("#ifndef CONTROLLER_PARAMS_H")
    L.append("#define CONTROLLER_PARAMS_H")
    L.append("")
    L.append(f"#define CTRL_TS            {TS:.8f}f   // s")
    L.append(f"#define CTRL_PERIOD_US     {int(round(TS * 1e6))}       // us, fixed-rate scheduler")
    L.append(f"#define CTRL_U_MAX         {U_MAX:.4f}f      // deg")
    L.append(f"#define CTRL_DU_MAX        {DU_MAX:.4f}f      // deg per sample")
    L.append("")

    for ax in AXES:
        p = plants[ax]
        d = designs[ax]
        U = ax.upper()
        L.append(f"// ===================== {U} =====================")
        L.append(f"// plant: K={p.K:.4f} deg/deg, wn={p.wn:.3f} rad/s, "
                 f"zeta={p.zeta:.4f}, delay={p.delay} samples")
        L.append("")

        g = d["pid"]
        L.append(f"#define {U}_PID_KP        {g.kp:+.8f}f")
        L.append(f"#define {U}_PID_KI        {g.ki:+.8f}f")
        L.append(f"#define {U}_PID_KD        {g.kd:+.8f}f")
        L.append(f"#define {U}_PID_TF        {g.tf:.8f}f")
        L.append("")

        r = d["rst"]
        L.append(f"#define {U}_RST_NR        {len(r.R)}")
        L.append(f"#define {U}_RST_NS        {len(r.S)}")
        L.append(f"#define {U}_RST_NT        {len(r.T)}")
        L.append(_arr(f"{U}_RST_R", r.R))
        L.append(_arr(f"{U}_RST_S", r.S))
        L.append(_arr(f"{U}_RST_T", r.T))
        L.append("")

        k = d["lqr"]
        kx = np.ravel(k.Kx)
        L.append(f"#define {U}_LQR_N         {len(kx)}")
        L.append(_arr(f"{U}_LQR_KX", kx))
        L.append(f"#define {U}_LQR_KI        {k.Ki:+.8f}f")
        L.append("")

        q = d["lqg"]
        n = q.A.shape[0]
        L.append(f"#define {U}_LQG_N         {n}")
        L.append(_arr(f"{U}_LQG_A", q.A))
        L.append(_arr(f"{U}_LQG_B", q.B))
        L.append(_arr(f"{U}_LQG_C", q.C))
        L.append(_arr(f"{U}_LQG_KX", q.Kx))
        L.append(f"#define {U}_LQG_KI        {q.Ki:+.8f}f")
        L.append(_arr(f"{U}_LQG_L", q.L))
        L.append("")

        m = d["mrac"]
        base = m.baseline
        nt = len(m.theta0)
        L.append(f"#define {U}_MRAC_NTHETA   {nt}")
        L.append(f"#define {U}_MRAC_NU       {len(base.R) - 1}")
        L.append(f"#define {U}_MRAC_NY       {len(base.S)}")
        L.append(f"#define {U}_MRAC_NRF      {len(base.T)}")
        L.append(_arr(f"{U}_MRAC_THETA0", m.theta0))
        L.append(_arr(f"{U}_MRAC_GAMMA", m.gamma))
        L.append(_arr(f"{U}_MRAC_LIM", m.theta_lim))
        L.append(_arr(f"{U}_MRAC_AM", m.am))
        L.append(_arr(f"{U}_MRAC_BM", m.bm))
        L.append(f"#define {U}_MRAC_SIGMA    {m.sigma:.8f}f")
        L.append(f"#define {U}_MRAC_DEADZONE {m.dead_zone:.8f}f")
        L.append(f"#define {U}_MRAC_SIGN     {np.sign(p.K):+.1f}f")
        L.append("")

    L.append("#endif  // CONTROLLER_PARAMS_H")

    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(L) + "\n")
    if verbose:
        print(f"wrote {path}")
    return path

if __name__ == "__main__":
    generate()
