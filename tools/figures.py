from __future__ import annotations

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .config import (AXES, CONTROLLER_LABELS, CONTROLLERS, FIGURES, SCENARIOS,
                     TS, T_DIST_START, T_TOTAL, T_TRACK_END)
from .design import plant_response, controller_response

COL = 3.5
plt.rcParams.update({
    "font.size": 8,
    "axes.labelsize": 8,
    "axes.titlesize": 8,
    "legend.fontsize": 7,
    "xtick.labelsize": 7,
    "ytick.labelsize": 7,
    "lines.linewidth": 1.0,
    "axes.grid": True,
    "grid.alpha": 0.3,
    "grid.linewidth": 0.4,
    "figure.dpi": 200,
    "savefig.bbox": "tight",
    "savefig.pad_inches": 0.02,
    "pdf.fonttype": 42,
})

STYLE = {
    "pid":  dict(color="#4C72B0", ls="-"),
    "rst":  dict(color="#DD8452", ls="--"),
    "lqg":  dict(color="#55A868", ls="-."),
    "lqr":  dict(color="#8172B3", ls=(0, (3, 1, 1, 1))),
    "mrac": dict(color="#C44E52", ls=":"),
}

def _save(fig, name):
    fig.savefig(FIGURES / f"{name}.pdf")
    plt.close(fig)

def fig_identification(plants):
    from .identification import load_prbs

    fig, axes = plt.subplots(2, 1, figsize=(COL, 2.9), sharex=False)
    for ax_obj, axis in zip(axes, AXES):
        t, u, y = load_prbs(axis)
        p = plants[axis]
        yhat = p.simulate(u)
        split = int(0.7 * len(y))
        sl = slice(split, min(split + 500, len(y)))
        ax_obj.plot(t[sl] - t[sl][0], y[sl], color="0.25", lw=0.9, label="measured")
        ax_obj.plot(t[sl] - t[sl][0], yhat[sl], color="#C44E52", lw=0.9,
                    ls="--", label=f"model ({p.fit_valid:.0f}\\%)")
        ax_obj.set_ylabel(f"{axis} [deg]")
        ax_obj.legend(loc="upper right", ncol=2, frameon=False,
                      handlelength=1.6, borderaxespad=0.2)
    axes[-1].set_xlabel("time [s]")
    _save(fig, "identification")

def fig_design_check(plants, designs):
    w = np.logspace(-1, np.log10(np.pi / TS) - 1e-9, 800)
    fig, axes = plt.subplots(1, 2, figsize=(COL * 2, 2.0), sharey=True)
    for ax_obj, axis in zip(axes, AXES):
        p = plants[axis]
        for n in CONTROLLERS:
            if n == "mrac":
                continue

            resp = controller_response(n, designs[axis])
            L = -resp(w) * plant_response(p, w, ts=TS)
            S = np.abs(1.0 / (1.0 + L))
            ax_obj.semilogx(w, 20 * np.log10(S), label=CONTROLLER_LABELS[n],
                            **STYLE[n])
        ax_obj.axhline(20 * np.log10(1.6), color="0.4", lw=0.6, ls=":")
        ax_obj.text(0.12, 20 * np.log10(1.6) + 0.6, "$M_s=1.6$", fontsize=6,
                    color="0.35")
        ax_obj.set_title(f"{axis} axis")
        ax_obj.set_xlabel(r"$\omega$ [rad/s]")
        ax_obj.set_xlim(w[0], w[-1])
    axes[0].set_ylabel(r"$|S|$ [dB]")
    axes[0].legend(loc="lower right", frameon=False, handlelength=1.8)
    _save(fig, "sensitivity")

def fig_error_traces(runs, axis, scenario, name):
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(COL, 2.7), sharex=True,
                                 gridspec_kw=dict(height_ratios=[1, 0.55]))
    ref = runs[(axis, scenario, "pid")]
    a1.plot(ref.t, ref.d, color="0.65", lw=0.8, label="base tilt")
    for n in CONTROLLERS:
        r = runs[(axis, scenario, n)]
        a1.plot(r.t, r.e, label=CONTROLLER_LABELS[n], **STYLE[n])
        a2.plot(r.t, r.u, **STYLE[n])
    a1.set_ylabel("error [deg]")
    a2.set_ylabel("command [deg]")
    a2.set_xlabel("time [s]")
    a1.legend(loc="upper left", ncol=3, frameon=False, handlelength=1.4,
              columnspacing=0.8, borderaxespad=0.2, fontsize=6)
    a1.set_xlim(0, T_TOTAL)
    _save(fig, name)

def fig_error_zoom(runs, axis, scenario, name):
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(COL * 2, 1.9))
    for n in CONTROLLERS:
        r = runs[(axis, scenario, n)]
        m1 = (r.t >= 0.5) & (r.t <= 6.0)
        a1.plot(r.t[m1], r.e[m1], label=CONTROLLER_LABELS[n], **STYLE[n])
        m2 = (r.t >= 58.0) & (r.t <= 70.0)
        a2.plot(r.t[m2], r.e[m2], **STYLE[n])
    a1.set_title("acquisition step")
    a2.set_title("sweep through the slosh band")
    for a in (a1, a2):
        a.set_xlabel("time [s]")
    a1.set_ylabel("error [deg]")
    a1.legend(loc="upper right", frameon=False, ncol=2, handlelength=1.5)
    _save(fig, name)

def fig_robustness(out):
    fig, axes = plt.subplots(1, 2, figsize=(COL * 2, 1.9))
    x = np.arange(len(CONTROLLERS))
    width = 0.38
    for j, (key, lbl, unit) in enumerate(
            [("gain_db", r"gain error tolerated", "dB"),
             ("freq_pct", r"resonance shift tolerated", r"\%")]):
        ax = axes[j]
        for i, axis in enumerate(AXES):
            vals = [out["frontier"][axis][n][key] for n in CONTROLLERS]
            ax.bar(x + (i - 0.5) * width, vals, width,
                   label=axis, color=["#4C72B0", "#DD8452"][i], alpha=0.85)
        ax.set_xticks(x)
        ax.set_xticklabels([CONTROLLER_LABELS[n] for n in CONTROLLERS])
        ax.set_ylabel(f"{lbl} [{unit}]")
        ax.grid(axis="x", visible=False)
    axes[0].legend(frameon=False, ncol=2)
    _save(fig, "robustness")

def fig_mrac_adaptation(runs, axis, name):
    fig, ax = plt.subplots(figsize=(COL, 1.7))
    for sc, ls in zip(SCENARIOS, ["-", "--", ":"]):
        r = runs[(axis, sc, "mrac")]
        if r.theta is None:
            continue
        th = r.theta
        rel = np.linalg.norm(th - th[0], axis=1) / max(np.linalg.norm(th[0]), 1e-9)
        ax.plot(r.t[:len(rel)], 100 * rel, ls=ls, lw=1.0,
                label=SCENARIOS[sc]["label"])
    ax.set_xlabel("time [s]")
    ax.set_ylabel(r"$\|\theta-\theta_0\|/\|\theta_0\|$ [\%]")
    ax.legend(frameon=False, loc="upper left")
    _save(fig, name)

def fig_mrac_sweep(out):
    fig, ax = plt.subplots(figsize=(COL, 1.8))
    ax2 = ax.twinx()
    for axis, mk in zip(AXES, ["o", "s"]):
        rows = out["mrac_sweep"][axis]
        tau = [r["tau"] for r in rows]
        ax.semilogx(tau, [r["dist_rmse"] for r in rows], marker=mk, ms=3,
                    color="#4C72B0", label=f"{axis}: dist. RMSE")
        ax2.semilogx(tau, [r["acq_iae"] for r in rows], marker=mk, ms=3,
                     ls="--", color="#C44E52", label=f"{axis}: acq. IAE")
    ax.set_xlabel(r"adaptation time constant $\tau_a$ [s]")
    ax.set_ylabel("disturbance RMSE [deg]", color="#4C72B0")
    ax2.set_ylabel("acquisition IAE [deg s]", color="#C44E52")
    ax2.grid(False)
    ax.tick_params(axis="y", colors="#4C72B0")
    ax2.tick_params(axis="y", colors="#C44E52")
    _save(fig, "mrac_sweep")

def make_all_figures(out, runs, plants, designs):
    FIGURES.mkdir(parents=True, exist_ok=True)
    plt.rcParams["text.usetex"] = False
    fig_identification(plants)
    fig_design_check(plants, designs)
    fig_robustness(out)
    for axis in AXES:
        fig_error_traces(runs, axis, "full", f"traces_{axis}")
    fig_error_zoom(runs, "pitch", "full", "zoom_pitch")
    fig_mrac_adaptation(runs, "pitch", "mrac_theta")
    if "mrac_sweep" in out:
        fig_mrac_sweep(out)
