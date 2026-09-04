from __future__ import annotations

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .config import (AXES, CONTROLLER_LABELS, CONTROLLERS, FIGURES, SCENARIOS,
                     TS, T_DIST_START, T_TOTAL, T_TRACK_END)
from .design import MS_TARGET, plant_response, controller_response

COL = 3.5          # one IEEEtran column
WIDE = 7.16        # both columns (figure*)

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

# Five traces on one pair of axes cannot be read.  Every comparison figure
# below therefore splits the controllers over two stacked panels: the base
# tilt and the two extreme designs first, the remaining three underneath on
# identical axes, so the panels can be compared by eye.
GROUP_A = ["pid", "lqr"]
GROUP_B = ["rst", "lqg", "mrac"]

TILT_COLOR = "0.55"


def _save(fig, name):
    fig.savefig(FIGURES / f"{name}.pdf")
    plt.close(fig)


def _legend(ax, handles=None, labels=None, ncol=3, loc="upper left"):
    kw = dict(loc=loc, ncol=ncol, frameon=True, framealpha=0.9,
              edgecolor="none", handlelength=1.8, columnspacing=1.0,
              borderaxespad=0.25, handletextpad=0.5)
    lg = (ax.legend(handles, labels, **kw) if handles is not None
          else ax.legend(**kw))
    lg.get_frame().set_facecolor("white")
    return lg


def fig_identification(plants):
    from .identification import (COHERENCE_FLOOR, etfe, identify_axis,
                                 prepare_axis)

    fig, axes = plt.subplots(2, 2, figsize=(WIDE, 2.7),
                             gridspec_kw=dict(width_ratios=[1.35, 1],
                                              hspace=0.28, wspace=0.20))
    for row, axis in enumerate(AXES):
        t, u, y, ts, _ = prepare_axis(axis)
        m = identify_axis(axis, verbose=False)
        sp = etfe(u, y, ts)

        a = axes[row][0]
        split = int(0.72 * len(y))
        sl = slice(split, min(split + 420, len(y)))
        a.plot(t[sl] - t[sl][0], y[sl], color="0.25", lw=0.9, label="measured")
        a.plot(t[sl] - t[sl][0], m.model_output(u)[sl], color="#C44E52", lw=0.9,
               ls="--", label=f"model, {m.cv_mean:.0f}% cross-validated fit")
        a.set_ylabel(f"{axis} [deg]")
        a.set_xlim(0, (t[sl] - t[sl][0])[-1])
        _legend(a, ncol=2, loc="upper left")

        b = axes[row][1]
        f = sp["f"][1:]
        b.loglog(f, sp["mag"][1:], color="0.55", lw=0.7, label="measured")
        b.loglog(f, np.abs(m.frequency_response(2 * np.pi * f)), color="#C44E52",
                 lw=1.1, label="model")
        low = sp["coh"][1:] < COHERENCE_FLOOR
        b.fill_between(f, 1e-3, 1e3, where=low, color="0.88", lw=0, zorder=0)
        b.set_ylim(max(sp["mag"][1:].min(), 1e-2), sp["mag"][1:].max() * 3.0)
        b.set_xlim(f[0], f[-1])
        b.set_ylabel(r"$|G|$ [deg/deg]")
        if row == 0:
            _legend(b, ncol=1, loc="lower left")
            b.text(0.985, 0.93, "shaded: coherence $<0.8$", fontsize=6,
                   color="0.35", ha="right", va="top", transform=b.transAxes)

    axes[1][0].set_xlabel("time [s]")
    axes[1][1].set_xlabel("frequency [Hz]")
    _save(fig, "identification")


def fig_design_check(plants, designs):
    """|S| for every loop, showing that the Ms budget is spent, not asserted."""
    w = np.logspace(-1, np.log10(np.pi / TS) - 1e-9, 800)
    fig, axes = plt.subplots(1, 2, figsize=(COL * 2, 1.85), sharey=True,
                             gridspec_kw=dict(wspace=0.14))
    for ax_obj, axis in zip(axes, AXES):
        p = plants[axis]
        # MRAC's nominal loop is the RST loop by construction, so the two
        # curves coincide exactly.  Draw RST thicker and MRAC last, on top of
        # it, so both are visible and the coincidence is the point.
        for n in ["pid", "lqr", "lqg", "rst", "mrac"]:
            resp = controller_response(n, designs[axis])
            L = -resp(w) * plant_response(p, w, ts=TS)
            S = np.abs(1.0 / (1.0 + L))
            lw = 1.9 if n == "rst" else 1.0
            ax_obj.semilogx(w, 20 * np.log10(S), label=CONTROLLER_LABELS[n],
                            lw=lw, **STYLE[n])
        db = 20 * np.log10(MS_TARGET)
        ax_obj.axhline(db, color="0.35", lw=0.7, ls=":")
        ax_obj.text(0.12, db + 1.6, f"$M_s={MS_TARGET:.2f}$", fontsize=6.5,
                    color="0.3", va="bottom")
        ax_obj.set_ylim(None, db + 5.0)
        ax_obj.set_title(f"{axis} axis")
        ax_obj.set_xlabel(r"$\omega$ [rad/s]")
        ax_obj.set_xlim(w[0], w[-1])
    axes[0].set_ylabel(r"$|S|$ [dB]")
    _legend(axes[0], ncol=3, loc="lower right")
    axes[1].text(0.98, 0.06, "MRAC (dotted) lies on RST (thick dashed):\n"
                 "the two coincide at the nominal operating point",
                 fontsize=6, color="0.35", ha="right", va="bottom",
                 transform=axes[1].transAxes)
    _save(fig, "sensitivity")


def _phase_marks(ax, y):
    """Light separators for the four phases of the disturbance protocol."""
    for x in (T_TRACK_END, 36.0, 52.0):
        ax.axvline(x, color="0.8", lw=0.5, ls="-", zorder=0)
    for x, lbl in ((10.0, "acquisition"), (28.0, "steps"),
                   (44.0, "rocking"), (61.0, "sweep")):
        ax.text(x, y, lbl, fontsize=6, color="0.5", ha="center", va="bottom")


def fig_error_traces(runs, axis, scenario, name):
    """Error over the whole run, split so that no panel carries five traces."""
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(COL, 2.5), sharex=True,
                                 gridspec_kw=dict(hspace=0.16))

    ref = runs[(axis, scenario, "pid")]
    lo = min(min(runs[(axis, scenario, n)].e.min() for n in CONTROLLERS),
             ref.d.min())
    hi = max(max(runs[(axis, scenario, n)].e.max() for n in CONTROLLERS),
             ref.d.max())
    pad = 0.10 * (hi - lo)
    ylim = (lo - 2.0 * pad, hi + 2.6 * pad)   # room below for the phase
                                             # labels, above for the legend

    a1.fill_between(ref.t, 0, ref.d, color=TILT_COLOR, alpha=0.20, lw=0)
    a1.plot(ref.t, ref.d, color=TILT_COLOR, lw=0.8, label="base tilt")
    for n in GROUP_A:
        r = runs[(axis, scenario, n)]
        a1.plot(r.t, r.e, label=CONTROLLER_LABELS[n], **STYLE[n])
    for n in GROUP_B:
        r = runs[(axis, scenario, n)]
        a2.plot(r.t, r.e, label=CONTROLLER_LABELS[n], **STYLE[n])

    for a in (a1, a2):
        a.set_xlim(0, T_TOTAL)
        a.set_ylim(*ylim)
        a.set_ylabel("error [deg]")
    _legend(a1, ncol=3, loc="upper left")
    _legend(a2, ncol=3, loc="upper left")
    _phase_marks(a2, ylim[0] + 0.02 * (ylim[1] - ylim[0]))
    a2.set_xlabel("time [s]")
    _save(fig, name)


def fig_error_zoom(runs, axis, scenario, name):
    """The two windows that decide the comparison, controllers split 2 and 3."""
    fig, axes = plt.subplots(2, 2, figsize=(COL * 2, 2.9), sharex="col",
                             gridspec_kw=dict(hspace=0.16, wspace=0.16))
    windows = ((0.5, 6.0, "acquisition step"),
               (58.0, 70.0, "sweep through the slosh band"))
    for col, (t0, t1, title) in enumerate(windows):
        lim = []
        for row, group in enumerate((GROUP_A, GROUP_B)):
            ax = axes[row][col]
            for n in group:
                r = runs[(axis, scenario, n)]
                m = (r.t >= t0) & (r.t <= t1)
                ax.plot(r.t[m], r.e[m], label=CONTROLLER_LABELS[n], **STYLE[n])
                lim += [r.e[m].min(), r.e[m].max()]
            ax.set_xlim(t0, t1)
        lo, hi = min(lim), max(lim)
        pad = 0.10 * (hi - lo)
        for row in (0, 1):
            axes[row][col].set_ylim(lo - pad, hi + 2.2 * pad)
            _legend(axes[row][col], ncol=3, loc="upper right")
        axes[0][col].set_title(title)
        axes[1][col].set_xlabel("time [s]")
    for row in (0, 1):
        axes[row][0].set_ylabel("error [deg]")
    _save(fig, name)


def fig_command(runs, axis, scenario, name):
    """Commanded increment, split 2 and 3 like every other comparison here.

    The saturation limit is far above the traces, so drawing it in would leave
    both panels four-fifths empty; the headroom is annotated instead.
    """
    from .config import U_MAX
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(COL, 2.45), sharex=True,
                                 gridspec_kw=dict(hspace=0.16))
    lim = []
    for ax_obj, group in ((a1, GROUP_A), (a2, GROUP_B)):
        for n in group:
            r = runs[(axis, scenario, n)]
            ax_obj.plot(r.t, r.u, label=CONTROLLER_LABELS[n], **STYLE[n])
            lim += [r.u.min(), r.u.max()]

    lo, hi = min(lim), max(lim)
    pad = 0.10 * (hi - lo)
    peak = max(float(np.max(np.abs(runs[(axis, scenario, n)].u)))
               for n in CONTROLLERS)
    for ax_obj in (a1, a2):
        ax_obj.set_xlim(0, T_TOTAL)
        ax_obj.set_ylim(lo - pad, hi + 2.8 * pad)
        ax_obj.set_ylabel("command [deg]")
        _legend(ax_obj, ncol=3, loc="upper left")
    a1.text(T_TOTAL * 0.99, hi + 2.5 * pad,
            f"peak {peak:.2f} deg against a {U_MAX:.0f} deg limit",
            fontsize=6, color="0.4", ha="right", va="top")
    a2.set_xlabel("time [s]")
    _save(fig, name)


def fig_robustness(out):
    fig, axes = plt.subplots(1, 2, figsize=(COL * 2, 1.9),
                             gridspec_kw=dict(wspace=0.22))
    x = np.arange(len(CONTROLLERS))
    width = 0.38
    for j, (key, lbl, unit) in enumerate(
            [("gain_db", "gain error tolerated", "dB"),
             ("freq_pct", "resonance shift tolerated", "%")]):
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


def make_all_figures(out, runs, plants, designs):
    FIGURES.mkdir(parents=True, exist_ok=True)
    plt.rcParams["text.usetex"] = False
    fig_identification(plants)
    fig_design_check(plants, designs)
    fig_robustness(out)
    for axis in AXES:
        fig_error_traces(runs, axis, "full", f"traces_{axis}")
    fig_error_zoom(runs, "pitch", "full", "zoom_pitch")
    fig_command(runs, "pitch", "full", "command_pitch")
