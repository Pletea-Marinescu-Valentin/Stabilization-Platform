from __future__ import annotations

import numpy as np

from .config import (AXES, CONTROLLER_LABELS, CONTROLLERS, RESULTS, SCENARIOS,
                     TS, WC, ZETA)

TABLES = "tables"

def _path(name):
    d = RESULTS / TABLES
    d.mkdir(parents=True, exist_ok=True)
    return d / name

def _fmt(x, nd=2):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return "--"
    return f"{x:.{nd}f}"

def _bold_min(vals, nd=2, lower_is_better=True):
    arr = np.array([v if v is not None and np.isfinite(v) else np.nan for v in vals])
    if np.all(np.isnan(arr)):
        return ["--"] * len(vals)
    idx = int(np.nanargmin(arr) if lower_is_better else np.nanargmax(arr))
    out = []
    for i, v in enumerate(arr):
        s = _fmt(v, nd)
        out.append(f"\\textbf{{{s}}}" if i == idx else s)
    return out

def table_models(out):
    p = out["plants"]
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Identified axis models, $G(s)=K\omega_n^2/(s^2+2\zeta\omega_n s+\omega_n^2)\,e^{-\tau s}$.}",
        r"\label{tab:models}", r"\begin{tabular}{lccccc}", r"\toprule",
        r"Axis & $K$ [\si{\degree\per\degree}] & $\omega_n$ [\si{\radian\per\second}] & "
        r"$f_n$ [\si{\hertz}] & $\zeta$ & fit [\%] \\", r"\midrule",
    ]
    for ax in AXES:
        m = p[ax]
        lines.append(f"{ax.capitalize()} & {m['K']:.3f} & {m['wn']:.2f} & "
                     f"{m['f_n']:.2f} & {m['zeta']:.3f} & {m['fit_valid']:.1f} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("models.tex").write_text("\n".join(lines))

def table_scenarios(out):
    short = {"Empty cup": "Empty", "Half full": "Half",
             "Three-quarters full": "3/4 full"}
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Payload scenarios. Added inertia lowers each axis resonance "
        r"as $\sqrt{J_0/J}$, and the liquid adds a lightly damped slosh mode at "
        r"$\omega_s$. Frequencies in \si{\radian\per\second}.}",
        r"\label{tab:scenarios}", r"\setlength{\tabcolsep}{4pt}",
        r"\begin{tabular}{lccccc}", r"\toprule",
        r"Case & $m$ [\si{\gram}] & $\omega_n^{\mathrm{roll}}$ & "
        r"$\omega_n^{\mathrm{pitch}}$ & $\omega_s$ & $\zeta_s$ \\", r"\midrule",
    ]
    for r in out["scenarios"]:
        ws = "--" if not np.isfinite(r["w_slosh"]) else f"{r['w_slosh']:.1f}"
        zs = "--" if not np.isfinite(r["zeta_slosh"]) else f"{r['zeta_slosh']:.4f}"
        label = short.get(r["label"], r["label"])
        lines.append(f"{label} & {r['mass']:.0f} & "
                     f"{r['wn_roll']:.1f} & {r['wn_pitch']:.1f} & {ws} & {zs} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("scenarios.tex").write_text("\n".join(lines))

def table_verification(out):
    v = out["verification"]
    f = out["frontier"]
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{All five controllers verified against the common robustness "
        r"specification, and their tolerance to plant error. $M_s$ is matched by "
        r"construction; the bandwidth column is what that equal margin bought, "
        r"and the last two columns are what the structures do \emph{not} share.}",
        r"\label{tab:verification}", r"\begin{tabular}{llccccc}", r"\toprule",
        r"Axis & Ctrl & $\omega_B$ & $M_s$ & PM & $\Delta K$ & $\Delta\omega_n$ \\",
        r" & & [\si{\radian\per\second}] & & [\si{\degree}] & [\si{\decibel}] & [\%] \\",
        r"\midrule",
    ]
    for ax in AXES:
        for i, n in enumerate(CONTROLLERS):
            a, fr = v[ax][n], f[ax][n]
            head = ax.capitalize() if i == 0 else ""
            lines.append(
                f"{head} & {CONTROLLER_LABELS[n]} & {a['bandwidth']:.2f} & "
                f"{a['modulus_margin']:.2f} & {a['phase_margin_deg']:.0f} & "
                f"{fr['gain_db']:.1f} & {fr['freq_pct']:.0f} \\\\")
        if ax != AXES[-1]:
            lines.append(r"\midrule")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("verification.tex").write_text("\n".join(lines))

def _metric_rows(out, axis, window, keys):
    got = {}
    for r in out["metrics"]:
        if r["axis"] == axis and r["window"] == window:
            got[(r["scenario"], r["controller"])] = r
    return got

def table_results(out, window, caption, label, fname):
    keys = [("RMSE", "RMSE", 2), ("IAE", "IAE", 1), ("peak", "Peak", 2),
            ("effort", "Effort", 2)]
    lines = [
        r"\begin{table}[t]", r"\centering", f"\\caption{{{caption}}}",
        f"\\label{{{label}}}", r"\footnotesize",
        r"\setlength{\tabcolsep}{3pt}",
        r"\begin{tabular}{ll" + "c" * (len(keys) * 1) + r"c}", r"\toprule",
        r"Case & Ctrl & " + " & ".join(k[1] for k in keys) + r" & Score \\",
        r"\midrule",
    ]
    for ax in AXES:
        lines.append(r"\multicolumn{" + str(2 + len(keys) + 1) +
                     r"}{l}{\textit{" + ax.capitalize() + r" axis}} \\")
        got = _metric_rows(out, ax, window, keys)
        for sc in SCENARIOS:
            cols = {k[0]: [] for k in keys}
            for n in CONTROLLERS:
                r = got.get((sc, n))
                for k, _, _ in keys:
                    cols[k].append(r[k] if r else None)
            scores = out["scores"].get(f"{ax}|{sc}|{window}", {}).get("score", {})
            sc_vals = [scores.get(n) for n in CONTROLLERS]
            fmtd = {k: _bold_min(cols[k], nd) for k, _, nd in keys}
            fmt_sc = _bold_min(sc_vals, 2)
            for i, n in enumerate(CONTROLLERS):
                head = SCENARIOS[sc]["label"] if i == 0 else ""
                row = [head, CONTROLLER_LABELS[n]]
                row += [fmtd[k][i] for k, _, _ in keys]
                row += [fmt_sc[i]]
                lines.append(" & ".join(row) + r" \\")
            if sc != list(SCENARIOS)[-1]:
                lines.append(r"\addlinespace[2pt]")
        if ax != AXES[-1]:
            lines.append(r"\midrule")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path(fname).write_text("\n".join(lines))

def table_scores_compact(out):
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Composite score in every measured case, normalised against the "
        r"best controller of that case (1.00 = best, 2.00 = twice its cost). "
        r"Lower is better; bold marks the best of each column.}",
        r"\label{tab:scores}", r"\footnotesize", r"\setlength{\tabcolsep}{4pt}",
        r"\begin{tabular}{ll ccc ccc}", r"\toprule",
        r"& & \multicolumn{3}{c}{Roll} & \multicolumn{3}{c}{Pitch} \\",
        r"\cmidrule(lr){3-5}\cmidrule(lr){6-8}",
        r"Window & Ctrl & empty & half & 3/4 & empty & half & 3/4 \\", r"\midrule",
    ]
    for win, wl in (("acq", "Acquis."), ("dist", "Disturb.")):
        cells = {n: [] for n in CONTROLLERS}
        for ax in AXES:
            for sc in SCENARIOS:
                s = out["scores"].get(f"{ax}|{sc}|{win}", {}).get("score", {})
                for n in CONTROLLERS:
                    cells[n].append((s.get(n), ax, sc))
        best_idx = []
        for j in range(6):
            col = [cells[n][j][0] for n in CONTROLLERS]
            arr = np.array([v if v is not None else np.nan for v in col])
            best_idx.append(int(np.nanargmin(arr)) if not np.all(np.isnan(arr)) else -1)
        for i, n in enumerate(CONTROLLERS):
            row = [wl if i == 0 else "", CONTROLLER_LABELS[n]]
            for j, (v, ax, sc) in enumerate(cells[n]):
                s = _fmt(v, 2)
                if i == best_idx[j]:
                    s = f"\\textbf{{{s}}}"
                row.append(s)
            lines.append(" & ".join(row) + r" \\")
        if win == "acq":
            lines.append(r"\midrule")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("scores.tex").write_text("\n".join(lines))

def table_raw_pitch(out):
    got = {}
    for r in out["metrics"]:
        if r["axis"] == "pitch" and r["window"] == "dist":
            got[(r["scenario"], r["controller"])] = r
    fill_effect, worst = 0.0, CONTROLLERS[0]
    for n in CONTROLLERS:
        a, b = got.get(("empty", n)), got.get(("full", n))
        if not a or not b or not a["RMSE"]:
            continue
        pct = abs(100.0 * (b["RMSE"] / a["RMSE"] - 1.0))
        if pct > fill_effect:
            fill_effect, worst = pct, n

    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Pitch axis, disturbance window: raw metrics in degrees. "
        r"Filling the cup moves RMSE by at most "
        f"\\SI{{{fill_effect:.1f}}}{{\\percent}} ({CONTROLLER_LABELS[worst]}), "
        r"so what separates the rows is the control structure and not the "
        r"payload.}",
        r"\label{tab:rawpitch}", r"\footnotesize",
        r"\begin{tabular}{lcccccc}", r"\toprule",
        r"& \multicolumn{2}{c}{Empty} & \multicolumn{2}{c}{Half full} & "
        r"\multicolumn{2}{c}{3/4 full} \\",
        r"\cmidrule(lr){2-3}\cmidrule(lr){4-5}\cmidrule(lr){6-7}",
        r"Ctrl & RMSE & peak & RMSE & peak & RMSE & peak \\", r"\midrule",
    ]
    for n in CONTROLLERS:
        row = [CONTROLLER_LABELS[n]]
        for sc in SCENARIOS:
            r = got.get((sc, n))
            row += [_fmt(r["RMSE"], 2) if r else "--",
                    _fmt(r["peak"], 2) if r else "--"]
        lines.append(" & ".join(row) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("rawpitch.tex").write_text("\n".join(lines))

def table_cost(out):
    c = out["cost"]
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Cost of one controller update: floating-point operations and the "
        r"resulting time on the \SI{600}{\mega\hertz} Cortex-M7, against a "
        r"\SI{31.25}{\milli\second} budget.}",
        r"\label{tab:cost}", r"\begin{tabular}{lccc}", r"\toprule",
        r"Controller & flops & time [\si{\micro\second}] & budget [\%] \\", r"\midrule",
    ]
    for n in CONTROLLERS:
        us = c[n]["us_at_600mhz"]
        lines.append(f"{CONTROLLER_LABELS[n]} & {c[n]['flops']} & {us:.3f} & "
                     f"{100 * us * 1e-6 / TS:.4f} \\\\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("cost.tex").write_text("\n".join(lines))

def table_sensitivity(out):
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Share of random weight vectors, drawn uniformly from the simplex, "
        r"for which each controller attains the best composite score.}",
        r"\label{tab:sensitivity}", r"\begin{tabular}{ll" + "c" * len(CONTROLLERS) + "}", r"\toprule",
        r"Axis & Window & " + " & ".join(CONTROLLER_LABELS[n] for n in CONTROLLERS)
        + r" \\", r"\midrule",
    ]
    for ax in AXES:
        for win, wl in (("acq", "Acquisition"), ("dist", "Disturbance")):
            agg = {n: [] for n in CONTROLLERS}
            for sc in SCENARIOS:
                d = out["scores"].get(f"{ax}|{sc}|{win}")
                if d:
                    for n in CONTROLLERS:
                        agg[n].append(d["win_pct"].get(n, 0.0))
            vals = [float(np.mean(agg[n])) if agg[n] else 0.0 for n in CONTROLLERS]
            cells = _bold_min(vals, 0, lower_is_better=False)
            lines.append(f"{ax.capitalize()} & {wl} & " + " & ".join(cells) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("sensitivity.tex").write_text("\n".join(lines))

def table_slosh(out):
    lines = [
        r"\begin{table}[t]", r"\centering",
        r"\caption{Slosh-mode stability, predicted on the identified axis model "
        r"with the analytical slosh doublet added; the empty cup has no free "
        r"surface, hence no such mode, and does not appear. "
        r"$\zeta_s^{\mathrm{crit}}$ is the least liquid damping for which the "
        r"loop remains stable; the margin is $\zeta_s/\zeta_s^{\mathrm{crit}}$. "
        r"Bold marks the largest margin of each row; underlined values are below "
        r"one, that is, the predicted margin does not cover the liquid present.}",
        r"\label{tab:slosh}", r"\begin{tabular}{ll" + "c" * len(CONTROLLERS) + "}", r"\toprule",
        r"Axis & Case & " + " & ".join(CONTROLLER_LABELS[n] for n in CONTROLLERS)
        + r" \\", r"\midrule",
    ]
    for key, r in out.get("slosh", {}).items():
        ax, sc = key.split("|")
        margins = [r[n]["margin"] for n in CONTROLLERS]
        finite = [m for m in margins if m is not None]
        best = max(finite) if finite else None
        cells = []
        for m in margins:
            if m is None:
                cells.append("unst.")
            elif m < 1.0:
                cells.append(f"\\underline{{{m:.2f}}}")
            elif best is not None and abs(m - best) < 1e-9:
                cells.append(f"\\textbf{{{m:.2f}}}")
            else:
                cells.append(f"{m:.2f}")
        lines.append(f"{ax.capitalize()} & {SCENARIOS[sc]['label']} & "
                     + " & ".join(cells) + r" \\")
    lines += [r"\bottomrule", r"\end{tabular}", r"\end{table}"]
    _path("slosh.tex").write_text("\n".join(lines))

def write_all_tables(out):
    table_slosh(out)
    table_models(out)
    table_scenarios(out)
    table_verification(out)
    table_results(out, "acq",
                  "Acquisition window (\\SIrange{1}{20}{\\second}): recovery from a "
                  "\\SI{6}{\\degree} step of base tilt. Score is normalised against "
                  "the best controller in each row group; lower is better.",
                  "tab:acq", "results_acq.tex")
    table_results(out, "dist",
                  "Disturbance window (\\SIrange{20}{80}{\\second}): trapezoidal tilts, "
                  "rocking and a sweep through the slosh band.",
                  "tab:dist", "results_dist.tex")
    table_cost(out)
    table_sensitivity(out)
    table_scores_compact(out)
    table_raw_pitch(out)
