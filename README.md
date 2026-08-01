# Hold My Coffee - Beverage Stabilization Platform

---

## Overview

Hold My Coffee is a **3-DOF active stabilization platform** that keeps a
near-horizontal surface under a cup while the base is disturbed. It controls
pitch, roll and vertical translation.

This repository contains the embedded firmware, the Python design and
analysis toolchain, and the experimental datasets behind a comparative study
of **PID, RST, LQR, LQG and MRAC**. The point of the study is that all five
are held to the **same robustness constraint** — an identical peak
sensitivity `Ms` — with closed-loop bandwidth left as the free variable. A
comparison that instead fixes the performance specification lets a structure
buy performance by quietly accepting a thinner stability margin, and the
same five designs under that protocol put the pitch PID at `Ms = 1.60`
against `1.25` for the model-based ones.

## What is measured and what is computed

- **Measured on the hardware:** the PRBS identification records in
  `identification/roll_id.csv` and `identification/pitch_id.csv`. Each axis
  was excited with a 1 deg pseudo-random binary sequence for 60 s at 32 Hz,
  and the axis models are estimated from those two records.
- **Computed on the identified models:** everything else. The design step and
  its loop analysis (the `Ms` constraint, the achievable bandwidths, the
  slosh-mode margins), and the closed-loop runs themselves, which are
  evaluated on the identified models with the measured transport delay, the
  measured IMU noise and quantisation, and the actuator limits.

The closed-loop comparison is therefore model-based, not a hardware
experiment, and `results/summary.json` records this in its `source` field.
Evaluating on the model is also what makes the protocol enforceable: peak
sensitivity is a property of the loop transfer function, so it can be
constrained exactly at synthesis and verified exactly afterwards, whereas
repeated runs on hardware would only ever estimate it.

If closed-loop logs measured on the platform are placed in `experiments/`,
`run_all` uses those instead and `source` becomes `"measured"`; see
`tools/ingest.py` for the expected columns and the disturbance protocol.

---

## Reproducing everything

```bash
pip install -r requirements.txt
python -m tools.run_all          # identification -> design -> evaluation -> tables + figures
python -m tools.export_firmware  # regenerate firmware/teensy/controller_params.h
```

`run_all` writes per-run CSVs and `metrics.csv` into `results/`, the LaTeX
tables into `results/tables/` and the figures into `paper/figures/` (the
manuscript itself lives outside this repository). The generated results are
committed, so the numbers in the paper can be checked without running
anything; see `results/README.md`.

---

## Adding a new set of runs

Drop the SD-card logs into `experiments/` and re-run the two commands above.

### File naming

One file per controller and payload, containing **both axes**:

```
experiments/<controller>_<scenario>.csv
```

`<controller>` is `pid`, `rst`, `lqr`, `lqg` or `mrac` (`adaptive` is accepted
as an alias for `mrac`); `<scenario>` is `empty`, `half` or `full`. A leading
`log_` is ignored, so `log_pid_half.csv` also works. Fifteen files in total.
The analysis keys on the filename, not on the `mode` column.

### Columns

This is exactly what the Teensy firmware already writes, so no change is
needed if the logs come off the SD card:

| Column | Unit | Required | Meaning |
|---|---|---|---|
| `time_ms` | ms | **yes** | `millis()` at the control step; need not be uniform |
| `mode` | — | no | 0=LQR, 1=PID, 2=RST, 3=LQG, 4=MRAC |
| `roll_deg` | deg | **yes** | measured roll from the IMU, world frame |
| `pitch_deg` | deg | **yes** | measured pitch from the IMU, world frame |
| `e_roll` | deg | no | roll error; recomputed from `roll_deg` if absent |
| `e_pitch` | deg | no | pitch error; recomputed from `pitch_deg` if absent |
| `u_roll` | deg | **yes** | commanded roll position increment |
| `u_pitch` | deg | **yes** | commanded pitch position increment |
| `motor_roll` | rev | no | Moteus reported position |
| `motor_pitch` | rev | no | Moteus reported position |
| `missed` | count | no | missed scheduler ticks; useful as a health check |

Only five columns are strictly required: a time column plus the two angles
and the two commands. Common alternative names are accepted
(`control_roll`/`control_pitch`, `error_roll`, `t_ms`).

### Disturbance protocol

The analysis windows are fixed, so the servo rig must apply this sequence
with `t = 0` at the first logged sample. Total run length 80 s.

| From | To | Base tilt |
|---|---|---|
| 0 s | 1 s | 0 deg, settle |
| 1 s | 20 s | +6 deg step, held (*acquisition* window) |
| 21 s | 23.8 s | +5 deg trapezoid, 0.4 s ramps, 2 s hold |
| 26 s | 28.8 s | −5 deg trapezoid |
| 31 s | 33.8 s | +7 deg trapezoid |
| 36 s | 52 s | ±4 deg sine at 0.25 Hz |
| 52 s | 70 s | ±2 deg sweep, 0.2 → 4 Hz (through the slosh band) |
| 71 s | 80 s | +3 deg, held |

Everything from 20 s on is the *disturbance* window. Amplitudes stay under
the 11.3 deg at which the roll axis would saturate. Run every controller and
every fill level against the same sequence.

---

## Hardware

| Component | Description |
|-----------|-------------|
| **MCU #1** | Teensy 4.1 - ARM Cortex-M7 @ 600 MHz (pitch/roll control, FDCAN) |
| **MCU #2** | Arduino UNO - ATmega328P (height control, ToF sensor) |
| **Motors** | mj5208 brushless motors (x3) |
| **Controllers** | Moteus r4.11 (x3) - FDCAN @ 1 Mbps |
| **IMU** | Adafruit BNO055 - NDOF mode, quaternion output |
| **Distance** | VL53L0X Time-of-Flight sensor |
| **Linear Motion** | MGN12H linear rails + T8 lead screw (2 mm pitch) |
| **Power** | HRB 4S LiPo 6000 mAh, 14.8 V, 50C + 300 A switch |

---

## Repository structure

```
tools/                 design and analysis in Python
  config.py              all rates, limits and the common specification, defined once
  identification.py      output-error identification with order selection
  plant.py               axis models, payload scaling, liquid slosh mode
  design.py              the five syntheses, loop analysis, robustness frontier
  controllers.py         discrete control laws, written as the firmware runs them
  protocol.py            the disturbance protocol and the run container
  ingest.py              reads the hardware logs, checks timing, resamples them
  metrics.py             metrics, composite score, weight-sensitivity analysis
  report.py / figures.py LaTeX tables and figures
  run_all.py             the whole pipeline
  export_firmware.py     generates firmware/teensy/controller_params.h

firmware/
  teensy/                outer attitude loop, fixed-rate scheduler
  arduino_uno/           height axis and the servo disturbance rig

dashboard/               live telemetry UI (BLE / USB serial)
identification/          the two PRBS records the models are estimated from
results/                 generated: run CSVs, metrics, summary, LaTeX tables
experiments/             optional: closed-loop logs measured on the platform
```

---

## Control strategies

All five are synthesised on the same identified model, at 32 Hz, with
integral action, identical actuator limits and an identical peak sensitivity
of 1.30. Bandwidth is the free variable and is what the comparison measures.

| Controller | Design |
|------------|--------|
| **PID** | Parallel form, filtered derivative, back-calculation anti-windup; three gains fitted at each candidate bandwidth |
| **RST** | Exact pole placement via the Bezout identity, integrator forced in `R` |
| **LQR** | Optimal feedback with an integral state and no estimator: because the identified model has no numerator dynamics, the state `[y(k), y(k-1), u(k-1)]` is directly available |
| **LQG** | The same feedback law with the state supplied by a Kalman filter fixed by the measured noise statistics; weights swept for maximum bandwidth within the `Ms` budget |
| **MRAC** | Direct MRAC, normalised gradient with sigma-modification, dead zone and projection; initialised at the exact model-matching (RST) solution |

---

## Composite score

Composite indices normalised by constants fixed in advance are fragile: set
them above the values actually observed and every term collapses towards
zero, leaving the index decided by whichever metric sits nearest its cap.

The score used here normalises **within each test case against the best
controller of that case**, so it is dimensionless and scale-free: 1.00 means
best in that case, 2.00 means twice its cost. Weights are 0.40 IAE, 0.25
RMSE, 0.20 peak error, 0.15 control effort, and `metrics.weight_sensitivity`
reports how often each controller wins under randomly redrawn weights.

---

## Design notes

- Sampling is **32 Hz** (31.25 ms), released by a hardware timer; the logged
  runs come in at 32.0 ± 1.4 ms, 2.4 % slow with 4.4 % jitter, and are
  resampled onto the nominal grid before any metric is computed. The
  identified **two-sample transport delay (62.5 ms)** is the binding
  constraint on achievable bandwidth.
- Controller gains in `firmware/teensy/controller_params.h` come from
  `tools/export_firmware.py`, which emits the same equal-robustness designs
  the study reports, so the coefficients that run are the ones that were
  analysed. Select a law over serial with `0`-`4`; the mode switch resets
  both axes.
- The identified numerator zero is not statistically identifiable (it moves
  from -0.30 to -2.32 across sub-records), so both axes are modelled as
  minimum-phase second-order resonances with delay.

---

## License

MIT License - see LICENSE file.
