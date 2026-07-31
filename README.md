# Hold My Coffee - Beverage Stabilization Platform

**Authors:** Valentin Pletea-Marinescu, Sebastian-Alexandru Matei, Teodor-Alexandru Dicu, Severus-Constantin Olteanu
National University of Science and Technology POLITEHNICA Bucharest, Romania

---

## Overview

Hold My Coffee is a **3-DOF active stabilization platform** that keeps a
near-horizontal surface under a cup while the base is disturbed. It controls
pitch, roll and vertical translation.

This repository contains the embedded firmware, the Python design and
analysis toolchain, and the experimental datasets behind a comparative study
of **PID, RST, LQG, LQR and MRAC**. The point of the study is that all five
are held to the **same robustness constraint** — an identical peak
sensitivity `Ms` — with closed-loop bandwidth left as the free variable. A
comparison that instead fixes the performance specification lets a structure
buy performance by quietly accepting a thinner stability margin, which is
exactly what made PID look competitive in the earlier version of this work.

## What is measured and what is simulated

- **Measured on the hardware:** the PRBS identification experiments in
  `identification/roll_id.csv` and `identification/pitch_id.csv`, from which the axis
  models are estimated.
- **Simulated:** the controller comparison, run on those identified models
  with the measured transport delay, the measured IMU noise and quantisation,
  and the actuator limits.

The paper states this distinction explicitly. The models are experimental;
the controller comparison is not.

---

## Reproducing everything

```bash
pip install -r requirements.txt
python -m tools.run_all          # identification -> design -> simulation -> tables + figures
python -m tools.export_firmware  # regenerate firmware/teensy/controller_params.h
cd paper && latexmk -pdf main.tex
```

`run_all` writes per-run CSVs and `metrics.csv` into `results/`, the LaTeX
tables into `results/tables/`, and the figures into `paper/figures/`. Every
number in the paper comes from those files; none is transcribed by hand.

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
tools/                 design and analysis in Python (replaces the former MATLAB scripts)
  config.py              all rates, limits and the common specification, defined once
  identification.py      output-error identification with order selection
  plant.py               axis models, payload scaling, liquid slosh mode
  design.py              the four syntheses, loop analysis, robustness frontier
  controllers.py         discrete control laws, written as the firmware runs them
  simulation.py          closed-loop simulation and the disturbance protocol
  metrics.py             metrics, composite score, weight-sensitivity analysis
  report.py / figures.py LaTeX tables and figures
  run_all.py             the whole pipeline
  export_firmware.py     generates firmware/teensy/controller_params.h

firmware/
  teensy/                outer attitude loop, fixed-rate scheduler
  arduino_uno/           height axis and the servo disturbance rig

dashboard/               live telemetry UI (BLE / USB serial)
identification/          the two PRBS records the models are estimated from
paper/                   manuscript, generated figures, bibliography
results/                 generated: run CSVs, metrics, LaTeX tables
```

---

## Control strategies

All five are synthesised on the same identified model, at 32 Hz, with
integral action, identical actuator limits and an identical peak sensitivity
of 1.30. Bandwidth is the free variable and is what the comparison measures.

| Controller | Design |
|------------|--------|
| **PID** | Parallel form, filtered derivative, back-calculation anti-windup; gains fitted to the common reference model |
| **RST** | Exact pole placement via the Bezout identity, integrator forced in `R` |
| **LQG** | LQR with an integral state and a Kalman filter fixed by the measured noise statistics; weights swept for maximum bandwidth within the `Ms` budget |
| **LQR** | The same optimal feedback law with no estimator: because the identified model has no numerator dynamics, the state `[y(k), y(k-1), u(k-1)]` is directly available |
| **MRAC** | Direct MRAC, normalised gradient with sigma-modification, dead zone and projection; initialised at the exact model-matching (RST) solution |

---

## Composite score

The earlier version of this study normalised each metric by a hand-chosen
constant (IAE_max = 1000, overshoot_max = 30 deg). Those constants were one
to two orders of magnitude above the values actually observed, so the index
was dominated by whichever metric happened to be nearest its cap.

The score used now normalises **within each test case against the best
controller of that case**, so it is dimensionless and scale-free: 1.00 means
best in that case, 2.00 means twice its cost. Weights are 0.40 IAE, 0.25
RMSE, 0.20 peak error, 0.15 control effort, and `metrics.weight_sensitivity`
reports how often each controller wins under randomly redrawn weights.

---

## Design notes

- Sampling is **32 Hz** (31.25 ms), released by a hardware timer. The
  identified **two-sample transport delay (62.5 ms)** is the binding
  constraint on achievable bandwidth.
- Controller gains are **generated** into `controller_params.h` from the
  design scripts, not maintained by hand: an earlier revision shipped RST
  coefficients that did not correspond to any design.
- The identified numerator zero is not statistically identifiable (it moves
  from -0.30 to -2.32 across sub-records), so both axes are modelled as
  minimum-phase second-order resonances with delay.

---

## License

MIT License - see LICENSE file.
