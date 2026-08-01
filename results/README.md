# Generated results

Everything in this directory is produced by

```bash
python -m tools.run_all
```

and is committed so the numbers in the paper can be checked without running
anything.

## Provenance

These are **not** hardware logs. The plant models are identified from the
PRBS experiments in `identification/`, which are measurements on the physical
platform; the closed-loop runs here are then evaluated on those identified
models, reproducing the measured transport delay, the measured IMU noise and
quantisation, and the actuator limits. `summary.json` records this in its
`source` field, which reads `"model"` for these files.

If closed-loop logs measured on the platform are placed in `experiments/`,
`run_all` uses them instead and `source` becomes `"measured"`.

## Contents

| File | What it holds |
|---|---|
| `run_<axis>_<controller>_<scenario>.csv` | one 80 s run, 2560 samples at 31.25 ms |
| `metrics.csv` | every metric for each run, split into the two windows |
| `summary.json` | identified models, designs, margins, scores, slosh analysis |
| `tables/*.tex` | the LaTeX tables the paper includes verbatim |

Run CSV columns: `time_s`, `base_tilt_deg` (the disturbance applied),
`angle_deg` (platform angle), `angle_meas_deg` (after noise and the
0.0625 deg quantiser), `error_deg`, `command_deg`.
