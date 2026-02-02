# Hold My Coffee - Beverage Stabilization Platform

**Authors:** Valentin Pletea-Marinescu, Sebastian-Alexandru Matei, Teodor-Alexandru Dicu, Severus-Constantin Olteanu  
National University of Science and Technology POLITEHNICA Bucharest, Romania

---

## Overview

Hold My Coffee is a **3-DOF active stabilization platform** designed to maintain a flat, near-horizontal surface while supporting a cup or mug, even in the presence of disturbances or on a moving base. The system controls pitch, roll, and vertical translation.

This repository contains the embedded firmware, MATLAB analysis scripts, and experimental datasets for a comparative study of four control strategies: **PID, RST, LQG, and MRAC**.

---

## Features

- 3 Degrees of Freedom: Pitch, Roll, Vertical height
- Dual-microcontroller real-time architecture (Teensy 4.1 + Arduino UNO)
- Moteus r4.11 motor controllers with Field-Oriented Control (FOC)
- BNO055 9-DOF IMU (quaternion output at 100 Hz) + VL53L0X ToF sensor
- Cascaded control loops running at 20 Hz (outer) / kHz (inner)
- Modular 3D-printed ABS/PETG + aluminum extrusion structure

---

## Hardware

| Component | Description |
|-----------|-------------|
| **MCU #1** | Teensy 4.1 - ARM Cortex-M7 @ 600 MHz (pitch/roll control, FDCAN) |
| **MCU #2** | Arduino UNO - ATmega328P (height control, ToF sensor) |
| **Motors** | mj5208 brushless motors (x3) |
| **Controllers** | Moteus r4.11 (x3) - FDCAN @ 1 Mbps |
| **IMU** | Adafruit BNO055 - NDOF mode, drift-free quaternions |
| **Distance** | VL53L0X Time-of-Flight sensor |
| **Linear Motion** | MGN12H linear rails + T8 lead screw (2 mm pitch) |
| **Power** | HRB 4S LiPo 6000 mAh, 14.8 V, 50C + 300 A switch |

---

## Repository Structure

```
firmware/
  arduino_uno/    - Height control firmware
  teensy/         - Main stabilization firmware (PID, RST, LQG, MRAC)

scripts/
  pitch_tf.m, roll_tf.m           - System identification
  pitch_pid.m, roll_pid.m         - PID controller design (SIMC tuning)
  pitch_rst.m, roll_rst.m         - RST controller design (pole-placement)
  pitch_lqg.m, roll_lqg.m         - LQG controller design (LQR + Kalman)
  pitch_adaptive.m, roll_adaptive.m - MRAC controller design
  analyze_controller.m            - Single controller analysis
  compare_controllers.m           - Multi-controller comparison plots
  compute_cpa_score.m             - Composite Performance Assessment

datasets/
  pitch_id.csv, roll_id.csv       - System identification data
  log_pid.csv, log_rst.csv, ...   - Experimental logs per controller
```

---

## Control Strategies

| Controller | Description |
|------------|-------------|
| **PID** | SIMC-tuned, simple and robust baseline |
| **RST** | Discrete pole-placement with 2-DOF structure |
| **LQG** | LQR state-feedback + Kalman observer |
| **MRAC** | Model Reference Adaptive Control with MIT rule |

---

## Composite Performance Assessment (CPA)

A weighted metric combining:
- **IAE** (Integral Absolute Error) - weight 0.45
- **Overshoot** (peak deviation) - weight 0.25
- **RMSE** (Root Mean Square Error) - weight 0.20
- **Control Effort** (sum of |delta u|) - weight 0.10

Lower CPA score = better performance.

---

## License

MIT License - see LICENSE file.
