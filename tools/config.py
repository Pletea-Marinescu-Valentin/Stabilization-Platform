from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DATASETS = ROOT / "identification"
RESULTS = ROOT / "results"
FIGURES = ROOT / "paper" / "figures"

TS = 0.03125
FS = 1.0 / TS

INPUT_DELAY_SAMPLES = 2

WC = 5.0
ZETA = 0.9

U_MAX = 5.0
DU_MAX = 1.5

IMU_QUANT = 1.0 / 16.0
IMU_NOISE_STD = 0.045
IMU_BIAS_WALK = 0.002

CUP_RADIUS = 0.035
CUP_HEIGHT = 0.100
PLATFORM_INERTIA = 3.2e-3
PAYLOAD_RADIUS = 0.060

SCENARIOS = {
    "empty": dict(label="Empty cup",         fill=0.00, mass_g=334.0),
    "half":  dict(label="Half full",         fill=0.50, mass_g=467.0),
    "full":  dict(label="Three-quarters full", fill=0.75, mass_g=581.0),
}

CONTROLLERS = ["pid", "rst", "lqg", "lqr", "mrac"]
CONTROLLER_LABELS = {"pid": "PID", "rst": "RST", "lqg": "LQG",
                     "lqr": "LQR", "mrac": "MRAC"}
AXES = ["roll", "pitch"]

T_TOTAL = 80.0
T_TRACK_END = 20.0
T_DIST_START = 20.0

RANDOM_SEED = 20260731
