// Controller parameters for both attitude axes.
// Every controller is designed on the same identified model and held
// to the same robustness constraint: Ms <= 1.3, Ts = 0.03125 s (32.00 Hz),
// with closed-loop bandwidth pushed as far as that constraint allows.
#ifndef CONTROLLER_PARAMS_H
#define CONTROLLER_PARAMS_H

#define CTRL_TS            0.03125000f   // s
#define CTRL_PERIOD_US     31250       // us, fixed-rate scheduler
#define CTRL_U_MAX         5.0000f      // deg
#define CTRL_DU_MAX        1.5000f      // deg per sample

// ===================== ROLL =====================
// plant: K=-2.2187 deg/deg, wn=46.671 rad/s, zeta=0.1428, delay=2 samples

#define ROLL_PID_KP        -0.00326176f
#define ROLL_PID_KI        -1.79537457f
#define ROLL_PID_KD        -0.00256669f
#define ROLL_PID_TF        0.12500000f

#define ROLL_RST_NR        3
#define ROLL_RST_NS        3
#define ROLL_RST_NT        3
static const float ROLL_RST_R[3] = {1.00000000f, -1.63792462f, 0.63792462f};
static const float ROLL_RST_S[3] = {-0.01315602f, 0.00271207f, -0.00867515f};
static const float ROLL_RST_T[3] = {-0.01315602f, 0.00271207f, -0.00867515f};

#define ROLL_LQR_N         3
static const float ROLL_LQR_KX[3] = {-0.04894199f, 0.05293458f, 0.25883931f};
#define ROLL_LQR_KI        +3.08955070f

#define ROLL_LQG_N         2
static const float ROLL_LQG_A[4] = {0.20614658f, -0.65940501f, 1.00000000f, 0.00000000f};
static const float ROLL_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float ROLL_LQG_C[2] = {0.00000000f, -3.22435637f};
static const float ROLL_LQG_KX[2] = {0.18710527f, 0.08145019f};
#define ROLL_LQG_KI        +2.32397593f
static const float ROLL_LQG_L[2] = {0.17705243f, -0.05609175f};

#define ROLL_MRAC_NTHETA   8
#define ROLL_MRAC_NU       2
#define ROLL_MRAC_NY       3
#define ROLL_MRAC_NRF      3
static const float ROLL_MRAC_THETA0[8] = {1.63792462f, -0.63792462f, 0.01315602f, -0.00271207f, 0.00867515f, -0.01315602f, 0.00271207f, -0.00867515f};
static const float ROLL_MRAC_GAMMA[8] = {0.00086162f, 0.00034078f, 0.00001538f, 0.00000994f, 0.00001305f, 0.00001538f, 0.00000994f, 0.00001305f};
static const float ROLL_MRAC_LIM[8] = {0.68792834f, 0.28792834f, 0.03802090f, 0.03384332f, 0.03622855f, 0.03802090f, 0.03384332f, 0.03622855f};
static const float ROLL_MRAC_AM[3] = {1.00000000f, -1.63792462f, 0.68034432f};
static const float ROLL_MRAC_BM[2] = {0.00000000f, 0.04241970f};
#define ROLL_MRAC_SIGMA    0.05000000f
#define ROLL_MRAC_DEADZONE 0.15000000f
#define ROLL_MRAC_SIGN     -1.0f

// ===================== PITCH =====================
// plant: K=-6.2155 deg/deg, wn=26.220 rad/s, zeta=0.1210, delay=2 samples

#define PITCH_PID_KP        -0.00098285f
#define PITCH_PID_KI        -0.26712301f
#define PITCH_PID_KD        -0.00064443f
#define PITCH_PID_TF        0.12500000f

#define PITCH_RST_NR        3
#define PITCH_RST_NS        3
#define PITCH_RST_NT        3
static const float PITCH_RST_R[3] = {1.00000000f, -1.63792462f, 0.63792462f};
static const float PITCH_RST_S[3] = {-0.01185474f, 0.01475267f, -0.00972276f};
static const float PITCH_RST_T[3] = {-0.01185474f, 0.01475267f, -0.00972276f};

#define PITCH_LQR_N         3
static const float PITCH_LQR_KX[3] = {-0.06432036f, 0.06557829f, 0.28611321f};
#define PITCH_LQR_KI        +0.93695438f

#define PITCH_LQG_N         2
static const float PITCH_LQG_A[4] = {1.24445283f, -0.82015802f, 1.00000000f, 0.00000000f};
static const float PITCH_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float PITCH_LQG_C[2] = {0.00000000f, -3.57828878f};
static const float PITCH_LQG_KX[2] = {0.20468475f, -0.09933553f};
#define PITCH_LQG_KI        +0.66200449f
static const float PITCH_LQG_L[2] = {-0.19576727f, -0.33887014f};

#define PITCH_MRAC_NTHETA   8
#define PITCH_MRAC_NU       2
#define PITCH_MRAC_NY       3
#define PITCH_MRAC_NRF      3
static const float PITCH_MRAC_THETA0[8] = {1.63792462f, -0.63792462f, 0.01185474f, -0.01475267f, 0.00972276f, -0.01185474f, 0.01475267f, -0.00972276f};
static const float PITCH_MRAC_GAMMA[8] = {0.00086162f, 0.00034078f, 0.00001471f, 0.00001621f, 0.00001359f, 0.00001471f, 0.00001621f, 0.00001359f};
static const float PITCH_MRAC_LIM[8] = {0.68792834f, 0.28792834f, 0.03750039f, 0.03865956f, 0.03664760f, 0.03750039f, 0.03865956f, 0.03664760f};
static const float PITCH_MRAC_AM[3] = {1.00000000f, -1.63792462f, 0.68034432f};
static const float PITCH_MRAC_BM[2] = {0.00000000f, 0.04241970f};
#define PITCH_MRAC_SIGMA    0.05000000f
#define PITCH_MRAC_DEADZONE 0.15000000f
#define PITCH_MRAC_SIGN     -1.0f

#endif  // CONTROLLER_PARAMS_H
