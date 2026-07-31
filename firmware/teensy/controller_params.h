// -------------------------------------------------------------
// GENERATED FILE -- do not edit by hand.
// Produced by  python -m tools.export_firmware
//
// Every controller below is designed on the same identified model
// and to the same closed-loop specification:
//   wc = 5.0 rad/s, zeta = 0.9, Ts = 0.03125 s (32.00 Hz)
// -------------------------------------------------------------
#ifndef CONTROLLER_PARAMS_H
#define CONTROLLER_PARAMS_H

#define CTRL_TS            0.03125000f   // s
#define CTRL_PERIOD_US     31250       // us, fixed-rate scheduler
#define CTRL_U_MAX         5.0000f      // deg
#define CTRL_DU_MAX        1.5000f      // deg per sample

// ===================== ROLL =====================
// plant: K=-2.2673 deg/deg, wn=43.863 rad/s, zeta=0.1150, delay=2 samples

#define ROLL_PID_KP        -0.00139057f
#define ROLL_PID_KI        -1.38235582f
#define ROLL_PID_KD        -0.00267572f
#define ROLL_PID_TF        0.12500000f

#define ROLL_RST_NR        3
#define ROLL_RST_NS        3
#define ROLL_RST_NT        3
static const float ROLL_RST_R[3] = {1.00000000f, -1.73360152f, 0.73360152f};
static const float ROLL_RST_S[3] = {-0.00681276f, 0.00241660f, -0.00497108f};
static const float ROLL_RST_T[3] = {-0.00681276f, 0.00241660f, -0.00497108f};

#define ROLL_LQG_N         2
static const float ROLL_LQG_A[4] = {0.35471626f, -0.72967298f, 1.00000000f, 0.00000000f};
static const float ROLL_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float ROLL_LQG_C[2] = {0.00000000f, -3.11739921f};
static const float ROLL_LQG_KX[2] = {0.24023942f, -0.28704696f};
#define ROLL_LQG_KI        +1.23101702f
static const float ROLL_LQG_L[2] = {0.18997128f, -0.10941707f};

#define ROLL_MRAC_NTHETA   8
#define ROLL_MRAC_NU       2
#define ROLL_MRAC_NY       3
#define ROLL_MRAC_NRF      3
static const float ROLL_MRAC_THETA0[8] = {1.73360152f, -0.73360152f, 0.00681276f, -0.00241660f, 0.00497108f, -0.00681276f, 0.00241660f, -0.00497108f};
static const float ROLL_MRAC_GAMMA[8] = {0.00091195f, 0.00039111f, 0.00001258f, 0.00001029f, 0.00001162f, 0.00001258f, 0.00001029f, 0.00001162f};
static const float ROLL_MRAC_LIM[8] = {0.72811264f, 0.32811264f, 0.03739713f, 0.03563867f, 0.03666046f, 0.03739713f, 0.03563867f, 0.03666046f};
static const float ROLL_MRAC_AM[3] = {1.00000000f, -1.73360152f, 0.75483960f};
static const float ROLL_MRAC_BM[2] = {0.00000000f, 0.02123808f};
#define ROLL_MRAC_SIGMA    0.05000000f
#define ROLL_MRAC_DEADZONE 0.15000000f
#define ROLL_MRAC_SIGN     -1.0f

// ===================== PITCH =====================
// plant: K=-6.3536 deg/deg, wn=25.040 rad/s, zeta=0.1274, delay=2 samples

#define PITCH_PID_KP        -0.00056844f
#define PITCH_PID_KI        -0.43341608f
#define PITCH_PID_KD        -0.00096370f
#define PITCH_PID_TF        0.12500000f

#define PITCH_RST_NR        3
#define PITCH_RST_NS        3
#define PITCH_RST_NT        3
static const float PITCH_RST_R[3] = {1.00000000f, -1.73360152f, 0.73360152f};
static const float PITCH_RST_S[3] = {-0.00633813f, 0.00818800f, -0.00519258f};
static const float PITCH_RST_T[3] = {-0.00633813f, 0.00818800f, -0.00519258f};

#define PITCH_LQG_N         2
static const float PITCH_LQG_A[4] = {1.29186413f, -0.81926066f, 1.00000000f, 0.00000000f};
static const float PITCH_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float PITCH_LQG_C[2] = {0.00000000f, -3.35084255f};
static const float PITCH_LQG_KX[2] = {0.21224275f, -0.09563298f};
#define PITCH_LQG_KI        +0.73624734f
static const float PITCH_LQG_L[2] = {-0.24791653f, -0.37953163f};

#define PITCH_MRAC_NTHETA   8
#define PITCH_MRAC_NU       2
#define PITCH_MRAC_NY       3
#define PITCH_MRAC_NRF      3
static const float PITCH_MRAC_THETA0[8] = {1.73360152f, -0.73360152f, 0.00633813f, -0.00818800f, 0.00519258f, -0.00633813f, 0.00818800f, -0.00519258f};
static const float PITCH_MRAC_GAMMA[8] = {0.00091195f, 0.00039111f, 0.00001233f, 0.00001329f, 0.00001173f, 0.00001233f, 0.00001329f, 0.00001173f};
static const float PITCH_MRAC_LIM[8] = {0.72811264f, 0.32811264f, 0.03720728f, 0.03794723f, 0.03674906f, 0.03720728f, 0.03794723f, 0.03674906f};
static const float PITCH_MRAC_AM[3] = {1.00000000f, -1.73360152f, 0.75483960f};
static const float PITCH_MRAC_BM[2] = {0.00000000f, 0.02123808f};
#define PITCH_MRAC_SIGMA    0.05000000f
#define PITCH_MRAC_DEADZONE 0.15000000f
#define PITCH_MRAC_SIGN     -1.0f

#endif  // CONTROLLER_PARAMS_H
