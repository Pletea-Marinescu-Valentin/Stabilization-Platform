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
// plant: K=-2.2673 deg/deg, wn=43.863 rad/s, zeta=0.1150, delay=2 samples

#define ROLL_PID_KP        -0.00660314f
#define ROLL_PID_KI        -1.44338366f
#define ROLL_PID_KD        -0.00107531f
#define ROLL_PID_TF        0.12500000f

#define ROLL_RST_NR        3
#define ROLL_RST_NS        3
#define ROLL_RST_NT        3
static const float ROLL_RST_R[3] = {1.00000000f, -1.60311742f, 0.60311742f};
static const float ROLL_RST_S[3] = {-0.01624828f, 0.00576353f, -0.01185593f};
static const float ROLL_RST_T[3] = {-0.01624828f, 0.00576353f, -0.01185593f};

#define ROLL_LQR_N         3
static const float ROLL_LQR_KX[3] = {-0.03724765f, 0.05276899f, 0.22544620f};
#define ROLL_LQR_KI        +2.49292341f

#define ROLL_LQG_N         2
static const float ROLL_LQG_A[4] = {0.35471626f, -0.72967298f, 1.00000000f, 0.00000000f};
static const float ROLL_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float ROLL_LQG_C[2] = {0.00000000f, -3.11739921f};
static const float ROLL_LQG_KX[2] = {0.17260661f, 0.02779209f};
#define ROLL_LQG_KI        +1.96086792f
static const float ROLL_LQG_L[2] = {0.18997128f, -0.10941707f};

#define ROLL_MRAC_NTHETA   8
#define ROLL_MRAC_NU       2
#define ROLL_MRAC_NY       3
#define ROLL_MRAC_NRF      3
static const float ROLL_MRAC_THETA0[8] = {1.60311742f, -0.60311742f, 0.01624828f, -0.00576353f, 0.01185593f, -0.01624828f, 0.00576353f, -0.01185593f};
static const float ROLL_MRAC_GAMMA[8] = {0.00084331f, 0.00032247f, 0.00001681f, 0.00001135f, 0.00001452f, 0.00001681f, 0.00001135f, 0.00001452f};
static const float ROLL_MRAC_LIM[8] = {0.67330932f, 0.27330932f, 0.03856166f, 0.03436776f, 0.03680472f, 0.03856166f, 0.03436776f, 0.03680472f};
static const float ROLL_MRAC_AM[3] = {1.00000000f, -1.60311742f, 0.65376979f};
static const float ROLL_MRAC_BM[2] = {0.00000000f, 0.05065236f};
#define ROLL_MRAC_SIGMA    0.05000000f
#define ROLL_MRAC_DEADZONE 0.15000000f
#define ROLL_MRAC_SIGN     -1.0f

// ===================== PITCH =====================
// plant: K=-6.3536 deg/deg, wn=25.040 rad/s, zeta=0.1274, delay=2 samples

#define PITCH_PID_KP        -0.00105886f
#define PITCH_PID_KI        -0.26575034f
#define PITCH_PID_KD        -0.00066805f
#define PITCH_PID_TF        0.12500000f

#define PITCH_RST_NR        3
#define PITCH_RST_NS        3
#define PITCH_RST_NT        3
static const float PITCH_RST_R[3] = {1.00000000f, -1.60311742f, 0.60311742f};
static const float PITCH_RST_S[3] = {-0.01511631f, 0.01952821f, -0.01238420f};
static const float PITCH_RST_T[3] = {-0.01511631f, 0.01952821f, -0.01238420f};

#define PITCH_LQR_N         3
static const float PITCH_LQR_KX[3] = {-0.06654650f, 0.07141255f, 0.29208311f};
#define PITCH_LQR_KI        +0.80858064f

#define PITCH_LQG_N         2
static const float PITCH_LQG_A[4] = {1.29186413f, -0.81926066f, 1.00000000f, 0.00000000f};
static const float PITCH_LQG_B[2] = {1.00000000f, 0.00000000f};
static const float PITCH_LQG_C[2] = {0.00000000f, -3.35084255f};
static const float PITCH_LQG_KX[2] = {0.20194129f, -0.12710464f};
#define PITCH_LQG_KI        +0.48556404f
static const float PITCH_LQG_L[2] = {-0.24791653f, -0.37953163f};

#define PITCH_MRAC_NTHETA   8
#define PITCH_MRAC_NU       2
#define PITCH_MRAC_NY       3
#define PITCH_MRAC_NRF      3
static const float PITCH_MRAC_THETA0[8] = {1.60311742f, -0.60311742f, 0.01511631f, -0.01952821f, 0.01238420f, -0.01511631f, 0.01952821f, -0.01238420f};
static const float PITCH_MRAC_GAMMA[8] = {0.00084331f, 0.00032247f, 0.00001622f, 0.00001852f, 0.00001480f, 0.00001622f, 0.00001852f, 0.00001480f};
static const float PITCH_MRAC_LIM[8] = {0.67330932f, 0.27330932f, 0.03810887f, 0.03987363f, 0.03701603f, 0.03810887f, 0.03987363f, 0.03701603f};
static const float PITCH_MRAC_AM[3] = {1.00000000f, -1.60311742f, 0.65376979f};
static const float PITCH_MRAC_BM[2] = {0.00000000f, 0.05065236f};
#define PITCH_MRAC_SIGMA    0.05000000f
#define PITCH_MRAC_DEADZONE 0.15000000f
#define PITCH_MRAC_SIGN     -1.0f

#endif  // CONTROLLER_PARAMS_H
