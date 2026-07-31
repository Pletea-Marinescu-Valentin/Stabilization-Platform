#include <ACAN_T4.h>
#include "Moteus.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>

#include "controller_params.h"

static const float DEG_TO_MOTOR_REV = 0.01745329f;

#define MOTEUS_MODE_FAULT 1
#define MAX_TORQUE        7.0f

enum ControlMode { MODE_PID = 1, MODE_RST = 2, MODE_LQG = 3, MODE_MRAC = 4 };
static volatile int controlMode = MODE_PID;

static const float TARGET_ROLL_DEG  = -175.44f;
static const float TARGET_PITCH_DEG = -0.20f;

Moteus moteus_pitch, moteus_roll, moteus_height;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

IntervalTimer controlTimer;
static volatile bool tick = false;
static volatile uint32_t missedTicks = 0;

void onControlTick() {
    if (tick) missedTicks++;
    tick = true;
}

static inline float wrap180(float deg) {

    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

struct AxisController {

    float u_prev;
    bool  saturated;

    float kp, ki, kd, tf;
    float integral, deriv, e_prev, aFilt, kdGain, tt;

    const float *R, *S, *T;
    int nR, nS, nT;
    float uHist[4], yHist[4], rHist[4];

    const float *A, *B, *C, *Kx, *L;
    float Ki_lqg;
    int   n;
    float xhat[4], xint;

    const float *theta0, *gamma, *lim, *am, *bm;
    int   nTheta, nU, nY, nRf;
    float theta[8], ymHist[4], rmHist[4];
    float sigma, deadzone, sign;
};

static AxisController rollC, pitchC;

static void axisReset(AxisController *c) {
    c->u_prev = 0.0f; c->saturated = false;
    c->integral = c->deriv = c->e_prev = 0.0f;
    for (int i = 0; i < 4; i++) {
        c->uHist[i] = c->yHist[i] = c->rHist[i] = 0.0f;
        c->xhat[i] = 0.0f; c->ymHist[i] = c->rmHist[i] = 0.0f;
    }
    c->xint = 0.0f;
    for (int i = 0; i < c->nTheta; i++) c->theta[i] = c->theta0[i];
}

static float limitCommand(AxisController *c, float u) {
    float du = clampf(u - c->u_prev, -CTRL_DU_MAX, CTRL_DU_MAX);
    float ul = clampf(c->u_prev + du, -CTRL_U_MAX, CTRL_U_MAX);
    c->saturated = fabsf(ul - u) > 1e-6f;
    c->u_prev = ul;
    return ul;
}

static float stepPID(AxisController *c, float r, float y) {
    float e = r - y;
    c->deriv = c->aFilt * c->deriv + c->kdGain * (e - c->e_prev);
    float uRaw = c->kp * e + c->integral + c->deriv;
    float u = limitCommand(c, uRaw);
    c->integral += c->ki * CTRL_TS * e;
    if (c->tt > 0.0f) c->integral += (CTRL_TS / c->tt) * (u - uRaw);
    c->e_prev = e;
    return u;
}

static float stepRST(AxisController *c, float r, float y) {
    for (int i = c->nS - 1; i > 0; i--) c->yHist[i] = c->yHist[i - 1];
    c->yHist[0] = y;
    for (int i = c->nT - 1; i > 0; i--) c->rHist[i] = c->rHist[i - 1];
    c->rHist[0] = r;

    float acc = 0.0f;
    for (int i = 0; i < c->nT; i++) acc += c->T[i] * c->rHist[i];
    for (int i = 0; i < c->nS; i++) acc -= c->S[i] * c->yHist[i];
    for (int i = 1; i < c->nR; i++) acc -= c->R[i] * c->uHist[i - 1];

    float u = limitCommand(c, acc);
    for (int i = c->nR - 2; i > 0; i--) c->uHist[i] = c->uHist[i - 1];
    if (c->nR > 1) c->uHist[0] = u;
    return u;
}

static float stepLQG(AxisController *c, float r, float y) {
    float uRaw = -c->Ki_lqg * c->xint;
    for (int i = 0; i < c->n; i++) uRaw -= c->Kx[i] * c->xhat[i];
    float u = limitCommand(c, uRaw);

    float yhat = 0.0f;
    for (int i = 0; i < c->n; i++) yhat += c->C[i] * c->xhat[i];
    float innov = y - yhat;

    float xn[4];
    for (int i = 0; i < c->n; i++) {
        float acc = 0.0f;
        for (int j = 0; j < c->n; j++) acc += c->A[i * c->n + j] * c->xhat[j];
        xn[i] = acc + c->B[i] * u + c->L[i] * innov;
    }
    for (int i = 0; i < c->n; i++) c->xhat[i] = xn[i];

    if (!c->saturated) c->xint += CTRL_TS * (r - y);
    return u;
}

static float stepMRAC(AxisController *c, float r, float y) {

    for (int i = c->nRf - 1; i > 0; i--) c->rmHist[i] = c->rmHist[i - 1];
    c->rmHist[0] = r;
    float ym = 0.0f;
    for (int i = 0; i < 2; i++) ym += c->bm[i] * c->rmHist[i];
    for (int i = 1; i < 3; i++) ym -= c->am[i] * c->ymHist[i - 1];
    for (int i = 2; i > 0; i--) c->ymHist[i] = c->ymHist[i - 1];
    c->ymHist[0] = ym;

    for (int i = c->nY - 1; i > 0; i--) c->yHist[i] = c->yHist[i - 1];
    c->yHist[0] = y;
    for (int i = c->nRf - 1; i > 0; i--) c->rHist[i] = c->rHist[i - 1];
    c->rHist[0] = r;

    float phi[8];
    int k = 0;
    for (int i = 0; i < c->nU;  i++) phi[k++] = c->uHist[i];
    for (int i = 0; i < c->nY;  i++) phi[k++] = c->yHist[i];
    for (int i = 0; i < c->nRf; i++) phi[k++] = c->rHist[i];

    float uRaw = 0.0f;
    for (int i = 0; i < c->nTheta; i++) uRaw += c->theta[i] * phi[i];
    float u = limitCommand(c, uRaw);

    float e1 = y - ym;
    if (fabsf(e1) > c->deadzone && !c->saturated) {
        float norm = 1.0f;
        for (int i = 0; i < c->nTheta; i++) norm += phi[i] * phi[i];
        float g = c->sign * e1 / norm;
        for (int i = 0; i < c->nTheta; i++) c->theta[i] -= c->gamma[i] * g * phi[i];
    }
    for (int i = 0; i < c->nTheta; i++) {
        c->theta[i] += c->sigma * CTRL_TS * (c->theta0[i] - c->theta[i]);
        c->theta[i] = clampf(c->theta[i], c->theta0[i] - c->lim[i],
                                          c->theta0[i] + c->lim[i]);
    }

    for (int i = c->nU - 1; i > 0; i--) c->uHist[i] = c->uHist[i - 1];
    if (c->nU > 0) c->uHist[0] = u;
    return u;
}

static float stepAxis(AxisController *c, float r, float y) {
    switch (controlMode) {
        case MODE_RST:  return stepRST(c, r, y);
        case MODE_LQG:  return stepLQG(c, r, y);
        case MODE_MRAC: return stepMRAC(c, r, y);
        default:        return stepPID(c, r, y);
    }
}

static void configureAxes() {
    rollC.kp = ROLL_PID_KP; rollC.ki = ROLL_PID_KI;
    rollC.kd = ROLL_PID_KD; rollC.tf = ROLL_PID_TF;
    rollC.R = ROLL_RST_R; rollC.S = ROLL_RST_S; rollC.T = ROLL_RST_T;
    rollC.nR = ROLL_RST_NR; rollC.nS = ROLL_RST_NS; rollC.nT = ROLL_RST_NT;
    rollC.A = ROLL_LQG_A; rollC.B = ROLL_LQG_B; rollC.C = ROLL_LQG_C;
    rollC.Kx = ROLL_LQG_KX; rollC.L = ROLL_LQG_L;
    rollC.Ki_lqg = ROLL_LQG_KI; rollC.n = ROLL_LQG_N;
    rollC.theta0 = ROLL_MRAC_THETA0; rollC.gamma = ROLL_MRAC_GAMMA;
    rollC.lim = ROLL_MRAC_LIM; rollC.am = ROLL_MRAC_AM; rollC.bm = ROLL_MRAC_BM;
    rollC.nTheta = ROLL_MRAC_NTHETA; rollC.nU = ROLL_MRAC_NU;
    rollC.nY = ROLL_MRAC_NY; rollC.nRf = ROLL_MRAC_NRF;
    rollC.sigma = ROLL_MRAC_SIGMA; rollC.deadzone = ROLL_MRAC_DEADZONE;
    rollC.sign = ROLL_MRAC_SIGN;

    pitchC.kp = PITCH_PID_KP; pitchC.ki = PITCH_PID_KI;
    pitchC.kd = PITCH_PID_KD; pitchC.tf = PITCH_PID_TF;
    pitchC.R = PITCH_RST_R; pitchC.S = PITCH_RST_S; pitchC.T = PITCH_RST_T;
    pitchC.nR = PITCH_RST_NR; pitchC.nS = PITCH_RST_NS; pitchC.nT = PITCH_RST_NT;
    pitchC.A = PITCH_LQG_A; pitchC.B = PITCH_LQG_B; pitchC.C = PITCH_LQG_C;
    pitchC.Kx = PITCH_LQG_KX; pitchC.L = PITCH_LQG_L;
    pitchC.Ki_lqg = PITCH_LQG_KI; pitchC.n = PITCH_LQG_N;
    pitchC.theta0 = PITCH_MRAC_THETA0; pitchC.gamma = PITCH_MRAC_GAMMA;
    pitchC.lim = PITCH_MRAC_LIM; pitchC.am = PITCH_MRAC_AM; pitchC.bm = PITCH_MRAC_BM;
    pitchC.nTheta = PITCH_MRAC_NTHETA; pitchC.nU = PITCH_MRAC_NU;
    pitchC.nY = PITCH_MRAC_NY; pitchC.nRf = PITCH_MRAC_NRF;
    pitchC.sigma = PITCH_MRAC_SIGMA; pitchC.deadzone = PITCH_MRAC_DEADZONE;
    pitchC.sign = PITCH_MRAC_SIGN;

    for (AxisController *c : {&rollC, &pitchC}) {
        c->aFilt  = c->tf / (CTRL_TS + c->tf);
        c->kdGain = c->kd / (CTRL_TS + c->tf);
        c->tt     = (c->ki != 0.0f) ? fabsf(c->kp / c->ki) : 0.0f;
        axisReset(c);
    }
}

static void readAttitude(float *rollDeg, float *pitchDeg) {
    imu::Quaternion q = bno.getQuat();
    const float w = q.w(), x = q.x(), y = q.y(), z = q.z();

    *rollDeg = atan2f(2.0f * (w * x + y * z),
                      1.0f - 2.0f * (x * x + y * y)) * 57.29577951f;

    const float sp = 2.0f * (w * y - z * x);
    *pitchDeg = (2.0f * atan2f(sqrtf(fmaxf(0.0f, 1.0f + sp)),
                               sqrtf(fmaxf(0.0f, 1.0f - sp)))
                 - 1.57079633f) * 57.29577951f;
}

File logFile;

static void initSD() {
    if (!SD.begin(BUILTIN_SDCARD)) { Serial.println("SD init failed"); return; }
    logFile = SD.open("log.csv", FILE_WRITE);
    if (logFile) {
        logFile.println("time_ms,mode,roll_deg,pitch_deg,e_roll,e_pitch,"
                        "u_roll,u_pitch,motor_roll,motor_pitch,missed");
        logFile.flush();
    }
}

void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 2000) {}

    Serial1.begin(115200);
    initSD();

    if (!bno.begin()) { Serial.println("BNO055 not found"); while (1) {} }
    bno.setExtCrystalUse(true);

    ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);
    ACAN_T4::can3.beginFD(settings);

    moteus_pitch.options_.id  = 1; moteus_pitch.Initialize();  delay(200);
    moteus_roll.options_.id   = 2; moteus_roll.Initialize();   delay(200);
    moteus_height.options_.id = 3; moteus_height.Initialize(); delay(200);

    configureAxes();
    controlTimer.begin(onControlTick, CTRL_PERIOD_US);
}

void loop() {

    if (moteus_pitch.SetQuery())  moteus_pitch.Poll();
    if (moteus_roll.SetQuery())   moteus_roll.Poll();
    if (moteus_height.SetQuery()) moteus_height.Poll();

    if (Serial.available()) {
        char ch = Serial.read();
        if (ch >= '1' && ch <= '4') {
            controlMode = ch - '0';
            axisReset(&rollC);
            axisReset(&pitchC);
        }
    }

    if (!tick) return;
    tick = false;

    float rollDeg, pitchDeg;
    readAttitude(&rollDeg, &pitchDeg);

    float eRoll  = wrap180(TARGET_ROLL_DEG  - rollDeg);
    float ePitch = wrap180(TARGET_PITCH_DEG - pitchDeg);

    float uRoll  = stepAxis(&rollC,  0.0f, -eRoll);
    float uPitch = stepAxis(&pitchC, 0.0f, -ePitch);

    if (static_cast<int>(moteus_roll.last_result().values.mode) == MOTEUS_MODE_FAULT) {
        moteus_roll.SetStop();
        axisReset(&rollC);
        return;
    }

    float mRoll = moteus_roll.last_result().values.position;
    if (!isnan(mRoll)) {
        Moteus::PositionMode::Command cmd;
        cmd.position = mRoll - uRoll * DEG_TO_MOTOR_REV;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_roll.BeginPosition(cmd);
    }

    float mPitch = moteus_pitch.last_result().values.position;
    if (!isnan(mPitch)) {
        Moteus::PositionMode::Command cmd;
        cmd.position = mPitch - uPitch * DEG_TO_MOTOR_REV;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_pitch.BeginPosition(cmd);
    }

    if (logFile) {
        logFile.printf("%lu,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%lu\n",
                       millis(), controlMode, rollDeg, pitchDeg, eRoll, ePitch,
                       uRoll, uPitch, mRoll, mPitch, missedTicks);
        static uint32_t lastFlush = 0;
        if (millis() - lastFlush > 500) { logFile.flush(); lastFlush = millis(); }
    }
}
