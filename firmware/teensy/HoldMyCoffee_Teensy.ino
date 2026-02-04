#include <ACAN_T4.h>
#include "Moteus.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SD.h>
#include <SPI.h>

File logFile;

// ----------------- Logging setup -----------------
void initSD() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card failed!");
    } else {
        logFile = SD.open("log.csv", FILE_WRITE);
        if (logFile) {
            logFile.println("time_ms,mode,pitch_deg,roll_deg,target_pitch,target_roll,control_pitch,control_roll,motor_pitch_pos,motor_roll_pos");
            logFile.flush();
        }
    }
}

void writeLog(unsigned long t_ms, int mode,
              float pitch_deg, float roll_deg,
              float target_pitch, float target_roll,
              float control_pitch, float control_roll,
              float motor_pitch_pos, float motor_roll_pos)
{
    if (!logFile) return;

    logFile.print(t_ms); logFile.print(",");
    logFile.print(mode); logFile.print(",");
    logFile.print(pitch_deg); logFile.print(",");
    logFile.print(roll_deg); logFile.print(",");
    logFile.print(target_pitch); logFile.print(",");
    logFile.print(target_roll); logFile.print(",");
    logFile.print(control_pitch); logFile.print(",");
    logFile.print(control_roll); logFile.print(",");
    logFile.print(motor_pitch_pos); logFile.print(",");
    logFile.print(motor_roll_pos);
    logFile.println();
}

// ----------------- end logging --------------------

float normalizeAngleDifference(float target, float current) {
    float diff = target - current;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;
    return diff;
}

const float MAX_STEP_PER_CYCLE = 0.35f;

// PID Params
#define PITCH_KP  0.039635f
#define PITCH_KI  0.001982f
#define PITCH_KD  0.001516f
#define ROLL_KP   0.044402f
#define ROLL_KI   0.002220f
#define ROLL_KD   0.000987f
#define MAX_TORQUE 7.0f

Moteus moteus_pitch;
Moteus moteus_roll;
Moteus moteus_hight;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

const float TARGET_PITCH = -0.2f;
const float TARGET_ROLL  = -175.44f;
const float TARGET_DISTANCE = 50.0f;
const float MM_TO_ROT = 0.5f;
const float MAX_POS = 0.5f;
const float MIN_POS = -20.0f;

float distance_from_uno = 0.0f;
float pitch_integral = 0.0f;
float roll_integral  = 0.0f;
unsigned long prev_time = 0;
float pitch_prev_error = 0.0f;
float roll_prev_error  = 0.0f;

#define FILTER_SIZE 5
float distance_buffer[FILTER_SIZE];
int buffer_index = 0;
bool buffer_filled = false;

int controlMode = 1;

// RST parameters
float R_pitch[2] = {0.0063f, 0.0089f};
float S_pitch[2] = {0.0719f, -0.1039f};
float T_pitch = 0.1f;
float u_pitch_prev[2] = {0.0f, 0.0f};
float e_pitch_prev[1] = {0.0f};

float R_roll[2] = {0.0048f, 0.0032f};
float S_roll[2] = {-0.0262f, -0.0600f};
float T_roll    = 0.1f;
float u_roll_prev[2] = {0.0f, 0.0f};
float e_roll_prev[1] = {0.0f};

// LQG parameters
const float Ad_p[2][2] = {{0.535957f, -17.481335f}, {0.025576f, 0.695961f}};
const float Bd_p[2] = {0.025576f, 0.000445f};
const float Cd_p[2] = {-2.596083f, 55.202550f};
const float K_lqg_p[2] = {0.079650f, 0.001463f};
const float L_p[2] = {-0.167880f, 0.011632f};

const float Ad_r[2][2] = {{0.057574f, -39.524690f}, {0.019538f, 0.207428f}};
const float Bd_r[2] = {0.019538f, 0.000392f};
const float Cd_r[2] = {-1.855063f, 36.240060f};
const float K_lqg_r[2] = {0.064978f, 0.000494f};
const float L_r[2] = {-0.191099f, 0.003226f};

float x_hat_p[2] = {0.0f, 0.0f};
float x_hat_r[2] = {0.0f, 0.0f};
float u_prev_lqg_p = 0.0f;
float u_prev_lqg_r = 0.0f;

// Adaptive parameters
float K_adaptive_pitch = 0.02f;
float K_adaptive_roll = 0.02f;
float adaptive_pitch_ref = 0.0f;
float adaptive_roll_ref = 0.0f;
#define ADAPTATION_RATE 0.001f

float smoothed_error_mm = 0.0f;
unsigned long last_ok_time = 0;

float last_u_pitch = 0.0f;
float last_u_roll  = 0.0f;
float last_target_pitch_pos = 0.0f;
float last_target_roll_pos  = 0.0f;
float last_motor_pitch_pos = 0.0f;
float last_motor_roll_pos  = 0.0f;
float last_motor_height_pos = 0.0f;

#define LOOP_INTERVAL_MS 20
unsigned long current_time = 0;
unsigned long last_time = 0;

void addDistanceSample(float new_sample) {
    distance_buffer[buffer_index] = new_sample;
    buffer_index = (buffer_index + 1) % FILTER_SIZE;
    if (buffer_index == 0) buffer_filled = true;
}

float getAverageDistance() {
    int count = buffer_filled ? FILTER_SIZE : buffer_index;
    if (count == 0) return distance_from_uno;
    float sum = 0.0f;
    for (int i = 0; i < count; i++) sum += distance_buffer[i];
    return sum / count;
}

void setup() {
    Serial.begin(115200);
    unsigned long startWait = millis();
    while (!Serial && (millis() - startWait < 2000)) {}

    Serial1.begin(115200);

    initSD();

    if (!bno.begin()) {
        Serial.println("BNO055 Failed!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    ACAN_T4FD_Settings settings(1000000, DataBitRateFactor::x1);
    ACAN_T4::can3.beginFD(settings);

    moteus_pitch.options_.id = 1; moteus_pitch.Initialize(); delay(200);
    moteus_roll.options_.id = 2;  moteus_roll.Initialize();  delay(200);
    moteus_hight.options_.id = 3; moteus_hight.Initialize(); delay(200);
}

void loop() {
    current_time = millis();

    if (current_time - last_time >= LOOP_INTERVAL_MS) {
        last_time = current_time;

        if (Serial.available()) {
            char c = Serial.read();
            if (c == '1') { controlMode = 1; Serial.println("Control Mode: PID"); }
            else if (c == '2') { controlMode = 2; Serial.println("Control Mode: RST"); }
            else if (c == '3') { controlMode = 3; Serial.println("Control Mode: LQG"); }
            else if (c == '4') { controlMode = 4; Serial.println("Control Mode: Adaptive"); }
        }

        if (Serial1.available()) {
            String msg = Serial1.readStringUntil('\n');
            msg.trim();
            if (msg.startsWith("DIST:")) {
                float val = msg.substring(5).toFloat();
                if (val > 0.0f && val < 10000.0f) {
                    distance_from_uno = val;
                    addDistanceSample(val);
                }
            }
        }

        imu::Quaternion quat = bno.getQuat();
        float w = quat.w(), x = quat.x(), y = quat.y(), z = quat.z();
        float sinr_cosp = 2.0f * (w * x + y * z);
        float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
        float roll_rad = atan2(sinr_cosp, cosr_cosp);
        float sinp = 2.0f * (w * y - z * x);
        float pitch_rad = (abs(sinp) >= 1) ? copysign(M_PI / 2, sinp) : asin(sinp);
        float roll_deg  = roll_rad  * 180.0f / M_PI;
        float pitch_deg = pitch_rad * 180.0f / M_PI;

        // Loop timing measurement
        static unsigned long loop_start = 0;
        static unsigned long loop_count = 0;
        static float avg_period_ms = 0;
        unsigned long now_loop = micros();
        if (loop_start > 0) {
            float period_ms = (now_loop - loop_start) / 1000.0f;
            avg_period_ms = avg_period_ms * 0.99f + period_ms * 0.01f;
            loop_count++;
            if (loop_count % 100 == 0) {
                Serial.print("Avg loop period: ");
                Serial.print(avg_period_ms);
                Serial.print(" ms, Freq: ");
                Serial.print(1000.0f / avg_period_ms);
                Serial.println(" Hz");
            }
        }
        loop_start = now_loop;

        float avg_distance = getAverageDistance();
        unsigned long now = micros();
        float dt = (prev_time > 0) ? (now - prev_time) / 1e6f : 0.002f;
        prev_time = now;
        if (dt > 0.1f) dt = 0.002f;

        if (controlMode == 1) runPID(pitch_deg, roll_deg, dt);
        else if (controlMode == 2) runRST(pitch_deg, roll_deg, dt);
        else if (controlMode == 3) runLQG(pitch_deg, roll_deg, dt);
        else if (controlMode == 4) runAdaptive(pitch_deg, roll_deg, dt);

        /*
        // HEIGHT control
        if (moteus_hight.SetQuery() && !isnan(moteus_hight.last_result().values.position)) {
            last_ok_time = millis();
            float raw_error_mm = TARGET_DISTANCE - avg_distance;
            const float alpha = 0.5f;
            smoothed_error_mm = smoothed_error_mm * (1.0f - alpha) + raw_error_mm * alpha;

            float target_pos;
            if (fabs(smoothed_error_mm) > 5.0f)
                target_pos = constrain(smoothed_error_mm * MM_TO_ROT, MIN_POS, MAX_POS);
            else
                target_pos = moteus_hight.last_result().values.position;

            last_motor_height_pos = moteus_hight.last_result().values.position;

            Moteus::PositionMode::Command h_cmd;
            h_cmd.position = target_pos;
            h_cmd.maximum_torque = MAX_TORQUE;
            moteus_hight.BeginPosition(h_cmd);
        }
        */

        if (!isnan(moteus_pitch.last_result().values.position))
            last_motor_pitch_pos = moteus_pitch.last_result().values.position;
        if (!isnan(moteus_roll.last_result().values.position))
            last_motor_roll_pos = moteus_roll.last_result().values.position;

        writeLog(
            millis(),
            controlMode,
            pitch_deg,
            roll_deg,
            (last_target_pitch_pos * 180.0f / M_PI),
            (last_target_roll_pos  * 180.0f / M_PI),
            last_u_pitch,
            last_u_roll,
            last_motor_pitch_pos,
            last_motor_roll_pos
        );
        if (millis() % 100 == 0) logFile.flush();

        moteus_pitch.Poll();
        moteus_roll.Poll();
        //moteus_hight.Poll();
    }

    delay(1);
}

// ==================== CONTROLLERS ====================

// ---- PID ----
void runPID(float pitch_deg, float roll_deg, float dt) {
    float pitch_error = TARGET_PITCH - pitch_deg;
    pitch_integral += pitch_error * dt;
    pitch_integral = constrain(pitch_integral, -50.0f, 50.0f);
    float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
    pitch_prev_error = pitch_error;
    float pitch_correction_deg =
        (PITCH_KP * pitch_error) +
        (PITCH_KI * pitch_integral) +
        (PITCH_KD * pitch_derivative);

    if (moteus_pitch.SetQuery() && !isnan(moteus_pitch.last_result().values.position)) {
        last_ok_time = millis();
        float current_pitch_pos = moteus_pitch.last_result().values.position;
        float pitch_correction_rad = pitch_correction_deg * M_PI / 180.0f;
        float target_pitch_pos = current_pitch_pos - pitch_correction_rad;

        last_u_pitch = pitch_correction_deg;
        last_target_pitch_pos = target_pitch_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_pitch_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_pitch.BeginPosition(cmd);
    }

    float roll_error = normalizeAngleDifference(TARGET_ROLL, roll_deg);
    roll_integral += roll_error * dt;
    roll_integral = constrain(roll_integral, -50.0f, 50.0f);
    float roll_derivative = (roll_error - roll_prev_error) / dt;
    roll_prev_error = roll_error;
    float roll_correction_deg =
        (ROLL_KP * roll_error) +
        (ROLL_KI * roll_integral) +
        (ROLL_KD * roll_derivative);

    if (moteus_roll.SetQuery() && !isnan(moteus_roll.last_result().values.position)) {
        last_ok_time = millis();
        float current_roll_pos = moteus_roll.last_result().values.position;
        float roll_correction_rad = roll_correction_deg * M_PI / 180.0f;
        float target_roll_pos = current_roll_pos - roll_correction_rad;

        last_u_roll = roll_correction_deg;
        last_target_roll_pos = target_roll_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_roll_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_roll.BeginPosition(cmd);
    }
}

// ---- RST ----
void runRST(float pitch_deg, float roll_deg, float dt) {
    float e_pitch = TARGET_PITCH - pitch_deg;

    float u_pitch =
    - R_pitch[0] * u_pitch_prev[0]
    - R_pitch[1] * u_pitch_prev[1]
    + S_pitch[0] * e_pitch
    + S_pitch[1] * e_pitch_prev[0]
    + T_pitch * e_pitch;

    u_pitch_prev[1] = u_pitch_prev[0];
    u_pitch_prev[0] = u_pitch;

    e_pitch_prev[0] = e_pitch;

    if (moteus_pitch.SetQuery() && !isnan(moteus_pitch.last_result().values.position)) {
        last_ok_time = millis();
        float current_pitch_pos = moteus_pitch.last_result().values.position;
        float pitch_correction_rad = u_pitch * M_PI / 180.0f;
        float target_pitch_pos = current_pitch_pos - pitch_correction_rad;

        last_u_pitch = u_pitch;
        last_target_pitch_pos = target_pitch_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_pitch_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_pitch.BeginPosition(cmd);
    }

    float e_roll = normalizeAngleDifference(TARGET_ROLL, roll_deg);

    float u_roll =
    - R_roll[0] * u_roll_prev[0]
    - R_roll[1] * u_roll_prev[1]
    + S_roll[0] * e_roll
    + S_roll[1] * e_roll_prev[0]
    + T_roll * e_roll;

    u_roll_prev[1] = u_roll_prev[0];
    u_roll_prev[0] = u_roll;

    e_roll_prev[0] = e_roll;

    if (moteus_roll.SetQuery() && !isnan(moteus_roll.last_result().values.position)) {
        last_ok_time = millis();
        float current_roll_pos = moteus_roll.last_result().values.position;
        float roll_correction_rad = u_roll * M_PI / 180.0f;
        float target_roll_pos = current_roll_pos - roll_correction_rad;

        last_u_roll = u_roll;
        last_target_roll_pos = target_roll_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_roll_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_roll.BeginPosition(cmd);
    }
}

// ---- LQG ----
void runLQG(float pitch_deg, float roll_deg, float dt) {
    float x_pred_p[2];
    x_pred_p[0] = Ad_p[0][0]*x_hat_p[0] + Ad_p[0][1]*x_hat_p[1] + Bd_p[0]*u_prev_lqg_p;
    x_pred_p[1] = Ad_p[1][0]*x_hat_p[0] + Ad_p[1][1]*x_hat_p[1] + Bd_p[1]*u_prev_lqg_p;
    
    float y_pred_p = Cd_p[0]*x_pred_p[0] + Cd_p[1]*x_pred_p[1];
    float innov_p = pitch_deg - y_pred_p;
    
    x_hat_p[0] = x_pred_p[0] + L_p[0]*innov_p;
    x_hat_p[1] = x_pred_p[1] + L_p[1]*innov_p;
    
    float e_p = TARGET_PITCH - pitch_deg;
    float u_pitch = K_lqg_p[0]*e_p - K_lqg_p[1]*x_hat_p[1];
    
    u_prev_lqg_p = u_pitch;
    
    if (moteus_pitch.SetQuery() && !isnan(moteus_pitch.last_result().values.position)) {
        last_ok_time = millis();
        float current_pitch_pos = moteus_pitch.last_result().values.position;
        float pitch_correction_rad = u_pitch * M_PI / 180.0f;
        float target_pitch_pos = current_pitch_pos - pitch_correction_rad;
        
        last_u_pitch = u_pitch;
        last_target_pitch_pos = target_pitch_pos;
        
        Moteus::PositionMode::Command cmd;
        cmd.position = target_pitch_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_pitch.BeginPosition(cmd);
    }
    
    float x_pred_r[2];
    x_pred_r[0] = Ad_r[0][0]*x_hat_r[0] + Ad_r[0][1]*x_hat_r[1] + Bd_r[0]*u_prev_lqg_r;
    x_pred_r[1] = Ad_r[1][0]*x_hat_r[0] + Ad_r[1][1]*x_hat_r[1] + Bd_r[1]*u_prev_lqg_r;
    
    float y_pred_r = Cd_r[0]*x_pred_r[0] + Cd_r[1]*x_pred_r[1];
    
    float innov_r = roll_deg - y_pred_r;
    
    x_hat_r[0] = x_pred_r[0] + L_r[0]*innov_r;
    x_hat_r[1] = x_pred_r[1] + L_r[1]*innov_r;
    
    float e_r = normalizeAngleDifference(TARGET_ROLL, roll_deg);
    float u_roll = K_lqg_r[0]*e_r - K_lqg_r[1]*x_hat_r[1];
    
    u_prev_lqg_r = u_roll;
    
    if (moteus_roll.SetQuery() && !isnan(moteus_roll.last_result().values.position)) {
        last_ok_time = millis();
        float current_roll_pos = moteus_roll.last_result().values.position;
        float roll_correction_rad = u_roll * M_PI / 180.0f;
        float target_roll_pos = current_roll_pos - roll_correction_rad;
        
        last_u_roll = u_roll;
        last_target_roll_pos = target_roll_pos;
        
        Moteus::PositionMode::Command cmd;
        cmd.position = target_roll_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_roll.BeginPosition(cmd);
    }
}

// ---- Adaptive ----
void runAdaptive(float pitch_deg, float roll_deg, float dt) {
    // === PITCH ===
    float e_pitch = TARGET_PITCH - pitch_deg;
    float tau = 0.5f;
    adaptive_pitch_ref += dt/tau * (TARGET_PITCH - adaptive_pitch_ref);
    float e_track_pitch = adaptive_pitch_ref - pitch_deg;

    float u_pitch = K_adaptive_pitch * e_pitch;
    u_pitch = constrain(u_pitch, -10.0f, 10.0f);

    K_adaptive_pitch += ADAPTATION_RATE * e_track_pitch * e_pitch * dt;
    K_adaptive_pitch = constrain(K_adaptive_pitch, 0.01f, 0.15f);

    if (moteus_pitch.SetQuery() && !isnan(moteus_pitch.last_result().values.position)) {
        last_ok_time = millis();
        float current_pitch_pos = moteus_pitch.last_result().values.position;
        float pitch_correction_rad = u_pitch * M_PI / 180.0f;
        float target_pitch_pos = current_pitch_pos - pitch_correction_rad;

        last_u_pitch = u_pitch;
        last_target_pitch_pos = target_pitch_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_pitch_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_pitch.BeginPosition(cmd);
    }

    // === ROLL ===
    float e_roll = normalizeAngleDifference(TARGET_ROLL, roll_deg);
    adaptive_roll_ref += dt/tau * (TARGET_ROLL - adaptive_roll_ref);
    float e_track_roll = normalizeAngleDifference(adaptive_roll_ref, roll_deg);

    float u_roll = K_adaptive_roll * e_roll;
    u_roll = constrain(u_roll, -10.0f, 10.0f);

    K_adaptive_roll += ADAPTATION_RATE * e_track_roll * e_roll * dt;
    K_adaptive_roll = constrain(K_adaptive_roll, 0.01f, 0.15f);

    if (moteus_roll.SetQuery() && !isnan(moteus_roll.last_result().values.position)) {
        last_ok_time = millis();
        float current_roll_pos = moteus_roll.last_result().values.position;
        float roll_correction_rad = u_roll * M_PI / 180.0f;
        float target_roll_pos = current_roll_pos - roll_correction_rad;

        last_u_roll = u_roll;
        last_target_roll_pos = target_roll_pos;

        Moteus::PositionMode::Command cmd;
        cmd.position = target_roll_pos;
        cmd.maximum_torque = MAX_TORQUE;
        moteus_roll.BeginPosition(cmd);
    }
}