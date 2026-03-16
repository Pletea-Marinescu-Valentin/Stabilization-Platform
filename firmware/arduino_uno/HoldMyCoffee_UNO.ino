#include <Wire.h>
#include <Servo.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_VL53L0X.h>
#include <utility/imumaths.h>
#include <math.h>

Servo servo1; 
Servo servo2;

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

float p_ref = 0, r_ref = 0;
float distance_real = -1;
int distance_perp = -1;

float getRealDistance(float d_perp, float pitch_deg, float roll_deg) {
  float pitch_rad = pitch_deg * DEG_TO_RAD;
  float roll_rad  = roll_deg  * DEG_TO_RAD;
  return d_perp / (fabs(cos(pitch_rad)) * fabs(cos(roll_rad)));
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  servo1.attach(9);
  servo2.attach(10);

  Serial.println("UNO READY - BNO055 + VL53L0X (fast update)");

  if (!bno.begin()) {
    Serial.println("BNO055 not detected! Check wiring.");
    while (1);
  }

  delay(1000);

  if (!lox.begin()) {
    Serial.println("VL53L0X not detected! Check wiring or XSHUT HIGH");
    while (1);
  }

  Serial.println("Sensors initialized successfully!");

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  p_ref = atan2(grav.x(), sqrt(grav.y()*grav.y() + grav.z()*grav.z())) * 57.2958;
  r_ref = atan2(grav.y(), grav.z()) * 57.2958;

  servo1.write(90);
  servo2.write(90);
}

void loop() {

  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

  float current_p = atan2(grav.x(), sqrt(grav.y()*grav.y() + grav.z()*grav.z())) * 57.2958;
  float current_r = atan2(grav.y(), grav.z()) * 57.2958;

  float diff_p = current_p - p_ref;
  float diff_r = current_r - r_ref;

  int s1_cmd = 90 - (int)diff_r;
  int s2_cmd = 90 + (int)diff_p;

  servo1.write(constrain(s2_cmd, 0, 180));
  servo2.write(constrain(s1_cmd, 0, 180));

  Serial.print("P:"); Serial.print(current_p,2);
  Serial.print(" R:"); Serial.println(current_r,2);

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);

  distance_perp = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  if (distance_perp >= 0) {
    distance_real = getRealDistance(distance_perp, current_p, current_r);
    Serial.print("DIST:");
    Serial.println(distance_real, 2);
  }

  Serial.print(" Dp:");
  if(distance_perp>=0) Serial.print(distance_perp);
  else Serial.print("N/A");

  Serial.print(" Dr:");
  if(distance_perp>=0) Serial.print(distance_real,2);
  else Serial.print("N/A");

  Serial.println(" mm");

  delay(20);
}