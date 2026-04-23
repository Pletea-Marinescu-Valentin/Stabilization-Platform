#include <Servo.h>

#define PPM_PIN 2
#define CHANNELS 8

volatile unsigned int ppm[CHANNELS];
volatile byte channel = 0;
volatile unsigned long lastMicros = 0;

Servo servo1, servo2, servo3;

float s1Curr = 20, s2Curr = 20, s3Curr = 160;
int s1Target, s2Target, s3Target;

unsigned long lastMoveTime = 0;
int moveSpeed = 35;
unsigned long lastDebug = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PPM_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PPM_PIN), readPPM, RISING);
  servo1.attach(5);
  servo2.attach(11);
  servo3.attach(9);
  servo1.write(s1Curr);
  servo2.write(s2Curr);
  servo3.write(s3Curr);
  delay(500);
}

void loop() {
  noInterrupts();
  int ch3 = ppm[2];
  int ch4 = ppm[3];
  int ch6 = ppm[5];
  interrupts();

  if (ch6 < 1250) {
    s1Target = 20;
    s2Target = 20;
    s3Target = 160;
  } else if (ch6 <= 1750) {
    s1Target = 45;
    s2Target = 45;
    s3Target = 130;
  } else {
    s1Target = map(constrain(ch4, 1330, 1670), 1330, 1670, 40, 60);
    s2Target = map(constrain(ch4, 1330, 1670), 1330, 1670, 60, 40);
    s3Target = map(constrain(ch3, 1100, 1900), 1100, 1900, 110, 135);
  }

  if (millis() - lastMoveTime >= moveSpeed) {
    lastMoveTime = millis();
    if (s1Curr < s1Target) s1Curr++;
    else if (s1Curr > s1Target) s1Curr--;
    if (s2Curr < s2Target) s2Curr++;
    else if (s2Curr > s2Target) s2Curr--;
    if (s3Curr < s3Target) s3Curr++;
    else if (s3Curr > s3Target) s3Curr--;
    servo1.write((int)s1Curr);
    servo2.write((int)s2Curr);
    servo3.write((int)s3Curr);
  }

  if (millis() - lastDebug > 200) {
    lastDebug = millis();
    Serial.print("CH3: "); Serial.print(ch3);
    Serial.print(" | CH4: "); Serial.print(ch4);
    Serial.print(" | CH6: "); Serial.print(ch6);
    Serial.print(" || S1: "); Serial.print((int)s1Curr);
    Serial.print(" S2: "); Serial.print((int)s2Curr);
    Serial.print(" S3: "); Serial.print((int)s3Curr);
    Serial.print(" | MODE: ");
    if (ch6 < 1250) Serial.print("SAFE");
    else if (ch6 <= 1750) Serial.print("MID");
    else Serial.print("MANUAL");
    Serial.println();
  }
}

void readPPM() {
  unsigned long now = micros();
  unsigned int diff = now - lastMicros;
  lastMicros = now;
  if (diff > 3000) {
    channel = 0;
  } else {
    if (channel < CHANNELS) {
      ppm[channel] = diff;
      channel++;
    }
  }
}