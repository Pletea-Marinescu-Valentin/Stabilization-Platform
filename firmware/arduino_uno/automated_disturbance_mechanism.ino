#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

bool experimentDone = false;

void setup() {

  delay(1000);

  servo1.write(20);
  servo1.attach(5);

  servo2.write(20);
  servo2.attach(11);

  servo3.write(160);
  servo3.attach(9);

  delay(1000);
}

void loop() {

  if (experimentDone) return;

  moveSingle(servo1, 20, 45, 35);
  delay(1500);
  moveSingle(servo1, 45, 20, 35);
  delay(1500);

  moveSingle(servo2, 20, 45, 35);
  delay(1500);
  moveSingle(servo2, 45, 20, 35);
  delay(1500);

  moveSingle(servo3, 160, 125, 35);
  delay(1500);
  moveSingle(servo3, 125, 160, 35);
  delay(1500);

  movePair(servo1, 20, 45, servo2, 20, 45, 35);
  delay(1500);
  movePair(servo1, 45, 20, servo2, 45, 20, 35);
  delay(1500);

  movePair(servo1, 20, 45, servo3, 160, 125, 35);
  delay(1500);
  movePair(servo1, 45, 20, servo3, 125, 160, 35);
  delay(1500);

  movePair(servo2, 20, 45, servo3, 160, 125, 35);
  delay(1500);
  movePair(servo2, 45, 20, servo3, 125, 160, 35);
  delay(1500);

  move3Servos(20, 45, 20, 45, 160, 125, 35);
  delay(3000);

  moveSingle(servo1, 45, 75, 35);
  delay(1500);
  moveSingle(servo1, 75, 45, 35);
  delay(1500);

  moveSingle(servo2, 45, 75, 35);
  delay(1500);
  moveSingle(servo2, 75, 45, 35);
  delay(1500);

  moveSingle(servo3, 125, 100, 35);
  delay(1500);
  moveSingle(servo3, 100, 125, 35);
  delay(1500);

  movePair(servo1, 45, 75, servo2, 45, 75, 35);
  delay(1500);
  movePair(servo1, 75, 45, servo2, 75, 45, 35);
  delay(1500);

  movePair(servo1, 45, 75, servo3, 125, 100, 35);
  delay(1500);
  movePair(servo1, 75, 45, servo3, 100, 125, 35);
  delay(1500);

  movePair(servo2, 45, 75, servo3, 125, 100, 35);
  delay(1500);
  movePair(servo2, 75, 45, servo3, 100, 125, 35);

  experimentDone = true;
}

void moveSingle(Servo &s, int posStart, int posEnd, int speedDelay) {

  int pos = posStart;

  while (pos != posEnd) {

    if (pos < posEnd) pos++;
    else if (pos > posEnd) pos--;

    s.write(pos);

    delay(speedDelay);
  }
}

void movePair(Servo &sa, int saStart, int saEnd,
              Servo &sb, int sbStart, int sbEnd,
              int speedDelay) {

  int posA = saStart;
  int posB = sbStart;

  while (posA != saEnd || posB != sbEnd) {

    if (posA < saEnd) posA++;
    else if (posA > saEnd) posA--;

    if (posB < sbEnd) posB++;
    else if (posB > sbEnd) posB--;

    sa.write(posA);
    sb.write(posB);

    delay(speedDelay);
  }
}

void move3Servos(int s1Start, int s1End,
                 int s2Start, int s2End,
                 int s3Start, int s3End,
                 int speedDelay) {

  int pos1 = s1Start;
  int pos2 = s2Start;
  int pos3 = s3Start;

  while (pos1 != s1End || pos2 != s2End || pos3 != s3End) {

    if (pos1 < s1End) pos1++;
    else if (pos1 > s1End) pos1--;

    if (pos2 < s2End) pos2++;
    else if (pos2 > s2End) pos2--;

    if (pos3 < s3End) pos3++;
    else if (pos3 > s3End) pos3--;

    servo1.write(pos1);
    servo2.write(pos2);
    servo3.write(pos3);

    delay(speedDelay);
  }
}
