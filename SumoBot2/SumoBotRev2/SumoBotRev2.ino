#include <Servo.h>
#include <NewPing.h>

#define BUTTON_PIN A7

#define FL_IR_PIN A2
#define FR_IR_PIN 10
#define BL_IR_PIN A6
#define BR_IR_PIN 4

#define FL_TRIG_PIN A0
#define FL_ECHO_PIN A1
#define FR_TRIG_PIN 9
#define FR_ECHO_PIN 9
#define BL_TRIG_PIN A5
#define BL_ECHO_PIN A4
#define BR_TRIG_PIN 5
#define BR_ECHO_PIN 6

#define FL_MOT_PIN 12
#define FR_MOT_PIN 11
#define BL_MOT_PIN 3
#define BR_MOT_PIN 2

#define MAX_DIST 100

Servo flMot, frMot, blMot, brMot;
NewPing flSonar(FL_TRIG_PIN, FL_ECHO_PIN, MAX_DIST);
NewPing frSonar(FR_TRIG_PIN, FR_ECHO_PIN, MAX_DIST);
NewPing blSonar(BL_TRIG_PIN, BL_ECHO_PIN, MAX_DIST);
NewPing brSonar(BR_TRIG_PIN, BR_ECHO_PIN, MAX_DIST);

bool movingForward = true;
int hittingLineInt;

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(FL_IR_PIN, INPUT);
  pinMode(FR_IR_PIN, INPUT);
  pinMode(BL_IR_PIN, INPUT);
  pinMode(BR_IR_PIN, INPUT);

  pinMode(FL_TRIG_PIN, OUTPUT);
  pinMode(FR_TRIG_PIN, OUTPUT);
  pinMode(BL_TRIG_PIN, OUTPUT);
  pinMode(BR_TRIG_PIN, OUTPUT);

  pinMode(FL_ECHO_PIN, INPUT);
  pinMode(FR_ECHO_PIN, INPUT);
  pinMode(BL_ECHO_PIN, INPUT);
  pinMode(BR_ECHO_PIN, INPUT);

  flMot.attach(FL_MOT_PIN);
  frMot.attach(FR_MOT_PIN);
  blMot.attach(BL_MOT_PIN);
  brMot.attach(BR_MOT_PIN);

  while (analogRead(BUTTON_PIN) > 5) {
    delay(10);
  }
}

void loop() {
  hittingLineInt = hittingLine();

  if (hittingLineInt != -1) {
    if (hittingLineInt == 1) move(-100, -100);
    else move(100, 100);
    delay(15);
  }

  if (isSeeing()) {
    trackStep();
  }
  else {
    pointTowardOpponent();
  }
}

void trackStep() {
  int flVal = flSonar.ping_cm(); //assigns read values to variables so reading doesn't happen multiple times
  int frVal = frSonar.ping_cm();
  int blVal = blSonar.ping_cm();
  int brVal = brSonar.ping_cm();

  if (flVal != 0 && flVal < frVal && flVal < blVal && flVal < brVal) move(50, 100); //if the front left sensor is the smallest and not zero, turn forward-left
  else if (frVal != 0 && frVal < blVal && frVal < brVal) move(100, 50); //same as above but for front right
  else if (blVal != 0 && blVal < brVal) move(-50, -100); //back left
  else move(-100, -50); //back right
}

void pointTowardOpponent() {
  move(100, -100);
  while (!isSeeing()) {
    delay(15);
  }
}

int hittingLine() { //returns 0 if back hitting, 1 if front hitting, -1 if not hitting
  bool backHitting =  !digitalRead(BL_IR_PIN) || !digitalRead(BR_IR_PIN);
  bool frontHitting = !digitalRead(FL_IR_PIN) || !digitalRead(FR_IR_PIN);

  if (backHitting)  return 0;
  if (frontHitting) return 1;
  else              return -1;
}

boolean isSeeing() {
  int flVal = flSonar.ping_cm();
  int frVal = frSonar.ping_cm();
  int blVal = blSonar.ping_cm();
  int brVal = brSonar.ping_cm();

  return !(flVal + frVal + blVal + brVal == 0);
}

void move(int l, int r) {
  setMotor(1, -1 * l);
  setMotor(0, r);

  movingForward = l > 0 && r > 0;
}

void setMotor(int side, int motorSpeed) { //0 for right, 1 for left
  motorSpeed = map(motorSpeed, -100, 100, 0, 180);

  if (side) {
    flMot.write(motorSpeed);
    brMot.write(motorSpeed);
  }
  else {
    frMot.write(motorSpeed);
    blMot.write(motorSpeed);
  }
}
