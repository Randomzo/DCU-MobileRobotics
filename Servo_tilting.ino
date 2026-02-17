#include <ESP32Servo.h>
#define leftServoPin 3 // left servo control
#define rightServoPin 46 // right servo control

Servo leftServo;
Servo rightServo;

int currentNode = 0;
bool clockwise = true;
bool navigating = true;

void setup() {
  // put your setup code here, to run once:
  leftServo.setPeriodHertz(50); // PWM frequency for SG90
  leftServo.attach(leftServoPin, 500, 2400); // Minimum and maximum pulse width (in µs) to go from 0° to 180
  rightServo.setPeriodHertz(50); // PWM frequency for SG90
  rightServo.attach(rightServoPin, 500, 2400); // Minimum and maximum pulse width (in µs) to go from 0° to 180
}
void tiltLeft() {
  leftServo.write(180);
  rightServo.write(180);
  delay(10);
}

void tiltRight() {
  leftServo.write(0);
  rightServo.write(0);
  delay(10);
}

void straighten() {
  leftServo.write(90);
  rightServo.write(90);
  delay(10);
}

void celebrate() {
  for (int i = 0; i =< 5; i++) {

    leftServo.write(180);
    rightServo.write(0);
    delay(500);

    leftServo.write(0);
    rightServo.write(180);
    delay(500);

    tiltLeft();
    delay(500);

    tiltRight();
    delay(500);
  }
}

void servoTilt() {
  if (navigating == true) {
    switch(currentNode) {
      case 0:
        if (clockwise == true) {straighten();}
        else {tiltLeft();}
        break;
      case 1:
        straighten();
        break;
      case 2:
        if (clockwise == true) {tiltRight();}
        else {straighten();}
        break;
      case 3:
        if (clockwise == true) {straighten();}
        else {tiltLeft();}
        break;
      case 4:
        if (clockwise == true) {tiltRight();}
        else {straighten();}
        break;
      case 5:
        if (clockwise == true) {tiltRight();}
        else {tiltLeft();}
        break;
      case 6:
        if (clockwise == true) {tiltRight();}
        else {tiltLeft();}
        break;
    }
  }
  else {
    celebrate();
  }
}

void loop() {
  servoTilt();
}
