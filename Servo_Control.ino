#include <ESP32Servo.h>
#define Speed1 8 // Output pin used
#define Speed2 9

Servo servo1;
Servo servo2;

void setup() {
  servo1.setPeriodHertz(50); // PWM frequency for SG90
  servo1.attach(Speed1, 500, 2400); // Minimum and maximum pulse width (in µs) to go from 0° to 180
  servo2.setPeriodHertz(50); // PWM frequency for SG90
  servo2.attach(Speed2, 500, 2400); // Minimum and maximum pulse width (in µs) to go from 0° to 180
 }
void loop() {
 //rotation from 0 to 180°
  for (int pos = 0; pos <= 180; pos += 1) {
    servo1.write(pos);
    servo2.write(180 - pos);
    delay(10);
  }
 // Rotation from 180° to 0
  for (int pos = 180; pos >= 0; pos -= 1) {
    servo1.write(pos);
    servo2.write(180 - pos);
    delay(10);
  }
}
