// ---------- Sensors ----------
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15};   // change pins as needed
int threshold = 1000;               // <1000 = white (line), >1000 = black (background)


// ---------- Motors ----------
int motor1PWM   = 37;
int motor1Phase = 38;
int motor2PWM   = 39;
int motor2Phase = 2;


// ---------- Tuning ----------
int baseSpeed = 200;     // base speed 0-255
float Kp = 190.0;         // proportional gain
float Kd = 45.0;         // derivative gain
float lastError = 0;


void setup() {
  // Motor pins
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);


  // Sensor pins
  for(int i=0;i<5;i++) pinMode(AnalogPin[i], INPUT);


  // Motors forward by default
  digitalWrite(motor1Phase,HIGH);
  digitalWrite(motor2Phase,HIGH);
}


// ---------- Read sensors ----------
void ReadSensors() {
  for(int i=0;i<5;i++){
    AnalogValue[i] = analogRead(AnalogPin[i]);
  }
}


// ---------- Calculate weighted error ----------
float GetError() {
  float numerator = 0;
  float denominator = 0;
  for(int i=0;i<5;i++){
    int val = AnalogValue[i];
    // invert for white line on black background
    val = 4095 - val;
    if(val < threshold) val = threshold; // clamp min to threshold
    numerator += (i-2)*val;   // weight: -2..+2
    denominator += val;
  }
  if(denominator == 0) return lastError; // line lost
  return numerator / denominator;
}


// ---------- Motor update ----------
void SetMotors(int leftSpeed, int rightSpeed){
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed,0,255);


  analogWrite(motor1PWM,leftSpeed);
  analogWrite(motor2PWM,rightSpeed);


  // Direction is always forward
  digitalWrite(motor1Phase,HIGH);
  digitalWrite(motor2Phase,HIGH);
}


// ---------- Main loop ----------
void loop()
{
  ReadSensors();


  // ---- Find min sensor (white line) ----
  int minIndex = 0;
  int minValue = AnalogValue[0];


  for (int i = 1; i < 5; i++) {
    if (AnalogValue[i] < minValue) {
      minValue = AnalogValue[i];
      minIndex = i;
    }
  }


  // ---- LINE LOST ----
  if (minValue > threshold) {
    // Spin in last known direction
    int spin = 130;
    if (lastError < 0) {
      SetMotors(baseSpeed + spin, baseSpeed - spin); // turn right
    } else {
      SetMotors(baseSpeed - spin, baseSpeed + spin); // turn left
    }
    return;
  }


  // ---- SHARP EDGE DETECTION ----
  if (minIndex == 0) {
    // HARD RIGHT TURN
    SetMotors(220, 40);
    lastError = -3;
    return;
  }


  if (minIndex == 4) {
    // HARD LEFT TURN
    SetMotors(40, 220);
    lastError = 3;
    return;
  }


  // ---- NORMAL PD CONTROL ----
  float error = minIndex - 2;  // -2,-1,0,1,2
  float correction = Kp * error + Kd * (error - lastError);
  lastError = error;


  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;


  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);


  SetMotors(leftSpeed, rightSpeed);
}
