// ---------- Sensors ----------
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5] = {4,5,6,7,15};   // change pins as needed
int threshold = 1000;               // <1000 = white (line), >1000 = black (background)

// ---------- Motors ----------
int motor1PWM   = 37; //right motor
int motor1Phase = 38;
int motor2PWM   = 39; //left motor
int motor2Phase = 20;


// ---------- Tuning ----------
int baseSpeed = 200;     // base speed 0-255
float Kp = 190.0;         // proportional gain
float Kd = 45.0;         // derivative gain
float lastError = 0;

//Pathfinding
int currentNode = 10;
bool clockwise = 0;
bool atNode = 0;
bool middleTrack = 0;

void setup() {

  Serial.begin(9600);
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
  int detectingWhite = 0;
  for(int i=0;i<5;i++){
    AnalogValue[i] = analogRead(AnalogPin[i]);
    if (AnalogValue[i] < threshold) {detectingWhite++;}
  }
  if (detectingWhite > 3) {atNode = 1;}
  else {atNode = 0;}
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

void readNode() {
  switch(currentNode) {
    case 10:
      currentNode = 0;
      break;
    case 0:
      if (clockwise == 1) {currentNode = 4;}
      else {currentNode = 6;}
      break;
    case 6:
      if (clockwise == 1) {currentNode = 0;}
      else {currentNode = 2;}
      break;
    case 2:
      if (clockwise == 1) {currentNode = 6;}
      else {currentNode = 3;}
      break;
    case 3:
      if (clockwise == 1) {currentNode = 2;}
      else {currentNode = 7;}
      break;
    case 7:
      if (clockwise == 1) {currentNode = 3;}
      else {currentNode = 4;}
      break;
    case 4:
      if (clockwise == 1) {currentNode = 7;}
      else {currentNode = 0;}
      break;
  }
  Serial.print("This is node ");
  Serial.println(currentNode);
}

void lineFollowing() {
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

void motorStop() {
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

void turn180() {
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);

  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);

  delay(750);
  clockwise = !clockwise;
}

void turnRight() {
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);

  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);

  delay(375);
}

void turnLeft() {
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW);

  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);

  delay(375);
}

//
void goToNode(int target) {
  if (target == 1) {
    while (currentNode != target) {
      while (currentNode != 6 || 7) {
        ReadSensors();
        if (atNode == 1) {
          readNode();
          delay(100);
        }
        lineFollowing();
      }       
      if (currentNode == 6) {
        if (clockwise == 0) {
          turnRight();
          
        }
      }
    }
  }
  while (currentNode != target) {
     ReadSensors();
    if (atNode == 1) {
      readNode();
      delay(100);
    }
    lineFollowing();
  }
}

// ---------- Main loop ----------
void loop()
{
  goToNode(4);
  turn180();
  goToNode(0);
  while(true) {motorStop();}
}
