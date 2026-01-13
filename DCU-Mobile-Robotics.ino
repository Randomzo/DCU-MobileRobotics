// Motor pin definitions (CHANGE if needed)
int motor1PWM   = 37;   // AENABLE
int motor1Phase = 38;   // APHASE

int motor2PWM   = 39;   // BENABLE
int motor2Phase = 2;   // BPHASE

void setup() 
{
  Serial.begin(9600);
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);
}

void loop()
{
  
  GoForward(2000,245,255);
  GoStop(2000);
  GoBackward(2000,245,255);
  GoStop(2000);
}

void GoForward(int time,int speed1,int speed2)
  {
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, speed1); // set speed of motor
  analogWrite(motor2PWM, speed2);
  Serial.println("Forward"); // Display motor direction
  delay(time);
  }
void GoBackward(int time,int speed1,int speed2)
 {
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, speed1); // set speed of motor
  analogWrite(motor2PWM, speed2);
  Serial.println("Yeild"); // Display motor direction
  delay(time);
  }
void GoStop(int time)
{
  analogWrite(motor1PWM, 0); // set speed of motor
  analogWrite(motor2PWM, 0);
  delay(time);
}

