#define LeftSharp 12   
#define RightSharp 13
#define TRIG_PIN 17
#define ECHO_PIN 18

void setup() {
  Serial.begin(115200);
  analogReadResolution(12); // ESP32 ADC is 12-bit (0–4095)
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LeftSharp, INPUT);
  pinMode(RightSharp, INPUT);
}

float sharpDist(int sensor) {
  int adcValue = analogRead(sensor);
  float voltage = adcValue * (3.3 / 4095.0);

  // GP2Y0A41SK0F distance formula (approximation)
  float distance = 12.08 * pow(voltage, -1.058);

  return distance;
}

float ultrasonicMeasure() {
  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // measure duration of pulse from ECHO pin
  float duration_us = pulseIn(ECHO_PIN, HIGH);

  // calculate the distance
  float distance_cm = 0.017 * duration_us;

  return distance_cm;
}

float middleDist() {
  float filterArray[20]; // array to store data samples from sensor

  // 1. TAKING MULTIPLE MEASUREMENTS AND STORE IN AN ARRAY
  for (int sample = 0; sample < 20; sample++) {
    filterArray[sample] = ultrasonicMeasure();
    delay(30); // to avoid untrasonic interfering
  }

  // 2. SORTING THE ARRAY IN ASCENDING ORDER
  for (int i = 0; i < 19; i++) {
    for (int j = i + 1; j < 20; j++) {
      if (filterArray[i] > filterArray[j]) {
        float swap = filterArray[i];
        filterArray[i] = filterArray[j];
        filterArray[j] = swap;
      }
    }
  }

  // 3. FILTERING NOISE
  // + the five smallest samples are considered as noise -> ignore it
  // + the five biggest  samples are considered as noise -> ignore it
  // ----------------------------------------------------------------
  // => get average of the 10 middle samples (from 5th to 14th)
  double sum = 0;
  for (int sample = 5; sample < 15; sample++) {
    sum += filterArray[sample];
  }

  float distance = sum / 10;
  
  return distance;
}

void loop() {
  float leftDist = sharpDist(LeftSharp);
  float rightDist = sharpDist(RightSharp);
  float middleDistance = middleDist();

  Serial.print(leftDist);
  Serial.print("cm      ");
  Serial.print(middleDistance);
  Serial.print("cm      ");
  Serial.print(rightDist);
  Serial.println("cm");

  delay(500);
}