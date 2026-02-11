#include <Arduino.h>
#include <WiFi.h>

// ===================== WIFI / SERVER =====================
const char* ssid     = "ssid";
const char* password = "internet password";

static const char* apiHost = "3.250.38.184";
static const int   apiPort = 8000;
static const char* TEAM_ID = "team id";

WiFiClient client;
static const uint32_t HTTP_TIMEOUT_MS = 10000;

void connectToWiFi(uint32_t timeoutMs = 20000) {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(200);

  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(300);
  }

  if (WiFi.status() == WL_CONNECTED) Serial.println("WiFi: connected");
  else                               Serial.println("WiFi: FAILED");
}

bool ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) return true;
  connectToWiFi();
  return (WiFi.status() == WL_CONNECTED);
}

bool httpConnect() {
  if (!ensureWiFi()) {
    Serial.println("WiFi: not connected (cannot reach server)");
    return false;
  }

  if (client.connected()) {
    client.stop();
    delay(50);
  }

  bool ok = client.connect(apiHost, apiPort);
  if (ok) Serial.println("Server: connected");
  else    Serial.println("Server: FAILED");
  return ok;
}

void httpStop() {
  if (client.connected()) {
    client.stop();
    delay(50);
  }
}

String readHTTPResponse() {
  String response = "";
  uint32_t startTime = millis();

  while (millis() - startTime < HTTP_TIMEOUT_MS) {
    while (client.available()) response += (char)client.read();
    if (!client.connected() && response.length() > 0) break;
    delay(10);
  }
  return response;
}

String extractBody(const String& response) {
  int bodyStart = response.indexOf("\r\n\r\n");
  if (bodyStart == -1) return response;
  String body = response.substring(bodyStart + 4);
  body.trim();
  return body;
}

String apiPostArrived(int position) {
  if (!httpConnect()) return "";

  String postData = "position=" + String(position);

  String request = String("POST /api/arrived/") + TEAM_ID + " HTTP/1.1\r\n";
  request += "Host: " + String(apiHost) + "\r\n";
  request += "Content-Type: application/x-www-form-urlencoded\r\n";
  request += "Content-Length: " + String(postData.length()) + "\r\n";
  request += "Connection: close\r\n\r\n";
  request += postData;

  client.print(request);
  client.flush();

  String body = extractBody(readHTTPResponse());
  httpStop();

  Serial.print("Server said (next): ");
  Serial.println(body);
  return body;
}

bool isFinishedMessage(String s) {
  s.trim();
  s.toLowerCase();
  return (s == "finished");
}

// ===================== ROBOT HARDWARE CONFIGURATION =====================
// ---------- Sensors ----------
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5]   = {4,5,6,7,15};
int threshold      = 1000;

// ---------- Motors ----------
int motor1PWM   = 37;
int motor1Phase = 38;
int motor2PWM   = 39;
int motor2Phase = 20;

// ---------- Line Following Tuning (KP/KD increased significantly) ----------
int   baseSpeed   = 200;     // 0-255
float Kp          = 320.0;   // increased
float Kd          = 110.0;   // increased
float lastError   = 0;

// ---------- Turn tuning ----------
const int TURN_PRE_FORWARD_MS = 120;
const int TURN_PRE_PAUSE_MS   = 50;
const int TURN90_MS           = 350;

// ---------- Parking behavior ----------
const unsigned long PARK_WAIT_MS = 1500;

// ===================== ULTRASONIC (HC-SR04) =====================
// NOTE: HC-SR04 ECHO is 5V. Use a voltage divider to 3.3V for ESP32 input.
const int US_TRIG_PIN = 16;
const int US_ECHO_PIN = 17;

// Stop as close as practical (HC-SR04 gets unreliable below ~2–3cm)
const float PARK_STOP_CM = 5.0f;
const uint32_t US_SAMPLE_MS = 40;

// ===================== GRAPH / PATHFINDING =====================
const int NUM_NODES = 7;

// Movement: 1=forward, -1=180, 2=left, 3=right
int antiClockwiseMatrix[NUM_NODES][NUM_NODES] = {
  {0,  0,  0,  0, -1,  1,  0},  // 0
  {0,  0,  0,  0,  0,  1,  1},  // 1  <-- FIX: 1->5 is STRAIGHT (was -1 causing spinning)
  {0,  0,  0,  1,  0, -1,  0},  // 2
  {0,  0, -1,  0,  0,  0,  1},  // 3
  {1,  0,  0,  0,  0,  0, -1},  // 4
  {-1, 2,  1,  0,  0,  0,  0},  // 5
  {0,  2,  0, -1,  1,  0,  0}   // 6
};

int clockwiseMatrix[NUM_NODES][NUM_NODES] = {
  {0,  0,  0,  0,  1, -1,  0},  // 0
  {0,  0,  0,  0,  0,  1, -1},  // 1
  {0,  0,  0,  1,  0, -1,  0},  // 2
  {0,  0,  1,  0,  0,  0, -1},  // 3
  {-1, 0,  0,  0,  0,  0,  1},  // 4
  {1,  3, -1,  0,  0,  0,  0},  // 5
  {0,  3,  0,  1, -1,  0,  0}   // 6
};

int path[10];
int pathStep   = 0;
int pathLength = 0;

// ===================== NAV STATE =====================
int  currentNode = -1;
int  currentGoal = -1;
bool clockwise    = false;
bool atNode       = false;
bool navigating   = false;
bool pathComputed = false;
bool startingUp   = true;

bool finishedAll     = false;
bool reachedGoalFlag = false;

// Node latch (re-arm when leaving node OR short timeout)
bool nodeArmed = true;
unsigned long lastNodeTime = 0;

// ===================== PARKING STATE MACHINE =====================
// Parking is triggered when server says "5":
// navigate to node 5, then drive straight using ultrasonic until close to wall.
// If it detects lots of white during approach, it backs away and stops.
enum ParkingStage : uint8_t {
  PARK_NONE = 0,
  PARK_TO_5,
  PARK_APPROACH_WALL,
  PARK_STOPPED
};
ParkingStage parkingStage = PARK_NONE;

float minApproachCm = 9999.0f;
uint32_t lastUSSample = 0;

// ===================== PROTOTYPES =====================
void ReadSensors();
bool isNodeNow();
bool detectNodeStable();
bool hasLeftNodeArea();

void SetMotors(int leftSpeed, int rightSpeed);
void motorStop();

void turn180();
void turn90Left();
void turn90Right();

void lineFollowing();

bool dijkstraPathfinding(int startIdx, int endIdx);
void navigateToTarget(int targetNode);
void executeMovement(int movementType);

int movementForEdge(int from, int to);

void handleStartup();
void handleNodeArrival();
void printCurrentStatus();

float ultrasonicReadCm();
void startParkingTo5();
void parkingApproachWallTick();
void backAwayFromWhiteAndStop();

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);

  for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);

  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  connectToWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    ESP.restart();
  }

  Serial.println("Robot Navigation + Server + Parking@5 + Ultrasonic wall stop");
  Serial.println("================================");
}

// ===================== LOOP =====================
void loop() {
  if (finishedAll) {
    motorStop();
    delay(10);
    return;
  }

  if (parkingStage == PARK_APPROACH_WALL) {
    parkingApproachWallTick();
    delay(5);
    return;
  }

  if (parkingStage == PARK_STOPPED) {
    motorStop();
    delay(10);
    return;
  }

  // Re-arm node detection
  if (!nodeArmed) {
    if (hasLeftNodeArea() || (millis() - lastNodeTime > 250)) {
      nodeArmed = true;
    }
  }

  if (startingUp) {
    handleStartup();
  } else if (navigating) {
    if (!atNode) {
      lineFollowing();
      if (nodeArmed && detectNodeStable()) {
        handleNodeArrival();
      }
    }
  } else {
    motorStop();
  }

  // Server logic: ONLY when reachedGoalFlag and NOT in ultrasonic approach
  if (!startingUp && !navigating && reachedGoalFlag && parkingStage != PARK_APPROACH_WALL && parkingStage != PARK_STOPPED) {
    reachedGoalFlag = false;

    String resp = apiPostArrived(currentNode);
    if (resp.length() == 0) {
      finishedAll = true;
      motorStop();
    } else if (isFinishedMessage(resp)) {
      Serial.println("Server said: Finished");
      finishedAll = true;
      motorStop();
    } else {
      int nextNode = resp.toInt();

      if (nextNode == 5) {
        startParkingTo5();
      } else {
        parkingStage = PARK_NONE;
        currentGoal = nextNode;
        navigateToTarget(currentGoal);
      }
    }
  }

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 2000) {
    printCurrentStatus();
    lastPrintTime = millis();
  }

  delay(10);
}

// ===================== STARTUP =====================
void handleStartup() {
  lineFollowing();

  if (nodeArmed && detectNodeStable()) {
    motorStop();
    delay(200);

    currentNode = 0;
    startingUp = false;

    reachedGoalFlag = true;

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;

    Serial.println("STARTUP: Node detected -> currentNode=0, requesting server.");
  }
}

// ===================== NODE ARRIVAL =====================
void handleNodeArrival() {
  atNode = true;
  motorStop();
  delay(200);

  if (!pathComputed || !navigating) {
    atNode = false;
    return;
  }

  if (pathStep >= pathLength) {
    navigating = false;
    pathComputed = false;
    atNode = false;
    return;
  }

  // Node correction: lock to expected node in path
  currentNode = path[pathStep];
  Serial.print("Node arrival -> currentNode corrected to expected: ");
  Serial.println(currentNode);

  // Goal reached
  if (currentNode == currentGoal) {
    navigating = false;
    pathComputed = false;

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;

    // If we parked at 5, begin ultrasonic approach (straight)
    if (parkingStage == PARK_TO_5 && currentGoal == 5) {
      Serial.println("Parking: reached node 5 -> ultrasonic straight approach");
      motorStop();
      delay(PARK_WAIT_MS);

      minApproachCm = 9999.0f;
      lastUSSample = 0;
      parkingStage = PARK_APPROACH_WALL;
      return;
    }

    // Normal mode: notify server
    reachedGoalFlag = true;
    return;
  }

  // Continue path
  if (pathStep >= pathLength - 1) {
    navigating = false;
    pathComputed = false;
    atNode = false;
    return;
  }

  int from = path[pathStep];
  int to   = path[pathStep + 1];
  int movement = movementForEdge(from, to);

  pathStep++;
  executeMovement(movement);

  nodeArmed = false;
  lastNodeTime = millis();
  atNode = false;
}

// ===================== PARKING TRIGGER =====================
void startParkingTo5() {
  Serial.println("Server said 5 -> navigate to node 5, then ultrasonic stop.");

  parkingStage = PARK_TO_5;
  currentGoal = 5;

  navigateToTarget(currentGoal);

  nodeArmed = false;
  lastNodeTime = millis();
  atNode = false;
}

// ===================== ULTRASONIC APPROACH (AFTER STRAIGHT) =====================
void parkingApproachWallTick() {
  // Drive straight
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);

  // If it detects a lot of white (node/white patch), move away and stop.
  if (isNodeNow()) {
    backAwayFromWhiteAndStop();
    return;
  }

  uint32_t now = millis();
  if (now - lastUSSample < US_SAMPLE_MS) return;
  lastUSSample = now;

  float cm = ultrasonicReadCm();
  if (cm <= 0.0f) return;

  if (cm < minApproachCm) {
    minApproachCm = cm;
    Serial.print("Approach cm (new min): ");
    Serial.println(minApproachCm, 2);
  }

  if (minApproachCm <= PARK_STOP_CM) {
    Serial.println("Ultrasonic: stop distance reached -> STOPPED");
    motorStop();
    parkingStage = PARK_STOPPED;
  }
}

void backAwayFromWhiteAndStop() {
  Serial.println("White detected during approach -> backing away and STOP");

  // Back away briefly
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(220);

  motorStop();
  parkingStage = PARK_STOPPED;
}

float ultrasonicReadCm() {
  digitalWrite(US_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(US_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(US_TRIG_PIN, LOW);

  uint32_t duration = pulseIn(US_ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return 0.0f;

  return (float)duration / 58.0f;
}

// ===================== PATHFINDING =====================
int movementForEdge(int from, int to) {
  return clockwise ? clockwiseMatrix[from][to] : antiClockwiseMatrix[from][to];
}

bool dijkstraPathfinding(int startIdx, int endIdx) {
  pathLength = 0;
  pathStep = 0;

  int dist[NUM_NODES];
  int prev[NUM_NODES];
  bool visited[NUM_NODES];

  for (int i = 0; i < NUM_NODES; i++) {
    dist[i] = 9999;
    prev[i] = -1;
    visited[i] = false;
  }
  dist[startIdx] = 0;

  for (int count = 0; count < NUM_NODES; count++) {
    int minDist = 9999;
    int u = -1;

    for (int v = 0; v < NUM_NODES; v++) {
      if (!visited[v] && dist[v] < minDist) {
        minDist = dist[v];
        u = v;
      }
    }
    if (u == -1 || u == endIdx) break;
    visited[u] = true;

    for (int v = 0; v < NUM_NODES; v++) {
      bool edgeExists = (antiClockwiseMatrix[u][v] != 0) || (clockwiseMatrix[u][v] != 0);
      if (edgeExists) {
        int newDist = dist[u] + 1;
        if (newDist < dist[v]) {
          dist[v] = newDist;
          prev[v] = u;
        }
      }
    }
  }

  if (dist[endIdx] == 9999) return false;

  int cur = endIdx;
  while (cur != -1) {
    path[pathLength++] = cur;
    cur = prev[cur];
  }

  for (int i = 0; i < pathLength / 2; i++) {
    int t = path[i];
    path[i] = path[pathLength - 1 - i];
    path[pathLength - 1 - i] = t;
  }

  return true;
}

void navigateToTarget(int targetNode) {
  if (currentNode < 0 || currentNode >= NUM_NODES) {
    Serial.println("ERROR: currentNode invalid, cannot navigate.");
    navigating = false;
    pathComputed = false;
    motorStop();
    return;
  }

  if (!dijkstraPathfinding(currentNode, targetNode)) {
    Serial.println("ERROR: No path found. Stopping.");
    navigating = false;
    pathComputed = false;
    motorStop();
    return;
  }

  pathComputed = true;
  navigating = true;

  if (pathLength > 1 && currentNode == path[0]) {
    int firstMove = movementForEdge(path[0], path[1]);
    pathStep = 1;
    atNode = false;

    executeMovement(firstMove);

    nodeArmed = false;
    lastNodeTime = millis();
  } else {
    pathStep = 0;
  }
}

// ===================== SENSORS / NODE DETECTION =====================
void ReadSensors() {
  for (int i = 0; i < 5; i++) AnalogValue[i] = analogRead(AnalogPin[i]);
}

bool isNodeNow() {
  ReadSensors();
  int whiteCount = 0;
  for (int i = 0; i < 5; i++) {
    if (AnalogValue[i] < threshold) whiteCount++;
  }
  return (whiteCount >= 4);
}

bool detectNodeStable() {
  int hits = 0;
  const int samples = 6;

  for (int k = 0; k < samples; k++) {
    if (isNodeNow()) hits++;
    delay(5);
  }
  return (hits >= 4);
}

bool hasLeftNodeArea() {
  int notNodeHits = 0;
  const int samples = 10;

  for (int k = 0; k < samples; k++) {
    if (!isNodeNow()) notNodeHits++;
    delay(5);
  }
  return (notNodeHits >= 7);
}

// ===================== MOTOR CONTROL =====================
void SetMotors(int leftSpeed, int rightSpeed) {
  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(motor1PWM, leftSpeed);
  analogWrite(motor2PWM, rightSpeed);

  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
}

void motorStop() {
  analogWrite(motor1PWM, 0);
  analogWrite(motor2PWM, 0);
}

// ===================== TURNS =====================
void turn180() {
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(700);
  motorStop();
}

void turn90Left() {
  // forward nudge
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN_PRE_FORWARD_MS);
  motorStop();
  delay(TURN_PRE_PAUSE_MS);

  // pivot left
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN90_MS);
  motorStop();
}

void turn90Right() {
  // forward nudge
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN_PRE_FORWARD_MS);
  motorStop();
  delay(TURN_PRE_PAUSE_MS);

  // pivot right
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN90_MS);
  motorStop();
}

// ===================== MOVEMENT EXECUTION =====================
void executeMovement(int movementType) {
  motorStop();
  delay(120);

  switch (movementType) {
    case 1: // forward
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(320);
      motorStop();
      delay(60);
      break;

    case -1: // 180° and flip matrix
      turn180();
      clockwise = !clockwise;

      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(320);
      motorStop();
      delay(60);
      break;

    case 2: // left
      turn90Left();
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(320);
      motorStop();
      delay(60);
      break;

    case 3: // right
      turn90Right();
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(320);
      motorStop();
      delay(60);
      break;

    default:
      motorStop();
      delay(300);
      break;
  }

  nodeArmed = false;
  lastNodeTime = millis();
}

// ===================== LINE FOLLOWING =====================
void lineFollowing() {
  ReadSensors();

  int minIndex = 0;
  int minValue = AnalogValue[0];

  for (int i = 1; i < 5; i++) {
    if (AnalogValue[i] < minValue) {
      minValue = AnalogValue[i];
      minIndex = i;
    }
  }

  if (minIndex == 0) {
    SetMotors(220, 40);
    lastError = -3;
    return;
  }
  if (minIndex == 4) {
    SetMotors(40, 220);
    lastError = 3;
    return;
  }

  float error = minIndex - 2;
  float correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  SetMotors(leftSpeed, rightSpeed);
}

// ===================== STATUS =====================
void printCurrentStatus() {
  Serial.println("\n=== CURRENT STATUS ===");
  Serial.print("currentNode: "); Serial.println(currentNode);
  Serial.print("currentGoal: "); Serial.println(currentGoal);
  Serial.print("parkingStage: "); Serial.println((int)parkingStage);

  Serial.print("state: ");
  if (startingUp) Serial.println("STARTUP");
  else if (navigating) Serial.println("NAVIGATING");
  else Serial.println("STOPPED");

  Serial.print("pathStep: "); Serial.println(pathStep);
  Serial.print("pathLength: "); Serial.println(pathLength);

  if (pathComputed && pathStep < pathLength) {
    Serial.print("nextExpectedNode: ");
    Serial.println(path[pathStep]);
  } else {
    Serial.println("nextExpectedNode: (none)");
  }

  Serial.print("nodeArmed: "); Serial.println(nodeArmed ? "YES" : "NO");
  Serial.print("matrix: "); Serial.println(clockwise ? "CLOCKWISE" : "ANTI-CLOCKWISE");

  if (parkingStage == PARK_APPROACH_WALL || parkingStage == PARK_STOPPED) {
    Serial.print("minApproachCm: ");
    Serial.println(minApproachCm, 2);
  }

  Serial.println("======================");
}
