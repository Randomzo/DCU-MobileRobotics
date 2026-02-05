#include <Arduino.h>
#include <WiFi.h>

// ===================== WIFI / SERVER =====================
const char* ssid     = "inteernet name here";
const char* password = "password here";

static const char* apiHost = "3.250.38.184";
static const int   apiPort = 8000;
static const char* TEAM_ID = "team name here";

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

// ===================== PATHFINDING / DRIVE =====================

// Sensors
int AnalogValue[5] = {0, 0, 0, 0, 0};
int AnalogPin[5]   = {4, 5, 6, 7, 15};
int threshold      = 1000;

// Motors
int motor1PWM   = 37;
int motor1Phase = 38;
int motor2PWM   = 39;
int motor2Phase = 20;

// Line following
int   baseSpeed = 200;
float Kp = 190.0;
float Kd = 45.0;
float lastError = 0;

// Turn tuning (YOUR improved movement constants)
static const int TURN_PRE_FORWARD_MS = 70;  // tiny forward nudge
static const int TURN_PRE_PAUSE_MS   = 40;  // settle before pivot
static const int TURN90_MS           = 375; // your original value

// Graph
const int NUM_NODES = 7;

int antiClockwiseMatrix[NUM_NODES][NUM_NODES] = {
  {0,  0,  0,  0, -1,  1,  0},
  {0,  0,  0,  0,  0,  1, -1},
  {0,  0,  0,  1,  0, -1,  0},
  {0,  0, -1,  0,  0,  0,  1},
  {1,  0,  0,  0,  0,  0, -1},
  {-1, 2,  1,  0,  0,  0,  0},
  {0,  2,  0, -1,  1,  0,  0}
};

int clockwiseMatrix[NUM_NODES][NUM_NODES] = {
  {0,  0,  0,  0,  1, -1,  0},
  {0,  0,  0,  0,  0, -1,  1},
  {0,  0,  0,  1,  0, -1,  0},
  {0,  0,  1,  0,  0,  0, -1},
  {-1, 0,  0,  0,  0,  0,  1},
  {1,  3, -1,  0,  0,  0,  0},
  {0,  3,  0,  1, -1,  0,  0}
};

int path[10];
int pathStep = 0;
int pathLength = 0;

bool clockwise = false;
bool atNode = false;
bool navigating = false;
bool pathComputed = false;
bool startingUp = true;

bool nodeArmed = true;
unsigned long lastNodeTime = 0;

int currentNode = -1;
int targetNode = -1;
bool reachedTargetFlag = false;

bool finishedAll = false;

// ========== SENSOR / NODE DETECTION ==========
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

  bool nodeDetected = (hits >= 4);

  if (nodeDetected) {
    Serial.print("NODE DETECT. hits=");
    Serial.print(hits);
    Serial.print(" sensors: ");
    ReadSensors();
    for (int i = 0; i < 5; i++) {
      Serial.print(AnalogValue[i]);
      Serial.print(" ");
    }
    Serial.println();
  }

  return nodeDetected;
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

// ========== MOTOR CONTROL ==========
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

void turn180() {
  Serial.println("Turning 180°...");
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(700);
  motorStop();
}

// ========== 90° TURNS (IMPROVED: tiny forward nudge before turn) ==========
void turn90Left() {
  Serial.println("Turning 90° left...");

  // 1) tiny forward nudge
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN_PRE_FORWARD_MS);
  motorStop();
  delay(TURN_PRE_PAUSE_MS);

  // 2) pivot left
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, LOW);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN90_MS);
  motorStop();
}

void turn90Right() {
  Serial.println("Turning 90° right...");

  // 1) tiny forward nudge
  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN_PRE_FORWARD_MS);
  motorStop();
  delay(TURN_PRE_PAUSE_MS);

  // 2) pivot right
  digitalWrite(motor1Phase, LOW);
  digitalWrite(motor2Phase, HIGH);
  analogWrite(motor1PWM, baseSpeed);
  analogWrite(motor2PWM, baseSpeed);
  delay(TURN90_MS);
  motorStop();
}

int movementForEdge(int from, int to) {
  return clockwise ? clockwiseMatrix[from][to] : antiClockwiseMatrix[from][to];
}

// ========== MOVEMENT EXECUTION (WITH RE-ARM FIX) ==========
void executeMovement(int movementType) {
  motorStop();
  delay(200);

  switch (movementType) {
    case 1: // forward
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(500);
      motorStop();
      delay(150);
      break;

    case -1: // 180° and flip matrix
      turn180();
      clockwise = !clockwise;
      Serial.print("Matrix is now: ");
      Serial.println(clockwise ? "CLOCKWISE" : "ANTI-CLOCKWISE");

      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(500);
      motorStop();
      delay(150);
      break;

    case 2:
      turn90Left(); // includes nudge
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(500);
      motorStop();
      delay(150);
      break;

    case 3:
      turn90Right(); // includes nudge
      digitalWrite(motor1Phase, HIGH);
      digitalWrite(motor2Phase, HIGH);
      analogWrite(motor1PWM, baseSpeed);
      analogWrite(motor2PWM, baseSpeed);
      delay(500);
      motorStop();
      delay(150);
      break;

    default:
      Serial.println("ERROR: movementType=0 (no edge). Stopping.");
      motorStop();
      delay(500);
      break;
  }

  // CRITICAL FIX: disarm node detection until we leave node area (not a long time gate)
  nodeArmed = false;
  lastNodeTime = millis();
}

// ========== LINE FOLLOWING ==========
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

  if (minIndex == 0) { SetMotors(220, 40); lastError = -3; return; }
  if (minIndex == 4) { SetMotors(40, 220); lastError =  3; return; }

  float error = minIndex - 2;
  float correction = Kp * error + Kd * (error - lastError);
  lastError = error;

  int leftSpeed  = baseSpeed - correction;
  int rightSpeed = baseSpeed + correction;

  leftSpeed  = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  SetMotors(leftSpeed, rightSpeed);
}

// ========== PATHFINDING ==========
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

void navigateToTarget(int newTarget) {
  targetNode = newTarget;
  reachedTargetFlag = false;

  if (!dijkstraPathfinding(currentNode, targetNode)) {
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
  } else {
    pathStep = 0;
  }
}

// ========== STATUS PRINTING ==========
void printCurrentStatus() {
  Serial.println("\n=== CURRENT STATUS ===");
  Serial.print("Current Node (logical): ");
  Serial.println(currentNode);
  Serial.print("Target Node: ");
  Serial.println(targetNode);

  Serial.print("State: ");
  if (startingUp) Serial.println("STARTUP");
  else if (navigating) Serial.println("NAVIGATING");
  else Serial.println("STOPPED");

  Serial.print("pathStep (next expected node idx): ");
  Serial.println(pathStep);
  Serial.print("pathLength: ");
  Serial.println(pathLength);

  if (pathComputed && pathStep < pathLength) {
    Serial.print("Next expected node: ");
    Serial.println(path[pathStep]);
  } else if (pathComputed) {
    Serial.println("Next expected node: (none)");
  }

  Serial.print("Node armed: ");
  Serial.println(nodeArmed ? "YES" : "NO");

  Serial.print("Matrix: ");
  Serial.println(clockwise ? "CLOCKWISE" : "ANTI-CLOCKWISE");
  Serial.println("====================\n");
}

// ========== STARTUP / NODE HANDLING ==========
void handleStartup() {
  // Re-arm based on leaving node area (plus small safety timeout)
  if (!nodeArmed) {
    if (hasLeftNodeArea() || (millis() - lastNodeTime > 250)) nodeArmed = true;
  }

  lineFollowing();

  if (nodeArmed && detectNodeStable()) {
    motorStop();
    delay(200);

    // Start at node 0
    currentNode = 0;

    // Immediately ask server for the first next node
    reachedTargetFlag = true;
    targetNode = 0;
    navigating = false;
    pathComputed = false;

    startingUp = false;

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;

    printCurrentStatus();
  }
}

void handleNodeArrival() {
  atNode = true;
  motorStop();
  delay(250);

  if (!pathComputed || !navigating) {
    atNode = false;
    return;
  }

  if (pathStep >= pathLength) {
    navigating = false;
    pathComputed = false;
    reachedTargetFlag = true;
    atNode = false;
    return;
  }

  currentNode = path[pathStep];

  // ONLY notify server when we hit the TARGET node
  if (currentNode == targetNode) {
    navigating = false;
    pathComputed = false;
    reachedTargetFlag = true;

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;

    printCurrentStatus();
    return;
  }

  if (pathStep >= pathLength - 1) {
    navigating = false;
    pathComputed = false;
    reachedTargetFlag = true;
    atNode = false;
    return;
  }

  int from = path[pathStep];
  int to   = path[pathStep + 1];
  int movement = movementForEdge(from, to);

  pathStep++;
  executeMovement(movement);

  atNode = false;

  printCurrentStatus();
}

void startNextLegTo(int nextNode) {
  if (nextNode < 0 || nextNode >= NUM_NODES) {
    finishedAll = true;
    motorStop();
    return;
  }

  if (nextNode == currentNode) {
    // Already there: ask server again
    reachedTargetFlag = true;
    targetNode = currentNode;
    navigating = false;
    pathComputed = false;
    return;
  }

  navigateToTarget(nextNode);
}

// ===================== SETUP / LOOP =====================
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

  connectToWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    delay(3000);
    ESP.restart();
  }
}

void loop() {
  if (startingUp) {
    handleStartup();
  }

  if (!startingUp && navigating) {
    // Re-arm based on leaving node area (plus small safety timeout)
    if (!nodeArmed) {
      if (hasLeftNodeArea() || (millis() - lastNodeTime > 250)) nodeArmed = true;
    }

    if (!atNode) {
      lineFollowing();
      if (nodeArmed && detectNodeStable()) {
        handleNodeArrival();
      }
    }
  }

  // ONLY talk to server when we reached the TARGET (or node 0 at startup)
  if (!startingUp && !navigating && reachedTargetFlag && !finishedAll) {
    reachedTargetFlag = false;

    String serverResp = apiPostArrived(currentNode);
    if (serverResp.length() == 0) {
      finishedAll = true;
      motorStop();
    } else if (isFinishedMessage(serverResp)) {
      Serial.println("Server said: Finished");
      finishedAll = true;
      motorStop();
    } else {
      int nextNode = serverResp.toInt();
      startNextLegTo(nextNode);
    }
  }

  if (finishedAll) motorStop();
  delay(10);
}
