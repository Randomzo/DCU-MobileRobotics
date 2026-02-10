#include <Arduino.h>

// ========== ROBOT HARDWARE CONFIGURATION ==========
// ---------- Sensors ----------
int AnalogValue[5] = {0,0,0,0,0};
int AnalogPin[5]   = {4,5,6,7,15};
int threshold      = 1000;  // keep same threshold

// ---------- Motors ----------
int motor1PWM   = 37;
int motor1Phase = 38;
int motor2PWM   = 39;
int motor2Phase = 20;

// ---------- Line Following Tuning ----------
int baseSpeed   = 200;     // 0-255
float Kp        = 190.0;
float Kd        = 45.0;
float lastError = 0;

// ---------- Turn tuning (NEW) ----------
const int TURN_PRE_FORWARD_MS = 120;  // tiny forward nudge before 90° turn
const int TURN_PRE_PAUSE_MS   = 50;   // short pause between nudge and turn
const int TURN90_MS           = 375;  // your existing 90° timing

// ========== PATHFINDING CONFIGURATION ==========
const int START_NODE  = 2;
const int TARGET_NODE = 6;

// ========== NAVIGATION VARIABLES ==========
int currentNode = START_NODE;

bool clockwise    = false;   // false = anti-clockwise
bool atNode       = false;
bool navigating   = false;
bool pathComputed = false;
bool startingUp   = true;

// Node detection latch (debounce)
bool nodeArmed = true;
unsigned long lastNodeTime = 0;

int pathStep   = 0;          // index of NEXT expected node in path[]
int pathLength = 0;

// ========== GRAPH DATA STRUCTURES ==========
const int NUM_NODES = 7;

// Movement: 1=forward, -1=180, 2=left, 3=right
int antiClockwiseMatrix[NUM_NODES][NUM_NODES] = {
    {0,  0,  0,  0, -1,  1,  0},  // 0
    {0,  0,  0,  0,  0,  1, -1},  // 1
    {0,  0,  0,  1,  0, -1,  0},  // 2
    {0,  0, -1,  0,  0,  0,  1},  // 3
    {1,  0,  0,  0,  0,  0, -1},  // 4
    {-1, 2,  1,  0,  0,  0,  0},  // 5
    {0,  2,  0, -1,  1,  0,  0}   // 6
};

int clockwiseMatrix[NUM_NODES][NUM_NODES] = {
    {0,  0,  0,  0,  1, -1,  0},  // 0
    {0,  0,  0,  0,  0, -1,  1},  // 1
    {0,  0,  0,  1,  0, -1,  0},  // 2
    {0,  0,  1,  0,  0,  0, -1},  // 3
    {-1, 0,  0,  0,  0,  0,  1},  // 4
    {1,  3, -1,  0,  0,  0,  0},  // 5
    {0,  3,  0,  1, -1,  0,  0}   // 6
};

// Path storage (nodes only)
int path[10];

// ========== FUNCTION PROTOTYPES ==========
void ReadSensors();
void SetMotors(int leftSpeed, int rightSpeed);
void motorStop();
void turn180();
void turn90Left();
void turn90Right();
void lineFollowing();

bool dijkstraPathfinding(int startIdx, int endIdx);
void navigateToTarget(int targetNode);

void executeMovement(int movementType);

// Node detection (more sensitive)
bool isNodeNow();            // 4/5 white
bool detectNodeStable();     // stable detection using isNodeNow()
bool hasLeftNodeArea();      // re-arm based on !isNodeNow()

int movementForEdge(int from, int to);

void handleStartup();
void handleNodeArrival();
void printCurrentStatus();

// ========== SETUP ==========
void setup() {
  Serial.begin(9600);

  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1Phase, OUTPUT);
  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2Phase, OUTPUT);

  for (int i = 0; i < 5; i++) pinMode(AnalogPin[i], INPUT);

  digitalWrite(motor1Phase, HIGH);
  digitalWrite(motor2Phase, HIGH);

  delay(1000);

  Serial.println("Robot Navigation System Started");
  Serial.print("Start Node: ");  Serial.println(START_NODE);
  Serial.print("Target Node: "); Serial.println(TARGET_NODE);
  Serial.println("================================");
}

// ========== MAIN LOOP ==========
void loop() {
  if (startingUp) {
    handleStartup();
  }
  else if (navigating) {

    // Re-arm node detection: either we left the node area OR timeout
    if (!nodeArmed) {
      if (hasLeftNodeArea() || (millis() - lastNodeTime > 700)) {
        nodeArmed = true;
      }
    }

    if (!atNode) {
      lineFollowing();

      if (nodeArmed && detectNodeStable()) {
        handleNodeArrival();
      }
    }
  }
  else {
    motorStop();
  }

  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime > 2000) {
    printCurrentStatus();
    lastPrintTime = millis();
  }

  delay(10);
}

// ========== HELPER: MOVEMENT FOR EDGE (LIVE) ==========
int movementForEdge(int from, int to) {
  return clockwise ? clockwiseMatrix[from][to] : antiClockwiseMatrix[from][to];
}

// ========== STARTUP ==========
void handleStartup() {
  if (START_NODE >= 0 && START_NODE < NUM_NODES) {
    Serial.println("STARTUP: Starting at a known node. Beginning navigation immediately.");
    startingUp = false;
    navigateToTarget(TARGET_NODE);

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;
    return;
  }

  if (!nodeArmed) {
    if (hasLeftNodeArea() || (millis() - lastNodeTime > 700)) {
      nodeArmed = true;
    }
  }

  lineFollowing();

  if (nodeArmed && detectNodeStable()) {
    Serial.println("STARTUP: Node detected (stable). Transitioning to navigation.");
    motorStop();
    delay(200);

    if (START_NODE == -1 && currentNode == START_NODE) {
      currentNode = 0;
      Serial.print("Startup: Setting currentNode = ");
      Serial.println(currentNode);
    }

    startingUp = false;
    Serial.println("Startup complete. Beginning navigation.");

    navigateToTarget(TARGET_NODE);

    nodeArmed = false;
    lastNodeTime = millis();
    atNode = false;
  }
}

// ========== NODE ARRIVAL ==========
void handleNodeArrival() {
  atNode = true;
  motorStop();
  delay(250);

  if (!pathComputed || !navigating) {
    Serial.println("Node arrival but no active navigation. Stopping.");
    navigating = false;
    motorStop();
    while(true) delay(1000);
  }

  if (pathStep >= pathLength) {
    Serial.println("PathStep past end of path. Stopping.");
    navigating = false;
    pathComputed = false;
    motorStop();
    while(true) delay(1000);
  }

  currentNode = path[pathStep];

  Serial.print("Node detected. Assigning currentNode = ");
  Serial.println(currentNode);

  if (currentNode == TARGET_NODE) {
    Serial.print("Reached TARGET node ");
    Serial.println(TARGET_NODE);
    navigating = false;
    pathComputed = false;
    motorStop();
    while(true) delay(1000);
  }

  // SPECIAL CASE: 1 -> (5 or 6)
  if (pathStep > 0) {
    int prevNode = path[pathStep - 1];
    if (prevNode == 1 && (currentNode == 5 || currentNode == 6)) {
      Serial.println("Special case: 1 -> (5 or 6). Mandatory 90° LEFT + force ANTI-CLOCKWISE.");
      motorStop();
      delay(200);
      turn90Left();          // now includes tiny forward nudge (see function below)
      clockwise = false;
      Serial.println("Matrix forced to: ANTI-CLOCKWISE");
      delay(200);
    }
  }

  if (pathStep >= pathLength - 1) {
    Serial.println("End of path reached. Stopping.");
    navigating = false;
    pathComputed = false;
    motorStop();
    while(true) delay(1000);
  }

  int from = path[pathStep];
  int to   = path[pathStep + 1];

  int movement = movementForEdge(from, to);

  Serial.print("Next edge: ");
  Serial.print(from);
  Serial.print(" -> ");
  Serial.print(to);
  Serial.print(" | movement (live) = ");
  Serial.println(movement);

  pathStep++;

  executeMovement(movement);

  nodeArmed = false;
  lastNodeTime = millis();
  atNode = false;
}

// ========== DIJKSTRA (PATH NODES ONLY) ==========
bool dijkstraPathfinding(int startIdx, int endIdx) {
  Serial.print("Computing path from node ");
  Serial.print(startIdx);
  Serial.print(" to node ");
  Serial.println(endIdx);

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

  if (dist[endIdx] == 9999) {
    Serial.println("ERROR: No path found!");
    return false;
  }

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

  Serial.print("Computed path: ");
  for (int i = 0; i < pathLength; i++) {
    Serial.print(path[i]);
    if (i < pathLength - 1) Serial.print(" -> ");
  }
  Serial.println();

  return true;
}

// ========== NAVIGATION ==========
void navigateToTarget(int targetNode) {
  Serial.println("Starting navigation to target...");

  if (!dijkstraPathfinding(currentNode, targetNode)) {
    Serial.println("ERROR: Failed to compute path. Stopping.");
    navigating = false;
    motorStop();
    return;
  }

  pathComputed = true;
  navigating = true;

  if (pathLength > 1 && currentNode == path[0]) {
    int firstMove = movementForEdge(path[0], path[1]);

    Serial.print("At start node ");
    Serial.print(currentNode);
    Serial.print(". First edge ");
    Serial.print(path[0]);
    Serial.print(" -> ");
    Serial.print(path[1]);
    Serial.print(" | movement (live) = ");
    Serial.println(firstMove);

    pathStep = 1;
    atNode = false;

    executeMovement(firstMove);

    nodeArmed = false;
    lastNodeTime = millis();
  } else {
    pathStep = 0;
  }

  Serial.print("Initial pathStep (next expected node idx) = ");
  Serial.println(pathStep);
}

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

// ========== 90° TURNS (UPDATED: tiny forward nudge before turn) ==========
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

// ========== MOVEMENT EXECUTION ==========
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

// ========== STATUS PRINTING ==========
void printCurrentStatus() {
  Serial.println("\n=== CURRENT STATUS ===");
  Serial.print("Current Node (logical): ");
  Serial.println(currentNode);
  Serial.print("Target Node: ");
  Serial.println(TARGET_NODE);

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
