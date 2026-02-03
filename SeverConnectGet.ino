#include <WiFi.h>

const char* ssid = "internet name ";
const char* password = "password";

// API server details
static const char* apiHost = "3.250.38.184";
static const int apiPort = 8000;

// TEAM ID
static const char* TEAM_ID = "team id";

WiFiClient client;

// --------------------------- WiFi ---------------------------
void connectToWiFi(uint32_t timeoutMs = 20000) {
  Serial.println();
  Serial.print("Connecting to network: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.disconnect(true, true);
  delay(200);

  WiFi.begin(ssid, password);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection FAILED.");
    Serial.print("Status code: ");
    Serial.println((int)WiFi.status());
    return;
  }

  Serial.println("WiFi connected.");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

// --------------------------- HTTP helpers ---------------------------
static const uint32_t HTTP_TIMEOUT_MS = 10000; // Increased timeout

bool httpConnect() {
  if (client.connected()) {
    client.stop(); // Ensure clean connection
    delay(50);
  }
  
  Serial.print("Connecting to API server ");
  Serial.print(apiHost);
  Serial.print(":");
  Serial.println(apiPort);
  
  if (client.connect(apiHost, apiPort)) {
    Serial.println("Connected to server");
    return true;
  } else {
    Serial.println("Connection to server failed");
    return false;
  }
}

void httpStop() {
  if (client.connected()) {
    client.stop();
    delay(50);
  }
}

// Read full HTTP response
String readHTTPResponse() {
  String response = "";
  uint32_t startTime = millis();
  
  while (millis() - startTime < HTTP_TIMEOUT_MS) {
    while (client.available()) {
      char c = client.read();
      response += c;
    }
    
    // Check if response is complete (ends with \r\n\r\n or connection closed)
    if (response.indexOf("\r\n\r\n") != -1 && !client.connected()) {
      break;
    }
    
    delay(10);
  }
  
  return response;
}

// Extract body from HTTP response
String extractBody(const String& response) {
  int bodyStart = response.indexOf("\r\n\r\n");
  if (bodyStart == -1) {
    return response; // Might be plain text without headers
  }
  
  String body = response.substring(bodyStart + 4);
  body.trim();
  return body;
}

// --------------------------- API calls ---------------------------
// GET /api/getRoute/TEAM_ID
String apiGetRoute() {
  if (!httpConnect()) {
    Serial.println("Failed to connect for GET route");
    return "";
  }

  String request = String("GET /api/getRoute/") + TEAM_ID + " HTTP/1.1\r\n";
  request += "Host: " + String(apiHost) + "\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";

  Serial.println("Sending GET route request:");
  Serial.println(request);

  client.print(request);
  client.flush();

  String response = readHTTPResponse();
  httpStop();

  Serial.println("Raw response:");
  Serial.println(response);
  
  String body = extractBody(response);
  Serial.print("Route from server: ");
  Serial.println(body);

  return body;
}

// POST /api/arrived/TEAM_ID
String apiPostArrived(int position) {
  if (!httpConnect()) {
    Serial.println("Failed to connect for POST arrived");
    return "";
  }

  String postData = "position=" + String(position);
  
  String request = String("POST /api/arrived/") + TEAM_ID + " HTTP/1.1\r\n";
  request += "Host: " + String(apiHost) + "\r\n";
  request += "Content-Type: application/x-www-form-urlencoded\r\n";
  request += "Content-Length: " + String(postData.length()) + "\r\n";
  request += "Connection: close\r\n";
  request += "\r\n";
  request += postData;

  Serial.println("Sending POST arrived request:");
  Serial.println(request);

  client.print(request);
  client.flush();

  String response = readHTTPResponse();
  httpStop();

  Serial.println("Raw POST response:");
  Serial.println(response);
  
  String body = extractBody(response);
  Serial.print("Next destination from server: ");
  Serial.println(body);

  return body;
}

// Parse comma-separated route
int parseRoute(const String& routeStr, int* routeArray, int maxSize) {
  int count = 0;
  int startIdx = 0;
  int commaIdx = 0;
  
  while (commaIdx != -1 && count < maxSize) {
    commaIdx = routeStr.indexOf(',', startIdx);
    
    if (commaIdx == -1) {
      // Last number
      String numStr = routeStr.substring(startIdx);
      numStr.trim();
      if (numStr.length() > 0) {
        routeArray[count++] = numStr.toInt();
      }
    } else {
      String numStr = routeStr.substring(startIdx, commaIdx);
      numStr.trim();
      if (numStr.length() > 0) {
        routeArray[count++] = numStr.toInt();
      }
      startIdx = commaIdx + 1;
    }
  }
  
  return count;
}

// --------------------------- Robot motion ---------------------------
void moveRobotTo(int targetPosition) {
  Serial.print("MOVING to position: ");
  Serial.println(targetPosition);
  
  // TODO: Replace with your actual robot movement code
  // This should control motors to navigate to the target position
  
  // Simulate movement time
  delay(1000);
  Serial.println("Arrived at position");
}

// --------------------------- Main ---------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== Robot Route Follower ===");
  
  // Connect to WiFi
  connectToWiFi();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to connect to WiFi. Restarting...");
    delay(5000);
    ESP.restart();
  }
  
  // Get the route from server
  String routeStr = apiGetRoute();
  if (routeStr.length() == 0) {
    Serial.println("ERROR: No route received from server");
    return;
  }
  
  // Parse the route
  const int MAX_ROUTE_LENGTH = 20;
  int route[MAX_ROUTE_LENGTH];
  int routeLength = parseRoute(routeStr, route, MAX_ROUTE_LENGTH);
  
  if (routeLength == 0) {
    Serial.println("ERROR: Could not parse route");
    return;
  }
  
  Serial.print("Parsed route (");
  Serial.print(routeLength);
  Serial.print(" positions): ");
  for (int i = 0; i < routeLength; i++) {
    Serial.print(route[i]);
    if (i < routeLength - 1) Serial.print(",");
  }
  Serial.println();
  
  // First position is where we start
  int currentPosition = route[0];
  Serial.print("Starting at position: ");
  Serial.println(currentPosition);
  
  // If you're physically starting at position 0, but the route says otherwise,
  // you might need to move to the first position first
  if (currentPosition != 0) {
    Serial.println("Note: Route doesn't start at 0. Adjust if needed.");
  }
  
  // Follow the route by posting arrivals to server
  for (int i = 0; i < routeLength; i++) {
    // For first position, we're already there
    if (i > 0) {
      // Move to the next position in our local route
      moveRobotTo(route[i]);
      currentPosition = route[i];
    }
    
    // Notify server we've arrived
    String serverResponse = apiPostArrived(currentPosition);
    
    // Check server response
    if (serverResponse.length() == 0) {
      Serial.println("ERROR: No response from server");
      break;
    }
    
    if (serverResponse.equalsIgnoreCase("Finished")) {
      Serial.println("Server says: Finished!");
      break;
    }
    
    // Server might give us the next destination
    int nextFromServer = serverResponse.toInt();
    Serial.print("Server next destination: ");
    Serial.println(nextFromServer);
    
    // Optional: Verify server's next matches our route
    if (i + 1 < routeLength && nextFromServer != route[i + 1]) {
      Serial.println("WARNING: Server's next destination doesn't match local route!");
      // You might want to use server's response as the truth
    }
    
    delay(500); // Brief delay between requests
  }
  
  Serial.println("\n=== Route Complete ===");
}

void loop() {
  // Done - just idle
  delay(1000);
}