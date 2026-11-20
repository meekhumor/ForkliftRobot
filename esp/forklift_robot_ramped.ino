/*
  FOUR-WHEEL NORMAL-WHEEL FORKLIFT ROBOT - ESP32 MOTOR CONTROLLER FIRMWARE
  Receives motor & lift control instructions from Jetson Dashboard over Wi-Fi
  + SMOOTH ACCELERATION / DECELERATION (slew-rate limited motors)
  + WATCHDOG FAILSAFE TIMEOUT (500 ms)

  Hardware mapping:
  - 4 DC Drive Motors (LF, RF, LB, RB) - Differential / Skid-steer
  - 1 DC Lift Motor (Raise / Lower forks)
  - Fixed standard wheels (NOT mecanum)
*/

#include <WiFi.h>
#include <WebServer.h>
#include <esp_arduino_version.h>

// ---------------- Wi-Fi: join phone hotspot ----------------
// Ensure your phone hotspot is enabled with 2.4 GHz compatible mode.
const char* WIFI_SSID     = "iPhone";
const char* WIFI_PASSWORD = "12345678";

// Backup ESP32 network if phone hotspot is unavailable
const char* BACKUP_AP_SSID = "Forklift-Robot-Backup";
const char* BACKUP_AP_PASS = "robot123";

WebServer server(80);

unsigned long lastWiFiRetry = 0;
bool wasWiFiConnected = false;
const unsigned long WIFI_RETRY_INTERVAL_MS = 10000;

// ---------------- Motor pins ----------------
const int LF_DIR = 32, LF_PWM = 33;
const int RF_DIR = 12, RF_PWM = 13;
const int LB_DIR = 18, LB_PWM = 19;
const int RB_DIR = 26, RB_PWM = 25;

const int LIFT_DIR = 23, LIFT_PWM = 27;

// Invert motor direction if needed during physical verification
int LF_SIGN = 1;
int RF_SIGN = 1;
int LB_SIGN = 1;
int RB_SIGN = 1;
int LIFT_SIGN = 1;

const int DIR_FORWARD_LEVEL = HIGH;

// ---------------- PWM ----------------
const int PWM_FREQ = 20000;
const int PWM_BITS = 8;

enum { M_LF = 0, M_RF, M_LB, M_RB, M_LIFT, M_COUNT };

const int dirPins[M_COUNT] = {
  LF_DIR, RF_DIR, LB_DIR, RB_DIR, LIFT_DIR
};

const int pwmPins[M_COUNT] = {
  LF_PWM, RF_PWM, LB_PWM, RB_PWM, LIFT_PWM
};

// ---------------- Ramp (acceleration) settings ----------------
const float DRIVE_ACCEL_RATE = 400.0f;
const float DRIVE_DECEL_RATE = 600.0f;

const float LIFT_ACCEL_RATE  = 300.0f;
const float LIFT_DECEL_RATE  = 500.0f;

const int MOTOR_MIN_PWM = 0;
const unsigned long RAMP_INTERVAL_MS = 10;

int   targetVal[M_COUNT]  = {0, 0, 0, 0, 0};   // requested value (-255..255)
float currentVal[M_COUNT] = {0, 0, 0, 0, 0};   // value actually applied
unsigned long lastRampUpdate = 0;

void pwmSetup() {
  for (int i = 0; i < M_COUNT; i++) {
    pinMode(dirPins[i], OUTPUT);
    digitalWrite(dirPins[i], LOW);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pwmPins[i], PWM_FREQ, PWM_BITS);
    ledcWrite(pwmPins[i], 0);
#else
    ledcSetup(i, PWM_FREQ, PWM_BITS);
    ledcAttachPin(pwmPins[i], i);
    ledcWrite(i, 0);
#endif
  }
}

inline void pwmWrite(int motor, int duty) {
  duty = constrain(duty, 0, 255);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pwmPins[motor], duty);
#else
  ledcWrite(motor, duty);
#endif
}

// Low-level: send value directly to the H-bridge driver
void applyMotor(int motor, int value) {
  value = constrain(value, -255, 255);

  if (value == 0) {
    pwmWrite(motor, 0);
    return;
  }

  bool forward = value > 0;
  digitalWrite(
    dirPins[motor],
    forward ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL
  );
  pwmWrite(motor, abs(value));
}

// Set target value for smooth ramp update
void setMotor(int motor, int value) {
  targetVal[motor] = constrain(value, -255, 255);
}

// Immediately stop all motors skipping the ramp (Emergency stop & Failsafe)
void hardStopAll() {
  for (int i = 0; i < M_COUNT; i++) {
    targetVal[i]  = 0;
    currentVal[i] = 0;
    applyMotor(i, 0);
  }
}

// Ramp update loop
void updateRamp() {
  unsigned long now = millis();
  if (now - lastRampUpdate < RAMP_INTERVAL_MS) return;

  float dt = (now - lastRampUpdate) / 1000.0f;
  lastRampUpdate = now;

  for (int i = 0; i < M_COUNT; i++) {
    float target = (float)targetVal[i];
    float cur    = currentVal[i];

    if (cur == target) continue;

    bool isLift = (i == M_LIFT);
    float accel = isLift ? LIFT_ACCEL_RATE : DRIVE_ACCEL_RATE;
    float decel = isLift ? LIFT_DECEL_RATE : DRIVE_DECEL_RATE;

    bool sameDirection = (cur == 0) || ((cur > 0) == (target > 0));
    bool speedingUp = sameDirection && (fabsf(target) > fabsf(cur));
    float rate = speedingUp ? accel : decel;

    float step = rate * dt;
    float diff = target - cur;

    if (fabsf(diff) <= step) {
      cur = target;
    } else {
      cur += (diff > 0) ? step : -step;
    }

    if (MOTOR_MIN_PWM > 0 && target != 0 && fabsf(cur) < MOTOR_MIN_PWM &&
        (cur == 0 || ((cur > 0) == (target > 0)))) {
      cur = (target > 0) ? MOTOR_MIN_PWM : -MOTOR_MIN_PWM;
      if (fabsf(target) < MOTOR_MIN_PWM) cur = target;
    }

    currentVal[i] = cur;
    applyMotor(i, (int)lroundf(cur));
  }
}

// ---------------- Differential Drive Helpers ----------------
void driveSides(int leftSpeed, int rightSpeed) {
  setMotor(M_LF, leftSpeed  * LF_SIGN);
  setMotor(M_LB, leftSpeed  * LB_SIGN);
  setMotor(M_RF, rightSpeed * RF_SIGN);
  setMotor(M_RB, rightSpeed * RB_SIGN);
}

void moveForward(int speed) {
  driveSides(speed, speed);
}

void moveBackward(int speed) {
  driveSides(-speed, -speed);
}

void turnLeft(int speed) {
  driveSides(-speed, speed);
}

void turnRight(int speed) {
  driveSides(speed, -speed);
}

void stopDrive() {
  driveSides(0, 0);
}

// ---------------- Lift Motor Helpers ----------------
void liftUp(int speed) {
  setMotor(M_LIFT, speed * LIFT_SIGN);
}

void liftDown(int speed) {
  setMotor(M_LIFT, -speed * LIFT_SIGN);
}

void liftStop() {
  setMotor(M_LIFT, 0);
}

void stopAll() {
  stopDrive();
  liftStop();
}

// ---------------- Safety Watchdog ----------------
const unsigned long FAILSAFE_MS = 500;
unsigned long lastCommand = 0;
bool driveActive = false;
bool liftActive = false;

// ---------------- HTTP API Handlers ----------------

void setCORS() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "*");
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
}

void handleRoot() {
  setCORS();
  String html = "<!DOCTYPE html><html><head><title>Forklift ESP32 Motor Controller</title>"
                "<style>body{font-family:sans-serif;background:#0b1120;color:#e2e8f0;padding:20px;line-height:1.6;}"
                "h2{color:#38bdf8;}code{background:#1e293b;padding:2px 6px;border-radius:4px;}</style></head><body>"
                "<h2>Forklift ESP32 Motor Controller</h2>"
                "<p>Status: <strong>ONLINE</strong></p>"
                "<p>IP Address: <code>" + WiFi.localIP().toString() + "</code></p>"
                "<p>Jetson Dashboard communicates with this device via HTTP endpoints:</p>"
                "<ul>"
                "<li><code>/cmd?a=ACTION&s=SPEED</code> (F, B, L, R, UP, DOWN, S, LS, X, PING)</li>"
                "<li><code>/drive?l=LEFT&r=RIGHT</code> (-255..255)</li>"
                "<li><code>/status</code> (JSON telemetry)</li>"
                "</ul>"
                "</body></html>";
  server.send(200, "text/html", html);
}

void handleCmd() {
  setCORS();

  if (!server.hasArg("a")) {
    server.send(400, "text/plain", "missing action (param 'a')");
    return;
  }

  String action = server.arg("a");
  int speed = server.hasArg("s") ? server.arg("s").toInt() : 120;
  speed = constrain(speed, 0, 255);

  lastCommand = millis();

  if      (action == "F")    { moveForward(speed); driveActive = true; }
  else if (action == "B")    { moveBackward(speed); driveActive = true; }
  else if (action == "L")    { turnLeft(speed); driveActive = true; }
  else if (action == "R")    { turnRight(speed); driveActive = true; }
  else if (action == "UP")   { liftUp(speed); liftActive = true; }
  else if (action == "DOWN") { liftDown(speed); liftActive = true; }
  else if (action == "S")    { stopDrive(); driveActive = false; }
  else if (action == "LS")   { liftStop(); liftActive = false; }
  else if (action == "X")    { hardStopAll(); driveActive = false; liftActive = false; }
  else if (action == "PING") { /* Watchdog keepalive heartbeat */ }
  else {
    server.send(400, "text/plain", "unknown command");
    return;
  }

  server.send(200, "text/plain", "OK");
}

void handleDrive() {
  setCORS();

  if (!server.hasArg("l") || !server.hasArg("r")) {
    server.send(400, "text/plain", "missing l or r parameters");
    return;
  }

  int l = server.arg("l").toInt();
  int r = server.arg("r").toInt();

  l = constrain(l, -255, 255);
  r = constrain(r, -255, 255);

  lastCommand = millis();
  driveSides(l, r);
  driveActive = (l != 0 || r != 0);

  server.send(200, "text/plain", "OK");
}

void handleStatus() {
  setCORS();

  String json = "{";
  json += "\"status\":\"online\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"rssi\":" + String(WiFi.RSSI()) + ",";
  json += "\"driveActive\":" + String(driveActive ? "true" : "false") + ",";
  json += "\"liftActive\":" + String(liftActive ? "true" : "false") + ",";
  json += "\"currentL\":" + String((int)currentVal[M_LF]) + ",";
  json += "\"currentR\":" + String((int)currentVal[M_RF]) + ",";
  json += "\"currentLift\":" + String((int)currentVal[M_LIFT]) + ",";
  json += "\"uptime_ms\":" + String(millis());
  json += "}";

  server.send(200, "application/json", json);
}

void handleOptions() {
  setCORS();
  server.send(204);
}

void printWiFiDetails() {
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("==========================================");
    Serial.println(">>> PHONE HOTSPOT CONNECTED <<<");
    Serial.print("Hotspot: "); Serial.println(WIFI_SSID);
    Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
    Serial.print("API Base URL: http://"); Serial.println(WiFi.localIP());
    Serial.println("==========================================");
  } else {
    Serial.println("Phone hotspot not connected yet.");
  }

  Serial.println("Backup AP:");
  Serial.print("SSID: "); Serial.println(BACKUP_AP_SSID);
  Serial.print("URL: http://"); Serial.println(WiFi.softAPIP());
}

void setup() {
  Serial.begin(115200);
  delay(300);
  pwmSetup();
  hardStopAll();

  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  WiFi.softAP(BACKUP_AP_SSID, BACKUP_AP_PASS);

  Serial.println("\nConnecting to phone hotspot: " + String(WIFI_SSID) + "...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  unsigned long startAttempt = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  wasWiFiConnected = (WiFi.status() == WL_CONNECTED);
  printWiFiDetails();

  // Register endpoints
  server.on("/", HTTP_GET, handleRoot);
  server.on("/cmd", HTTP_GET, handleCmd);
  server.on("/cmd", HTTP_POST, handleCmd);
  server.on("/cmd", HTTP_OPTIONS, handleOptions);

  server.on("/drive", HTTP_GET, handleDrive);
  server.on("/drive", HTTP_POST, handleDrive);
  server.on("/drive", HTTP_OPTIONS, handleOptions);

  server.on("/status", HTTP_GET, handleStatus);
  server.on("/status", HTTP_OPTIONS, handleOptions);

  server.begin();
  Serial.println("HTTP Motor Server started on port 80.");

  lastCommand = millis();
  lastWiFiRetry = millis();
  lastRampUpdate = millis();
}

void loop() {
  server.handleClient();
  updateRamp();

  // Monitor Wi-Fi connection
  bool connectedNow = (WiFi.status() == WL_CONNECTED);
  if (connectedNow && !wasWiFiConnected) {
    wasWiFiConnected = true;
    Serial.println("Hotspot reconnected.");
    printWiFiDetails();
  } else if (!connectedNow && wasWiFiConnected) {
    wasWiFiConnected = false;
    Serial.println("Hotspot disconnected. Reconnecting...");
  }

  if (!connectedNow && millis() - lastWiFiRetry >= WIFI_RETRY_INTERVAL_MS) {
    lastWiFiRetry = millis();
    WiFi.disconnect(false, false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  // Failsafe watchdog: If Jetson stops refreshing commands, stop all motors immediately
  if ((driveActive || liftActive) && (millis() - lastCommand > FAILSAFE_MS)) {
    hardStopAll();
    driveActive = false;
    liftActive = false;
    Serial.println("WATCHDOG FAILSAFE: command timeout, all motors stopped.");
  }
}
