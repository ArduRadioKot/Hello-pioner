/*
 * ESP8266 - Complete Robot Controller
 * Combines WiFi AP web interface with direct robot motor control
 * Can fully control robot independently
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>

// Motor control pins
#define LEFT_MOTOR_A 5     // D1 on NodeMCU
#define LEFT_MOTOR_B 4     // D2 on NodeMCU  
#define RIGHT_MOTOR_A 0    // D3 on NodeMCU
#define RIGHT_MOTOR_B 2    // D4 on NodeMCU

// Encoders and stepper removed for simplicity

// WiFi settings
const char* ap_ssid = "RobotESP8266";
const char* ap_password = "12345678";

ESP8266WebServer server(80);

// Movement parameters
const int DRIVE_DIST_PWM_DEFAULT = 150;
const float DRIVE_DIST_M_PER_2S_AT_150 = 1.0f;
const int TURN_PWM_DEFAULT = 150;
const float TURN_90_DEG_MS_AT_150 = 400.0f;

// Control variables
bool drivingDist = false;
uint32_t driveEndTime = 0;
bool turning = false;
uint32_t turnEndTime = 0;

// Simple motor control only

// Simple motor control
void setMotorsPWM(int leftA, int leftB, int rightA, int rightB) {
  leftA = constrain(leftA, 0, 255);
  leftB = constrain(leftB, 0, 255);
  rightA = constrain(rightA, 0, 255);
  rightB = constrain(rightB, 0, 255);
  
  analogWrite(LEFT_MOTOR_A, leftA);
  analogWrite(LEFT_MOTOR_B, leftB);
  analogWrite(RIGHT_MOTOR_A, rightA);
  analogWrite(RIGHT_MOTOR_B, rightB);
}

// Command processing
bool parseDriveDist(const String& cmd, float* dist_m, int* pwm) {
  int i1 = cmd.indexOf(' ');
  if (i1 < 0) return false;
  int i2 = cmd.indexOf(' ', i1 + 1);
  *dist_m = cmd.substring(i1 + 1, i2 > 0 ? i2 : cmd.length()).toFloat();
  *pwm = DRIVE_DIST_PWM_DEFAULT;
  if (i2 > 0) {
    String rest = cmd.substring(i2 + 1);
    rest.trim();
    if (rest.length() > 0) *pwm = rest.toInt();
  }
  *pwm = constrain(*pwm, 1, 255);
  return (*dist_m > 0.01f || *dist_m < -0.01f);
}

bool parseTurn(const String& cmd, float* angle_deg, int* pwm) {
  int i1 = cmd.indexOf(' ');
  if (i1 < 0) return false;
  int i2 = cmd.indexOf(' ', i1 + 1);
  *angle_deg = cmd.substring(i1 + 1, i2 > 0 ? i2 : cmd.length()).toFloat();
  *pwm = TURN_PWM_DEFAULT;
  if (i2 > 0) {
    String rest = cmd.substring(i2 + 1);
    rest.trim();
    if (rest.length() > 0) *pwm = rest.toInt();
  }
  *pwm = constrain(*pwm, 1, 255);
  return (*angle_deg > 0.1f || *angle_deg < -0.1f);
}

bool parseSetPWM(const String& command, int* leftA_PWM, int* leftB_PWM, int* rightA_PWM, int* rightB_PWM) {
  int index = command.indexOf(' ');
  if (index == -1) return false;
  String params = command.substring(index + 1);
  params.trim();

  int idx1 = params.indexOf(' ');
  if (idx1 == -1) return false;
  int idx2 = params.indexOf(' ', idx1 + 1);
  if (idx2 == -1) return false;
  int idx3 = params.indexOf(' ', idx2 + 1);
  if (idx3 == -1) return false;

  *leftA_PWM  = params.substring(0, idx1).toInt();
  *leftB_PWM  = params.substring(idx1 + 1, idx2).toInt();
  *rightA_PWM = params.substring(idx2 + 1, idx3).toInt();
  *rightB_PWM = params.substring(idx3 + 1).toInt();
  return true;
}

void processCommand(String command) {
  command.trim();
  
  if (command.startsWith("SET_PWM")) {
    int leftA_PWM = 0, leftB_PWM = 0, rightA_PWM = 0, rightB_PWM = 0;
    if (parseSetPWM(command, &leftA_PWM, &leftB_PWM, &rightA_PWM, &rightB_PWM)) {
      setMotorsPWM(leftA_PWM, leftB_PWM, rightA_PWM, rightB_PWM);
      drivingDist = false;
      turning = false;
      Serial.println("OK: Set PWM");
    }
    return;
  }
  
  if (command.startsWith("TURN")) {
    float angle = 0.0f;
    int pwm = TURN_PWM_DEFAULT;
    if (parseTurn(command, &angle, &pwm)) {
      drivingDist = false;
      unsigned long time_ms = (unsigned long)(fabs(angle) / 90.0f * TURN_90_DEG_MS_AT_150 * (float)TURN_PWM_DEFAULT / (float)pwm);
      if (angle > 0) {
        setMotorsPWM(pwm, 0, 0, pwm);  // left
      } else {
        setMotorsPWM(0, pwm, pwm, 0);  // right
      }
      turnEndTime = millis() + time_ms;
      turning = true;
      Serial.print("OK: TURN "); Serial.print(angle, 0); Serial.print(" deg, "); Serial.print(time_ms); Serial.println(" ms");
    } else {
      Serial.println("ERROR: TURN angle_deg [pwm]");
    }
    return;
  }
  
  if (command.startsWith("DRIVE_DIST")) {
    float d = 0.0f;
    int pwm = DRIVE_DIST_PWM_DEFAULT;
    if (parseDriveDist(command, &d, &pwm)) {
      turning = false;
      float abs_d = fabs(d);
      unsigned long time_ms = (unsigned long)(abs_d * 2000.0f * (float)DRIVE_DIST_PWM_DEFAULT / (float)pwm);
      if (d >= 0) {
        setMotorsPWM(0, pwm, 0, pwm);  // forward
      } else {
        setMotorsPWM(pwm, 0, pwm, 0);  // backward
      }
      driveEndTime = millis() + time_ms;
      drivingDist = true;
      Serial.print("OK: DRIVE_DIST "); Serial.print(d, 2); Serial.print(" m, PWM "); Serial.print(pwm); Serial.print(", "); Serial.print(time_ms / 1000.0f, 1); Serial.println(" s");
    } else {
      Serial.println("ERROR: DRIVE_DIST distance_m [pwm]");
    }
    return;
  }
  
  if (command.startsWith("STOP")) {
    drivingDist = false;
    turning = false;
    setMotorsPWM(0, 0, 0, 0);
    Serial.println("OK: Stopped");
    return;
  }
  
  Serial.println("ERROR: Unknown command");
}

// Web interface
String getHtml() {
  return R"RAW(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP8266 Robot Control</title>
  <style>
    body { font-family: sans-serif; max-width: 400px; margin: 20px auto; padding: 10px; }
    h1 { font-size: 1.2em; color: #333; }
    label { display: block; margin-top: 10px; }
    input { width: 100%; padding: 8px; box-sizing: border-box; }
    button { margin-top: 12px; padding: 10px 16px; width: 100%; font-size: 1em; }
    .go { background: #2e7d32; color: white; border: none; }
    .stop { background: #c62828; color: white; border: none; margin-top: 8px; }
    .hint { font-size: 0.85em; color: #666; margin-top: 4px; }
    .status { background: #f5f5f5; padding: 10px; margin: 10px 0; border-radius: 4px; }
  </style>
</head>
<body>
  <h1>ESP8266 Robot Control</h1>
  
  <div class="status">
    <strong>Status:</strong> <span id="status">Ready</span><br>
    <strong>Motors:</strong> <span id="motorStatus">Stopped</span>
  </div>

  <h1>Distance Control</h1>
  <label>Distance (m): <input type="number" id="dist" step="0.1" value="1"></label>
  <p class="hint">>0 forward, &lt;0 backward</p>
  <label>PWM (1-255, default 150): <input type="number" id="pwm" min="1" max="255" placeholder="150"></label>
  <div style="display: flex; gap: 8px;">
    <button class="go" style="flex: 1;" onclick="driveDist(1)">Forward</button>
    <button class="go" style="flex: 1;" onclick="driveDist(-1)">Backward</button>
  </div>
  <button class="go" style="margin-top: 8px;" onclick="driveDistFromInput()">Drive N meters</button>

  <h1 style="margin-top: 24px;">Turn Control</h1>
  <p class="hint">&gt;0 left, &lt;0 right. 90 deg in 400ms at PWM 150</p>
  <label>Angle (deg): <input type="number" id="angle" step="15" value="90"></label>
  <label>PWM (optional): <input type="number" id="turnPwm" min="1" max="255" placeholder="150"></label>
  <div style="display: flex; gap: 8px; margin-top: 8px;">
    <button class="go" style="flex: 1;" onclick="turn(-90)">Right 90 deg</button>
    <button class="go" style="flex: 1;" onclick="turn(90)">Left 90 deg</button>
  </div>
  <div style="display: flex; gap: 8px; margin-top: 4px;">
    <button class="go" style="flex: 1;" onclick="turn(-45)">Right 45 deg</button>
    <button class="go" style="flex: 1;" onclick="turn(45)">Left 45 deg</button>
  </div>
  <button class="go" style="margin-top: 8px;" onclick="turnByInput()">Turn by angle</button>

  <h1 style="margin-top: 24px;">Direct PWM Control</h1>
  <label>Left Motor A: <input type="number" id="la" min="0" max="255" value="0"></label>
  <label>Left Motor B: <input type="number" id="lb" min="0" max="255" value="0"></label>
  <label>Right Motor A: <input type="number" id="ra" min="0" max="255" value="0"></label>
  <label>Right Motor B: <input type="number" id="rb" min="0" max="255" value="0"></label>
  <button class="go" onclick="setPwm()">Set PWM</button>

  <button class="stop" onclick="stop()">STOP</button>
  <p id="msg"></p>
  
  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('status').textContent = data.status;
          document.getElementById('motorStatus').textContent = data.status;
        })
        .catch(() => {});
    }
    
    function driveDist(sign) {
      var d = document.getElementById('dist').value;
      var pwm = document.getElementById('pwm').value;
      var v = sign ? sign * Math.abs(parseFloat(d) || 1) : parseFloat(d);
      if (!isFinite(v) || Math.abs(v) < 0.01) { 
        document.getElementById('msg').textContent = 'Distance |d| >= 0.01'; 
        return; 
      }
      document.getElementById('msg').textContent = 'Sending...';
      var url = '/drive_dist?d=' + encodeURIComponent(v);
      if (pwm) url += '&pwm=' + encodeURIComponent(pwm);
      fetch(url)
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    function driveDistFromInput() { driveDist(0); }
    
    function turn(angle) {
      document.getElementById('msg').textContent = 'Turning ' + angle + ' deg...';
      var url = '/turn?angle=' + angle;
      fetch(url)
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    function turnByInput() {
      var angle = document.getElementById('angle').value;
      var pwm = document.getElementById('turnPwm').value;
      if (!angle) { 
        document.getElementById('msg').textContent = 'Enter angle'; 
        return; 
      }
      document.getElementById('msg').textContent = 'Turning...';
      var url = '/turn?angle=' + encodeURIComponent(angle);
      if (pwm) url += '&pwm=' + encodeURIComponent(pwm);
      fetch(url)
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    function setPwm() {
      var la = document.getElementById('la').value;
      var lb = document.getElementById('lb').value;
      var ra = document.getElementById('ra').value;
      var rb = document.getElementById('rb').value;
      document.getElementById('msg').textContent = 'Setting PWM...';
      var url = '/pwm?la=' + encodeURIComponent(la) + '&lb=' + encodeURIComponent(lb) + '&ra=' + encodeURIComponent(ra) + '&rb=' + encodeURIComponent(rb);
      fetch(url)
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    function stop() {
      document.getElementById('msg').textContent = 'Sending STOP...';
      fetch('/stop')
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: STOP')
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    // Update status every 100ms
    setInterval(updateStatus, 100);
    updateStatus();
  </script>
</body>
</html>
)RAW";
}

// Web handlers
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", getHtml());
}

void handleStatus() {
  String json = "{\"status\":\"";
  json += drivingDist ? "Driving" : (turning ? "Turning" : "Ready");
  json += "\"}";
  server.send(200, "application/json", json);
}

void handleStop() {
  processCommand("STOP");
  server.send(200, "text/plain", "OK");
}

void handleDriveDist() {
  if (!server.hasArg("d")) {
    server.send(400, "text/plain", "d (meters) required");
    return;
  }
  float d = server.arg("d").toFloat();
  if (fabs(d) < 0.01f) {
    server.send(400, "text/plain", "|d| must be >= 0.01");
    return;
  }
  char buf[64];
  if (server.hasArg("pwm")) {
    int pwm = server.arg("pwm").toInt();
    pwm = constrain(pwm, 1, 255);
    snprintf(buf, sizeof(buf), "DRIVE_DIST %.2f %d", d, pwm);
  } else {
    snprintf(buf, sizeof(buf), "DRIVE_DIST %.2f", d);
  }
  processCommand(buf);
  server.send(200, "text/plain", buf);
}

void handleTurn() {
  if (!server.hasArg("angle")) {
    server.send(400, "text/plain", "angle (degrees) required");
    return;
  }
  float angle = server.arg("angle").toFloat();
  char buf[64];
  if (server.hasArg("pwm")) {
    int pwm = server.arg("pwm").toInt();
    pwm = constrain(pwm, 1, 255);
    snprintf(buf, sizeof(buf), "TURN %.0f %d", angle, pwm);
  } else {
    snprintf(buf, sizeof(buf), "TURN %.0f", angle);
  }
  processCommand(buf);
  server.send(200, "text/plain", buf);
}


void handlePwm() {
  if (!server.hasArg("la") || !server.hasArg("lb") || !server.hasArg("ra") || !server.hasArg("rb")) {
    server.send(400, "text/plain", "la lb ra rb required");
    return;
  }
  char buf[64];
  snprintf(buf, sizeof(buf), "SET_PWM %s %s %s %s",
           server.arg("la").c_str(), server.arg("lb").c_str(),
           server.arg("ra").c_str(), server.arg("rb").c_str());
  processCommand(buf);
  server.send(200, "text/plain", "OK");
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  yield();
  Serial.println("ESP8266 Robot Controller Starting...");
  yield();

  // Setup motor pins only
  pinMode(LEFT_MOTOR_A, OUTPUT);
  yield();
  pinMode(LEFT_MOTOR_B, OUTPUT);
  yield();
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  yield();
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  yield();
  
  // Stop motors initially
  digitalWrite(LEFT_MOTOR_A, LOW);
  digitalWrite(LEFT_MOTOR_B, LOW);
  digitalWrite(RIGHT_MOTOR_A, LOW);
  digitalWrite(RIGHT_MOTOR_B, LOW);
  yield();
  
  Serial.println("Motors initialized - encoders and stepper removed");
  yield();

  // Setup WiFi AP with proper configuration
  WiFi.disconnect(true);
  yield();
  delay(500);
  yield();
  WiFi.mode(WIFI_AP);
  yield();
  delay(500);
  yield();
  
  // Set AP configuration
  IPAddress ap_ip(192, 168, 4, 1);
  IPAddress ap_gateway(192, 168, 4, 1);
  IPAddress ap_subnet(255, 255, 255, 0);
  yield();
  
  bool success = WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  yield();
  if (!success) {
    Serial.println("Failed to configure AP");
  }
  yield();
  
  // Start AP with channel 1, max 4 clients
  success = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);
  yield();
  
  if (success) {
    delay(2000); // Wait for AP to start
    yield();
    IPAddress ip = WiFi.softAPIP();
    yield();
    
    Serial.println("WiFi AP Started Successfully!");
    yield();
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    yield();
    Serial.print("Password: ");
    Serial.println(ap_password);
    yield();
    Serial.print("IP Address: ");
    Serial.println(ip);
    yield();
    Serial.print("Gateway: ");
    Serial.println(ap_gateway);
    yield();
    Serial.print("Subnet: ");
    Serial.println(ap_subnet);
    yield();
    Serial.print("Max connections: ");
    Serial.println(WiFi.softAPgetStationNum());
    yield();
  } else {
    Serial.println("Failed to start WiFi AP!");
    yield();
    Serial.println("Retrying...");
    yield();
    delay(1000);
    yield();
    // Retry once
    success = WiFi.softAP(ap_ssid, ap_password);
    yield();
    if (success) {
      Serial.println("WiFi AP Started on retry!");
      yield();
      Serial.print("SSID: ");
      Serial.println(ap_ssid);
      yield();
      Serial.print("IP: ");
      Serial.println(WiFi.softAPIP());
      yield();
    } else {
      Serial.println("Failed to start AP even on retry!");
      yield();
    }
  }

  // Setup web server
  server.on("/", handleRoot);
  yield();
  server.on("/status", handleStatus);
  yield();
  server.on("/drive_dist", handleDriveDist);
  yield();
  server.on("/turn", handleTurn);
  yield();
  server.on("/stop", handleStop);
  yield();
  server.on("/pwm", handlePwm);
  yield();
  server.begin();
  yield();
  
  Serial.println("ESP8266 Robot Ready!");
  yield();
}

void loop() {
  yield();
  server.handleClient();
  yield();
  
  // Process serial commands
  if (Serial.available()) {
    yield();
    String command = Serial.readStringUntil('\n');
    yield();
    processCommand(command);
    yield();
  }

  // Check movement timeouts
  if (drivingDist && millis() >= driveEndTime) {
    yield();
    setMotorsPWM(0, 0, 0, 0);
    yield();
    drivingDist = false;
    yield();
  }

  if (turning && millis() >= turnEndTime) {
    yield();
    setMotorsPWM(0, 0, 0, 0);
    yield();
    turning = false;
    yield();
  }

  // Print status every 100ms
  static uint32_t printTimer = 0;
  static uint32_t wifiTimer = 0;
  if (millis() - printTimer > 100) {
    yield();
    Serial.print(" Status: ");
    if (drivingDist) Serial.print("Driving");
    else if (turning) Serial.print("Turning");
    else Serial.print("Ready");
    yield();
    Serial.print(" WiFi_ST="); Serial.print(WiFi.status());
    yield();
    Serial.print(" Clients="); Serial.println(WiFi.softAPgetStationNum());
    yield();
    printTimer = millis();
  }
  
  // Check WiFi status every 5 seconds
  if (millis() - wifiTimer > 5000) {
    yield();
    Serial.print("WiFi Status: ");
    yield();
    Serial.println(WiFi.status());
    yield();
    Serial.print("AP IP: ");
    yield();
    Serial.println(WiFi.softAPIP());
    yield();
    Serial.print("Connected clients: ");
    yield();
    Serial.println(WiFi.softAPgetStationNum());
    yield();
    wifiTimer = millis();
  }
  
  yield();
}
