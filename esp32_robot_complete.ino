/*
 * ESP32 - Complete Robot Controller
 * Combines WiFi AP web interface with direct robot motor control
 * Can fully control robot independently
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

// Motor control pins
#define LEFT_MOTOR_A 27    // GPIO27
#define LEFT_MOTOR_B 32    // GPIO32
#define RIGHT_MOTOR_A 33   // GPIO33
#define RIGHT_MOTOR_B 26   // GPIO26

// Encoder pins
#define LEFT_ENCODER_A 14   // GPIO14
#define LEFT_ENCODER_B 12   // GPIO12
#define RIGHT_ENCODER_A 13   // GPIO13
#define RIGHT_ENCODER_B 15   // GPIO15

// Stepper motor pins
#define STEPPER_STEP 4     // GPIO4
#define STEPPER_DIR  2     // GPIO2
#define STEPPER_EN  0      // GPIO0

// WiFi settings
const char* ap_ssid = "RobotESP32";
const char* ap_password = "12345678";

WebServer server(80);

// Robot parameters
const float WHEEL_DIAMETER_MM = 68.0f;
const float WHEEL_BASE_MM = 185.0f;
const float ENCODER_RESOLUTION = 330.0f;
const float TICKS_TO_MM = (3.14159265f * WHEEL_DIAMETER_MM) / ENCODER_RESOLUTION;
const float WHEEL_BASE_M = WHEEL_BASE_MM / 1000.0f;

// Movement parameters
const int DRIVE_DIST_PWM_DEFAULT = 150;
const float DRIVE_DIST_M_PER_2S_AT_150 = 1.0f;
const int TURN_PWM_DEFAULT = 150;
const float TURN_90_DEG_MS_AT_150 = 400.0f;

// Control variables
volatile long left_encoder_value = 0;
volatile long right_encoder_value = 0;
long last_left_encoder = 0;
long last_right_encoder = 0;

float xPos = 0.0f, yPos = 0.0f, theta = 0.0f;
float targetX = 0.0f, targetY = 0.0f;
bool movingToTarget = false;

bool drivingDist = false;
uint32_t driveEndTime = 0;

bool turning = false;
uint32_t turnEndTime = 0;

bool stepperRunning = false;
long stepperTarget = 0;
long stepperCurrent = 0;
uint32_t stepperLastStep = 0;
const int STEPPER_SPEED_US = 500; // microseconds per step

// Control constants
const float KP_ANGLE = 2.0f;
const float KP_DIST = 0.5f;
const float MAX_LIN_M_S = 0.25f;
const float MAX_ANG_RAD_S = 1.2f;
const float ARRIVAL_DIST_M = 0.03f;

const long LIFT_UP_POS = 400;
const long LIFT_DOWN_POS = 0;

// Encoder interrupts
void IRAM_ATTR left_interrupt() {
  digitalRead(LEFT_ENCODER_B) ? left_encoder_value++ : left_encoder_value--;
}

void IRAM_ATTR right_interrupt() {
  digitalRead(RIGHT_ENCODER_B) ? right_encoder_value++ : right_encoder_value--;
}

// Odometry
void updateOdometry() {
  long dL = left_encoder_value - last_left_encoder;
  long dR = right_encoder_value - last_right_encoder;
  last_left_encoder = left_encoder_value;
  last_right_encoder = right_encoder_value;

  float distL_mm = dL * TICKS_TO_MM;
  float distR_mm = dR * TICKS_TO_MM;
  float distL_m = distL_mm / 1000.0f;
  float distR_m = distR_mm / 1000.0f;

  float deltaS = (distL_m + distR_m) * 0.5f;
  float deltaTheta = (distR_m - distL_m) / WHEEL_BASE_M;

  theta += deltaTheta;
  xPos += deltaS * cos(theta);
  yPos += deltaS * sin(theta);
}

float normalizeAngle(float a) {
  while (a > 3.14159265f) a -= 6.28318531f;
  while (a < -3.14159265f) a += 6.28318531f;
  return a;
}

// Motor control
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

void setVelocity(float vLin, float vAng) {
  float vL = vLin - vAng * (WHEEL_BASE_M * 0.5f);
  float vR = vLin + vAng * (WHEEL_BASE_M * 0.5f);

  const float gain = 255.0f / 0.25f;
  int leftPwm = (int)(vL * gain);
  int rightPwm = (int)(vR * gain);
  leftPwm = constrain(leftPwm, -255, 255);
  rightPwm = constrain(rightPwm, -255, 255);

  int leftA = 0, leftB = 0, rightA = 0, rightB = 0;
  if (leftPwm >= 0) { leftB = leftPwm; leftA = 0; } else { leftA = -leftPwm; leftB = 0; }
  if (rightPwm >= 0) { rightB = rightPwm; rightA = 0; } else { rightA = -rightPwm; rightB = 0; }

  setMotorsPWM(leftA, leftB, rightA, rightB);
}

void driveToTarget() {
  float dx = targetX - xPos;
  float dy = targetY - yPos;
  float dist = sqrt(dx * dx + dy * dy);

  if (dist < ARRIVAL_DIST_M) {
    setMotorsPWM(0, 0, 0, 0);
    movingToTarget = false;
    Serial.println("OK: Arrived");
    return;
  }

  float angleToTarget = atan2(dy, dx);
  float angleErr = normalizeAngle(angleToTarget - theta);

  float w = KP_ANGLE * angleErr;
  w = constrain(w, -MAX_ANG_RAD_S, MAX_ANG_RAD_S);

  float v = KP_DIST * dist;
  v = constrain(v, 0.0f, MAX_LIN_M_S);

  setVelocity(v, w);
}

// Stepper control
void updateStepper() {
  if (!stepperRunning) return;
  
  if (micros() - stepperLastStep >= STEPPER_SPEED_US) {
    if (stepperCurrent < stepperTarget) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEPPER_STEP, LOW);
      stepperCurrent++;
      stepperLastStep = micros();
    } else if (stepperCurrent > stepperTarget) {
      digitalWrite(STEPPER_STEP, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEPPER_STEP, LOW);
      stepperCurrent--;
      stepperLastStep = micros();
    } else {
      stepperRunning = false;
    }
  }
}

void setStepperTarget(long target) {
  stepperTarget = target;
  stepperRunning = true;
  digitalWrite(STEPPER_EN, LOW); // Enable stepper
}

// Command processing
bool parseGoTo(const String& cmd, float* x, float* y) {
  int i1 = cmd.indexOf(' ');
  if (i1 < 0) return false;
  int i2 = cmd.indexOf(' ', i1 + 1);
  if (i2 < 0) return false;
  *x = cmd.substring(i1 + 1, i2).toFloat();
  *y = cmd.substring(i2 + 1).toFloat();
  return true;
}

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
      movingToTarget = false;
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
      movingToTarget = false;
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
      movingToTarget = false;
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
  
  if (command.startsWith("GO_TO")) {
    float x = 0, y = 0;
    if (parseGoTo(command, &x, &y)) {
      targetX = x;
      targetY = y;
      movingToTarget = true;
      drivingDist = false;
      turning = false;
      Serial.print("OK: Going to "); Serial.print(x, 3); Serial.print(" "); Serial.println(y, 3);
    } else {
      Serial.println("ERROR: GO_TO x y (meters)");
    }
    return;
  }
  
  if (command.startsWith("LIFT_UP")) {
    setStepperTarget(LIFT_UP_POS);
    Serial.println("OK: Lift up");
    return;
  }
  
  if (command.startsWith("LIFT_DOWN")) {
    setStepperTarget(LIFT_DOWN_POS);
    Serial.println("OK: Lift down");
    return;
  }
  
  if (command.startsWith("STOP")) {
    movingToTarget = false;
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
  return R"(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32 Robot Control</title>
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
  <h1>ESP32 Robot Control</h1>
  
  <div class="status">
    <strong>Status:</strong> <span id="status">Ready</span><br>
    <strong>Position:</strong> X:<span id="xPos">0.00</span> Y:<span id="yPos">0.00</span> Angle:<span id="theta">0.00</span><br>
    <strong>Encoders:</strong> L:<span id="leftEnc">0</span> R:<span id="rightEnc">0</span>
  </div>

  <h1>Distance Control</h1>
  <label>Distance (m): <input type="number" id="dist" step="0.1" value="1"></label>
  <p class="hint">>0 forward, <0 backward</p>
  <label>PWM (1-255, default 150): <input type="number" id="pwm" min="1" max="255" placeholder="150"></label>
  <div style="display: flex; gap: 8px;">
    <button class="go" style="flex: 1;" onclick="driveDist(1)">Forward</button>
    <button class="go" style="flex: 1;" onclick="driveDist(-1)">Backward</button>
  </div>
  <button class="go" style="margin-top: 8px;" onclick="driveDistFromInput()">Drive N meters</button>

  <h1 style="margin-top: 24px;">Turn Control</h1>
  <p class="hint">>0 left, <0 right. 90° in 400ms at PWM 150</p>
  <label>Angle (deg): <input type="number" id="angle" step="15" value="90"></label>
  <label>PWM (optional): <input type="number" id="turnPwm" min="1" max="255" placeholder="150"></label>
  <div style="display: flex; gap: 8px; margin-top: 8px;">
    <button class="go" style="flex: 1;" onclick="turn(-90)">Right 90°</button>
    <button class="go" style="flex: 1;" onclick="turn(90)">Left 90°</button>
  </div>
  <div style="display: flex; gap: 8px; margin-top: 4px;">
    <button class="go" style="flex: 1;" onclick="turn(-45)">Right 45°</button>
    <button class="go" style="flex: 1;" onclick="turn(45)">Left 45°</button>
  </div>
  <button class="go" style="margin-top: 8px;" onclick="turnByInput()">Turn by angle</button>

  <h1 style="margin-top: 24px;">Navigation</h1>
  <label>Target X (m): <input type="number" id="targetX" step="0.1" value="1"></label>
  <label>Target Y (m): <input type="number" id="targetY" step="0.1" value="0"></label>
  <button class="go" onclick="goToTarget()">Go to target</button>

  <h1 style="margin-top: 24px;">Lift Control</h1>
  <div style="display: flex; gap: 8px;">
    <button class="go" style="flex: 1;" onclick="liftUp()">Lift Up</button>
    <button class="go" style="flex: 1;" onclick="liftDown()">Lift Down</button>
  </div>

  <button class="stop" onclick="stop()">STOP</button>
  <p id="msg"></p>
  
  <script>
    function updateStatus() {
      fetch('/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('status').textContent = data.status;
          document.getElementById('xPos').textContent = data.x.toFixed(3);
          document.getElementById('yPos').textContent = data.y.toFixed(3);
          document.getElementById('theta').textContent = data.theta.toFixed(3);
          document.getElementById('leftEnc').textContent = data.leftEnc;
          document.getElementById('rightEnc').textContent = data.rightEnc;
        })
        .catch(() => {});
    }
    
    function liftUp() {
      document.getElementById('msg').textContent = 'Lifting up...';
      fetch('/lift_up')
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
    }
    
    function liftDown() {
      document.getElementById('msg').textContent = 'Lifting down...';
      fetch('/lift_down')
        .then(r => r.text())
        .then(t => document.getElementById('msg').textContent = 'Sent: ' + t)
        .catch(() => document.getElementById('msg').textContent = 'Error');
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
      document.getElementById('msg').textContent = 'Turning ' + angle + '°...';
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
    
    function goToTarget() {
      var x = document.getElementById('targetX').value;
      var y = document.getElementById('targetY').value;
      if (!x || !y) { 
        document.getElementById('msg').textContent = 'Enter X and Y'; 
        return; 
      }
      document.getElementById('msg').textContent = 'Navigating...';
      fetch('/go_to?x=' + encodeURIComponent(x) + '&y=' + encodeURIComponent(y))
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
)";
}

// Web handlers
void handleRoot() {
  server.send(200, "text/html; charset=utf-8", getHtml());
}

void handleStatus() {
  String json = "{\"status\":\"";
  json += drivingDist ? "Driving" : (turning ? "Turning" : (movingToTarget ? "Navigating" : "Ready"));
  json += "\",\"x\":" + String(xPos, 3);
  json += ",\"y\":" + String(yPos, 3);
  json += ",\"theta\":" + String(theta, 3);
  json += ",\"leftEnc\":" + String(left_encoder_value);
  json += ",\"rightEnc\":" + String(right_encoder_value);
  json += "}";
  server.send(200, "application/json", json);
}

void handleLiftUp() {
  processCommand("LIFT_UP");
  server.send(200, "text/plain", "LIFT_UP");
}

void handleLiftDown() {
  processCommand("LIFT_DOWN");
  server.send(200, "text/plain", "LIFT_DOWN");
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

void handleGoTo() {
  if (!server.hasArg("x") || !server.hasArg("y")) {
    server.send(400, "text/plain", "x and y required");
    return;
  }
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  char buf[64];
  snprintf(buf, sizeof(buf), "GO_TO %.3f %.3f", x, y);
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
  Serial.println("ESP32 Robot Controller Starting...");

  // Setup pins
  pinMode(LEFT_MOTOR_A, OUTPUT);
  pinMode(LEFT_MOTOR_B, OUTPUT);
  pinMode(RIGHT_MOTOR_A, OUTPUT);
  pinMode(RIGHT_MOTOR_B, OUTPUT);
  
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  
  pinMode(STEPPER_STEP, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_EN, OUTPUT);
  
  digitalWrite(STEPPER_EN, HIGH); // Disable stepper initially
  digitalWrite(STEPPER_STEP, LOW);
  digitalWrite(STEPPER_DIR, HIGH);

  // Setup encoder interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), left_interrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), right_interrupt, RISING);

  // Setup WiFi AP with proper configuration
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_AP);
  delay(500);
  
  // Set AP configuration
  IPAddress ap_ip(192, 168, 4, 1);
  IPAddress ap_gateway(192, 168, 4, 1);
  IPAddress ap_subnet(255, 255, 255, 0);
  
  bool success = WiFi.softAPConfig(ap_ip, ap_gateway, ap_subnet);
  if (!success) {
    Serial.println("Failed to configure AP");
  }
  
  // Start AP with channel 1, max 4 clients
  success = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);
  
  if (success) {
    delay(2000); // Wait for AP to start
    IPAddress ip = WiFi.softAPIP();
    
    Serial.println("WiFi AP Started Successfully!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("IP Address: ");
    Serial.println(ip);
    Serial.print("Gateway: ");
    Serial.println(ap_gateway);
    Serial.print("Subnet: ");
    Serial.println(ap_subnet);
    Serial.print("Channel: ");
    Serial.println(WiFi.softAPchannel());
    Serial.print("Max connections: ");
    Serial.println(WiFi.softAPgetStationNum());
  } else {
    Serial.println("Failed to start WiFi AP!");
    Serial.println("Retrying...");
    delay(1000);
    // Retry once
    success = WiFi.softAP(ap_ssid, ap_password);
    if (success) {
      Serial.println("WiFi AP Started on retry!");
      Serial.print("SSID: ");
      Serial.println(ap_ssid);
      Serial.print("IP: ");
      Serial.println(WiFi.softAPIP());
    } else {
      Serial.println("Failed to start AP even on retry!");
    }
  }

  // Setup web server
  server.on("/", handleRoot);
  server.on("/status", handleStatus);
  server.on("/drive_dist", handleDriveDist);
  server.on("/turn", handleTurn);
  server.on("/go_to", handleGoTo);
  server.on("/lift_up", handleLiftUp);
  server.on("/lift_down", handleLiftDown);
  server.on("/stop", handleStop);
  server.on("/pwm", handlePwm);
  server.begin();
  
  Serial.println("ESP32 Robot Ready!");
}

void loop() {
  server.handleClient();
  
  // Process serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  // Update robot state
  updateOdometry();
  updateStepper();

  if (movingToTarget) {
    driveToTarget();
  }

  if (drivingDist && millis() >= driveEndTime) {
    setMotorsPWM(0, 0, 0, 0);
    drivingDist = false;
  }

  if (turning && millis() >= turnEndTime) {
    setMotorsPWM(0, 0, 0, 0);
    turning = false;
  }

  // Print status every 100ms
  static uint32_t printTimer = 0;
  static uint32_t wifiTimer = 0;
  if (millis() - printTimer > 100) {
    Serial.print(" x="); Serial.print(xPos, 3);
    Serial.print(" y="); Serial.print(yPos, 3);
    Serial.print(" th="); Serial.print(theta, 3);
    Serial.print(" L="); Serial.print(left_encoder_value);
    Serial.print(" R="); Serial.print(right_encoder_value);
    Serial.print(" WiFi_ST="); Serial.print(WiFi.status());
    Serial.print(" Clients="); Serial.println(WiFi.softAPgetStationNum());
    printTimer = millis();
  }
  
  // Check WiFi status every 5 seconds
  if (millis() - wifiTimer > 5000) {
    Serial.print("WiFi Status: ");
    Serial.println(WiFi.status());
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Connected clients: ");
    Serial.println(WiFi.softAPgetStationNum());
    wifiTimer = millis();
  }
}
