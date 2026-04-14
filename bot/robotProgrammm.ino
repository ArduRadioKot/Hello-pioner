#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>

Adafruit_MPU6050 mpu;

#define DIR_1 27 
#define DIR_2 32 
#define SPEED_1 26
#define SPEED_2 33
#define SERVO_PIN 25

// WiFi настройки
const char* ssid = "myW";
const char* password = "1q2w3e4r";

WebServer server(80);
Servo lifter;

// Система позиционирования
float posX = 0.0;
float posY = 0.0;
float posAngle = 0.0;
bool isLifterUp = false;  // Переименовано для избежания конфликта

// Смещение датчика относительно центра (см)
float sensorOffsetX = 3.0;
float sensorOffsetY = 0.0;

// Структура точки маршрута
struct RoutePoint {
  float x, y, angle;
  String action;
  float value;
  unsigned long timestamp;
  RoutePoint(float _x, float _y, float _ang, String _act, float _val) 
    : x(_x), y(_y), angle(_ang), action(_act), value(_val), timestamp(millis()) {}
};

std::vector<RoutePoint> routeHistory;
const int MAX_HISTORY = 200;

// Параметры движения
const int BASE_SPEED = 200;
float SPEED_TO_CM_PER_SEC = 35.0;

// Калибровка поворотов
float TURN_TIME_PER_DEGREE_FAST = 6.3;
float TURN_TIME_PER_DEGREE_MED = 0.7;
float TURN_TIME_PER_DEGREE_SLOW = 0.4;

// Калибровка гироскопа
float gyroOffsetZ = 0.0;
unsigned long lastGyroTime = 0;

// HTTP обработчики
void handleRoot() {
  String html = "<html><body>";
  html += "<h1>Robot Control</h1>";
  html += "<p>Position: X=" + String(posX) + " Y=" + String(posY) + " Angle=" + String(posAngle) + "</p>";
  html += "<p>Lifter: " + String(isLifterUp ? "UP" : "DOWN") + "</p>";
  html += "<form action='/command' method='POST'>";
  html += "<textarea name='commands' rows='10' cols='40'></textarea><br>";
  html += "<input type='submit' value='Send'>";
  html += "</form>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleCommand() {
  if (server.hasArg("commands")) {
    String commands = server.arg("commands");
    server.send(200, "text/plain", "OK\nExecuting commands");
    executeCommands(commands);
  } else {
    server.send(400, "text/plain", "Missing commands");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"x\":" + String(posX) + ",";
  json += "\"y\":" + String(posY) + ",";
  json += "\"angle\":" + String(posAngle) + ",";
  json += "\"lifter\":\"" + String(isLifterUp ? "UP" : "DOWN") + "\",";
  json += "\"history_size\":" + String(routeHistory.size());
  json += "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

void setup() {
  Serial.begin(115200);
  
  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(SPEED_1, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  
  // Сервопривод
  lifter.attach(SERVO_PIN);
  lifter.write(0);  // DOWN позиция
  
  if (!mpu.begin()) {
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  delay(2000);
  
  float sumZ = 0;
  for(int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sumZ += g.gyro.z;
    delay(5);
  }
  gyroOffsetZ = sumZ / 100.0;
  
  lastGyroTime = millis();
  
  // WiFi подключение
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  
  // HTTP сервер
  server.on("/", handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop() {
  server.handleClient();
}

// ========== ВЫПОЛНЕНИЕ КОМАНД ==========

void executeCommands(String commands) {
  int start = 0;
  int end = commands.indexOf('\n');
  
  while (end != -1) {
    String line = commands.substring(start, end);
    line.trim();
    if (line.length() > 0) {
      executeCommand(line);
    }
    start = end + 1;
    end = commands.indexOf('\n', start);
  }
  
  // Последняя строка
  String line = commands.substring(start);
  line.trim();
  if (line.length() > 0) {
    executeCommand(line);
  }
}

void executeCommand(String cmd) {
  cmd.toUpperCase();
  
  if (cmd.startsWith("GO_LOCAL")) {
    float x = 0, y = 0;
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    if (secondSpace != -1) {
      x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
      y = cmd.substring(secondSpace + 1).toFloat();
    } else if (firstSpace != -1) {
      x = cmd.substring(firstSpace + 1).toFloat();
    }
    
    goToPoint(x, y);
  }
  else if (cmd == "UP") {
    lifterUp();
  }
  else if (cmd == "DOWN") {
    lifterDown();
  }
  else if (cmd.startsWith("DELAY")) {
    int space = cmd.indexOf(' ');
    if (space != -1) {
      int t = cmd.substring(space + 1).toInt();
      delay(t);
    }
  }
}

// ========== УПРАВЛЕНИЕ ПОДЪЁМНИКОМ ==========

void lifterUp() {
  lifter.write(180);
  isLifterUp = true;
  delay(500);
}

void lifterDown() {
  lifter.write(0);
  isLifterUp = false;
  delay(500);
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========

void addRoutePoint(String action, float value) {
  routeHistory.push_back(RoutePoint(posX, posY, posAngle, action, value));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
}

void getActualCenterPosition(float &centerX, float &centerY) {
  float rad = posAngle * PI / 180.0;
  float offsetGlobalX = sensorOffsetX * cos(rad) - sensorOffsetY * sin(rad);
  float offsetGlobalY = sensorOffsetX * sin(rad) + sensorOffsetY * cos(rad);
  centerX = posX - offsetGlobalX;
  centerY = posY - offsetGlobalY;
}

void updatePositionFromCenter(float centerX, float centerY, float angle) {
  float rad = angle * PI / 180.0;
  float offsetGlobalX = sensorOffsetX * cos(rad) - sensorOffsetY * sin(rad);
  float offsetGlobalY = sensorOffsetX * sin(rad) + sensorOffsetY * cos(rad);
  posX = centerX + offsetGlobalX;
  posY = centerY + offsetGlobalY;
  posAngle = angle;
}

void updateAngle() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;
  
  if (dt > 0.5) return;
  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  float gz = (g.gyro.z - gyroOffsetZ) * 180.0 / PI;
  if (abs(gz) < 1.0) gz = 0;
  
  posAngle += gz * dt;
  if (posAngle < 0) posAngle += 360.0;
  if (posAngle >= 360.0) posAngle -= 360.0;
}

float getTurnTimeMultiplier(float angle) {
  if (abs(angle) < 15) {
    return TURN_TIME_PER_DEGREE_SLOW;
  } else if (abs(angle) < 45) {
    return TURN_TIME_PER_DEGREE_MED;
  } else {
    return TURN_TIME_PER_DEGREE_FAST;
  }
}

// ========== ФУНКЦИИ ДВИЖЕНИЯ ==========

void forward(float length) {
  addRoutePoint("forward", length);
  
  float timeNeeded = length / SPEED_TO_CM_PER_SEC;
  unsigned long duration = timeNeeded * 1000;
  
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  unsigned long startTime = millis();
  
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  while (millis() - startTime < duration) {
    updateAngle();
    delay(10);
  }
  
  stop();
  
  float rad = posAngle * PI / 180.0;
  centerX += length * cos(rad);
  centerY += length * sin(rad);
  updatePositionFromCenter(centerX, centerY, posAngle);
}

void backward(float length) {
  addRoutePoint("backward", length);
  
  float timeNeeded = length / SPEED_TO_CM_PER_SEC;
  unsigned long duration = timeNeeded * 1000;
  
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  unsigned long startTime = millis();
  
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  while (millis() - startTime < duration) {
    updateAngle();
    delay(10);
  }
  
  stop();
  
  float rad = posAngle * PI / 180.0;
  centerX -= length * cos(rad);
  centerY -= length * sin(rad);
  updatePositionFromCenter(centerX, centerY, posAngle);
}

void left(float deg) {
  addRoutePoint("left", deg);
  
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  float multiplier = getTurnTimeMultiplier(deg);
  unsigned long turnTime = deg * multiplier;
  
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  unsigned long startTime = millis();
  while (millis() - startTime < turnTime) {
    updateAngle();
    delay(5);
  }
  
  stop();
  delay(50);
  
  posAngle += deg;
  if (posAngle >= 360) posAngle -= 360;
  
  updatePositionFromCenter(centerX, centerY, posAngle);
}

void right(float deg) {
  addRoutePoint("right", deg);
  
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  float multiplier = getTurnTimeMultiplier(deg);
  unsigned long turnTime = deg * multiplier;
  
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  unsigned long startTime = millis();
  while (millis() - startTime < turnTime) {
    updateAngle();
    delay(5);
  }
  
  stop();
  delay(50);
  
  posAngle -= deg;
  if (posAngle < 0) posAngle += 360;
  
  updatePositionFromCenter(centerX, centerY, posAngle);
}

void stop() {
  analogWrite(SPEED_1, 0);
  analogWrite(SPEED_2, 0);
  delay(100);
}

// ========== НАВИГАЦИЯ ==========

void goToPoint(float targetX, float targetY) {
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  float dx = targetX - centerX;
  float dy = targetY - centerY;
  float distance = sqrt(dx*dx + dy*dy);
  float targetAngle = atan2(dy, dx) * 180.0 / PI;
  if (targetAngle < 0) targetAngle += 360.0;
  
  float angleDiff = targetAngle - posAngle;
  if (angleDiff > 180) angleDiff -= 360;
  if (angleDiff < -180) angleDiff += 360;
  
  if (abs(angleDiff) > 5.0) {
    if (angleDiff > 0) left(angleDiff);
    else right(-angleDiff);
  }
  
  if (distance > 1.0) forward(distance);
}

void returnToHome() {
  if (routeHistory.empty()) return;
  
  for (int i = routeHistory.size() - 1; i >= 0; i--) {
    RoutePoint p = routeHistory[i];
    if (p.action == "forward") backward(p.value);
    else if (p.action == "backward") forward(p.value);
    else if (p.action == "left") right(p.value);
    else if (p.action == "right") left(p.value);
    delay(200);
  }
}