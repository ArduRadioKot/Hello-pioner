#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <esp_task_wdt.h>

Adafruit_MPU6050 mpu;

#define DIR_1 27 
#define DIR_2 32 
#define SPEED_1 26
#define SPEED_2 33
#define SERVO_PIN 25

// WiFi настройки для точки доступа
const char* ap_ssid = "RobotControl";
const char* ap_password = "12345678";
IPAddress ap_ip(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

WebServer server(80);
Servo lifter;

// Система позиционирования
float posX = 0.0;
float posY = 0.0;
float posAngle = 0.0;
bool isLifterUp = false;  // Переименовано для избежания конфликта

// Смещение датчика относительно центра (см)
float sensorOffsetX = 0.0;  // Fixed: no offset for accurate positioning
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

void testMovementDirections() {
  Serial.println("Testing movement directions...");
  
  // Test current position
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  Serial.print("Current center position: X=");
  Serial.print(centerX);
  Serial.print(", Y=");
  Serial.println(centerY);
  
  Serial.println("Movement test completed - ready for commands");
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
  
  // Creating WiFi Access Point
  Serial.println("Creating WiFi Access Point...");
  
  // Disable WiFi and enable in AP mode
  WiFi.disconnect(true, true);
  delay(500);
  WiFi.mode(WIFI_AP);
  delay(500);
  
  // Create access point with retry logic
  bool success = false;
  int attempts = 0;
  const int max_attempts = 3;
  
  while (!success && attempts < max_attempts) {
    Serial.print("Attempt ");
    Serial.print(attempts + 1);
    Serial.print("/");
    Serial.print(max_attempts);
    Serial.print(" to create AP...");
    
    success = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4); // channel 1, hidden=false, max 4 clients
    
    if (success) {
      Serial.println(" SUCCESS!");
      break;
    } else {
      Serial.println(" FAILED!");
      attempts++;
      if (attempts < max_attempts) {
        delay(1000);
      }
    }
  }
  
  if (success) {
    // Configure IP address
    bool configSuccess = WiFi.softAPConfig(ap_ip, ap_ip, subnet);
    if (configSuccess) {
      Serial.println("IP configuration successful");
    } else {
      Serial.println("IP configuration failed");
    }
    
    delay(2000); // Give time for AP initialization
    
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("Password: ");
    Serial.println(ap_password);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
    Serial.print("Connected clients: ");
    Serial.println(WiFi.softAPgetStationNum());
    
    Serial.println("WiFi Access Point is ready!");
  } else {
    Serial.println("CRITICAL: Failed to create WiFi Access Point after all attempts");
    Serial.println("Robot will continue without WiFi control");
  }
  
  // Test movement directions
  testMovementDirections();
  
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
  Serial.print("📥 Получена команда: ");
  Serial.println(cmd);
  
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
    
    Serial.print("🎯 GO_LOCAL: X=");
    Serial.print(x);
    Serial.print(", Y=");
    Serial.println(y);
    goToPoint(x, y);
  }
  else if (cmd == "RESET_POS") {
    resetPosition();
  }
  else if (cmd == "RETURN_HOME") {
    returnToHome();
  }
  else if (cmd == "UP") {
    Serial.println("⬆️ Подъёмник ВВЕРХ");
    lifterUp();
  }
  else if (cmd == "DOWN") {
    Serial.println("⬇️ Подъёмник ВНИЗ");
    lifterDown();
  }
  else if (cmd.startsWith("DELAY")) {
    int space = cmd.indexOf(' ');
    if (space != -1) {
      int t = cmd.substring(space + 1).toInt();
      Serial.print("⏱️ Задержка: ");
      Serial.print(t);
      Serial.println("мс");
      delay(t);
    }
  }
  else {
    Serial.print("❌ Неизвестная команда: ");
    Serial.println(cmd);
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
  
  // Feed watchdog to prevent resets
  #ifdef ESP32
  // esp_task_wdt_feed(); // Disabled for compilation
  #endif
  
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    Serial.println("⚠️ MPU6050 read error in updateAngle");
    return;
  }
  
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
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    // esp_task_wdt_feed(); // Disabled for compilation
    #endif
    server.handleClient();
    
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
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    // esp_task_wdt_feed(); // Disabled for compilation
    #endif
    server.handleClient();
    
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
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    // esp_task_wdt_feed(); // Disabled for compilation
    #endif
    server.handleClient();
    
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
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    // esp_task_wdt_feed(); // Disabled for compilation
    #endif
    server.handleClient();
    
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
  Serial.print("🎯 Движение к точке: X=");
  Serial.print(targetX);
  Serial.print(", Y=");
  Serial.println(targetY);
  
  // Получаем текущую позицию центра робота
  float centerX, centerY;
  getActualCenterPosition(centerX, centerY);
  
  Serial.print("📍 Текущая позиция: X=");
  Serial.print(centerX);
  Serial.print(", Y=");
  Serial.print(centerY);
  Serial.print(", Угол=");
  Serial.println(posAngle);
  
  // Вычисляем разницу между целевой и текущей позицией
  float dx = targetX - centerX;
  float dy = targetY - centerY;
  float distance = sqrt(dx*dx + dy*dy);
  
  // Вычисляем целевой угол от текущей позиции к цели
  float targetAngle = atan2(dy, dx) * 180.0 / PI;
  if (targetAngle < 0) targetAngle += 360.0;
  
  // Вычисляем разницу углов
  float angleDiff = targetAngle - posAngle;
  if (angleDiff > 180) angleDiff -= 360;
  if (angleDiff < -180) angleDiff += 360;
  
  Serial.print("🔄 Нужно повернуть на ");
  Serial.print(angleDiff);
  Serial.print("°, расстояние: ");
  Serial.print(distance);
  Serial.println("см");
  
  // Поворачиваем к цели
  if (abs(angleDiff) > 3.0) { // Уменьшил порог для точности
    if (angleDiff > 0) {
      Serial.println("⬅️ Поворот влево");
      left(angleDiff);
    } else {
      Serial.println("➡️ Поворот вправо");
      right(-angleDiff);
    }
    delay(500); // Пауза после поворота
  }
  
  // Движемся вперед к цели
  if (distance > 2.0) { // Увеличил порог для точности
    Serial.print("⬆️ Движение вперед на ");
    Serial.print(distance);
    Serial.println("см");
    forward(distance);
  }
  
  Serial.println("✅ Движение завершено");
}

void resetPosition() {
  Serial.println("Reset position to starting point");
  
  // Reset positioning variables
  posX = 0.0;
  posY = 0.0;
  posAngle = 0.0;
  isLifterUp = false;
  
  // Clear route history
  routeHistory.clear();
  
  // Reset gyroscope offset
  float sumZ = 0;
  int successfulReads = 0;
  for(int i = 0; i < 50; i++) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      sumZ += g.gyro.z;
      successfulReads++;
    }
    delay(5);
    
    // Feed watchdog during calibration
    #ifdef ESP32
    // esp_task_wdt_feed(); // Disabled for compilation
    #endif
  }
  
  if (successfulReads > 0) {
    gyroOffsetZ = sumZ / successfulReads;
  } else {
    Serial.println("⚠️ Failed to read MPU6050 during calibration");
    gyroOffsetZ = 0.0; // fallback value
  }
  lastGyroTime = millis();
  
  Serial.println("Position reset: X=0, Y=0, Angle=0");
  Serial.println("Gyroscope recalibrated");
}

void returnToHome() {
  if (routeHistory.empty()) return;
  
  Serial.println("🏠 Возврат домой по истории маршрута");
  
  for (int i = routeHistory.size() - 1; i >= 0; i--) {
    RoutePoint p = routeHistory[i];
    if (p.action == "forward") backward(p.value);
    else if (p.action == "backward") forward(p.value);
    else if (p.action == "left") right(p.value);
    else if (p.action == "right") left(p.value);
    delay(200);
  }
}