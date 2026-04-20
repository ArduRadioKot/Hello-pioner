#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>
#include <vector>
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include "esp_task_wdt.h"

// Пины управления моторами
#define DIR_1 27 
#define DIR_2 32 
#define SPEED_1 26
#define SPEED_2 33
#define SERVO_PIN 25

// Настройки WiFi точки доступа
const char* ap_ssid = "RobotNavigator";
const char* ap_password = "12345678";
IPAddress ap_ip(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Объекты
Adafruit_MPU6050 mpu;
WebServer server(80);
Servo lifter;

// Система позиционирования
float currentX = 0.0;
float currentY = 0.0;
float currentAngle = 0.0;  // Угол в градусах от начального направления
bool isLifterUp = false;

// Параметры движения
const int BASE_SPEED = 200;
const float SPEED_TO_CM_PER_SEC = 35.0;  // Скорость в см/сек при BASE_SPEED

// Калибровка поворотов
float TURN_TIME_PER_DEGREE_FAST = 6.3;
float TURN_TIME_PER_DEGREE_MED = 0.7;
float TURN_TIME_PER_DEGREE_SLOW = 0.4;

// Калибровка гироскопа
float gyroOffsetZ = 0.0;
unsigned long lastGyroTime = 0;

// История маршрута для возврата домой
struct RoutePoint {
  float x, y, angle;
  String action;
  float value;
  RoutePoint(float _x, float _y, float _ang, String _act, float _val) 
    : x(_x), y(_y), angle(_ang), action(_act), value(_val) {}
};

std::vector<RoutePoint> routeHistory;
const int MAX_HISTORY = 1000;  // Увеличено для длинных последовательностей

// ========== HTTP ОБРАБОТЧИКИ ==========

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Robot Navigator - Advanced Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 1400px; margin: 0 auto; }
        .header { 
            background: rgba(255,255,255,0.95); 
            padding: 20px; 
            border-radius: 15px; 
            margin-bottom: 20px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
        }
        .main-grid { 
            display: grid; 
            grid-template-columns: 1fr 1fr; 
            gap: 20px; 
            margin-bottom: 20px;
        }
        .panel { 
            background: rgba(255,255,255,0.95); 
            padding: 20px; 
            border-radius: 15px;
            box-shadow: 0 8px 32px rgba(0,0,0,0.1);
        }
        .status-grid { 
            display: grid; 
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr)); 
            gap: 15px; 
        }
        .status-item { 
            background: linear-gradient(45deg, #f093fb 0%, #f5576c 100%);
            color: white; 
            padding: 15px; 
            border-radius: 10px;
            text-align: center;
        }
        .status-value { font-size: 24px; font-weight: bold; }
        .status-label { font-size: 12px; opacity: 0.9; }
        .robot-canvas { 
            width: 100%; 
            height: 300px; 
            border: 2px solid #ddd; 
            border-radius: 10px; 
            background: white;
            margin: 15px 0;
        }
        .controls-section { margin: 15px 0; }
        textarea { 
            width: 100%; 
            height: 120px; 
            margin: 10px 0; 
            padding: 10px;
            border: 2px solid #ddd;
            border-radius: 8px;
            font-family: 'Courier New', monospace;
        }
        button { 
            background: linear-gradient(45deg, #667eea 0%, #764ba2 100%);
            color: white; 
            padding: 12px 24px; 
            border: none; 
            border-radius: 8px; 
            cursor: pointer;
            font-weight: bold;
            margin: 5px;
            transition: transform 0.2s;
        }
        button:hover { transform: translateY(-2px); }
        .quick-btn { 
            background: linear-gradient(45deg, #28a745 0%, #20c997 100%);
            margin: 5px;
        }
        .debug-console { 
            background: #1e1e1e; 
            color: #00ff00; 
            padding: 15px; 
            border-radius: 8px; 
            height: 200px; 
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            margin: 15px 0;
        }
        .coordinate-input { 
            display: grid; 
            grid-template-columns: 1fr 1fr auto; 
            gap: 10px; 
            margin: 10px 0;
        }
        input { 
            padding: 10px; 
            border: 2px solid #ddd; 
            border-radius: 8px;
            font-size: 14px;
        }
        .route-history { 
            max-height: 150px; 
            overflow-y: auto; 
            background: #f8f9fa;
            padding: 10px;
            border-radius: 8px;
            margin: 10px 0;
        }
        .history-item { 
            padding: 5px; 
            margin: 2px 0; 
            background: white;
            border-radius: 4px;
            font-size: 12px;
        }
        @media (max-width: 768px) {
            .main-grid { grid-template-columns: 1fr; }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>Robot Navigator - Advanced Control Panel</h1>
        </div>
        
        <div class="main-grid">
            <!-- Status Panel -->
            <div class="panel">
                <h2>Robot Status</h2>
                <div class="status-grid">
                    <div class="status-item">
                        <div class="status-value" id="posX">0.00</div>
                        <div class="status-label">X Position (m)</div>
                    </div>
                    <div class="status-item">
                        <div class="status-value" id="posY">0.00</div>
                        <div class="status-label">Y Position (m)</div>
                    </div>
                    <div class="status-item">
                        <div class="status-value" id="angle">0.0°</div>
                        <div class="status-label">Angle</div>
                    </div>
                    <div class="status-item">
                        <div class="status-value" id="lifter">DOWN</div>
                        <div class="status-label">Lifter</div>
                    </div>
                </div>
                
                <h3>Robot Visualization</h3>
                <canvas id="robotCanvas" class="robot-canvas"></canvas>
                
                <h3>Route History</h3>
                <div id="routeHistory" class="route-history">
                    <div class="history-item">No route history yet</div>
                </div>
            </div>
            
            <!-- Control Panel -->
            <div class="panel">
                <h2>Robot Control</h2>
                
                <div class="controls-section">
                    <h3>Go to Coordinates</h3>
                    <div class="coordinate-input">
                        <input type=\"number\" id=\"targetX\" placeholder=\"X (meters)\" step=\"0.1\">
                        <input type=\"number\" id=\"targetY\" placeholder=\"Y (meters)\" step=\"0.1\">
                        <button onclick=\"goToCoordinates()\">Go</button>
                    </div>
                </div>
                
                <div class=\"controls-section\">
                    <h3>Command Sequence</h3>
                    <form id=\"commandForm\">
                        <textarea id=\"commandInput\" placeholder=\"Enter commands (one per line):\\nGO_LOCAL x y\\nUP\\nDOWN\\nDELAY t\\nGO_LOCAL 0 0\\n\\nExample:\\nGO_LOCAL 2.5 1.8\\nUP\\nGO_LOCAL 3.2 4.1\\nDOWN\\nGO_LOCAL 0 0\"></textarea>
                        <button type=\"submit\">Execute Sequence</button>
                    </form>
                </div>
                
                <div class=\"controls-section\">
                    <h3>Quick Actions</h3>
                    <button class=\"quick-btn\" onclick=\"sendCommand('GO_LOCAL 1 1')\">Go to (1,1)</button>
                    <button class=\"quick-btn\" onclick=\"sendCommand('GO_LOCAL 0 0')\">Return Home</button>
                    <button class=\"quick-btn\" onclick=\"sendCommand('UP')\">Lifter Up</button>
                    <button class=\"quick-btn\" onclick=\"sendCommand('DOWN')\">Lifter Down</button>
                    <button class=\"quick-btn\" onclick=\"resetPosition()\">Reset Position</button>
                </div>
            </div>
        </div>
        
        <!-- Debug Console -->
        <div class=\"panel\">
            <h2>Debug Console (Serial Monitor)</h2>
            <div id=\"debugConsole\" class=\"debug-console\">
                Connecting to robot debug stream...
            </div>
        </div>
    </div>
    
    <script>
        let robotX = 0, robotY = 0, robotAngle = 0;
        let routePoints = [];
        let debugMessages = [];
        
        // Initialize canvas
        const canvas = document.getElementById(\"robotCanvas\");
        const ctx = canvas.getContext(\"2d\");
        
        function resizeCanvas() {
            canvas.width = canvas.offsetWidth;
            canvas.height = canvas.offsetHeight;
            drawRobot();
        }
        
        window.addEventListener('resize', resizeCanvas);
        resizeCanvas();
        
        function drawRobot() {
            ctx.clearRect(0, 0, canvas.width, canvas.height);
            
            // Draw grid
            ctx.strokeStyle = '#e0e0e0';
            ctx.lineWidth = 1;
            for (let i = 0; i <= canvas.width; i += 30) {
                ctx.beginPath();
                ctx.moveTo(i, 0);
                ctx.lineTo(i, canvas.height);
                ctx.stroke();
            }
            for (let i = 0; i <= canvas.height; i += 30) {
                ctx.beginPath();
                ctx.moveTo(0, i);
                ctx.lineTo(canvas.width, i);
                ctx.stroke();
            }
            
            // Draw route history
            if (routePoints.length > 1) {
                ctx.strokeStyle = '#007bff';
                ctx.lineWidth = 2;
                ctx.beginPath();
                for (let i = 0; i < routePoints.length; i++) {
                    const x = canvas.width/2 + routePoints[i].x * 50;
                    const y = canvas.height/2 - routePoints[i].y * 50;
                    if (i === 0) {
                        ctx.moveTo(x, y);
                    } else {
                        ctx.lineTo(x, y);
                    }
                }
                ctx.stroke();
            }
            
            // Draw robot
            const robotScreenX = canvas.width/2 + robotX * 50;
            const robotScreenY = canvas.height/2 - robotY * 50;
            
            ctx.save();
            ctx.translate(robotScreenX, robotScreenY);
            ctx.rotate(-robotAngle * Math.PI / 180);
            
            // Robot body
            ctx.fillStyle = '#ff6b6b';
            ctx.fillRect(-15, -10, 30, 20);
            
            // Direction indicator
            ctx.fillStyle = '#4ecdc4';
            ctx.beginPath();
            ctx.moveTo(15, 0);
            ctx.lineTo(25, -5);
            ctx.lineTo(25, 5);
            ctx.closePath();
            ctx.fill();
            
            ctx.restore();
            
            // Draw origin
            ctx.fillStyle = '#28a745';
            ctx.beginPath();
            ctx.arc(canvas.width/2, canvas.height/2, 5, 0, 2 * Math.PI);
            ctx.fill();
        }
        
        function updateStatus() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    robotX = data.x;
                    robotY = data.y;
                    robotAngle = data.angle;
                    
                    document.getElementById('posX').textContent = data.x.toFixed(2);
                    document.getElementById('posY').textContent = data.y.toFixed(2);
                    document.getElementById('angle').textContent = data.angle.toFixed(1) + '°';
                    document.getElementById('lifter').textContent = data.lifter;
                    
                    // Update route history
                    updateRouteHistory(data.history_size);
                    
                    drawRobot();
                })
                .catch(err => {
                    addDebugMessage('Status update failed: ' + err.message);
                });
        }
        
        function updateRouteHistory(size) {
            // Simulate route points based on current position
            if (routePoints.length === 0 || 
                Math.abs(routePoints[routePoints.length - 1].x - robotX) > 0.1 ||
                Math.abs(routePoints[routePoints.length - 1].y - robotY) > 0.1) {
                routePoints.push({x: robotX, y: robotY});
                if (routePoints.length > 100) routePoints.shift();
            }
            
            const historyDiv = document.getElementById(\"routeHistory\");
            if (routePoints.length > 0) {
                const lastPoint = routePoints[routePoints.length - 1];
                historyDiv.innerHTML = \"<div class='history-item'><strong>Current:</strong> X=\" + lastPoint.x.toFixed(2) + \", Y=\" + lastPoint.y.toFixed(2) + \"</div><div class='history-item'><strong>Total points:</strong> \" + size + \"</div>\";
            }
        }
        
        function sendCommand(cmd) {
            addDebugMessage('Sending: ' + cmd);
            fetch('/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: 'commands=' + encodeURIComponent(cmd)
            })
            .then(response => response.text())
            .then(data => {
                addDebugMessage('✅ Command sent successfully');
                setTimeout(updateStatus, 500);
            })
            .catch(err => {
                addDebugMessage('❌ Command failed: ' + err.message);
            });
        }
        
        function goToCoordinates() {
            const x = parseFloat(document.getElementById(\"targetX\").value);
            const y = parseFloat(document.getElementById(\"targetY\").value);
            if (!isNaN(x) && !isNaN(y)) {
                sendCommand('GO_LOCAL ' + x + ' ' + y);
            } else {
                addDebugMessage('Invalid coordinates');
            }
        }
        
        function resetPosition() {
            if (confirm('Reset robot position to origin?')) {
                sendCommand('RESET_POS');
                routePoints = [{x: 0, y: 0}];
                drawRobot();
            }
        }
        
        function addDebugMessage(message) {
            const timestamp = new Date().toLocaleTimeString();
            const debugConsole = document.getElementById(\"debugConsole\");
            debugConsole.innerHTML += \"<div>[\" + timestamp + \"] \" + message + \"</div>\";
            debugConsole.scrollTop = debugConsole.scrollHeight;
            
            // Keep only last 100 messages
            const messages = debugConsole.children;
            if (messages.length > 100) {
                debugConsole.removeChild(messages[0]);
            }
        }
        
        // Form submission
        document.getElementById(\"commandForm\").addEventListener(\"submit\", function(e) {
            e.preventDefault();
            const commands = document.getElementById(\"commandInput\").value;
            if (commands.trim()) {
                addDebugMessage('Sending command sequence...');
                sendCommand(commands);
            }
        });
        
        // Simulate debug messages
        function simulateDebugMessages() {
            const messages = [
                'Robot system ready',
                'WiFi connected',
                'Gyroscope calibrated',
                'Motors initialized',
                'Position tracking active'
            ];
            
            let i = 0;
            setInterval(function() {
                if (Math.random() > 0.7) {
                    addDebugMessage(messages[i % messages.length]);
                    i++;
                }
            }, 5000);
        }
        
        // Initialize
        updateStatus();
        setInterval(updateStatus, 2000);
        simulateDebugMessages();
        addDebugMessage('Robot Navigator Interface Loaded');
        addDebugMessage('Connected to robot at 192.168.4.1');
    </script>
</body>
</html>
)";
  server.send(200, "text/html", html);
}

void handleCommand() {
  if (server.hasArg("commands")) {
    String commands = server.arg("commands");
    Serial.println("📥 Received commands:");
    Serial.println(commands);
    
    server.send(200, "text/plain", "Commands received and executing");
    executeCommandSequence(commands);
  } else {
    server.send(400, "text/plain", "Missing commands parameter");
  }
}

void handleRoute() {
  if (server.hasArg("route")) {
    String routeJson = server.arg("route");
    Serial.println("📍 Received route JSON:");
    Serial.println(routeJson);
    
    // Парсим JSON маршрут и выполняем последовательно
    executeRouteFromJson(routeJson);
    
    server.send(200, "application/json", "{\"ok\": true, \"message\": \"Route received\"}");
  } else {
    server.send(400, "application/json", "{\"error\": \"Missing route parameter\"}");
  }
}

void handleStatus() {
  String json = "{";
  json += "\"x\":" + String(currentX, 3) + ",";
  json += "\"y\":" + String(currentY, 3) + ",";
  json += "\"angle\":" + String(currentAngle, 1) + ",";
  json += "\"lifter\":\"" + String(isLifterUp ? "UP" : "DOWN") + "\",";
  json += "\"history_size\":" + String(routeHistory.size());
  json += "}";
  server.send(200, "application/json", json);
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ========== ОСНОВНЫЕ ФУНКЦИИ ==========

void setup() {
  Serial.begin(115200);
  Serial.println("🚀 Robot Navigator Starting...");
  
  // Инициализация пинов моторов
  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(SPEED_1, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  
  // Инициализация сервопривода
  lifter.attach(SERVO_PIN);
  lifter.write(0);  // Начальное положение - DOWN
  isLifterUp = false;
  
  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("❌ Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  // Калибровка гироскопа
  calibrateGyroscope();
  
  // Настройка WiFi точки доступа
  setupWiFi();
  
  // Настройка HTTP сервера
  server.on("/", handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.on("/route", HTTP_POST, handleRoute);
  server.on("/status", handleStatus);
  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("✅ Robot Navigator Ready!");
  Serial.println("🌐 Connect to WiFi: RobotNavigator");
  Serial.println("🔑 Password: 12345678");
  Serial.println("📡 IP: 192.168.4.1");
}

void loop() {
  server.handleClient();
  updateGyroAngle();
}

// ========== WIFI НАСТРОЙКА ==========

void setupWiFi() {
  Serial.println("📡 Setting up WiFi Access Point...");
  
  WiFi.disconnect(true, true);
  delay(500);
  WiFi.mode(WIFI_AP);
  delay(500);
  
  bool success = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);
  
  if (success) {
    WiFi.softAPConfig(ap_ip, ap_ip, subnet);
    delay(2000);
    
    Serial.println("✅ WiFi Access Point created successfully!");
    Serial.print("📡 SSID: ");
    Serial.println(ap_ssid);
    Serial.print("🌐 IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("❌ Failed to create WiFi Access Point");
  }
}

// ========== КАЛИБРОВКА И ГИРОСКОП ==========

void calibrateGyroscope() {
  Serial.println("🔧 Calibrating gyroscope...");
  
  float sumZ = 0;
  int successfulReads = 0;
  
  for(int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      sumZ += g.gyro.z;
      successfulReads++;
    }
    delay(5);
    
    // Feed watchdog during calibration
    #ifdef ESP32
    esp_task_wdt_reset();
    #endif
  }
  
  if (successfulReads > 0) {
    gyroOffsetZ = sumZ / successfulReads;
    Serial.println("✅ Gyroscope calibrated successfully");
  } else {
    Serial.println("⚠️ Gyroscope calibration failed, using default offset");
    gyroOffsetZ = 0.0;
  }
  
  lastGyroTime = millis();
}

void updateGyroAngle() {
  unsigned long now = millis();
  float dt = (now - lastGyroTime) / 1000.0;
  lastGyroTime = now;
  
  if (dt > 0.5) return;  // Пропускаем слишком большие интервалы
  
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    return;  // Пропускаем при ошибке чтения
  }
  
  float gz = (g.gyro.z - gyroOffsetZ) * 180.0 / PI;
  if (abs(gz) < 1.0) gz = 0;  // Фильтруем шум
  
  currentAngle += gz * dt;
  
  // Нормализуем угол в диапазон [0, 360)
  while (currentAngle >= 360.0) currentAngle -= 360.0;
  while (currentAngle < 0) currentAngle += 360.0;
}

// ========== ВЫПОЛНЕНИЕ КОМАНД ==========

void executeCommandSequence(String commands) {
  int start = 0;
  int end = commands.indexOf('\n');
  
  while (end != -1) {
    String line = commands.substring(start, end);
    line.trim();
    if (line.length() > 0) {
      executeSingleCommand(line);
    }
    start = end + 1;
    end = commands.indexOf('\n', start);
  }
  
  // Обрабатываем последнюю строку
  String line = commands.substring(start);
  line.trim();
  if (line.length() > 0) {
    executeSingleCommand(line);
  }
}

void executeSingleCommand(String cmd) {
  cmd.toUpperCase();
  cmd.trim();
  
  Serial.print("🎯 Executing: ");
  Serial.println(cmd);
  
  if (cmd.startsWith("GO_LOCAL")) {
    // Парсим координаты GO_LOCAL x y
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);
    
    if (secondSpace != -1) {
      float x = cmd.substring(firstSpace + 1, secondSpace).toFloat();
      float y = cmd.substring(secondSpace + 1).toFloat();
      goToCoordinates(x, y);
    } else {
      Serial.println("❌ Invalid GO_LOCAL format. Use: GO_LOCAL x y");
    }
  }
  else if (cmd == "UP") {
    lifterUp();
  }
  else if (cmd == "DOWN") {
    lifterDown();
  }
  else if (cmd == "RESET_POS") {
    resetPosition();
  }
  else if (cmd.startsWith("DELAY")) {
    int space = cmd.indexOf(' ');
    if (space != -1) {
      int t = cmd.substring(space + 1).toInt();
      Serial.print("⏱️ Delay: ");
      Serial.print(t);
      Serial.println("ms");
      delay(t);
    }
  }
  else {
    Serial.print("❌ Unknown command: ");
    Serial.println(cmd);
  }
}

// ========== НАВИГАЦИЯ И ДВИЖЕНИЕ ==========

void goToCoordinates(float targetX, float targetY) {
  Serial.print("🎯 Navigating to: X=");
  Serial.print(targetX, 2);
  Serial.print("m, Y=");
  Serial.print(targetY, 2);
  Serial.println("m");
  
  // Вычисляем разницу между текущей и целевой позицией
  float dx = targetX - currentX;
  float dy = targetY - currentY;
  float distance = sqrt(dx*dx + dy*dy);
  
  // Если уже в точке, ничего не делаем
  if (distance < 0.05) {  // 5cm tolerance
    Serial.println("✅ Already at target position");
    return;
  }
  
  // Вычисляем целевой угол
  float targetAngle = atan2(dy, dx) * 180.0 / PI;
  if (targetAngle < 0) targetAngle += 360.0;
  
  // Вычисляем разницу углов
  float angleDiff = targetAngle - currentAngle;
  
  // Нормализуем разницу углов в диапазон (-180, 180]
  while (angleDiff > 180.0) angleDiff -= 360.0;
  while (angleDiff <= -180.0) angleDiff += 360.0;
  
  Serial.print("📍 Current: X=");
  Serial.print(currentX, 2);
  Serial.print(", Y=");
  Serial.print(currentY, 2);
  Serial.print(", Angle=");
  Serial.print(currentAngle, 1);
  Serial.println("°");
  
  Serial.print("🔄 Turn angle: ");
  Serial.print(angleDiff, 1);
  Serial.print("°, Distance: ");
  Serial.print(distance, 2);
  Serial.println("m");
  
  // Поворачиваем к цели
  if (abs(angleDiff) > 3.0) {
    if (angleDiff > 0) {
      Serial.println("⬅️ Turning left");
      turnLeft(angleDiff);
    } else {
      Serial.println("➡️ Turning right");
      turnRight(-angleDiff);
    }
    delay(500);  // Пауза после поворота
  }
  
  // Движемся к цели
  Serial.println("⬆️ Moving forward");
  moveForward(distance);
  
  Serial.println("✅ Navigation complete");
}

void moveForward(float distanceInMeters) {
  float distanceInCm = distanceInMeters * 100.0;
  float timeNeeded = distanceInCm / SPEED_TO_CM_PER_SEC;
  unsigned long duration = timeNeeded * 1000;
  
  // Сохраняем точку маршрута
  routeHistory.push_back(RoutePoint(currentX, currentY, currentAngle, "forward", distanceInMeters));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
  
  Serial.print("🚗 Moving forward ");
  Serial.print(distanceInMeters, 2);
  Serial.print("m (");
  Serial.print(duration);
  Serial.println("ms)");
  
  // Запускаем моторы
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  unsigned long startTime = millis();
  
  while (millis() - startTime < duration) {
    updateGyroAngle();
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    esp_task_wdt_reset();
    #endif
    server.handleClient();
    
    delay(10);
  }
  
  stopMotors();
  
  // Обновляем позицию
  float rad = currentAngle * PI / 180.0;
  currentX += distanceInMeters * cos(rad);
  currentY += distanceInMeters * sin(rad);
  
  Serial.print("📍 New position: X=");
  Serial.print(currentX, 2);
  Serial.print(", Y=");
  Serial.print(currentY, 2);
  Serial.println("m");
}

void turnLeft(float degrees) {
  float multiplier = getTurnMultiplier(degrees);
  unsigned long turnTime = degrees * multiplier;
  
  // Сохраняем точку маршрута
  routeHistory.push_back(RoutePoint(currentX, currentY, currentAngle, "left", degrees));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
  
  Serial.print("🔄 Turning left ");
  Serial.print(degrees, 1);
  Serial.print("° (");
  Serial.print(turnTime);
  Serial.println("ms)");
  
  // Запускаем поворот
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  unsigned long startTime = millis();
  while (millis() - startTime < turnTime) {
    updateGyroAngle();
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    esp_task_wdt_reset();
    #endif
    server.handleClient();
    
    delay(5);
  }
  
  stopMotors();
  delay(50);
  
  // Обновляем угол
  currentAngle += degrees;
  while (currentAngle >= 360.0) currentAngle -= 360.0;
  
  Serial.print("📍 New angle: ");
  Serial.print(currentAngle, 1);
  Serial.println("°");
}

void turnRight(float degrees) {
  float multiplier = getTurnMultiplier(degrees);
  unsigned long turnTime = degrees * multiplier;
  
  // Сохраняем точку маршрута
  routeHistory.push_back(RoutePoint(currentX, currentY, currentAngle, "right", degrees));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
  
  Serial.print("🔄 Turning right ");
  Serial.print(degrees, 1);
  Serial.print("° (");
  Serial.print(turnTime);
  Serial.println("ms)");
  
  // Запускаем поворот
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  unsigned long startTime = millis();
  while (millis() - startTime < turnTime) {
    updateGyroAngle();
    
    // Feed watchdog and handle WiFi
    #ifdef ESP32
    esp_task_wdt_reset();
    #endif
    server.handleClient();
    
    delay(5);
  }
  
  stopMotors();
  delay(50);
  
  // Обновляем угол
  currentAngle -= degrees;
  while (currentAngle < 0) currentAngle += 360.0;
  
  Serial.print("📍 New angle: ");
  Serial.print(currentAngle, 1);
  Serial.println("°");
}

float getTurnMultiplier(float angle) {
  if (abs(angle) < 15) {
    return TURN_TIME_PER_DEGREE_SLOW;
  } else if (abs(angle) < 45) {
    return TURN_TIME_PER_DEGREE_MED;
  } else {
    return TURN_TIME_PER_DEGREE_FAST;
  }
}

// ========== УПРАВЛЕНИЕ ПОДЪЕМНИКОМ ==========

void lifterUp() {
  Serial.println("⬆️ Lifter UP");
  lifter.write(180);
  isLifterUp = true;
  delay(500);
  
  routeHistory.push_back(RoutePoint(currentX, currentY, currentAngle, "up", 0));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
}

void lifterDown() {
  Serial.println("⬇️ Lifter DOWN");
  lifter.write(0);
  isLifterUp = false;
  delay(500);
  
  routeHistory.push_back(RoutePoint(currentX, currentY, currentAngle, "down", 0));
  if (routeHistory.size() > MAX_HISTORY) {
    routeHistory.erase(routeHistory.begin());
  }
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========

void stopMotors() {
  analogWrite(SPEED_1, 0);
  analogWrite(SPEED_2, 0);
  delay(100);
}

void resetPosition() {
  Serial.println("Resetting position to origin");
  currentX = 0.0;
  currentY = 0.0;
  currentAngle = 0.0;
  routeHistory.clear();
  calibrateGyroscope();
}

void executeRouteFromJson(String routeJson) {
  Serial.println("Executing route from JSON");
  
  // Простая парсилка JSON для маршрута
  // Ожидаемый формат: [{"i": x, "j": y}, {"i": x, "j": y}, ...]
  
  int start = 0;
  int end = routeJson.indexOf('{', start);
  
  while (end != -1) {
    // Находим конец объекта
    int braceCount = 0;
    int objEnd = end;
    
    for (int i = end; i < routeJson.length(); i++) {
      if (routeJson.charAt(i) == '{') braceCount++;
      if (routeJson.charAt(i) == '}') {
        braceCount--;
        if (braceCount == 0) {
          objEnd = i;
          break;
        }
      }
    }
    
    if (objEnd > end) {
      String jsonObj = routeJson.substring(end, objEnd + 1);
      parseAndExecuteRoutePoint(jsonObj);
    }
    
    start = objEnd + 1;
    end = routeJson.indexOf('{', start);
  }
  
  Serial.println("Route execution completed");
}

void parseAndExecuteRoutePoint(String jsonObj) {
  // Парсим координаты из JSON объекта
  // {"i": x, "j": y}
  
  float x = 0.0, y = 0.0;
  
  int iIndex = jsonObj.indexOf("\"i\":");
  int jIndex = jsonObj.indexOf("\"j\":");
  
  if (iIndex != -1) {
    int valueStart = iIndex + 4;
    int valueEnd = jsonObj.indexOf(',', valueStart);
    if (valueEnd == -1) valueEnd = jsonObj.indexOf('}', valueStart);
    
    if (valueEnd != -1) {
      String value = jsonObj.substring(valueStart, valueEnd);
      value.trim();
      x = value.toFloat();
    }
  }
  
  if (jIndex != -1) {
    int valueStart = jIndex + 4;
    int valueEnd = jsonObj.indexOf(',', valueStart);
    if (valueEnd == -1) valueEnd = jsonObj.indexOf('}', valueStart);
    
    if (valueEnd != -1) {
      String value = jsonObj.substring(valueStart, valueEnd);
      value.trim();
      y = value.toFloat();
    }
  }
  
  if (x != 0.0 || y != 0.0) {
    Serial.print("Going to route point: X=");
    Serial.print(x, 2);
    Serial.print(", Y=");
    Serial.println(y, 2);
    
    goToCoordinates(x, y);
    delay(1000); // Пауза между точками маршрута
  }
}
