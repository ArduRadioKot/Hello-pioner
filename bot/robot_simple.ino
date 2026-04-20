#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Пины управления моторами
#define DIR_1 27 
#define DIR_2 32 
#define SPEED_1 26
#define SPEED_2 33

// Объекты
Adafruit_MPU6050 mpu;

// Настройки WiFi точки доступа
const char* ap_ssid = "RobotSimple";
const char* ap_password = "12345678";
IPAddress ap_ip(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Объекты
WebServer server(80);

// Параметры движения
const int BASE_SPEED = 200;
const float SPEED_TO_CM_PER_SEC = 35.0;  // Скорость в см/сек при BASE_SPEED

// Калибровка поворотов
float TURN_TIME_PER_DEGREE = 6.0;

// Калибровка гироскопа
float gyroOffsetZ = 0.0;
unsigned long lastGyroTime = 0;

// Переменные стабилизации
float currentAngle = 0.0;
float targetAngle = 0.0;
bool isTurning = false;

// ========== HTTP ОБРАБОТЧИКИ ==========

void handleRoot() {
  String html = R"(
<!DOCTYPE html>
<html>
<head>
    <title>Simple Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body { 
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; 
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
        }
        .container { 
            background: rgba(255, 255, 255, 0.95); 
            border-radius: 20px; 
            box-shadow: 0 20px 40px rgba(0, 0, 0, 0.1);
            padding: 30px;
            width: 90%;
            max-width: 500px;
        }
        h1 { 
            text-align: center; 
            color: #333; 
            margin-bottom: 30px;
            font-size: 2em;
        }
        .control-group { 
            margin-bottom: 25px; 
        }
        label { 
            display: block; 
            margin-bottom: 8px; 
            color: #555; 
            font-weight: 600; 
        }
        input { 
            width: 100%; 
            padding: 12px; 
            border: 2px solid #e1e1e1; 
            border-radius: 10px; 
            font-size: 16px;
            margin-bottom: 10px;
        }
        button { 
            background: linear-gradient(45deg, #667eea, #764ba2);
            color: white; 
            padding: 12px 24px; 
            border: none; 
            border-radius: 10px; 
            cursor: pointer;
            font-weight: bold;
            margin: 5px;
            width: 100%;
            font-size: 16px;
        }
        button:hover { 
            transform: translateY(-2px); 
            box-shadow: 0 10px 20px rgba(102, 126, 234, 0.3);
        }
        .direction-controls { 
            display: grid; 
            grid-template-columns: 1fr 1fr 1fr; 
            gap: 10px; 
            margin-bottom: 20px;
        }
        .status { 
            background: #f8f9fa; 
            border-radius: 10px; 
            padding: 15px; 
            margin-bottom: 20px;
            text-align: center;
        }
        .log { 
            background: #1e1e1e; 
            color: #00ff00; 
            border-radius: 10px; 
            padding: 15px; 
            height: 150px; 
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 12px;
            margin-top: 20px;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>Simple Robot Control</h1>
        
        <div class="status">
            <strong>Status:</strong> <span id="status">Ready</span>
        </div>
        
        <div class="control-group">
            <label>Distance (cm):</label>
            <input type="number" id="distance" placeholder="Enter distance in cm" value="50">
        </div>
        
        <div class="control-group">
            <label>Turn Angle (degrees):</label>
            <input type="number" id="angle" placeholder="Enter angle in degrees" value="90">
        </div>
        
        <div class="direction-controls">
            <button onclick="moveForward()">Forward</button>
            <button onclick="moveBackward()">Backward</button>
            <button onclick="stopMotors()">Stop</button>
            <button onclick="turnLeft()">Turn Left</button>
            <button onclick="turnRight()">Turn Right</button>
            <button onclick="moveDistance()">Move Distance</button>
        </div>
        
        <div class="log" id="log">
            Robot control ready...
        </div>
    </div>
    
    <script>
        function addLog(message) {
            const log = document.getElementById("log");
            const timestamp = new Date().toLocaleTimeString();
            log.innerHTML += "<div>[" + timestamp + "] " + message + "</div>";
            log.scrollTop = log.scrollHeight;
        }
        
        function updateStatus(status) {
            document.getElementById("status").textContent = status;
        }
        
        function sendCommand(action, value = 0) {
            addLog("Sending: " + action + " " + value);
            updateStatus("Moving...");
            
            fetch('/command', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: "action=" + action + "&value=" + value
            })
            .then(response => response.text())
            .then(data => {
                addLog("Response: " + data);
                updateStatus("Ready");
            })
            .catch(err => {
                addLog("Error: " + err.message);
                updateStatus("Error");
            });
        }
        
        function moveForward() {
            sendCommand("forward");
        }
        
        function moveBackward() {
            sendCommand("backward");
        }
        
        function turnLeft() {
            const angle = document.getElementById("angle").value || 90;
            sendCommand("left", angle);
        }
        
        function turnRight() {
            const angle = document.getElementById("angle").value || 90;
            sendCommand("right", angle);
        }
        
        function stopMotors() {
            sendCommand("stop");
        }
        
        function moveDistance() {
            const distance = document.getElementById("distance").value || 50;
            sendCommand("move_distance", distance);
        }
        
        // Keyboard controls
        document.addEventListener("keydown", function(e) {
            switch(e.key) {
                case "ArrowUp":
                case "w":
                case "W":
                    moveForward();
                    break;
                case "ArrowDown":
                case "s":
                case "S":
                    moveBackward();
                    break;
                case "ArrowLeft":
                case "a":
                case "A":
                    turnLeft();
                    break;
                case "ArrowRight":
                case "d":
                case "D":
                    turnRight();
                    break;
                case " ":
                    stopMotors();
                    break;
            }
        });
        
        addLog("Simple Robot Control Loaded");
        addLog("Use arrow keys or WASD to control");
    </script>
</body>
</html>
)";
  server.send(200, "text/html", html);
}

void handleCommand() {
  if (server.hasArg("action")) {
    String action = server.arg("action");
    float value = server.arg("value").toFloat();
    
    Serial.print("Command: ");
    Serial.print(action);
    if (value > 0) {
      Serial.print(" Value: ");
      Serial.println(value);
    } else {
      Serial.println();
    }
    
    if (action == "forward") {
      moveForwardContinuous();
    } else if (action == "backward") {
      moveBackwardContinuous();
    } else if (action == "left") {
      turnLeft(value);
    } else if (action == "right") {
      turnRight(value);
    } else if (action == "stop") {
      stopMotors();
    } else if (action == "move_distance") {
      moveForwardDistance(value);
    }
    
    server.send(200, "text/plain", "Command executed");
  } else {
    server.send(400, "text/plain", "Missing action parameter");
  }
}

void handleNotFound() {
  server.send(404, "text/plain", "Not found");
}

// ========== ГИРОСКОП И СТАБИЛИЗАЦИЯ ==========

void calibrateGyroscope() {
  Serial.println("Calibrating gyroscope...");
  
  float sumZ = 0;
  for(int i = 0; i < 100; i++) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      sumZ += g.gyro.z;
    }
    delay(5);
  }
  
  gyroOffsetZ = sumZ / 100.0;
  lastGyroTime = millis();
  currentAngle = 0.0;
  targetAngle = 0.0;
  
  Serial.println("Gyroscope calibrated");
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

void stabilizeMotors() {
  if (isTurning) return;  // Не стабилизируем во время поворота
  
  float angleError = targetAngle - currentAngle;
  
  // Нормализуем ошибку в диапазон (-180, 180]
  while (angleError > 180.0) angleError -= 360.0;
  while (angleError <= -180.0) angleError += 360.0;
  
  // Если ошибка меньше 3 градусов, ничего не делаем
  if (abs(angleError) < 3.0) {
    stopMotors();
    return;
  }
  
  // Корректируем движение
  if (angleError > 0) {
    // Нужно повернуть вправо
    digitalWrite(DIR_1, HIGH);
    digitalWrite(DIR_2, LOW);
    analogWrite(SPEED_1, BASE_SPEED / 2);  // Половинная скорость для коррекции
    analogWrite(SPEED_2, BASE_SPEED / 2);
  } else {
    // Нужно повернуть влево
    digitalWrite(DIR_1, LOW);
    digitalWrite(DIR_2, HIGH);
    analogWrite(SPEED_1, BASE_SPEED / 2);
    analogWrite(SPEED_2, BASE_SPEED / 2);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Simple Robot Starting...");
  
  // Инициализация пинов моторов
  pinMode(DIR_1, OUTPUT);
  pinMode(DIR_2, OUTPUT);
  pinMode(SPEED_1, OUTPUT);
  pinMode(SPEED_2, OUTPUT);
  
  // Инициализация MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) delay(10);
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  
  // Калибровка гироскопа
  calibrateGyroscope();
  
  // Настройка WiFi точки доступа
  WiFi.disconnect(true, true);
  delay(500);
  WiFi.mode(WIFI_AP);
  delay(500);
  
  bool success = WiFi.softAP(ap_ssid, ap_password, 1, 0, 4);
  
  if (success) {
    WiFi.softAPConfig(ap_ip, ap_ip, subnet);
    delay(2000);
    
    Serial.println("WiFi Access Point created successfully!");
    Serial.print("SSID: ");
    Serial.println(ap_ssid);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("Failed to create WiFi Access Point");
  }
  
  // Настройка HTTP сервера
  server.on("/", handleRoot);
  server.on("/command", HTTP_POST, handleCommand);
  server.onNotFound(handleNotFound);
  server.begin();
  
  Serial.println("Simple Robot Ready!");
  Serial.println("Connect to WiFi: SimpleRobot");
  Serial.println("Password: 12345678");
  Serial.println("IP: 192.168.4.1");
}

void loop() {
  server.handleClient();
  updateGyroAngle();
}

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========

void moveForwardContinuous() {
  Serial.println("Moving forward continuously");
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
}

void moveBackwardContinuous() {
  Serial.println("Moving backward continuously");
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
}

void moveForwardDistance(float distanceCm) {
  Serial.print("Moving forward ");
  Serial.print(distanceCm);
  Serial.println(" cm");
  
  float timeNeeded = distanceCm / SPEED_TO_CM_PER_SEC;
  unsigned long duration = timeNeeded * 1000;
  unsigned long startTime = millis();
  
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  // Movement with stabilization
  while (millis() - startTime < duration) {
    updateGyroAngle();
    stabilizeMotors();
    delay(10);
  }
  
  stopMotors();
  Serial.println("Movement completed");
}

void turnLeft(float degrees) {
  Serial.print("Turning left ");
  Serial.print(degrees);
  Serial.println(" degrees");
  
  // Set target angle
  targetAngle = currentAngle + degrees;
  while (targetAngle >= 360.0) targetAngle -= 360.0;
  while (targetAngle < 0) targetAngle += 360.0;
  
  isTurning = true;
  unsigned long turnTime = degrees * TURN_TIME_PER_DEGREE;
  unsigned long startTime = millis();
  
  digitalWrite(DIR_1, LOW);
  digitalWrite(DIR_2, HIGH);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  // Turn with stabilization
  while (millis() - startTime < turnTime) {
    updateGyroAngle();
    delay(5);
  }
  
  isTurning = false;
  stopMotors();
  Serial.println("Turn completed");
}

void turnRight(float degrees) {
  Serial.print("Turning right ");
  Serial.print(degrees);
  Serial.println(" degrees");
  
  // Set target angle
  targetAngle = currentAngle - degrees;
  while (targetAngle >= 360.0) targetAngle -= 360.0;
  while (targetAngle < 0) targetAngle += 360.0;
  
  isTurning = true;
  unsigned long turnTime = degrees * TURN_TIME_PER_DEGREE;
  unsigned long startTime = millis();
  
  digitalWrite(DIR_1, HIGH);
  digitalWrite(DIR_2, LOW);
  analogWrite(SPEED_1, BASE_SPEED);
  analogWrite(SPEED_2, BASE_SPEED);
  
  // Turn with stabilization
  while (millis() - startTime < turnTime) {
    updateGyroAngle();
    delay(5);
  }
  
  isTurning = false;
  stopMotors();
  Serial.println("Turn completed");
}

void stopMotors() {
  Serial.println("Stopping motors");
  analogWrite(SPEED_1, 0);
  analogWrite(SPEED_2, 0);
}
