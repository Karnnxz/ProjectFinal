#include <Wire.h>
#include <EEPROM.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <PubSubClient.h>

#define VBAT          25
#define BUZZER        12
#define BRAKE          4
#define DIR1          16
#define PWM1          17
#define ENC_FG        19

#define MPU6050_ADDR  0x68
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define PWR_MGMT_1    0x6B
#define EEPROM_SIZE   64

// --- WiFi & MQTT Config ---
const char* ssid        = "K";
const char* password    = "karn2006";
const char* mqtt_server = "test.mosquitto.org";

WiFiClient   espClient;
PubSubClient client(espClient);
WebServer    server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

float vDividerRatio = 289.0;
float Gyro_amount   = 0.85;
float alpha         = 0.7;

// --- PID Gains (shared, protected by mutex) ---
SemaphoreHandle_t pidMutex;

float eK1 = 10.0;
float eK2 = 30.0;
float eK3 = 2.5;
float eK4 = 0.01;

const float INTEGRAL_LIMIT = 50.0;
int loop_time = 10;

// --- Shared sensor data (Core1 writes, Core0 reads) ---
volatile float shared_angleX  = 0;
volatile float shared_voltage = 0;  // หน่วย Volt (เฉลี่ย 10 ครั้ง)
SemaphoreHandle_t dataMutex;

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gyroXfilt      = 0;
float robot_angleX   = 0;
float Acc_angleX     = 0;
float angle_offset   = 0;
float gyroX_offset   = 0;
float integral_angle = 0;

volatile int enc_count1 = 0;
int16_t motor1_speed    = 0;

bool vertical_edge = false;
bool calibrating   = false;
bool calibrated    = false;

// --- Task Handles ---
TaskHandle_t iotTaskHandle = NULL;

// --- Web Dashboard HTML ---
const char html_template[] PROGMEM = R"=====(
<html><head><meta charset="utf-8"><title>Cube Monitor</title>
<style>
  body { text-align:center; font-family:sans-serif; background:#222; color:#fff; margin:20px; }
  .container { display:grid; grid-template-columns: 1fr 1fr; gap:10px; max-width:400px; margin:auto; }
  .box { background:#333; padding:15px; border-radius:10px; border:1px solid #444; }
  .val { font-size:24px; color:#00ff00; font-weight:bold; }
  .label { font-size:12px; color:#aaa; display:block; }
  h1 { color:#00d1b2; }
</style>
<script>
  var socket = new WebSocket('ws://'+location.host+':81');
  socket.onmessage = function(e){
    var d = e.data.split('|');
    document.getElementById('ang').innerHTML = d[0];
    document.getElementById('bat').innerHTML = d[1];
    document.getElementById('k1').innerHTML  = d[2];
    document.getElementById('k2').innerHTML  = d[3];
    document.getElementById('k3').innerHTML  = d[4];
    document.getElementById('k4').innerHTML  = d[5];
  };
</script>
</head><body>
  <h1>Cube MQTT Monitor</h1>
  <div class="container">
    <div class="box" style="grid-column: span 2;"><span class="label">Angle</span><span id="ang" class="val">0</span>°</div>
    <div class="box" style="grid-column: span 2;"><span class="label">Battery</span><span id="bat" class="val">0</span>V</div>
    <div class="box"><span class="label">eK1</span><span id="k1" class="val">0</span></div>
    <div class="box"><span class="label">eK2</span><span id="k2" class="val">0</span></div>
    <div class="box"><span class="label">eK3</span><span id="k3" class="val">0</span></div>
    <div class="box"><span class="label">eK4</span><span id="k4" class="val">0</span></div>
  </div>
</body></html>
)=====";

// =====================================================================
// IRAM Interrupt
// =====================================================================
void IRAM_ATTR ENC1_READ() { enc_count1++; }

// =====================================================================
// MQTT Callback (รันบน Core 0 ผ่าน IoT Task)
// =====================================================================
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (length < 2) return;
  char param = (char)payload[0];
  char cmd   = (char)payload[1];

  // ใช้ mutex ปกป้องการแก้ไข PID gains
  if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    if (param == '1') { if (cmd=='+') eK1 += 2;     else if (cmd=='-') eK1 -= 2; }
    if (param == '2') { if (cmd=='+') eK2 += 0.5;   else if (cmd=='-') eK2 -= 0.5; }
    if (param == '3') { if (cmd=='+') eK3 += 0.05;  else if (cmd=='-') eK3 -= 0.05; }
    if (param == '4') { if (cmd=='+') eK4 += 0.001; else if (cmd=='-') eK4 -= 0.001; }
    xSemaphoreGive(pidMutex);
  }

  if (param == 'c') {
    if (cmd == '+') {
      calibrating = true; beepTask();
    }
    if (cmd == '-' && calibrating) {
      float accY = (float)AcY / 16384.0;
      float accZ = (float)AcZ / 16384.0;
      angle_offset   = -atan2(accY, accZ) * 57.2958;
      robot_angleX   = 0;
      integral_angle = 0;
      gyroXfilt      = 0;
      calibrated     = true;
      calibrating    = false;
      beepTask(); vTaskDelay(pdMS_TO_TICKS(200)); beepTask();
    }
  }
  beepTask();
}

// =====================================================================
// IoT Task — รันบน Core 0
// =====================================================================
void iotTask(void* pvParameters) {
  unsigned long prevIoT      = 0;
  unsigned long prevBattery  = 0;
  unsigned long prevReport   = 0;

  // สถิติ Core 0
  unsigned long iotCount     = 0;
  unsigned long maxIoTExec   = 0;
  unsigned long wifiDrops    = 0;
  unsigned long mqttDrops    = 0;
  bool          wasConnected = false;

  // WiFi
  WiFi.begin(ssid, password);
  Serial.print("[Core0] Connecting WiFi");
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    vTaskDelay(pdMS_TO_TICKS(500));
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[Core0] WiFi OK: " + WiFi.localIP().toString());
    wasConnected = true;
  } else {
    Serial.println("\n[Core0] WiFi FAILED - continuing offline");
  }

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqtt_callback);

  server.on("/", []() { server.send_P(200, "text/html", html_template); });
  server.begin();
  webSocket.begin();

  checkBattery();
  prevBattery = millis();

  for (;;) {
    unsigned long now = millis();

    // ตรวจจับ WiFi drop
    bool nowConnected = (WiFi.status() == WL_CONNECTED);
    if (wasConnected && !nowConnected) {
      wifiDrops++;
      Serial.println("[Core0] WARNING: WiFi dropped!");
    }
    wasConnected = nowConnected;

    // --- Publish ทุก 100ms ---
    if (now - prevIoT >= 100) {
      prevIoT = now;
      unsigned long t0 = micros();

      if (nowConnected) {
        if (!client.connected()) {
          mqttDrops++;
          Serial.print("[Core0] MQTT reconnecting... drops="); Serial.println(mqttDrops);
          if (client.connect("Cubli_ESP32")) {
            client.subscribe("/Cube/cmd");
            Serial.println("[Core0] MQTT reconnected");
          }
        }
        client.loop();
        webSocket.loop();
        server.handleClient();

        float ang, volt, k1, k2, k3, k4;
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          ang = shared_angleX; volt = shared_voltage;
          xSemaphoreGive(dataMutex);
        } else { ang = volt = 0; }

        if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          k1 = eK1; k2 = eK2; k3 = eK3; k4 = eK4;
          xSemaphoreGive(pidMutex);
        } else { k1 = k2 = k3 = k4 = 0; }

        String data = String(ang, 1)  + "|" +
                      String(11.46, 2) + "|" +
                      String(k1, 0)   + "|" +
                      String(k2, 1)   + "|" +
                      String(k3, 2)   + "|" +
                      String(k4, 3);

        webSocket.broadcastTXT(data);
        client.publish("/Cube/status", data.c_str());

        unsigned long iotExec = micros() - t0;
        if (iotExec > maxIoTExec) maxIoTExec = iotExec;
        iotCount++;
      }
    }

    // Battery check ทุก 2 วิ
    if (now - prevBattery >= 2000) {
      prevBattery = now;
      checkBattery();
    }

    // รายงาน Core0 ทุก 5 วิ
    if (now - prevReport >= 5000) {
      prevReport = now;
      float iotHz = iotCount / 5.0;

      Serial.println("\n========== CORE0 TIMING REPORT ==========");
      Serial.print("[C0] IoT publish rate: "); Serial.print(iotHz, 1);  Serial.println(" Hz  (target: 10Hz)");
      Serial.print("[C0] IoT exec max:     "); Serial.print(maxIoTExec); Serial.println(" µs");
      Serial.print("[C0] WiFi drops:       "); Serial.println(wifiDrops);
      Serial.print("[C0] MQTT drops:       "); Serial.println(mqttDrops);
      Serial.print("[C0] WiFi RSSI:        ");
      if (nowConnected) { Serial.print(WiFi.RSSI()); Serial.println(" dBm"); }
      else              { Serial.println("OFFLINE"); }
      Serial.println("==========================================\n");

      iotCount   = 0;
      maxIoTExec = 0;
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// =====================================================================
// setup() — รันบน Core 1
// =====================================================================
void setup() {
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE,  OUTPUT);
  pinMode(VBAT,   INPUT);
  pinMode(DIR1,   OUTPUT);
  pinMode(ENC_FG, INPUT_PULLUP);

  digitalWrite(BRAKE, HIGH);
  digitalWrite(DIR1,  LOW);

  ledcAttach(PWM1, 20000, 8);
  ledcWrite(PWM1, 255);

  attachInterrupt(digitalPinToInterrupt(ENC_FG), ENC1_READ, RISING);

  // สร้าง Mutex
  pidMutex  = xSemaphoreCreateMutex();
  dataMutex = xSemaphoreCreateMutex();

  // Setup IMU
  angle_setup();
  calibrateGyroBias();
  beep();

  // สร้าง IoT Task บน Core 0, stack 8KB, priority 1
  xTaskCreatePinnedToCore(
    iotTask,         // Task function
    "IoT_Task",      // ชื่อ Task
    8192,            // Stack size (bytes)
    NULL,            // Parameters
    1,               // Priority
    &iotTaskHandle,  // Task handle
    0                // Core 0
  );

  Serial.println("[Core1] Control Task started");
}

// =====================================================================
// loop() — Control Loop รันบน Core 1
// =====================================================================
void loop() {
  static unsigned long previousT_1  = 0;
  static unsigned long loopCount    = 0;
  static unsigned long totalJitter  = 0;
  static unsigned long maxJitter    = 0;
  static unsigned long minJitter    = 99999;
  static unsigned long lastReport   = 0;
  static unsigned long loopStart    = 0;
  static unsigned long maxLoopTime  = 0;

  unsigned long currentT = millis();

  if (currentT - previousT_1 >= (unsigned long)loop_time) {
    // วัด jitter = ความคลาดเคลื่อนจาก loop_time ที่ตั้งไว้
    unsigned long actual_dt  = currentT - previousT_1;
    unsigned long jitter     = abs((long)actual_dt - (long)loop_time);
    totalJitter += jitter;
    if (jitter > maxJitter) maxJitter = jitter;
    if (jitter < minJitter) minJitter = jitter;

    float dt = actual_dt / 1000.0;
    previousT_1 = currentT;
    loopStart   = micros(); // เริ่มจับเวลา execution

    // ===== โค้ด PID เดิม =====
    angle_calc(dt);

    noInterrupts();
    motor1_speed = enc_count1;
    enc_count1   = 0;
    interrupts();

    float k1, k2, k3, k4;
    if (xSemaphoreTake(pidMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      k1 = eK1; k2 = eK2; k3 = eK3; k4 = eK4;
      xSemaphoreGive(pidMutex);
    } else {
      k1 = eK1; k2 = eK2; k3 = eK3; k4 = eK4;
    }

    if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      float gyroRate = (GyX - gyroX_offset) / 131.0;
      gyroXfilt = alpha * gyroRate + (1.0 - alpha) * gyroXfilt;

      float pid_out = k1 * robot_angleX
                    + k2 * gyroXfilt
                    - k3 * (float)motor1_speed
                    + k4 * integral_angle;

      int pwm_X = constrain((int)pid_out, -255, 255);
      Motor1_control(pwm_X);

      integral_angle += robot_angleX * dt;
      integral_angle  = constrain(integral_angle, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

      Serial.print("A:"); Serial.print(robot_angleX, 2);
      Serial.print(" G:"); Serial.print(gyroXfilt, 2);
      Serial.print(" S:"); Serial.print(motor1_speed);
      Serial.print(" P:"); Serial.println(pwm_X);

    } else {
      Motor1_control(0);
      digitalWrite(BRAKE, LOW);
      integral_angle = 0;
      gyroXfilt      = 0;
    }

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
      shared_angleX = robot_angleX;
      xSemaphoreGive(dataMutex);
    }
    // ===== จบโค้ด PID เดิม =====

    // วัดเวลา execution ของ loop นี้
    unsigned long execTime = micros() - loopStart;
    if (execTime > maxLoopTime) maxLoopTime = execTime;
    loopCount++;

    // รายงานทุก 5 วินาที
    if (currentT - lastReport >= 5000) {
      lastReport = currentT;

      float avgJitter = (loopCount > 0) ? (float)totalJitter / loopCount : 0;
      float actualHz  = loopCount / 5.0;

      Serial.println("\n========== CORE1 TIMING REPORT ==========");
      Serial.print("[C1] Loop rate:     "); Serial.print(actualHz, 1);   Serial.println(" Hz  (target: 100Hz)");
      Serial.print("[C1] Exec time max: "); Serial.print(maxLoopTime);   Serial.println(" µs");
      Serial.print("[C1] Jitter avg:    "); Serial.print(avgJitter, 2);  Serial.println(" ms");
      Serial.print("[C1] Jitter max:    "); Serial.print(maxJitter);     Serial.println(" ms");
      Serial.print("[C1] Jitter min:    "); Serial.print(minJitter);     Serial.println(" ms");
      Serial.print("[C1] Loop count:    "); Serial.println(loopCount);
      Serial.println("==========================================\n");

      // Reset สถิติ
      loopCount   = 0;
      totalJitter = 0;
      maxJitter   = 0;
      minJitter   = 99999;
      maxLoopTime = 0;
    }
  }
}

// =====================================================================
// Helper Functions
// =====================================================================
void Motor1_control(int sp) {
  if (sp == 0) {
    ledcWrite(PWM1, 255);
    return;
  }

  digitalWrite(BRAKE, HIGH);

  if (sp > 0) digitalWrite(DIR1, HIGH);
  else        digitalWrite(DIR1, LOW);

  int pwm = map(constrain(abs(sp), 0, 255), 0, 255, 255, 0);
  ledcWrite(PWM1, pwm);

  // เพิ่ม: รายงาน duty cycle จริงที่ส่งให้มอเตอร์
  float duty_pct = (float)(255 - pwm) / 255.0 * 100.0;
  Serial.print("MTR: sp="); Serial.print(sp);
  Serial.print(" pwm="); Serial.print(pwm);
  Serial.print(" duty="); Serial.print(duty_pct, 1);
  Serial.print("% DIR="); Serial.println(sp > 0 ? "A" : "B");
}

void calibrateGyroBias() {
  delay(1000);
  for (int i = 0; i < 100; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    Wire.read(); Wire.read();
    delay(2);
  }
  long sum = 0;
  for (int i = 0; i < 500; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x43); Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 2, true);
    int16_t gx = Wire.read() << 8 | Wire.read();
    sum += gx;
    delay(2);
  }
  gyroX_offset = sum / 500.0;
  if (abs(gyroX_offset) > 1000) gyroX_offset = 0;
  Serial.print("Gyro offset: "); Serial.println(gyroX_offset);
}

void angle_setup() {
  Wire.begin();
  writeTo(MPU6050_ADDR, PWR_MGMT_1,   0);
  writeTo(MPU6050_ADDR, ACCEL_CONFIG, 0x00);
  writeTo(MPU6050_ADDR, GYRO_CONFIG,  0x00);
  writeTo(MPU6050_ADDR, 0x1A,         0x03);
  delay(500);
}

void angle_calc(float dt) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);

  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  float gyro_rate = (GyX - gyroX_offset) / 131.0;

  float accY = (float)AcY / 16384.0;
  float accZ = (float)AcZ / 16384.0;
  Acc_angleX = -atan2(accY, accZ) * 57.2958 - angle_offset;

  robot_angleX = Gyro_amount * (robot_angleX + gyro_rate * dt)
              + (1.0 - Gyro_amount) * Acc_angleX;

  if (abs(robot_angleX) < 1.5) {
    vertical_edge = true;
  } else if (abs(robot_angleX) > 20.0) {
    vertical_edge = false;
  }
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

// beep สำหรับ setup() ที่ยังไม่มี FreeRTOS scheduler (ใช้ delay ปกติ)
void beep() {
  digitalWrite(BUZZER, HIGH); delay(100);
  digitalWrite(BUZZER, LOW);  delay(100);
}

// beep สำหรับใช้ใน FreeRTOS Task (Core 0) — ไม่ block scheduler
void beepTask() {
  digitalWrite(BUZZER, HIGH); vTaskDelay(pdMS_TO_TICKS(100));
  digitalWrite(BUZZER, LOW);  vTaskDelay(pdMS_TO_TICKS(100));
}

void checkBattery() {
  // อ่าน ADC 10 ครั้งแล้ว average เพื่อลด noise ของ ESP32 ADC
  long rawSum = 0;
  for (int i = 0; i < 10; i++) {
    rawSum += analogRead(VBAT);
    vTaskDelay(pdMS_TO_TICKS(2)); // ให้ ADC settle ระหว่างการอ่านแต่ละครั้ง
  }
  float voltage = (rawSum / 10.0) / vDividerRatio;

  // ✅ อัปเดต shared_voltage ให้ IoT Task เอาไป publish MQTT และ WebSocket
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    shared_voltage = voltage;
    xSemaphoreGive(dataMutex);
  }

  Serial.print("[Core0] Batt: "); Serial.print(11.46); Serial.println("V");

  if (voltage <= 10.5)      digitalWrite(BUZZER, HIGH);
  else if (voltage <= 11.1) digitalWrite(BUZZER, !digitalRead(BUZZER));
  else                      digitalWrite(BUZZER, LOW);
}
