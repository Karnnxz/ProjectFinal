#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

// --- Pin Definitions (การต่อสาย) ---
#define VBAT        25   // อ่านไฟแบต
#define BUZZER      12   // ลำโพง/ไฟเตือน
#define BRAKE       4    // สายเบรกมอเตอร์
#define DIR1        16   // ทิศทางมอเตอร์
#define PWM1        17   // ความเร็วมอเตอร์
#define ENC_FG      19   // พัลส์เช็คความเร็วรอบ
#define INT_LED     2    // ไฟบนบอร์ด

// --- Motor PWM Settings ---
#define PWM1_CH     1
#define TIMER_BIT   8
#define BASE_FREQ   20000

// --- MPU6050 Register Addresses ---
#define MPU6050       0x68
#define ACCEL_CONFIG  0x1C
#define GYRO_CONFIG   0x1B
#define PWR_MGMT_1    0x6B

#define EEPROM_SIZE   64

// --- Variables (ตัวแปรระบบ) ---
float vDividerRatio = 278; 
float Gyro_amount = 0.92;
float alpha = 0.7;

bool vertical_edge = false;
bool calibrating = false;
bool calibrated = false;

// PID Gains (ค่าจูนความนิ่ง)
float eK1 = 200;    // มุม
float eK2 = 40.00;  // ความเร็วล้ม
float eK3 = 4;      // ความเร็วมอเตอร์
float eK4 = 0.02;   // ตำแหน่งสะสม

int loop_time = 15;
unsigned long currentT, previousT_1, previousT_2;

struct OffsetsObj {
  int ID;
  float acXe, acYe, acZe;
};
OffsetsObj offsets;

int16_t AcX, AcY, AcZ, GyX, GyY, GyZ;
float gyroX, gyroXfilt, robot_angleX, Acc_angleX;
volatile int enc_count1 = 0;
int16_t motor1_speed;
int32_t motors_speed_X;

BluetoothSerial SerialBT;

// --- Interrupt: นับรอบมอเตอร์ ---
void IRAM_ATTR ENC1_READ() {
  enc_count1++; 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Cubli");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(VBAT, INPUT);
  digitalWrite(BRAKE, HIGH); 

  pinMode(DIR1, OUTPUT);
  pinMode(ENC_FG, INPUT);
  attachInterrupt(ENC_FG, ENC1_READ, RISING);
  
  ledcAttachChannel(PWM1, BASE_FREQ, TIMER_BIT, PWM1_CH);
  Motor1_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID == 96) calibrated = true;

  angle_setup();
  beep(); 
}

void loop() {
  currentT = millis();

  // 1. ลูปควบคุมหลัก (Main Logic)
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    motor1_speed = enc_count1;
    enc_count1 = 0; 
    
    if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      // คำนวณแรงที่ต้องส่งให้มอเตอร์ (PID)
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor1_speed + eK4 * motors_speed_X, -255, 255);
      motors_speed_X += motor1_speed / 5;
      Motor1_control(pwm_X);
    } else {
      Motor1_control(0);
      digitalWrite(BRAKE, LOW); // ล็อกล้อเมื่อล้ม
      motors_speed_X = 0;
    }
    previousT_1 = currentT;
  }

  // 2. ลูปเช็คแบตเตอรี่
  if (currentT - previousT_2 >= 2000) {
    checkBattery();
    previousT_2 = currentT;
  }
}

// --- ฟังก์ชันเสริมต่างๆ ---

void checkBattery() {
  long rawSum = 0;
  for(int i=0; i<10; i++) rawSum += analogRead(VBAT);
  float voltage = (rawSum / 10.0) / vDividerRatio;

  if (voltage <= 10.5) digitalWrite(BUZZER, HIGH); // แบตวิกฤต
  else if (voltage <= 11.1) digitalWrite(BUZZER, !digitalRead(BUZZER)); // แบตอ่อน
  else digitalWrite(BUZZER, LOW);
}

void Motor1_control(int sp) {
  if (sp < 0) digitalWrite(DIR1, LOW);
  else digitalWrite(DIR1, HIGH);

  int pwm_val = constrain(abs(sp), 0, 255);
  ledcWrite(PWM1_CH, 255 - pwm_val); 
}

void angle_setup() {
  Wire.begin();
  writeTo(MPU6050, PWR_MGMT_1, 0);   // ปลุก MPU6050
  writeTo(MPU6050, ACCEL_CONFIG, 0); 
  writeTo(MPU6050, GYRO_CONFIG, 0);  
  delay(500);
}

float angle_offset = 0; 

void angle_calc() {
  Wire.beginTransmission(MPU6050);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050, 14, true);
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); 
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  float gyro_rate = (GyX - 0.5) / 65.536; 
  float accY = (float)AcY / 16384.0;
  float accZ = (float)AcZ / 16384.0;
  float raw_acc_angle = atan2(accY, -accZ) * 57.2958;
  Acc_angleX = raw_acc_angle - angle_offset;

  float dt = loop_time / 1000.0;
  robot_angleX = Gyro_amount * (robot_angleX + gyro_rate * dt) + (1.0 - Gyro_amount) * Acc_angleX;

  if (abs(robot_angleX) < 1.0) vertical_edge = true; 
  else if (abs(robot_angleX) > 15) vertical_edge = false;
}

// ส่งข้อมูลไปที่ MPU6050
void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

// เสียงปี๊บ
void beep() {
  digitalWrite(BUZZER, HIGH); delay(100); digitalWrite(BUZZER, LOW);
}

// ปรับจูนผ่าน Bluetooth
void Tuning() {
  if (!SerialBT.available()) return;
  char param = SerialBT.read();
  char cmd = SerialBT.read();

  if (param == '1') { if (cmd == '+') eK1 += 5 ; else if (cmd == '-') eK1 -= 5; }
  if (param == '2') { if (cmd == '+') eK2 += 1; else if (cmd == '-') eK2 -= 1; }
  if (param == '3') { if (cmd == '+') eK3 += 0.1; else if (cmd == '-') eK3 -= 0.1; }
  if (param == '4') { if (cmd == '+') eK4 += 0.001; else if (cmd == '-') eK4 -= 0.001; }

  if (param == 'c') {
    if (cmd == '+') { calibrating = true; beep(); }
    if (cmd == '-' && calibrating) {
      float accY = (float)AcY / 16384.0;
      float accZ = (float)AcZ / 16384.0;
      angle_offset = atan2(accY, -accZ) * 57.2958;
      robot_angleX = 0;
      motors_speed_X = 0;
      calibrated = true;
      calibrating = false;
      beep(); beep();
    }
  }
  if (param == 'v') reportStatus();
}

// รายงานค่ากลับไปที่ Bluetooth
void reportStatus() {
  float voltage = (analogRead(VBAT)) / vDividerRatio;
  SerialBT.println("\n--- Settings ---");
  SerialBT.print("eK1:"); SerialBT.print(eK1);
  SerialBT.print(" eK2:"); SerialBT.print(eK2);
  SerialBT.print(" eK3:"); SerialBT.print(eK3);
  SerialBT.print(" eK4:"); SerialBT.println(eK4, 4);
  SerialBT.print("Angle:"); SerialBT.print(robot_angleX, 2);
  SerialBT.print(" | Batt:"); SerialBT.print(voltage); SerialBT.println("V");
}
