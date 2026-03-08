#include <Wire.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

// --- Pin Definitions ---
#define VBAT        25
#define BUZZER      12   // ต่อผ่าน Transistor ขับ LED หรือ Buzzer
#define BRAKE       4   // สายสีเขียวของมอเตอร์ 3640
#define DIR1        16    // สายสีเหลือง
#define PWM1        17   // สายสีขาว
#define ENC_FG      19   // สายสีน้ำเงิน (FG) ต่อ Pull-up 10k + Cap 0.1uF
#define INT_LED     2

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

// --- Variables ---
float vDividerRatio = 278; // ปรับตามมัลติมิเตอร์สำหรับ R 33k/10k
float Gyro_amount = 0.92;
float alpha = 0.7;

bool vertical_edge = false;
bool calibrating = false;
bool calibrated = false;

// PID Gains สำหรับ 1 แกน (มวล 1kg)
float eK1 = 200;
float eK2 = 40.00; 
float eK3 = 4;
float eK4 = 0.02;

int loop_time = 15;
unsigned long currentT, previousT_1, previousT_2;

struct OffsetsObj {
  int ID;
  float acXe, acYe, acZe;
};
OffsetsObj offsets;

int16_t AcX, AcY, AcZ, AcXc, AcYc, AcZc, GyX, GyY, GyZ;
float gyroX, gyroXfilt, robot_angleX, Acc_angleX;
volatile int enc_count1 = 0;
int16_t motor1_speed;
int32_t motors_speed_X;

BluetoothSerial SerialBT;

// --- Interrupt Service Routine สำหรับ FG ---
void IRAM_ATTR ENC1_READ() {
  // สาย FG ของ 3640 ส่งพัลส์ตามความเร็ว
  enc_count1++; 
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Cubli");
  EEPROM.begin(EEPROM_SIZE);

  pinMode(BUZZER, OUTPUT);
  pinMode(BRAKE, OUTPUT);
  pinMode(VBAT, INPUT);
  digitalWrite(BRAKE, HIGH); // ปลดเบรกเพื่อให้มอเตอร์หมุนได้

  // Motor 1 Setup
  pinMode(DIR1, OUTPUT);
  pinMode(ENC_FG, INPUT);
  attachInterrupt(ENC_FG, ENC1_READ, RISING);
  
  ledcAttachChannel(PWM1, BASE_FREQ, TIMER_BIT, PWM1_CH);
  Motor1_control(0);

  EEPROM.get(0, offsets);
  if (offsets.ID == 96) calibrated = true;

  angle_setup();
  beep(); // พร้อมทำงาน
}


unsigned long lastReportBT = 0;
int last_pwm_X = 0; // เก็บค่าไว้รายงาน

void loop() {
  currentT = millis();

  // 1. ลูปควบคุมหลัก (ทุก 15ms)
  if (currentT - previousT_1 >= loop_time) {
    Tuning();
    angle_calc();

    motor1_speed = enc_count1;
    enc_count1 = 0; // Reset count เพื่อหาความเร็วในรอบถัดไป
    
    if (vertical_edge && calibrated && !calibrating) {
      digitalWrite(BRAKE, HIGH);
      gyroX = GyX / 131.0;
      gyroXfilt = alpha * gyroX + (1 - alpha) * gyroXfilt;
      
      int pwm_X = constrain(eK1 * robot_angleX + eK2 * gyroXfilt + eK3 * motor1_speed + eK4 * motors_speed_X, -255, 255);
      last_pwm_X = pwm_X;
      motors_speed_X += motor1_speed / 5;
      Motor1_control(pwm_X);
    } else {
      Motor1_control(0);
      digitalWrite(BRAKE, LOW); // ล็อกเบรกเมื่อไม่ได้ตั้งตรง
      motors_speed_X = 0;
    }
    previousT_1 = currentT;
  }

  // 2. ลูปตรวจวัดแบตเตอรี่ (ทุก 2 วินาที)
  if (currentT - previousT_2 >= 2000) {
    checkBattery();
    previousT_2 = currentT;
  }
}

void checkBattery() {
  long rawSum = 0;
  for(int i=0; i<10; i++) rawSum += analogRead(VBAT);
  float voltage = (rawSum / 10.0) / vDividerRatio;

  Serial.print("Batt: "); Serial.print(voltage); Serial.println("V");
  if (SerialBT.connected()) {
    SerialBT.print("Voltage: "); SerialBT.println(voltage);
  }

  if (voltage <= 10.5) {           // วิกฤต
    digitalWrite(BUZZER, HIGH); 
  } else if (voltage <= 11.1) {    // เตือน
    digitalWrite(BUZZER, !digitalRead(BUZZER)); // กะพริบ
  } else {
    digitalWrite(BUZZER, LOW);
  }
}

void Motor1_control(int sp) {
  // สำหรับมวล 1kg มอเตอร์ต้องสั่ง DIR สลับกันได้จริง
  if (sp < 0) {
    digitalWrite(DIR1, LOW); // ลองสลับ LOW/HIGH ตรงนี้ดู
  } else {
    digitalWrite(DIR1, HIGH);
  }

  // ปรับ PWM (0-255)
  // มอเตอร์ 3640 บางรุ่นใช้ 0 คือหยุด บางรุ่นใช้ 255 คือหยุด
  int pwm_val = constrain(abs(sp), 0, 255);
  ledcWrite(PWM1_CH, 255 - pwm_val); // ถ้าล้อไม่หมุนเลย ลองเปลี่ยนเป็น ledcWrite(PWM1_CH, pwm_val);
}

void angle_setup() {
  Wire.begin();
  writeTo(MPU6050, PWR_MGMT_1, 0);
  writeTo(MPU6050, ACCEL_CONFIG, 0); 
  writeTo(MPU6050, GYRO_CONFIG, 0); 
  delay(500);
}

float angle_offset = 0; // ตัวแปรเก็บค่ามุมตอนตั้งตรง

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

  // 1. คำนวณความเร็วเชิงมุม (Gyro Rate) และหักลบ Offset
  // หมายเหตุ: GyX_offset ควรหาตอน setup หรือตอนสั่ง c+
  float gyro_rate = (GyX - 0.5) / 65.536; 

  // 2. คำนวณมุมจาก Accel (หักลบ Offset ที่คุณวัดมา)
  float accY = (float)AcY / 16384.0;
  float accZ = (float)AcZ / 16384.0;
  float raw_acc_angle = atan2(accY, -accZ) * 57.2958;
  Acc_angleX = raw_acc_angle - angle_offset;

  // 3. Complementary Filter (ตัวเอกของงาน)
  // เพิ่มการคูณ DT (Delta Time) ที่แม่นยำ
  float dt = loop_time / 1000.0;
  robot_angleX = Gyro_amount * (robot_angleX + gyro_rate * dt) + (1.0 - Gyro_amount) * Acc_angleX;

  // ตรวจสอบสถานะเพื่อสั่งเริ่มทำงาน
  if (abs(robot_angleX) < 1.0) { 
    vertical_edge = true; 
  } else if (abs(robot_angleX) > 15) {
    vertical_edge = false;
  }
}

void writeTo(byte device, byte address, byte value) {
  Wire.beginTransmission(device);
  Wire.write(address);
  Wire.write(value);
  Wire.endTransmission(true);
}

void beep() {
  digitalWrite(BUZZER, LOW); delay(100); digitalWrite(BUZZER, LOW);
}

void Tuning() {
  if (!SerialBT.available()) return;
  
  char param = SerialBT.read();
  char cmd = SerialBT.read();

  // ปรับ PID ผ่านมือถือ (1+, 2+, 3+, 4-)
  if (param == '1') { if (cmd == '+') eK1 += 5 ; else if (cmd == '-') eK1 -= 5; }
  if (param == '2') { if (cmd == '+') eK2 += 1; else if (cmd == '-') eK2 -= 1; }
  if (param == '3') { if (cmd == '+') eK3 += 0.1; else if (cmd == '-') eK3 -= 0.1; }
  if (param == '4') { if (cmd == '+') eK4 += 0.001; else if (cmd == '-') eK4 -= 0.001; }

  // --- ระบบ Calibrate เซ็ตศูนย์ ---
  if (param == 'c') {
    if (cmd == '+') {
      calibrating = true;
      beep();
      SerialBT.println("Setting Zero...");
    }
    if (cmd == '-' && calibrating) {
      // อ่านค่าดิบ ณ วินาทีที่กด c- มาเป็นจุด 0
      float accY = (float)AcY / 16384.0;
      float accZ = (float)AcZ / 16384.0;
      angle_offset = atan2(accY, -accZ) * 57.2958;
      
      robot_angleX = 0;    // รีเซ็ตมุมปัจจุบันให้เป็น 0 ทันที
      motors_speed_X = 0;  // ล้างค่าสะสมมอเตอร์
      
      calibrated = true;
      calibrating = false;
      beep(); beep();
      SerialBT.print("Done! Offset: "); SerialBT.println(angle_offset);
    }
  }

  if (param == 'v') reportStatus();
}

void reportStatus() {
  float voltage = (analogRead(VBAT)) / vDividerRatio;
  SerialBT.println("\n--- Settings ---");
  SerialBT.print("eK1:"); SerialBT.print(eK1);
  SerialBT.print(" eK2:"); SerialBT.print(eK2);
  SerialBT.print(" eK3:"); SerialBT.print(eK3);
  SerialBT.print(" eK4:"); SerialBT.println(eK4, 4);
  SerialBT.print("Angle:"); SerialBT.print(robot_angleX, 2);
  SerialBT.print(" | Batt:"); SerialBT.print(voltage); SerialBT.println("V");
  SerialBT.println("----------------");
}
