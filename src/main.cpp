// ===== TEST GABUNGAN: IMU MPU6050 (RPY) + HC-SR04 (jarak) =====
// Output tiap 50 ms (20 Hz), satu baris:
//   RPY  : roll, pitch, yaw (derajat)
//   G    : kecepatan putar gyro X, Y, Z (deg/s)       -> Gz = kecepatan belok
//   LA   : akselerasi linear X, Y, Z frame sensor (m/s^2, gravitasi dibuang)
//   WA   : akselerasi linear X, Y, Z frame dunia  (m/s^2, tidak terpengaruh miring)
//   Jarak: cm
// Jarak invalid (timeout / di luar rentang) -> pakai nilai valid sebelumnya,
// "--" hanya sebelum ada data valid pertama.
// Ketik 'z' di Serial Monitor = reset yaw ke 0.
//
// Wiring:
//   MPU6050 : SDA->GPIO21, SCL->GPIO22, VCC->3V3, GND, AD0->GND (0x68)
//   HC-SR04 : TRIG->GPIO5, ECHO->GPIO18 lewat pembagi (1k seri + 2k ke GND),
//             VCC->VIN (5V), GND bersama
//
// Saat boot IMU dikalibrasi beberapa detik: robot HARUS DIAM dan rata.
// Yaw drift pelan (tidak ada magnetometer).
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// ---------- pin ----------
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_TRIG 5
#define PIN_ECHO 18

// ---------- konstanta ----------
#define PRINT_PERIOD_MS 50     // cetak ke serial (20 Hz)
#define SONAR_PERIOD_MS 50     // jeda antar trigger HC-SR04
#define ECHO_TIMEOUT_US 30000  // tanpa echo selama ini = invalid
#define DIST_MIN_CM 2.0f       // di luar rentang = invalid (nilai sebelumnya dipakai)
#define DIST_MAX_CM 400.0f

// ================= IMU =================
MPU6050 mpu;
bool imuReady = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3] = {0, 0, 0};  // radian: [0]=yaw [1]=pitch [2]=roll
VectorInt16 gyroRaw, accRaw, accLin, accWorld;

// skala FIFO DMP (MotionApps20): gyro +-2000 dps, accel 1 g = 8192
#define GYRO_LSB_PER_DPS 16.4f
#define ACC_LSB_PER_G 8192.0f
#define G_MS2 9.80665f
float yawOffset = 0;       // derajat

void i2cScan() {
  Serial.print("# I2C scan:");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      n++;
    }
  }
  Serial.println(n ? "" : " tidak ada perangkat! cek kabel SDA/SCL/VCC/GND");
}

void imuInit() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  i2cScan();

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("# MPU6050 tidak merespon. Cek wiring/alamat. Lanjut tanpa IMU.");
    return;
  }
  uint8_t st = mpu.dmpInitialize();
  if (st != 0) {
    Serial.printf("# DMP gagal init (kode %u). Lanjut tanpa IMU.\n", st);
    return;
  }
  Serial.println("# Kalibrasi IMU... JANGAN DIGERAKKAN");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println();
  mpu.setDMPEnabled(true);
  imuReady = true;
  Serial.println("# IMU siap");
}

void imuUpdate() {
  if (!imuReady) return;
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gyroRaw, fifoBuffer);
    mpu.dmpGetAccel(&accRaw, fifoBuffer);
    mpu.dmpGetLinearAccel(&accLin, &accRaw, &gravity);
    accWorld = accLin;
    accWorld.rotate(&q);  // putar ke frame dunia pakai quaternion
  }
}

float wrap180(float d) {
  while (d > 180) d -= 360;
  while (d < -180) d += 360;
  return d;
}

// ================= HC-SR04 =================
// Non-blocking: trigger berbasis millis(), durasi echo diukur via interrupt CHANGE.
volatile uint32_t echoStartUs = 0;
volatile uint32_t echoDurUs = 0;
volatile bool echoReady = false;

void IRAM_ATTR onEcho() {
  uint32_t now = micros();
  if (digitalRead(PIN_ECHO)) {
    echoStartUs = now;              // rising: mulai ukur
  } else {
    echoDurUs = now - echoStartUs;  // falling: selesai
    echoReady = true;
  }
}

uint32_t lastTrigMs = 0;
uint32_t trigUs = 0;
bool sonarWaiting = false;
float distance = -1;  // cm, data valid terakhir (-1 = belum pernah valid)

void sonarInit() {
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), onEcho, CHANGE);
}

void sonarUpdate() {
  uint32_t nowMs = millis();

  if (!sonarWaiting && nowMs - lastTrigMs >= SONAR_PERIOD_MS) {
    lastTrigMs = nowMs;
    echoReady = false;
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);
    trigUs = micros();
    sonarWaiting = true;
  }
  if (!sonarWaiting) return;

  if (echoReady) {
    noInterrupts();
    uint32_t dur = echoDurUs;
    echoReady = false;
    interrupts();
    float cm = dur * 0.0343f / 2.0f;  // kecepatan suara 343 m/s
    if (cm >= DIST_MIN_CM && cm <= DIST_MAX_CM) distance = cm;  // invalid: tetap nilai lama
    sonarWaiting = false;
  } else if (micros() - trigUs > ECHO_TIMEOUT_US) {
    sonarWaiting = false;  // timeout: tetap nilai lama
  }
}

// ================= main =================
uint32_t lastPrintMs = 0;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("# TEST GABUNGAN IMU + HC-SR04");
  sonarInit();
  imuInit();
  Serial.println("# Format: RPY(deg) | G xyz(deg/s) | LA xyz(m/s2) | WA xyz(m/s2) | Jarak(cm). 'z' = reset yaw.");
}

void loop() {
  imuUpdate();
  sonarUpdate();

  if (Serial.available() && Serial.read() == 'z') {
    yawOffset = ypr[0] * RAD_TO_DEG;
    Serial.println("# yaw direset");
  }

  if (millis() - lastPrintMs >= PRINT_PERIOD_MS) {
    lastPrintMs = millis();
    float roll  = ypr[2] * RAD_TO_DEG;
    float pitch = ypr[1] * RAD_TO_DEG;
    float yaw   = wrap180(ypr[0] * RAD_TO_DEG - yawOffset);
    const float kG = 1.0f / GYRO_LSB_PER_DPS;
    const float kA = G_MS2 / ACC_LSB_PER_G;
    Serial.printf("RPY %7.2f %7.2f %7.2f | G %7.1f %7.1f %7.1f | LA %6.2f %6.2f %6.2f | WA %6.2f %6.2f %6.2f | Jarak ",
                  roll, pitch, yaw,
                  gyroRaw.x * kG, gyroRaw.y * kG, gyroRaw.z * kG,
                  accLin.x * kA, accLin.y * kA, accLin.z * kA,
                  accWorld.x * kA, accWorld.y * kA, accWorld.z * kA);
    if (distance < 0) Serial.println("--");
    else Serial.printf("%.2f\n", distance);
  }
}
