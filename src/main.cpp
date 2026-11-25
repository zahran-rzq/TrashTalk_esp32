// ===== FIRMWARE TEST: IMU + HC-SR04 + MOTOR (L298N) + SERVO, protokol biner =====
// Semua komunikasi dalam FRAME BINER (header AA 55 + CRC16), format di include/protocol.h.
//   ESP32 -> PC : TELEMETRY 20 Hz (RPY, gyro, akselerasi, jarak, status motor/servo)
//   PC -> ESP32 : CMD_MOTOR, CMD_STOP, CMD_SERVO, CMD_RESET_YAW
// Program PC: pc/robot_teleop.c (kontrol keyboard), pc/robot_rx.c (monitor saja).
// Pesan teks berawalan '#' hanya saat boot (parser PC mengabaikannya).
//
// Failsafe: motor berhenti jika tidak ada CMD_MOTOR/CMD_STOP selama CMD_TIMEOUT_MS (500 ms).
// Jarak invalid (timeout / di luar rentang) -> nilai valid sebelumnya; -1 = belum pernah valid.
//
// Wiring:
//   MPU6050 : SDA->GPIO21, SCL->GPIO22, VCC->3V3, GND, AD0->GND (0x68)
//   HC-SR04 : TRIG->GPIO5, ECHO->GPIO18 lewat pembagi (1k seri + 2k ke GND),
//             VCC->VIN (5V), GND bersama
//   L298N   : ENA->13, IN1->12, IN2->14, IN3->26, IN4->25, ENB->33
//             (jumper ENA/ENB dicabut; GPIO12 strapping pin, harus LOW saat boot)
//   Servo   : sinyal->GPIO32, daya dari 5V L298N (bukan dari ESP32)
//   LED     : onboard GPIO2 (kedip lambat = kalibrasi, kedip cepat = loop jalan)
//
// Saat boot IMU dikalibrasi beberapa detik: robot HARUS DIAM dan rata.
// Yaw drift pelan (tidak ada magnetometer).
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "protocol.h"
#include <esp_timer.h>

// ---------- pin ----------
#define PIN_SDA 21
#define PIN_SCL 22
#define PIN_TRIG 5
#define PIN_ECHO 18
#define PIN_ENA 13    // PWM motor kiri
#define PIN_IN1 12    // strapping pin!
#define PIN_IN2 14
#define PIN_IN3 26
#define PIN_IN4 25
#define PIN_ENB 33    // PWM motor kanan
#define PIN_SERVO 32
#define PIN_LED 2      // LED biru onboard DevKit V1

// ---------- konstanta ----------
#define SEND_PERIOD_MS 50      // kirim frame ke PC (20 Hz)
#define SONAR_PERIOD_MS 50     // jeda antar trigger HC-SR04
#define ECHO_TIMEOUT_US 30000  // tanpa echo selama ini = invalid
#define DIST_MIN_CM 2.0f       // di luar rentang = invalid (nilai sebelumnya dipakai)
#define DIST_MAX_CM 400.0f
#define PWM_FREQ 1000          // PWM motor (Hz)
#define PWM_RES 8              // 8 bit -> duty 0..255
#define SERVO_FREQ 50
#define SERVO_RES 16
#define SERVO_MIN_US 500       // pulsa di 0 derajat
#define SERVO_MAX_US 2400      // pulsa di 180 derajat
#define SERVO_START_DEG 90
#define LED_SLOW_BLINK_US 500000  // kedip lambat saat kalibrasi: toggle tiap 0.5 s (1 Hz)
#define LED_FAST_BLINK_MS 100     // kedip cepat saat loop jalan: toggle tiap 100 ms (5 Hz)
#define CH_ENA 0               // kanal LEDC (dipakai di core 2.x; core 3.x pakai pin)
#define CH_ENB 1
#define CH_SERVO 2

// ================= MOTOR & SERVO =================
// PWM dibungkus agar kompatibel ESP32 Arduino core 2.x dan 3.x.
void pwmInit(uint8_t pin, uint8_t ch, uint32_t freq, uint8_t res) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(pin, freq, res, ch);
#else
  ledcSetup(ch, freq, res);
  ledcAttachPin(pin, ch);
#endif
}

void pwmWrite(uint8_t pin, uint8_t ch, uint32_t duty) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  (void)ch;
  ledcWrite(pin, duty);
#else
  (void)pin;
  ledcWrite(ch, duty);
#endif
}

int16_t motorL = 0, motorR = 0;  // kecepatan yang sedang dipakai
uint8_t servoDeg = SERVO_START_DEG;

// Matikan semua pin motor secepat mungkin (dipanggil paling awal di setup)
void motorPinsSafe() {
  const uint8_t pins[] = {PIN_ENA, PIN_IN1, PIN_IN2, PIN_IN3, PIN_IN4, PIN_ENB};
  for (uint8_t p : pins) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }
}

void motorSide(uint8_t inA, uint8_t inB, uint8_t pinEn, uint8_t ch, int v) {
  if (v > 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else if (v < 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else {
    digitalWrite(inA, LOW);  // 0 = lepas (coast)
    digitalWrite(inB, LOW);
  }
  pwmWrite(pinEn, ch, abs(v));
}

void setMotor(int l, int r) {
  motorL = constrain(l, -255, 255);
  motorR = constrain(r, -255, 255);
  motorSide(PIN_IN1, PIN_IN2, PIN_ENA, CH_ENA, motorL);
  motorSide(PIN_IN3, PIN_IN4, PIN_ENB, CH_ENB, motorR);
}

void setServo(int deg) {
  servoDeg = constrain(deg, 0, 180);
  uint32_t us = SERVO_MIN_US + (uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * servoDeg / 180;
  uint32_t duty = us * ((1UL << SERVO_RES) - 1) / (1000000UL / SERVO_FREQ);  // us -> duty
  pwmWrite(PIN_SERVO, CH_SERVO, duty);
}

void actuatorInit() {
  pwmInit(PIN_ENA, CH_ENA, PWM_FREQ, PWM_RES);
  pwmInit(PIN_ENB, CH_ENB, PWM_FREQ, PWM_RES);
  pwmInit(PIN_SERVO, CH_SERVO, SERVO_FREQ, SERVO_RES);
  setMotor(0, 0);
  setServo(SERVO_START_DEG);
}

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

// ================= komunikasi =================
uint8_t txSeq = 0;
FrameParser rx;
uint32_t lastCmdMs = 0;
bool failsafe = true;  // true = motor dihentikan karena belum/tidak ada perintah

void sendFrame(uint8_t type, const void *payload, uint8_t len) {
  uint8_t buf[FRAME_OVERHEAD + FRAME_MAX_PAYLOAD];
  size_t n = frame_build(buf, type, txSeq++, payload, len);
  Serial.write(buf, n);  // satu kali write, tidak terpotong
}

void handleFrame() {
  switch (rx.type) {
    case CMD_MOTOR:
      if (rx.len == sizeof(CmdMotorPayload)) {
        CmdMotorPayload c;
        memcpy(&c, rx.payload, sizeof(c));
        setMotor(c.left, c.right);
        lastCmdMs = millis();
        failsafe = false;
      }
      break;
    case CMD_STOP:
      setMotor(0, 0);
      lastCmdMs = millis();
      failsafe = false;
      break;
    case CMD_SERVO:
      if (rx.len == sizeof(CmdServoPayload)) setServo(((CmdServoPayload *)rx.payload)->deg);
      break;
    case CMD_RESET_YAW:
      yawOffset = ypr[0] * RAD_TO_DEG;
      break;
  }
}

void sendTelemetry() {
  const float kG = 1.0f / GYRO_LSB_PER_DPS;
  const float kA = G_MS2 / ACC_LSB_PER_G;
  TelemetryPayload p;
  p.t_ms  = millis();
  p.roll  = ypr[2] * RAD_TO_DEG;
  p.pitch = ypr[1] * RAD_TO_DEG;
  p.yaw   = wrap180(ypr[0] * RAD_TO_DEG - yawOffset);
  p.gx  = gyroRaw.x * kG;   p.gy  = gyroRaw.y * kG;   p.gz  = gyroRaw.z * kG;
  p.lax = accLin.x * kA;    p.lay = accLin.y * kA;    p.laz = accLin.z * kA;
  p.wax = accWorld.x * kA;  p.way = accWorld.y * kA;  p.waz = accWorld.z * kA;
  p.dist_cm = distance;
  p.motor_l = motorL;
  p.motor_r = motorR;
  p.servo_deg = servoDeg;
  p.flags = (failsafe ? TLM_FLAG_FAILSAFE : 0) | (imuReady ? TLM_FLAG_IMU_OK : 0);
  sendFrame(FRAME_TYPE_TELEMETRY, &p, sizeof(p));
}

// ================= LED status =================
// Kalibrasi/inisialisasi : kedip lambat (dari esp_timer, karena kalibrasi memblok setup)
// Loop berjalan          : kedip cepat 5 Hz, di-toggle dari loop() (bukan timer),
//                          jadi jika loop macet LED ikut berhenti berkedip.
esp_timer_handle_t ledTimer = nullptr;
bool ledState = false;

void ledBlinkCb(void *) {
  ledState = !ledState;
  digitalWrite(PIN_LED, ledState);
}

void ledStartSlowBlink() {
  pinMode(PIN_LED, OUTPUT);
  esp_timer_create_args_t args = {};
  args.callback = ledBlinkCb;
  args.name = "led";
  esp_timer_create(&args, &ledTimer);
  esp_timer_start_periodic(ledTimer, LED_SLOW_BLINK_US);
}

void ledStopSlowBlink() {
  esp_timer_stop(ledTimer);
  esp_timer_delete(ledTimer);
  ledTimer = nullptr;
}

// ================= main =================
uint32_t lastSendMs = 0;

void setup() {
  motorPinsSafe();  // PALING AWAL: cegah robot loncat saat boot
  ledStartSlowBlink();
  Serial.begin(115200);
  delay(500);
  Serial.println("# FIRMWARE TEST IMU + HC-SR04 + MOTOR + SERVO (protokol biner)");
  actuatorInit();
  sonarInit();
  imuInit();
  parser_init(&rx);
  while (Serial.available()) Serial.read();  // buang perintah yang menumpuk saat kalibrasi
  ledStopSlowBlink();
  Serial.println("# Siap. Mulai kirim frame telemetri.");
}

void loop() {
  // kedip cepat = tanda loop berjalan
  static uint32_t lastLedMs = 0;
  if (millis() - lastLedMs >= LED_FAST_BLINK_MS) {
    lastLedMs = millis();
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState);
  }

  imuUpdate();
  sonarUpdate();

  while (Serial.available()) {
    if (parser_feed(&rx, (uint8_t)Serial.read())) handleFrame();
  }

  // failsafe motor
  if (!failsafe && millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    setMotor(0, 0);
    failsafe = true;
  }

  if (millis() - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = millis();
    sendTelemetry();
  }
}
