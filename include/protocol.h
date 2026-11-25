// ===== Protokol biner ESP32 <-> PC =====
// File ini dipakai BERSAMA oleh firmware (src/main.cpp) dan program PC (pc/*.c).
// Ubah format di sini saja, lalu compile ulang keduanya.
//
// Susunan SATU frame (dua arah sama; multi-byte little-endian):
//
//   offset  ukuran  isi
//   0       1       HDR0 = 0xAA
//   1       1       HDR1 = 0x55
//   2       1       TYPE  (lihat FRAME_TYPE_* / CMD_*)
//   3       1       LEN   = panjang payload (byte)
//   4       1       SEQ   = nomor urut 0..255 (deteksi frame hilang)
//   5       LEN     PAYLOAD
//   5+LEN   2       CRC16-CCITT (low byte dulu), dihitung dari TYPE s/d akhir PAYLOAD
//
// ESP32 -> PC (20 Hz):
//   0x01 TELEMETRY  TelemetryPayload (62 byte)
//
// PC -> ESP32:
//   0x10 CMD_MOTOR      CmdMotorPayload: int16 kiri, int16 kanan (-255..255, negatif = mundur)
//   0x11 CMD_STOP       (tanpa payload)  stop kedua motor
//   0x12 CMD_SERVO      CmdServoPayload: uint8 sudut 0..180
//   0x13 CMD_RESET_YAW  (tanpa payload)  yaw = 0
//
// Failsafe: ESP32 menghentikan motor jika tidak menerima CMD_MOTOR/CMD_STOP
// selama CMD_TIMEOUT_MS. PC wajib mengirim ulang perintah motor (keepalive ~100 ms).
// Servo tidak terkena failsafe.
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <string.h>

#define FRAME_HDR0 0xAA
#define FRAME_HDR1 0x55
#define FRAME_OVERHEAD 7  // HDR0 HDR1 TYPE LEN SEQ + CRC(2)
#define FRAME_MAX_PAYLOAD 128

// tipe frame
#define FRAME_TYPE_TELEMETRY 0x01
#define CMD_MOTOR 0x10
#define CMD_STOP 0x11
#define CMD_SERVO 0x12
#define CMD_RESET_YAW 0x13

#define CMD_TIMEOUT_MS 500

// bit TelemetryPayload.flags
#define TLM_FLAG_FAILSAFE 0x01  // motor sedang dihentikan oleh failsafe
#define TLM_FLAG_IMU_OK 0x02    // IMU terdeteksi & DMP jalan

typedef struct __attribute__((packed)) {
  uint32_t t_ms;            // millis() ESP32 saat frame dibuat
  float roll, pitch, yaw;   // derajat, yaw -180..180
  float gx, gy, gz;         // gyro, deg/s
  float lax, lay, laz;      // akselerasi linear frame sensor, m/s^2
  float wax, way, waz;      // akselerasi linear frame dunia, m/s^2
  float dist_cm;            // jarak HC-SR04, cm (-1 = belum pernah valid)
  int16_t motor_l, motor_r; // kecepatan motor yang SEDANG dipakai (-255..255)
  uint8_t servo_deg;        // sudut servo sekarang
  uint8_t flags;            // TLM_FLAG_*
} TelemetryPayload;

typedef struct __attribute__((packed)) {
  int16_t left, right;  // -255..255
} CmdMotorPayload;

typedef struct __attribute__((packed)) {
  uint8_t deg;  // 0..180
} CmdServoPayload;

#ifdef __cplusplus
static_assert(sizeof(TelemetryPayload) == 62, "ukuran TelemetryPayload harus 62 byte");
#else
_Static_assert(sizeof(TelemetryPayload) == 62, "ukuran TelemetryPayload harus 62 byte");
#endif

// ---------------- CRC ----------------
// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF. Bisa dipanggil bertahap.
static inline uint16_t crc16_ccitt(uint16_t crc, const uint8_t *data, size_t len) {
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

// ---------------- bangun frame ----------------
// Tulis frame ke out (ukuran minimal FRAME_OVERHEAD + len). Return panjang frame.
static inline size_t frame_build(uint8_t *out, uint8_t type, uint8_t seq,
                                 const void *payload, uint8_t len) {
  out[0] = FRAME_HDR0;
  out[1] = FRAME_HDR1;
  out[2] = type;
  out[3] = len;
  out[4] = seq;
  if (len) memcpy(&out[5], payload, len);
  uint16_t crc = crc16_ccitt(0xFFFF, &out[2], 3 + (size_t)len);
  out[5 + len] = (uint8_t)(crc & 0xFF);
  out[6 + len] = (uint8_t)(crc >> 8);
  return FRAME_OVERHEAD + (size_t)len;
}

// ---------------- parser (state machine per byte) ----------------
// Byte di luar frame (mis. teks '#' saat boot) otomatis dibuang.
typedef enum { PS_HDR0, PS_HDR1, PS_TYPE, PS_LEN, PS_SEQ, PS_PAYLOAD, PS_CRC_L, PS_CRC_H } ParseState;

typedef struct {
  ParseState state;
  uint8_t type, len, seq, idx;
  uint8_t payload[FRAME_MAX_PAYLOAD];
  uint16_t crc_rx;
  // statistik
  uint32_t ok, crc_err, len_err, lost;
  int last_seq;  // -1 = belum ada
} FrameParser;

static inline void parser_init(FrameParser *p) {
  memset(p, 0, sizeof(*p));
  p->state = PS_HDR0;
  p->last_seq = -1;
}

// Masukkan satu byte. Return true jika satu frame valid lengkap
// (hasil di p->type, p->len, p->seq, p->payload).
static inline bool parser_feed(FrameParser *p, uint8_t b) {
  switch (p->state) {
    case PS_HDR0:
      if (b == FRAME_HDR0) p->state = PS_HDR1;
      break;
    case PS_HDR1:
      if (b == FRAME_HDR1) p->state = PS_TYPE;
      else p->state = (b == FRAME_HDR0) ? PS_HDR1 : PS_HDR0;  // AA AA 55 tetap tersinkron
      break;
    case PS_TYPE:
      p->type = b;
      p->state = PS_LEN;
      break;
    case PS_LEN:
      if (b > FRAME_MAX_PAYLOAD) {  // panjang mustahil -> sinkron ulang
        p->len_err++;
        p->state = PS_HDR0;
      } else {
        p->len = b;
        p->state = PS_SEQ;
      }
      break;
    case PS_SEQ:
      p->seq = b;
      p->idx = 0;
      p->state = p->len ? PS_PAYLOAD : PS_CRC_L;
      break;
    case PS_PAYLOAD:
      p->payload[p->idx++] = b;
      if (p->idx >= p->len) p->state = PS_CRC_L;
      break;
    case PS_CRC_L:
      p->crc_rx = b;
      p->state = PS_CRC_H;
      break;
    case PS_CRC_H: {
      p->crc_rx |= (uint16_t)b << 8;
      p->state = PS_HDR0;
      uint8_t head[3] = {p->type, p->len, p->seq};
      uint16_t crc = crc16_ccitt(0xFFFF, head, 3);
      crc = crc16_ccitt(crc, p->payload, p->len);
      if (crc != p->crc_rx) {
        p->crc_err++;
        return false;
      }
      if (p->last_seq >= 0) p->lost += (uint8_t)(p->seq - p->last_seq - 1);
      p->last_seq = p->seq;
      p->ok++;
      return true;
    }
  }
  return false;
}

#endif
