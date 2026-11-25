// ===== Definisi frame biner telemetri ESP32 -> PC =====
// File ini dipakai BERSAMA oleh firmware (src/main.cpp) dan program PC (pc/robot_rx.c).
// Ubah format di sini saja, lalu compile ulang keduanya.
//
// Susunan frame (semua multi-byte little-endian):
//
//   offset  ukuran  isi
//   0       1       HDR0 = 0xAA
//   1       1       HDR1 = 0x55
//   2       1       TYPE  (0x01 = telemetri)
//   3       1       LEN   = panjang payload (byte)
//   4       1       SEQ   = nomor urut 0..255 (deteksi frame hilang)
//   5       LEN     PAYLOAD (TelemetryPayload)
//   5+LEN   2       CRC16-CCITT (low byte dulu), dihitung dari TYPE s/d akhir PAYLOAD
//
// Total frame telemetri = 5 + 56 + 2 = 63 byte.
#ifndef TELEMETRY_FRAME_H
#define TELEMETRY_FRAME_H

#include <stdint.h>
#include <stddef.h>

#define FRAME_HDR0 0xAA
#define FRAME_HDR1 0x55
#define FRAME_TYPE_TELEMETRY 0x01
#define FRAME_OVERHEAD 7          // HDR0 HDR1 TYPE LEN SEQ + CRC(2)
#define FRAME_MAX_PAYLOAD 128

typedef struct __attribute__((packed)) {
  uint32_t t_ms;           // millis() ESP32 saat frame dibuat
  float roll, pitch, yaw;  // derajat, yaw -180..180
  float gx, gy, gz;        // gyro, deg/s
  float lax, lay, laz;     // akselerasi linear frame sensor, m/s^2
  float wax, way, waz;     // akselerasi linear frame dunia, m/s^2
  float dist_cm;           // jarak HC-SR04, cm (-1 = belum pernah valid)
} TelemetryPayload;

#ifdef __cplusplus
static_assert(sizeof(TelemetryPayload) == 56, "ukuran payload harus 56 byte");
#else
_Static_assert(sizeof(TelemetryPayload) == 56, "ukuran payload harus 56 byte");
#endif

// CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF. Bisa dipanggil bertahap (crc lanjutan).
static inline uint16_t crc16_ccitt(uint16_t crc, const uint8_t *data, size_t len) {
  while (len--) {
    crc ^= (uint16_t)(*data++) << 8;
    for (int i = 0; i < 8; i++)
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}

#endif
