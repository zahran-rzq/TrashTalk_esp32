// ===== Penerima & parser frame telemetri ESP32 (Linux) =====
// Build : gcc -O2 -Wall -I../include -o robot_rx robot_rx.c
// Jalan : ./robot_rx /dev/ttyUSB0          (default /dev/ttyUSB0, 115200 baud)
//
// Format frame: lihat include/telemetry_frame.h
// Parser = state machine per byte: cari header 0xAA 0x55, baca TYPE/LEN/SEQ/PAYLOAD,
// cek CRC16. Byte di luar frame (mis. teks '#' saat boot ESP32) otomatis dibuang.
// Asumsi: PC little-endian (x86/ARM umumnya), sama dengan ESP32.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>

#include "telemetry_frame.h"

// ---------------- parser ----------------
typedef enum { S_HDR0, S_HDR1, S_TYPE, S_LEN, S_SEQ, S_PAYLOAD, S_CRC_L, S_CRC_H } ParseState;

typedef struct {
  ParseState state;
  uint8_t type, len, seq, idx;
  uint8_t payload[FRAME_MAX_PAYLOAD];
  uint16_t crc_rx;
  // statistik
  uint32_t ok, crc_err, len_err, lost;
  int last_seq;  // -1 = belum ada
} Parser;

static void parser_init(Parser *p) {
  memset(p, 0, sizeof(*p));
  p->state = S_HDR0;
  p->last_seq = -1;
}

// Masukkan satu byte. Return true jika satu frame valid lengkap (data di p->type/len/payload).
static bool parser_feed(Parser *p, uint8_t b) {
  switch (p->state) {
    case S_HDR0:
      if (b == FRAME_HDR0) p->state = S_HDR1;
      break;
    case S_HDR1:
      if (b == FRAME_HDR1) p->state = S_TYPE;
      else p->state = (b == FRAME_HDR0) ? S_HDR1 : S_HDR0;  // 0xAA 0xAA 0x55 tetap tersinkron
      break;
    case S_TYPE:
      p->type = b;
      p->state = S_LEN;
      break;
    case S_LEN:
      if (b > FRAME_MAX_PAYLOAD) {  // panjang mustahil -> sinkron ulang
        p->len_err++;
        p->state = S_HDR0;
      } else {
        p->len = b;
        p->state = S_SEQ;
      }
      break;
    case S_SEQ:
      p->seq = b;
      p->idx = 0;
      p->state = p->len ? S_PAYLOAD : S_CRC_L;
      break;
    case S_PAYLOAD:
      p->payload[p->idx++] = b;
      if (p->idx >= p->len) p->state = S_CRC_L;
      break;
    case S_CRC_L:
      p->crc_rx = b;
      p->state = S_CRC_H;
      break;
    case S_CRC_H: {
      p->crc_rx |= (uint16_t)b << 8;
      p->state = S_HDR0;
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

// ---------------- serial ----------------
static int serial_open(const char *dev) {
  int fd = open(dev, O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;
  struct termios t;
  if (tcgetattr(fd, &t) < 0) { close(fd); return -1; }
  cfmakeraw(&t);                 // mode biner murni: tanpa echo, tanpa konversi \r\n
  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~CRTSCTS;
  t.c_cc[VMIN] = 1;              // read() blok sampai ada minimal 1 byte
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }
  tcflush(fd, TCIOFLUSH);
  return fd;
}

static volatile sig_atomic_t running = 1;
static void on_sigint(int s) { (void)s; running = 0; }

// ---------------- main ----------------
int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  int fd = serial_open(dev);
  if (fd < 0) {
    fprintf(stderr, "Gagal buka %s: %s\n", dev, strerror(errno));
    fprintf(stderr, "Cek port (ls /dev/ttyUSB*) dan izin (sudo usermod -aG dialout $USER).\n");
    return 1;
  }
  // sigaction tanpa SA_RESTART: Ctrl+C memutus read() yang sedang blok (signal() glibc tidak)
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_sigint;
  sigaction(SIGINT, &sa, NULL);
  printf("Membaca %s ... Ctrl+C untuk keluar.\n", dev);

  Parser p;
  parser_init(&p);
  uint8_t buf[256];

  while (running) {
    ssize_t n = read(fd, buf, sizeof(buf));
    if (n < 0) {
      if (errno == EINTR) continue;
      perror("read");
      break;
    }
    for (ssize_t i = 0; i < n; i++) {
      if (!parser_feed(&p, buf[i])) continue;

      if (p.type == FRAME_TYPE_TELEMETRY && p.len == sizeof(TelemetryPayload)) {
        TelemetryPayload t;
        memcpy(&t, p.payload, sizeof(t));  // memcpy: aman dari masalah alignment
        printf("t=%8u seq=%3u | RPY %7.2f %7.2f %7.2f | G %7.1f %7.1f %7.1f"
               " | LA %6.2f %6.2f %6.2f | WA %6.2f %6.2f %6.2f | Jarak ",
               t.t_ms, p.seq, t.roll, t.pitch, t.yaw, t.gx, t.gy, t.gz,
               t.lax, t.lay, t.laz, t.wax, t.way, t.waz);
        if (t.dist_cm < 0) printf("--");
        else printf("%.2f", t.dist_cm);
        printf(" | ok %u crc_err %u hilang %u\n", p.ok, p.crc_err, p.lost);
      } else {
        printf("# frame tipe 0x%02X len %u tidak dikenal\n", p.type, p.len);
      }
    }
  }

  printf("\nSelesai. ok=%u crc_err=%u len_err=%u hilang=%u\n", p.ok, p.crc_err, p.len_err, p.lost);
  close(fd);
  return 0;
}
