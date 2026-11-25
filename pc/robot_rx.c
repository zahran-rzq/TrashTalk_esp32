// ===== Penerima & parser frame telemetri ESP32 (Linux) =====
// Build : gcc -O2 -Wall -I../include -o robot_rx robot_rx.c
// Jalan : ./robot_rx /dev/ttyUSB0          (default /dev/ttyUSB0, 115200 baud)
//
// Monitor saja (tidak mengirim perintah). Untuk kontrol motor/servo: robot_teleop.c
// Format frame + parser: include/protocol.h (dipakai bersama firmware).
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

#include "protocol.h"
#include "serial_port.h"

// ---------------- serial ----------------

static volatile sig_atomic_t running = 1;
static void on_sigint(int s) { (void)s; running = 0; }

// ---------------- main ----------------
int main(int argc, char **argv) {
  const char *dev = argc > 1 ? argv[1] : "/dev/ttyUSB0";
  int fd = serial_open(dev, 1);
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

  FrameParser p;
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
        printf(" | M %4d %4d S %3u%s", t.motor_l, t.motor_r, t.servo_deg,
               (t.flags & TLM_FLAG_FAILSAFE) ? " FAILSAFE" : "");
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
