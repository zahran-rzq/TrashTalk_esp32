// ===== Teleop: kontrol motor & servo robot ESP32 dari keyboard (Linux) =====
// Build : gcc -O2 -Wall -I../include -o robot_teleop robot_teleop.c
// Jalan : ./robot_teleop [port] [--speed N]
//         ./robot_teleop /dev/ttyUSB0 --speed 150
//
// Tombol (mode "kunci": tekan sekali, robot terus jalan sampai ditekan stop):
//   w = maju      s = mundur     a = putar kiri    d = putar kanan
//   spasi / x = STOP
//   + / -  = kecepatan +-10          (0..255)
//   j / l  = servo -10 / +10 derajat   k = servo 90
//   z      = reset yaw               q / Ctrl+C = keluar (motor distop)
//
// Pengaman:
//   - Perintah motor dikirim ulang tiap KEEPALIVE_MS. Jika program mati/USB lepas,
//     failsafe ESP32 menghentikan motor setelah CMD_TIMEOUT_MS (500 ms).
//
// Format frame + parser: include/protocol.h (dipakai bersama firmware).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/select.h>

#include "protocol.h"
#include "serial_port.h"

#define KEEPALIVE_MS 100
#define DISPLAY_MS 100

// ================= API kirim perintah (bisa dipakai ulang di program lain) =================
static uint8_t g_tx_seq = 0;

static int send_frame(int fd, uint8_t type, const void *payload, uint8_t len) {
  uint8_t buf[FRAME_OVERHEAD + FRAME_MAX_PAYLOAD];
  size_t n = frame_build(buf, type, g_tx_seq++, payload, len);
  return write(fd, buf, n) == (ssize_t)n ? 0 : -1;
}

static int robot_motor(int fd, int left, int right) {
  CmdMotorPayload c;
  c.left = (int16_t)(left < -255 ? -255 : left > 255 ? 255 : left);
  c.right = (int16_t)(right < -255 ? -255 : right > 255 ? 255 : right);
  return send_frame(fd, CMD_MOTOR, &c, sizeof(c));
}

static int robot_stop(int fd) { return send_frame(fd, CMD_STOP, NULL, 0); }

static int robot_servo(int fd, int deg) {
  CmdServoPayload c;
  c.deg = (uint8_t)(deg < 0 ? 0 : deg > 180 ? 180 : deg);
  return send_frame(fd, CMD_SERVO, &c, sizeof(c));
}

static int robot_reset_yaw(int fd) { return send_frame(fd, CMD_RESET_YAW, NULL, 0); }

// ================= serial & terminal =================

static struct termios g_term_old;
static bool g_term_raw = false;

static void term_raw(void) {
  if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &g_term_old) < 0) return;
  struct termios t = g_term_old;
  t.c_lflag &= ~(ICANON | ECHO);  // baca per tombol, tanpa echo (Ctrl+C tetap jalan)
  t.c_cc[VMIN] = 0;
  t.c_cc[VTIME] = 0;
  tcsetattr(STDIN_FILENO, TCSANOW, &t);
  g_term_raw = true;
}

static void term_restore(void) {
  if (g_term_raw) tcsetattr(STDIN_FILENO, TCSANOW, &g_term_old);
  g_term_raw = false;
}

static double now_s(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static volatile sig_atomic_t running = 1;
static void on_sigint(int s) { (void)s; running = 0; }

// ================= main =================
typedef enum { DIR_STOP, DIR_FWD, DIR_BACK, DIR_LEFT, DIR_RIGHT } Dir;
static const char *DIR_NAME[] = {"STOP", "MAJU", "MUNDUR", "KIRI", "KANAN"};

int main(int argc, char **argv) {
  const char *dev = "/dev/ttyUSB0";
  int speed = 150;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--speed") && i + 1 < argc) speed = atoi(argv[++i]);
    else if (argv[i][0] != '-') dev = argv[i];
    else {
      fprintf(stderr, "Pakai: %s [port] [--speed N]\n", argv[0]);
      return 1;
    }
  }

  int fd = serial_open(dev, 0);
  if (fd < 0) {
    fprintf(stderr, "Gagal buka %s: %s\n", dev, strerror(errno));
    fprintf(stderr, "Cek port (ls /dev/ttyUSB*) dan izin (sudo usermod -aG dialout $USER).\n");
    return 1;
  }

  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = on_sigint;  // tanpa SA_RESTART: select() terputus saat Ctrl+C
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  term_raw();

  FrameParser p;
  parser_init(&p);
  TelemetryPayload tlm;
  memset(&tlm, 0, sizeof(tlm));
  tlm.dist_cm = -1;
  double tlm_time = 0;  // 0 = belum pernah terima

  Dir dir = DIR_STOP;
  int servo = 90;
  double last_tx = 0, last_disp = 0;
  bool motor_dirty = true;

  while (running) {
    // ---- tunggu data serial / tombol, maks 20 ms ----
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(fd, &rfds);
    if (g_term_raw) FD_SET(STDIN_FILENO, &rfds);
    struct timeval tv = {0, 20000};
    int r = select(fd + 1, &rfds, NULL, NULL, &tv);
    if (r < 0 && errno != EINTR) { perror("select"); break; }

    // ---- terima telemetri ----
    if (r > 0 && FD_ISSET(fd, &rfds)) {
      uint8_t buf[256];
      ssize_t n = read(fd, buf, sizeof(buf));
      if (n < 0 && errno != EAGAIN && errno != EINTR) { perror("read serial"); break; }
      for (ssize_t i = 0; i < n; i++) {
        if (parser_feed(&p, buf[i]) && p.type == FRAME_TYPE_TELEMETRY &&
            p.len == sizeof(TelemetryPayload)) {
          memcpy(&tlm, p.payload, sizeof(tlm));
          tlm_time = now_s();
        }
      }
    }

    // ---- baca tombol ----
    if (r > 0 && g_term_raw && FD_ISSET(STDIN_FILENO, &rfds)) {
      char c;
      while (read(STDIN_FILENO, &c, 1) == 1) {
        Dir nd = dir;
        switch (c) {
          case 'w': nd = DIR_FWD; break;
          case 's': nd = DIR_BACK; break;
          case 'a': nd = DIR_LEFT; break;
          case 'd': nd = DIR_RIGHT; break;
          case ' ': case 'x': nd = DIR_STOP; break;
          case '+': case '=': speed = speed + 10 > 255 ? 255 : speed + 10; motor_dirty = true; break;
          case '-': case '_': speed = speed - 10 < 0 ? 0 : speed - 10; motor_dirty = true; break;
          case 'j': servo = servo - 10 < 0 ? 0 : servo - 10; robot_servo(fd, servo); break;
          case 'l': servo = servo + 10 > 180 ? 180 : servo + 10; robot_servo(fd, servo); break;
          case 'k': servo = 90; robot_servo(fd, servo); break;
          case 'z': robot_reset_yaw(fd); break;
          case 'q': running = 0; break;
        }
        if (nd != dir) { dir = nd; motor_dirty = true; }
      }
    }

    // ---- hitung kecepatan ----
    int left = 0, right = 0;
    switch (dir) {
      case DIR_FWD:   left = speed;  right = speed;  break;
      case DIR_BACK:  left = -speed; right = -speed; break;
      case DIR_LEFT:  left = -speed; right = speed;  break;
      case DIR_RIGHT: left = speed;  right = -speed; break;
      default: break;
    }

    // ---- kirim perintah motor (langsung saat berubah + keepalive) ----
    double t = now_s();
    if (motor_dirty || t - last_tx >= KEEPALIVE_MS / 1000.0) {
      if (left == 0 && right == 0) robot_stop(fd);
      else robot_motor(fd, left, right);
      last_tx = t;
      motor_dirty = false;
    }

    // ---- tampilan ----
    if (t - last_disp >= DISPLAY_MS / 1000.0) {
      last_disp = t;
      double age = tlm_time > 0 ? t - tlm_time : -1;
      printf("\033[H\033[J");  // bersihkan layar
      printf("== ROBOT TELEOP  %s ==\n", dev);
      printf("w/s maju/mundur  a/d putar  spasi stop  +/- speed  j/l/k servo  z reset yaw  q keluar\n\n");
      printf("Perintah : %-6s speed %3d  -> kiri %4d kanan %4d\n", DIR_NAME[dir], speed, left, right);
      printf("Servo    : %3d deg\n\n", servo);
      if (age < 0) {
        printf("Telemetri: belum ada data (ESP32 masih boot/kalibrasi?)\n");
      } else {
        printf("Telemetri: umur %.2f s%s\n", age, age > 0.5 ? "  !! DATA TERLAMBAT" : "");
        printf("  ESP32  : motor %4d %4d  servo %3u%s%s\n", tlm.motor_l, tlm.motor_r, tlm.servo_deg,
               (tlm.flags & TLM_FLAG_FAILSAFE) ? "  FAILSAFE" : "",
               (tlm.flags & TLM_FLAG_IMU_OK) ? "" : "  IMU TIDAK ADA");
        printf("  RPY    : %7.2f %7.2f %7.2f deg\n", tlm.roll, tlm.pitch, tlm.yaw);
        printf("  Gyro   : %7.1f %7.1f %7.1f deg/s\n", tlm.gx, tlm.gy, tlm.gz);
        printf("  LinAcc : %6.2f %6.2f %6.2f m/s2\n", tlm.lax, tlm.lay, tlm.laz);
        printf("  WldAcc : %6.2f %6.2f %6.2f m/s2\n", tlm.wax, tlm.way, tlm.waz);
        if (tlm.dist_cm < 0) printf("  Jarak  : --\n");
        else printf("  Jarak  : %.1f cm\n", tlm.dist_cm);
      }
      printf("\nFrame ok %u  crc_err %u  hilang %u\n", p.ok, p.crc_err, p.lost);
      fflush(stdout);
    }
  }

  // ---- keluar: stop motor (kirim beberapa kali agar pasti sampai) ----
  for (int i = 0; i < 3; i++) {
    robot_stop(fd);
    usleep(20000);
  }
  tcdrain(fd);
  term_restore();
  printf("\nMotor distop. Keluar.\n");
  close(fd);
  return 0;
}
