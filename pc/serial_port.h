// ===== Buka port serial ke ESP32 (Linux), dipakai bersama pc/*.c =====
// Soal auto-reset:
//   Board ESP32 DevKit punya rangkaian auto-reset: DTR/RTS chip USB-serial
//   tersambung (lewat 2 transistor) ke pin EN (reset) dan GPIO0 (mode boot).
//   Linux MENYALAKAN DTR & RTS setiap port dibuka. Yang memicu reset adalah
//   PERUBAHAN DTR/RTS (dua sinyal tidak berubah tepat bersamaan).
//
//   Hasil uji di board ini (CH9102, ttyACM0):
//     - DTR/RTS dibiarkan menyala & tetap menyala setelah port ditutup
//       -> buka port berikutnya TIDAK reset, data langsung masuk.
//     - DTR/RTS dimatikan (atau HUPCL aktif mematikannya saat close)
//       -> buka port berikutnya kernel menyalakannya lagi -> ESP32 RESET.
//
//   Jadi: JANGAN ubah DTR/RTS, dan matikan HUPCL agar DTR/RTS tetap menyala
//   setelah program ditutup. Reset hanya terjadi sekali, di pembukaan pertama
//   setelah USB dicolok.
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

// vmin: 1 = read() blok sampai ada data, 0 = read() langsung kembali (untuk select()).
// Return fd, atau -1 jika gagal (errno terisi).
static inline int serial_open(const char *dev, int vmin) {
  int fd = open(dev, O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;

  struct termios t;
  if (tcgetattr(fd, &t) < 0) { close(fd); return -1; }
  cfmakeraw(&t);                      // mode biner murni
  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~(CRTSCTS | HUPCL);    // tanpa flow control; DTR/RTS tidak dimatikan saat close
  t.c_cc[VMIN] = vmin ? 1 : 0;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }

  tcflush(fd, TCIOFLUSH);  // buang data lama di buffer
  return fd;
}

#endif
