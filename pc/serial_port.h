// ===== Buka port serial ke ESP32 (Linux), dipakai bersama pc/*.c =====
// Kenapa DTR/RTS dilepas:
//   Board ESP32 DevKit punya rangkaian auto-reset: DTR/RTS chip USB-serial
//   tersambung (lewat 2 transistor) ke pin EN (reset) dan GPIO0 (mode boot).
//   Saat port dibuka, Linux mengaktifkan DTR & RTS. Pada kombinasi tertentu
//   (sering di CH340) EN tertahan LOW -> ESP32 tertahan reset (tampak "freeze",
//   LED mati) atau GPIO0 LOW -> masuk mode download. Dengan melepas DTR & RTS,
//   EN dan GPIO0 kembali HIGH -> ESP32 jalan normal (paling buruk reset sekali).
//   HUPCL dimatikan agar menutup program tidak memicu reset lagi.
#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>

// vmin: 1 = read() blok sampai ada data, 0 = read() langsung kembali (untuk select()).
// Return fd, atau -1 jika gagal (errno terisi).
static inline int serial_open(const char *dev, int vmin) {
  int fd = open(dev, O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;

  // lepas DTR & RTS secepat mungkin, dalam satu panggilan
  int bits = TIOCM_DTR | TIOCM_RTS;
  ioctl(fd, TIOCMBIC, &bits);

  struct termios t;
  if (tcgetattr(fd, &t) < 0) { close(fd); return -1; }
  cfmakeraw(&t);                      // mode biner murni
  cfsetispeed(&t, B115200);
  cfsetospeed(&t, B115200);
  t.c_cflag |= CLOCAL | CREAD;
  t.c_cflag &= ~(CRTSCTS | HUPCL);    // tanpa flow control, tanpa drop DTR saat close
  t.c_cc[VMIN] = vmin ? 1 : 0;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) < 0) { close(fd); return -1; }

  // tcsetattr bisa menyalakan lagi DTR/RTS di beberapa driver -> lepas lagi
  ioctl(fd, TIOCMBIC, &bits);
  tcflush(fd, TCIOFLUSH);
  return fd;
}

#endif
