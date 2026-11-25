# TrashTalk ESP32

Firmware robot diferensial berbasis ESP32 + program PC (C, Linux) yang saling bicara lewat USB serial
memakai **protokol biner** (header + CRC16).

- **ESP32 → PC** (20 Hz): roll/pitch/yaw, gyro, akselerasi linear, jarak ultrasonik, status motor & servo.
- **PC → ESP32**: kecepatan motor kiri/kanan, stop, sudut servo, reset yaw.

## Daftar isi

- [Struktur repo](#struktur-repo)
- [Hardware & wiring](#hardware--wiring)
- [Cara menjalankan](#cara-menjalankan)
- [Perilaku firmware](#perilaku-firmware)
- [Protokol serial](#protokol-serial)
- [Troubleshooting](#troubleshooting)

## Struktur repo

```
trashtalk/
├── platformio.ini        # konfigurasi PlatformIO (board esp32doit-devkit-v1, lib MPU6050)
├── include/
│   └── protocol.h        # DEFINISI PROTOKOL: frame, payload, CRC16, parser (dipakai firmware & PC)
├── src/
│   └── main.cpp          # firmware ESP32
└── pc/
    ├── robot_teleop.c    # kontrol motor/servo dari keyboard + tampilan telemetri
    ├── robot_rx.c        # monitor telemetri saja (tidak mengirim perintah)
    └── serial_port.h     # buka port serial (raw 115200, lepas DTR/RTS)
```

`include/protocol.h` adalah satu-satunya sumber kebenaran format protokol. Jika diubah,
compile ulang **firmware dan program PC**.

## Hardware & wiring

ESP32 DevKit V1 (30 pin), driver motor L298N, 2 motor DC, MPU6050 (I2C), HC-SR04, 1 servo (SG90/MG90S).

| GPIO | Ke | Catatan |
|---|---|---|
| 13 | L298N ENA | PWM motor kiri, 1 kHz, 8 bit |
| 12 | L298N IN1 | **strapping pin**, harus LOW saat boot (pasang pull-down 10k jika gagal boot) |
| 14 | L298N IN2 | |
| 26 | L298N IN3 | |
| 25 | L298N IN4 | |
| 33 | L298N ENB | PWM motor kanan, 1 kHz, 8 bit |
| 32 | Servo (sinyal) | 50 Hz, pulsa 500–2400 µs |
| 21 | MPU6050 SDA | I2C 400 kHz, alamat 0x68 (AD0 ke GND) |
| 22 | MPU6050 SCL | |
| 5 | HC-SR04 TRIG | |
| 18 | HC-SR04 ECHO | **wajib pembagi tegangan**: 1k seri + 2k ke GND (5V → 3.3V) |
| 2 | LED onboard | indikator status |

**Daya**

- USB laptop → ESP32. 3V3 ESP32 → MPU6050. VIN (5V) → HC-SR04.
- Baterai 7–12 V → input 12V L298N → motor. Regulator 5V L298N → servo.
- Jumper ENA/ENB di L298N **dicabut** (PWM dari ESP32).
- 5V L298N **jangan** disambung ke ESP32 selama USB dipakai.
- Semua GND disatukan (kabel GND pendek & tebal dari baterai → L298N → ESP32).

## Cara menjalankan

### 1. Firmware ESP32

Butuh [PlatformIO](https://platformio.org/) (CLI atau ekstensi VSCode). Library MPU6050
(`electroniccats/MPU6050`) diunduh otomatis.

```bash
pio run                    # compile
pio run -t upload          # upload ke ESP32
```

Setelah upload, **biarkan robot diam dan rata** selama kalibrasi IMU (beberapa detik, LED kedip lambat).
Begitu LED kedip cepat, firmware siap.

> `pio device monitor` hanya akan menampilkan karakter acak setelah boot, karena datanya biner.
> Gunakan program PC di bawah.

### 2. Program PC (Linux)

```bash
cd pc
gcc -O2 -Wall -I../include -o robot_teleop robot_teleop.c
gcc -O2 -Wall -I../include -o robot_rx robot_rx.c
```

Cari port ESP32: `ls /dev/ttyACM* /dev/ttyUSB*` (chip CH9102 → `ttyACM0`, CP2102/CH340 → `ttyUSB0`).
User harus ada di grup `dialout`: `sudo usermod -aG dialout $USER`, lalu login ulang.

Hanya satu program yang boleh membuka port pada satu waktu (tutup `pio device monitor` dulu).

#### `robot_teleop`: kontrol keyboard

```bash
./robot_teleop /dev/ttyACM0 --speed 150
```

| Tombol | Fungsi |
|---|---|
| `w` / `s` | maju / mundur |
| `a` / `d` | putar kiri / kanan di tempat |
| spasi / `x` | stop |
| `+` / `-` | kecepatan ±10 (0..255) |
| `j` / `l` | servo −10° / +10° |
| `k` | servo ke 90° |
| `z` | reset yaw ke 0 |
| `q` / Ctrl+C | keluar (motor otomatis distop) |

Gerak bersifat "terkunci": tekan `w` sekali, robot terus maju sampai ditekan stop atau arah lain.
Program mengirim ulang perintah motor tiap 100 ms agar failsafe ESP32 tidak aktif.

Layar menampilkan perintah yang dikirim, status motor/servo yang dilaporkan ESP32, telemetri lengkap,
umur data, dan statistik frame (ok / CRC error / hilang).

#### `robot_rx`: monitor saja

```bash
./robot_rx /dev/ttyACM0
```

Mencetak satu baris per frame telemetri. Tidak mengirim perintah apa pun, jadi motor tetap diam.

## Perilaku firmware

| Fitur | Keterangan |
|---|---|
| Boot aman | Semua pin motor di-set LOW paling awal di `setup()` agar robot tidak loncat. |
| Failsafe motor | Motor berhenti jika tidak ada `CMD_MOTOR`/`CMD_STOP` selama **500 ms**. Servo tidak terkena failsafe. |
| IMU | MPU6050 dengan DMP (fusi sensor di chip). Yaw dari gyro saja → **drift pelan** (tidak ada magnetometer). |
| HC-SR04 | Non-blocking: trigger tiap 50 ms, durasi echo via interrupt. Rentang valid 2–400 cm. Pembacaan invalid → nilai valid sebelumnya dipakai. |
| Loop | Tanpa `delay()`, semua tugas berbasis timer `millis()`. |
| Perintah saat boot | Perintah yang masuk selama kalibrasi dibuang. |

**LED onboard (GPIO2)**

| Pola | Arti |
|---|---|
| Kedip lambat (1 Hz) | Boot / kalibrasi IMU, robot harus diam |
| Kedip cepat (5 Hz) | Loop berjalan normal |
| Diam (nyala / mati) | Loop macet, atau ESP32 tertahan reset |

Konstanta yang sering diubah ada di bagian atas `src/main.cpp`: `SEND_PERIOD_MS`, `SONAR_PERIOD_MS`,
`DIST_MIN_CM`/`DIST_MAX_CM`, `PWM_FREQ`, `SERVO_MIN_US`/`SERVO_MAX_US`, `SERVO_START_DEG`.

## Protokol serial

UART 115200 baud, 8N1. Semua data dalam **frame biner**; arah ESP32 → PC dan PC → ESP32 memakai
susunan yang sama. Pesan teks berawalan `#` hanya muncul saat ESP32 boot dan otomatis diabaikan parser.

### Susunan frame

```
+------+------+------+-----+-----+-------------+-------+-------+
| 0xAA | 0x55 | TYPE | LEN | SEQ | PAYLOAD ... | CRC_L | CRC_H |
+------+------+------+-----+-----+-------------+-------+-------+
   1      1      1      1     1     LEN byte       1       1
```

| Field | Keterangan |
|---|---|
| `AA 55` | Header, penanda awal frame |
| `TYPE` | Jenis frame (tabel di bawah) |
| `LEN` | Panjang payload dalam byte (maks 128) |
| `SEQ` | Nomor urut 0..255, naik tiap frame; dipakai PC untuk menghitung frame hilang |
| `PAYLOAD` | Data, **little-endian**, struct `packed` |
| `CRC` | CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`) dari `TYPE` s/d akhir `PAYLOAD`, byte rendah dulu |

Frame dengan CRC salah atau `LEN` tidak sesuai tipe dibuang diam-diam (tidak ada ACK/NACK).
Parser adalah state machine per byte, sehingga sinkron ulang otomatis setelah data rusak.

### Tipe frame

| TYPE | Arah | Nama | LEN | Payload |
|---|---|---|---|---|
| `0x01` | ESP32 → PC | `FRAME_TYPE_TELEMETRY` | 62 | `TelemetryPayload` |
| `0x10` | PC → ESP32 | `CMD_MOTOR` | 4 | `int16 left`, `int16 right` (−255..255, negatif = mundur, 0 = lepas) |
| `0x11` | PC → ESP32 | `CMD_STOP` | 0 | – |
| `0x12` | PC → ESP32 | `CMD_SERVO` | 1 | `uint8 deg` (0..180) |
| `0x13` | PC → ESP32 | `CMD_RESET_YAW` | 0 | – |

Nilai di luar batas dipotong oleh ESP32 (mis. 300 → 255).

### `TelemetryPayload` (62 byte)

| Offset | Tipe | Field | Satuan / arti |
|---|---|---|---|
| 0 | `uint32` | `t_ms` | `millis()` ESP32 |
| 4 | `float` | `roll` | derajat |
| 8 | `float` | `pitch` | derajat |
| 12 | `float` | `yaw` | derajat, −180..180 |
| 16 | `float` | `gx` | gyro X, deg/s |
| 20 | `float` | `gy` | gyro Y, deg/s |
| 24 | `float` | `gz` | gyro Z, deg/s (kecepatan belok) |
| 28 | `float` | `lax` | akselerasi linear X frame sensor, m/s² (tanpa gravitasi) |
| 32 | `float` | `lay` | ″ Y |
| 36 | `float` | `laz` | ″ Z |
| 40 | `float` | `wax` | akselerasi linear X frame dunia, m/s² |
| 44 | `float` | `way` | ″ Y |
| 48 | `float` | `waz` | ″ Z |
| 52 | `float` | `dist_cm` | jarak HC-SR04, cm (`-1` = belum pernah valid) |
| 56 | `int16` | `motor_l` | kecepatan motor kiri yang sedang dipakai |
| 58 | `int16` | `motor_r` | kecepatan motor kanan yang sedang dipakai |
| 60 | `uint8` | `servo_deg` | sudut servo sekarang |
| 61 | `uint8` | `flags` | bit0 `TLM_FLAG_FAILSAFE` (motor dihentikan failsafe), bit1 `TLM_FLAG_IMU_OK` |

Frame telemetri total 69 byte × 20 Hz ≈ 1,4 kB/s.

### Contoh byte perintah

```
CMD_MOTOR kiri 150, kanan 150   : AA 55 10 04 00  96 00  96 00  5F 93
CMD_MOTOR kiri -100, kanan 200  : AA 55 10 04 01  9C FF  C8 00  76 B3
CMD_STOP                        : AA 55 11 00 02  8D 98
CMD_SERVO 90°                   : AA 55 12 01 03  5A  D3 EB
CMD_RESET_YAW                   : AA 55 13 00 04  2B 96
```

(`96 00` = 150; `9C FF` = −100 sebagai int16 little-endian.)

### Aturan pemakaian

1. **Keepalive wajib.** Selama robot harus bergerak, kirim ulang `CMD_MOTOR` minimal tiap < 500 ms
   (disarankan 100 ms). Jika tidak, ESP32 menghentikan motor dan menyalakan flag `FAILSAFE`.
2. **Tidak ada ACK.** Untuk memastikan perintah diterima, baca `motor_l`, `motor_r`, `servo_deg`, `flags`
   di telemetri.
3. **Endianness.** Payload little-endian; program PC mengasumsikan host little-endian (x86/ARM).

### Memakai protokol di program C lain

```c
#include "protocol.h"     // -I../include
#include "serial_port.h"

int fd = serial_open("/dev/ttyACM0", 0);

// kirim
uint8_t buf[FRAME_OVERHEAD + FRAME_MAX_PAYLOAD];
CmdMotorPayload m = {150, 150};
write(fd, buf, frame_build(buf, CMD_MOTOR, seq++, &m, sizeof(m)));

// terima
FrameParser p;
parser_init(&p);
// untuk tiap byte b dari read():
if (parser_feed(&p, b) && p.type == FRAME_TYPE_TELEMETRY && p.len == sizeof(TelemetryPayload)) {
  TelemetryPayload t;
  memcpy(&t, p.payload, sizeof(t));
}
```

Fungsi siap pakai `robot_motor()`, `robot_stop()`, `robot_servo()`, `robot_reset_yaw()` ada di
`pc/robot_teleop.c`.

## Troubleshooting

| Gejala | Penyebab & solusi |
|---|---|
| ESP32 seperti freeze / LED mati saat program PC dibuka | Sinyal DTR/RTS menahan pin EN (reset). `serial_port.h` sudah melepas DTR/RTS; ESP32 mungkin reset sekali lalu kalibrasi ulang. Jika masih, tekan tombol EN. |
| ESP32 reset tiap program PC dijalankan | Perilaku rangkaian auto-reset DevKit. Robot harus diam selama kalibrasi ulang. |
| Port hilang dari `/dev` | Chip USB-serial kehilangan daya/kontak: kabel/colokan longgar, servo atau beban lain menarik daya dari 5V USB, arus motor lewat GND USB. Pantau `journalctl -kf \| grep -i usb` sambil menjalankan robot. Pasang kapasitor 100 nF di terminal motor dan 470–1000 µF di input L298N. |
| Port pindah ke `ttyACM1` | USB tersambung ulang saat program lama masih membuka port. Tutup program, cek `ls /dev/ttyACM*`. |
| `Gagal buka ... Permission denied` | Tambah user ke grup `dialout`, lalu login ulang. |
| Telemetri "belum ada data" | ESP32 masih kalibrasi, port salah, atau baud bukan 115200. |
| `I2C scan` tidak menemukan `0x68` | Cek kabel SDA/SCL/VCC/GND MPU6050. Firmware tetap jalan tanpa IMU (flag `IMU_OK` = 0). |
| Jarak selalu `--` | Cek pembagi tegangan ECHO, VCC HC-SR04 dari VIN 5V, dan GND bersama. |
| Gagal boot setelah wiring L298N | GPIO12 (IN1) tertarik HIGH saat boot; pasang pull-down 10k ke GND. |
| Motor berputar terbalik | Tukar kabel motor di OUT1/OUT2 atau OUT3/OUT4 (bukan ubah kode). |
| Satu motor tidak jalan | Ukur tegangan pin ENA/IN di L298N; tukar motor ke channel lain untuk memisahkan masalah motor vs driver. |
