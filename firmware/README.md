# Firmware

Arduino sketches for the Teensy 4.1. In the Arduino IDE a sketch folder must have the
same name as its `.ino` file, so keep that convention when adding sketches.

| Sketch | Purpose |
|--------|---------|
| `flight_controller/` | Flight firmware: SBUS input, gyro, 1 kHz rate PID, motor mixer, DShot300 output, SD logging |
| `tests/gyro_test/` | IMU bring-up and SD logging of gyro data |
| `tests/rx_test/` | SBUS receiver decoding |
| `tests/rx_gyro_test/` | Receiver + gyro together |
| `tests/esc_rx_gyro_test/` | Receiver + gyro + ESC output (no PID) |

## Flight controller modules

| File | Role |
|------|------|
| `flight_controller.ino` | Setup, main 1 kHz loop, serial commands |
| `gyro.h` | SPI IMU setup and gyro read + filtering |
| `rx.h` | SBUS decoding, frame-lost / failsafe detection |
| `pid.h` | Rate PID per axis (gains at the top of the file) |
| `motor_mix.h` | Quad-X mixer, idle/max limits |
| `esc.h` | Bit-banged DShot300 on pins 2-5 |
| `sd_card.h` | Binary flight log on the built-in microSD, USB download |
| `spi.h` | SPI register read/write helpers |

**Serial commands (USB):** `q` close the log file and stop, `d` download all logs
(refused while armed). Use `tools/download_logs.py` rather than typing `d` by hand.

## Building

1. Arduino IDE 2.x + [Teensyduino](https://www.pjrc.com/teensy/td_download.html).
2. Board: *Teensy 4.1*, CPU speed 600 MHz.
3. The flight controller only uses the Teensy core (`SPI`, `SD`). No extra library is needed.

## Libraries

`libraries/` is a copy of Arduino libraries used by earlier experiments. The Arduino
IDE does **not** read it: it uses `Documents/Arduino/libraries`. It will be removed;
install from the Library Manager instead:

| Library | Version |
|---------|---------|
| Adafruit BNO055 | 1.6.4 |
| Adafruit BusIO | 1.17.4 |
| Adafruit Unified Sensor | 1.1.15 |
| ArduinoJson | 7.4.2 |
| ArxTypeTraits | 0.3.2 |
| Eigen (ArduinoEigen) | 0.3.2 |
| PPM-reader | 1.2.0 |
| PPM | 1.1.5 |
| SparkFun BMP581 Arduino Library | 1.0.1 |

## Versions

See [`../CHANGELOG.md`](../CHANGELOG.md). Tag every version you fly: `git tag -a fw-v11 -m "..."`.
