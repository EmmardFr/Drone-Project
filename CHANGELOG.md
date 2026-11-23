# Changelog

Flight-controller firmware (`firmware/flight_controller/`). Each version is a git tag
`fw-vN`. Gains are written as `P / I / D`. Versions v1-v10 were reconstructed from
the former `pid_esc_rx_gyro_vN` folders.

## Unreleased
- Repository restructured (see README). No firmware change.

## fw-v10 - flown
- D-term low-pass cutoff raised from 10 Hz to 25 Hz.
- Loop `dt` clamped to 1-5 ms to avoid I/D spikes on slow iterations.
- Yaw stick rate scale x100 → x200.
- Gains: roll/pitch 1.20 / 3.50 / 0.022, yaw 2.00 / 2.20 / 0.
- Log file starts with a text column header followed by binary records.

## fw-v9
- Failsafe on lost frames: SBUS frame-lost flag must persist 70 ms before triggering.
- `dt` added to the log records.

## fw-v8
- Low-pass filtered D-term (10 Hz cutoff).
- Gyro low-pass filter alpha 0.85 → 0.95.
- I limit 30.
- Gains: roll 1.20 / 4.0 / 0.045, pitch 1.10 / 4.0 / 0.045, yaw 1.40 / 2.2 / 0.04.

## fw-v7
- Back to fixed `dt` = 1 ms, I limit 10, gains 1.0 / 4.0 / 0.
- Binary SD logging re-enabled.

## fw-v6
- Loop `dt` measured with `micros()` instead of assumed 1 ms.
- Gains: roll 1.0 / 5.0 / 0, pitch 1.0 / 4.0 / 0, yaw 1.0 / 5.0 / 0.

## fw-v5
- P-only test (1.0 / 0 / 0), I limit 15.
- PID gains written on every log row.

## fw-v4
- Back to CSV logging.
- Gains 0.80 / 3.50 / 0.0001 (yaw I 5.0), I limit 25.

## fw-v3
- Binary SD logging (`LogRecord` struct) with PID gains in the file header.

## fw-v2
- Accumulating integral with safety limit (10).
- P, I and D contributions logged separately; logs renamed `logsXXXX.csv`.

## fw-v1
- First rate PID controller (moving-window integral), DShot ESC output, SBUS input,
  CSV logging to SD.
