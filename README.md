# Drone-Project

A from-scratch FPV quadcopter: custom carrier PCB, Teensy 4.1 flight firmware with a
rate (gyro) PID controller, flight-log tooling, and learning-based control research.

**Roadmap:** stable rate mode → attitude/angle mode → position control and autonomous
flight → reinforcement-learning controllers → custom ESC and flight-controller boards.

## System overview

```
 Radio TX ──► RXSR receiver ──SBUS──► Teensy 4.1 ◄──SPI── IMU (gyro)
                                         │   │
                              microSD ◄──┘   └──DShot300──► 4-in-1 ESC ──► 4 motors
                                         ▲
          LiPo ──► buck converter ───────┘   (all on the carrier PCB)
```

| Function        | Implementation                                           |
|-----------------|----------------------------------------------------------|
| MCU             | Teensy 4.1 (600 MHz Cortex-M7)                           |
| Receiver        | FrSky RXSR, SBUS on `Serial1` (100 kbaud, 8E2, inverted) |
| IMU             | SPI gyro, chip select on pin 10                          |
| ESC             | 4-in-1 ESC, bit-banged DShot300 on pins 2, 3, 4, 5       |
| Control loop    | 1 kHz rate PID (roll / pitch / yaw) + motor mixer        |
| Logging         | Binary log on the built-in microSD, downloaded over USB  |

## Repository layout

| Folder      | Contents |
|-------------|----------|
| [`firmware/`](firmware/) | Arduino sketches for the Teensy: the flight controller and bench test sketches |
| [`hardware/`](hardware/) | KiCad projects (carrier PCB revisions, sensor test board, power regulator) |
| [`tools/`](tools/)       | Python scripts to download flight logs from the drone and plot them |
| [`ml/`](ml/)             | Learning experiments: RL (CartPole for now) and a physics-informed plant model (PINN) |
| [`data/`](data/)         | Sample flight logs (versioned) and local raw logs/plots (git-ignored) |
| [`docs/`](docs/)         | Component datasheets and project documentation |

## Quick start

**Flash the flight controller**
1. Install the Arduino IDE with [Teensyduino](https://www.pjrc.com/teensy/td_download.html).
2. Open `firmware/flight_controller/flight_controller.ino`, select *Teensy 4.1*, upload.
3. Libraries: see [`firmware/README.md`](firmware/README.md).

**Download and plot a flight log**
```bash
python -m pip install -r requirements.txt
python tools/download_logs.py      # drone connected over USB, disarmed
python tools/plot_logs.py          # set the log name at the top of the script
```

## Versioning

- Firmware versions are git tags: `fw-v1` … `fw-v10`. See [`CHANGELOG.md`](CHANGELOG.md).
  - Compare two versions: `git diff fw-v8 fw-v10 -- firmware/flight_controller`
  - Get an old version back: `git checkout fw-v7 -- firmware/flight_controller`
- The state of the repo before the restructuring (with all `_vN` folder copies) is the tag `pre-cleanup`.
- **Do not copy a folder to make a new version.** Edit in place, commit, and tag
  versions you fly: `git tag -a fw-v11 -m "short description"`.

### Commit messages

Prefix with the area that changed, then say what changed:

```
fw: add notch filter on gyro
hw: carrier v2 - move SBUS pad
tools: plot_logs reads binary logs
ml: PPO on cartpole with KL early stop
docs: wiring diagram
```

## License

[MIT](LICENSE)
