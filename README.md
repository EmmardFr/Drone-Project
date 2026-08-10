# Drone-Project

- `Hardware/` contains hardware design files:
  - `Datasheets/`: component datasheets (IMUs, barometers, and related parts) used during hardware selection.
  - `Graphics/`: project graphics and visual assets.
  - `Schematics/`: standalone schematic libraries/references for key components (e.g., sensors and Teensy 4.1).
  - PCB designs, in development order:
    1. `ismd_flight_computer (obselete)/` (deprecated early prototype): initial all-in-one flight-computer concept combining Teensy 4.1, multiple IMU/barometer options, and onboard power circuitry.
    2. `ismd_fc_sensortest/`: dedicated sensor test board used to compare multiple IMU and barometer options around a Teensy 4.1 base.
    3. `Carrier_PCB/`: first carrier board revision for system integration (power distribution, receiver I/O, ESC/motor signal breakout, and module connectors).
    4. `Carrier_PCB_v2/`: second carrier board revision refining the carrier design and production-ready outputs.
- `software/` contains firmware, libraries, and software tooling.