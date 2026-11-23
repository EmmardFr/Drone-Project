# Hardware

KiCad projects, in development order.

| Folder | Status | Description |
|--------|--------|-------------|
| `archive/ismd_flight_computer/` | Obsolete | First all-in-one concept: Teensy 4.1, several IMU/baro options, onboard power |
| `sensor_test_board/` | Done | Board to compare IMUs (ICM-42688-P, BMI088, BMI270, ISM330DLC, BHI360) and barometers (BMP581, DPS368, ICP-20100, MS5611, LPS22DF, ENS220) around a Teensy 4.1 |
| `carrier_pcb_v1/` | Superseded | First carrier board: Teensy 4.1 sockets, IMU module socket, RXSR and ESC connectors, test points |
| `carrier_pcb_v2/` | **Current** | Second carrier revision: JST-SH ESC connector, PicoBlade RXSR connector, Keystone test points |
| `power_regulator/` | In progress | Power regulator board. `reference/` holds third-party reference designs |
| `kicad_libraries/` | - | Symbols and footprints for the sensors and the Teensy 4.1 |

## Conventions

- Each board folder has a `production/` folder with the fabrication outputs
  (gerbers zip, BOM, positions, netlist) as sent to the fab.
- Order a board → tag the commit: `git tag -a hw-carrier-v2 -m "ordered <date>"`.
- KiCad backups (`*-backups/`), `.history/` and `*.kicad_prl` are git-ignored: git is the backup.
- Datasheets live in [`../docs/datasheets/`](../docs/datasheets/).
