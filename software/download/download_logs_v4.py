# ============================================================
# download_logs.py
# ============================================================

# Requires: pip install pyserial
# take binary files as input (arduino code v3, v7, v8, v9, v10)

import serial
import struct
import csv
from pathlib import Path


# ============================================================
# Configuration
# ============================================================

SERIAL_PORT = "COM4"
BAUDRATE = 115200

LOCAL_DIR = Path(
    "C:/Users/zorba/Documents/DroneProject/code/software/logs"
)

LOCAL_DIR.mkdir(parents=True, exist_ok=True)


# ============================================================
# Binary format
# ============================================================

GAINS_FORMAT = "<9f"
GAINS_SIZE = struct.calcsize(GAINS_FORMAT)

# C++ LogRecord:
#
# uint32_t time_us
# float dt
# 7 floats
# 3 uint8_t
# 1 byte padding
# 9 floats
#
# Total: 76 bytes
#
RECORD_FORMAT = "<If7f3B x9f"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)


CSV_HEADER = [
    "time_us",
    "dt",
    "gyroRoll",
    "gyroPitch",
    "gyroYaw",
    "rxThrottle",
    "desiredRollRate",
    "desiredPitchRate",
    "desiredYawRate",
    "armSwitch",
    "frameLost",
    "failsafe",
    "pRoll",
    "iRoll",
    "dRoll",
    "pPitch",
    "iPitch",
    "dPitch",
    "pYaw",
    "iYaw",
    "dYaw",
    "roll_coeff_p",
    "roll_coeff_i",
    "roll_coeff_d",
    "pitch_coeff_p",
    "pitch_coeff_i",
    "pitch_coeff_d",
    "yaw_coeff_p",
    "yaw_coeff_i",
    "yaw_coeff_d",
]


# ============================================================
# Convert binary file to CSV
# ============================================================

def convert_bin_to_csv(bin_file):

    csv_file = bin_file.with_suffix(".csv")

    print(f"Converting {bin_file.name} -> {csv_file.name}")

    with open(bin_file, "rb") as f:

        # ----------------------------------------------------
        # Read ASCII column header
        # ----------------------------------------------------

        header = f.readline().decode(
            "ascii",
            errors="strict"
        ).strip()

        # ----------------------------------------------------
        # Read PID gains
        # ----------------------------------------------------

        gains_data = f.read(GAINS_SIZE)

        if len(gains_data) != GAINS_SIZE:
            raise RuntimeError(
                f"{bin_file.name}: "
                f"incomplete PID gains header"
            )

        (
            roll_p,
            roll_i,
            roll_d,
            pitch_p,
            pitch_i,
            pitch_d,
            yaw_p,
            yaw_i,
            yaw_d,
        ) = struct.unpack(
            GAINS_FORMAT,
            gains_data
        )

        print(
            f"  Roll PID:  "
            f"P={roll_p:.4f}, "
            f"I={roll_i:.4f}, "
            f"D={roll_d:.4f}"
        )

        print(
            f"  Pitch PID: "
            f"P={pitch_p:.4f}, "
            f"I={pitch_i:.4f}, "
            f"D={pitch_d:.4f}"
        )

        print(
            f"  Yaw PID:   "
            f"P={yaw_p:.4f}, "
            f"I={yaw_i:.4f}, "
            f"D={yaw_d:.4f}"
        )

        # ----------------------------------------------------
        # Write CSV
        # ----------------------------------------------------

        record_count = 0

        with open(
            csv_file,
            "w",
            newline=""
        ) as csv_out:

            writer = csv.writer(csv_out)

            writer.writerow(CSV_HEADER)

            while True:

                data = f.read(RECORD_SIZE)

                if len(data) == 0:
                    break

                if len(data) != RECORD_SIZE:
                    raise RuntimeError(
                        f"{bin_file.name}: "
                        f"incomplete final record: "
                        f"{len(data)} / {RECORD_SIZE} bytes"
                    )

                values = struct.unpack(
                    RECORD_FORMAT,
                    data
                )

                (
                    time_us,
                    dt,

                    gyroRoll,
                    gyroPitch,
                    gyroYaw,

                    rxThrottle,
                    desiredRollRate,
                    desiredPitchRate,
                    desiredYawRate,

                    armSwitch,
                    frameLost,
                    failsafe,

                    pRoll,
                    iRoll,
                    dRoll,

                    pPitch,
                    iPitch,
                    dPitch,

                    pYaw,
                    iYaw,
                    dYaw,
                ) = values

                writer.writerow([
                    time_us,
                    dt,

                    gyroRoll,
                    gyroPitch,
                    gyroYaw,

                    rxThrottle,
                    desiredRollRate,
                    desiredPitchRate,
                    desiredYawRate,

                    armSwitch,
                    frameLost,
                    failsafe,

                    pRoll,
                    iRoll,
                    dRoll,

                    pPitch,
                    iPitch,
                    dPitch,

                    pYaw,
                    iYaw,
                    dYaw,

                    # PID gains
                    roll_p,
                    roll_i,
                    roll_d,

                    pitch_p,
                    pitch_i,
                    pitch_d,

                    yaw_p,
                    yaw_i,
                    yaw_d,
                ])

                record_count += 1

    print(f"  {record_count} records converted.")


# ============================================================
# Download one binary file
# ============================================================

def download_file(ser, filename, file_size):

    local_bin = LOCAL_DIR / filename

    print(
        f"Downloading {filename} "
        f"({file_size:,} bytes)..."
    )

    ser.write(b"R")
    ser.flush()

    remaining = file_size

    with open(local_bin, "wb") as f:

        while remaining > 0:

            chunk = ser.read(
                min(4096, remaining)
            )

            if not chunk:
                raise RuntimeError(
                    f"Timeout while downloading "
                    f"{filename}. "
                    f"Remaining: {remaining} bytes."
                )

            f.write(chunk)
            remaining -= len(chunk)

    # --------------------------------------------------------
    # Read END marker
    # --------------------------------------------------------

    end = ser.read_until(b"\n")

    if end != b"END\n":
        raise RuntimeError(
            f"Unexpected end marker for "
            f"{filename}: {end!r}"
        )

    # --------------------------------------------------------
    # Verify size
    # --------------------------------------------------------

    actual_size = local_bin.stat().st_size

    if actual_size != file_size:
        raise RuntimeError(
            f"Size mismatch for {filename}: "
            f"expected {file_size}, "
            f"received {actual_size}"
        )

    print(
        f"  Saved: {local_bin} "
        f"({actual_size:,} bytes)"
    )

    return local_bin


# ============================================================
# Download all files
# ============================================================

def download_all_files():

    print()
    print("============================================================")
    print("Connecting to Teensy")
    print("============================================================")

    with serial.Serial(
        SERIAL_PORT,
        BAUDRATE,
        timeout=2
    ) as ser:

        ser.reset_input_buffer()
        ser.reset_output_buffer()

        # ----------------------------------------------------
        # Ask Teensy to start file download
        # ----------------------------------------------------

        ser.write(b"d")
        ser.flush()

        print("Waiting for file list...")
        print()

        while True:

            line = ser.readline()

            if not line:
                raise RuntimeError(
                    "Timeout waiting for Teensy."
                )

            line = line.decode(
                "ascii",
                errors="replace"
            ).strip()

            # ------------------------------------------------
            # FILE <filename> <size>
            # ------------------------------------------------

            if line.startswith("FILE "):

                parts = line.split()

                if len(parts) != 3:
                    raise RuntimeError(
                        f"Invalid FILE message: {line}"
                    )

                filename = parts[1]
                file_size = int(parts[2])

                local_bin = LOCAL_DIR / filename

                # ------------------------------------------------
                # Skip if already downloaded and size matches
                # ------------------------------------------------

                if local_bin.exists():

                    local_size = local_bin.stat().st_size

                    if local_size == file_size:

                        print(
                            f"Skipping {filename} "
                            f"(already downloaded)"
                        )

                        ser.write(b"S")
                        ser.flush()

                        continue

                # ------------------------------------------------
                # Download
                # ------------------------------------------------

                bin_file = download_file(
                    ser,
                    filename,
                    file_size
                )

                # ------------------------------------------------
                # Convert immediately
                # ------------------------------------------------

                convert_bin_to_csv(bin_file)

                print()

            # ------------------------------------------------
            # Done
            # ------------------------------------------------

            elif line == "DONE":

                print()
                print("============================================================")
                print("DOWNLOAD COMPLETE")
                print("============================================================")

                break

            else:

                print(f"Teensy: {line}")


# ============================================================
# Main
# ============================================================

if __name__ == "__main__":
    download_all_files()