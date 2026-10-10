# requires "pip install pyserial"

import serial
import serial.tools.list_ports
import time
from pathlib import Path


BAUDRATE = 115200
OUTPUT_DIR = Path("logs")
COMMAND = b"d"


def find_teensy():
    """
    Try to automatically find the Teensy USB serial port.
    """

    ports = list(serial.tools.list_ports.comports())

    if not ports:
        raise RuntimeError("No serial ports found.")

    print("Available serial ports:")

    for port in ports:
        print(f"  {port.device} - {port.description}")

    # Try to identify Teensy automatically.
    for port in ports:
        description = (port.description or "").lower()
        manufacturer = (port.manufacturer or "").lower()
        product = (port.product or "").lower()

        text = f"{description} {manufacturer} {product}"

        if "teensy" in text:
            print(f"\nTeensy found on {port.device}")
            return port.device

    # If automatic detection fails, ask the user.
    print("\nCould not automatically identify the Teensy.")

    return input("Enter the Teensy COM port manually: ").strip()


def read_line(ser):
    """
    Read one ASCII line from the Teensy.
    """
    line = ser.readline()

    if not line:
        raise RuntimeError("Timeout waiting for Teensy.")

    return line.decode("utf-8", errors="replace").strip()


def receive_exactly(ser, size, output_file):
    """
    Receive exactly 'size' bytes from the Teensy.

    Returns the number of bytes received.
    """

    remaining = size
    total_received = 0

    while remaining > 0:
        chunk_size = min(4096, remaining)

        data = ser.read(chunk_size)

        if not data:
            raise RuntimeError(
                f"Timeout while downloading file. "
                f"Received {total_received}/{size} bytes."
            )

        output_file.write(data)

        total_received += len(data)
        remaining -= len(data)

        # Progress display
        percentage = total_received * 100 / size

        print(
            f"\r    {total_received:,}/{size:,} bytes "
            f"({percentage:6.2f}%)",
            end="",
            flush=True,
        )

    print()

    return total_received


def main():

    print("================================")
    print("     Teensy SD Card Downloader")
    print("================================")
    print()

    # Find Teensy.
    port = find_teensy()

    print(f"\nOpening {port}...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=BAUDRATE,
            timeout=2,
            write_timeout=2,
        )
    except serial.SerialException as e:
        raise RuntimeError(f"Could not open {port}: {e}")

    try:

        # Teensy may reset / reconnect when USB is opened.
        time.sleep(1)

        # Clear old USB data.
        ser.reset_input_buffer()
        ser.reset_output_buffer()

        print("Requesting SD card files...")

        # Send download command.
        ser.write(COMMAND)
        ser.flush()

        # Create output directory.
        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        files_downloaded = 0
        total_bytes = 0

        while True:

            line = read_line(ser)

            print(f"Teensy: {line}")

            # -----------------------------
            # END OF TRANSFER
            # -----------------------------

            if line == "DONE":
                break

            # -----------------------------
            # ERROR
            # -----------------------------

            if line.startswith("ERROR"):
                raise RuntimeError(f"Teensy reported: {line}")

            # -----------------------------
            # FILE HEADER
            #
            # FILE gyro0001.csv 123456
            # -----------------------------

            if line.startswith("FILE "):

                parts = line.split()

                if len(parts) != 3:
                    raise RuntimeError(
                        f"Invalid FILE header: {line}"
                    )

                filename = parts[1]

                try:
                    file_size = int(parts[2])
                except ValueError:
                    raise RuntimeError(
                        f"Invalid file size: {parts[2]}"
                    )

                # Security: only allow simple filenames.
                safe_filename = Path(filename).name

                if safe_filename != filename:
                    raise RuntimeError(
                        f"Unsafe filename received: {filename}"
                    )

                output_path = OUTPUT_DIR / safe_filename

                print()
                print(f"Downloading: {filename}")
                print(f"Size:        {file_size:,} bytes")
                print(f"Saving to:   {output_path}")

                # Tell Teensy we are ready.
                ser.write(b"R")
                ser.flush()

                # Receive exact file size.
                with open(output_path, "wb") as f:

                    received = receive_exactly(
                        ser,
                        file_size,
                        f,
                    )

                # Verify byte count.
                if received != file_size:
                    raise RuntimeError(
                        f"Size mismatch for {filename}: "
                        f"expected {file_size}, "
                        f"received {received}"
                    )

                print(
                    f"    ✓ Received exactly "
                    f"{received:,} bytes"
                )

                # Wait for END marker.
                end_line = read_line(ser)

                if end_line != "END":
                    raise RuntimeError(
                        f"Expected END, received: {end_line}"
                    )

                print("    ✓ File verified")

                files_downloaded += 1
                total_bytes += received

                continue

            # -----------------------------
            # UNKNOWN MESSAGE
            # -----------------------------

            print(f"Warning: unknown message: {line}")

        print()
        print("================================")
        print("Download completed successfully")
        print("================================")
        print(f"Files downloaded: {files_downloaded}")
        print(f"Total bytes:      {total_bytes:,}")
        print(f"Output directory: {OUTPUT_DIR.resolve()}")

    finally:
        ser.close()


if __name__ == "__main__":
    main()