import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# ============================================================

# Configuration

# ============================================================

filename = "logs0302"

input_file = Path(f"C:/Users/zorba/Documents/DroneProject/code/software/logs/{filename}.csv")
output_dir = Path("Teensy Dev Gyro Plots")
output_dir.mkdir(parents=True, exist_ok=True)

# ============================================================

# Load CSV

# ============================================================

df = pd.read_csv(input_file)

# Convert time from microseconds to seconds

time_s = df["time_us"] / 1e6

# ============================================================

# 1. Gyroscope

# ============================================================

plt.figure(figsize=(12, 6))

plt.plot(time_s, df["gyroX"], label="gyroX")
plt.plot(time_s, df["gyroY"], label="gyroY")
plt.plot(time_s, df["gyroZ"], label="gyroZ")

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(f"{filename} - Gyroscope")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(
output_dir / f"{filename}_gyro.png",
dpi=300,
bbox_inches="tight"
)

plt.show()

# ============================================================

# 2. Receiver inputs

# ============================================================

plt.figure(figsize=(12, 8))

plt.plot(time_s, df["rxThrottle"], label="Throttle")
plt.plot(time_s, df["rxRoll"], label="Roll")
plt.plot(time_s, df["rxPitch"], label="Pitch")
plt.plot(time_s, df["rxYaw"], label="Yaw")

plt.xlabel("Time [s]")
plt.ylabel("RX command")
plt.title(f"{filename} - Receiver Inputs")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(
output_dir / f"{filename}_rx.png",
dpi=300,
bbox_inches="tight"
)

plt.show()

# ============================================================

# 3. Arm / failsafe / frame loss

# ============================================================

plt.figure(figsize=(12, 5))

plt.step(
time_s,
df["armSwitch"],
where="post",
label="Arm switch"
)

plt.step(
time_s,
df["frameLost"],
where="post",
label="Frame lost"
)

plt.step(
time_s,
df["failsafe"],
where="post",
label="Failsafe"
)

plt.xlabel("Time [s]")
plt.ylabel("State")
plt.title(f"{filename} - Safety States")
plt.yticks([0, 1])
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(
output_dir / f"{filename}_states.png",
dpi=300,
bbox_inches="tight"
)

plt.show()

# ============================================================

# 4. PID outputs

# ============================================================

plt.figure(figsize=(12, 6))

plt.plot(time_s, df["pidRoll"], label="PID Roll")
plt.plot(time_s, df["pidPitch"], label="PID Pitch")
plt.plot(time_s, df["pidYaw"], label="PID Yaw")

plt.xlabel("Time [s]")
plt.ylabel("PID output")
plt.title(f"{filename} - PID Outputs")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(
output_dir / f"{filename}_pid.png",
dpi=300,
bbox_inches="tight"
)

plt.show()

# ============================================================

# 5. Motor outputs

# ============================================================

plt.figure(figsize=(12, 6))

plt.plot(time_s, df["motor1"], label="Motor 1")
plt.plot(time_s, df["motor2"], label="Motor 2")
plt.plot(time_s, df["motor3"], label="Motor 3")
plt.plot(time_s, df["motor4"], label="Motor 4")

plt.xlabel("Time [s]")
plt.ylabel("Motor command")
plt.title(f"{filename} - Motor Outputs")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig(
output_dir / f"{filename}_motors.png",
dpi=300,
bbox_inches="tight"
)

plt.show()