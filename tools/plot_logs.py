# to use with arduino code v3, v7, v8, v9, v10

# ============================================================
# plot_logs.py
# ============================================================

import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path
import numpy as np


# ============================================================
# Configuration
# ============================================================

filename = "logs0337"

input_file = Path(
    f"C:/Users/zorba/Documents/DroneProject/code/software/logs/{filename}.csv"
)

output_dir = Path("Teensy Dev Gyro Plots")
output_dir.mkdir(parents=True, exist_ok=True)


# ============================================================
# Load CSV
# ============================================================

df = pd.read_csv(input_file)

time_s = df["time_us"] / 1e6


# ============================================================
# Read PID gains
# ============================================================

roll_p = df["roll_coeff_p"].iloc[0]
roll_i = df["roll_coeff_i"].iloc[0]
roll_d = df["roll_coeff_d"].iloc[0]

pitch_p = df["pitch_coeff_p"].iloc[0]
pitch_i = df["pitch_coeff_i"].iloc[0]
pitch_d = df["pitch_coeff_d"].iloc[0]

yaw_p = df["yaw_coeff_p"].iloc[0]
yaw_i = df["yaw_coeff_i"].iloc[0]
yaw_d = df["yaw_coeff_d"].iloc[0]


# ============================================================
# Print PID information
# ============================================================

print()
print("============================================================")
print(f"Flight log: {filename}")
print("============================================================")

print(
    f"Roll  PID: P = {roll_p:.4f}, "
    f"I = {roll_i:.4f}, "
    f"D = {roll_d:.4f}"
)

print(
    f"Pitch PID: P = {pitch_p:.4f}, "
    f"I = {pitch_i:.4f}, "
    f"D = {pitch_d:.4f}"
)

print(
    f"Yaw   PID: P = {yaw_p:.4f}, "
    f"I = {yaw_i:.4f}, "
    f"D = {yaw_d:.4f}"
)

print("============================================================")
print()


# ============================================================
# Gyroscope statistics
# ============================================================

print("============================================================")
print("GYROSCOPE STATISTICS")
print("============================================================")

gyro_axes = {
    "Roll": df["gyroRoll"],
    "Pitch": df["gyroPitch"],
    "Yaw": df["gyroYaw"]
}

for axis, data in gyro_axes.items():

    mean = data.mean()
    std = data.std()
    rms = np.sqrt(np.mean(data ** 2))
    minimum = data.min()
    maximum = data.max()
    peak_to_peak = maximum - minimum

    print()
    print(f"{axis}:")
    print(f"    Mean:         {mean:10.4f} deg/s")
    print(f"    Std dev:      {std:10.4f} deg/s")
    print(f"    RMS:          {rms:10.4f} deg/s")
    print(f"    Min:          {minimum:10.4f} deg/s")
    print(f"    Max:          {maximum:10.4f} deg/s")
    print(f"    Peak-to-peak: {peak_to_peak:10.4f} deg/s")

print()
print("============================================================")
print()


# ============================================================
# Loop dt statistics
# ============================================================

dt_us = df["dt"] * 1e6

print("============================================================")
print("CONTROL LOOP DT STATISTICS")
print("============================================================")

print(
    f"Mean dt:          {dt_us.mean():10.3f} us"
)

print(
    f"Std dev:          {dt_us.std():10.3f} us"
)

print(
    f"Min dt:           {dt_us.min():10.3f} us"
)

print(
    f"Max dt:           {dt_us.max():10.3f} us"
)

print(
    f"Mean frequency:   {1.0 / df['dt'].mean():10.2f} Hz"
)

print(
    f"Max frequency:    {1.0 / df['dt'].min():10.2f} Hz"
)

print(
    f"Min frequency:    {1.0 / df['dt'].max():10.2f} Hz"
)

print("============================================================")
print()


# ============================================================
# Helper function
# ============================================================

def save_figure(fig, filename):
    fig.tight_layout()
    fig.savefig(
        output_dir / filename,
        dpi=300,
        bbox_inches="tight"
    )


# ============================================================
# 1. ROLL
# ============================================================

fig_roll_rate = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["gyroRoll"],
    label="Measured roll rate"
)

plt.plot(
    time_s,
    df["desiredRollRate"],
    label="Desired roll rate"
)

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(f"{filename} - Roll Rate Tracking")

plt.grid(True)
plt.legend()

save_figure(
    fig_roll_rate,
    f"{filename}_roll_rate.png"
)


fig_roll_pid = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["dRoll"],
    label="D"
)

plt.plot(
    time_s,
    df["pRoll"],
    label="P"
)

plt.plot(
    time_s,
    df["iRoll"],
    label="I"
)

plt.xlabel("Time [s]")
plt.ylabel("PID contribution")

plt.title(
    f"{filename} - Roll PID Terms "
    f"(P={roll_p:g}, I={roll_i:g}, D={roll_d:g})"
)

plt.grid(True)
plt.legend()

save_figure(
    fig_roll_pid,
    f"{filename}_roll_pid.png"
)


# ============================================================
# 2. PITCH
# ============================================================

fig_pitch_rate = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["gyroPitch"],
    label="Measured pitch rate"
)

plt.plot(
    time_s,
    df["desiredPitchRate"],
    label="Desired pitch rate"
)

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(f"{filename} - Pitch Rate Tracking")

plt.grid(True)
plt.legend()

save_figure(
    fig_pitch_rate,
    f"{filename}_pitch_rate.png"
)


fig_pitch_pid = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["dPitch"],
    label="D"
)

plt.plot(
    time_s,
    df["pPitch"],
    label="P"
)

plt.plot(
    time_s,
    df["iPitch"],
    label="I"
)

plt.xlabel("Time [s]")
plt.ylabel("PID contribution")

plt.title(
    f"{filename} - Pitch PID Terms "
    f"(P={pitch_p:g}, I={pitch_i:g}, D={pitch_d:g})"
)

plt.grid(True)
plt.legend()

save_figure(
    fig_pitch_pid,
    f"{filename}_pitch_pid.png"
)


# ============================================================
# 3. YAW
# ============================================================

fig_yaw_rate = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["gyroYaw"],
    label="Measured yaw rate"
)

plt.plot(
    time_s,
    df["desiredYawRate"],
    label="Desired yaw rate"
)

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(f"{filename} - Yaw Rate Tracking")

plt.grid(True)
plt.legend()

save_figure(
    fig_yaw_rate,
    f"{filename}_yaw_rate.png"
)


fig_yaw_pid = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["dYaw"],
    label="D"
)

plt.plot(
    time_s,
    df["pYaw"],
    label="P"
)

plt.plot(
    time_s,
    df["iYaw"],
    label="I"
)

plt.xlabel("Time [s]")
plt.ylabel("PID contribution")

plt.title(
    f"{filename} - Yaw PID Terms "
    f"(P={yaw_p:g}, I={yaw_i:g}, D={yaw_d:g})"
)

plt.grid(True)
plt.legend()

save_figure(
    fig_yaw_pid,
    f"{filename}_yaw_pid.png"
)


# ============================================================
# 4. ALL MEASURED GYRO AXES
# ============================================================

fig_gyro = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["gyroRoll"],
    label="Roll"
)

plt.plot(
    time_s,
    df["gyroPitch"],
    label="Pitch"
)

plt.plot(
    time_s,
    df["gyroYaw"],
    label="Yaw"
)

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(f"{filename} - Measured Angular Rates")

plt.grid(True)
plt.legend()

save_figure(
    fig_gyro,
    f"{filename}_all_gyro.png"
)


# ============================================================
# 5. RECEIVER / DESIRED RATES
# ============================================================

fig_desired = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    df["desiredRollRate"],
    label="Desired Roll"
)

plt.plot(
    time_s,
    df["desiredPitchRate"],
    label="Desired Pitch"
)

plt.plot(
    time_s,
    df["desiredYawRate"],
    label="Desired Yaw"
)

plt.xlabel("Time [s]")
plt.ylabel("Desired angular rate [deg/s]")
plt.title(f"{filename} - Desired Angular Rates")

plt.grid(True)
plt.legend()

save_figure(
    fig_desired,
    f"{filename}_desired_rates.png"
)

# ============================================================
# 6. DT VS TIME
# ============================================================

fig_dt = plt.figure(figsize=(12, 6))

plt.plot(
    time_s,
    dt_us,
    label="Loop dt"
)

plt.axhline(
    1000,
    linestyle="--",
    label="Target = 1000 us"
)

plt.xlabel("Time [s]")
plt.ylabel("Loop period [us]")
plt.title(f"{filename} - Control Loop dt")

plt.grid(True)
plt.legend()

save_figure(
    fig_dt,
    f"{filename}_dt.png"
)


# ============================================================
# 7. SAFETY STATES
# ============================================================

fig_states = plt.figure(figsize=(12, 5))

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

save_figure(
    fig_states,
    f"{filename}_states.png"
)


# ============================================================
# Show all plots
# ============================================================

plt.show()