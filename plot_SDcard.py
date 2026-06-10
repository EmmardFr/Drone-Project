import pandas as pd
import matplotlib.pyplot as plt

filename = "gyro0052"

df = pd.read_csv(f"D:/{filename}.csv")

time_s = df["time_us"] / 1e6

plt.figure(figsize=(12,6))

plt.plot(time_s, df["gyroX"], label="gyroX")
plt.plot(time_s, df["gyroY"], label="gyroY")
plt.plot(time_s, df["gyroZ"], label="gyroZ")

plt.xlabel("Time [s]")
plt.ylabel("Angular rate [deg/s]")
plt.title(filename)
plt.grid(True)
plt.legend()

plt.savefig(f"Teensy Dev Gyro Plots/{filename}.png", bbox_inches="tight", dpi=300)
plt.show()