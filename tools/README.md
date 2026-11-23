# Tools

Run from anywhere; paths are resolved relative to the repository.

| Script | What it does |
|--------|--------------|
| `download_logs.py` | Connects to the Teensy over USB serial, downloads every binary log from the SD card and converts it to CSV in `data/logs/` |
| `plot_logs.py` | Plots gyro rates vs. setpoints, PID terms and loop dt for one log; saves to `data/plots/` |

```bash
python -m pip install -r requirements.txt
python tools/download_logs.py
python tools/plot_logs.py
```

Settings are at the top of each script: `SERIAL_PORT` (default `COM4`) in
`download_logs.py`, and `filename` (log to plot) in `plot_logs.py`.
Both expect binary-format logs (firmware `fw-v3`, and `fw-v7` or later).
