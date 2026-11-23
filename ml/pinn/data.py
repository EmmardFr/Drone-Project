# ============================================================
# data.py
# ============================================================

# Flight log pipeline for rate-dynamics modelling.
#
# Loads the decoded CSV logs, keeps the armed segments, replays the
# firmware mixer to recover the motor commands, and cuts the result into
# fixed-length windows for multi-step rollout fitting.

import numpy as np
import pandas as pd
from pathlib import Path


# ============================================================
# Firmware constants
# ============================================================

# motor_mix.h
MOTOR_IDLE = 130
MOTOR_MAX = 2047

# pid.h / computePower() clamps
U_LIMIT = {"Roll": 1000.0, "Pitch": 1000.0, "Yaw": 2000.0}

# gyro.h low pass filter, 1 kHz loop
GYRO_ALPHA = 0.95

# pid.h, as flown. The sample logs were recorded with firmware v8/v9,
# not with v10 as it stands today: v10 raised the D cutoff to 25 Hz and
# retuned the gains. The replica check in closed_loop.py confirms 10 Hz
# reproduces the logged D term (correlation 0.999, amplitude ratio 1.00,
# against 0.97 / 2.24 at 25 Hz).
I_LIMIT = 30.0
D_CUTOFF_HZ = 10.0
DT_CLAMP = (0.001, 0.005)

# pid_esc_rx_gyro_v10.ino
LOOP_PERIOD_US = 1000.0
FS = 1000.0

# A tick longer than this is an SD stall, not a control cycle.
DT_STALL_S = 0.002

AXES = ["Roll", "Pitch", "Yaw"]
MOTORS = ["m1", "m2", "m3", "m4"]

# Sign of (roll, pitch, yaw) per motor, from computePower()
MIX = {
    "m1": (-1, -1, -1),
    "m2": (-1, +1, +1),
    "m3": (+1, -1, +1),
    "m4": (+1, +1, -1),
}


# ============================================================
# Paths
# ============================================================

def find_log_dir(start=None):
    """Locate software/sample-flight-logs without hardcoding a path."""

    here = Path(start or Path(__file__).parent).resolve()

    sibling = here.parent / "sample-flight-logs"

    if sibling.is_dir():
        return sibling

    for p in [here, *here.parents]:
        candidate = p / "software" / "sample-flight-logs"

        if candidate.is_dir():
            return candidate

    raise FileNotFoundError("sample-flight-logs not found")


# ============================================================
# Loading
# ============================================================

COLS = (
    ["time_us", "dt", "armSwitch", "frameLost", "rxThrottle"]
    + [f"gyro{a}" for a in AXES]
    + [f"desired{a}Rate" for a in AXES]
    + [f"{t}{a}" for a in AXES for t in "pid"]
)

# The 9 gain columns are constant within a file; read once, not per row.
GAIN_COLS = [
    f"{a.lower()}_coeff_{t}" for a in AXES for t in "pid"
]

DTYPES = {c: np.float32 for c in COLS}
DTYPES["time_us"] = np.int64
DTYPES["armSwitch"] = np.uint8
DTYPES["frameLost"] = np.uint8


def load_gains(path):
    """PID gains for one flight, as [3, 3] = axis x (p, i, d)."""

    row = pd.read_csv(path, nrows=1, usecols=GAIN_COLS).iloc[0]

    return np.array(
        [[row[f"{a.lower()}_coeff_{t}"] for t in "pid"] for a in AXES],
        dtype=np.float32,
    )


def load_log(path):
    """Load one flight log and add the reconstructed actuator commands.

    Returns the full frame, including disarmed samples.
    """

    df = pd.read_csv(path, usecols=COLS, dtype=DTYPES)

    df["t"] = df["time_us"] / 1e6

    # --------------------------------------------------------
    # Per-axis controller output (pid.roll/pitch/yaw)
    # --------------------------------------------------------

    for a in AXES:
        df[f"u{a}"] = (
            (df[f"p{a}"] + df[f"i{a}"] + df[f"d{a}"])
            .clip(-U_LIMIT[a], U_LIMIT[a])
        )

    # --------------------------------------------------------
    # Replay computePower()
    # --------------------------------------------------------

    thr = MOTOR_IDLE + df["rxThrottle"] * (MOTOR_MAX - MOTOR_IDLE)

    for m, (sr, sp, sy) in MIX.items():
        df[m] = (
            thr
            + sr * df["uRoll"]
            + sp * df["uPitch"]
            + sy * df["uYaw"]
        ).clip(MOTOR_IDLE, MOTOR_MAX)

    # On a dropped RC frame the firmware re-sent prev_motor* instead of
    # calling the mixer, so the reconstruction does not hold there.
    df.loc[df["frameLost"] > 0, MOTORS] = np.nan

    return df


def armed_segments(df):
    """Split into separate armed flights. Returns a list of frames."""

    a = df["armSwitch"].to_numpy().astype(np.int8)
    edges = np.diff(a)

    starts = list(np.where(edges == 1)[0] + 1)
    ends = list(np.where(edges == -1)[0] + 1)

    if a[0] == 1:
        starts.insert(0, 0)
    if a[-1] == 1:
        ends.append(len(a) - 1)

    return [
        df.iloc[s:e].reset_index(drop=True)
        for s, e in zip(starts, ends)
    ]


def load_flights(names=None, log_dir=None):
    """Load logs by stem name. Returns {name: [segment frames]}."""

    log_dir = Path(log_dir) if log_dir else find_log_dir()

    files = sorted(log_dir.glob("logs*.csv"))

    if names is not None:
        wanted = set(names)
        files = [f for f in files if f.stem in wanted]

        missing = wanted - {f.stem for f in files}

        if missing:
            raise FileNotFoundError(f"no such logs: {sorted(missing)}")

    return {f.stem: armed_segments(load_log(f)) for f in files}


# ============================================================
# Windowing
# ============================================================

def normalized_commands(seg):
    """Motor commands as thrust fraction in [0, 1]."""

    u = seg[MOTORS].to_numpy(dtype=np.float32)

    return (u - MOTOR_IDLE) / (MOTOR_MAX - MOTOR_IDLE)


def make_windows(segments, length=200, stride=50, dt_stall=DT_STALL_S):
    """Cut segments into fixed-length windows.

    Returns (u, omega, dt) with shapes
        u     [B, N, 4]   normalized motor commands
        omega [B, N, 3]   measured body rates, deg/s
        dt    [B, N]      true per-sample timestep, s

    Windows containing a loop stall or an invalid reconstruction are
    dropped.
    """

    U, W, D = [], [], []

    for seg in segments:

        u = normalized_commands(seg)
        w = seg[[f"gyro{a}" for a in AXES]].to_numpy(dtype=np.float32)
        d = seg["dt"].to_numpy(dtype=np.float32)

        n = len(seg)

        for i in range(0, n - length + 1, stride):

            sl = slice(i, i + length)

            du = u[sl]
            dd = d[sl]

            if not np.isfinite(du).all():
                continue

            if (dd > dt_stall).any():
                continue

            U.append(du)
            W.append(w[sl])
            D.append(dd)

    if not U:
        raise ValueError("no usable windows")

    return (
        np.stack(U).astype(np.float32),
        np.stack(W).astype(np.float32),
        np.stack(D).astype(np.float32),
    )


def dataset(names, length=200, stride=50, log_dir=None):
    """Windows for the named logs, concatenated."""

    flights = load_flights(names, log_dir=log_dir)

    segments = [seg for segs in flights.values() for seg in segs]

    return make_windows(segments, length=length, stride=stride)



# ============================================================
# Closed-loop windows
# ============================================================

def closed_loop_dataset(names, length=400, stride=200, log_dir=None):
    """Windows for simulating the controller and plant together.

    Unlike make_windows(), nothing here is taken from the feedback path:
    the simulation is driven by the pilot's sticks and throttle only. The
    logged controller terms come along for the replica check and for
    initialising the PID states.

    Returns a dict of arrays:
        setpoint [B, N, 3]   desired body rates, deg/s
        throttle [B, N]      stick throttle, 0..1
        omega    [B, N, 3]   measured (filtered) body rates, deg/s
        dt       [B, N]      true per-sample timestep, s
        terms    [B, N, 3, 3]  logged P, I, D contributions per axis
        gains    [B, 3, 3]   p, i, d gain per axis
        integ0   [B, 3]      integrator state at window start
        dfilt0   [B, 3]      filtered-derivative state at window start
        thrust0  [B, 4]      per-motor thrust fraction at window start
    """

    log_dir = Path(log_dir) if log_dir else find_log_dir()

    files = sorted(log_dir.glob("logs*.csv"))

    if names is not None:
        wanted = set(names)
        files = [f for f in files if f.stem in wanted]

    out = {k: [] for k in (
        "setpoint", "throttle", "omega", "dt",
        "terms", "gains", "integ0", "dfilt0", "thrust0",
    )}

    for path in files:

        gains = load_gains(path)
        df = load_log(path)

        for seg in armed_segments(df):

            sp = seg[[f"desired{a}Rate" for a in AXES]].to_numpy(np.float32)
            thr = seg["rxThrottle"].to_numpy(np.float32)
            w = seg[[f"gyro{a}" for a in AXES]].to_numpy(np.float32)
            d = seg["dt"].to_numpy(np.float32)
            u = normalized_commands(seg)

            # [N, 3 axes, 3 terms]
            terms = np.stack(
                [seg[[f"{t}{a}" for t in "pid"]].to_numpy(np.float32)
                 for a in AXES],
                axis=1,
            )

            n = len(seg)

            for i in range(0, n - length + 1, stride):

                sl = slice(i, i + length)

                if not np.isfinite(u[sl]).all():
                    continue

                if (d[sl] > DT_STALL_S).any():
                    continue

                # The firmware keeps I and the filtered D as state. Both
                # are recoverable from the log by dividing out the gain,
                # except where a gain is zero (yaw D on some flights).
                with np.errstate(divide="ignore", invalid="ignore"):
                    integ0 = np.where(
                        gains[:, 1] != 0, terms[i, :, 1] / gains[:, 1], 0.0
                    )
                    dfilt0 = np.where(
                        gains[:, 2] != 0, terms[i, :, 2] / gains[:, 2], 0.0
                    )

                out["setpoint"].append(sp[sl])
                out["throttle"].append(thr[sl])
                out["omega"].append(w[sl])
                out["dt"].append(d[sl])
                out["terms"].append(terms[sl])
                out["gains"].append(gains)
                out["integ0"].append(integ0.astype(np.float32))
                out["dfilt0"].append(dfilt0.astype(np.float32))
                out["thrust0"].append(u[i] ** 2)

    if not out["omega"]:
        raise ValueError("no usable windows")

    return {k: np.stack(v).astype(np.float32) for k, v in out.items()}


# ============================================================
# Self check
# ============================================================

if __name__ == "__main__":

    log_dir = find_log_dir()
    print(f"Log directory: {log_dir}")
    print()

    for name, segs in load_flights(log_dir=log_dir).items():

        total = sum(len(s) for s in segs)

        stalls = sum(int((s["dt"] > DT_STALL_S).sum()) for s in segs)
        bad = sum(int(s["m1"].isna().sum()) for s in segs)

        u, w, d = make_windows(segs)

        print(
            f"{name}: {len(segs)} segment(s), {total:,} armed samples, "
            f"{stalls} stall(s), {bad} invalid, "
            f"{len(u):,} windows"
        )
