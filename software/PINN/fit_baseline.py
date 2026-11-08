# ============================================================
# fit_baseline.py
# ============================================================

# Baseline rate-dynamics model, fitted by multi-step rollout.
#
# Per axis, two states and three parameters:
#
#     s_dot     = (tau_axis - s) / tau_m      actuator lag
#     omega_dot = k * s - omega / tau_a       rate dynamics
#
# where tau_axis is the mixer torque proxy built from the reconstructed
# motor commands, with thrust taken as the square of the normalized
# command. The prediction is passed through the same low pass filter the
# firmware applies to the gyro before it is compared with the log.
#
# Fitting integrates the model over short windows and compares
# trajectories, so the noisy derivative of the measured rate is never
# needed.

import argparse
import numpy as np
import torch
from pathlib import Path
from tqdm import tqdm

import data


# ============================================================
# Configuration
# ============================================================

TRAIN_LOGS = ["logs0311", "logs0330"]
VAL_LOGS = ["logs0326"]
TEST_LOGS = ["logs0306"]

HORIZONS_MS = [50, 100, 200, 500]

OUTPUT_DIR = Path(__file__).parent / "baseline_plots"

# Sign of each motor's thrust in the axis torque, from computePower()
MIX_MATRIX = torch.tensor(
    [
        [-1.0, -1.0, +1.0, +1.0],   # roll
        [-1.0, +1.0, -1.0, +1.0],   # pitch
        [-1.0, +1.0, +1.0, -1.0],   # yaw
    ]
)


# ============================================================
# Model
# ============================================================

# Physically plausible bounds, so the optimizer cannot escape into a
# degenerate fit by freezing the actuator state.
TAU_M_RANGE = (0.005, 0.300)    # motor and ESC spool-up [s]
TAU_A_RANGE = (0.050, 5.000)    # aerodynamic rate damping [s]


def _bounded(theta, lo, hi):
    return lo + (hi - lo) * torch.sigmoid(theta)


def _unbounded(value, lo, hi):
    x = (value - lo) / (hi - lo)
    return float(np.log(x / (1.0 - x)))


class RateModel(torch.nn.Module):
    """Per-axis first order actuator lag plus damped rate dynamics."""

    def __init__(self, k=3000.0, tau_m=0.03, tau_a=0.5, trim=0.0):

        super().__init__()

        # Gain stays positive through exp(); the time constants are held
        # inside their physical range by a sigmoid.
        self.log_k = torch.nn.Parameter(
            torch.full((3,), float(np.log(k)))
        )
        self.theta_m = torch.nn.Parameter(
            torch.full((3,), _unbounded(tau_m, *TAU_M_RANGE))
        )
        self.theta_a = torch.nn.Parameter(
            torch.full((3,), _unbounded(tau_a, *TAU_A_RANGE))
        )

        # The mixer torque proxy is not zero in trim: motor, prop and CG
        # mismatch means hover sits at a nonzero differential, which the
        # PID holds through its I term.
        self.trim = torch.nn.Parameter(torch.full((3,), float(trim)))

    def taus(self):
        return (
            _bounded(self.theta_m, *TAU_M_RANGE),
            _bounded(self.theta_a, *TAU_A_RANGE),
        )

    def torque(self, u):
        """Mixer torque proxy from normalized motor commands."""

        thrust = u ** 2

        return thrust @ MIX_MATRIX.T.to(u.dtype) - self.trim

    def forward(self, u, dt, omega0):
        """Roll the model out over a window.

        u      [B, N, 4]   normalized motor commands
        dt     [B, N]      per-sample timestep
        omega0 [B, 3]      initial measured rate

        Returns the filtered prediction [B, N, 3].
        """

        k = self.log_k.exp()
        tau_m, tau_a = self.taus()

        tau = self.torque(u)

        # The actuator starts settled, the plant starts at the measured
        # rate, and the output filter starts matched to it.
        s = tau[:, 0, :]
        omega = omega0
        y = omega0

        out = [y]

        for n in range(u.shape[1] - 1):

            h = dt[:, n].unsqueeze(-1)

            s = s + h * (tau[:, n, :] - s) / tau_m
            omega = omega + h * (k * s - omega / tau_a)

            # Same EMA the firmware applies in gyro.h
            y = data.GYRO_ALPHA * y + (1.0 - data.GYRO_ALPHA) * omega

            out.append(y)

        return torch.stack(out, dim=1)

    def report(self):

        tau_m, tau_a = self.taus()

        return (
            self.log_k.exp().detach().numpy(),
            tau_m.detach().numpy(),
            tau_a.detach().numpy(),
            self.trim.detach().numpy(),
        )


# ============================================================
# Data helpers
# ============================================================

def to_tensors(windows):

    u, w, d = windows

    return (
        torch.from_numpy(u),
        torch.from_numpy(w),
        torch.from_numpy(d),
    )


def rollout_loss(model, u, w, d, scale):

    pred = model(u, d, w[:, 0, :])

    return (((pred - w) / scale) ** 2).mean()


# ============================================================
# Training
# ============================================================

def train(model, train_set, val_set, scale, args):

    u, w, d = train_set

    opt = torch.optim.Adam(
        [p for p in model.parameters() if p.requires_grad],
        lr=args.lr
    )

    n = len(u)
    history = []

    for epoch in tqdm(range(args.epochs), desc="training"):

        perm = torch.randperm(n)

        total = 0.0
        batches = 0

        for i in range(0, n, args.batch):

            idx = perm[i:i + args.batch]

            loss = rollout_loss(model, u[idx], w[idx], d[idx], scale)

            opt.zero_grad()
            loss.backward()
            opt.step()

            total += loss.item()
            batches += 1

        with torch.no_grad():
            val = rollout_loss(model, *val_set, scale).item()

        history.append((total / batches, val))

    return history


# ============================================================
# Evaluation
# ============================================================

def horizon_errors(model, windows, horizons):
    """Rollout RMSE per axis at each horizon, against two baselines."""

    u, w, d = windows

    with torch.no_grad():
        pred = model(u, d, w[:, 0, :])

    rows = {}

    for h in horizons:

        if h >= w.shape[1]:
            continue

        err_model = pred[:, h, :] - w[:, h, :]
        err_hold = w[:, 0, :] - w[:, h, :]
        err_zero = -w[:, h, :]

        rows[h] = {
            "model": err_model.pow(2).mean(0).sqrt().numpy(),
            "persistence": err_hold.pow(2).mean(0).sqrt().numpy(),
            "zero": err_zero.pow(2).mean(0).sqrt().numpy(),
        }

    return rows


def print_errors(rows, label):

    print()
    print("=" * 72)
    print(f"Rollout RMSE [deg/s] - {label}")
    print("=" * 72)
    print(
        f"{'horizon':>8}  {'method':<12}"
        f"{'roll':>9}{'pitch':>9}{'yaw':>9}{'mean':>9}"
    )

    for h, methods in rows.items():
        for name in ("model", "persistence", "zero"):
            v = methods[name]
            print(
                f"{h:>6} ms  {name:<12}"
                f"{v[0]:9.2f}{v[1]:9.2f}{v[2]:9.2f}{v.mean():9.2f}"
            )
        print()


# ============================================================
# Plots
# ============================================================

def plot_history(history, out_dir):

    import matplotlib.pyplot as plt

    train_loss = [h[0] for h in history]
    val_loss = [h[1] for h in history]

    fig = plt.figure(figsize=(10, 5))

    plt.plot(train_loss, label="train")
    plt.plot(val_loss, label="validation")

    plt.xlabel("Epoch")
    plt.ylabel("Normalized rollout MSE")
    plt.yscale("log")
    plt.title("Baseline rate model - fit")
    plt.grid(True, alpha=0.3)
    plt.legend()

    fig.savefig(out_dir / "loss.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_rollouts(model, windows, out_dir, count=4):

    import matplotlib.pyplot as plt

    u, w, d = windows

    # Pick the most dynamic held-out windows.
    rank = w.std(dim=1).mean(dim=1).argsort(descending=True)
    picks = rank[:count]

    with torch.no_grad():
        pred = model(u[picks], d[picks], w[picks, 0, :])

    t = np.arange(w.shape[1]) / data.FS

    fig, axs = plt.subplots(
        count, 3,
        figsize=(14, 2.6 * count),
        sharex=True
    )

    for r in range(count):
        for c, axis in enumerate(data.AXES):

            ax = axs[r, c]

            ax.plot(t, w[picks[r], :, c].numpy(),
                    lw=1.0, color="tab:orange", label="measured")
            ax.plot(t, pred[r, :, c].numpy(),
                    lw=1.2, color="tab:blue", ls="--", label="model")

            ax.grid(True, alpha=0.3)

            if r == 0:
                ax.set_title(axis)
            if c == 0:
                ax.set_ylabel("rate [deg/s]")
            if r == 0 and c == 2:
                ax.legend(fontsize=8)

    for c in range(3):
        axs[-1, c].set_xlabel("Time [s]")

    fig.suptitle("Held-out rollouts", y=1.0)
    fig.tight_layout()
    fig.savefig(out_dir / "rollouts.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# ============================================================
# Per-flight gain check
# ============================================================

def per_flight_gains(args, scale, base_state):

    print()
    print("=" * 72)
    print("Per-flight gain (k refitted alone, lags held fixed)")
    print("=" * 72)

    for name in TRAIN_LOGS + VAL_LOGS + TEST_LOGS:

        windows = to_tensors(
            data.dataset([name], length=args.length, stride=args.stride)
        )

        model = RateModel()
        model.load_state_dict(base_state)

        model.theta_m.requires_grad_(False)
        model.theta_a.requires_grad_(False)
        model.trim.requires_grad_(False)

        opt = torch.optim.Adam([model.log_k], lr=args.lr)

        u, w, d = windows
        n = len(u)

        for _ in range(args.gain_epochs):

            perm = torch.randperm(n)

            for i in range(0, n, args.batch):

                idx = perm[i:i + args.batch]

                loss = rollout_loss(model, u[idx], w[idx], d[idx], scale)

                opt.zero_grad()
                loss.backward()
                opt.step()

        k, _, _, _ = model.report()

        print(
            f"  {name}: k = "
            f"{k[0]:8.1f} (roll)  {k[1]:8.1f} (pitch)  {k[2]:8.1f} (yaw)"
        )


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description="Fit the baseline rate-dynamics model."
    )

    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--batch", type=int, default=512)
    parser.add_argument("--lr", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--length", type=int, default=200)
    parser.add_argument("--stride", type=int, default=50)
    parser.add_argument("--eval-length", type=int, default=501)
    parser.add_argument("--eval-stride", type=int, default=250)
    parser.add_argument("--gain-epochs", type=int, default=15)
    parser.add_argument(
        "--fix-tau-m",
        type=float,
        default=None,
        help="hold the motor lag at this value [s] instead of fitting it"
    )
    parser.add_argument("--per-flight", action="store_true")
    parser.add_argument("--no-plots", action="store_true")

    args = parser.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    # --------------------------------------------------------
    # Data
    # --------------------------------------------------------

    print("Loading flights...")

    train_set = to_tensors(
        data.dataset(TRAIN_LOGS, length=args.length, stride=args.stride)
    )
    val_set = to_tensors(
        data.dataset(VAL_LOGS, length=args.length, stride=args.stride)
    )
    test_set = to_tensors(
        data.dataset(
            TEST_LOGS,
            length=args.eval_length,
            stride=args.eval_stride
        )
    )

    print(
        f"  train {len(train_set[0]):,} windows "
        f"({'+'.join(TRAIN_LOGS)})"
    )
    print(f"  val   {len(val_set[0]):,} windows ({'+'.join(VAL_LOGS)})")
    print(f"  test  {len(test_set[0]):,} windows ({'+'.join(TEST_LOGS)})")

    # Per-axis scale so roll does not dominate yaw in the loss.
    scale = train_set[1].reshape(-1, 3).std(dim=0)

    print(
        f"  axis scale [deg/s]: "
        f"{scale[0]:.2f} {scale[1]:.2f} {scale[2]:.2f}"
    )

    # --------------------------------------------------------
    # Fit
    # --------------------------------------------------------

    with torch.no_grad():
        thrust = train_set[0] ** 2
        trim0 = (thrust @ MIX_MATRIX.T).reshape(-1, 3).mean(dim=0)

    model = RateModel()

    with torch.no_grad():
        model.trim.copy_(trim0)

        if args.fix_tau_m is not None:
            model.theta_m.fill_(
                _unbounded(args.fix_tau_m, *TAU_M_RANGE)
            )

    if args.fix_tau_m is not None:
        model.theta_m.requires_grad_(False)

    history = train(model, train_set, val_set, scale, args)

    print(
        f"final loss: train {history[-1][0]:.4f}, "
        f"val {history[-1][1]:.4f}"
    )

    # --------------------------------------------------------
    # Parameters
    # --------------------------------------------------------

    k, tau_m, tau_a, trim = model.report()

    print()
    print("=" * 72)
    print("Identified parameters")
    print("=" * 72)
    print(
        f"{'axis':<8}{'k [lumped]':>14}{'tau_m [ms]':>14}"
        f"{'tau_a [s]':>14}{'trim':>12}"
    )

    for i, axis in enumerate(data.AXES):
        print(
            f"{axis:<8}{k[i]:14.1f}{tau_m[i] * 1e3:14.1f}"
            f"{tau_a[i]:14.3f}{trim[i]:12.5f}"
        )

    print()
    print("k is J^-1 * thrust-constant in arbitrary units: thrust is never")
    print("measured, so it carries no physical inertia. Calibrate the mass")
    print("and geometry to put a scale on it.")

    # --------------------------------------------------------
    # Evaluation
    # --------------------------------------------------------

    rows = horizon_errors(model, test_set, HORIZONS_MS)

    print_errors(rows, f"held out {'+'.join(TEST_LOGS)}")

    gate = rows.get(100)

    if gate is not None:
        passed = gate["model"].mean() < gate["persistence"].mean()
        print(
            f"gate: model {'beats' if passed else 'DOES NOT BEAT'} "
            f"persistence at 100 ms "
            f"({gate['model'].mean():.2f} vs "
            f"{gate['persistence'].mean():.2f} deg/s)"
        )

    # --------------------------------------------------------
    # Plots and per-flight gains
    # --------------------------------------------------------

    if not args.no_plots:

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        plot_history(history, OUTPUT_DIR)
        plot_rollouts(model, test_set, OUTPUT_DIR)

        print()
        print(f"plots written to {OUTPUT_DIR}")

    if args.per_flight:
        per_flight_gains(args, scale, model.state_dict())


if __name__ == "__main__":
    main()
