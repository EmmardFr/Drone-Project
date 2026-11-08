# ============================================================
# closed_loop.py
# ============================================================

# Closed-loop simulation of controller plus plant, and identification of
# the plant parameters from flight logs.
#
# The controller and the mixer are exact replicas of the firmware and
# have no free parameters. Only the plant is learned. The simulation is
# driven by the pilot's sticks and throttle, which are the only signals
# in the log that the feedback loop does not generate, so the fit cannot
# recover the controller instead of the dynamics.
#
#   setpoint ->  PID (pid.h)  ->  mixer (motor_mix.h)  ->  plant
#                  ^                                        |
#                  +------- gyro filter (gyro.h) <----------+

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

OUTPUT_DIR = Path(__file__).parent / "closed_loop_plots"

# Thrust sign of each motor in the axis torque, from computePower()
MIX_MATRIX = torch.tensor(
    [
        [-1.0, -1.0, +1.0, +1.0],   # roll
        [-1.0, +1.0, -1.0, +1.0],   # pitch
        [-1.0, +1.0, +1.0, -1.0],   # yaw
    ]
)

# Signs applied to the per-axis command when forming each motor command
MOTOR_SIGNS = torch.tensor(
    [
        [-1.0, -1.0, -1.0],   # m1
        [-1.0, +1.0, +1.0],   # m2
        [+1.0, -1.0, +1.0],   # m3
        [+1.0, +1.0, -1.0],   # m4
    ]
)

U_LIMITS = torch.tensor(
    [data.U_LIMIT["Roll"], data.U_LIMIT["Pitch"], data.U_LIMIT["Yaw"]]
)

RC_D = 1.0 / (2.0 * np.pi * data.D_CUTOFF_HZ)

# Keeps a diverging simulation from going non-finite mid-rollout.
OMEGA_CLAMP = 2000.0

TAU_M_RANGE = (0.002, 0.300)
TAU_A_RANGE = (0.050, 5.000)


def _bounded(theta, lo, hi):
    return lo + (hi - lo) * torch.sigmoid(theta)


def _unbounded(value, lo, hi):
    x = (value - lo) / (hi - lo)
    return float(np.log(x / (1.0 - x)))


# ============================================================
# Plant
# ============================================================

class Plant(torch.nn.Module):
    """Learned rigid-body rate dynamics with per-motor thrust lag.

        T_dot     = (c - T) / tau_m                 thrust spool-up
        tau       = k * (MIX @ T) - trim            axis torque
        omega_dot = tau - omega / tau_a + cross     Euler rotation
    """

    def __init__(self, k=1500.0, tau_m=0.03, tau_a=0.3):

        super().__init__()

        self.log_k = torch.nn.Parameter(
            torch.full((3,), float(np.log(k)))
        )

        # One spool-up constant: the four motors are identical.
        self.theta_m = torch.nn.Parameter(
            torch.tensor(_unbounded(tau_m, *TAU_M_RANGE))
        )
        self.theta_a = torch.nn.Parameter(
            torch.full((3,), _unbounded(tau_a, *TAU_A_RANGE))
        )

        # omega x J omega, as inertia ratios. Zero for a perfectly
        # symmetric airframe, so that is where it starts.
        self.cross = torch.nn.Parameter(torch.zeros(3))

        # Hover is not at zero differential: motor, prop and CG mismatch.
        self.trim = torch.nn.Parameter(torch.zeros(3))

    def constants(self):

        return (
            self.log_k.exp(),
            _bounded(self.theta_m, *TAU_M_RANGE),
            _bounded(self.theta_a, *TAU_A_RANGE),
        )

    def derivative(self, omega, thrust):
        """omega_dot [B, 3] from body rate and the four motor thrusts."""

        k, _, tau_a = self.constants()

        torque = k * (thrust @ MIX_MATRIX.T.to(thrust.dtype)) - self.trim

        wx, wy, wz = omega[:, 0], omega[:, 1], omega[:, 2]

        # deg/s * deg/s -> deg/s^2 needs one conversion back to radians
        gyroscopic = torch.stack(
            [
                self.cross[0] * wy * wz,
                self.cross[1] * wz * wx,
                self.cross[2] * wx * wy,
            ],
            dim=1,
        ) * (np.pi / 180.0)

        return torque - omega / tau_a + gyroscopic

    def report(self):

        k, tau_m, tau_a = self.constants()

        return {
            "k": k.detach().numpy(),
            "tau_m": float(tau_m.detach()),
            "tau_a": tau_a.detach().numpy(),
            "cross": self.cross.detach().numpy(),
            "trim": self.trim.detach().numpy(),
        }


# ============================================================
# Simulation
# ============================================================

def simulate(plant, batch, steps=None, follow_log=False):
    """Run controller and plant together over a batch of windows.

    follow_log=True feeds the *measured* rate to the controller instead
    of the simulated one, which isolates the PID replica from the plant.

    Returns simulated rate [B, N, 3] and controller terms [B, N, 3, 3].
    """

    sp = batch["setpoint"]
    thr = batch["throttle"]
    dt = batch["dt"]
    gains = batch["gains"]
    meas = batch["omega"]

    n = steps or sp.shape[1]

    gp, gi, gd = gains[:, :, 0], gains[:, :, 1], gains[:, :, 2]

    # --------------------------------------------------------
    # States
    # --------------------------------------------------------

    integral = batch["integ0"]
    d_filt = batch["dfilt0"]

    thrust = batch["thrust0"]

    omega = meas[:, 0, :]        # plant state
    y = meas[:, 0, :]            # filtered gyro the controller sees
    y_prev = meas[:, 0, :]

    _, tau_m, _ = plant.constants()

    out_omega = [y]
    out_terms = []

    for i in range(n):

        h = dt[:, i].unsqueeze(-1)

        # ----------------------------------------------------
        # computePid() from pid.h
        # ----------------------------------------------------

        hc = h.clamp(*data.DT_CLAMP)

        gyro = meas[:, i, :] if follow_log else y
        gyro_prev = meas[:, i - 1, :] if (follow_log and i > 0) else y_prev

        error = sp[:, i, :] - gyro

        p_term = gp * error

        integral = (integral + error * hc).clamp(
            -data.I_LIMIT, data.I_LIMIT
        )
        i_term = gi * integral

        alpha_d = hc / (RC_D + hc)
        raw_d = -(gyro - gyro_prev) / hc
        d_filt = d_filt + alpha_d * (raw_d - d_filt)
        d_term = gd * d_filt

        out_terms.append(torch.stack([p_term, i_term, d_term], dim=-1))

        # ----------------------------------------------------
        # computePower() from motor_mix.h
        # ----------------------------------------------------

        u = (p_term + i_term + d_term).clamp(
            -U_LIMITS.to(sp.dtype), U_LIMITS.to(sp.dtype)
        )

        throttle_cmd = (
            data.MOTOR_IDLE
            + thr[:, i].unsqueeze(-1) * (data.MOTOR_MAX - data.MOTOR_IDLE)
        )

        motors = (
            throttle_cmd + u @ MOTOR_SIGNS.T.to(u.dtype)
        ).clamp(data.MOTOR_IDLE, data.MOTOR_MAX)

        command = (
            (motors - data.MOTOR_IDLE)
            / (data.MOTOR_MAX - data.MOTOR_IDLE)
        ) ** 2

        # ----------------------------------------------------
        # Plant
        # ----------------------------------------------------

        thrust = thrust + h * (command - thrust) / tau_m

        omega = (
            omega + h * plant.derivative(omega, thrust)
        ).clamp(-OMEGA_CLAMP, OMEGA_CLAMP)

        # gyro.h low pass
        y_prev = y
        y = data.GYRO_ALPHA * y + (1.0 - data.GYRO_ALPHA) * omega

        out_omega.append(y)

    return (
        torch.stack(out_omega[:n], dim=1),
        torch.stack(out_terms, dim=1),
    )


# ============================================================
# Data helpers
# ============================================================

def to_tensors(arrays):
    return {k: torch.from_numpy(v) for k, v in arrays.items()}


def slice_batch(batch, idx=None, steps=None):

    out = {}

    for k, v in batch.items():

        x = v if idx is None else v[idx]

        if steps is not None and x.dim() >= 2 and x.shape[1] > steps:
            if k in ("setpoint", "throttle", "omega", "dt", "terms"):
                x = x[:, :steps]

        out[k] = x

    return out


def loss_fn(plant, batch, steps, scale):

    sim, _ = simulate(plant, batch, steps=steps)

    return (
        ((sim - batch["omega"][:, :steps]) / scale) ** 2
    ).mean()


# ============================================================
# PID replica check
# ============================================================

def replica_check(batch, limit=256):
    """Drive the replica with the logged rate and compare its terms."""

    sub = slice_batch(batch, idx=torch.arange(min(limit, len(batch["omega"]))))

    plant = Plant()

    with torch.no_grad():
        _, terms = simulate(plant, sub, follow_log=True)

    logged = sub["terms"]

    print()
    print("=" * 72)
    print("PID replica vs logged controller terms (driven by measured rate)")
    print("=" * 72)
    print(f"{'axis':<8}{'P rms':>12}{'I rms':>12}{'D rms':>12}{'of signal':>14}")

    for a, axis in enumerate(data.AXES):

        row = f"{axis:<8}"
        ref = 0.0

        for t in range(3):

            err = (terms[:, :, a, t] - logged[:, :, a, t])
            rms = err.pow(2).mean().sqrt().item()
            ref = max(ref, logged[:, :, a, t].pow(2).mean().sqrt().item())

            row += f"{rms:12.3f}"

        print(row + f"{ref:14.3f}")


# ============================================================
# Training
# ============================================================

def train(plant, train_batch, val_batch, scale, args):

    opt = torch.optim.Adam(plant.parameters(), lr=args.lr)

    n = len(train_batch["omega"])
    history = []

    stages = [
        (int(s), int(e))
        for s, e in zip(args.stages, args.stage_epochs)
    ]

    for steps, epochs in stages:

        desc = f"window {steps} ms"

        for _ in tqdm(range(epochs), desc=desc):

            perm = torch.randperm(n)
            total, batches = 0.0, 0

            for i in range(0, n, args.batch):

                sub = slice_batch(train_batch, perm[i:i + args.batch])

                loss = loss_fn(plant, sub, steps, scale)

                if not torch.isfinite(loss):
                    continue

                opt.zero_grad()
                loss.backward()

                torch.nn.utils.clip_grad_norm_(plant.parameters(), 1.0)
                opt.step()

                total += loss.item()
                batches += 1

            with torch.no_grad():
                val = loss_fn(plant, val_batch, steps, scale).item()

            history.append((steps, total / max(batches, 1), val))

    return history


# ============================================================
# Evaluation
# ============================================================

def horizon_errors(plant, batch, horizons):

    steps = batch["omega"].shape[1]

    with torch.no_grad():
        sim, _ = simulate(plant, batch)

    meas = batch["omega"]

    rows = {}

    for h in horizons:

        if h >= steps:
            continue

        rows[h] = {
            "model": (sim[:, h] - meas[:, h]).pow(2).mean(0).sqrt().numpy(),
            "persistence": (
                meas[:, 0] - meas[:, h]
            ).pow(2).mean(0).sqrt().numpy(),
            "zero": meas[:, h].pow(2).mean(0).sqrt().numpy(),
        }

    return rows


def print_errors(rows, label):

    print()
    print("=" * 72)
    print(f"Closed-loop rollout RMSE [deg/s] - {label}")
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


def print_parameters(plant, args):

    r = plant.report()

    print()
    print("=" * 72)
    print("Identified plant parameters")
    print("=" * 72)
    print(f"motor spool-up tau_m : {r['tau_m'] * 1e3:8.1f} ms")
    print()
    print(f"{'axis':<8}{'k [lumped]':>14}{'tau_a [s]':>12}"
          f"{'cross':>10}{'trim':>10}")

    for i, axis in enumerate(data.AXES):
        print(
            f"{axis:<8}{r['k'][i]:14.1f}{r['tau_a'][i]:12.3f}"
            f"{r['cross'][i]:10.3f}{r['trim'][i]:10.2f}"
        )

    print()

    if args.jxx:

        # k = (thrust constant * arm) / J, so pinning J pins the rest.
        print(
            f"with Jxx = {args.jxx:g} kg*m^2 given: "
            f"roll torque constant = {r['k'][0] * args.jxx:.4g} "
            f"N*m per unit thrust fraction"
        )
        print(
            f"  implied Jyy = {args.jxx * r['k'][0] / r['k'][1]:.5g}, "
            f"Jzz = {args.jxx * r['k'][0] / r['k'][2]:.5g} kg*m^2 "
            f"(equal motor geometry assumed)"
        )

    else:
        print("k is (thrust constant * arm) / J in arbitrary units. Pass")
        print("--jxx with a measured roll inertia to put a scale on it;")
        print("a bifilar pendulum test is enough.")


# ============================================================
# Plots
# ============================================================

def plot_history(history, out_dir):

    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(10, 5))

    plt.plot([h[1] for h in history], label="train")
    plt.plot([h[2] for h in history], label="validation")

    for i in range(1, len(history)):
        if history[i][0] != history[i - 1][0]:
            plt.axvline(i, color="k", ls=":", lw=0.8)

    plt.xlabel("Epoch (dotted: window length increase)")
    plt.ylabel("Normalized simulation MSE")
    plt.yscale("log")
    plt.title("Closed-loop fit")
    plt.grid(True, alpha=0.3)
    plt.legend()

    fig.savefig(out_dir / "loss.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_rollouts(plant, batch, out_dir, count=4):

    import matplotlib.pyplot as plt

    meas = batch["omega"]

    rank = meas.std(dim=1).mean(dim=1).argsort(descending=True)
    picks = rank[:count]

    sub = slice_batch(batch, picks)

    with torch.no_grad():
        sim, _ = simulate(plant, sub)

    t = np.arange(meas.shape[1]) / data.FS

    fig, axs = plt.subplots(count, 3, figsize=(14, 2.6 * count), sharex=True)

    for r in range(count):
        for c, axis in enumerate(data.AXES):

            ax = axs[r, c]

            ax.plot(t, sub["omega"][r, :, c].numpy(), lw=1.0,
                    color="tab:orange", label="measured")
            ax.plot(t, sim[r, :, c].numpy(), lw=1.2,
                    color="tab:blue", ls="--", label="simulated")
            ax.plot(t, sub["setpoint"][r, :, c].numpy(), lw=0.8,
                    color="tab:green", alpha=0.6, label="setpoint")

            ax.grid(True, alpha=0.3)

            if r == 0:
                ax.set_title(axis)
            if c == 0:
                ax.set_ylabel("rate [deg/s]")
            if r == 0 and c == 2:
                ax.legend(fontsize=8)

    for c in range(3):
        axs[-1, c].set_xlabel("Time [s]")

    fig.suptitle("Held-out closed-loop simulation", y=1.0)
    fig.tight_layout()
    fig.savefig(out_dir / "rollouts.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# ============================================================
# Main
# ============================================================

def main():

    parser = argparse.ArgumentParser(
        description="Identify plant parameters by closed-loop simulation."
    )

    parser.add_argument("--batch", type=int, default=256)
    parser.add_argument("--lr", type=float, default=0.05)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--length", type=int, default=500)
    parser.add_argument("--stride", type=int, default=250)
    parser.add_argument(
        "--stages", type=int, nargs="+", default=[100, 250, 500],
        help="window lengths of the curriculum, in samples"
    )
    parser.add_argument(
        "--stage-epochs", type=int, nargs="+", default=[40, 30, 20]
    )
    parser.add_argument(
        "--jxx", type=float, default=None,
        help="measured roll inertia [kg*m^2], to scale the gains"
    )
    parser.add_argument("--no-plots", action="store_true")

    args = parser.parse_args()

    if len(args.stages) != len(args.stage_epochs):
        parser.error("--stages and --stage-epochs must have equal length")

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    print("Loading flights...")

    train_batch = to_tensors(
        data.closed_loop_dataset(
            TRAIN_LOGS, length=args.length, stride=args.stride
        )
    )
    val_batch = to_tensors(
        data.closed_loop_dataset(
            VAL_LOGS, length=args.length, stride=args.stride
        )
    )
    test_batch = to_tensors(
        data.closed_loop_dataset(
            TEST_LOGS, length=args.length, stride=args.stride // 2
        )
    )

    for name, b in (
        ("train", train_batch), ("val", val_batch), ("test", test_batch)
    ):
        print(f"  {name:<6}{len(b['omega']):>6,} windows")

    scale = train_batch["omega"].reshape(-1, 3).std(dim=0)

    # --------------------------------------------------------
    # The controller replica must be right before the plant fit
    # means anything.
    # --------------------------------------------------------

    replica_check(train_batch)

    # --------------------------------------------------------
    # Fit
    # --------------------------------------------------------

    plant = Plant()

    history = train(plant, train_batch, val_batch, scale, args)

    print(
        f"final loss: train {history[-1][1]:.4f}, val {history[-1][2]:.4f}"
    )

    print_parameters(plant, args)

    rows = horizon_errors(plant, test_batch, HORIZONS_MS)
    print_errors(rows, f"held out {'+'.join(TEST_LOGS)}")

    gate = rows.get(100)

    if gate is not None:
        print(
            f"gate: model {'beats' if gate['model'].mean() < gate['persistence'].mean() else 'DOES NOT BEAT'} "
            f"persistence at 100 ms "
            f"({gate['model'].mean():.2f} vs {gate['persistence'].mean():.2f} deg/s)"
        )

    if not args.no_plots:

        OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

        plot_history(history, OUTPUT_DIR)
        plot_rollouts(plant, test_batch, OUTPUT_DIR)

        print()
        print(f"plots written to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
