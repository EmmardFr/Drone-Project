# Baseline results

Run: `python3 fit_baseline.py --epochs 150 --eval-stride 100 --per-flight`

Train `logs0311` + `logs0330`, validate `logs0326`, hold out `logs0306`.

## Rollout RMSE on held-out logs0306 [deg/s, mean over axes]

| horizon | model | persistence | zero |
|---|---|---|---|
| 50 ms | 5.42 | 6.20 | 6.97 |
| 100 ms | 5.10 | 6.07 | 7.13 |
| 200 ms | 5.73 | 6.97 | 7.07 |
| 500 ms | 6.94 | 8.57 | 6.99 |

The plan's gate (beat persistence at 100 ms) passes. **It is a weak gate and the
model does not deserve the pass.**

## The model barely uses the command

Ablation, same training setup, gain forced to zero so the actuator input cannot
influence anything:

| | val loss | 50 ms | 100 ms | 200 ms |
|---|---|---|---|---|
| full model | 0.154 | 5.43 | 5.18 | 6.16 |
| decay only (k = 0) | 0.164 | 5.72 | 5.48 | 5.89 |

The motor commands buy 0.30 deg/s at 100 ms, about 6%, and *nothing* at 200 ms.
`baseline_plots/rollouts.png` shows what the model actually learned: an exponential
decay of the initial condition toward zero. It beats persistence because the true
rate also trends toward zero, not because it predicts the response to the
actuators.

## Why: the data is closed-loop

The motor commands are computed *from* omega by the PID every tick. A fast
actuator model therefore produces a command-rate correlation of the wrong sign at
short lag, and the optimizer escapes by low-passing the input until it carries no
information. Symptoms, all consistent:

- `tau_m` runs to whatever bound it is given (257-284 ms against a 300 ms cap,
  30-218 **seconds** when unbounded) — the actuator state freezes.
- The fit is nearly flat in `tau_m`: holding it at 20 / 50 / 100 ms costs only
  0.153 -> 0.163 val loss. The data does not identify it.
- Per-flight gain `k` varies 2-3x across the four flights (roll 630-1280). Some of
  that is battery sag, but that spread is too large for one airframe.

The instantaneous correlation between the mixer torque proxy and a Savitzky-Golay
estimate of omega-dot is only 0.11-0.17, so there is signal, but it is weak
relative to the feedback path.

## What to do next

Fit in **closed-loop simulation**: simulate the PID from `pid.h` together with the
plant, drive the pair with the logged stick setpoints (`desired*Rate`, which are
exogenous), and compare simulated omega against measured. The PID gains are in the
logs and the controller is fully known — including the 25 Hz D-term filter and the
I-limit — so this is reconstructable. That removes the feedback correlation, which
is the actual blocker, and it is the right harness for a PINN or a grey-box with
cross-coupling anyway.

Secondary, cheaper: train on windows selected for stick activity rather than all
windows, so the exogenous excitation is not swamped by hover.

## Note on parameters

`k` is `J^-1 * thrust_constant` in arbitrary units. Thrust is never measured, so no
physical moment of inertia can be read off these numbers. Weighing the quad and
computing a slab-model J would fix the scale.

---

# Closed-loop identification

Run: `python3 closed_loop.py --length 600 --stride 300`

`closed_loop.py` simulates the PID (`pid.h`) and the mixer (`motor_mix.h`) as exact
replicas with no free parameters, driven only by the logged stick setpoints and
throttle, with the learned plant and the gyro filter closing the loop. Only the
plant is fitted. Multiple shooting over 600 ms windows, curriculum 100 -> 250 ->
500 samples, PID states initialised from the logged terms.

## The replica check found a firmware mismatch

Driving the controller replica with the *measured* rate should reproduce the logged
P, I and D columns exactly. It did not: D was off by 2.2x in amplitude. The sample
logs were flown with **v8/v9, not v10** — `D_CUTOFF_HZ` was 10 Hz then and v10
raised it to 25 Hz (the gains in the logs differ from v10's constants too).

With the cutoff set to the as-flown 10 Hz:

| axis | P err | I err | D err | signal RMS |
|---|---|---|---|---|
| roll | 0.000 | 0.019 | 0.074 | 10.6 |
| pitch | 0.000 | 0.016 | 0.053 | 25.6 |
| yaw | 0.000 | 0.011 | 0.033 | 16.7 |

Residual under 1% of signal. The controller replica is correct; what remains is the
plant.

## Identified parameters

| axis | k [lumped] | tau_a [s] | cross | trim |
|---|---|---|---|---|
| roll | 36492 | 0.072 | 2.50 | -17.6 |
| pitch | 26459 | 0.149 | -8.57 | -21.1 |
| yaw | 2737 | 1.891 | 0.63 | -23.3 |

`tau_m` (motor spool-up): runs to the lower bound, whatever bound it is given
(5.1 ms against a 5 ms floor, 2.1 ms against a 2 ms floor).

Reproducible: three seeds give the same optimum to within 2% on every parameter.

Sane: yaw gain is 13x smaller than roll and yaw damping 26x slower, which is what
you expect when yaw torque comes from propeller drag reaction rather than thrust
differential.

Not sane: the gyroscopic coefficients should be inertia ratios of order one, and
pitch comes out at -8.6. It is absorbing unmodelled cross-coupling.

## tau_m is now identified, and it wants to be zero

Profile over `tau_m`, refitting everything else at each point:

| tau_m | 5 ms | 10 ms | 20 ms | 30 ms | 50 ms |
|---|---|---|---|---|---|
| val loss | 0.1440 | 0.1460 | 0.1526 | 0.1620 | 0.1835 |

Monotone, 27% across the range — compare the open-loop fit, where the same sweep
was flat and non-monotone. The closed-loop formulation constrains the parameter
that the open-loop one could not.

It prefers zero because the 8 Hz gyro filter sits inside the loop and already
supplies the lag. What the data constrains is the *total* loop lag, not its split
between motor and filter. Separating them needs an independent measurement of one
of the two — a thrust-stand step response would do it.

## Held-out logs0306, rollout RMSE [deg/s, mean over axes]

| horizon | closed-loop sim | persistence | zero |
|---|---|---|---|
| 50 ms | 5.67 | 5.98 | 7.03 |
| 100 ms | 5.93 | 6.06 | 7.00 |
| 200 ms | 5.73 | 7.01 | 6.91 |
| 500 ms | **4.86** | 8.44 | 6.97 |

The open-loop baseline degraded with horizon and was beaten by a decay model. This
one *improves* with horizon and beats both trivial baselines at 500 ms, from stick
inputs alone. `closed_loop_plots/rollouts.png` shows it following the measured
trajectory through the whole 600 ms window; it tracks the trend and misses the
vibration, which is the correct thing to miss.

Caveat on credit: part of the fit's success is the controller tracking the
setpoint, which would happen with an approximately right plant. The 500 ms number
is still meaningful — a wrong plant diverges — but it is not purely a test of the
dynamics.

## Getting physical inertia

`--jxx <value>` scales the gains once you measure roll inertia; the script then
prints the implied thrust constant and the other two inertias. A bifilar pendulum
test gives that number in half an hour. Without it, `k` stays in arbitrary units.

## Next

1. Measure Jxx, rerun with `--jxx`, and the parameters become physical.
2. Replace the gyroscopic term with proper inertia ratios constrained to a physical
   range, so it cannot absorb cross-coupling.
3. Then, and only then, a learned residual on top — the structured model now fits
   well enough that whatever it misses is worth modelling.
