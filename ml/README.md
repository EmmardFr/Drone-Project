# Machine learning

| Folder | Description |
|--------|-------------|
| `rl/cartpole/` | Learning RL on Gymnasium `CartPole-v1`: A2C, n-step-return A2C, PPO (with and without fixed seeds) |
| `pinn/` | Physics-informed model of the drone's rate dynamics, fitted on the flight logs in `data/sample_flight_logs/`. Results and analysis in [`pinn/RESULTS.md`](pinn/RESULTS.md) |

```bash
python -m pip install -r requirements.txt
python ml/rl/cartpole/cartPole_PPO.py
python ml/pinn/fit_baseline.py --epochs 150 --eval-stride 100 --per-flight
```

Training plots are written next to the scripts and are git-ignored
(except the PINN figures referenced in `RESULTS.md`).
