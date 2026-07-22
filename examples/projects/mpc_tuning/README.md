# MPC lap-time tuning (path + corridor, from rest)

Per-model search for fastest one lap on the rounded-rectangle circuit
(`CIRCUIT_LX=30`, `LY=18`, `R=10`, half-width=3). Design notes:
[docs/plans/mpc-tuning.md](../../../docs/plans/mpc-tuning.md).

## Layout

| Path | Role |
| --- | --- |
| `track.py` | Circuit geometry + `ReferenceTrack` |
| `models.py` | Four plant builders (config-driven) |
| `harness.py` | `run_lap(model, config) → LapResult` |
| `search.py` | Phased CLI search |
| `configs/seed_*.json` | Starting hyperparameters |
| `configs/best_*.json` | Frozen winners (Phase 4) |
| `results/*.jsonl` | Trial logs (gitignored) |

Models: `kinematic`, `kinematic_rate`, `dynamic_rate`, `dynamic_engine`.

## Run

From repo root with conda env `minilink`:

```bash
export PYTHONPATH=.
export MPLBACKEND=Agg

# Phase 0 — Engine must complete one valid lap (blocking)
python examples/projects/mpc_tuning/search.py --model dynamic_engine --phase 0

# Phase 1 — coarse random grid (all models)
python examples/projects/mpc_tuning/search.py --model all --phase 1 --max-trials 40

# Phase 2 — realtime (median solve ≤ 0.2 s @ MPC_DT=0.2)
python examples/projects/mpc_tuning/search.py --model all --phase 2 --max-trials 20

# Phase 3 — minimize lap time under validity + RT gates
python examples/projects/mpc_tuning/search.py --model all --phase 3 --max-trials 40

# Phase 4 — write best_*.json + summary table
python examples/projects/mpc_tuning/search.py --model all --phase 4
```

Single model: `--model kinematic` (etc.).

## Gates

- Lap finished within `TF_SIM_MAX=90` s (arc progress ≥ path length)
- Mean path error &lt; corridor half-width; max &lt; 2× half-width
- From Phase 2: median replan `solve_s` ≤ 0.2 s (first replan excluded)

Matched plant params during search (no intentional mass mismatch).
