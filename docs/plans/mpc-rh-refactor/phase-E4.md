# Phase E4 — Merge parametric MPC into TOP

**Status:** done  
**Depends on:** [E3](phase-E3.md) · contracts: [vision.md](vision.md)

## Goal

One `TrajectoryOptimizationPlanner`: single `__init__`; `solve_trajectory` and
`solve_trajectory_from` always work. Optional `compile_parametric_program()`
unlocks fast bind-only from-solves. Delete `MPCPlanner`. Sync DESIGN / ROADMAP.

## Contract (vision)

```python
planner = TrajectoryOptimizationPlanner(problem, transcription=dc, options=opts)
plan = planner.solve_trajectory()              # always rebuild
plan = planner.solve_trajectory_from(x_meas)   # rebuild if not compiled

planner.compile_parametric_program()           # optional acceleration
plan = planner.solve_trajectory_from(x_meas)   # bind + solve only
```

Controllers duck-type a TOP and auto-call `compile_parametric_program()` so
MPC ticks never re-transcribe.

## Files

| Path | Change |
| --- | --- |
| `minilink/planning/trajectory_optimization/planner.py` | Dual-speed from-API + compile |
| `minilink/planning/mpc/planner.py` / `options.py` | Delete after retarget |
| `minilink/planning/mpc/_block_common.py` + controllers | Duck-type TOP; auto-compile |
| `minilink/planning/mpc/transcription.py` + parametric_* | Fold / relocate under trajopt |
| Demos / tests | `TrajectoryOptimizationPlanner` |
| `benchmarks/` | Before/after trajopt parity suite |
| DESIGN / ROADMAP | Maturity + batch wording → rebuild / compile |

## Steps

| Step | Work |
| --- | --- |
| E4.0 | Expand this card + vision |
| E4.1 | Capture `e4_trajopt_parity.json` **before** code |
| E4.2 | TOP: `compile_parametric_program` + dual-speed `solve_trajectory_from` |
| E4.3 | Delete `MPCPlanner`; retarget demos/tests/controllers |
| E4.4 | Fold `transcribe_parametric`; relocate `parametric_*` |
| E4.5 | DESIGN / ROADMAP; after-compare; mark done |

## Constraints

- No PoC controller retirement (E5); no scene `params` (E7); no broadcast (E8).
- No silent `--update` of parity baseline after a bad merge.
- Rebuild TOP NumPy default stays for offline trajopt.

## Gate

```bash
python benchmarks/run_e4_trajopt_parity.py --capture   # before only
python benchmarks/run_e4_trajopt_parity.py

pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_jax_direct_collocation.py \
       tests/unittest/test_planning_architecture.py \
       tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_hybrid_straight_line.py \
       tests/unittest/test_mpc_hybrid_warm_start_parity.py \
       tests/unittest/test_mpc_solve_trajectory_from.py

MPLBACKEND=Agg python examples/scripts/mpc/demo_mpc_minimal.py
MPLBACKEND=Agg python examples/scripts/mpc/demo_mpc_straight.py
python benchmarks/run_regression_check.py --suite integration
ruff check . && ruff format --check .
```

## Exit

- No separate MPC NLP planner class
- Controllers / demos use TOP (+ compile for MPC)
- Parity suite green vs pre-merge baseline

Next: [phase-E5.md](phase-E5.md).
