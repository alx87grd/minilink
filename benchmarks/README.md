# Benchmarks

Performance tracking and end-to-end numerical regression for minilink — not
collected by default pytest (pytest only smoke-tests benchmark helpers) and not
shipped in the pip package. Helpers are grouped by what they measure;
`common.py` holds shared plumbing, `systems/` holds synthetic fixture systems,
and `scenarios/` holds canonical integration-check setups.

The modules import minilink exactly like an external user. Nothing inside
`minilink/` may import from here (the unittest smoke tests in
`tests/unittest/test_benchmark_smoke.py` may).

Benchmark fixtures intentionally differ from unittest planning fixtures: e.g.
`planning_rrt.holonomic_problem()` uses a dense 18-sphere scene for timing
studies, while `tests/unittest/planning_helpers.py` keeps a minimal obstacle
scene for fast RRT contract tests.

## Layout

| Path | Role |
| --- | --- |
| [`run_regression_check.py`](run_regression_check.py) | Single entry point for committed baselines |
| [`suites/core_perf.py`](suites/core_perf.py) | Fast compile/`f()`/sim final-state tier-1 gate |
| [`suites/integration_check.py`](suites/integration_check.py) | Trajectory checkpoint goldens + solve-time gates |
| [`scenarios/`](scenarios/) | Frozen scenario configs (double pendulum, showcase pendulum, cart-pole trajopt) |
| [`run_e4_trajopt_parity.py`](run_e4_trajopt_parity.py) | E4 trajopt / MPC parametric regression parity |
| [`run_f_mpc_parity.py`](run_f_mpc_parity.py) | F MPC `control/mpc` regression parity — hybrid / hand-loop / dual-rate |
| [`baselines/*.json`](baselines/) | Committed golden metrics |

## Running

From the repo root with the **`minilink`** conda env active (see
[environment.yml](../environment.yml)), or any environment with the extras you
want to measure (`minilink[jax]` for JAX variants; `cyipopt` for Ipopt
variants — both are skipped gracefully when missing):

```bash
python benchmarks/run_pendulum_f_speed.py        # f() call speed, single plant
python benchmarks/run_diagram_f_speed.py         # f() call speed, dense diagram
python benchmarks/run_step_speed.py            # step() call speed, leaf StepSystem
python benchmarks/run_step_diagram_speed.py    # step() call speed, step diagram
python benchmarks/run_simulator_standard.py      # one variant on standard cases
python benchmarks/run_simulator_speed_matrix.py  # solver x backend sweep (euler vs euler_fixedsteps)
python benchmarks/run_simulator_speed_manual.py  # hand-picked simulator runs
python benchmarks/run_optimizer_backends.py      # NLP solver presets on textbook problems
python benchmarks/run_trajopt_backends.py        # trajopt transcription x backend sweep
python benchmarks/run_trajopt_solver_presets.py  # direct-collocation solver presets
python benchmarks/run_dp_backends.py             # value-iteration loop/numpy/jax backends
python benchmarks/run_rrt_nearest_backends.py    # RRT nearest brute_force vs kd_tree
python benchmarks/run_regression_check.py                  # tier-1 core_perf baseline
python benchmarks/run_regression_check.py --suite integration  # trajectory + trajopt gate
python benchmarks/run_regression_check.py --suite all        # both suites
python benchmarks/run_regression_check.py --update           # refresh committed JSON
python benchmarks/run_e4_trajopt_parity.py --capture         # E4 parity: capture baseline JSON
python benchmarks/run_e4_trajopt_parity.py                   # E4 parity: compare to baseline
python benchmarks/run_f_mpc_parity.py --capture              # F MPC parity: capture baseline JSON
python benchmarks/run_f_mpc_parity.py                        # F MPC parity: compare to baseline
```

## Regression baselines

Two suites share [`run_regression_check.py`](run_regression_check.py) — **not** run by
default pytest (pytest only smoke-tests loader/compare logic):

| Suite | Baseline | What it gates |
| --- | --- | --- |
| `core_perf` (default) | [`baselines/core_perf.json`](baselines/core_perf.json) | Compile/`f()` speed, diagram `dx` accuracy, sim final-state goldens |
| `integration` | [`baselines/integration_check.json`](baselines/integration_check.json) | Trajectory checkpoints, showcase cart-pole trajopt (SciPy SLSQP + JAX compile), solve times |
| E4 trajopt parity | [`baselines/e4_trajopt_parity.json`](baselines/e4_trajopt_parity.json) | TOP rebuild + MPC parametric from-solve: cartpole, pendulum, bicycle (traj + timing) |
| F MPC parity | [`baselines/f_mpc_parity.json`](baselines/f_mpc_parity.json) | `control/mpc` product path: hybrid ZOH, hand-loop, dual-rate (traj + timing, factor 2×) |

**Speed** metrics use a loose multiplicative factor (default **4×**, override with
`MINILINK_BENCH_REGRESSION_FACTOR`). **Accuracy** metrics use absolute ceilings:

- `rel_err_l2` — candidate final state vs live `scipy_ultra` truth (max **1%**)
- `truth.x_tf` — live truth final state vs **committed golden vector** (catches broken
  truth/`Simulator` pipeline even when candidate-vs-truth still looks fine)
- `checkpoint_t*` — state samples along the trajectory vs committed goldens
- `diagram_dense_f.numpy.dx_residual` — native `diagram.f` vs compiled evaluator

```bash
python benchmarks/run_regression_check.py
python benchmarks/run_regression_check.py --suite integration
python benchmarks/run_regression_check.py --suite all
python benchmarks/run_regression_check.py --suite integration --update   # after intentional changes
```

Review the JSON diff before committing an `--update`. Tier-2 runners (`run_*_speed.py`,
trajopt/optimizer sweeps) remain deep manual studies and are not gated in v1.
