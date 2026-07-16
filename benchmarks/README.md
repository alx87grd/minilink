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
| [`run_regression_check.py`](run_regression_check.py) | Single entry point for all committed baselines |
| [`suites/core_perf.py`](suites/core_perf.py) | Fast compile/`f()`/sim final-state tier-1 gate |
| [`suites/integration_check.py`](suites/integration_check.py) | Trajectory checkpoint goldens + JAX trajopt solve-time gates |
| [`suites/solve_speed.py`](suites/solve_speed.py) | Standalone NLP + NumPy pendulum trajopt solve wall-time gates |
| [`scenarios/`](scenarios/) | Frozen scenario configs (double pendulum, showcase pendulum, cart-pole trajopt, E4/F MPC parity) |
| [`run_e4_trajopt_parity.py`](run_e4_trajopt_parity.py) | Thin wrapper → `--suite e4` (capture/compare) |
| [`run_f_mpc_parity.py`](run_f_mpc_parity.py) | Thin wrapper → `--suite f_mpc` (capture/compare) |
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
python benchmarks/run_regression_check.py --suite integration  # trajectory + JAX trajopt gate
python benchmarks/run_regression_check.py --suite solve_speed  # NLP + NumPy trajopt solve gates
python benchmarks/run_regression_check.py --suite e4           # E4 trajopt/MPC parametric parity
python benchmarks/run_regression_check.py --suite f_mpc        # F MPC hybrid/hand-loop/dual-rate
python benchmarks/run_regression_check.py --suite all          # all Layer-B suites
python benchmarks/run_regression_check.py --suite all --tiny   # CI smoke workload
python benchmarks/run_regression_check.py --update           # refresh committed JSON
python benchmarks/run_e4_trajopt_parity.py --capture         # same as --suite e4 --update
python benchmarks/run_f_mpc_parity.py --capture              # same as --suite f_mpc --update
```

## Regression baselines (Layer B)

All gated suites run through [`run_regression_check.py`](run_regression_check.py).
Pytest only smoke-tests helpers (`test_benchmark_smoke.py`); **GitHub CI** runs
`--suite all --tiny` with JAX installed (see below).

| Suite | Baseline | What it gates |
| --- | --- | --- |
| `core_perf` (default) | [`baselines/core_perf.json`](baselines/core_perf.json) | Compile/`f()` speed ratios, diagram `dx` accuracy, sim final-state goldens |
| `integration` | [`baselines/integration_check.json`](baselines/integration_check.json) | Trajectory checkpoints, showcase cart-pole JAX trajopt accuracy + **`solve_s`** |
| `solve_speed` | [`baselines/solve_speed.json`](baselines/solve_speed.json) | **`Optimizer.solve_s`** on textbook NLPs + **NumPy pendulum trajopt `solve_s`** |
| `e4` | [`baselines/e4_trajopt_parity.json`](baselines/e4_trajopt_parity.json) | TOP rebuild + MPC parametric JAX trajopt trajectories + **`solve_s` / `total_s`** |
| `f_mpc` | [`baselines/f_mpc_parity.json`](baselines/f_mpc_parity.json) | `control/mpc` hybrid/hand-loop/dual-rate trajectories + **`nlp_s`** (factor **2×** locally) |

**Speed** metrics use a multiplicative factor (default **4×** per baseline JSON, override
with `--factor` or `MINILINK_BENCH_REGRESSION_FACTOR`). A **4×** ceiling catches a **10×**
slowdown on the next manual baseline refresh; CI uses **6×** on NLP solve gates only
(see below). **Accuracy** metrics use absolute ceilings:

- `rel_err_l2` — candidate final state vs live `scipy_ultra` truth (max **1%**)
- `truth.x_tf` — live truth final state vs **committed golden vector**
- `checkpoint_t*` — state samples along the trajectory vs committed goldens
- `diagram_dense_f.numpy.dx_residual` — native `diagram.f` vs compiled evaluator

### CI vs local

**CI** (`.github/workflows/test.yml`, `regression` job):

```bash
python benchmarks/run_regression_check.py --suite all --tiny \
  --factor 6 \
  --speed-gate-suffixes solve_s,nlp_s,speedup
```

Enforces accuracy goldens plus **NLP/trajopt solve wall times** (`solve_s`, `nlp_s`,
speedup ratios). End-to-end `wall_s` / `total_s` are reported but not gated in CI
(cross-runner variance).

**Local pre-handoff** (reference machine, per-suite factors from JSON):

```bash
python benchmarks/run_regression_check.py --suite all
```

```bash
python benchmarks/run_regression_check.py
python benchmarks/run_regression_check.py --suite integration
python benchmarks/run_regression_check.py --suite all
python benchmarks/run_regression_check.py --suite integration --update   # after intentional changes
```

Review the JSON diff before committing an `--update`. Tier-2 runners (`run_*_speed.py`,
trajopt/optimizer sweeps, Pyro parity) remain manual Layer-C studies.
