# Benchmarks

Performance tracking and end-to-end numerical regression for minilink — not
collected by default pytest (pytest only guards benchmark helper imports) and not
shipped in the pip package. Helpers are grouped by what they measure;
`common.py` holds shared plumbing, `systems/` holds synthetic fixture systems,
and `scenarios/` holds canonical integration-check setups.

The modules import minilink exactly like an external user. Nothing inside
`minilink/` may import from here (the unittest helper guards in
`tests/unittest/test_benchmark_helpers.py` may).

Benchmark fixtures intentionally differ from unittest planning fixtures: e.g.
`planning_rrt.holonomic_problem()` uses a dense 18-sphere scene for timing
studies, while `tests/unittest/planning_helpers.py` keeps a minimal obstacle
scene for fast RRT contract tests.

## Layout

| Path | Role |
| --- | --- |
| [`run_regression_check.py`](run_regression_check.py) | Single entry point for all committed baselines |
| [`run_study.py`](run_study.py) | **Layer C** unified machine-exploration presets (replaces tier-2 `run_*_speed.py` / backend sweeps) |
| [`studies/presets.py`](studies/presets.py) | Preset implementations invoked by `run_study.py` |
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
python benchmarks/run_study.py --list
python benchmarks/run_study.py --preset f_eval --plant pendulum
python benchmarks/run_study.py --preset sim --mode matrix
python benchmarks/run_study.py --preset trajopt --mode backends
python benchmarks/run_study.py --preset optimizer
python benchmarks/run_study.py --preset dp
python benchmarks/run_study.py --preset rrt_nearest
python benchmarks/run_pendulum_f_speed.py        # deprecated shim → f_eval pendulum
python benchmarks/run_diagram_f_speed.py         # deprecated shim → f_eval diagram_dense
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
python benchmarks/run_regression_check.py --suite all --tiny   # CI reduced workload
python benchmarks/run_regression_check.py --update           # refresh committed JSON
python benchmarks/run_e4_trajopt_parity.py --capture         # same as --suite e4 --update
python benchmarks/run_f_mpc_parity.py --capture              # same as --suite f_mpc --update
```

## Regression baselines (Layer B)

All gated suites run through [`run_regression_check.py`](run_regression_check.py).
Pytest only guards benchmark helpers (`test_benchmark_helpers.py`); **GitHub CI** runs
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

### Host speed context (multi-machine reference)

Speed gates still use **one committed baseline** per metric. Additionally,
`benchmarks/host_profiles/*.json` stores timings from known machines (fast
workstation, Linux cloud VM, …). After each suite, regression prints an
**informational** table comparing the current run to those profiles — useful
when a cloud agent sees an `nlp_s` failure that is normal on a slow host.

Record your machine (does not change gates):

```bash
python benchmarks/run_regression_check.py --suite f_mpc --tiny \
  --record-host-profile my_machine --host-profile-label "Description"
```

Skip context: `--no-host-context`.

```bash
python benchmarks/run_regression_check.py
python benchmarks/run_regression_check.py --suite integration
python benchmarks/run_regression_check.py --suite all
python benchmarks/run_regression_check.py --suite integration --update   # after intentional changes
```

Review the JSON diff before committing an `--update`. Tier-2 runners are **deprecated
shims** forwarding to [`run_study.py`](run_study.py); Pyro parity remains manual Layer-C
only (external Pyro env).
