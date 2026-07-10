# Benchmarks

Performance tracking for minilink — not correctness tests (no asserts, not
collected by pytest) and not shipped in the pip package. Helpers are grouped
by what they measure; `common.py` holds shared plumbing and `systems/` holds
synthetic fixture systems.

The modules import minilink exactly like an external user. Nothing inside
`minilink/` may import from here (the unittest smoke tests in
`tests/unittest/test_benchmark_smoke.py` may).

Benchmark fixtures intentionally differ from unittest planning fixtures: e.g.
`planning_rrt.holonomic_problem()` uses a dense 18-sphere scene for timing
studies, while `tests/unittest/planning_helpers.py` keeps a minimal obstacle
scene for fast RRT contract tests.

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
python benchmarks/run_simulator_speed_matrix.py  # solver x backend sweep
python benchmarks/run_simulator_speed_manual.py  # hand-picked simulator runs
python benchmarks/run_optimizer_backends.py      # NLP solver presets on textbook problems
python benchmarks/run_trajopt_backends.py        # trajopt transcription x backend sweep
python benchmarks/run_trajopt_solver_presets.py  # direct-collocation solver presets
python benchmarks/run_dp_backends.py             # value-iteration loop/numpy/jax backends
python benchmarks/run_rrt_nearest_backends.py    # RRT nearest brute_force vs kd_tree
python benchmarks/run_regression_check.py        # tier-1 speed + accuracy vs committed baseline
python benchmarks/run_regression_check.py --update  # refresh benchmarks/baselines/core_perf.json
```

## Regression baselines

Tier-1 regression lives in [`baselines/core_perf.json`](baselines/core_perf.json) and is
checked manually via [`run_regression_check.py`](run_regression_check.py) — **not** by
default pytest (pytest only smoke-tests loader/compare logic).

**Speed** metrics use a loose multiplicative factor (default **4×**, override with
`MINILINK_BENCH_REGRESSION_FACTOR`). **Accuracy** metrics use absolute ceilings:

- `rel_err_l2` — candidate final state vs live `scipy_ultra` truth (max **1%**)
- `truth.x_tf` — live truth final state vs **committed golden vector** (catches broken
  truth/`Simulator` pipeline even when candidate-vs-truth still looks fine)
- `diagram_dense_f.numpy.dx_residual` — native `diagram.f` vs compiled evaluator

```bash
python benchmarks/run_regression_check.py
python benchmarks/run_regression_check.py --update   # after intentional perf/accuracy changes
```

Review the JSON diff before committing an `--update`. Tier-2 runners (`run_*_speed.py`,
trajopt/optimizer sweeps) remain deep manual studies and are not gated in v1.
