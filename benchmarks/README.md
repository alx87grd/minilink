# Benchmarks

Performance tracking for minilink — not correctness tests (no asserts, not
collected by pytest) and not shipped in the pip package. Helpers are grouped
by what they measure; `common.py` holds shared plumbing and `systems/` holds
synthetic fixture systems.

The modules import minilink exactly like an external user. Nothing inside
`minilink/` may import from here (the unittest smoke tests in
`tests/unittest/` may).

## Running

From the repo root, in an environment with the extras you want to measure
(`minilink[jax]` for JAX variants; `cyipopt` for Ipopt variants — both are
skipped gracefully when missing):

```bash
python benchmarks/run_pendulum_f_speed.py        # f() call speed, single plant
python benchmarks/run_diagram_f_speed.py         # f() call speed, dense diagram
python benchmarks/run_simulator_standard.py      # one variant on standard cases
python benchmarks/run_simulator_speed_matrix.py  # solver x backend sweep
python benchmarks/run_simulator_speed_manual.py  # hand-picked simulator runs
python benchmarks/run_optimizer_backends.py      # NLP solver presets on textbook problems
python benchmarks/run_trajopt_backends.py        # trajopt transcription x backend sweep
python benchmarks/run_trajopt_solver_presets.py  # direct-collocation solver presets
python benchmarks/run_dp_backends.py             # value-iteration loop/numpy/jax backends
python benchmarks/run_rrt_nearest_backends.py    # RRT nearest brute_force vs kd_tree
```
