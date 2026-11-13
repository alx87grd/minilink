# Benchmarks

The regression gate of minilink: end-to-end accuracy goldens and guarded solve times,
checked against committed baselines. It is not collected by pytest (pytest guards its
helpers in `tests/unittest/test_benchmark_helpers.py`) and not shipped in the pip package.

The modules import minilink exactly like an external user. Nothing inside `minilink/` may
import from here (the unit tests may: the helper guards in `test_benchmark_helpers.py`, the
fixture systems in `test_mechanical_robotics.py` and `flatten_diagram_helpers.py`, and the
public-name inventory in `test_public_names.py`).

## Layout

| Path | Role |
| --- | --- |
| [`run_regression_check.py`](run_regression_check.py) | The one entry point: runs the suites, compares them with their baselines, prints the host context |
| [`suites/`](suites/) | The gated suites: `core_perf`, `integration_check`, `solve_speed`, `dp_speed` |
| [`scenarios/`](scenarios/) | Frozen scenario configs: double pendulum, showcase pendulum, cart-pole trajopt, the trajectory-optimization and MPC parity suites (`e4`, `f_mpc`) |
| `simulation.py`, `f_evaluators.py`, `optimization.py`, `trajopt.py`, `dynamic_programming.py` | The measuring helpers the suites call, by what they measure |
| `baseline.py`, `common.py`, `host_profiles.py` | The baseline files and their comparison, shared plumbing, the host speed context |
| [`systems/`](systems/) | Synthetic fixture systems (pendulum twins, many spheres, dense networks) |
| [`baselines/*.json`](baselines/) | The committed goldens, one file per suite |
| [`host_profiles/*.json`](host_profiles/) | Recorded timings per machine (context only, not a gate) |
| [`public_names.py`](public_names.py) | Not part of the gate: the public-name inventory, every facade name and the files using it; `--untested` lists the names no test uses (held empty by `tests/unittest/test_public_names.py`) |

## Running

From the repo root with the **`minilink`** conda env active (see
[environment.yml](../environment.yml)), or any environment with the libraries you want to
measure (JAX for the JAX variants; `cyipopt` for the Ipopt variants — both are skipped
gracefully when missing). The CI and full-local commands are in the Agent table of
[tests/README.md](../tests/README.md#entry-points); this is the CLI.

```bash
python benchmarks/run_regression_check.py                      # core_perf baseline (default)
python benchmarks/run_regression_check.py --suite integration  # trajectory + JAX trajopt gate
python benchmarks/run_regression_check.py --suite solve_speed  # NLP + NumPy trajopt solve gates
python benchmarks/run_regression_check.py --suite dp_speed     # value-iteration sweep speed + backend agreement
python benchmarks/run_regression_check.py --suite e4           # E4 trajopt/MPC parametric parity
python benchmarks/run_regression_check.py --suite f_mpc        # F MPC hybrid/hand-loop/dual-rate
python benchmarks/run_regression_check.py --suite all          # all regression-gate suites
python benchmarks/run_regression_check.py --suite all --tiny   # CI reduced workload

python benchmarks/run_regression_check.py --update             # refresh committed JSON
```

## Regression baselines

| Suite | Baseline | What it gates |
| --- | --- | --- |
| `core_perf` (default) | [`baselines/core_perf.json`](baselines/core_perf.json) | Compile/`f()` speed ratios, diagram `dx` accuracy, sim final-state goldens, the batch-rollout claim (`rollout_batch` on the pendulum, plain and a parameter family, and their ratio **`family_speedup`**) |
| `integration` | [`baselines/integration_check.json`](baselines/integration_check.json) | Trajectory checkpoints, showcase cart-pole JAX trajopt accuracy + **`solve_s`** |
| `solve_speed` | [`baselines/solve_speed.json`](baselines/solve_speed.json) | **`Optimizer.solve_s`** on textbook NLPs + **NumPy pendulum trajopt `solve_s`** |
| `dp_speed` | [`baselines/dp_speed.json`](baselines/dp_speed.json) | Value-iteration **`solve_s`** (numpy table + jax device loop), jax **`speedup`**, and jax-vs-numpy `J` / policy agreement |
| `e4` | [`baselines/e4_trajopt_parity.json`](baselines/e4_trajopt_parity.json) | TOP rebuild + MPC parametric JAX trajopt trajectories + **`solve_s` / `total_s`** |
| `f_mpc` | [`baselines/f_mpc_parity.json`](baselines/f_mpc_parity.json) | MPC trajectories (the sampled loop `(mpc % dt) @ plant`, a host loop on the planner's face, dual rate) + **`nlp_s`** (factor **2×** locally) |

**Speed** metrics use a multiplicative factor (default **4×** per baseline JSON, override
with `--factor` or `MINILINK_BENCH_REGRESSION_FACTOR`). A **4×** ceiling catches a **10×**
slowdown on the next manual baseline refresh; CI uses **10×** on NLP solve gates only
(see below). **Accuracy** metrics use absolute ceilings:

- `rel_err_l2` — candidate final state vs live `scipy_ultra` truth (max **1%**)
- `truth.x_tf` — live truth final state vs **committed golden vector**
- `checkpoint_t*` — state samples along the trajectory vs committed goldens
- `diagram_dense_f.numpy.dx_residual` — native `diagram.f` vs compiled evaluator

### CI vs local

The two commands — the CI flags of the `regression` job in
`.github/workflows/test.yml`, and the full local pre-handoff run — are written
once, in the Agent table of [tests/README.md](../tests/README.md#entry-points).
What the flags mean:

- `--suite all --tiny` — every gated suite on the reduced CI workload.
- `--speed-gate-suffixes solve_s,nlp_s,speedup` — CI gates the **NLP/trajopt
  solve wall times** (`solve_s`, `nlp_s`, speedup ratios) besides the accuracy
  goldens. End-to-end `wall_s` / `total_s` are reported but not gated in CI
  (cross-runner variance).
- `--factor 10 --speed-slack-s 0.1` — a time gate fails above
  `baseline * 10 + 0.1 s`: the e4 / f_mpc solves take a few milliseconds, and on
  a shared runner they swing 4–19× the macOS baseline run to run with no code
  change, so a factor alone flips at random.

The same job runs the flagship demos, so JAX-required smokes are not skipped
(the ``test`` job installs no JAX). Locally, `--suite all` without flags uses
the per-suite factors from the baseline JSON.

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

Run `--update` only after an intentional change, and review the JSON diff before
committing it.
