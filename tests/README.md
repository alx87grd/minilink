# Test suite organization

**Entry points (human · agent · CI):** see [Entry points](#entry-points) below — authoritative for all three audiences.

Six-layer vision (detail): [docs/plans/test-benchmark-consolidation.md](../docs/plans/test-benchmark-consolidation.md).

---

## Entry points

**Prerequisites:** repo root, conda env **`minilink`**, `PYTHONPATH=.` (see [README.md#install](../README.md#install)).

### At a glance

| Who | When | Run |
| --- | --- | --- |
| **Human** | Daily dev | `pytest` |
| **Human** | Before push | `ruff check . && ruff format --check . && pytest` |
| **Human** | Big sim/trajopt/MPC change | `python benchmarks/run_regression_check.py --suite all` |
| **Human** | Backend/perf investigation (new machine, GPU) | `python benchmarks/run_study.py --list` |
| **Human** | Demo/catalog sanity | `python examples/scripts/_smoke/run_catalog_smokes.py --fast` |
| **Agent** | Pre-push / handoff | Same as human “Before push”; scope pytest per [AGENTS.md](../AGENTS.md) |
| **Agent** | Compile/sim/trajopt edits | Add L2 regression (command below) |
| **CI** | Every PR | [`.github/workflows/test.yml`](../.github/workflows/test.yml) — `test` + `regression` jobs |

### Human — copy/paste

```bash
conda activate minilink

# L1 — default (library contracts; includes L4 contract + L6 smoke bridge via pytest)
pytest

# Minimal deps only (skips JAX, plotly, pygame, …)
pytest -m "not optional"

# Headless optional renderers (pygame, full optional set)
SDL_VIDEODRIVER=dummy pytest

# L2 — regression (pre-handoff on compile / Simulator / trajopt / MPC)
PYTHONPATH=. python benchmarks/run_regression_check.py --suite all

# L2 — same flags as CI
PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny \
  --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup

# L6 — catalog + flagship demos (scripts; also run via test_smoke_runners.py in pytest)
python examples/scripts/_smoke/run_catalog_smokes.py --fast
python examples/scripts/_smoke/run_flagship_demos.py

# L4 — graphics headless PNG (optional; also covered by pytest smoke runner)
python examples/scripts/_smoke/run_flagship_graphics.py

# L5 — visual checklist (local only; you confirm pixels)
python examples/scripts/_smoke/run_graphics_visual_check.py

# L3 — backend/computer performance exploration (local; prints tables, no CI gate)
python benchmarks/run_study.py --list
python benchmarks/run_study.py --preset f_eval --plant pendulum      # native vs NumPy vs JAX f()
python benchmarks/run_study.py --preset sim --mode matrix            # solver × backend sweep
python benchmarks/run_study.py --preset trajopt --mode backends       # trajopt backend sweep
python benchmarks/run_study.py --preset dp                           # DP loop/numpy/jax
python benchmarks/run_study.py --preset rrt_nearest                  # RRT nearest backends
# Detail: benchmarks/README.md
```

### Performance / backends — L2 vs L3

| | **L2** `run_regression_check.py` | **L3** `run_study.py` |
| --- | --- | --- |
| **Purpose** | Pass/fail vs committed JSON baselines | Explore tables on *your* machine/GPU |
| **Speed** | Gated: `speedup`, `solve_s`, `nlp_s` (CI: factor 6×) | Report only — compare runs yourself |
| **Accuracy** | Gated: trajectory goldens, `rel_err_l2`, checkpoints | — |
| **CI** | ✅ `regression` job | ❌ never |
| **When** | After compile/sim/trajopt/MPC changes; before merge | New machine, backend tuning, perf investigation |

L2 suites: `core_perf` (compile/`f()` speed ratios + sim accuracy), `integration`, `solve_speed`, `e4`, `f_mpc`.
Full catalog: [benchmarks/README.md](../benchmarks/README.md).

`test_benchmark_smoke.py` (in `pytest`) only checks that benchmark **helpers import and return rows** — it does not measure or gate performance.

### Agent — copy/paste

Rules: [AGENTS.md](../AGENTS.md). **Do not invent commands** — use this table.

| Situation | Command |
| --- | --- |
| Always before push | `ruff check . && ruff format --check .` |
| Docs/markdown only | skip pytest |
| Narrow module change | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or handoff | `pytest` |
| Compile / `Simulator` / trajopt / MPC | L2: `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Backend perf exploration (no gate) | L3: `python benchmarks/run_study.py --list` → see [benchmarks/README.md](../benchmarks/README.md) |
| User-facing demo/smoke change | `python examples/scripts/_smoke/run_catalog_smokes.py --fast` and/or `run_flagship_demos.py` |

L1 discovery path: `tests/unittest/` only. Demos live under `examples/scripts/` — **L6 smokes, not duplicated in L1**.

### CI — job → command

Workflow: [`.github/workflows/test.yml`](../.github/workflows/test.yml).

| Job | Trigger | Exact steps |
| --- | --- | --- |
| **`test`** | Python 3.10–3.13 matrix | `pip install -e ".[dev]"` → `ruff check .` → `ruff format --check .` → `pytest` |
| **`regression`** | After `test` passes; Python 3.12 + JAX | `pip install -e ".[dev,jax]"` → `PYTHONPATH=$PWD python benchmarks/run_regression_check.py --suite all --tiny --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup` |

What pytest covers in CI: **L1** (all `tests/unittest/`), **L4** (`test_flagship_graphics_contract.py`), **L6 bridge** (`test_smoke_runners.py` subprocesses catalog/flagship/graphics scripts). **L2** is the separate `regression` job. **L3/L5** are not CI.

---

## Six layers (L1 → L6)

| Layer | Purpose | Entry command | CI job |
| --- | --- | --- | --- |
| **L1** Contracts | API types, shapes, compile/sim/MPC behavior | `pytest` | `test` |
| **L2** Regression | Accuracy goldens + guarded NLP/trajopt solve time | `run_regression_check.py --suite all` | `regression` |
| **L3** Benchmark utility | Machine/GPU exploration tables | `run_study.py --list` | — |
| **L4** Graphics auto | Draw-list + headless PNG smoke | `test_flagship_graphics_contract.py` (in `pytest`) | `test` |
| **L5** Graphics visual | You confirm Meshcat/MPL/Plotly locally | `run_graphics_visual_check.py` | — |
| **L6** Smoke runners | Catalog + flagship demos must not throw | `run_catalog_smokes.py`, `run_flagship_demos.py` | `test` (via `test_smoke_runners.py`) |

## Local environment

Setup: [README.md#install](../README.md#install). Non-interactive:

```bash
conda run -n minilink python -m pytest
```

## Philosophy

Tests guard **stable public contracts** (compile evaluators, planning transcriptions,
graphics frame keys, catalog equation references)—not implementation trivia or
third-party print formatting. Prefer one parametrized or table-driven test over
many near-duplicate files.

**Domain modules** (22 files after L1 consolidation): `test_core`, `test_backends`,
`test_compile`, `test_diagrams`, `test_simulation`, `test_step_discrete`,
`test_hybrid`, `test_dynamics_catalog`, `test_mechanical_robotics`, `test_blocks`,
`test_control_analysis`, `test_costs_optimizer`, `test_planning`, `test_mpc`,
`test_graphics`, `test_geometry`, `test_engine_jax`, `test_jax_planning`,
`test_symbolic`, `test_benchmark_smoke`, `test_smoke_runners`,
`test_flagship_graphics_contract`. Re-merge helper:
`scripts/merge_l1_tests.py`.

Kinematic render smoke (L4): ``run_flagship_graphics.py`` and manifest under
``tests/fixtures/kinematic_baseline/``.
Regenerate manifest: `python tests/fixtures/kinematic_baseline/regenerate_manifest.py`.

Shared fixtures: `graphics_contract_helpers.py` (draw-list resolution),
`planning_helpers.py` (RRT holonomic obstacle scene).

Benchmark **Layer B** regression lives under repo-root `benchmarks/`; helper API
drift guards in `test_benchmark_smoke.py`. See [Entry points](#entry-points) for
L2 CI vs local commands and [benchmarks/README.md](../benchmarks/README.md).

**Layer L6** scripts (also invoked from `test_smoke_runners.py` in pytest):

```bash
python examples/scripts/_smoke/run_catalog_smokes.py --fast
python examples/scripts/_smoke/run_flagship_demos.py
python examples/scripts/_smoke/run_all_demos.py --flagship-only --continue-on-error
```

Nightly/manual full demo sweep: `run_all_demos.py`. L4 PNG smoke:
`run_flagship_graphics.py`. L5 visual: `run_graphics_visual_check.py`.

`tests/manual/` and `tests/bugs/` are removed — use `examples/scripts/` for
smoke scripts and unittest for contracts.

## Core behavior without optional extras

```bash
pytest -m "not optional"
```

Optional tests skip at runtime when their dependency is not installed (see [Marker policy](#marker-policy)).

## Full functionality run

With the **`minilink`** conda env:

```bash
conda activate minilink
SDL_VIDEODRIVER=dummy pytest
```

Or install all pip extras in another Python 3.10+ environment:

```bash
pip install -e ".[dev,symbolic,jax,visualization,plotting,ipopt]"
SDL_VIDEODRIVER=dummy pytest
```

`SDL_VIDEODRIVER=dummy` lets pygame smoke tests initialize in headless CI or
Cursor Cloud sessions.

## Marker policy

- `optional`: any test that needs a dependency outside the base package
- `jax`: tests requiring `jax` / `jaxlib`
- `symbolic`: tests requiring `sympy`
- `visualization`: tests requiring `meshcat` or `pygame`
- `plotting`: tests requiring `plotly`
- `ipopt`: tests requiring `cyipopt`

When adding optional behavior, put the import inside a guarded block and add the
appropriate marker(s). This keeps `pytest -m "not optional"` a dependable
behavior suite for minimal installations.
