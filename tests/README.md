# Test suite organization

**Entry points (human · agent · CI):** see [Entry points](#entry-points) below — authoritative for all three audiences.

Six-layer vision (detail): [docs/plans/test-benchmark-consolidation.md](../docs/plans/test-benchmark-consolidation.md).

---

## Entry points

**Prerequisites:** repo root, conda env **`minilink`**, `PYTHONPATH=.` (see [README.md#install](../README.md#install)).

### Folder layout (everything under `tests/` or `benchmarks/`)

```
tests/
  README.md              ← this file (entry points)
  run/                   ← human IDE launchers (click Run)
  smoke/                 ← L6 catalog + demo smoke scripts
  unittest/              ← L1 pytest contracts
  fixtures/              ← graphics/regression fixtures
  merge_l1_tests.py      ← dev tool to re-merge domain modules

benchmarks/
  run_regression_check.py   ← L2 gates (+ host speed context)
  run_study.py              ← L3 backend exploration
  host_profiles/*.json      ← recorded timings per machine (context only)
  baselines/*.json          ← committed regression goldens
```

Demos stay in `examples/scripts/` for teaching; **smoke runners** that execute them live in `tests/smoke/`.

### Human (IDE — open script, click **Run**)

| When | Open and Run |
| --- | --- |
| **Daily** | [`tests/run/run_l1_tests.py`](run/run_l1_tests.py) |
| **Before push** | [`tests/run/run_pre_push.py`](run/run_pre_push.py) |
| **After sim/trajopt/MPC work** | [`tests/run/run_l2_regression.py`](run/run_l2_regression.py) |
| **Demo/catalog sanity** | [`tests/run/run_l6_smokes.py`](run/run_l6_smokes.py) |
| **Backend perf tables** | [`tests/run/run_l3_benchmark_study.py`](run/run_l3_benchmark_study.py) |
| **Pick one check** | [`tests/run/run_checks.py`](run/run_checks.py) — set `CHECK = "l1"` at top |

L5 visual checklist: [`tests/smoke/run_graphics_visual_check.py`](smoke/run_graphics_visual_check.py).

Optional toggles: constants at the top of each `tests/run/*.py` file.

### Agent (terminal / CI — exact commands)

| Situation | Command |
| --- | --- |
| Always before push | `ruff check . && ruff format --check .` |
| Docs/markdown only | skip pytest |
| Narrow module change | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or handoff | `pytest` |
| Compile / `Simulator` / trajopt / MPC | `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Backend perf exploration | `python benchmarks/run_study.py --list` |
| Demo smoke change | `python tests/smoke/run_catalog_smokes.py --fast` and/or `run_flagship_demos.py` |

Rules: [AGENTS.md](../AGENTS.md).

### CI (GitHub Actions)

Workflow: [`.github/workflows/test.yml`](../.github/workflows/test.yml).

| Job | Steps |
| --- | --- |
| **`test`** | ruff + `pytest` (py 3.10–3.13) |
| **`regression`** | `run_regression_check.py --suite all --tiny …` (py 3.12 + JAX) |

### At a glance

| Layer | Human IDE (`tests/run/`) | Agent / CI | CI job |
| --- | --- | --- | --- |
| **L1** | `run_l1_tests.py` | `pytest` | `test` |
| **L2** | `run_l2_regression.py` | `benchmarks/run_regression_check.py` | `regression` |
| **L3** | `run_l3_benchmark_study.py` | `benchmarks/run_study.py` | — |
| **L4** | (in L1 pytest) | `test_flagship_graphics_contract.py` | `test` |
| **L5** | `tests/smoke/run_graphics_visual_check.py` | local | — |
| **L6** | `run_l6_smokes.py` | `tests/smoke/run_*.py` | `test` |
| **Pre-push** | `run_pre_push.py` | ruff + pytest | `test` |

### Performance — L2 gates vs L3 exploration vs host context

| | **L2** regression | **L3** `run_study.py` | **Host profiles** |
| --- | --- | --- | --- |
| **Role** | Pass/fail vs `benchmarks/baselines/` | Backend tables on your machine | **Context only** — timings from other hosts |
| **Location** | `benchmarks/` | `benchmarks/` | `benchmarks/host_profiles/*.json` |
| **CI** | ✅ | ❌ | printed on L2 runs (informational) |

L2 prints a **host speed context** table after each suite (compare current run to recorded profiles). Gates unchanged. Record your machine:

```bash
python benchmarks/run_regression_check.py --suite f_mpc --tiny \
  --record-host-profile my_laptop --host-profile-label "My workstation"
```

Detail: [benchmarks/README.md](../benchmarks/README.md).

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
[`merge_l1_tests.py`](merge_l1_tests.py).

Kinematic render smoke (L4): ``run_flagship_graphics.py`` and manifest under
``tests/fixtures/kinematic_baseline/``.
Regenerate manifest: `python tests/fixtures/kinematic_baseline/regenerate_manifest.py`.

Shared fixtures: `graphics_contract_helpers.py` (draw-list resolution),
`planning_helpers.py` (RRT holonomic obstacle scene).

Benchmark **Layer B** regression lives under repo-root `benchmarks/`; helper API
drift guards in `test_benchmark_smoke.py`. See [Entry points](#entry-points) for
L2 CI vs local commands and [benchmarks/README.md](../benchmarks/README.md).

**Layer L6** scripts in [`tests/smoke/`](smoke/) (also invoked from `test_smoke_runners.py`):

```bash
python tests/smoke/run_catalog_smokes.py --fast
python tests/smoke/run_flagship_demos.py
python tests/smoke/run_all_demos.py --flagship-only --continue-on-error
```

IDE launchers: [`tests/run/`](run/). L4 PNG smoke: `tests/smoke/run_flagship_graphics.py`.

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
