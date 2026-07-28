# Test suite organization

**Entry points (human · agent · CI):** see [Entry points](#entry-points) below — authoritative for all three audiences.

Six-layer vision (historical plan detail): [docs/plans/test-benchmark-consolidation.md](../docs/plans/test-benchmark-consolidation.md). Notebook smoke is an additional layer on top of that vision.

---

## Entry points

**Prerequisites:** repo root, conda env **`minilink`**, `PYTHONPATH=.` (see [README.md#install](../README.md#install)).

### Folder layout (everything under `tests/` or `benchmarks/`)

```
tests/
  README.md              ← this file (entry points)
  run/                   ← human IDE launchers (click Run)
  demo_checks/           ← catalog + demo check scripts
  unittest/              ← contract tests (pytest)
  fixtures/              ← graphics/regression fixtures
  merge_contract_tests.py ← dev tool to re-merge domain modules

benchmarks/
  run_regression_check.py   ← regression gates (+ host speed context)
  run_study.py              ← benchmark study (backend exploration)
  host_profiles/*.json      ← recorded timings per machine (context only)
  baselines/*.json          ← committed regression goldens
```

Demos stay in `examples/scripts/` for teaching; **demo-check runners** that execute them live in `tests/demo_checks/`.

### Human (IDE — open script, click **Run**)

| When | Open and Run |
| --- | --- |
| **Daily** | [`tests/run/run_contract_tests.py`](run/run_contract_tests.py) |
| **Before push** | [`tests/run/run_pre_push.py`](run/run_pre_push.py) |
| **After sim/trajopt/MPC work** | [`tests/run/run_regression_gates.py`](run/run_regression_gates.py) |
| **Demo/catalog sanity** | [`tests/run/run_demo_checks.py`](run/run_demo_checks.py) |
| **Notebook smoke** | [`tests/run/run_notebook_checks.py`](run/run_notebook_checks.py) |
| **Backend perf tables** | [`tests/run/run_benchmark_study.py`](run/run_benchmark_study.py) |
| **Pick one check** | [`tests/run/run_checks.py`](run/run_checks.py) — set `CHECK = "contract_tests"` at top |

Graphics visual checklist (local only): [`tests/demo_checks/run_graphics_visual_check.py`](demo_checks/run_graphics_visual_check.py).

Optional toggles: constants at the top of each `tests/run/*.py` file.

### Agent (terminal / CI — exact commands)

| Situation | Command |
| --- | --- |
| Always before push | `ruff check . && ruff format --check .` |
| Docs/markdown only | skip pytest |
| Narrow module change | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or handoff | `pytest` |
| Compile / `Simulator` / trajopt / MPC | `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Backend perf exploration | `python benchmarks/run_study.py --list` |
| Demo-check change | `python tests/demo_checks/run_catalog_checks.py --fast` and/or `run_flagship_demos.py` |
| Notebook change | `MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py` |

Rules: [AGENTS.md](../AGENTS.md).

### CI (GitHub Actions)

Workflow: [`.github/workflows/test.yml`](../.github/workflows/test.yml).

| Job | Steps |
| --- | --- |
| **`test`** | ruff + `pytest` (py 3.10–3.13; demo-check bridge; JAX demos may skip) |
| **`regression`** | regression gates `--suite all --tiny …` + **flagship demos** + **notebook smoke** (py 3.12 + JAX + viz) |

### At a glance

| Layer | Human IDE (`tests/run/`) | Agent / CI | CI job |
| --- | --- | --- | --- |
| **Contract tests** | `run_contract_tests.py` | `pytest` | `test` |
| **Regression gates** | `run_regression_gates.py` | `benchmarks/run_regression_check.py` | `regression` |
| **Benchmark study** | `run_benchmark_study.py` | `benchmarks/run_study.py` | — |
| **Graphics contract** | (in contract tests) | `test_flagship_graphics_contract.py` | `test` |
| **Graphics visual** | `tests/demo_checks/run_graphics_visual_check.py` | local | — |
| **Demo checks** | `run_demo_checks.py` | `tests/demo_checks/run_*.py` | `test` + `regression` |
| **Notebook smoke** | `run_notebook_checks.py` | `tests/demo_checks/run_notebook_checks.py` | `regression` |
| **Pre-push** | `run_pre_push.py` | ruff + pytest | `test` |

### Performance — regression gates vs benchmark study vs host context

| | **Regression gates** | **Benchmark study** `run_study.py` | **Host profiles** |
| --- | --- | --- | --- |
| **Role** | Pass/fail vs `benchmarks/baselines/` | Backend tables on your machine | **Context only** — timings from other hosts |
| **Location** | `benchmarks/` | `benchmarks/` | `benchmarks/host_profiles/*.json` |
| **CI** | ✅ | ❌ | printed on regression runs (informational) |

Regression gates print a **host speed context** table after each suite (compare current run to recorded profiles). Gates unchanged. Record your machine:

```bash
python benchmarks/run_regression_check.py --suite f_mpc --tiny \
  --record-host-profile my_laptop --host-profile-label "My workstation"
```

Detail: [benchmarks/README.md](../benchmarks/README.md).

---

## Validation layers

| Layer | Purpose | Entry command | CI job |
| --- | --- | --- | --- |
| **Contract tests** | API types, shapes, compile/sim/MPC behavior | `pytest` | `test` |
| **Regression gates** | Accuracy goldens + guarded NLP/trajopt solve time | `run_regression_check.py --suite all` | `regression` |
| **Benchmark study** | Machine/GPU exploration tables | `run_study.py --list` | — |
| **Graphics contract** | Draw-list + headless PNG checks | `test_flagship_graphics_contract.py` (in `pytest`) | `test` |
| **Graphics visual** | You confirm Meshcat/MPL/Plotly locally | `run_graphics_visual_check.py` | — |
| **Demo checks** | Catalog + flagship demos must not throw | `run_catalog_checks.py`, `run_flagship_demos.py` | `test` (pytest bridge) + `regression` (full flagships w/ JAX) |
| **Notebook smoke** | Teaching notebooks' code cells must not throw | `run_notebook_checks.py` | `regression` |

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

**Domain modules** (22 files after contract-test consolidation): `test_core`, `test_backends`,
`test_compile`, `test_diagrams`, `test_simulation`, `test_step_discrete`,
`test_hybrid`, `test_dynamics_catalog`, `test_mechanical_robotics`, `test_blocks`,
`test_control_analysis`, `test_costs_optimizer`, `test_planning`, `test_mpc`,
`test_graphics`, `test_geometry`, `test_engine_jax`, `test_jax_planning`,
`test_symbolic`, `test_benchmark_helpers`, `test_demo_check_runners`,
`test_flagship_graphics_contract`. Re-merge helper:
[`merge_contract_tests.py`](merge_contract_tests.py).

Kinematic render check (graphics contract): ``run_flagship_graphics.py`` and manifest under
``tests/fixtures/kinematic_baseline/``.
Regenerate manifest: `python tests/fixtures/kinematic_baseline/regenerate_manifest.py`.

Shared fixtures: `graphics_contract_helpers.py` (draw-list resolution),
`planning_helpers.py` (RRT holonomic obstacle scene).

Benchmark regression lives under repo-root `benchmarks/`; helper API
drift guards in `test_benchmark_helpers.py`. See [Entry points](#entry-points) for
regression CI vs local commands and [benchmarks/README.md](../benchmarks/README.md).

**Demo checks** live in [`tests/demo_checks/`](demo_checks/) (assertions and reports
there). A thin pytest bridge ([`test_demo_check_runners.py`](unittest/test_demo_check_runners.py))
invokes those runners so the CI ``test`` job covers catalog / non-JAX flagships /
graphics without re-implementing checks as pytest cases. The CI ``regression``
job (JAX installed) re-runs ``run_flagship_demos.py`` so JAX flagships are gated,
and runs ``run_notebook_checks.py`` so teaching notebooks' code cells do not throw.

Demos and teaching notebooks must **not** branch on smoke/CI env vars — the runner
only sets headless display knobs. A contract test scans ``examples/notebooks/`` and
``examples/scripts/`` for forbidden harness names (see
``test_examples_have_no_smoke_env_hooks``).

```bash
python tests/demo_checks/run_catalog_checks.py --fast
# Flagship whitelist: subprocess each demo's __main__ (no demo source changes):
python tests/demo_checks/run_flagship_demos.py
# Teaching notebooks under examples/notebooks/ (auto-discovered; overrides for deps/timeouts):
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
# Nightly / local: every example script:
python tests/demo_checks/run_all_demos.py --continue-on-error
```

Opt-in pytest bridge for notebooks: `MINILINK_NOTEBOOK_CHECKS=1 pytest tests/unittest/test_demo_check_runners.py`.

IDE launchers: [`tests/run/`](run/). Headless PNG check: `tests/demo_checks/run_flagship_graphics.py`.

`tests/manual/` and `tests/bugs/` are removed — use `examples/scripts/` for
demos and unittest for contracts.

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

`SDL_VIDEODRIVER=dummy` lets pygame tests initialize in headless CI or
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
