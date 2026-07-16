# Test suite organization

**Entry points (human · agent · CI):** see [Entry points](#entry-points) below — authoritative for all three audiences.

Six-layer vision (detail): [docs/plans/test-benchmark-consolidation.md](../docs/plans/test-benchmark-consolidation.md).

---

## Entry points

**Prerequisites:** repo root, conda env **`minilink`**, `PYTHONPATH=.` (see [README.md#install](../README.md#install)).

### Human (IDE — open script, click **Run**)

No terminal commands to remember. Scripts live in [`scripts/checks/`](../scripts/checks/).

| When | Open and Run |
| --- | --- |
| **Daily** | [`scripts/checks/run_l1_tests.py`](../scripts/checks/run_l1_tests.py) |
| **Before push** | [`scripts/checks/run_pre_push.py`](../scripts/checks/run_pre_push.py) |
| **After sim/trajopt/MPC work** | [`scripts/checks/run_l2_regression.py`](../scripts/checks/run_l2_regression.py) |
| **Demo/catalog sanity** | [`scripts/checks/run_l6_smokes.py`](../scripts/checks/run_l6_smokes.py) |
| **Backend perf tables (local)** | [`scripts/checks/run_l3_benchmark_study.py`](../scripts/checks/run_l3_benchmark_study.py) |
| **One file, pick check** | [`scripts/checks/run_checks.py`](../scripts/checks/run_checks.py) — set `CHECK = "l1"` at top |

Optional toggles (constants at top of each script): `EXTRA_ARGS`, `MARKER`, `CI_MODE`, `PRESET`, etc.

### Agent (terminal / CI — exact commands)

Agents and GitHub Actions use **CLI only** (same underlying tools as the IDE scripts).

| Situation | Command |
| --- | --- |
| Always before push | `ruff check . && ruff format --check .` |
| Docs/markdown only | skip pytest |
| Narrow module change | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or handoff | `pytest` |
| Compile / `Simulator` / trajopt / MPC | `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Backend perf exploration (no gate) | `python benchmarks/run_study.py --list` → [benchmarks/README.md](../benchmarks/README.md) |
| User-facing demo/smoke change | `python examples/scripts/_smoke/run_catalog_smokes.py --fast` and/or `run_flagship_demos.py` |

Rules: [AGENTS.md](../AGENTS.md). L1 path: `tests/unittest/` only. Demos: L6 smokes under `examples/scripts/`.

### CI (GitHub Actions)

Workflow: [`.github/workflows/test.yml`](../.github/workflows/test.yml).

| Job | Trigger | Exact steps |
| --- | --- | --- |
| **`test`** | Python 3.10–3.13 matrix | `pip install -e ".[dev]"` → `ruff check .` → `ruff format --check .` → `pytest` |
| **`regression`** | After `test` passes; Python 3.12 + JAX | `pip install -e ".[dev,jax]"` → `PYTHONPATH=$PWD python benchmarks/run_regression_check.py --suite all --tiny --factor 6 --speed-gate-suffixes solve_s,nlp_s,speedup` |

Pytest in CI: **L1** + **L4** + **L6 bridge** (`test_smoke_runners.py`). **L2** = `regression` job. **L3/L5** not in CI.

### Human (terminal — optional)

Same tools as IDE scripts; use when you prefer the shell.

```bash
conda activate minilink
pytest                                                    # L1 daily
ruff check . && ruff format --check . && pytest           # pre-push
PYTHONPATH=. python benchmarks/run_regression_check.py --suite all   # L2
python examples/scripts/_smoke/run_catalog_smokes.py --fast          # L6
python benchmarks/run_study.py --list                                 # L3
```

### At a glance (all audiences)

| Layer | Human IDE | Agent / CI command | CI job |
| --- | --- | --- | --- |
| **L1** Contracts | `run_l1_tests.py` | `pytest` | `test` |
| **L2** Regression | `run_l2_regression.py` | `run_regression_check.py --suite all …` | `regression` |
| **L3** Benchmark study | `run_l3_benchmark_study.py` | `run_study.py --preset …` | — |
| **L4** Graphics auto | (in L1 pytest) | `test_flagship_graphics_contract.py` | `test` |
| **L5** Graphics visual | `run_graphics_visual_check.py` | local only | — |
| **L6** Demo smokes | `run_l6_smokes.py` | `run_catalog_smokes.py` / `run_flagship_demos.py` | `test` (subprocess) |
| **Pre-push** | `run_pre_push.py` | ruff + pytest | `test` |

### Performance / backends — L2 vs L3

| | **L2** `run_l2_regression.py` / `run_regression_check.py` | **L3** `run_l3_benchmark_study.py` / `run_study.py` |
| --- | --- | --- |
| **Purpose** | Pass/fail vs committed JSON baselines | Explore tables on *your* machine/GPU |
| **Speed** | Gated: `speedup`, `solve_s`, `nlp_s` (CI: factor 6×) | Report only — compare runs yourself |
| **Accuracy** | Gated: trajectory goldens, `rel_err_l2`, checkpoints | — |
| **CI** | ✅ `regression` job | ❌ never |
| **When** | After compile/sim/trajopt/MPC changes; before merge | New machine, backend tuning, perf investigation |

L2 suites: `core_perf`, `integration`, `solve_speed`, `e4`, `f_mpc`. Detail: [benchmarks/README.md](../benchmarks/README.md).

`test_benchmark_smoke.py` (in `pytest`) only checks benchmark **helpers import** — not performance gates.

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
