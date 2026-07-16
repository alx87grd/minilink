# Test suite organization

The default pytest discovery lives in `tests/unittest`.

## Local environment

Setup: [README.md#install](../README.md#install) (conda env **`minilink`**, `PYTHONPATH` = repo root).
Agent rules: [AGENTS.md](../AGENTS.md).

From repo root:

```bash
conda activate minilink
python -m pytest
```

Full suite (optional deps and headless pygame):

```bash
conda activate minilink
SDL_VIDEODRIVER=dummy python -m pytest
```

Non-interactive:

```bash
conda run -n minilink python -m pytest
```

Marker policy and pip-only installs are below; prefer the conda env so optional
tests behave consistently across machines.

## Philosophy

Tests guard **stable public contracts** (compile evaluators, planning transcriptions,
graphics frame keys, catalog equation references)—not implementation trivia or
third-party print formatting. Prefer one parametrized or table-driven test over
many near-duplicate files.

**Domain modules** (~21 files after L1 consolidation): `test_core`, `test_backends`,
`test_compile`, `test_diagrams`, `test_simulation`, `test_step_discrete`,
`test_hybrid`, `test_dynamics_catalog`, `test_mechanical_robotics`, `test_blocks`,
`test_control_analysis`, `test_costs_optimizer`, `test_planning`, `test_mpc`,
`test_graphics`, `test_geometry`, `test_engine_jax`, `test_jax_planning`,
`test_symbolic`, `test_benchmark_smoke`, `test_smoke_runners`. Re-merge helper:
`scripts/merge_l1_tests.py`.

Deep dynamics checks for a few representative plants
live in `test_dynamics_catalog.py`; broad catalog smoke in L6
(`run_catalog_smokes.py`); kinematic render smoke in `test_graphics.py`
(manifest under `tests/fixtures/kinematic_baseline/`).
Regenerate manifest: `python tests/fixtures/kinematic_baseline/regenerate_manifest.py`.

Shared fixtures: `graphics_contract_helpers.py` (draw-list resolution),
`planning_helpers.py` (RRT holonomic obstacle scene).

Benchmark **Layer B** regression (accuracy goldens + NLP/trajopt **solve speed** gates)
lives under repo-root `benchmarks/`; pytest smoke-tests helpers in
`test_benchmark_smoke.py`. **CI** runs `python benchmarks/run_regression_check.py
--suite all --tiny` with JAX; local pre-handoff: `--suite all` on a reference machine
(see [benchmarks/README.md](../benchmarks/README.md)).

**Layer L6** catalog and flagship demo smokes (must not throw):

```bash
python examples/scripts/_smoke/run_catalog_smokes.py --fast
python examples/scripts/_smoke/run_flagship_demos.py
```

Pytest: `tests/unittest/test_smoke_runners.py`. **Layer L4** graphics headless:
`python examples/scripts/_smoke/run_flagship_graphics.py`. **Layer L5** visual (local):
`python examples/scripts/_smoke/run_graphics_visual_check.py`.

`tests/manual/` and `tests/bugs/` are removed — use `examples/scripts/` for
smoke scripts and unittest for contracts.

## Core behavior without optional extras

Run tests that should pass with only the required project dependencies and the
`dev` extra installed:

```bash
pytest -m "not optional"
```

This excludes tests that need optional extras such as JAX, SymPy, meshcat,
pygame, plotly, or cyipopt.

## Default local run

```bash
pytest
```

Optional tests remain collected, but skip at runtime when their dependency is
not installed.

## Full functionality run

With the `minilink` conda env (recommended):

```bash
conda activate minilink
SDL_VIDEODRIVER=dummy pytest -m optional
SDL_VIDEODRIVER=dummy pytest
```

Or install all pip extras in another Python 3.10+ environment:

```bash
pip install -e ".[dev,symbolic,jax,visualization,plotting,ipopt]"
SDL_VIDEODRIVER=dummy pytest -m optional
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
