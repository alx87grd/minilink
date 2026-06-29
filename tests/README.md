# Test suite organization

The default pytest discovery lives in `tests/unittest`.

## Philosophy

Tests guard **stable public contracts** (compile evaluators, planning transcriptions,
graphics frame keys, catalog equation references)—not implementation trivia or
third-party print formatting. Prefer one parametrized or table-driven test over
many near-duplicate files. Deep dynamics checks for a few representative plants
live in `test_catalog_plant_contracts.py`; broad catalog smoke in
`test_catalog_migration.py`; kinematic render smoke in
`test_kinematic_regression.py` (manifest only — PNGs are local/gitignored).

Shared fixtures: `graphics_contract_helpers.py` (draw-list resolution),
`planning_helpers.py` (RRT holonomic obstacle scene).

Benchmark **performance** helpers live under repo-root `benchmarks/`; import smoke
only in `test_benchmark_smoke.py` (not correctness asserts).

`tests/manual/` and `tests/bugs/` are removed — use `examples/scripts/` for
smoke scripts and unittest for contracts.

## Core behavior without optional extras

Run tests that should pass with only the required project dependencies and the
`dev` extra installed:

```bash
pytest -m "not optional"
```

This excludes tests that need optional extras such as JAX, SymPy, meshcat, or
pygame.

## Default local run

```bash
pytest
```

Optional tests remain collected, but skip at runtime when their dependency is
not installed.

## Full functionality run

Install all extras, then run either the optional subset or the whole suite:

```bash
pip install -e ".[dev,symbolic,jax,visualization]"
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

When adding optional behavior, put the import inside a guarded block and add the
appropriate marker(s). This keeps `pytest -m "not optional"` a dependable
behavior suite for minimal installations.
