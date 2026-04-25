# Test suite organization

The default suite is split into core behavioral tests and optional-extra tests.

## Core behavior

Run this on a minimal development install:

```bash
python -m pytest -m "not optional"
```

Core tests cover package behavior that should work without optional extras such as
JAX, SymPy, MeshCat, or Pygame.

## Optional functionality

Run these after installing the matching extras:

```bash
python -m pytest -m optional
python -m pytest -m jax
python -m pytest -m symbolic
python -m pytest -m visualization
```

The optional markers are:

- `jax`: JAX-backed evaluators, mechanics, and physics.
- `symbolic`: SymPy-backed symbolic mechanics.
- `visualization`: MeshCat and Pygame renderer smoke tests.

`pytest` without a marker still discovers all unit tests; optional tests skip when
their dependency is unavailable.
