 # Test suite organization

The default pytest discovery lives in `tests/unittest`.

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
