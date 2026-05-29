# AGENTS.md

## Cursor Cloud specific instructions

### Project overview

minilink is a pure Python library (no external services, databases, or web servers).
See [README.md](README.md) for user workflows, the easy interface
(`compute_trajectory`, facades, composition shortcuts), and lower-level APIs.
See [DESIGN.md](DESIGN.md) for architecture and [flows.md](flows.md) for call paths.

### Environment

- **Python 3.10+** required (`pyproject.toml` specifies `>=3.10`).
- System `graphviz` package must be installed (`apt install graphviz`) for the Python `graphviz` bindings to work.
- Install dev dependencies: `pip install -e ".[dev]"` from the repo root.
- Optional extras: `.[jax]`, `.[symbolic]`, `.[visualization]` — JAX tests are skipped gracefully if not installed.

### Running tests and lint

```bash
# Lint
ruff check .
ruff format --check .

# Tests (configured via pyproject.toml to use tests/unittest/)
pytest
```

### Running examples/demos

Example scripts live in `examples/scripts/`. Run from the repo root:

```bash
python examples/scripts/<script>.py
```

For headless environments, set `matplotlib.use('Agg')` before importing minilink graphical modules (or set `MPLBACKEND=Agg`). Animation demos (`animate()`) require a display; plotting/simulation works headless.

### Gotchas

- The `plot_diagram()` method may emit D-Bus errors in headless environments — these are harmless and do not affect output.
- `plot_trajectory(..., show=False)` suppresses display for trajectory figures.
- The `agent.md` file references a `dev-h26` conda env for the maintainer's local machine. In cloud agents, use system Python 3.10+ with `pip install -e ".[dev]"` instead.
