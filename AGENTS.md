# AGENTS.md

Cloud-agent notes. Full rules: [agent.md](agent.md). User API: [README.md](README.md).

## Environment

- Python 3.10+; `pip install -e ".[dev]"` from repo root.
- System `graphviz` for diagram bindings (`apt install graphviz`).
- Optional extras: `.[jax]`, `.[symbolic]`, `.[visualization]`, `.[plotting]`, `.[ipopt]`.

## Verify

```bash
ruff check .
ruff format --check .
pytest
```

## Demos

```bash
python examples/scripts/<script>.py   # from repo root
```

Headless: `MPLBACKEND=Agg` before graphical imports. `plot_diagram()` D-Bus
warnings in headless environments are harmless. `animate()` needs a display;
plotting/simulation work headless with `show=False`.

Maintainer conda env names (e.g. `dev-h26`) are informal—in cloud, use system
Python 3.10+ with pip extras.
