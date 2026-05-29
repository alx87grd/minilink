# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## Quick start

Compose → simulate → plot on one object. No manual `compile()` for everyday work.

```python
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step

diagram = Step() >> Integrator()
traj = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(traj, signals=("x", "u"))
```

Closed loop: `Controller() @ Plant()` (`r`, `y`, `u` ports). Catalog plants:
`minilink.dynamics.catalog.*`. Demos: `examples/scripts/`.

## Easy interface

| Goal | Call |
| --- | --- |
| Simulate / force | `compute_trajectory(...)`, `compute_forced(u, ...)` |
| Plot / phase / diagram | `plot_trajectory`, `plot_phase_plane`, `plot_diagram` |
| Animate | `animate(traj)` |
| Compose | `+`, `>>`, `@`, `.autowire()` |

Results cache on `sys.traj`. Lower level when needed: `Simulator`, `compile()`,
explicit `DiagramSystem.connect` — see [flows.md](flows.md).

## Install

```bash
pip install minilink
pip install "minilink[jax]" "minilink[plotting]" "minilink[visualization]"  # optional
pip install -e ".[dev]" && pytest   # dev clone
```

Requires system `graphviz` for diagrams. Import from defining modules (no top-level
re-exports yet), e.g. `minilink.core.system`, `minilink.core.blocks.basic`.

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [flows.md](flows.md) — minimal call chains
- [ROADMAP.md](ROADMAP.md) — maturity and priorities
- [agent.md](agent.md) — maintainer / agent rules

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` → defaults (never `params or self.params`).
