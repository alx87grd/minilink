# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## Quick start

Compose → simulate → plot. No manual `compile()` for everyday use.

```python
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step

diagram = Step() >> Integrator()
traj = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(traj, signals=("x", "u"))
```

Closed loop: `Controller() @ Plant()` using ports `r`, `y`, `u`.

```python
from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

closed = PendulumPDController() @ Pendulum()
traj = closed.compute_trajectory(tf=10.0)
closed.animate(traj)
```

## Easy interface (default)

All methods live on `System` / `DiagramSystem`. Latest rollout stored on `sys.traj`.

| Goal | Call |
| --- | --- |
| Simulate | `compute_trajectory(tf=..., dt=..., solver=..., compile_backend="numpy")` |
| Force input | `compute_forced(u, input_port_id=...)` |
| Time plot | `plot_trajectory(traj, signals=("x", "u", "block:port"))` |
| Phase plane | `plot_phase_plane(traj, x_axis=0, y_axis=1)` |
| Diagram | `plot_diagram()` |
| Animate | `animate(traj, renderer="matplotlib")` |
| Compile | `compile(backend="numpy")` |
| Compose | `a + b`, `a >> b`, `ctl @ plant`, `.autowire()` |

Common kwargs: `t0`, `tf`, `n_steps`, `dt`, `x0`, `solver`, `compile_backend`,
`verbose`. Plotly: `pip install "minilink[plotting]"` and `backend="plotly"`.

## Workflows (pointers)

**Catalog plant** — `minilink.dynamics.catalog.*` (e.g. `Pendulum`, `CartPole`,
`DynamicBicycle`); bases in `dynamics/abstraction`.

**Custom block** — subclass `DynamicSystem` with `input_dim`/`output_dim` for
simple SISO, or `StaticSystem` + `add_input_port` / `add_output_port` for
multi-port blocks. Use `get_port_values_from_u(u, "r", "y")` in equations.
Examples in [DESIGN.md](DESIGN.md) §4 and `examples/scripts/diagrams/`.

**Explicit diagram** — when shortcuts are insufficient:

```python
from minilink.core.diagram import DiagramSystem

d = DiagramSystem()
d.connection_verbose = False
d.add_subsystem(plant, "plant")
d.connect("controller", "u", "plant", "u")
d.connect_new_output_port("plant", "y", "y")
```

**Trajopt** — `PlanningProblem` + transcription + `TrajectoryOptimizationPlanner`.
Full example: `examples/scripts/trajectory_optimization/demo_cartpole_direct_collocation.py`.
Costs go on the problem, not the plant.

**Standalone NLP** — `MathematicalProgram` + `Optimizer` (TRL 1). Example:
`examples/scripts/optimization/demo_simple_optimization.py`.

## Lower level (when needed)

- **`Simulator`** — multiple rollouts, `solve_forced`, inspect solver without
  mutating `sys.traj`.
- **`compile()`** — benchmarks, JAX JIT, custom integrators. Recompile when JAX
  diagram params change (parametric tier WIP).
- **Explicit wiring** — canonical for nested/nonstandard topology; shortcuts
  cannot merge two diagrams or nest with `>>`.

Call chains: [flows.md](flows.md). Contracts: [DESIGN.md](DESIGN.md).

## Imports

No top-level re-exports yet. Import from defining modules, e.g.
`minilink.core.system`, `minilink.core.blocks.basic`, `minilink.simulation.simulator`,
`minilink.planning.problems`.

## Install

```bash
pip install minilink
pip install "minilink[jax]"            # optional: JAX
pip install "minilink[visualization]"  # meshcat, pygame
pip install "minilink[plotting]"       # plotly
pip install "minilink[symbolic]"       # sympy
pip install "minilink[ipopt]"          # ipopt
```

Dev clone: `pip install -e ".[dev]"` then `pytest`. System package `graphviz`
needed for diagrams (`apt install graphviz`).

## Design rules (summary)

- NumPy baseline; JAX optional and explicit.
- Equation paths native-array; convert only at boundaries.
- `params is None` → defaults; never `params or self.params`.

Full rules: [DESIGN.md](DESIGN.md) §1 and §4.

## Docs and examples

| Doc | Purpose |
| --- | --- |
| [DESIGN.md](DESIGN.md) | Principles and contracts |
| [flows.md](flows.md) | Minimal call chains |
| [ROADMAP.md](ROADMAP.md) | Maturity and priorities |
| [agent.md](agent.md) | Agent/maintainer rules |

Demos: `examples/scripts/{diagrams,plots,animation,trajectory_optimization,...}`.
Run from repo root: `python examples/scripts/diagrams/demo_diagram_shortcuts.py`.

Headless: `MPLBACKEND=Agg` or `show=False` on plot calls.
