# minilink

`minilink` is a Python-native block-diagram framework for modeling, compiling, simulating, and visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab demo: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## Key Strengths

- **Composable MIMO modeling** with named ports and diagrams
- **Compiled execution** through flat `ExecutionPlan` evaluators for NumPy and JAX (`compile_backend` can be `"auto"`)
- **Differentiable-friendly design** with functional `f(x, u, t)` style APIs
- **Headless-first core** decoupled from graphics and solver backends
- **Canonical `Trajectory` and `compute_forced`** for run-level inputs alongside `Simulator`

## Quick Start

```python
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.graphical.plotting import plot_trajectory
from minilink.simulation.simulator import Simulator

diagram = DiagramSystem()
diagram.add_subsystem(Integrator(), "plant")
diagram.add_subsystem(Step(), "source")
diagram.connect("source", "y", "plant", "u")

traj = Simulator(diagram, tf=10.0).solve()
plot_trajectory(diagram, traj)
```

## Tech Stack

- **Core**: NumPy
- **Simulation**: `minilink.simulation` with SciPy and fixed-step backends
- **Compilation**: `DynamicsEvaluator` on NumPy or JAX
- **Visualization**: multiple rendering backends (Matplotlib, MeshCat, Pygame) for animation; Graphviz for diagram graphs
- **Acceleration**: optional JAX JIT and autodiff

## Current State

- `core/` is the most mature subsystem and defines the main modeling contract.
- The official simulation path is `System.compute_trajectory(...)` / `minilink.simulation.Simulator`.
- The official trajectory object is `minilink.core.trajectory.Trajectory`.
- The compile pipeline, evaluators, and simulator are in architecture-validation / integration stage (`compile_backend` can be `"auto"`; long JAX sims may use an auto fixed-step path when the grid is uniform and non-stiff).
- `graphical/` has a **stabilizing** matplotlib layer—`matplotlib_style` sizing and theming, notebook-aware stacked plot height, and trajectory **plot** modes—while other renderers and hooks remain early work.
- `mechanics/`, symbolic mechanics, and `physics/` are early MVP work.
- `dynamics/` holds reusable plant models; `core/blocks/` is wiring and signal primitives; `control/` is controller blocks; `planning/` has early family-level architecture contracts; these layers are still maturing.
- `dynamics/pendulum/` includes tutorial `CartPole` and `DoublePendulum` on `MechanicalSystem`; see `tests/manual/demo_cartpole_doublependulum.py` for a quick animation smoke script.

## Documentation Guide

- **[DESIGN.md](DESIGN.md)**: architecture, core contracts, and coding standards
- **[ROADMAP.md](ROADMAP.md)**: subsystem maturity and priorities
- **[agent.md](agent.md)**: project-specific AI collaboration rules, the TRL lifecycle table, and a **supplemental** banding for recent matplotlib and simulation features (read with `DESIGN.md` / `ROADMAP.md`)

## Benchmarks

- **API**: `minilink.benchmark` — `benchmark_f_speeds` / `print_f_speed_table` for compiled `f` timing; `benchmark_sim_speed_matrix` (pass a `pairs` list, e.g. `DEFAULT_SWEEP_PAIRS`) and `benchmark_sim_backend` on a built `system`; `run_standard_sim_suite` for the three standard cases; optional stress systems in `benchmark/scenario/` (see [DESIGN.md](DESIGN.md)).
- **Scripts**: flat runners under `tests/benchmark/` (for example `benchmark_simulator_speed_matrix.py`, `benchmark_simulator_standard.py`); execute with `python tests/benchmark/<script>.py` from the repo root with the package on `PYTHONPATH`.

## Examples

- **Scripts**: `examples/scripts/` for diagram compilation, internal signals, animations, and JAX physics demos
- **Manual**: `tests/manual/demo_cartpole_doublependulum.py` — quick `compute_forced` + `animate` smoke test for the pendulum demos
- **Notebook**: [Colab Tutorial](https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing)
