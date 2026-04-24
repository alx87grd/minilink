# minilink

`minilink` is a Python-native block-diagram framework for modeling, compiling, simulating, and visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab demo: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## Key Strengths

- **Composable MIMO modeling** with named ports and diagrams
- **Compiled execution** through flat `ExecutionPlan` evaluators for NumPy and JAX
- **Differentiable-friendly design** with functional `f(x, u, t)` style APIs
- **Headless-first core** decoupled from graphics and solver backends
- **Pyro successor direction** for robotics and control workflows

## Quick Start

```python
from minilink.blocks.basic import Integrator
from minilink.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.graphical.plotting import plot_trajectory
from minilink.simulation import Simulator

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
- **Visualization**: Matplotlib, MeshCat, Pygame, Graphviz
- **Acceleration**: optional JAX JIT and autodiff

## Current State

- `core/` is the most mature subsystem and defines the main modeling contract.
- The official simulation path is `System.compute_trajectory(...)` / `minilink.simulation.Simulator`.
- The official trajectory object is `minilink.core.trajectory.Trajectory`.
- The compile pipeline, evaluators, and simulator are in architecture-validation / integration stage.
- `graphical/`, `mechanics/`, symbolic mechanics, and `physics/` are early MVP work.
- `blocks/`, `planning/`, and `control/` are still exploratory rather than stabilized library layers.

## Documentation Guide

- **[DESIGN.md](DESIGN.md)**: architecture, core contracts, and coding standards
- **[ROADMAP.md](ROADMAP.md)**: subsystem maturity, priorities, and Pyro migration direction
- **[agent.md](agent.md)**: project-specific AI collaboration rules and TRL definitions

## Benchmarks

- **API**: `minilink.benchmark` — `benchmark_f_speeds` / `print_f_speed_table` for compiled `f` timing; `benchmark_sim_speed_matrix` (pass a `pairs` list, e.g. `DEFAULT_SWEEP_PAIRS`) and `benchmark_sim_backend` on a built `system`; `run_standard_sim_suite` for the three standard cases (see **§4.6** in [DESIGN.md](DESIGN.md)).
- **Scripts**: flat runners under `tests/benchmark/` (for example `benchmark_simulator_speed_matrix.py`, `benchmark_simulator_standard.py`); execute with `python tests/benchmark/<script>.py` from the repo root with the package on `PYTHONPATH`.

## Examples

- **Scripts**: `examples/scripts/` for diagram compilation, internal signals, animations, and JAX physics demos
- **Notebook**: [Colab Tutorial](https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing)
