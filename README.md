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
- `symbolic/mechanics/`, `dynamics/`, and `physics/` are early MVP work.
- Curated plants live under `dynamics/catalog/`; inheritance abstractions under `dynamics/abstraction/`. `core/blocks/` is wiring and signal primitives; `control/` is controller blocks; `planning/` has early family-level architecture contracts; these layers are still maturing.
- `dynamics/catalog/pendulum/` includes tutorial `CartPole` and `DoublePendulum` on `MechanicalSystem`. For a quick smoke path, run `python minilink/dynamics/catalog/pendulum/cartpole.py` or `python minilink/dynamics/catalog/pendulum/double_pendulum.py` from the repo root (each module has a small `__main__` using `compute_forced` + `animate`), or see `tests/unittest/test_pendulum_plants.py`.

## Documentation Guide

- **[DESIGN.md](DESIGN.md)**: architecture, core contracts, **on-disk package layout and module naming (§2)**, and coding standards
- **[ROADMAP.md](ROADMAP.md)**: subsystem maturity and priorities
- **[agent.md](agent.md)**: project-specific AI collaboration rules, the TRL lifecycle table, and a **supplemental** banding for recent matplotlib and simulation features (read with `DESIGN.md` / `ROADMAP.md`)

## Benchmarks

- **API**: `minilink.compile.benchmark` / `minilink.simulation.benchmark` / `minilink.planning.trajectory_optimization.benchmark` — benchmark functions return structured results and print helpers format tables; optional stress systems live in `minilink/simulation/scenarios/` (see [DESIGN.md](DESIGN.md)).
- **Scripts**: flat runners under `tests/benchmark/` (for example `benchmark_simulator_speed_matrix.py`, `benchmark_simulator_standard.py`); execute with `python tests/benchmark/<script>.py` from the repo root with the package on `PYTHONPATH`.

## Examples

- **Scripts**: `examples/scripts/` for diagram compilation, internal signals, animations, and JAX physics demos
- **Manual**: `tests/manual/` for one-off experiments (pendulum `compute_forced` + `animate` smokes: see **Current State** above)
- **Notebook**: [Colab Tutorial](https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing)

### JAX vs NumPy

Minilink runs end-to-end on NumPy alone. JAX is an optional accelerator: install with `pip install minilink[jax]` to unlock JAX-traced plants (`Jax<X>` classes), JIT-compiled simulator rollouts (`Simulator(..., compile_backend="jax"|"auto")`), and analytic gradients in trajectory optimization (`Jax*Transcription`). Without JAX, those classes still **import**; only their methods raise. Maintainer-facing rules and expectations (twin pattern, backends, tests without JAX at collection time) are in [`agent.md`](agent.md) §3.1 and [`DESIGN.md`](DESIGN.md) §2.8 / §3.5.
