# minilink

`minilink` is a Python-native block-diagram simulation framework for building and analyzing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

## ⚡ Key Strengths

-   **MIMO Composable Modeling**: Connect N arbitrary blocks via named ports.
-   **Optimized Execution**: Compiled evaluators (`compile` / `compile_diagram`) with flat `ExecutionPlan` execution for ODE integration and JAX.
-   **Differentiable Simulation**: Functional paths designed for JAX-based tracing and autodiff.
-   **Headless-First Design**: Pure NumPy core, independent of graphics and simulation backends.
-   **Pyro Successor**: A flexible, port-based foundation for the [pyro](https://github.com/SherbyRobotics/pyro) robotics toolbox.

## 🚀 Quick Start

```python
from minilink.blocks.basic import Integrator
from minilink.blocks.sources import Step
from minilink.core.analysis import Simulator
from minilink.core.diagram import DiagramSystem
from minilink.graphical.plotting import plot_trajectory

# 1. Assemble a simple integrator system
diagram = DiagramSystem()
diagram.add_subsystem(Integrator(), "plant")
diagram.add_subsystem(Step(), "source")

# 2. Wire the diagram
diagram.connect("source", "y", "plant", "u")

# 3. Simulate and analyze
sim = Simulator(diagram)
traj = sim.solve(tf=10)
plot_trajectory(diagram, traj)  # or sim.solve(tf=10, show=True)
```

## 🛠 Tech Stack
-   **Core**: NumPy
-   **Simulation**: SciPy (`solve_ivp`); compiled `DynamicsEvaluator` (`f`, `h`, `outputs`, optional JAX JIT)
-   **Visualization**: Matplotlib, MeshCat, Pygame; Graphviz for diagrams
-   **Acceleration**: JAX (optional; leaf and diagram evaluators JIT core callables)

---

## 📖 Documentation Guide

-   **[DESIGN.md](DESIGN.md)**: Deep dive into the architecture, signals, systems, and coding conventions.
-   **[ROADMAP.md](ROADMAP.md)**: Project status, development phases, and the Pyro 2.0 migration plan.

---

## 💡 Examples
-   **Interactive Demos**: See `examples/scripts/` — diagrams (`demo_diagram_compiling.py`, `demo_internal_signals.py`), JAX physics (`demo_physics_jax_*.py`), animations (`demo_animations.py`).
-   **Notebooks**: [Colab Tutorial](https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing)
