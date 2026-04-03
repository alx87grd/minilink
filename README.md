# minilink

`minilink` is a Python-native block-diagram simulation framework for building and analyzing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

## ⚡ Key Strengths

-   **MIMO Composable Modeling**: Connect N arbitrary blocks via named ports.
-   **Optimized Execution**: Compiled topological graphs (`f_fast`) for efficient ODE integration.
-   **Differentiable Simulation**: Functional paths designed for JAX-based tracing and autodiff.
-   **Headless-First Design**: Pure NumPy core, independent of graphics and simulation backends.
-   **Pyro Successor**: A flexible, port-based foundation for the [pyro](https://github.com/SherbyRobotics/pyro) robotics toolbox.

## 🚀 Quick Start

```python
import numpy as np
from minilink import DiagramSystem, Integrator, Step, Simulator

# 1. Assemble a simple integrator system
diagram = DiagramSystem()
diagram.add_subsystem(Integrator(), "plant")
diagram.add_subsystem(Step(), "source")

# 2. Wire the diagram
diagram.connect("source", "y", "plant", "u")

# 3. Simulate and analyze
sim = Simulator(diagram)
traj = sim.solve(tf=10)
traj.plot_trajectory()
```

## 🛠 Tech Stack
-   **Core**: NumPy
-   **Simulation**: SciPy (`solve_ivp`)
-   **Visualization**: Matplotlib, Graphviz
-   **Acceleration**: JAX (Optional)

---

## 📖 Documentation Guide

-   **[DESIGN.md](DESIGN.md)**: Deep dive into the architecture, signals, systems, and coding conventions.
-   **[ROADMAP.md](ROADMAP.md)**: Project status, development phases, and the Pyro 2.0 migration plan.

---

## 💡 Examples
-   **Interactive Demos**: See the `examples/` directory for pendulums, controllers, and JAX examples.
-   **Notebooks**: [Colab Tutorial](https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing)
