# minilink

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

`minilink` is a small Python framework for **building and simulating dynamical systems as block diagrams**. You compose systems from interconnected blocks (integrators, controllers, pendulums, signal sources, etc.), wire their input/output ports together in a `DiagramSystem`, and then simulate trajectories, plot signals, and optionally animate the resulting motion.

`minilink` is a **pure Python simulation library** intended for use from scripts, notebooks, and interactive sessions.

## Key Ideas

- **Block-diagram modeling**: Define `System` objects with typed input/output ports and states, then connect them into larger diagrams.
- **Compiled execution plans**: `DiagramSystem.compile()` flattens the connection graph into a highly optimized `f_fast` evaluation plan for efficient ODE integration.
- **Simulation & analysis**: The `Simulator` runs continuous or discrete-time simulations (via NumPy / SciPy) and returns trajectories that can be inspected, plotted, or used to reconstruct internal per-port signals.
- **Visualization & animation**: Built-in plotting helpers (matplotlib) and graphical primitives let systems expose geometry for animations (e.g., pendulum demos).

## Tech Stack

- **Python**, using:
  - **NumPy** for numerical arrays and math
  - **SciPy** (`scipy.integrate.solve_ivp`) for ODE solving
  - **matplotlib** for plotting and animation
  - **Graphviz** for diagram visualization

## Learn More

- **Architecture overview**: see `architecture.md` for a deeper description of the core design and future JAX/XLA backend plans.
- **Coding conventions**: see `CONVENTIONS.md` for naming and API style guidelines.
- **Roadmap / TODOs**: see `todos.md` for planned improvements and known architectural issues.

## Examples

- Colab demo notebook: https://colab.research.google.com/drive/13tnYyZMz4bLFzYLdj88H6cqO6tZg6Xp7?usp=sharing
