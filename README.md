# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab: [demo notebook](https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing)

## Goal

Minilink is designed for dynamical-system models where the code reads as close
as possible to textbook math. Systems are regular Python objects, equations are
written with NumPy-style array operations, and diagrams compose plants,
controllers, sources, and analysis blocks without a GUI or code generation step.

The core idea is simple: everything is a `System`. A plant is a system, a
controller is a system, a source block is a system, and a full diagram is also a
system that can be simulated, plotted, compiled, or embedded in a larger model.

Systems represent equations and interfaces. They do not hide the evolving
simulation state internally; state trajectories live in simulation results.

## Install

Minilink requires Python 3.10+. Conda is recommended because diagram rendering
and some optimization backends depend on native libraries.

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda create -n minilink -c conda-forge python=3.10 numpy scipy matplotlib graphviz python-graphviz
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

Optional features:

```bash
conda install -c conda-forge jax jaxlib meshcat-python pygame plotly sympy ipopt cyipopt
```

Graphviz is used by `plot_diagram()` for diagram topology rendering; it is not
required for writing model equations.

## Technology

Minilink keeps the user-facing API small, while the execution path supports
larger diagrams and repeated simulation or optimization.

- **System hierarchy**: `System` is the mother class. Common subclasses include
  `DynamicSystem`, `StaticSystem`, mechanical abstractions, source blocks,
  controllers, catalog plants, and `DiagramSystem`.
- **Textbook equations**: dynamic models implement `f(x, u, t, params)`, so
  equation code can stay close to forms like `dx = A @ x + B @ u`.
- **Stateless model objects**: a system defines equations, ports, parameters,
  and initial conditions. The evolving state belongs to the simulator and
  returned trajectory, not to hidden mutable block state.
- **Composable diagrams**: diagrams flatten subsystem states and port
  connections into one system-level interface. A diagram can be used anywhere a
  system can.
- **Compiled execution**: wired diagrams are converted into a flat execution
  plan. With the JAX backend, diagram dynamics and outputs can be JIT-compiled
  for fast simulation and optimization workflows.
- **Layered dependencies**: NumPy is the baseline, SciPy handles common ODE
  solvers, Matplotlib handles signal plots, Graphviz handles topology diagrams,
  and optional layers add JAX, symbolic mechanics, animation, and Ipopt.

For example, the class hierarchy can go from a generic system contract to a
domain-specific model:

```text
System
  -> DynamicSystem
    -> MechanicalSystem
      -> Pendulum
```

## Quick start

```python
from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

controller = PendulumPDController()
plant = Pendulum()

plant.x0[0] = 2.0
plant.params["l"] = 5.0
plant.params["m"] = 1.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()
```

## Define a system

Custom dynamics subclass `DynamicSystem` and implement `f(x, u, t, params)`.
Use object defaults only when `params is None`.

```python
import numpy as np

from minilink.core.blocks.sources import Step
from minilink.core.system import DynamicSystem


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, expose_state=True)
        self.params = {"m": 1.0, "k": 4.0, "c": 0.3}

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        m, k, c = params["m"], params["k"], params["c"]

        position, velocity = x
        force = u[0]

        dx = np.zeros(2)
        dx[0] = velocity
        dx[1] = (force - c * velocity - k * position) / m
        return dx


sys = MassSpringDamper()
sys.x0[0] = 1.0

step = Step(final_value=np.array([10.0]), step_time=2.0)
diagram = step >> sys

diagram.compute_trajectory(tf=20.0)
diagram.plot_trajectory(signals=("x", "step:y"))
```

## Diagram shortcuts

| Shortcut | Meaning |
| --- | --- |
| `a + b + c` | Add subsystems without wiring |
| `source >> plant` | Chain output to input |
| `controller @ plant` | Build a simple feedback diagram |
| `.autowire(strict=True)` | Connect matching named ports |

See `examples/scripts/diagrams/demo_diagram_shortcuts.py` for shortcut and
explicit versions side by side.

## Lower-level APIs

Use the facade methods for common workflows:

- `compute_trajectory(...)`
- `plot_trajectory(...)`
- `plot_diagram(...)`
- `animate(...)`

Use lower-level APIs when you need explicit control:

- `DiagramSystem.add_subsystem(...)` and `DiagramSystem.connect(...)` for custom
  topology;
- `Simulator` for solver, time-grid, and backend control;
- `compile()` / `DynamicsEvaluator` for repeated evaluation and JAX execution.

## Examples

| Interest | Start here |
| --- | --- |
| Diagrams | `examples/scripts/diagrams/` |
| Plotting | `examples/scripts/plots/` |
| Animation | `examples/scripts/animation/` |
| Optimization | `examples/scripts/optimization/` |
| Trajectory optimization | `examples/scripts/trajectory_optimization/` |
| Symbolic mechanics | `examples/scripts/symbolic/` |
| Physics | `examples/scripts/physics/` |

Catalog plants live under `minilink.dynamics.catalog.*`.

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [flows.md](flows.md) — minimal call chains
- [ROADMAP.md](ROADMAP.md) — maturity and priorities
- [agent.md](agent.md) — maintainer / agent rules

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` means object defaults, never `params or self.params`.
