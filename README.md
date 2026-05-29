# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab: [https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing](https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing)

## Tech Stack

- **Core**: NumPy
- **Simulation**: `minilink.simulation` with SciPy or custom jit-accelerated backends
- **Compilation**: `DynamicsEvaluator` on NumPy or JAX
- **Visualization**: Matplotlib, MeshCat, Pygame, Graphviz
- **Acceleration**: optional JAX JIT and autodiff

## Quick start

```python
from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

controller = PendulumPDController()  # u = Kp * (r - theta) - Kd * theta_dot
plant = Pendulum()  # theta_ddot = -(g / l) * sin(theta) + tau / (m * l**2)

plant.x0[0] = 2.0
plant.params["l"] = 5.0
plant.params["m"] = 1.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()
```

```python

import numpy as np
from minilink.core.system import DynamicSystem
from minilink.core.blocks.sources import Step

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

# Standalone system
sys = MassSpringDamper()
sys.x0[0] = 1.0  # released from rest at position 1
sys.plot_diagram()
sys.compute_trajectory(tf=20.0)
sys.plot_trajectory()

# Source
step = Step()
step.params["final_value"] = np.array([10.0])
step.params["step_time"] = 2.0

# Diagram source + system
diagram = step >> sys
diagram.plot_diagram()
diagram.compute_trajectory(tf=20.0)
diagram.plot_trajectory(signals=("x", "step:y"))
```

Catalog of plants: `minilink.dynamics.catalog.*`. 
Demos: `examples/scripts/`.

Lower level API when needed: `Simulator`, `compile()`,
explicit `DiagramSystem.connect` — see [flows.md](flows.md).

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [flows.md](flows.md) — minimal call chains
- [ROADMAP.md](ROADMAP.md) — maturity and priorities
- [agent.md](agent.md) — maintainer / agent rules

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` → defaults (never `params or self.params`).
