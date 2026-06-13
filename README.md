# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Start here: [showcase notebook](examples/notebooks/demo_showcase.ipynb) ·
[Colab demo](https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing) ·
[design contracts](DESIGN.md) · [roadmap](ROADMAP.md)

## Why minilink

Minilink is designed for dynamical-system models where the code reads as close
as possible to textbook math. Systems are regular Python objects, equations are
written with NumPy-style array operations, and diagrams compose plants,
controllers, sources, and analysis blocks without a GUI or code generation step.

The core idea is simple: everything is a `System`. A plant is a system, a
controller is a system, a source block is a system, and a full diagram is also a
system that can be simulated, plotted, compiled, or embedded in a larger model.

Systems represent equations and interfaces. They do not hide the evolving
simulation state internally; state trajectories live in simulation results.

## Quick start

```python
from minilink.control.linear import PDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

controller = PDController()
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

## Features

### Models that read like the textbook

Custom dynamics subclass `DynamicSystem` and implement `f(x, u, t, params)`;
equation code stays close to forms like `dx = A @ x + B @ u`. Use object
defaults only when `params is None`.

```python
import numpy as np

from minilink.blocks.sources import Step
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

### Diagrams from operators

Diagrams flatten subsystem states and port connections into one system-level
interface, so a diagram can be used anywhere a system can — including inside
another diagram.

| Shortcut | Meaning |
| --- | --- |
| `a + b + c` | Add subsystems without wiring |
| `source >> plant` | Chain output to input |
| `controller @ plant` | Build a simple feedback diagram |
| `.autowire(strict=True)` | Connect matching named ports |

Explicit wiring (`add_subsystem` / `connect`) is always available when the
shortcuts are too implicit; see
`examples/scripts/diagrams/demo_diagram_shortcuts.py` for both versions side by
side. Any internal signal can be plotted by `"subsystem_id:port_id"` name.

### One call to simulate, plot, animate

Facade methods cover the common workflows: `compute_trajectory(...)` simulates
with SciPy or fixed-step solvers and returns a `Trajectory`;
`plot_trajectory(...)` stacks labeled, unit-aware signal plots (matplotlib or
plotly); `plot_phase_plane(...)` draws vector fields with overlaid
trajectories; `plot_diagram()` renders the wiring topology (Graphviz/Mermaid).

`animate()` plays a trajectory through swappable renderers: matplotlib
(inline HTML in notebooks), plotly, meshcat (3D in the browser), or pygame.
`game()` runs the model interactively with keyboard input — useful for
building intuition about a plant before designing a controller.

### Compiled execution and JAX

Wired diagrams compile into a flat execution plan. The NumPy backend removes
the recursive port-resolution overhead; the JAX backend JIT-compiles dynamics
and outputs, and keeps them traceable for autodiff:

```python
evaluator = diagram.compile(backend="jax")
dx = evaluator.f(x, u, 0.0)              # JIT-compiled flat dynamics

import jax
A = jax.jacfwd(lambda x: evaluator.f(x, u, 0.0))(x)   # exact linearization
```

### Analyze and design

Characterize a plant and design a controller from the same `System`. `analysis`
verbs return data or an `LTISystem`; `control` design factories return ready-to-wire
blocks:

```python
import numpy as np

from minilink.analysis.linearize import linearize
from minilink.control.lqr import lqr
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

plant = InvertedPendulum()
lti = linearize(plant, x_bar=[0.0, 0.0])             # → LTISystem at upright
controller = lqr(lti.A(), lti.B(), Q=np.diag([10.0, 1.0]), R=[[1.0]])

diagram = DiagramSystem()                             # full-state feedback
diagram.add_subsystem(controller, "lqr")
diagram.add_subsystem(plant, "plant")
diagram.connect("plant", "x", "lqr", "x")
diagram.connect("lqr", "u", "plant", "u")

plant.x0 = np.array([0.4, 0.0])
diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
```

### Trajectory optimization

Planning problems combine a system, boundary conditions, and a cost; pluggable
transcriptions (direct collocation, single/multiple shooting) turn them into
nonlinear programs solved by SciPy or Ipopt, optionally with JAX-exact
gradients:

```python
import numpy as np

from minilink.core.costs import QuadraticCost
from minilink.dynamics.catalog.pendulum.cartpole import CartPole
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

sys = CartPole()
sys.inputs["u"].lower_bound[0] = -10.0
sys.inputs["u"].upper_bound[0] = 10.0

x_goal = np.array([0.0, np.pi, 0.0, 0.0])
problem = PlanningProblem(
    sys=sys,
    x_start=np.array([-2.0, 1.0, 0.0, 0.0]),
    x_goal=x_goal,
    cost=QuadraticCost.from_system(sys, Q=np.diag([1.0, 1.0, 0.0, 0.0]), xbar=x_goal),
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(tf=5.0, n_steps=50)
    ),
)
traj = planner.compute_solution()
sys.animate(traj)
```

### Plant catalog

Ready-to-use models under `minilink.dynamics.catalog.*`, each with parameters,
labeled ports, and animation geometry:

| Domain | Models |
| --- | --- |
| `pendulum` | `Pendulum`, `DoublePendulum`, `Acrobot`, `CartPole`, rotating cart-poles |
| `manipulators` | one- to five-link arms, planar and 3D |
| `vehicles` | kinematic and dynamic bicycle models, longitudinal propulsion, suspension |
| `aerial` | planar drones, plane, rocket |
| `marine` | planar boat, boat in current |
| `mass_spring_damper` | one- to three-mass chains, floating variants |
| `equations` | integrator chains, Van der Pol oscillator |

### Symbolic mechanics (experimental)

`minilink.symbolic.mechanics` derives equations of motion symbolically (SymPy,
Lagrange or Kane) from a DH-chain description and exports the result as a
regular minilink mechanical system — including a JAX-traceable variant.

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

Alternatively, a plain editable install works in any Python 3.10+ environment
(`pip install -e ".[dev]"`, with extras `.[jax]`, `.[symbolic]`,
`.[visualization]`, `.[plotting]`, `.[ipopt]`).

Graphviz is used by `plot_diagram()` for diagram topology rendering; it is not
required for writing model equations.

## Call chains

Minimal paths for debugging and extending workflows. Contracts:
[DESIGN.md](DESIGN.md).

Facade methods for common workflows: `compute_trajectory(...)`, `plot_trajectory(...)`,
`plot_diagram(...)`, `animate(...)`. Use lower-level APIs when you need explicit
control: `DiagramSystem.add_subsystem(...)` / `connect(...)`, `Simulator`, or
`compile()` / `DynamicsEvaluator`.

### Package roles

| Package | Owns |
| --- | --- |
| `core` | `System`, `SystemFacades`, `DiagramSystem`, ports, `Trajectory`, sets, costs |
| `blocks` | generic wiring blocks (sources, `Integrator`, `TransferFunction`, routing, nonlinear, filters) |
| `control` | control laws and design factories (`PID`, `ProportionalController`, `LinearFeedbackController`, `lqr`) |
| `analysis` | characterization verbs (`linearize` → `LTISystem`, controllability/observability, equilibria) |
| `core/compile` | `ExecutionPlan`, `DynamicsEvaluator` |
| `simulation` | `Simulator`, solvers, time grids |
| `graphical` | plots, diagrams, animation (`Animator` + renderers) |
| `planning` | `PlanningProblem`, planners, transcriptions |
| `optimization` | `MathematicalProgram`, `Optimizer` |

### Main chains

```text
Model:     subclass System → f/h (+ ports or DynamicSystem options)

Compose:   + / >> / @ / autowire  →  DiagramSystem
           or add_subsystem + connect (+ connect_new_output_port)

Simulate:  compute_trajectory*  →  Simulator  →  compile  →  solve  →  Trajectory

Compile:   sys.compile(backend)  →  DynamicsEvaluator

Plot:      plot_trajectory*  →  graphical.signals  →  PlotResult
           plot_phase_plane* →  graphical.phase_plane
           plot_diagram      →  graphical.diagrams (Graphviz/Mermaid)

Animate:   animate* / render / game  →  Animator  →  renderer backend
           planner.plot_solution / animate_solution  →  problem.sys.*

Trajopt:   PlanningProblem + Transcription + TrajectoryOptimizationPlanner
           → transcribe → MathematicalProgram → Optimizer → Trajectory

NLP:       MathematicalProgram → Optimizer → OptimizationResult
```

- `Trajectory` is numeric only (`t`, `x`, `u`, optional `signals`); labels stay on `System`.
- Diagram internal signals in plots: `"subsystem_id:port_id"`.
- `DiagramSystem.connection_verbose` defaults to `True`; set `False` to quiet wiring.
- Shortcuts flatten diagram operands instead of nesting them; `+` does not infer cross-wiring.
- `compute_*` returns `Trajectory`; `plot_*` returns `PlotResult`; `show=False` skips display.

## Examples

| Interest | Start here |
| --- | --- |
| Feature tour | [examples/notebooks/demo_showcase.ipynb](examples/notebooks/demo_showcase.ipynb) |
| Diagrams | `examples/scripts/diagrams/` |
| Plotting | `examples/scripts/plots/` |
| Animation | `examples/scripts/animation/` |
| Optimization | `examples/scripts/optimization/` |
| Trajectory optimization | `examples/scripts/trajectory_optimization/` |
| Symbolic mechanics | `examples/scripts/symbolic/` |
| Physics engine | `examples/scripts/engine/` |

Catalog plants live under `minilink.dynamics.catalog.*`.

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [ROADMAP.md](ROADMAP.md) — maturity and priorities
- [agent.md](agent.md) — maintainer / agent rules

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` means object defaults, never `params or self.params`.
