# minilink

`minilink` is a Python-native block-diagram framework for modeling,
compiling, simulating, optimizing, and visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab demo: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## What It Is

- **Readable dynamics**: write `f(x, u, t, params)` and `h(x, u, t, params)` like
  textbook state-space equations.
- **Named-port diagrams**: compose MIMO blocks with explicit input and output ports.
- **One object, many workflows**: every `System` and `DiagramSystem` exposes
  simulate, plot, diagram, and animate methods on the same object.
- **Compiled evaluators**: compile leaves and diagrams to flat NumPy or JAX
  `DynamicsEvaluator` objects through `ExecutionPlan`.
- **Shared trajectories**: simulation, planning, plotting, and animation all use
  `Trajectory(t, x, u, signals)`.
- **Optimization and planning**: describe NLPs with `MathematicalProgram`; trajectory
  optimization transcribes `PlanningProblem` objects into that same layer.

## Quick Start

The usual path is: **compose a diagram → simulate → plot or animate**. You do
not need to call `compile()` or construct a `Simulator` for everyday work.

```python
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step

diagram = Step() >> Integrator()
traj = diagram.compute_trajectory(tf=10.0)
diagram.plot_trajectory(traj, signals=("x", "u"))
diagram.plot_diagram()
```

Closed-loop control uses the `@` operator with the standard port names
`r` (reference), `y` (measurement), and `u` (control):

```python
from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

closed = PendulumPDController() @ Pendulum()
traj = closed.compute_trajectory(tf=10.0)
closed.animate(traj)
```

Run examples from the repo root:

```bash
python examples/scripts/diagrams/demo_diagram_shortcuts.py
python examples/scripts/plots/demo_compute_and_plot_options.py
```

## Typical Workflows

### 1. Simulate and visualize (high level)

Every `System` and `DiagramSystem` shares the same facade methods:

| Goal | Call |
| --- | --- |
| Time-domain rollout | `compute_trajectory(tf=..., dt=...)` |
| Prescribed input | `compute_forced(u, input_port_id=...)` |
| Time signals | `plot_trajectory(traj, signals=("x", "u"))` |
| Phase plane | `plot_phase_plane(traj, x_axis=0, y_axis=1)` |
| Block diagram | `plot_diagram()` |
| Motion playback | `animate(traj, renderer="matplotlib")` |

`compute_trajectory` stores the result on `sys.traj`, so later calls can omit
the trajectory argument:

```python
sys.compute_trajectory(tf=6.0)
sys.plot_trajectory(signals=("x",))   # uses sys.traj
sys.animate()                         # uses sys.traj
```

Plotting backends: `backend="matplotlib"` (default) or `backend="plotly"`
(requires the `plotting` extra). Animation renderers include `matplotlib`,
`plotly`, `meshcat`, and `pygame` (see [Install](#install)).

### 2. Compose diagrams (shortcuts)

Shortcuts build ordinary `DiagramSystem` objects. Use them when the topology is
a simple series, parallel add, or standard feedback loop.

```python
# Parallel subsystems (no wiring inferred)
diagram = Step() + PropController() + Integrator()

# Series: default output "y" -> default input "u" or "r"
chain = Step() >> Integrator() >> Integrator()

# Closed loop: controller @ plant
closed = PropController() @ Integrator()

# Conservative autowire for unconnected ports with one safe match
auto = (Step() + PropController() + Integrator()).autowire(strict=True)
```

Import shortcuts from their defining modules:

```python
from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.blocks.sources import Step, WhiteNoise
```

When shortcuts are not enough (nested diagrams, nonstandard port names, multiple
outputs), use the explicit diagram API described in
[Lower-level diagram wiring](#lower-level-diagram-wiring).

### 3. Use a catalog plant

Ready-made plants live under `minilink.dynamics.catalog`:

```python
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

sys = Pendulum()
sys.params["l"] = 2.0
sys.x0 = [0.8, 0.0]
traj = sys.compute_trajectory(tf=6.0)
sys.plot_phase_plane(traj)
```

Other catalog entries include `CartPole`, `DoublePendulum`, and vehicle models
such as `DynamicBicycle`. Reusable LTI and mechanics bases live in
`minilink.dynamics.abstraction` (`StateSpaceSystem`, `MechanicalSystem`, …).

### 4. Define a custom block

There are two port styles. Pick the one that matches your block.

**Simple SISO plant or integrator** — opt into textbook ports in the constructor:

```python
from minilink.core.system import DynamicSystem

class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

    def f(self, x, u, t=0.0, params=None):
        return u

    def h(self, x, u, t=0.0, params=None):
        return x
```

**Multi-port controller or sensor block** — declare ports explicitly:

```python
from minilink.core.system import StaticSystem

class PDController(StaticSystem):
    def __init__(self):
        super().__init__()
        self.params = {"kp": 10.0, "kd": 1.0}
        self.add_input_port("r", nominal_value=0.0, labels=["reference"])
        self.add_input_port(
            "y", labels=["theta", "theta_dot"], units=["rad", "rad/s"]
        )
        self.add_output_port("u", function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        r, y = self.get_port_values_from_u(u, "r", "y")
        return [p["kp"] * (r[0] - y[0]) - p["kd"] * y[1]]
```

Control blocks use `r`, `y`, and `u`. Plants typically use `u` and `y`. Extract
named slices inside equation code with `get_port_values_from_u(u, "r", "y")`.

Base `System` and `StaticSystem` start with **no implicit ports**. `DynamicSystem`
creates standard `u` / `y` / optional `x` ports only through constructor options.

### 5. Trajectory optimization (planning layer)

Trajectory optimization is more explicit than simulation: you describe the
**problem**, pick a **transcription**, and run a **planner**. See
`examples/scripts/trajectory_optimization/demo_cartpole_direct_collocation.py`.

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
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

sys = CartPole()
x_goal = np.array([0.0, np.pi, 0.0, 0.0])
cost = QuadraticCost.from_system(
    sys,
    Q=np.diag([1.0, 1.0, 0.0, 0.0]),
    R=np.diag([1.0]),
    S=np.zeros((sys.n, sys.n)),
    xbar=x_goal,
    ubar=np.zeros(sys.m),
)
problem = PlanningProblem(
    sys=sys,
    x_start=np.array([-2.0, 1.0, 0.0, 0.0]),
    x_goal=x_goal,
    cost=cost,
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(tf=5.0, n_steps=50)
    ),
    options=TrajectoryOptimizationOptions(solve_disp=True),
)
traj = planner.compute_solution()
planner.problem.sys.animate(traj)
```

`Planner.plot_solution()` and `Planner.animate_solution()` delegate back to the
problem system's normal plotting and animation facades.

Costs are attached to the **planning problem**, not the plant, so one model can
support many tasks.

### 6. Standalone optimization (lower level)

For finite-dimensional NLPs without a dynamical transcription, build a
`MathematicalProgram` and an `Optimizer` directly. This path is lower level and
currently at early maturity (TRL 1). See
`examples/scripts/optimization/demo_simple_optimization.py`.

```python
from minilink.optimization.mathematical_program import MathematicalProgram
from minilink.optimization.optimizer import Optimizer

prog = MathematicalProgram(n_z=1, J=J, grad_J=grad_J, g=g, jac_g=jac_g)
opt = Optimizer(prog, z0=[2.0], method="scipy_slsqp")
result = opt.solve(disp=True)
```

Method presets: `scipy_slsqp`, `scipy_trust_constr`, `ipopt` (requires the
`ipopt` extra).

## The Easy Interface (System Facades)

High-level methods live on `System` and are inherited by `DiagramSystem`. They
import simulation and graphics lazily and do not change the math contract of
`f`, `h`, or port compute functions.

| Method | Purpose |
| --- | --- |
| `compute_trajectory(...)` | Simulate with nominal inputs; stores `self.traj` |
| `compute_forced(u, ...)` | Simulate with sampled or callable forcing |
| `plot_trajectory(...)` | Time-signal plot (`signals=("x", "u", "block:port", ...)`) |
| `plot_phase_plane(...)` | 2D vector field with optional trajectory overlay |
| `plot_diagram(...)` | Render block-diagram topology |
| `animate(...)` | Trajectory animation |
| `render(x, u, t, ...)` | Single-frame render |
| `compile(backend=...)` | Build a `DynamicsEvaluator` |
| `+`, `>>`, `@` | Diagram composition shortcuts |

Simulation kwargs shared by `compute_trajectory` and `compute_forced`:

- `t0`, `tf`, `n_steps`, `dt` — time grid
- `x0` — initial state (defaults to `sys.x0`)
- `solver` — preset name such as `"scipy"`, `"scipy_stiff"`, `"euler"`,
  `"rk4_fixedsteps"` (auto-selected when omitted)
- `compile_backend` — `"numpy"` (default on facades), `"jax"`, or `"auto"`
- `verbose` — print setup information

For forced inputs, `u` may be a full `(m, N)` array, a callable `u(t)`, or a
single-port signal when `input_port_id` is set.

## Lower-Level APIs

Use these when you need direct control over compilation, integration, or wiring.

### Simulation with `Simulator`

`System.compute_trajectory` is a thin wrapper around
`minilink.simulation.simulator.Simulator`. Construct `Simulator` directly when
you want one object to run multiple rollouts, inspect the solver backend, or
access `last_traj` without mutating `sys.traj`:

```python
from minilink.simulation.simulator import Simulator

sim = Simulator(diagram, tf=10.0, dt=0.01, compile_backend="numpy")
traj = sim.solve()
forced = sim.solve_forced(lambda t: [1.0], input_port_id="u")
```

### Compilation

```python
evaluator = diagram.compile(backend="numpy", verbose=True)
dx = evaluator.f(x, u, t=0.0)
```

For diagrams, `DiagramSystem.compile` also accepts `bind_params=False`. Diagram
parametric evaluators are still maturing; recompile when JAX diagram parameters
change unless you bind params at compile time.

Free function: `minilink.compile.compiler.compile(system, backend=...)`.

### Lower-level diagram wiring

The explicit API is the canonical interface for general topologies:

```python
from minilink.core.diagram import DiagramSystem

diagram = DiagramSystem()
diagram.connection_verbose = False   # quiet wiring (default is True)
diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(plant, "pendulum")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "controller", "r")
diagram.connect("step", "y", "controller", "r")
diagram.connect("controller", "u", "plant", "u")
diagram.connect("pendulum", "y", "controller", "y")
diagram.connect_new_output_port("pendulum", "y", "y")
```

Shortcut limitations (by design):

- `+` adds subsystems only; it does not merge two existing diagrams.
- `>>` does not connect into an existing diagram operand.
- Nested `DiagramSystem` operands are not supported by shortcuts.

### Internal diagram signals in plots

On a diagram, plot boundary signals with `"x"` and `"u"`, or internal subsystem
outputs with `"subsystem_id:port_id"`:

```python
diagram.plot_trajectory(traj, signals=("pendulum:y", "controller:u"))
```

## Imports

`minilink/__init__.py` is currently a namespace marker (no re-exports). Import
from the module that defines each symbol:

| Need | Import from |
| --- | --- |
| Core types | `minilink.core.system`, `minilink.core.diagram`, `minilink.core.trajectory` |
| Lightweight blocks | `minilink.core.blocks.basic`, `minilink.core.blocks.sources` |
| Catalog plants | `minilink.dynamics.catalog.<domain>.<module>` |
| Simulation | `minilink.simulation.simulator` |
| Compile | `minilink.compile.compiler` |
| Planning / trajopt | `minilink.planning.problems`, `minilink.planning.trajectory_optimization.*` |
| Optimization | `minilink.optimization.mathematical_program`, `minilink.optimization.optimizer` |
| Plotting helpers | `minilink.graphical.signals`, `minilink.graphical.phase_plane` |

Curated top-level exports are planned ([ROADMAP.md](ROADMAP.md) P1).

## Install

Core install:

```bash
pip install minilink
```

Optional extras:

```bash
pip install "minilink[jax]"            # JAX evaluators, JIT, autodiff
pip install "minilink[visualization]"  # meshcat / pygame renderers
pip install "minilink[plotting]"       # Plotly signal plots and notebook renderer
pip install "minilink[symbolic]"       # SymPy mechanics helpers
pip install "minilink[ipopt]"          # Ipopt optimizer adapter
```

Development install from a clone:

```bash
pip install -e ".[dev]"
# optional: pip install -e ".[dev,jax,plotting,visualization,symbolic,ipopt]"
pytest
```

System package `graphviz` must be available for diagram rendering
(`apt install graphviz` on Debian/Ubuntu).

## Design Rules (Short)

Details and edge cases are in [DESIGN.md](DESIGN.md).

- **NumPy baseline, JAX optional**: choose backends explicitly (`compile_backend`,
  evaluator backends); vocabulary in `minilink.compile.backend_policy`.
- **Native-array equation paths** (`f`, `h`, ports, sets, costs, programs):
  preserve the active backend; convert to NumPy/Python only at boundaries
  (evaluators, solvers, plotting, …).
- **Prefer one readable class** for simple algebra; optional `Jax<X>` twins only
  when one implementation cannot stay traceable for both backends.
- **`params is None`** uses object defaults; any other `params` value overrides—
  never treat empty dicts as falsy for “use defaults.”

## Main Packages

| Package | Role |
| --- | --- |
| `minilink.core` | `System`, `DiagramSystem`, ports, `Trajectory`, sets, costs, blocks |
| `minilink.compile` | `ExecutionPlan`, backend policy, NumPy/JAX evaluators |
| `minilink.simulation` | `Simulator`, solver backends, forced-input utilities |
| `minilink.optimization` | `MathematicalProgram`, `Optimizer`, program evaluators |
| `minilink.planning` | `PlanningProblem`, search, policy synthesis, trajectory optimization |
| `minilink.dynamics` | Plant abstractions and catalog models |
| `minilink.graphical` | Time signals, phase plane, diagrams, animation |
| `minilink.control` | Example controllers (e.g. `PendulumPDController`) |
| `minilink.symbolic` | Optional SymPy mechanics derivation |
| `minilink.physics` | JAX physics engine MVPs |

## Documentation

| Document | Contents |
| --- | --- |
| [DESIGN.md](DESIGN.md) | Architecture, object contracts, backend rules |
| [flows.md](flows.md) | Call-path map from user methods to implementation |
| [ROADMAP.md](ROADMAP.md) | Maturity matrix and active priorities |
| [agent.md](agent.md) | Maintainer and AI-agent contribution rules |

## Examples And Benchmarks

Runnable demos live in `examples/scripts/`:

| Folder | Topics |
| --- | --- |
| `diagrams/` | Shortcuts, closed loop, compiling, cascade control |
| `plots/` | Signal backends, phase plane, compute/plot options |
| `animation/` | Renderers, camera, interactive pygame |
| `trajectory_optimization/` | Direct collocation, vehicle examples, JAX/Ipopt |
| `optimization/` | Standalone NLP demo |
| `physics/` | JAX sphere-plane contact |
| `symbolic/` | SymPy multibody export |

```bash
python examples/scripts/diagrams/demo_diagram_shortcuts.py
python examples/scripts/plots/demo_graphical_backends.py
```

Headless environments: set `MPLBACKEND=Agg` before importing graphical modules,
or use `plot_trajectory(..., show=False)` and `plot_diagram(show_inline=False,
show_pdf=False)`.

- `tests/manual/` — one-off smoke tests and exploratory checks.
- `tests/benchmark/` — benchmark runners using subsystem-local helpers such as
  `minilink.simulation.benchmark` and `minilink.optimization.benchmark`.
