# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Start here: [showcase](examples/learn/intro/showcase_minilink.ipynb) ·
[JAX / autodiff showcase](examples/learn/intro/showcase_jax.ipynb) ·
[examples](examples/README.md) ·
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)

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

Optional JAX compile and autodiff through the same dynamics unlocks fast
trajopt, MPC, parameter fits, and learning — without a separate sim vs opt
stack. See the [JAX showcase](examples/learn/intro/showcase_jax.ipynb).

## Quick start

```python
from minilink import ImpedanceController, Pendulum

controller = ImpedanceController()
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

Longer scripts prefer band imports (`from minilink.catalog import …`,
`from minilink.control import …`, `from minilink.analysis import …`). See
[DESIGN.md §2](DESIGN.md#public-imports-teaching-first).

## Features

### Models that read like the textbook

Custom dynamics subclass `DynamicSystem` and implement `f`; equation code stays
close to forms like `dx = A @ x + B @ u`.

```python
import numpy as np
from minilink import DynamicSystem


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, expose_state=True)

    def f(self, x, u, t=0, params=None):

        m = 1.0
        k = 4.0
        c = 0.3

        p = x[0]
        pdot = x[1]
        F = u[0]

        pddot = (F - c * pdot - k * p) / m

        dx = np.array([pdot, pddot])

        return dx
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
`examples/demos/diagrams/diagram_shortcuts.py` for both versions side by
side. Any internal signal can be plotted by `"subsystem_id:port_id"` name.

### One call to simulate, plot, animate

Common facades: `compute_trajectory(...)` → `Trajectory`;
`plot_trajectory(...)` / `plot_phase_plane(...)` / `plot_diagram()`;
`animate()` (matplotlib, plotly, meshcat, pygame); `game()` for live keyboard
sessions. Continuous plants and diagrams use `Simulator`; static leaves use
`StaticSimulator`. Hybrid diagrams render plant + scheduled computer views.

### Compiled execution and JAX

Wired diagrams compile into a flat execution plan (NumPy or JAX). For autodiff
inside an outer `jit`, use the **trace tier** (`f_trace`, `f_trace_p`, …):

```python
import jax
import jax.numpy as jnp

evaluator = diagram.compile(backend="jax")
loss_and_grad = jax.jit(
    jax.value_and_grad(
        lambda theta: jnp.mean(
            (evaluator.f_trace_p(x, u, 0.0, {"plant": theta}) - dx_ref) ** 2
        )
    )
)
```

### Hybrid and discrete control

Discrete laws (MPC, sampled SMC, …) close the loop on continuous plants via
`StepSystem` → `Computer` → `HybridDiagram` (`Computer @ plant` or `mpc @ plant`):

```python
from minilink.control.mpc import ModelPredictiveController

mpc = ModelPredictiveController(planner, dt_mpc=0.1, warm_start=True)
diagram = mpc @ plant
diagram.compute_trajectory(tf=10.0)
```

Deploy ticks: `cmd = mpc.compute_command(y, t=t)`; `u = cmd.u_ff`. NumPy rebuild
mode and demos: `examples/demos/mpc/`.

### Analyze and design

Characterize a plant and design a controller from the same `System`:

```python
import numpy as np
from minilink import InvertedPendulum
from minilink.analysis.linearize import linearize
from minilink.control.lqr import lqr

plant = InvertedPendulum()
lti = linearize(plant, x_bar=[0.0, 0.0])
ctl = lqr(lti.A(), lti.B(), Q=np.diag([10.0, 1.0]), R=[[1.0]])
diagram = ctl @ plant
```

Also: `bode` / `plot_bode`, `modal_analysis`, ctrb/obsv, equilibria.

### Planning, search, and optimization

`PlanningProblem` combines a continuous system, start/goal boundaries, costs,
and spatial geometry. The same problem feeds trajopt, RRT/RRT*, and DP:

```python
import numpy as np
from minilink import CartPole
from minilink.core.costs import QuadraticCost
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

sys = CartPole()
x_goal = np.array([0.0, np.pi, 0.0, 0.0])
problem = PlanningProblem(
    sys=sys,
    x_start=np.array([-2.0, 1.0, 0.0, 0.0]),
    x_goal=x_goal,
    cost=QuadraticCost.from_system(sys, Q=np.diag([1.0, 1.0, 0.0, 0.0]), xbar=x_goal),
    tf=5.0,
)
traj = (
    TrajectoryOptimizationPlanner(
        problem, n_steps=50, transcription="direct_collocation"
    )
    .solve()
    .trajectory
)
```

### Plant catalog

Ready-to-use models via `minilink.catalog` (implementation under
`minilink.dynamics.catalog.*`), each with parameters, labeled ports, and
animation geometry:

| Domain | Models |
| --- | --- |
| `pendulum` | `Pendulum`, `DoublePendulum`, `Acrobot`, `CartPole`, rotating cart-poles |
| `manipulators` | one- to five-link arms, planar and 3D |
| `vehicles` | NumPy bicycles (`dynamic_bicycle`, `steering`); JAX ladder in `jax_vehicles`; `CarProfile`; propulsion, suspension |
| `aerial` | planar drones, plane, rocket |
| `marine` | planar boat, boat in current |
| `mass_spring_damper` | one- to three-mass chains, floating variants |
| `equations` | integrator chains, Van der Pol oscillator |

### Symbolic mechanics (experimental)

`minilink.symbolic.mechanics` derives EoM symbolically (SymPy, Lagrange or Kane)
from a DH-chain and exports a regular minilink mechanical system (including a
JAX-traceable variant).

## Technology

Minilink keeps the user-facing API small, while the execution path supports
larger diagrams and repeated simulation or optimization.

- **System hierarchy**: `System` is the base IO shell (`n` defaults to 0 for static
  blocks). Continuous plants subclass `DynamicSystem` (`f`, `h`). Mechanical
  abstractions, library blocks, controllers, catalog plants, and `DiagramSystem`
  compose on top.
- **Textbook equations**: dynamic models implement `f(x, u, t, params)` on
  `DynamicSystem`, so equation code can stay close to forms like `dx = A @ x + B @ u`.
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
System                    # static IO shell (n defaults to 0)
  -> DynamicSystem        # dx = f(x, u, t)
    -> MechanicalSystem
      -> Pendulum
```

## API stability (v0.1)

Minilink v0.1 freezes a **stable tier** for teaching; everything else is
**provisional**. Stable public names and semantics only change with a
deprecation note in the release notes; provisional APIs may change between
minor releases (maturity detail: [ROADMAP.md](ROADMAP.md)).

| Tier | Bands |
| --- | --- |
| **Stable** | `core/` (`System`, `DynamicSystem`, `StepSystem`, `DiagramSystem`, composition operators, `Trajectory`, compile facade), `simulation` (`Simulator`, `StaticSimulator`), `blocks/`, catalog teaching plants (`minilink.catalog`), basic `control/` (output, state, SISO, impedance, robotic, model-based, LQR), basic `analysis/` (linearize, modal, frequency, equilibria, discretize) |
| **Provisional** | `planning/` (trajopt, RRT, DP, spatial), `control.mpc`, hybrid (`StepDiagramSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`), `simulation.realtime`, `optimization/`, evaluator integration-helper grid beyond the documented subset ([DESIGN.md §5](DESIGN.md#compilation-and-simulation)), `estimation/` / `identification/` / `interfaces/` placeholders, quarantine (`symbolic/`, `dynamics/engines/`) |

## Install

Minilink requires Python 3.10+ (3.13 recommended for conda). Conda is recommended
because diagram rendering and some optimization backends depend on native libraries.

Full dev environment from [environment.yml](environment.yml) (core deps, optional
extras, pytest/ruff/sphinx, Jupyter):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

Minimal manual setup:

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda create -n minilink -c conda-forge python=3.13 numpy scipy matplotlib graphviz python-graphviz
conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

Optional features (included in `environment.yml`; install separately for minimal envs):

```bash
conda install -c conda-forge jax jaxlib meshcat-python pygame plotly sympy ipopt cyipopt
```

Alternatively, a plain editable install works in any Python 3.10+ environment
(`pip install -e ".[dev]"`, with extras `.[jax]`, `.[symbolic]`,
`.[visualization]`, `.[plotting]`, `.[ipopt]`).

Graphviz is used by `plot_diagram()` for diagram topology rendering; it is not
required for writing model equations.

## Testing

Use the **`minilink`** conda env above.
**Entry points:** [tests/README.md#entry-points](tests/README.md#entry-points)
(IDE: [`tests/run/run_contract_tests.py`](tests/run/run_contract_tests.py)).

## Call chains

Minimal paths for debugging and extending workflows. Contracts:
[DESIGN.md](DESIGN.md).

Facade methods for common workflows: `compute_trajectory(...)` (static leaves and
continuous/diagram systems via MRO), `plot_trajectory(...)`,
`plot_diagram(...)`, `animate(...)`. Use lower-level APIs when you need explicit
control: `DiagramSystem.add_subsystem(...)` / `connect(...)`, `Simulator`, or
`compile()` / `DynamicsEvaluator`.

### Package roles

| Package | Owns |
| --- | --- |
| `core` | `System`, façade mixins (`SharedSystemFacades`, `DynamicSystemFacades`, `StepSystemFacades`), `DiagramSystem`, ports, `Trajectory`, sets, costs |
| `blocks` | generic wiring blocks (sources, `Integrator`, `TransferFunction`, routing, nonlinear, filters, neural) |
| `control` | control laws and design factories (`FilteredController`, `ProportionalController`, `StateFeedbackController`, `lqr`, `modelbased`, `robotic`, `mpc`) |
| `analysis` | `linearize`, `structural`, `equilibria`, `modal` (`modal_analysis`, `animate_modal`) |
| `core/compile` | `ExecutionPlan`, `DynamicsEvaluator` |
| `simulation` | `Simulator`, `HybridSimulator`, `Computer`, solvers, time grids |
| `graphical` | plots, diagrams, animation (`Animator` + renderers) |
| `planning` | `PlanningProblem`, planners, transcriptions |
| `optimization` | `MathematicalProgram`, `Optimizer` |

### Main chains

```text
Model:     subclass System → f/h (+ ports or DynamicSystem options)

Compose:   + / >> / @ / autowire  →  DiagramSystem
           hybrid: block % dt  →  Computer; Computer @ plant  →  HybridDiagram
           or add_subsystem + connect (+ connect_new_output_port)

Simulate:  compute_trajectory*  →  StaticSimulator (static leaf) or Simulator (DynamicSystem / diagram)
           →  compile  →  solve  →  Trajectory
           StepSystem: compute_rollout  →  StepEvaluator.rollout (state-only k/x/u)
           StepDiagram + schedule: Computer.tick  →  signal histories (not evaluator rollout)
           HybridDiagram: compute_forced  →  HybridSimulator  →  HybridSimResult
           cache: self.traj (plant Trajectory), self.last_result (full result), self.rollout (computer)

Compile:   sys.compile(backend)  →  DynamicsEvaluator

Plot:      plot_trajectory*  →  graphical.signals  →  PlotResult
           plot_phase_plane* →  graphical.phase_plane
           plot_diagram      →  graphical.diagrams (DiagramSystem / StepDiagramSystem)
           HybridDiagram.plot_diagram  →  hybrid composite (Plant + Computer clusters)

Animate:   animate* / render  →  Animator  →  renderer backend
           game  →  simulation.realtime.RealtimeSimulator  →  live Animator frames
           HybridDiagram.animate  →  plant geometry + fine plant traj
           planner.plot_solution / animate_solution  →  problem.sys.*

Trajopt:   PlanningProblem + TrajectoryOptimizationPlanner
           (flat ``n_steps`` / ``transcription="…"``; optional Transcription)
           → transcribe → MathematicalProgram → Optimizer → TrajectoryPlan

NLP:       MathematicalProgram → Optimizer → OptimizationResult
```

- `Trajectory` is numeric only (`t`, `x`, `u`, optional `signals`); labels stay on `System`.
- Diagram internal signals in plots: `"sys_id:port_id"`, or ``(subsystem, "port")``
  tuples; shortcut-built diagrams default to ``ref`` / ``ctl`` / ``sys``.
- `DiagramSystem.connection_verbose` defaults to `False`; set `True` to print one line per connection.
- Shortcuts flatten diagram operands instead of nesting them; `+` does not infer cross-wiring.
- `compute_*` returns `Trajectory`; `plot_*` returns `PlotResult`; `show=False` skips display.

## Examples

Index and placement rules: [examples/README.md](examples/README.md)
(`scripts/` = canonical · `experimental/` = non-core / scenario WIP ·
`projects/` = multi-file).

| Interest | Start here |
| --- | --- |
| Feature tour (marketing) | [examples/learn/intro/showcase_minilink.ipynb](examples/learn/intro/showcase_minilink.ipynb) |
| Stateless / JAX / autodiff (marketing) | [examples/learn/intro/showcase_jax.ipynb](examples/learn/intro/showcase_jax.ipynb) |
| Module API intros | [examples/learn/intro/](examples/learn/intro/) (`00_core` … `10_graphical`) |
| Compile → evaluator API | [examples/learn/intro/07_compile.ipynb](examples/learn/intro/07_compile.ipynb) |
| Diagrams | `examples/demos/diagrams/` · [intro/core](examples/learn/intro/00_core.ipynb) |
| Blocks (routing, filters, nonlinear) | `examples/demos/blocks/` · [intro/blocks](examples/learn/intro/01_blocks.ipynb) |
| Control | `examples/demos/control/` · [intro/control](examples/learn/intro/03_control.ipynb) |
| Pyro SMC continuous (pendulum) | `examples/demos/control/sliding_mode_pendulum.py` |
| Hybrid / step (multi-rate, SMC compare, `Computer`) | `examples/demos/hybrid/` · `examples/demos/step/` · [intro/hybrid](examples/learn/intro/06_hybrid.ipynb) |
| MPC (minimal + dual-rate) | `examples/demos/mpc/` · [teaching/topics/mpc](examples/learn/teaching/topics/mpc.ipynb) · dual-rate: `examples/projects/mpc/mpc_dual_rate.py` |
| MPC scenarios (path / slalom / spatial) | `examples/projects/mpc/` · circuit: `examples/demos/mpc/mpc_car_circuit.py` |
| Robotic (impedance, computed torque, kinematic/nullspace, IK) | `examples/demos/robotic/` |
| Analysis (linearize, trim, ctrb/obsv, modal) | `examples/demos/analysis/` · [intro/analysis](examples/learn/intro/04_analysis.ipynb) |
| State-space / LQR | `examples/demos/statespace/` |
| Identification (param gradients) | `examples/demos/identification/` |
| Plotting | `examples/demos/plots/` · [intro/graphical](examples/learn/intro/10_graphical.ipynb) |
| Animation | `examples/demos/animation/` · [intro/graphical](examples/learn/intro/10_graphical.ipynb) |
| Realtime game mode (keyboard → live plant → `Trajectory`) | `examples/demos/realtime/game_cartpole.py` |
| Optimization | `examples/demos/optimization/` · [intro/optimization](examples/learn/intro/08_optimization.ipynb) |
| Planning (RRT, DP) | `examples/demos/planning/` · [intro/planning](examples/learn/intro/09_planning.ipynb) |
| Trajectory optimization | `examples/demos/trajopt/` · [car TrajOpt project](examples/projects/car_trajopt/) · [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/projects/car_trajopt/car_trajopt.ipynb) |
| Path tracking projects | `examples/projects/pathtracking/` · `examples/projects/car_trajopt/` |
| C export (P controller round-trip; filtered PID leaf) | `examples/demos/interfaces/c_export_proportional.py` · `c_export.py` |
| Solver benchmarks | [examples/tooling/notebooks/benchmark.ipynb](examples/tooling/notebooks/benchmark.ipynb) (uses repo-root `benchmarks/`) |

Catalog plants: `from minilink.catalog import …` (math under `minilink.dynamics.catalog.*`).

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [ROADMAP.md](ROADMAP.md) — TRL maturity and teaching-release priorities
- [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) — pyro 2.0 parity audit (library + all 195 demos)
- [docs/plans/](docs/plans/) — active design backlog
- [AGENTS.md](AGENTS.md) — contributor / agent rules
- API reference (Sphinx): [alx87grd.github.io/minilink](https://alx87grd.github.io/minilink/) (built from `main` via [.github/workflows/docs.yml](.github/workflows/docs.yml)); local build: `pip install -e ".[docs]" && sphinx-build -b html docs docs/_build/html`

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` means object defaults, never `params or self.params`. Coding
style: [AGENTS.md](AGENTS.md).
