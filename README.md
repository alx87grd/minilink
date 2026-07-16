# minilink

Python-native block-diagram framework for modeling, simulating, optimizing, and
visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Start here: [showcase notebook](examples/notebooks/demo_showcase.ipynb) ·
[compile → evaluator intro](examples/notebooks/demo_compile_evaluator.ipynb) ·
[notebooks folder](examples/notebooks/README.md) ·
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/tree/main/examples/notebooks)

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
from minilink.control.impedance import ImpedanceController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

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
diagram.plot_trajectory(signals=("x", "ref:y"))
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

Facade methods cover the common workflows: `compute_trajectory(...)` samples or
integrates on a time grid — static leaves via `StaticSimulator` (boundary IO in
`traj.signals`); continuous plants and diagrams via `Simulator` (SciPy or
fixed-step solvers) — and returns a `Trajectory`;
`plot_trajectory(...)` stacks labeled, unit-aware signal plots (matplotlib or
plotly); `plot_phase_plane(...)` draws vector fields with overlaid
trajectories; `plot_diagram()` renders wiring topology (Graphviz/Mermaid).
``HybridDiagram.plot_diagram()`` renders the composite plant + scheduled computer view.

`animate()` plays a trajectory through swappable renderers: matplotlib
(inline HTML in notebooks), plotly, meshcat (3D in the browser), or pygame.
`game()` runs the model interactively with keyboard input — useful for
building intuition about a plant before designing a controller.

### Compiled execution and JAX

Wired diagrams compile into a flat execution plan. The NumPy backend removes
the recursive port-resolution overhead; the JAX backend JIT-compiles dynamics
and outputs for fast simulation (`evaluator.f`, `evaluator.rk4_step`, …).
Integration rollouts use explicit names — e.g. `rk4_integrate_zoh`,
`rk4_integrate_linear`, `euler_integrate_zoh` (replacing the old bare `integrate`
and `rk4_integrate_forced`). Fixed-step `Simulator` presets: `euler` follows the
simulation time grid knot-by-knot; `euler_fixedsteps` uses uniform spacing via
compiled Euler rollouts.
For autodiff inside an outer `jit`, use the **trace tier** (pre-JIT siblings:
`f_trace`, `f_trace_p`, `rk4_step_trace`, …):

```python
evaluator = diagram.compile(backend="jax")
dx = evaluator.f(x, u, 0.0)              # fast tier (JIT)

import jax
loss_and_grad = jax.jit(jax.value_and_grad(
    lambda theta: jnp.mean((evaluator.f_trace_p(x, u, 0.0, {"plant": theta}) - dx_ref) ** 2)
))
```

### Hybrid and discrete control

Discrete control laws (like digital MPC or sampled Sliding Mode Control) can close the loop on continuous plants without breaking the continuous-time core or solver guarantees. `StepSystem` defines discrete logic, and `Computer` schedules it. The `%` and `@` operators build a `HybridDiagram` with Zero-Order Hold (ZOH) and sampling:

```python
from minilink.control.mpc import ModelPredictiveController

# controller is a discrete leaf; plant is a continuous DynamicSystem
mpc = ModelPredictiveController(planner, dt_mpc=0.1, warm_start=True)
diagram = mpc @ plant  # schedule + hybrid ZOH/sample wiring

diagram.compute_trajectory(tf=10.0)  # solves the plant exactly between ticks
diagram.animate()
```

Hand-loop or external deploy node (ROS-agnostic — no ROS2 package in minilink):

```python
cmd = mpc.compute_command(y, t=t_wall)  # replan tick → Command
u = cmd.u_ff
meta = mpc.get_solve_metadata()         # success / cost / solve_time_s
# or: cmd.metadata  (same SolveMetadata)
mpc.reset()                             # clear deploy counter + latch
```

### Analyze and design

Characterize a plant and design a controller from the same `System`. `analysis`
verbs return raw matrices, data, or an `LTISystem`; `control` design factories
return ready-to-wire blocks. ``modal_analysis`` returns open-loop poles and
mode shapes; the plant facade ``modal_analysis(..., mode=...)`` can also
animate modes:

```python
from minilink.analysis.modal import modal_analysis
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

plant = Pendulum()
poles, modes = modal_analysis(plant, x_bar=[0.0, 0.0])
plant.modal_analysis(x_bar=[0.0, 0.0], mode=0)      # one mode
plant.modal_analysis(x_bar=[0.0, 0.0], mode="all")  # every mode
```

```python
import numpy as np

from minilink.analysis.frequency import bode
from minilink.analysis.linearize import linearize, linearize_matrices
from minilink.control.lqr import lqr
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum

plant = InvertedPendulum()
A, B, C, D = linearize_matrices(plant, x_bar=[0.0, 0.0])  # raw arrays
lti = linearize(plant, x_bar=[0.0, 0.0])             # → LTISystem at upright
w, mag, phase = bode(plant, x_bar=[0.0, 0.0], input_port="u", output_index=0)
plant.plot_bode(x_bar=[0.0, 0.0], input_port="u", output_index=0)
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

### Planning, search, and optimization

`PlanningProblem` combines a continuous system, start/goal boundaries, cost functions, and spatial geometry (`Scene`, `Shape`, `Set`). The same problem definition powers multiple planners:

- **Trajectory Optimization**: Pluggable transcriptions (direct collocation, shooting) turn problems into nonlinear programs solved by SciPy or Ipopt (with exact JAX gradients).
- **Search (RRT / RRT*)**: Kinodynamic and steering extenders grow trees through collision-free state space.
- **Policy Synthesis**: Value iteration / dynamic programming over a `StateSpaceGrid` computes global cost-to-go and discrete optimal policies.

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
    tf=5.0,
)
planner = TrajectoryOptimizationPlanner(
    problem,
    transcription=DirectCollocationTranscription(
        DirectCollocationOptions(n_steps=50)
    ),
)
traj = planner.solve().trajectory
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

- **System hierarchy**: `System` is the base IO shell (`n` defaults to 0 for static
  blocks). Continuous plants subclass `DynamicSystem` (`f`, `h`). Mechanical
  abstractions, source blocks, controllers, catalog plants, and `DiagramSystem`
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

Use the **`minilink`** conda env above for local development and agent verification.
From repo root:

```bash
conda activate minilink
python -m pytest
```

For the full suite including optional backends and headless pygame smoke tests,
see [tests/README.md](tests/README.md).

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
| `control` | control laws and design factories (`FilteredController`, `ProportionalController`, `StateFeedbackController`, `lqr`, `modelbased`, `robotic`) |
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

Animate:   animate* / render / game  →  Animator  →  renderer backend
           HybridDiagram.animate  →  plant geometry + fine plant traj
           planner.plot_solution / animate_solution  →  problem.sys.*

Trajopt:   PlanningProblem + Transcription + TrajectoryOptimizationPlanner
           → transcribe → MathematicalProgram → Optimizer → Trajectory

NLP:       MathematicalProgram → Optimizer → OptimizationResult
```

- `Trajectory` is numeric only (`t`, `x`, `u`, optional `signals`); labels stay on `System`.
- Diagram internal signals in plots: `"sys_id:port_id"`, or ``(subsystem, "port")``
  tuples; shortcut-built diagrams default to ``ref`` / ``ctl`` / ``sys``.
- `DiagramSystem.connection_verbose` defaults to `False`; set `True` to print one line per connection.
- Shortcuts flatten diagram operands instead of nesting them; `+` does not infer cross-wiring.
- `compute_*` returns `Trajectory`; `plot_*` returns `PlotResult`; `show=False` skips display.

## Examples

| Interest | Start here |
| --- | --- |
| Feature tour | [examples/notebooks/demo_showcase.ipynb](examples/notebooks/demo_showcase.ipynb) |
| Compile → evaluator → fast dynamics primitives | [examples/notebooks/demo_compile_evaluator.ipynb](examples/notebooks/demo_compile_evaluator.ipynb) |
| Extended tour | [examples/notebooks/demo_overview.ipynb](examples/notebooks/demo_overview.ipynb) |
| Diagrams | `examples/scripts/diagrams/` |
| Step (discrete leaf, `compute_rollout`) | `examples/scripts/step/` |
| Hybrid (scheduled computer + continuous plant) | `examples/scripts/hybrid/demo_hybrid_multi_rate.py` |
| Minimal hybrid MPC (`ModelPredictiveController` + `mpc @ plant`) | `examples/scripts/hybrid/demo_mpc_hybrid_minimal.py` |
| Dual-rate hybrid MPC (`dual_rate_computer` + `u_nom`) | `examples/scripts/hybrid/demo_mpc_hybrid_dual_rate.py` |
| Hybrid MPC track + obstacles (`ModelPredictiveController` + `mpc @ plant`) | `examples/scripts/hybrid/demo_mpc_hybrid_track_lap.py` · [notebook](examples/notebooks/demo_mpc_hybrid_track_lap.ipynb) |
| Pyro SMC continuous (pendulum) | `examples/scripts/control/demo_sliding_mode_pendulum.py` |
| Pyro SMC continuous vs hybrid (pendulum) | `examples/scripts/hybrid/demo_smc_pendulum_compare.py` |
| Hand-loop MPC closed-loop / obstacle / stadium (``compute_command``) | `examples/scripts/mpc/` |
| Blocks (routing, filters, nonlinear) | `examples/scripts/blocks/` |
| Control | `examples/scripts/control/` |
| Robotic (impedance, computed torque, kinematic/nullspace, IK) | `examples/scripts/robotic/` |
| Analysis (linearize, trim, ctrb/obsv, modal) | `examples/scripts/analysis/` |
| State-space / LQR | `examples/scripts/statespace/` |
| Identification (param gradients) | `examples/scripts/identification/` |
| Plotting | `examples/scripts/plots/` |
| Animation | `examples/scripts/animation/` |
| Optimization | `examples/scripts/optimization/` |
| Planning (RRT, DP, corridor trajopt) | `examples/scripts/planning/` |
| MPC (rate-MPC bicycle demos; TOP `compile_parametric_program` + controller; legacy per-step trajopt: `demo_dynamic_bicycle_rate_mpc_straight_line_trajopt.py`; obstacle preset: `demo_dynamic_bicycle_rate_mpc_obstacle.py [small\|large]`; spatial scene guide: `demo_mpc_spatial_scene_guide.py`) | `examples/scripts/mpc/` · [spatial scene notebook](examples/notebooks/demo_mpc_spatial_scene_guide.ipynb) |
| Trajectory optimization | `examples/scripts/trajectory_optimization/` · [notebook](examples/notebooks/demo_bicycle_trajopt_obstacle_scene_compare.ipynb) · [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/notebooks/demo_bicycle_trajopt_obstacle_scene_compare.ipynb) |
| Symbolic mechanics | `examples/scripts/symbolic/` |
| Physics engine | `examples/scripts/engine/` |
| C export (P controller round-trip; filtered PID leaf) | `examples/scripts/interfaces/demo_c_export_proportional.py` · `demo_c_export.py` |
| Solver benchmarks | [examples/notebooks/simulation_benchmark.ipynb](examples/notebooks/simulation_benchmark.ipynb) (uses repo-root `benchmarks/`) |

Catalog plants live under `minilink.dynamics.catalog.*`.

## Docs

- [DESIGN.md](DESIGN.md) — principles and contracts
- [ROADMAP.md](ROADMAP.md) — maturity and priorities
- [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) — pyro 2.0 parity audit (library + all 195 demos)
- [docs/plans/](docs/plans/) — active design backlog
- [AGENTS.md](AGENTS.md) — contributor / agent rules
- API reference (Sphinx): [alx87grd.github.io/minilink](https://alx87grd.github.io/minilink/) (built from `main` via [.github/workflows/docs.yml](.github/workflows/docs.yml)); local build: `pip install -e ".[docs]" && sphinx-build -b html docs docs/_build/html`

Design rules: NumPy baseline, explicit JAX; native-array equation paths;
`params is None` means object defaults, never `params or self.params`. Coding
style: [AGENTS.md](AGENTS.md).
