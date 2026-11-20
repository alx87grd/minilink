# minilink

**Write the equations once. Simulate, analyze, control, plan, optimize, learn.**

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb)
[![Notebooks](https://img.shields.io/badge/notebooks-examples%2Ftutorial-F37626?logo=jupyter&logoColor=white)](https://github.com/alx87grd/minilink/tree/main/examples/tutorial)
[![PyPI](https://img.shields.io/pypi/v/minilink)](https://pypi.org/project/minilink/)
[![API](https://img.shields.io/badge/API-alx87grd.github.io%2Fminilink-2563eb)](https://alx87grd.github.io/minilink/)
[![License: MIT](https://img.shields.io/badge/license-MIT-lightgrey)](https://github.com/alx87grd/minilink/blob/main/LICENSE)

<table>
  <tr>
<td width="25%" align="center" valign="bottom"><img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/cartpole_swingup.gif" alt="Cart-pole swing-up, trajectory optimization" width="100%"/></td>
<td width="25%" align="center" valign="bottom"><img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/ur5_meshcat.gif" alt="UR5 arm, task-space impedance control" width="100%"/></td>
<td width="25%" align="center" valign="bottom"><img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/racecar_mpc.gif" alt="Racecar, model predictive control" width="100%"/></td>
<td width="25%" align="center" valign="bottom"><img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/rocket_landing.gif" alt="Rocket landing, reinforcement learning" width="100%"/></td>
  </tr>
  <tr>
<td width="25%" align="center" valign="top">Cart-pole swing-up, trajectory optimization</td>
<td width="25%" align="center" valign="top"><a href="#7-interactive-play-and-3-d-visualization">UR5 arm, task-space impedance control</a><br/><a href="https://alx87grd.github.io/minilink/_static/showcase/ur5_impedance.html">interactive 3-D</a></td>
<td width="25%" align="center" valign="top"><a href="#4-one-problem-every-planner">Racecar, model predictive control</a><br/><a href="https://alx87grd.github.io/minilink/_static/showcase/racecar_mpc.html">interactive 3-D</a></td>
<td width="25%" align="center" valign="top">Rocket landing, reinforcement learning</td>
  </tr>
</table>

Minilink is an open-source Python framework for dynamical systems, control, optimization, and learning.
Write a physical model once in pure textbook ODE equations as the core object in Minilink: a `System`. Compose models and blocks into block diagrams, which are also just new Systems. This unified `System` is the standard interface to all tools: classical and modern control, optimal control (MPC, MPPI), machine learning (neural networks, reinforcement learning), and parameter optimization.
All core equation paths are pure functions—arrays in, arrays out—allowing the exact same code to work with either a minimalist NumPy/SciPy/Matplotlib setup or a JAX backend to compile, batch, and autodifferentiate the same equations.
One planning problem is handed to every planner, and `compare` scores the controllers they return on the same problem.

## Install

Python 3.10+. **pip**:
- `pip install minilink` runs simulation, plots, LQR, and value iteration on standard NumPy, SciPy, and Matplotlib.
- `pip install "minilink[full]"` adds JAX, Plotly, MeshCat, Pygame, SymPy, and Gymnasium.

**Conda**, everything included (JAX, Graphviz, Ipopt, 3-D animation, Jupyter):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml && conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

**Colab**: open any notebook with "Open in Colab", or start yours with
`!git clone https://github.com/alx87grd/minilink` and `import sys; sys.path.insert(0, "/content/minilink")`.

*Optional system dependencies:* `plot_diagram()` requires Graphviz ([download](https://graphviz.org/download/), then `pip install graphviz`); the Ipopt solver is `conda install -c conda-forge cyipopt` (trajectory optimization runs on SciPy without it).

The snippets below run as one session; sections 3, 4 (MPC) and 6 use JAX.

## 1. A closed loop

```python
from minilink import Impedance, Pendulum

plant = Pendulum()
plant.x0 = [2.0, 0.0]

loop = Impedance(Kp=100.0, Kd=20.0) @ plant
loop.compute_trajectory(tf=6.0)
loop.plot_trajectory()
loop.animate()
```

`ctrl @ plant` closes the feedback loop, and the loop is itself a `System`.

## 2. Models and diagrams in code

A physical model is defined by three pure mathematical mappings:

$$\dot{x} = f(x, u, t; p) \quad \text{(dynamics)}$$
$$y = h(x, u, t; p) \quad \text{(outputs, default } y = x\text{)}$$
$$T = \mathrm{tf}(x, u, t; p) \quad \text{(body poses for 3D animation)}$$

**Functional purity** means equations are stateless: arrays in, arrays out. No simulation time or state is mutated on `self`.
This purity allows precise numerical integration, reproducible experiments, exact linear analysis, arbitrary composition into nested systems, parallel GPU rollouts, and automatic differentiation.

```python
import numpy as np
from minilink import PID, DynamicSystem


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(state_dim=2, input_dim=1, output_dim=1)

    def f(self, x, u, t=0, params=None):
        m, k, b = 1.0, 4.0, 0.3
        p, v = x
        a = (u[0] - b * v - k * p) / m
        return np.array([v, a])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])  # sensor measures position only


pid_loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ MassSpringDamper()
pid_loop.plot_diagram()
```

![closed-loop diagram](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/diagram_closed_loop.png)

Code is the single source of truth:

- **Algebraic shortcuts:** `ctrl @ plant` (feedback loop, with the error block the PID reads), `a >> b` (series), `a + b` (side by side), `sys % Δ` (zero-order hold digital clock sampling).
- **Explicit wiring:** `diag.connect("controller", "u", "plant", "u")` connects arbitrary multi-loop networks with immediate port checks.
- **Auto-generated diagrams:** `loop.plot_diagram()` draws the diagram above directly from code.

```python
from minilink import Step

reference = Step(final_value=[1.0], step_time=1.0)
digital = reference >> (Impedance(Kp=100.0, Kd=20.0) % 0.1) @ plant
digital.compute_trajectory(tf=6.0)
```

Because a composite diagram is itself a `DynamicSystem`, **every analysis verb works identically on closed loops**: `loop.linearize()`, `loop.transfer_function()`, `loop.plot_bode()`, `loop.plot_root_locus()`, `loop.plot_pzmap()`, and `loop.plot_step_response()`. State feedback is one call: `lqr` designs on the linear model at a point, and `kalman` and `lqg` are its duals for estimation.

```python
from minilink import lqr

loop.transfer_function()     # G(s) = 50 / (s² + 10 s + 54.91)
loop.plot_bode()
loop.plot_root_locus()

up = np.array([np.pi, 0.0])
balance = lqr(plant, Q=np.eye(2), R=np.eye(1), x_bar=up) @ plant
balance.compute_trajectory(tf=4.0, x0=[np.pi - 0.5, 0.0])
balance.animate()
```

Parameters are wired through the diagram too: `loop.params` is one nested tree,
`{'controller': {'Kp': ..., 'Kd': ...}, 'plant': {'m': ..., 'l': ...}}`, and a dotted path such
as `"plant.l"` names any number in any block. Sections 3, 5 and 6 use it.

## 3. Differentiable and batched with JAX

The exact same equation $f(x, u, t; p)$ traces under JAX without rewriting physics:

- **Exact sensitivities:** analytical Jacobians
  $$A = \frac{\partial f}{\partial x}, \quad B = \frac{\partial f}{\partial u}, \quad S_p = \frac{\partial f}{\partial p}$$
- **Batched rollouts:** vectorized simulation across parameter sweeps via JAX `vmap` (1,000 rollouts in 0.4 s on a cloud CPU vs. about 50 s in a NumPy loop).
- **Opt-in acceleration:** pure NumPy remains the default classroom engine (its Jacobians are finite differences); JAX is activated with a single flag (`backend="jax"`).

```python
x = np.array([1.0, 0.0])
A = plant.jacobian("f", "x", x)
S = plant.jacobian("f", "params", x)

lengths = np.linspace(0.5, 2.0, 1000)
starts = np.full((1000, 2), [1.0, 0.0])
swings = plant.compile(backend="jax").rollout_batch(
    starts, n_steps=1000, dt=0.005, params=dict(plant.params, l=lengths)
)
```

## 4. One problem, every planner

A `PlanningProblem` bundles the physical plant, boundary sets ($\mathcal{X}, \mathcal{U}, \mathcal{X}_f$), cost function $J$, and horizon $t_f$.
The identical problem definition is handed to completely different solver paradigms:

- **Dynamic programming (VI):** global value iteration on state/action grids.
- **Tree search (RRT):** kinodynamic motion planning.
- **Trajectory optimization (TrajOpt):** direct collocation and multiple shooting NLP.
- **Model predictive control (MPC):** trajectory optimization solved again at every tick from the measured state.
- **Reinforcement learning (RL):** PPO / SAC neural policy optimization, in the [showcase notebook](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb).

Here the task is to swing the pendulum up with a motor too weak to lift it directly; the cost
$J = \int (\tilde{x}^T Q \tilde{x} + u^T R u)\,dt + \tilde{x}(t_f)^T S\, \tilde{x}(t_f)$,
$\tilde{x} = x - x_{up}$, has a terminal weight $S$ on where the swing ends.

```python
from minilink import (
    BallSet, DynamicProgrammingPlanner, PlanningProblem, QuadraticCost,
    RRTPlanner, TrajectoryOptimizationPlanner,
)
from minilink.control import ModelPredictiveController

problem = PlanningProblem(
    plant, x0=[0.0, 0.0], Xf=BallSet(up, 0.2), horizon=6.0,
    cost=QuadraticCost.from_system(plant, x_bar=up, S=100 * np.eye(2)),
    X=plant.state.box, U=plant.input_box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05).solve()
rrt = RRTPlanner(problem, seed=0).solve()
opt = TrajectoryOptimizationPlanner(problem, n_steps=60).solve()

look_ahead = problem.replace(horizon=2.0, Xf=None)
mpc = ModelPredictiveController(TrajectoryOptimizationPlanner(look_ahead, n_steps=20, backend="jax")) % 0.1

swing_up = vi.controller @ plant
swing_up.compute_trajectory(tf=6.0, x0=[0.0, 0.0])
swing_up.animate()
```

Every planner returns a `PlanningSolution` whose controller is itself a `System`: a feedback law closes the loop via `controller @ plant`, an open-loop plan drives it via `controller >> plant`. MPC is a controller block already, a planner on a 0.1 s clock.

## 5. One score

`compare` scores every controller on the same problem: the same model, cost, sets, and draws. Here the mass is uncertain by ±20 %, and each controller runs on the same twenty draws:

```python
from minilink import Uniform, compare

uncertain = problem.replace(distributions={"plant": {"m": Uniform(0.8, 1.2)}})
print(compare(uncertain, VI=vi, RRT=rrt, TrajOpt=opt, MPC=mpc, n_trials=20))
```

```
===================================================================================
Comparison, Pendulum, horizon=6, criterion=expectation, 20 draws
===================================================================================
        │ nominal trajectory          │ simulation
        │     J  goal     constraints │  E[J]   σ(J)      max J  goal   constraints
────────┼─────────────────────────────┼────────────────────────────────────────────
VI      │ -                           │ 91.07  7.708      109.6  20/20  20/20
RRT     │ 149.2  reached  ok          │  7419   9385  2.641e+04  0/20   4/20
TrajOpt │  90.5  reached  ok          │  8018   5924  1.881e+04  0/20   7/20
MPC     │ -                           │ 130.3  95.56      403.2  17/20  20/20
===================================================================================
```

The two open-loop plans reach the goal on the nominal model and miss it when the mass changes; the two feedback laws reach it on most draws.

## 6. The unified optimization boundary

All transcriptions lower late to a single, canonical mathematical program:

$$\min_z J(z; p) \quad \text{s.t.} \quad g(z; p) \ge 0, \quad h(z; p) = 0$$

- **One program:** trajectory collocation, parameter tuning, and policy search all lower to `MathematicalProgram`.
- **Plant and controller co-design:** closed-loop objective gradients $\nabla_\theta J$ optimize control gains and physical parameters simultaneously, named by their paths in the parameter tree.
- **Pluggable solvers:** `method=` picks one: `l-bfgs`, `slsqp`, `trust-constr` (SciPy), `gd`, `adam` (first-order), `ipopt`; a program with constraints takes `slsqp`, `trust-constr`, or `ipopt`.

Here: bring the pendulum, released at 2 rad, to rest, by tuning the two gains and the rod length within bounds.

```python
from minilink import Optimizer

task = PlanningProblem(plant, x0=[2.0, 0.0], horizon=5.0, cost=QuadraticCost.from_system(plant, R=0.1 * np.eye(1)))
evaluator = task.compile(Impedance(Kp=1.0, Kd=1.0), backend="jax")
program = evaluator.program(
    ["controller.Kp", "controller.Kd", "plant.l"],
    bounds={"controller.Kp": (0.0, 50.0), "controller.Kd": (0.0, 20.0), "plant.l": (0.5, 2.0)},
)
best = Optimizer(program).solve()   # Kp = 1.47, Kd = 3.80, l = 0.5
```

The rod length goes to its lower bound, since the cost does not penalize it.

## 7. Interactive play and 3-D visualization

- **Real-time wall-clock play:** `plant.game()` drives plants with keyboard or gamepad (via Pygame).
- **Decoupled geometry:** body poses $T = \mathrm{tf}(x, u, t; p)$ describe kinematics without cluttering the differential equations with rendering code.
- **3-D in the browser:** `animate(traj, renderer="meshcat")` plays the trajectory in an interactive 3-D viewer with orbit, pan, and playback controls: inline in a notebook, in the browser from a script.
- **One way to plot, animate and save:** the plots (`plot_trajectory`, `plot_bode`, `plot_cost_to_go`, …) take `renderer="matplotlib"` or `"plotly"`, `show=`, and `ax=` (matplotlib axes, to draw on another plot's) and return a `PlotResult`; the animations return an `AnimationResult`; each saves itself: `result.save("bode.pdf")`, `anim.save("lap.html")`.
- **Robotics catalog:** kinematic and dynamic models for robot arms (`UR5Manipulator`), ground vehicles (`KinematicBicycle`, `UdeSRacecar`), drones (`Drone2D`), and aerospace vehicles (`Rocket`).

The same lines on a robot: a task-space impedance law moves the UR5's hand to a target.

```python
from minilink import TaskImpedance, UR5Manipulator

robot = UR5Manipulator()
target = Step(initial_value=[-0.8, -0.2, 0.0], final_value=[-0.5, 0.0, 0.5], step_time=1.0)
arm_loop = target >> TaskImpedance(robot, gravity_comp=True) @ robot
arm_loop.animate(arm_loop.compute_trajectory(tf=5.0), renderer="meshcat")
```

*Live interactive 3D demos:* [UR-5 Impedance Control](https://alx87grd.github.io/minilink/_static/showcase/ur5_impedance.html) • [Racecar MPC](https://alx87grd.github.io/minilink/_static/showcase/racecar_mpc.html).

## The Minilink backbone

Three mathematical foundations hold the framework together:
1. **The model (`System`):** continuous differential equations $\dot{x} = f(x, u, t; p)$ and observations $y = h(x, u, t; p)$.
2. **The task (`PlanningProblem`):** the model paired with boundary sets $\mathcal{X}, \mathcal{U}, \mathcal{X}_f$, cost $J$, and time horizon $t_f$.
3. **The solver boundary (`MathematicalProgram` & `Optimizer`):** a standard nonlinear program $\min_z J(z)$ s.t. $g(z) \ge 0,\ h(z) = 0$ connecting problems to plug-and-play numerical solvers.

One differentiable path connects modeling, analytical control, trajectory optimization, and learning:

![The Minilink Backbone](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/pipeline_map.png)

*Full pipeline reference: [docs/pipelines.md](https://github.com/alx87grd/minilink/blob/main/docs/pipelines.md).*

What Minilink is not:
1. **Not a heavy multibody contact engine:** for complex contact or high-DoF meshes, wrap an external multibody simulator as a `System` leaf.
2. **Not a sparse NLP modeling language:** Minilink programs and solvers do not exploit sparse Jacobian structure.
3. **Continuous time is primary:** dynamics $f$ represent instantaneous rates ($\dot{x} = dx/dt$), not discrete next-step maps; a clock is applied from outside with `%`.

## Learn more and tutorials

- [Interactive pitch deck](https://alx87grd.github.io/minilink/presentation.html): a 14-slide presentation.
- [Showcase notebook](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb): this page run end to end on the pendulum, with the plots, the animations, and reinforcement learning; about a minute to run.
- [Tutorials](https://github.com/alx87grd/minilink/tree/main/examples/tutorial): three layers: `basics/` (model, connect, simulate; start with `01_minilink`), `core/` (the foundation objects and the problem pipeline), `advanced/` (discrete and hybrid loops, optimal control, MPC, RL, differentiable loops).
- [Teaching notebooks](https://github.com/alx87grd/teaching): course and topic notebooks in their own repository: control, dynamic programming, LQR, MPC, reinforcement learning.
- [API reference](https://alx87grd.github.io/minilink/).

Minilink is the successor of [pyro](https://github.com/SherbyRobotics/pyro).
