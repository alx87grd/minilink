# minilink

**Write the equations once. Simulate, analyze, control, plan, optimize, learn.**

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb)
[![Notebooks](https://img.shields.io/badge/notebooks-examples%2Ftutorial-F37626?logo=jupyter&logoColor=white)](https://github.com/alx87grd/minilink/tree/main/examples/tutorial)
[![PyPI](https://img.shields.io/pypi/v/minilink)](https://pypi.org/project/minilink/)
[![API](https://img.shields.io/badge/API-alx87grd.github.io%2Fminilink-2563eb)](https://alx87grd.github.io/minilink/)
[![License: MIT](https://img.shields.io/badge/license-MIT-lightgrey)](https://github.com/alx87grd/minilink/blob/main/LICENSE)

<table>
  <tr>
    <td width="25%" align="center" valign="bottom">
      <img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/cartpole_swingup.gif" alt="trajectory optimization" width="100%"/>
    </td>
    <td width="25%" align="center" valign="bottom">
      <img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/ur5_meshcat.gif" alt="task-space impedance control" width="100%"/>
    </td>
    <td width="25%" align="center" valign="bottom">
      <img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/racecar_mpc.gif" alt="model predictive control" width="100%"/>
    </td>
    <td width="25%" align="center" valign="bottom">
      <img src="https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/rocket_landing.gif" alt="reinforcement learning" width="100%"/>
    </td>
  </tr>
  <tr>
    <td width="25%" align="center" valign="top">
      trajectory optimization
    </td>
    <td width="25%" align="center" valign="top">
      impedance control<br/>
      <a href="https://alx87grd.github.io/minilink/_static/showcase/ur5_impedance.html">interactive 3-D</a>
    </td>
    <td width="25%" align="center" valign="top">
      model predictive control<br/>
      <a href="https://alx87grd.github.io/minilink/_static/showcase/racecar_mpc.html">interactive 3-D</a>
    </td>
    <td width="25%" align="center" valign="top">
      reinforcement learning
    </td>
  </tr>
</table>

Minilink is an open-source Python framework for dynamical systems, control, and optimization.
Engineers, students, and researchers write a physical model once as a continuous `DynamicSystem`.
The same model powers simulation, classical and modern control, trajectory optimization, and reinforcement learning.
NumPy and SciPy provide a zero-friction default for courses; the identical equations compile, batch, and differentiate under JAX for research.

---

## 1. The Minilink Backbone

Three mathematical foundations hold the framework together:
1. **The Model (`System`):** Continuous differential equations $\dot{x} = f(x, u, t; p)$ and observations $y = h(x, u, t; p)$.
2. **The Task (`PlanningProblem`):** The model paired with boundary sets $\mathcal{X}, \mathcal{U}, \mathcal{X}_f$, cost $J$, and time horizon $t_f$.
3. **The Solver Boundary (`MathematicalProgram` & `Optimizer`):** Standard non-linear program $\min_z J(z) \text{ s.t. } g(z) \le 0, h(z) = 0$ connecting problems to plug-and-play numerical solvers.

One differentiable path connects modeling, analytical control, trajectory optimization, and learning:

![The Minilink Backbone](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/pipeline_map.png)

*Full pipeline reference: [docs/pipelines.md](https://github.com/alx87grd/minilink/blob/main/docs/pipelines.md).*

---

## 2. Continuous Core & Functional Purity

A physical model is defined by three pure mathematical mappings:

$$\dot{x} = f(x, u, t; p) \quad \text{(dynamics)}$$
$$y = h(x, u, t; p) \quad \text{(outputs, default } y = x\text{)}$$
$$T = \mathrm{tf}(x, u, t; p) \quad \text{(body poses for 3D animation)}$$

**Functional purity** means equations are stateless: arrays in, arrays out. No simulation time or state is mutated on `self`.
This purity guarantees exact numerical stepping, arbitrary block nesting, parallel GPU rollouts, and automatic differentiation.

```python
import numpy as np
from minilink import DynamicSystem


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1)

    def f(self, x, u, t=0, params=None):
        m, k, b = 1.0, 4.0, 0.3
        p, v = x
        a = (u[0] - b * v - k * p) / m
        return np.array([v, a])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])  # sensor measures position only


plant = MassSpringDamper()
plant.x0[0] = 1.0
plant.compute_trajectory(tf=10.0)
plant.plot_trajectory()
```

- **Classroom Default:** Pure NumPy and SciPy out of the box with zero compilation steps or license hurdles.
- **Extensible Leaves:** Easily wrap external C/Python physics simulators, hardware sensors, or custom dynamics as a `System` leaf.

---

## 3. Compositional Closure: Diagrams as Code

Code is the single source of truth. Minilink replaces Simulink's mouse-driven wiring and opaque binary `.slx` files with pure, version-controlled Python expressions:

- **Algebraic Shortcuts:** `ctrl @ plant` (feedback loop), `a >> b` (series), `a + b` (parallel), `sys % Δ` (zero-order hold digital clock sampling).
- **Explicit 1-Line Wiring:** `diag.connect("controller", "u", "plant", "u")` connects arbitrary multi-loop networks with immediate port checks.
- **Auto-Generated Diagrams:** `loop.plot_diagram()` generates clean, read-only visual inspection diagrams directly from code.

```python
from minilink import PID, MassSpringDamper

plant = MassSpringDamper()
ctrl = PID(kp=10.0, ki=1.0, kd=2.0)

# Feedback composition: loop is itself a pure DynamicSystem
loop = ctrl @ plant

loop.compute_trajectory(tf=10.0)
loop.plot_diagram()
loop.plot_trajectory()
```

![closed-loop diagram](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/diagram_closed_loop.png)

Because a composite diagram is itself a `DynamicSystem`, **every analysis verb works identically on closed loops**:
- `loop.linearize()`, `loop.bode()`, `loop.nyquist()`, `loop.root_locus()`, `loop.pzmap()`, `loop.step_response()`, and `loop.kalman()`.
- Runs headlessly in automated Linux Docker containers and CI/CD pipelines with zero display servers or seat license fees.

---

## 4. Interactive Play & 3D Visualization

- **Real-Time Wall-Clock Play:** `plant.game()` drives plants with keyboard or gamepad (via Pygame). Paces hardware-in-the-loop testbeds at up to 1 kHz.
- **Decoupled Geometry:** Body poses $T = \mathrm{tf}(x, u, t; p)$ describe kinematics without cluttering the differential equations with rendering code.
- **Hardware-Accelerated 3D WebGL:** `plant.animate(traj, renderer="meshcat")` opens an interactive 3D browser viewer with orbit, pan, and playback controls.
- **Multi-DoF Robotics Catalog:** Built-in kinematic and dynamic models for industrial manipulators (`UR5Manipulator`), ground vehicles (`KinematicBicycle`, `UdeSRacecar`), quadrotors (`Drone2D`, `Drone3D`), and aerospace vehicles (`Rocket`).

```python
from minilink import UR5Manipulator

robot = UR5Manipulator()
traj = robot.compute_trajectory(tf=5.0)
robot.animate(traj, renderer="meshcat")  # interactive 3D WebGL in browser
```

*Live interactive 3D demos:* [UR-5 Impedance Control](https://alx87grd.github.io/minilink/_static/showcase/ur5_impedance.html) • [Racecar MPC](https://alx87grd.github.io/minilink/_static/showcase/racecar_mpc.html).

---

## 5. Differentiable & Accelerated with JAX

The exact same equation $f(x, u, t; p)$ traces under JAX without rewriting physics:

- **Exact Machine-Precision Sensitivities:** Analytical Jacobians eliminate finite-difference approximation noise:
  $$A = \frac{\partial f}{\partial x}, \quad B = \frac{\partial f}{\partial u}, \quad S_p = \frac{\partial f}{\partial p}$$
- **Parallel GPU-Batched Rollouts:** Vectorized simulation across parameter sweeps via JAX `vmap` (1,000 rollouts in 27 ms vs. 32 s in pure Python).
- **Plant & Controller Co-Design:** Compute closed-loop objective gradients $\nabla_\theta J$ to optimize control gains and physical parameters simultaneously.
- **Opt-In Acceleration:** Pure NumPy remains the default classroom engine; JAX is activated with a single flag (`backend="jax"`).

```python
import jax
import numpy as np
from minilink import PID, Optimizer, Pendulum, PlanningProblem, QuadraticCost

plant = Pendulum()

# Exact machine-precision Jacobians
A = plant.jacobian("f", "x", x_eq)
S = plant.jacobian("f", "params", x_eq)

# Vectorized rollouts over 1,000 rod lengths in a single call
ev = plant.compile(backend="jax")
x0s = np.tile([1.0, 0.0], (1000, 1))
lengths = np.linspace(0.5, 2.0, 1000)
xs = ev.rollout_batch(x0s, n_steps=1000, dt=0.005, params=dict(plant.params, l=lengths))

# Closed-loop gradient descent on gains and physical parameters
tuning = PlanningProblem(plant, x_start=[1.0, 0.0], cost=QuadraticCost.from_system(plant), tf=4.0)
pe = tuning.compile(PID(2.0, 0.5, 1.0), dt=0.02)
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
best = Optimizer(pe.program(wrt=["controller.Kp", "plant.l"])).solve()
```

---

## 6. The Unified Optimization Boundary

All transcriptions lower late to a single, canonical mathematical program:

$$\min_z J(z; p) \quad \text{s.t.} \quad g(z; p) \le 0, \quad h(z; p) = 0$$

- **Lego-Block Decoupling:** Trajectory collocation, parameter tuning, and policy search all lower to `MathematicalProgram`.
- **Pluggable Solvers:** Decoupled interfaces allow any optimizer to run on any program:
  - First-order autodiff: `gd`, `adam`, `l-bfgs` (via JAX).
  - Constrained non-linear programming: SciPy (`SLSQP`), Ipopt (interior-point).
  - Custom user algorithms and native fixed-step solvers.

```python
from minilink import Optimizer

# Solvers plug into any problem definition
prog = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
opt = Optimizer(prog, method="l-bfgs")  # or method="ipopt", "slsqp", "adam"
sol = opt.solve()
```

---

## 7. One Problem, Every Planner

A `PlanningProblem` bundles the physical plant, boundary sets ($\mathcal{X}, \mathcal{U}, \mathcal{X}_f$), cost function $J$, and horizon $t_f$.
The identical problem definition is handed to completely different solver paradigms:

- **Dynamic Programming (VI):** Global value iteration on state/action grids.
- **Tree Search (RRT):** Kinodynamic motion planning with obstacle collision checking.
- **Trajectory Optimization (TrajOpt):** Direct collocation and multiple shooting NLP.
- **Reinforcement Learning (RL):** PPO / SAC neural policy optimization.

Every planner returns a `PlanningSolution` whose policy is itself a `System` that closes the loop via `policy @ plant`:

```python
import numpy as np
from minilink import (
    BallSet, DynamicProgrammingPlanner, Pendulum, PlanningProblem,
    QuadraticCost, RRTPlanner, TrajectoryOptimizationPlanner, compare,
)

plant = Pendulum()
x_down, x_up = np.array([0.0, 0.0]), np.array([np.pi, 0.0])

problem = PlanningProblem(
    sys=plant, x_start=x_down, Xf=BallSet(x_up, 0.2), tf=4.0,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=x_up),
    X=plant.state.box, U=plant.input_box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
rrt = RRTPlanner(problem, seed=0)
opt = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation")

vi_sol  = vi.solve()   # policy is a lookup controller
rrt_sol = rrt.solve()  # kinodynamic trajectory
opt_sol = opt.solve()  # continuous trajectory solution

# Closed-loop composition: policy is a standard System
loop = vi_sol.policy @ plant

# Objective benchmark across solver paradigms
print(compare(problem, VI=vi_sol, RRT=rrt_sol, TrajOpt=opt_sol, dt=0.05))
```

---

## 8. Honest Boundaries: What Minilink is NOT

Minilink maintains rigorous scope boundaries to protect simplicity and textbook readability:

1. **Not a Heavy Multibody Contact Engine:** Minilink is not MuJoCo, Bullet, or Isaac Sim. For complex contact or high-DoF meshes, wrap external simulators as a `System` leaf. Minilink focuses on control architecture, analytical derivatives, and trajectory optimization.
2. **Not a 100+ Knot Sparse NLP Modeling Language:** CasADi remains the gold standard for massive, sparse NLP collocation problems. Minilink treats NLP solvers as execution backends rather than rewriting a symbolic AD graph engine.
3. **Continuous-Time is Primary:** Dynamics $f$ represent instantaneous rates ($\dot{x} = dx/dt$), not discrete next-step maps. Discrete and hybrid clocks (`% Δ`) sit cleanly beside the continuous core without compromising it.

---

## 9. Key Advantages

| Advantage | What It Solves |
| --- | --- |
| **One Model Across All Domains** | Define physics once ($\dot{x} = f$); seamlessly move between frequency analysis, LQR, TrajOpt, and RL without rewriting models. |
| **Zero-Friction Classroom Adoption** | Pure NumPy/SciPy baseline; runs immediately in 1-click Google Colab with zero compiler or license setup on student laptops. |
| **Git-Native Diagrams as Code** | Pure Python operators (`>>`, `+`, `@`, `%`) replace opaque binary `.slx` files with clear, reviewable pull request diffs. |
| **Hardware-Accelerated with JAX** | Machine-precision autodiff sensitivities, GPU `vmap` rollouts, and exact plant-controller co-design ($\nabla_\theta J$). |
| **Unified Optimization Boundary** | Standardized `MathematicalProgram` and `PlanningProblem` decouple formulations from plug-and-play numerical solvers. |

---

## Install

Python 3.10+. **Recommended: conda**, everything included (JAX, Graphviz, Ipopt, 3-D animation, Jupyter):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml && conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

**Colab**: open any notebook with "Open in Colab", or start yours with:

```python
!git clone https://github.com/alx87grd/minilink
import sys; sys.path.insert(0, "/content/minilink")
```

**pip**:
- `pip install minilink` runs simulation, plots, LQR, and value iteration on standard NumPy, SciPy, and Matplotlib.
- `pip install "minilink[full]"` adds JAX, Plotly, MeshCat, Pygame, SymPy, and Gymnasium.

*Optional system dependencies:*
- `plot_diagram()` requires Graphviz ([download](https://graphviz.org/download/), then `pip install graphviz`).
- Ipopt solver: `conda install -c conda-forge cyipopt` (trajectory optimization runs on SciPy without it).

---

## Learn More & Tutorials

- [Interactive Pitch Deck](https://alx87grd.github.io/minilink/presentation.html) — self-contained 14-slide executive, academic, and industrial presentation deck.
- [Showcase Notebook](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb) — the complete tool ladder demonstrated on real plants.
- [JAX Showcase](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_jax.ipynb) — write $f$ once, get exact derivatives and batched GPU rollouts.
- [From RL to Bode Showcase](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_from_rl_to_bode.ipynb) — 6-axis robot, impedance loop, neural policy, and frequency response.
- [Tutorial Series 00–11](https://github.com/alx87grd/minilink/tree/main/examples/tutorial) — 12 guided notebooks covering core dynamics through reinforcement learning.
- [Teaching Notebooks](https://github.com/alx87grd/minilink/tree/main/examples/teaching) — swing-up control, dynamic programming, and PPO.
- [Interactive 3D WebGL Showcases](https://alx87grd.github.io/minilink/):
  - [UR-5 Robot Impedance Control](https://alx87grd.github.io/minilink/_static/showcase/ur5_impedance.html)
  - [Racecar Model Predictive Control](https://alx87grd.github.io/minilink/_static/showcase/racecar_mpc.html)
  - [ANCF Dynamic Tire Ring Prototype](https://alx87grd.github.io/minilink/_static/showcase/ancf_tire.html)
- [Examples Index](https://github.com/alx87grd/minilink/blob/main/examples/README.md) — complete catalog of scripts and demos.
- [API Reference](https://alx87grd.github.io/minilink/) — full documentation.
- [Developer Contracts](https://github.com/alx87grd/minilink/tree/main/docs/dev) — architectural constitution, rules, and design documents.

Minilink is the successor of [pyro](https://github.com/SherbyRobotics/pyro).
