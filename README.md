# minilink

**Write the equations once. Simulate, analyze, control, plan, optimize, learn.**

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb)
[![Notebooks](https://img.shields.io/badge/notebooks-examples%2Ftutorial-F37626?logo=jupyter&logoColor=white)](https://github.com/alx87grd/minilink/tree/main/examples/tutorial)
[![PyPI](https://img.shields.io/pypi/v/minilink)](https://pypi.org/project/minilink/)
[![API](https://img.shields.io/badge/API-alx87grd.github.io%2Fminilink-2563eb)](https://alx87grd.github.io/minilink/)
[![License: MIT](https://img.shields.io/badge/license-MIT-lightgrey)](LICENSE)

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

Minilink is an open-source Python toolbox for dynamical systems and control.
A model is a short class whose equations read like the textbook. Models
connect into block diagrams in code, with composition operators. Because
every model, controller and diagram is the same kind of object, one set of
tools applies to all of them: simulation and animation in 2D and 3D,
frequency-domain analysis, classical and state-space control, value
iteration, sampling-based planning, trajectory optimization, model predictive
control and reinforcement learning. The same equations compile and
differentiate under JAX, so a course model is also a high-performance
research model.

## Ten lines

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

![closed-loop diagram](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/diagram_closed_loop.png)

## What is a System

A model is dynamics `f`, outputs `h` and body poses `tf`, functions of the
state `x`, the input `u`, the time `t` and the parameters `params`:

    dx/dt = f(x, u, t; params)      dynamics
    y     = h(x, u, t; params)      one per output port, default y = x
    T     = tf(x, u, t; params)     body poses, for animation

Write `f`, and the plant simulates and plots. Write `h` to choose what the
sensor measures:

```python
import numpy as np
from minilink import DynamicSystem


class MassSpringDamper(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=1)

    def f(self, x, u, t=0, params=None):
        m, k, b = 1.0, 4.0, 0.3
        p, v = x

        # m p̈ + b ṗ + k p = u
        a = (u[0] - b * v - k * p) / m

        return np.array([v, a])

    def h(self, x, u, t=0, params=None):
        p, v = x

        return np.array([p])  # the sensor reads the position only


plant = MassSpringDamper()
plant.x0[0] = 1.0
plant.compute_trajectory(tf=10.0)
plant.plot_trajectory()
```

Add `tf` and a skin, and it animates on matplotlib, plotly, meshcat (3D) or
pygame, and you can drive it from the keyboard
([10_graphical](examples/tutorial/10_graphical.ipynb)).

`f`, `h` and `tf` are functions of `(x, u, t; params)` only: no hidden state on
the object. That one convention is what lets a model compose into diagrams,
run in batches and differentiate later. A diagram flattens to one state vector
and one `f`, so a closed loop linearizes, animates and nests like a plant.

## One model, every tool

![one System (f, h, tf) connected to simulation, analysis, control, planning, learning](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/bridges.svg)

The objects between the tools are the textbook's nouns. A `PlanningProblem`
is a system, a cost and boundary sets; every planner takes it and returns a
`PlanningSolution` (a policy, plus the trajectory when the solver computes one):

```python
from minilink import (
    BallSet, DynamicProgrammingPlanner, Pendulum, PlanningProblem,
    QuadraticCost, RRTPlanner, TrajectoryOptimizationPlanner,
)

plant = Pendulum()
x_down, x_up = np.array([0.0, 0.0]), np.array([np.pi, 0.0])
problem = PlanningProblem(
    sys=plant, x_start=x_down, x_goal=x_up, Xf=BallSet(x_up, 0.2), tf=4.0,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=x_up),
    X=plant.state.box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
vi.solve()                                   # value iteration on a grid
loop = vi.get_controller() @ plant           # the policy is a controller

rrt = RRTPlanner(problem, seed=0)
tree_traj = rrt.solve().trajectory           # kinodynamic tree search, bang-bang inputs

opt = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation")
opt_traj = opt.solve().trajectory            # direct collocation
```

Trajectory optimization transcribes the problem into a `MathematicalProgram`
solved by an `Optimizer`.

## Differentiable and compiled

The same `f` traces under JAX. One evaluator gives exact derivatives, batched
rollouts and gradients through a whole simulation:

```python
ev = plant.compile(backend="jax")

A = plant.jacobian("f", "x", x_bar)               # exact linearization
S = plant.jacobian("f", "params", x_bar)          # sensitivity to each physical parameter
xs = ev.rollout_batch(x0s, n_steps=1000, dt=0.005,
                      params=dict(plant.params, l=lengths))   # a family of rod lengths, one call
```

Measured in the showcase notebook: 1000 rollouts of 1000 RK4 steps take
27 ms as a compiled batch and about 32 s one step at a time in Python
(Apple M4 Max). Derivatives are exact to machine precision, float64 by
default. Under the hood, `compile()` lowers a leaf or a wired diagram to flat
NumPy or JAX primitives (`f`, `rk4_step`, `rk4_integrate_zoh`,
`rollout_batch`); the trace tier (`f_trace`, `f_trace_p`) is what you
differentiate inside your own `jit`. See
[07_compile](examples/tutorial/07_compile.ipynb) and the
[JAX showcase](examples/tutorial/showcase_jax.ipynb).

## Two audiences, one codebase

- **Teaching.** NumPy, SciPy and Matplotlib are enough for simulation, phase
  planes, animation, linearization, LQR and value iteration. Runs in Colab
  from one setup cell. One import line covers a course:
  `from minilink import Pendulum, PID, lqr, PlanningProblem, ReinforcementLearningPlanner`.
- **Research.** Optional JAX for compile and autodiff, Ipopt for large NLPs,
  meshcat for 3D, a hybrid stack for sampled MPC. Every catalog plant compiles on both
  backends.

## Install

Python 3.10+. Recommended: the conda environment from
[`environment.yml`](environment.yml).

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml && conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

Or open any notebook in Colab: the first cell clones the repository. Basic
tier, pip, and options: [install.md](install.md).

## Learn more

- [Showcase notebook](examples/tutorial/showcase_minilink.ipynb), the tool ladder on real plants
- [JAX showcase](examples/tutorial/showcase_jax.ipynb), write `f` once, get every gradient
- [From RL to Bode showcase](examples/tutorial/showcase_from_rl_to_bode.ipynb), six-axis robot, impedance loop, neural policy, frequency response and Lyapunov certificates
- [Tutorial series 00–11](examples/tutorial/), one notebook per package (core dynamics to reinforcement learning)
- [Teaching notebooks](examples/teaching/), swing-up, DP, PPO, robot equations of motion
- [Examples index](examples/README.md), demos and projects by chapter
- [API reference](https://alx87grd.github.io/minilink/), [DESIGN.md](DESIGN.md), [ROADMAP.md](ROADMAP.md), [tests](tests/README.md)
- [CONSTITUTION.md](CONSTITUTION.md) and [RULES.md](RULES.md), the design law and the code rules every contributor and agent follows

Minilink is the successor of [pyro](https://github.com/SherbyRobotics/pyro).
