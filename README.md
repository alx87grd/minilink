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

Minilink is an open-source Python framework for dynamical systems and control. A
model is a `DynamicSystem`: a short class whose equations read like the textbook.
Models and controllers, yours or from the catalog, compose into block diagrams with
three operators, and a diagram is itself a `DynamicSystem`. One toolbox works on any
of them: simulation and animation, linearization and frequency response, classical
and state-space design, value iteration, sampling-based planning, trajectory
optimization, model predictive control and reinforcement learning. NumPy, SciPy and
Matplotlib are enough for a course; the same equations compile and differentiate
under JAX for research.

## Write a model

A model is dynamics `f`, outputs `h` and body poses `tf`, functions of the state
`x`, the input `u`, the time `t` and the parameters `params`:

    dx/dt = f(x, u, t; params)      dynamics
    y     = h(x, u, t; params)      outputs, default y = x
    T     = tf(x, u, t; params)     body poses, for animation

Write `f`, and the model simulates and plots. Write `h` to choose what the sensor
measures:

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

Add `tf` and a skin, and it animates on matplotlib, plotly, meshcat (3-D) or pygame,
and you can drive it from the keyboard
([10_graphical](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/10_graphical.ipynb)).
`f`, `h` and `tf` read `(x, u, t; params)` and nothing else: no hidden state on the
object. That one convention is what lets a model compose, run in batches and
differentiate.

## Compose into a diagram

`controller @ plant` closes a loop, `a >> b` is a series connection, `a + b` puts
blocks side by side. The controller can be yours, or one of the catalog's:

```python
from minilink import PID

loop = PID(2.0, 0.5, 1.0) @ plant
loop.compute_trajectory(tf=10.0)
loop.plot_diagram()
loop.plot_trajectory()
```

![closed-loop diagram](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/diagram_closed_loop.png)

The operator inserts the error junction, names the two sides `controller` and `plant`,
and the loop linearizes, animates and nests like any other model. The catalog has
the plants a course needs, with their parameters and start as data (`Pendulum`,
`CartPole`, `DoublePendulum`, `UR5Manipulator`, `KinematicBicycle`, `UdeSRacecar`,
`Drone2D`, `Rocket`, `Boat2D`, …) and the control laws (`PID`, `Impedance`,
`StateFeedback`, `ComputedTorque`, `SlidingMode`,
`ModelPredictiveController`, `NeuralPolicyController`):

```python
from minilink import Pendulum

pendulum = Pendulum()
pendulum.params["l"] = 2.0
pendulum.x0[0] = 2.0
pendulum.compute_trajectory(tf=10.0)
pendulum.animate()
```

## One toolbox, every System

Every tool is a verb on a `System`, a leaf or a whole diagram alike: simulate, analyze and
design straight from the system; plan, learn and evaluate from a `PlanningProblem` built on
it. Every controller a tool returns is a `System` again, and the same equations run on NumPy
or trace under JAX. Each pipeline has its own figure in
[docs/pipelines.md](https://github.com/alx87grd/minilink/blob/main/docs/pipelines.md).

![Minilink on one page: the System, the PlanningProblem and the optimizer, six tools and JAX](https://raw.githubusercontent.com/alx87grd/minilink/main/docs/_static/pipeline_map.png)

| | |
| --- | --- |
| Simulate, plot, animate, play | `compute_trajectory`, `plot_trajectory`, `plot_phase_plane`, `animate`, `game` |
| Analyze | `linearize`, `bode`, `pzmap`, `margins`, `root_locus`, `step_response` |
| Design | `lqr`, `place`, `kalman`, `lqg`, `PID`, `Impedance` |
| Plan, evaluate, optimize and learn | `DynamicProgrammingPlanner`, `RRTPlanner`, `TrajectoryOptimizationPlanner`, `ModelPredictiveController`, `ReinforcementLearningPlanner`, `evaluate`, `compare`, `Optimizer` |

## One problem, every planner

A `PlanningProblem` is a system, a cost and boundary sets. Every planner takes the
same problem, from LQR to value iteration, tree search, trajectory optimization and
reinforcement learning, and returns a `PlanningSolution` whose policy is a controller:

```python
from minilink import (
    BallSet, DynamicProgrammingPlanner, PlanningProblem, QuadraticCost,
    RRTPlanner, TrajectoryOptimizationPlanner, compare,
)

plant = Pendulum()
x_down, x_up = np.array([0.0, 0.0]), np.array([np.pi, 0.0])
problem = PlanningProblem(
    sys=plant, x_start=x_down, x_goal=x_up, Xf=BallSet(x_up, 0.2), tf=4.0,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=x_up),
    X=plant.state.box, U=plant.input_box,
)

vi = DynamicProgrammingPlanner(problem, x_grid=(101, 101), u_grid=(11,), dt=0.05)
rrt = RRTPlanner(problem, seed=0)
opt = TrajectoryOptimizationPlanner(problem, n_steps=40, transcription="direct_collocation")

vi_sol = vi.solve()                   # value iteration: the policy is a controller
rrt_sol = rrt.solve()                 # kinodynamic tree search, bang-bang inputs
opt_sol = opt.solve()                 # direct collocation

loop = vi_sol.policy @ plant          # the script closes the loop
print(compare(problem, VI=vi_sol, RRT=rrt_sol, TrajOpt=opt_sol, dt=0.05))
```

## Fast and differentiable with JAX, optional

The same `f` traces under JAX. One compiled evaluator gives exact derivatives,
batched rollouts, and gradients of a closed-loop cost with respect to any gain or
physical parameter:

```python
import jax
from minilink import Optimizer

ev = plant.compile(backend="jax")
x0s = np.tile([1.0, 0.0], (1000, 1))              # 1000 rollouts released at 1 rad
lengths = np.linspace(0.5, 2.0, 1000)             # one rod length per rollout

A = plant.jacobian("f", "x", x_up)                # exact linearization
S = plant.jacobian("f", "params", x_up)           # sensitivity to each physical parameter
xs = ev.rollout_batch(x0s, n_steps=1000, dt=0.005,
                      params=dict(plant.params, l=lengths))   # a family of rod lengths, one call

tuning = PlanningProblem(plant, x_start=[1.0, 0.0], cost=QuadraticCost.from_system(plant), tf=4.0)
pe = tuning.compile(PID(2.0, 0.5, 1.0), dt=0.02)
dJ = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)  # gradient w.r.t. every gain and plant param
program = pe.program(wrt=["controller.Kp", "controller.Ki", "controller.Kd"])
result = Optimizer(program, method="auto").solve()            # L-BFGS on the closed-loop cost
```

1000 rollouts of 1000 RK4 steps take 27 ms as a compiled batch, against about
32 s one step at a time in Python (Apple M4 Max, the showcase notebook).
Derivatives are exact to machine precision, float64 by default. See
[07_compile](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/07_compile.ipynb)
and the
[JAX showcase](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_jax.ipynb).

## Install

Python 3.10+. **Recommended: conda**, everything included (JAX, Graphviz, Ipopt, 3-D
animation, Jupyter):

```bash
git clone https://github.com/alx87grd/minilink.git && cd minilink
conda env create -f environment.yml && conda activate minilink
conda env config vars set PYTHONPATH="$PWD" && conda deactivate && conda activate minilink
```

**Colab**: open any notebook with "Open in Colab", or start yours with

```python
!git clone https://github.com/alx87grd/minilink
import sys; sys.path.insert(0, "/content/minilink")
```

**pip**: `pip install minilink` runs simulation, plots, LQR and value iteration on NumPy,
SciPy and Matplotlib; `pip install "minilink[full]"` adds JAX, plotly, meshcat, pygame,
sympy and gymnasium. Two optional tools need a system install pip cannot do:
`plot_diagram()` needs Graphviz ([install](https://graphviz.org/download/), then
`pip install graphviz`) and the Ipopt solver needs `conda install -c conda-forge cyipopt`
(trajectory optimization runs on SciPy without it).

## Learn more

- [Showcase notebook](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb), the tool ladder on real plants
- [JAX showcase](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_jax.ipynb), write `f` once, get every gradient
- [From RL to Bode showcase](https://github.com/alx87grd/minilink/blob/main/examples/tutorial/showcase_from_rl_to_bode.ipynb), six-axis robot, impedance loop, neural policy, frequency response and Lyapunov certificates
- [Tutorial series 00–11](https://github.com/alx87grd/minilink/tree/main/examples/tutorial), one notebook per package (core dynamics to reinforcement learning)
- [Teaching notebooks](https://github.com/alx87grd/minilink/tree/main/examples/teaching), swing-up, DP, PPO
- [Examples index](https://github.com/alx87grd/minilink/blob/main/examples/README.md), demos and projects by chapter
- [API reference](https://alx87grd.github.io/minilink/)
- Contributors: [tests/README.md](https://github.com/alx87grd/minilink/blob/main/tests/README.md) and the contracts under [docs/dev](https://github.com/alx87grd/minilink/tree/main/docs/dev)

Minilink is the successor of [pyro](https://github.com/SherbyRobotics/pyro).
