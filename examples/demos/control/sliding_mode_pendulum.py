"""Pendulum regulation with Pyro sliding-mode control — continuous-time closed loop.

Run from repo root::

    python examples/demos/control/sliding_mode_pendulum.py

Pyro law on ``[q; dq]`` feedback (``closed_loop_qdq`` inserts ``Mux(q, dq) → smc.y``)::

    ref.r ─────────────► smc.r              ([q_d; dq_d])
    plant.q, plant.dq ─► smc.y              ([q; dq])
    smc.u ─────────────► plant.u            (τ)

**Discontinuous closed loop:** prefer auto **Euler** (``solver=None``) or
``solver="euler"`` with a small ``dt``. See `DESIGN.md` — *Discontinuous
closed loops — known issues*. Solver comparisons live in
``tests/unittest/test_discontinuous_solvers.py``.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

plant = Pendulum(length=1.0, mass=1.0)
model = Pendulum(length=1.0, mass=0.5)
plant.x0 = np.array([1.0, 0.0])

ref = Step(
    initial_value=np.array([np.pi, 0.0]),
    final_value=np.array([0.0, 0.0]),
    step_time=0.5,
)

smc = SlidingModeController(model, lam=20.0, gain=8.0, nab=0.15)

diagram = ref >> closed_loop_qdq(smc, plant)

diagram.plot_diagram()
traj = diagram.compute_trajectory(tf=10.0)
# traj = diagram.compute_trajectory(tf=10.0, dt=0.01, solver="euler")
diagram.plot_trajectory()
# diagram.animate()
