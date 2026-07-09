"""Pendulum regulation with Pyro sliding-mode control — continuous-time closed loop.

Run from repo root::

    python examples/scripts/control/demo_sliding_mode_pendulum.py

Pyro law on ``[q; dq]`` feedback (``closed_loop_qdq`` inserts ``Mux(q, dq) → smc.y``)::

    ref.r ─────────────► smc.r              ([q_d; dq_d])
    plant.q, plant.dq ─► smc.y              ([q; dq])
    smc.u ─────────────► plant.u            (τ)

Fixed-step RK4 is used here because SciPy adaptive solvers hang or overflow on
this closed loop.

**Known bug (continuous path):** ``DiagramSystem`` integration uses algebraic
``f_ivp`` feedback (``u`` re-evaluated at every RK sub-step, not sample-hold).
With these ICs/gains, ``sign(s)`` often stays one-signed, so ``tau`` looks
smooth (inverse dynamics + constant ``-K sign(s)``) rather than chattering.
SciPy and RK4 then look deceptively similar. Diagnostic:
``scratch/confirm_smc_solver_bug.py``. Fix tracked in ROADMAP §5.2.
"""

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([0.2, 0.0])

ref = Step(
    initial_value=np.array([0.0, 0.0]),
    final_value=np.array([np.pi + 0.25, 0.0]),
    step_time=0.0,
)

smc = SlidingModeController(plant, lam=2.0, gain=8.0, nab=0.15)

diagram = ref >> closed_loop_qdq(smc, plant)

diagram.plot_diagram()
diagram.compute_trajectory(tf=8.0, dt=0.1, solver="rk4_fixedsteps")
# diagram.compute_trajectory(tf=8.0, dt=0.1)
diagram.plot_trajectory()
diagram.animate()
