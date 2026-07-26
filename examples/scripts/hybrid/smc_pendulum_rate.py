"""Pendulum regulation: continuous Pyro SMC vs hybrid sampled SMC.

Compares fixed-step RK4 closed loop with
:class:`~minilink.control.modelbased.SlidingModeController` against the same law
clocked at ``TS`` via :func:`~minilink.core.hybrid_composition.hybrid_closed_loop`
and :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`.

**Known issues (continuous path):** same as
``examples/scripts/control/sliding_mode_pendulum.py`` — auto **Euler** with
finer ``dt`` is the recommended continuous path; see
`DESIGN.md` (*Discontinuous closed loops — known issues*).

Run from the repo root::

    python examples/scripts/hybrid/smc_pendulum_rate.py
"""

import numpy as np

from minilink.control.modelbased import SlidingModeController
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

TF = 5.0
SIM_DT = 0.001
TS = 0.05
REF = np.array([0.0, 0.0])

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([np.pi + 0.25, 0.0])

# plant.compute_trajectory()
# plant.animate()

smc_params = {"lam": 2.0, "gain": 8.0, "nab": 0.15}
ctl = SlidingModeController(plant, **smc_params)

# diagram_ct = closed_loop_qdq(ctl, plant)
# result_ct = diagram_ct.compute_forced(
#     REF,
#     input_port_id="r",
#     t0=0.0,
#     tf=TF,
#     dt=SIM_DT,
#     solver="rk4_fixedsteps",
# )
# result_ct = diagram_ct.reconstruct_internal_signals(result_ct)

# diagram_ct.plot_trajectory()

hybrid = hybrid_closed_loop(
    ctl,
    plant,
    schedule=TS,
    computer_in="y",
    plant_out="y",
)
result_hy = hybrid.compute_forced(
    REF,
    input_port_id="r",
    t0=0.0,
    tf=TF,
    plant_dt_inner=SIM_DT,
)


hybrid.plot_diagram()

hybrid.plot_trajectory()
hybrid.animate()
