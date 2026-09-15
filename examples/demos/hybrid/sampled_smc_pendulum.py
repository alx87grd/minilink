"""Pendulum regulation: continuous Pyro SMC vs hybrid sampled SMC."""

import numpy as np

from minilink import Pendulum, SlidingModeController
from minilink.core.hybrid_composition import hybrid_closed_loop

TF = 5.0
SIM_DT = 0.001
TS = 0.05
REF = np.array([0.0, 0.0])

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([np.pi + 0.25, 0.0])

smc_params = {"lam": 2.0, "gain": 8.0, "nab": 0.15}
ctl = SlidingModeController(plant, **smc_params)

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
