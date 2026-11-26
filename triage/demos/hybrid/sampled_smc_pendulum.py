"""Pendulum regulation: continuous Pyro SMC vs hybrid sampled SMC."""

import numpy as np

from minilink import Pendulum, SlidingMode

TF = 5.0
SIM_DT = 0.001
TS = 0.05
REF = np.array([0.0, 0.0])

plant = Pendulum(length=1.0, mass=1.0)
plant.x0 = np.array([np.pi + 0.25, 0.0])

smc_params = {"lam": 2.0, "gain": 8.0, "nab": 0.15}
ctl = SlidingMode(plant, **smc_params)
ctl.inputs["r"].nominal_value = REF

# The law on its clock, the plant integrated in substeps of SIM_DT between samples
hybrid = (ctl % TS) @ plant
hybrid.compute_trajectory(tf=TF, dt=SIM_DT)


hybrid.plot_diagram()

hybrid.plot_trajectory()
hybrid.animate()
