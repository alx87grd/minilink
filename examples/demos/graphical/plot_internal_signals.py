"""Choose which internal signals a diagram plot shows."""

import numpy as np

from minilink import ImpedanceController, Pendulum, Step

plant = Pendulum()
plant.params["l"] = 5.0
plant.x0[0] = 2.0

ref = Step(initial_value=np.array([0.0]), final_value=np.array([1.0]), step_time=10.0)
ctl = ImpedanceController()
ctl.params["Kp"] = 1000.0
ctl.params["Kd"] = 100.0

diagram = ref >> ctl @ plant
diagram.plot_diagram()
diagram.compute_trajectory(tf=20, show=False)

diagram.plot_trajectory(signals=("ref:y", "x", "sys:y", "ctl:u"))  # chosen
diagram.plot_trajectory()  # automatic selection
