"""Filtered PID step tracking with and without anti-windup."""

import numpy as np

from minilink.blocks.nonlinear import Saturation
from minilink.blocks.sources import Step
from minilink.control.pid import FilteredPIDController
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator

step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([4.0])
step.params["step_time"] = 3.0

sys = DoubleIntegrator()
sys.name = "sys"

pid = FilteredPIDController()
pid.params["kp"] = 5.0
pid.params["ki"] = 1.0
pid.params["kd"] = 3.0
pid.params["tau"] = 0.1
pid.params["u_min"] = -5.0
pid.params["u_max"] = 5.0

diagram = step >> pid @ sys

diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()

# Without anti-windup: external saturation clips u; the integrator keeps growing.
sat = Saturation()
sat.params["lower"] = -0.5
sat.params["upper"] = 0.5

pid2 = FilteredPIDController()
pid2.name = "pid2"
pid2.params["kp"] = pid.params["kp"]
pid2.params["ki"] = pid.params["ki"]
pid2.params["kd"] = pid.params["kd"]
pid2.params["tau"] = pid.params["tau"]
pid2.params["u_min"] = -np.inf
pid2.params["u_max"] = np.inf

diagram2 = step >> pid2 >> sat >> sys
diagram2.connect("sys", "y", "pid2", "y")
diagram2.plot_diagram()

diagram2.compute_trajectory(tf=20)
diagram2.plot_trajectory()
