import numpy as np

from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = -2.0

# Source input
step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([1.0])
step.params["step_time"] = 10.0

# Closed loop system
ctl = PendulumPDController()
ctl.params["Kp"] = 1000.0
ctl.params["Kd"] = 100.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")


# Unconnected controller -> plant
diagram.connect("step", "y", "controller", "ref")
diagram.name = "Pendulum alone"
diagram.plot_graphe()
diagram.compute_trajectory(tf=20)
diagram.animate()

# Open loop controller -> plant
diagram.connect("controller", "u", "plant", "u")
diagram.name = "Pendulum with Open Loop Controller"
diagram.plot_graphe()
diagram.compute_trajectory(tf=20)
diagram.animate()

# Closed loop controller -> plant
diagram.connect("plant", "y", "controller", "y")
diagram.name = "Closed Loop Pendulum "
diagram.plot_graphe()
diagram.compute_trajectory(tf=20)
diagram.animate()
