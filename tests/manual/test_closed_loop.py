import numpy as np

from minilink.blocks.basic import Pendulum, PendulumPDController
from minilink.blocks.sources import Step
from minilink.core.diagram import DiagramSystem

# Plant system
sys = Pendulum()
sys.params["m"] = 1.0
sys.params["l"] = 5.0
sys.x0[0] = 2.0

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

diagram.connect("step", "y", "controller", "ref")

diagram.compute_trajectory(tf=20)

diagram.connect("controller", "u", "plant", "u")
# diagram.compile()

diagram.compute_trajectory(tf=20)

diagram.connect("plant", "y", "controller", "y")
# diagram.compile()

diagram.compute_trajectory(tf=20)

diagram.plot_graphe()


diagram.animate()
# renderer = "meshcat"
# renderer = "pygame"
# diagram.animate(renderer=renderer)
