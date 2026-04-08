import numpy as np

from minilink.blocks.examples import Pendulum, PendulumPDController
from minilink.blocks.sources import Step, WhiteNoise
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

# Noisy input
noise = WhiteNoise(1)
noise.params["var"] = 1.0
noise.params["mean"] = 0.0
noise.params["seed"] = 1

# Noisy measurement
noise2 = WhiteNoise(1)
noise2.params["var"] = 0.1
noise2.params["mean"] = 0.0
noise2.params["seed"] = 2

# Closed loop system
ctl = PendulumPDController()
ctl.params["Kp"] = 1000.0
ctl.params["Kd"] = 100.0

# Diagram
diagram2 = DiagramSystem()

diagram2.add_subsystem(step, "step")
diagram2.add_subsystem(ctl, "controller")
diagram2.add_subsystem(sys, "plant")
diagram2.add_subsystem(noise, "noise")
diagram2.add_subsystem(noise2, "noise2")

diagram2.connect("step", "y", "controller", "ref")
diagram2.connect("controller", "u", "plant", "u")
diagram2.connect("plant", "y", "controller", "y")
diagram2.connect("noise", "y", "plant", "w")
diagram2.connect("noise2", "y", "plant", "v")
diagram2.plot_graphe()

print("Running simulation... this is fast with euler solver")
diagram2.compute_trajectory(tf=20, solver="euler", dt=0.01)

print("Running simulation... this take forever with scipy solver")
diagram2.compute_trajectory(tf=20)
