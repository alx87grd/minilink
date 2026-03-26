import numpy as np

from minilink.blocks.tests import Pendulum, PendulumPDController
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
# noise.show_noise_signal()

# Noisy measurement
noise2 = WhiteNoise(1)
noise2.params["var"] = 0.1
noise2.params["mean"] = 0.0
noise2.params["seed"] = 2
# noise2.show_noise_signal()

# Closed loop system
ctl = PendulumPDController()
ctl.params["Kp"] = 1000.0
ctl.params["Kd"] = 100.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")
diagram.add_subsystem(noise, "noise")
diagram.add_subsystem(noise2, "noise2")

diagram.connect("step", "y", "controller", "ref")
diagram.connect("controller", "u", "plant", "u")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("noise", "y", "plant", "w")
diagram.connect("noise2", "y", "plant", "v")
diagram.plot_graphe()


diagram.name = "Pendulum with Noise"
diagram.compute_trajectory(tf=20)

diagram.name = "Pendulum with Noise (0.001)"
diagram.subsystems["noise"].params["var"] = 0.001
diagram.compute_trajectory(tf=20)

diagram.name = "Pendulum with Noise (10.0)"
diagram.subsystems["noise"].params["var"] = 10.0
diagram.compute_trajectory(tf=20)
