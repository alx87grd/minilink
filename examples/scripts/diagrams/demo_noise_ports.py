import numpy as np

from minilink.control.pendulum_pd import PendulumPDController
from minilink.core.blocks.sources import Step, WhiteNoise
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

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
noise = WhiteNoise()

# Noisy measurement
noise2 = WhiteNoise()

# Closed loop system
ctl = PendulumPDController()
ctl.params["Kp"] = 100.0
ctl.params["Kd"] = 50.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")
diagram.add_subsystem(noise, "process_noise")
diagram.add_subsystem(noise2, "measurement_noise")

diagram.connect("step", "y", "controller", "r")
diagram.connect("controller", "u", "plant", "u")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("process_noise", "y", "plant", "w")
diagram.connect("measurement_noise", "y", "plant", "v")
diagram.plot_graphe()


diagram.name = "Pendulum without Noise"
diagram.subsystems["process_noise"].params["var"] = 0.0
diagram.subsystems["measurement_noise"].params["var"] = 0.0
diagram.compute_trajectory(tf=20)

diagram.name = "Pendulum with Measurement Noise "
diagram.subsystems["process_noise"].params["var"] = 0.0
diagram.subsystems["measurement_noise"].params["var"] = 1.0
# diagram.subsystems["measurement_noise"].show_signal()
diagram.compute_trajectory(tf=20)

diagram.name = "Pendulum with Process Noise "
diagram.subsystems["process_noise"].params["var"] = 100.0
diagram.subsystems["process_noise"].params["sample_period"] = 0.2
diagram.subsystems["measurement_noise"].params["var"] = 0.0
# diagram.subsystems["process_noise"].show_signal()
diagram.compute_trajectory(tf=20)
