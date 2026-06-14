import numpy as np

from minilink.blocks.sources import Step, WhiteNoise
from minilink.control.linear import PDController
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import PendulumWithNoisePort

# Plant system
sys = PendulumWithNoisePort()

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
noise.params["var"] = 0.1
noise.params["mean"] = 0.0
noise.params["seed"] = 1

# Noisy measurement
noise2 = WhiteNoise(1)
noise2.params["var"] = 0.01
noise2.params["mean"] = 0.0
noise2.params["seed"] = 2

# Closed loop system
ctl = PDController()
ctl.params["Kp"] = 1000.0
ctl.params["Kd"] = 100.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")
diagram.add_subsystem(noise, "noise")
diagram.add_subsystem(noise2, "noise2")

diagram.connect("step", "y", "controller", "r")
diagram.connect("controller", "u", "plant", "u")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("noise", "y", "plant", "w")
diagram.connect("noise2", "y", "plant", "v")

diagram.plot_diagram()


diagram.name = "Pendulum with Noise"

# diagram2.compute_trajectory(tf=20) # takes forever with scipy solver
diagram.compute_trajectory(tf=20, solver="euler", dt=0.01, show=False)

diagram.plot_trajectory(
    signals=("step:y", "x", "plant:y", "controller:u", "noise:y", "noise2:y"),
    backend="matplotlib",
)

# automatic
diagram.plot_trajectory()
