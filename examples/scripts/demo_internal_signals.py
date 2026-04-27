from minilink.graphical.plotting import plot_signals


import numpy as np

from minilink.control.pendulum_pd import PendulumPDController
from minilink.dynamics.pendulum.pendulum import Pendulum
from minilink.core.blocks.sources import Step, WhiteNoise
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
noise.params["var"] = 0.1
noise.params["mean"] = 0.0
noise.params["seed"] = 1

# Noisy measurement
noise2 = WhiteNoise(1)
noise2.params["var"] = 0.01
noise2.params["mean"] = 0.0
noise2.params["seed"] = 2

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

# diagram2.compute_trajectory(tf=20) # takes forever with scipy solver
traj = diagram.compute_trajectory(tf=20, solver="euler", dt=0.01, show=False)

traj_plus = diagram.compute_internal_signals(traj)

plot_signals(
    diagram,
    traj_plus,
    [
        {"sys": "step", "output": "y", "label": "ref"},
        {"sys": "plant", "state": "x", "label": "theta"},
        {"sys": "plant", "output": "y", "label": "y"},
        {"sys": "controller", "output": "u", "label": "u"},
        {"sys": "noise", "output": "y", "label": "w"},
        {"sys": "noise2", "output": "y", "label": "v"},
    ],
)
