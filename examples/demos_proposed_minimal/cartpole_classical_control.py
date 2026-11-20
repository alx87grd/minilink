"""Upright cart-pole: equilibrium, classical and state-space analysis, LQR, region of attraction."""

import numpy as np

from minilink import (
    CartPole,
    DiagramSystem,
    Lead,
    Saturation,
    controllability,
    lqr,
    observability,
)
from minilink.analysis import (
    plot_region_of_attraction,
    region_of_attraction,
    root_locus,
)

TF = 8.0  # [s]
FORCE = 20.0  # [N]
N_SAMPLES = 40  # region-of-attraction grid per axis

# The plant: cart position, pole angle from downward vertical

plant = CartPole()
x_up = np.array([0.0, np.pi, 0.0, 0.0])
plant.x0 = x_up.copy()

# Equilibrium at the inverted pose

x_eq = plant.find_equilibrium(x_guess=x_up + 0.1)

# Linear model at the equilibrium

lin = plant.linearize(x_eq)
print(lin)

# SISO channel force → pole angle

G = plant.transfer_function(x_bar=x_eq)
print(G)
plant.plot_pzmap(x_bar=x_eq)

# Classical design: lead compensator on the angle loop

C = Lead(K=1.0, z=2.0, p=20.0)
L = C >> plant
L.plot_root_locus()
root_locus(L)

# Structure at the upright

_, n_ctrb = controllability(plant, x_eq)
_, n_obsv = observability(plant, x_eq)
print(n_ctrb, n_obsv)

# LQR balance about the upright

Q = np.diag([1.0, 10.0, 1.0, 1.0])
R = np.array([[0.1]])
controller = lqr(plant, Q=Q, R=R, x_bar=x_eq)
closed_loop = controller @ plant
closed_loop.plot_diagram()

# Regulation from a displaced start

plant.x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])
closed_loop.compute_trajectory(tf=TF)
closed_loop.plot_trajectory()
closed_loop.animate()

# Vibration modes of the linearization

plant.animate_modal(x_eq)

# Region of attraction with force saturation

plant.inputs["u"].lower = np.array([-FORCE])
plant.inputs["u"].upper = np.array([FORCE])

loop = DiagramSystem()
loop.name = "Cart-pole under LQR"
loop.add_subsystem(lqr(plant, Q=Q, R=R, x_bar=x_eq), "controller")
loop.add_subsystem(Saturation(-FORCE, FORCE), "limit")
loop.add_subsystem(plant, "plant")
loop.connect("plant", "y", "controller", "x")
loop.connect("controller", "u", "limit", "u")
loop.connect("limit", "y", "plant", "u")

region = region_of_attraction(loop, x_eq, n_samples=N_SAMPLES)
print(region)
plot_region_of_attraction(loop, region, basin=True, trajectories=[plant.x0])
