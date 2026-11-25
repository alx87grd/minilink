"""Upright inverted pendulum: equilibrium, classical and state-space analysis, LQR, region of attraction."""

import numpy as np

from minilink import (
    DiagramSystem,
    InvertedPendulum,
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

########################################################
### Tunables
########################################################

TF = 6.0  # [s]
TORQUE = 10.0  # [Nm]

########################################################
### The plant
########################################################

plant = InvertedPendulum()
plant.x0 = np.array([0.0, 0.0])
x_up = plant.x0.copy()
x_down = np.array([np.pi, 0.0])

########################################################
### Equilibrium and linearization
########################################################

x_eq = plant.find_equilibrium(x_guess=x_down)

lin = plant.linearize(x_eq)
print(lin)

########################################################
### Classical frequency design
########################################################

G = plant.transfer_function(x_bar=x_eq)
print(G)
plant.plot_pzmap(x_bar=x_eq)

C = Lead(K=1.0, z=2.0, p=20.0)
L = C >> G
L.plot_root_locus()
root_locus(L)

plant.plot_bode()

########################################################
### Controllability and observability
########################################################

_, n_ctrb = controllability(plant, x_eq)
_, n_obsv = observability(plant, x_eq)
print(n_ctrb, n_obsv)

########################################################
### LQR regulation
########################################################

Q = np.diag([1.0, 0.01])
R = np.diag([0.01])
controller = lqr(plant, Q=Q, R=R, x_bar=x_eq)
closed_loop = controller @ plant
closed_loop.plot_diagram()

plant.x0 = np.array([0.35, 0.2])
closed_loop.compute_trajectory(tf=TF)
closed_loop.plot_trajectory()
closed_loop.animate()

########################################################
### Modal shapes of the linearization
########################################################

plant.animate_modal(x_eq)

########################################################
### Region of attraction under torque saturation
########################################################

plant.inputs["u"].lower = np.array([-TORQUE])
plant.inputs["u"].upper = np.array([TORQUE])
plant.state.lower = np.array([-4 * np.pi, -20.0])
plant.state.upper = -plant.state.lower

loop = DiagramSystem()
loop.name = "Inverted pendulum under LQR"
loop.add_subsystem(lqr(plant, Q=Q, R=R, x_bar=x_eq), "controller")
loop.add_subsystem(Saturation(-TORQUE, TORQUE), "limit")
loop.add_subsystem(plant, "plant")
loop.connect("plant", "y", "controller", "x")
loop.connect("controller", "u", "limit", "u")
loop.connect("limit", "y", "plant", "u")
loop.plot_diagram()

region = region_of_attraction(loop, x_eq)
print(region)

plot_region_of_attraction(loop, region)
