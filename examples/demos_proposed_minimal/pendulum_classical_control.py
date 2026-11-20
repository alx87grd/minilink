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

TF = 6.0  # [s]
TORQUE = 10.0  # [Nm]

# The plant: angle measured from the upright vertical

plant = InvertedPendulum()
plant.x0 = np.array([0.0, 0.0])
x_up = plant.x0.copy()

# Equilibrium: f(x, u) = 0 at the nominal upright

x_eq = plant.find_equilibrium(x_guess=x_up)

# Linear model at the equilibrium: ẋ = A (x − x̄) + B u

lin = plant.linearize(x_eq)
print(lin)

# SISO channel torque → angle: G(s), poles and zeros

G = plant.transfer_function(x_bar=x_eq)
print(G)
plant.plot_pzmap(x_bar=x_eq)

# Classical design: lead compensator, root locus of L = C G

C = Lead(K=1.0, z=2.0, p=20.0)
L = C >> plant
L.plot_root_locus()
root_locus(L)

# Structure: full-state controllability and observability before LQR

_, n_ctrb = controllability(plant, x_eq)
_, n_obsv = observability(plant, x_eq)
print(n_ctrb, n_obsv)

# State feedback: u = −K (x − x̄)

Q = np.diag([1.0, 0.01])
R = np.eye(1)
controller = lqr(plant, Q=Q, R=R, x_bar=x_eq)
closed_loop = controller @ plant
closed_loop.plot_diagram()

# Nonlinear regulation from a tilted start

plant.x0 = np.array([0.35, 0.2])
closed_loop.compute_trajectory(tf=TF)
closed_loop.plot_trajectory()
closed_loop.animate()

# Modes of the linearization at the equilibrium

plant.animate_modal(x_eq)

# Region of attraction: LQR through a torque saturator

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

region = region_of_attraction(loop, x_eq)
print(region)
simulated = region_of_attraction(loop, x_eq, method="simulation")
print(simulated)

plot_region_of_attraction(loop, region)
plot_region_of_attraction(
    loop,
    {"Lyapunov": region, "simulation": simulated},
    basin=True,
    trajectories=[x_eq + [0.4, 0.0]],
)
