"""Estimate where a saturating LQR loop holds a pendulum upright: its region of attraction."""

import numpy as np

from minilink import DiagramSystem, Pendulum, Saturation, lqr
from minilink.analysis import plot_region_of_attraction, region_of_attraction

TORQUE = 8.0  # Nm, against m g l = 9.81 Nm: too weak to hold the pendulum far over
Q = np.diag([1.0, 0.01])  # try diag([1000, 10]): faster poles, a smaller region
X_UP = np.array([np.pi, 0.0])  # upright

plant = Pendulum()
plant.inputs["u"].lower = np.array([-TORQUE])
plant.inputs["u"].upper = np.array([TORQUE])
plant.state.lower = np.array([-4 * np.pi, -20.0])
plant.state.upper = -plant.state.lower

# The actuator limit is a block of its own, so the region sees the
# saturation the real loop has, not the linear law the designer wrote.
loop = DiagramSystem()
loop.name = "Pendulum under LQR"
loop.add_subsystem(lqr(plant, Q=Q, R=np.eye(1), x_bar=X_UP), "controller")
loop.add_subsystem(Saturation(-TORQUE, TORQUE), "limit")
loop.add_subsystem(plant, "plant")
loop.connect("plant", "y", "controller", "x")
loop.connect("controller", "u", "limit", "u")
loop.connect("limit", "y", "plant", "u")
loop.plot_diagram()

# Find the equilibrium, linearize there, solve A'P + PA = -Q, then shrink the
# level set of V until V decreases at every sampled state inside it.
region = region_of_attraction(loop, X_UP)
print(f"V <= {region.level:.4g}, reaching {region.half_width} per state")

plot_region_of_attraction(loop, region)
plot_region_of_attraction(loop, region, basin=True, trajectories=[X_UP + [1.0, 0.0]])
