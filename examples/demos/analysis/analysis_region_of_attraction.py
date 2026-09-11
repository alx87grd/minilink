"""Certify where a saturating LQR loop is guaranteed to hold a pendulum upright."""

import numpy as np

from minilink import DiagramSystem, Pendulum, Saturation
from minilink.control.lqr import lqr_at_operating_point

TORQUE = 4.0  # Nm, against m g l = 9.81 Nm: too weak to hold the pendulum far over
Q = np.diag([1.0, 0.1])  # try diag([1000, 10]): faster poles, a smaller certificate
X_UP = np.array([np.pi, 0.0])  # upright

plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
plant.state.upper_bound = -plant.state.lower_bound

# The actuator limit is a block of its own, so the certificate sees the
# saturation the real loop has, not the linear law the designer wrote.
loop = DiagramSystem()
loop.name = "Pendulum under LQR"
loop.add_subsystem(lqr_at_operating_point(plant, X_UP, Q=Q, R=np.eye(1)), "ctl")
loop.add_subsystem(Saturation(-TORQUE, TORQUE), "limit")
loop.add_subsystem(plant, "plant")
loop.connect("plant", "y", "ctl", "x")
loop.connect("ctl", "u", "limit", "u")
loop.connect("limit", "y", "plant", "u")

# Find the equilibrium, linearize there, solve A'P + PA = -Q, then sweep the
# largest level set of V that still decreases along the true nonlinear loop.
certificate = loop.region_of_attraction(X_UP)
print(certificate)
print(certificate.verify())

certificate.plot(basin=True, verified=200)
