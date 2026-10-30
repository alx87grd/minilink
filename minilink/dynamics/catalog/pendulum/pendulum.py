"""Single pendulums: actuated, with noise ports, inverted, and two uncoupled ones."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2
from minilink.core.system import fastest_time_constant
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Circle,
    Rod,
    TorqueArrow,
)

###############################################################################
# Public API
###############################################################################


class Pendulum(MechanicalSystem):
    """Single actuated pendulum."""

    def __init__(self, length=1.0, mass=1.0):
        super().__init__(dof=1, actuators=1)

        # The plant's own range: angles and rates within ±2π, torques within ±5
        self.state.lower_bound = np.full(self.n, -2 * np.pi)
        self.state.upper_bound = np.full(self.n, 2 * np.pi)
        self.inputs["u"].lower_bound = np.full(self.inputs["u"].dim, -5.0)
        self.inputs["u"].upper_bound = np.full(self.inputs["u"].dim, 5.0)

        # Parameters
        self.params = {
            "m": mass,
            "l": length,
            "I": 1.0,
            "gravity": 9.81,
            "d": 0.0,
        }

        # Signal labels and units
        self.state.labels = ["theta", "dtheta"]
        self.state.units = ["rad", "rad/s"]
        self.inputs["u"].labels = ["tau"]
        self.inputs["u"].units = ["Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: no camera_scale, so the view follows params["l"]
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_plot_axes = (0, 1)

    @property
    def solver_info(self) -> dict:
        """The solver hints, the smallest time constant read from the current ``params`` (:meth:`fastest_mode`)."""
        info = self.own_solver_info
        info["smallest_time_constant"] = self.fastest_mode()

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

    def fastest_mode(self) -> float:
        """The solver hint: the time constant of the swing about the hanging rest, in seconds.

        With no damping the swing's modes are ``±i √(m g l / (m l² + I))``, so
        ``τ = √((m l² + I) / (m g l))``, ``√(l/g)`` for a point mass; the damping
        ``d`` moves them, and the upright's modes have the same magnitude.
        """
        params = self.params

        m = params["m"]
        l = params["l"]
        I = params["I"]
        gravity = params["gravity"]
        d = params["d"]

        # (m l² + I) θ̈ = −m g l θ − d θ̇, linearized about θ = 0 on the state [θ, θ̇]
        H = m * l**2 + I
        A = np.array([[0.0, 1.0], [-m * gravity * l / H, -d / H]])
        tau = fastest_time_constant(np.linalg.eigvals(A))

        return tau

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        I = params["I"]

        H = xp.array([[m * l**2 + I]])

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # A single joint has no Coriolis or centrifugal coupling: C = 0
        C = xp.zeros((1, 1))

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]

        theta = q[0]

        g = xp.array([m * gravity * l * xp.sin(theta)])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        d = params["d"]

        omega = dq[0]

        tau_d = xp.array([d * omega])

        return tau_d

    def get_kinematic_geometry(self):
        length = self.params["l"]

        return {"link": link_geometry(length)}

    def tf(self, x, u, t=0, params=None):
        return {"link": SE2(0.0, 0.0, x[0])}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        length = params["l"]
        torque = u[0]
        max_torque = self.inputs["u"].upper_bound[0]

        # A third of a turn at the torque limit
        sweep = torque * (2.0 * np.pi / 3.0) / max_torque
        arc = torque_arc(sweep, length / 5.0)

        return {"link": [arc]}


class NoisyPendulum(Pendulum):
    """Single actuated pendulum with process and measurement noise ports.

    ``d`` injects disturbance torque into the dynamics; ``n`` corrupts the
    measured angular velocity in ``y``.
    """

    def __init__(self):
        super().__init__()

        # Ports: the disturbance torque d and the measurement noise n
        self.add_input_port("d", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("n", dim=1, nominal_value=np.array([0.0]))

        # Signal labels and units
        self.inputs["d"].labels = ["disturbance"]
        self.inputs["d"].units = ["Nm"]
        self.inputs["n"].labels = ["noise"]
        self.inputs["n"].units = ["rad/s"]

        # The measurement reads the noise port
        self.outputs["y"].dependencies = ("n",)

    def B(self, q, params=None):
        xp = array_module(q)

        # The torque acts on the joint
        B = xp.array([[1.0]])

        return B

    def generalized_force(self, q, v, u, t=0.0, params=None):
        # The ports in order: the torque, then the disturbance
        tau = u[:1]
        d = u[1:2]

        B = self.B(q, params)

        # The disturbance enters with the actuator torque: B (τ + d)
        force = B @ (tau + d)

        return force

    def h(self, x, u, t=0.0, params=None):
        xp = array_module(x, u)

        k = self.dof

        # The third port: the measurement noise
        n = u[2:]

        # y = [θ; θ̇ + n]: the noise corrupts the measured angular velocity
        y = xp.concatenate([x[:k], x[k : k + 1] + n[0], x[k + 1 :]])

        return y


class InvertedPendulum(Pendulum):
    """Single actuated pendulum with upward zero-angle equilibrium."""

    def __init__(self):
        super().__init__()

    def g(self, q, params=None):
        # The upright zero angle flips the sign of the hanging pendulum's gravity term
        g = -super().g(q, params)

        return g

    def tf(self, x, u, t=0, params=None):
        xp = array_module(x)

        # θ is measured from the upward vertical (zero angle = upright); the shared
        # geometry draws angles from the downward vertical, so add π
        upright = xp.array([x[0] + xp.pi, x[1]])

        return super().tf(upright, u, t, params)


class TwoIndependentPendulums(MechanicalSystem):
    """Two uncoupled single pendulums sharing the same parameters."""

    def __init__(self):
        super().__init__(dof=2, actuators=2)

        # The plant's own range: angles and rates within ±2π, torques within ±5
        self.state.lower_bound = np.full(self.n, -2 * np.pi)
        self.state.upper_bound = np.full(self.n, 2 * np.pi)
        self.inputs["u"].lower_bound = np.full(self.inputs["u"].dim, -5.0)
        self.inputs["u"].upper_bound = np.full(self.inputs["u"].dim, 5.0)

        # Parameters
        self.params = {
            "m": 1.0,
            "l": 1.0,
            "I": 1.0,
            "gravity": 9.81,
            "d": 0.0,
        }

        # Signal labels and units
        self.state.labels = ["theta1", "theta2", "dtheta1", "dtheta2"]
        self.state.units = ["rad", "rad", "rad/s", "rad/s"]
        self.inputs["u"].labels = ["tau1", "tau2"]
        self.inputs["u"].units = ["Nm", "Nm"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics
        self.camera_target = np.array([0.0, 0.0, 0.0])
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 2.0

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        I = params["I"]

        # Each pendulum's inertia about its own pivot, uncoupled
        inertia = m * l**2 + I
        H = xp.diag(xp.array([inertia, inertia]))

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # Uncoupled single joints: C = 0
        C = xp.zeros((2, 2))

        return C

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]

        g = m * gravity * l * xp.sin(q)

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        d = params["d"]

        tau_d = d * xp.asarray(dq)

        return tau_d

    def get_kinematic_geometry(self):
        length = self.params["l"]

        geo = {}
        for i in (0, 1):
            geo[f"link{i}"] = link_geometry(length)

        return geo

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        length = params["l"]

        # The two pivots, 0.6 l either side of the origin
        anchors = [-0.6 * length, 0.6 * length]

        return {
            f"link{i}": SE2(anchor_x, 0.0, x[i]) for i, anchor_x in enumerate(anchors)
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        length = params["l"]
        torque_radius = length / 5.0
        max_torque = self.inputs["u"].upper_bound[0]

        # A third of a turn at the torque limit
        sweep_scale = 2.0 * np.pi / 3.0 / max_torque

        dyn = {}
        for i in (0, 1):
            arc = torque_arc(u[i] * sweep_scale, torque_radius)
            dyn[f"link{i}"] = [arc]

        return dyn


###############################################################################
# Internal machinery
###############################################################################


def link_geometry(length):
    """A hanging link: the pivot, the rod along −y, and the bob at its end."""
    radius = 0.08 * length
    pivot = Circle(radius=radius, center=[0.0, 0.0], color="blue", fill=True)
    rod = Rod(length=length, radius=0.03 * length, color="blue", linewidth=2)
    bob = Circle(radius=radius, center=[0.0, -length], color="blue", fill=True)
    link = [pivot, rod, bob]

    return link


def torque_arc(sweep, radius):
    """A torque arc about the pivot, starting along the hanging link."""
    arc = TorqueArrow(
        sweep=sweep,
        radius=radius,
        head_ratio=0.4,
        color="red",
        linewidth=2,
    )
    arc.local_transform = SE2(0.0, 0.0, -np.pi / 2.0)

    return arc


if __name__ == "__main__":
    sys = Pendulum()
    sys.x0 = np.array([0.7, 0.0])
    sys.compute_forced(
        lambda t: np.array([2.0 * np.sin(t)]),
        tf=5.0,
        n_steps=160,
        show=True,
        verbose=False,
    )
    sys.plot_phase_plane()
    sys.animate()
