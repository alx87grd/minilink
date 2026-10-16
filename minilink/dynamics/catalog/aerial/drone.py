"""Planar drones: two vertical thrusters, a lateral thruster, velocity inputs, the tunnel helicopter."""

from minilink.core.backends import array_module
from minilink.core.kinematics import SE2, translation
from minilink.core.system import DynamicSystem
from minilink.dynamics.abstraction.mechanical import MechanicalSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    Point,
    ground_line,
)

###############################################################################
# Public API
###############################################################################


class Drone2D(MechanicalSystem):
    """Planar drone with two vertical body-frame thrusters."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Drone"

        # Parameters
        self.params = {
            "mass": 1.0,
            "inertia": 0.1,
            "thruster_offset": 0.5,
            "gravity": 9.81,
            "cda": 0.1,
        }

        # Signal labels and units
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["T1", "T2"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a nominal body size, and a camera that follows the body
        self.width = 1.0
        self.height = 0.2
        self.camera_scale = 3.0
        self.camera_follow_frame = "body"

    def H(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        inertia = params["inertia"]

        H = xp.diag(xp.array([mass, mass, inertia]))

        return H

    def C(self, q, dq, params=None):
        xp = array_module(q)

        # A constant inertia matrix: no Coriolis or centrifugal terms
        C = xp.zeros((3, 3))

        return C

    def B(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        offset = params["thruster_offset"]

        theta = q[2]
        s = xp.sin(theta)
        c = xp.cos(theta)

        # Both thrusters push along the body vertical (−sin θ, cos θ); their
        # difference, at the lever arm ±l, yaws the body
        # fmt: off
        B = xp.array([
            [    -s,     -s],
            [     c,      c],
            [-offset, offset],
        ])
        # fmt: on

        return B

    def g(self, q, params=None):
        params = self.params if params is None else params
        xp = array_module(q)

        mass = params["mass"]
        gravity = params["gravity"]

        # The weight m g along +y (g sits on the left side of the equation of motion)
        g = xp.array([0.0, mass * gravity, 0.0])

        return g

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(dq)

        cda = params["cda"]

        vx = dq[0]
        vy = dq[1]
        omega = dq[2]

        # Quadratic aerodynamic drag c_dA v|v| on each translation, plus a light linear
        # damping, 10⁻² on every axis
        tau_d = xp.array(
            [
                cda * vx * xp.abs(vx) + 0.01 * vx,
                cda * vy * xp.abs(vy) + 0.01 * vy,
                0.01 * omega,
            ]
        )

        return tau_d

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=20.0)],
            "body": [drone_body(width=1.2, height=0.18)],
            "center": [Point(color="blue", marker="o", size=5)],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]

        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # The thrust arrows, 0.08 m per newton, at each thruster along the body vertical
        offset = self.params["thruster_offset"]
        scale = 0.08
        thrust_1 = Arrow(
            base=(-offset, 0.0),
            vector=(0.0, 1.0),
            scale=scale * u[0],
            color="red",
            linewidth=2,
        )
        thrust_2 = Arrow(
            base=(offset, 0.0),
            vector=(0.0, 1.0),
            scale=scale * u[1],
            color="red",
            linewidth=2,
        )

        return {"body": [thrust_1, thrust_2]}


class Drone2DWithSideThruster(Drone2D):
    """Planar drone with an added lateral body-frame thruster."""

    def __init__(self):
        super().__init__()
        self.name = "Planar Drone With Side Thruster"

        # Ports: a third thrust input, the lateral thruster TL
        self.inputs.clear()
        self.add_input_port(
            "u",
            dim=3,
            labels=["T1", "T2", "TL"],
            units=["N", "N", "N"],
        )

    def B(self, q, params=None):
        xp = array_module(q)

        theta = q[2]

        # The lateral thruster pushes along the body horizontal (cos θ, sin θ)
        lateral = xp.array([xp.cos(theta), xp.sin(theta), 0.0])

        # B = [B_vertical  b_lateral]: the two vertical thrusters, then the lateral one
        B_vertical = super().B(q, params)
        B = xp.column_stack([B_vertical, lateral])

        return B

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # The two vertical thrust arrows, then the lateral one at the c.g.
        dynamic = super().get_dynamic_geometry(x, u[:2], t)
        lateral = Arrow(
            base=(0.0, 0.0),
            vector=(1.0, 0.0),
            scale=0.08 * u[2],
            color="orange",
            linewidth=2,
        )
        dynamic["body"].append(lateral)

        return dynamic


class SpeedControlledDrone2D(DynamicSystem):
    """Planar drone abstraction with velocity inputs."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Speed Controlled Planar Drone"

        # Signal labels and units
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a camera that follows the body
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        xp = array_module(u)

        # ẋ = u: the commanded velocity is the position rate
        dx = xp.asarray(u)

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {"body": [drone_body(width=1.0, height=0.18)]}

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # The commanded velocity, drawn from the c.g. at 0.25 m per m/s
        velocity = Arrow(
            base=(0.0, 0.0),
            vector=(u[0], u[1]),
            scale=0.25,
            color="red",
            linewidth=2,
        )

        return {"body": [velocity]}


class ConstantSpeedHelicopterTunnel(DynamicSystem):
    """Constant-speed tunnel helicopter with vertical force input."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Helicopter Tunnel"

        # Parameters
        self.params = {
            "mass": 1.0,
            "vx": 10.0,
        }

        # Signal labels and units
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: a camera that follows the body
        self.camera_scale = 12.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        mass = params["mass"]
        vx = params["vx"]

        dy = x[0]
        force = u[0]

        # ÿ = F / m drives the altitude, and the cruise speed ẋ = v_x stays constant
        dx = xp.array([force / mass, dy, vx])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {
            "body": [
                Box(
                    length_x=1.0, length_y=0.35, length_z=0.2, color="blue", opacity=0.9
                ),
                Circle(radius=0.08, center=[-0.3, -0.2, 0.0], color="black", fill=True),
                Circle(radius=0.08, center=[0.3, -0.2, 0.0], color="black", fill=True),
            ]
        }

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[2], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        # The vertical force, drawn just ahead of the body at 0.2 m per newton
        force = Arrow(
            base=(0.6, 0.0),
            vector=(0.0, u[0]),
            scale=0.2,
            color="red",
            linewidth=2,
        )

        return {"body": [force]}


###############################################################################
# Internal machinery
###############################################################################


def drone_body(width=1.0, height=0.2):
    """Flat rectangular drone body centered on the c.g."""
    return Box(
        length_x=width, length_y=height, length_z=0.08, color="black", opacity=0.9
    )


if __name__ == "__main__":
    sys = Drone2D()
    # sys = Drone2DWithSideThruster()
    # sys = SpeedControlledDrone2D()
    # sys = ConstantSpeedHelicopterTunnel()

    # sys.x0 = np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0])

    sys.inputs["u"].nominal_value[0] = 5.0
    sys.inputs["u"].nominal_value[1] = 5.1

    sys.compute_trajectory(tf=2.0)

    sys.animate()
