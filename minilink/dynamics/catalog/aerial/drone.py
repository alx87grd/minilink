import numpy as np

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


def _drone_body(width=1.0, height=0.2):
    """Flat rectangular drone body centered on the c.g."""
    return Box(
        length_x=width, length_y=height, length_z=0.08, color="black", opacity=0.9
    )


class Drone2D(MechanicalSystem):
    """Planar drone with two vertical body-frame thrusters."""

    def __init__(self):
        super().__init__(dof=3, actuators=2)
        self.name = "Planar Drone"
        self.params = {
            "mass": 1.0,
            "inertia": 0.1,
            "thruster_offset": 0.5,
            "gravity": 9.81,
            "cda": 0.1,
        }
        self.state.labels = ["x", "y", "theta", "vx", "vy", "omega"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]
        self.inputs["u"].labels = ["T1", "T2"]
        self.inputs["u"].units = ["N", "N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.width = 1.0
        self.height = 0.2
        self.camera_scale = 3.0
        self.camera_follow_frame = "body"

    def H(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        inertia = params["inertia"]

        # diagonal translational and rotational inertia
        return np.diag([mass, mass, inertia])

    def C(self, q, dq, params=None):
        return np.zeros((3, 3))

    def B(self, q, params=None):
        params = self.params if params is None else params
        offset = params["thruster_offset"]
        s, c = np.sin(q[2]), np.cos(q[2])

        # both thrusters push along the body vertical; their difference yaws
        # fmt: off
        return np.array([
            [    -s,     -s],
            [     c,      c],
            [-offset, offset],
        ])
        # fmt: on

    def g(self, q, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        gravity = params["gravity"]

        # weight pulls down along +y in screen coordinates
        return np.array([0.0, mass * gravity, 0.0])

    def d(self, q, dq, u=None, t=0.0, params=None):
        params = self.params if params is None else params
        cda = params["cda"]

        # quadratic aero drag on translation plus light linear damping on all DOF
        return np.array(
            [
                cda * dq[0] * abs(dq[0]) + 0.01 * dq[0],
                cda * dq[1] * abs(dq[1]) + 0.01 * dq[1],
                0.01 * dq[2],
            ]
        )

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=20.0)],
            "body": [_drone_body(width=1.2, height=0.18)],
            "center": [Point(color="blue", marker="o", size=5)],
        }

    def tf(self, x, u, t=0, params=None):
        q = x[:3]
        return {
            "body": SE2(q[0], q[1], q[2]),
            "center": SE2(q[0], q[1], 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        offset = self.params["thruster_offset"]
        scale = 0.08
        return {
            "body": [
                Arrow(
                    base=(-offset, 0.0),
                    vector=(0.0, 1.0),
                    scale=scale * u[0],
                    color="red",
                    linewidth=2,
                ),
                Arrow(
                    base=(offset, 0.0),
                    vector=(0.0, 1.0),
                    scale=scale * u[1],
                    color="red",
                    linewidth=2,
                ),
            ]
        }


class Drone2DWithSideThruster(Drone2D):
    """Planar drone with an added lateral body-frame thruster."""

    def __init__(self):
        super().__init__()
        self.name = "Planar Drone With Side Thruster"
        self.inputs.clear()
        self.add_input_port(
            "u",
            dim=3,
            labels=["T1", "T2", "TL"],
            units=["N", "N", "N"],
        )

    def B(self, q, params=None):
        theta = q[2]

        # extend the two-thruster matrix with a body-lateral thruster column
        B = np.zeros((3, 3))
        B[:, :2] = super().B(q, params)
        B[0, 2] = np.cos(theta)
        B[1, 2] = np.sin(theta)
        return B

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        dynamic = super().get_dynamic_geometry(x, u[:2], t)
        dynamic["body"].append(
            Arrow(
                base=(0.0, 0.0),
                vector=(1.0, 0.0),
                scale=0.08 * u[2],
                color="orange",
                linewidth=2,
            )
        )
        return dynamic


class SpeedControlledDrone2D(DynamicSystem):
    """Planar drone abstraction with velocity inputs."""

    def __init__(self):
        super().__init__(n=2, input_dim=2, output_dim=2, expose_state=True)
        self.name = "Speed Controlled Planar Drone"
        self.state.labels = ["x", "y"]
        self.state.units = ["m", "m"]
        self.inputs["u"].labels = ["vx", "vy"]
        self.inputs["u"].units = ["m/s", "m/s"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.camera_scale = 10.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        # the commanded velocity is the position rate directly
        return np.asarray(u)

    def h(self, x, u, t=0.0, params=None):
        return x

    def get_kinematic_geometry(self):
        return {"body": [_drone_body(width=1.0, height=0.18)]}

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], x[1], 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return {
            "body": [
                Arrow(
                    base=(0.0, 0.0),
                    vector=(u[0], u[1]),
                    scale=0.25,
                    color="red",
                    linewidth=2,
                )
            ]
        }


class ConstantSpeedHelicopterTunnel(DynamicSystem):
    """Constant-speed tunnel helicopter with vertical force input."""

    def __init__(self):
        super().__init__(n=3, input_dim=1, output_dim=3, expose_state=True)
        self.name = "Constant Speed Helicopter Tunnel"
        self.params = {
            "mass": 1.0,
            "vx": 10.0,
        }
        self.state.labels = ["dy", "y", "x"]
        self.state.units = ["m/s", "m", "m"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphic parameters
        self.camera_scale = 12.0
        self.camera_follow_frame = "body"

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        mass = params["mass"]
        vx = params["vx"]

        # vertical force drives altitude; horizontal cruise speed stays constant
        return np.array([u[0] / mass, x[0], vx])

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
        return {
            "body": [
                Arrow(
                    base=(0.6, 0.0),
                    vector=(0.0, u[0]),
                    scale=0.2,
                    color="red",
                    linewidth=2,
                )
            ]
        }


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
