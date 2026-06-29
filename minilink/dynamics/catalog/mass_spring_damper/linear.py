import numpy as np

from minilink.dynamics.abstraction.state_space import StateSpaceSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    empty_transform,
    ground_line,
    identity_matrix,
    line_between_transform,
    scale_pose2d_matrix,
    spring_line,
    translation_matrix,
)
from minilink.graphical.animation.shapes_v2 import ArrowV2


def _mass_box(size=0.5, color="blue", opacity=0.9):
    return Box(
        length_x=size,
        length_y=size,
        length_z=0.2 * size,
        color=color,
        opacity=opacity,
    )


def _force_arrow_v2(force):
    """Honest force arrow keyed to the ``force`` frame (matches ``_force_arrow_transform``)."""
    if abs(force) < 1e-12:
        theta, length = 0.0, 0.0
    else:
        theta = 0.0 if force >= 0.0 else np.pi
        length = 0.3 * abs(force)
    d = np.array([np.cos(theta), np.sin(theta)])
    return [ArrowV2(base=(0.0, 0.0), vector=d, scale=length, color="red", linewidth=2)]


def _mass_output_matrix(count, output_mass):
    if output_mass < 1 or output_mass > count:
        output_mass = count
    C = np.zeros((1, 2 * count))
    C[0, output_mass - 1] = 1.0
    return C, f"x{output_mass}"


def _force_arrow_transform(x, force):
    if abs(force) < 1e-12:
        return scale_pose2d_matrix(x, 0.0, 0.0, 0.0)
    theta = 0.0 if force >= 0.0 else np.pi
    return scale_pose2d_matrix(x, 0.0, theta, 0.3 * abs(force))


class SingleMass(StateSpaceSystem):
    """Single linear mass-spring-damper model."""

    def __init__(self, mass=1.0, k=2.0, b=0.0):
        super().__init__(n=2, m=1, p=1, name="Single Mass Spring Damper")
        self.params = {"mass": float(mass), "k": float(k), "b": float(b)}
        self.camera_scale = 4.0

        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = ["x"]
        self.outputs["y"].units = ["m"]

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params
        mass, k, b = params["mass"], params["k"], params["b"]

        # mass * x'' + b * x' + k * x = u, state = [position, velocity]
        # fmt: off
        return np.array([
            [      0.0,       1.0],
            [-k / mass, -b / mass],
        ])
        # fmt: on

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params
        mass = params["mass"]

        # the force enters through the acceleration row
        return np.array(
            [
                [0.0],
                [1.0 / mass],
            ]
        )

    def C(self, t=0.0, params=None):
        # measure the position of the mass
        return np.array([[1.0, 0.0]])

    def D(self, t=0.0, params=None):
        return np.array([[0.0]])

    def get_kinematic_geometry(self):
        return [
            ground_line(length=8.0),
            spring_line(),
            _mass_box(size=0.6, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        mass_x = x[0]
        anchor = -2.0
        left_face = mass_x - 0.3
        spring = (
            line_between_transform([anchor, 0.0], [left_face, 0.0])
            if self.params["k"] != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring,
            translation_matrix(mass_x, 0.0, 0.0),
            _force_arrow_transform(mass_x + 0.35, u[0]),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {
            "world": [ground_line(length=8.0)],
            "spring": [spring_line()],
            "body": [_mass_box(size=0.6, color="blue")],
        }

    def tf_v2(self, x, u, t=0, params=None):
        mass_x = x[0]
        anchor = -2.0
        left_face = mass_x - 0.3
        spring = (
            line_between_transform([anchor, 0.0], [left_face, 0.0])
            if self.params["k"] != 0.0
            else empty_transform()
        )
        return {
            "world": identity_matrix(),
            "spring": spring,
            "body": translation_matrix(mass_x, 0.0, 0.0),
            "force": translation_matrix(mass_x + 0.35, 0.0, 0.0),
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        return {"force": _force_arrow_v2(u[0])}


class TwoMass(StateSpaceSystem):
    """Two-mass linear spring-damper chain with force on the second mass."""

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        super().__init__(n=4, m=1, p=1, name="Two Mass Spring Damper")
        self.output_mass = int(output_mass)
        self.params = {
            "m1": float(m),
            "m2": float(m),
            "k1": float(k),
            "k2": float(k),
            "b1": float(b),
            "b2": float(b),
        }
        self.camera_scale = 5.0

        _, output_label = _mass_output_matrix(2, self.output_mass)
        self.state.labels = ["x1", "x2", "dx1", "dx2"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params
        m1, m2 = params["m1"], params["m2"]
        k1, k2 = params["k1"], params["k2"]
        b1, b2 = params["b1"], params["b2"]

        # m1 x1'' = -(k1 + k2) x1 + k2 x2 - b1 x1';  m2 x2'' = k2 (x1 - x2) - b2 x2'
        # fmt: off
        return np.array([
            [            0.0,      0.0,      1.0,      0.0],
            [            0.0,      0.0,      0.0,      1.0],
            [-(k1 + k2) / m1,  k2 / m1, -b1 / m1,      0.0],
            [        k2 / m2, -k2 / m2,      0.0, -b2 / m2],
        ])
        # fmt: on

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params
        m2 = params["m2"]

        # the force enters on the second mass
        return np.array(
            [
                [0.0],
                [0.0],
                [0.0],
                [1.0 / m2],
            ]
        )

    def C(self, t=0.0, params=None):
        C, _ = _mass_output_matrix(2, self.output_mass)
        return C

    def D(self, t=0.0, params=None):
        return np.array([[0.0]])

    def get_kinematic_geometry(self):
        return [
            ground_line(length=10.0),
            spring_line(),
            spring_line(),
            _mass_box(size=0.55, color="green"),
            _mass_box(size=0.55, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        x1, x2 = x[0] - 2.0, x[1]
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.3, 0.0])
            if self.params["k1"] != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring1,
            line_between_transform([x1 + 0.3, 0.0], [x2 - 0.3, 0.0]),
            translation_matrix(x1, 0.0, 0.0),
            translation_matrix(x2, 0.0, 0.0),
            _force_arrow_transform(x2 + 0.35, u[0]),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {
            "world": [ground_line(length=10.0)],
            "spring1": [spring_line()],
            "spring2": [spring_line()],
            "body1": [_mass_box(size=0.55, color="green")],
            "body2": [_mass_box(size=0.55, color="blue")],
        }

    def tf_v2(self, x, u, t=0, params=None):
        x1, x2 = x[0] - 2.0, x[1]
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.3, 0.0])
            if self.params["k1"] != 0.0
            else empty_transform()
        )
        return {
            "world": identity_matrix(),
            "spring1": spring1,
            "spring2": line_between_transform([x1 + 0.3, 0.0], [x2 - 0.3, 0.0]),
            "body1": translation_matrix(x1, 0.0, 0.0),
            "body2": translation_matrix(x2, 0.0, 0.0),
            "force": translation_matrix(x2 + 0.35, 0.0, 0.0),
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        return {"force": _force_arrow_v2(u[0])}


class ThreeMass(StateSpaceSystem):
    """Three-mass linear spring-damper chain with force on the third mass."""

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        super().__init__(n=6, m=1, p=1, name="Three Mass Spring Damper")
        self.output_mass = int(output_mass)
        self.params = {
            "m1": float(m),
            "m2": float(m),
            "m3": float(m),
            "k1": float(k),
            "k2": float(k),
            "k3": float(k),
            "b1": float(b),
            "b2": float(b),
            "b3": float(b),
        }
        self.camera_scale = 6.0

        _, output_label = _mass_output_matrix(3, self.output_mass)
        self.state.labels = ["x1", "x2", "x3", "dx1", "dx2", "dx3"]
        self.state.units = ["m", "m", "m", "m/s", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params
        m1, m2, m3 = params["m1"], params["m2"], params["m3"]
        k1, k2, k3 = params["k1"], params["k2"], params["k3"]
        b1, b2, b3 = params["b1"], params["b2"], params["b3"]

        # nearest-neighbour spring/damper coupling along the chain
        # fmt: off
        return np.array([
            [            0.0,             0.0,      0.0,      1.0,      0.0,      0.0],
            [            0.0,             0.0,      0.0,      0.0,      1.0,      0.0],
            [            0.0,             0.0,      0.0,      0.0,      0.0,      1.0],
            [-(k1 + k2) / m1,         k2 / m1,      0.0, -b1 / m1,      0.0,      0.0],
            [        k2 / m2, -(k2 + k3) / m2,  k3 / m2,      0.0, -b2 / m2,      0.0],
            [            0.0,         k3 / m3, -k3 / m3,      0.0,      0.0, -b3 / m3],
        ])
        # fmt: on

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params
        m3 = params["m3"]

        # the force enters on the third mass
        return np.array(
            [
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [0.0],
                [1.0 / m3],
            ]
        )

    def C(self, t=0.0, params=None):
        C, _ = _mass_output_matrix(3, self.output_mass)
        return C

    def D(self, t=0.0, params=None):
        return np.array([[0.0]])

    def get_kinematic_geometry(self):
        return [
            ground_line(length=12.0),
            spring_line(),
            spring_line(),
            spring_line(),
            _mass_box(size=0.5, color="magenta"),
            _mass_box(size=0.5, color="green"),
            _mass_box(size=0.5, color="blue"),
            Arrow(color="red", linewidth=2, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        x1, x2, x3 = x[0] - 2.0, x[1], x[2] + 2.0
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.28, 0.0])
            if self.params["k1"] != 0.0
            else empty_transform()
        )
        return [
            identity_matrix(),
            spring1,
            line_between_transform([x1 + 0.28, 0.0], [x2 - 0.28, 0.0]),
            line_between_transform([x2 + 0.28, 0.0], [x3 - 0.28, 0.0]),
            translation_matrix(x1, 0.0, 0.0),
            translation_matrix(x2, 0.0, 0.0),
            translation_matrix(x3, 0.0, 0.0),
            _force_arrow_transform(x3 + 0.32, u[0]),
        ]

    # === v2 frame-keyed visualization contract ===========================

    def get_kinematic_geometry_v2(self):
        return {
            "world": [ground_line(length=12.0)],
            "spring1": [spring_line()],
            "spring2": [spring_line()],
            "spring3": [spring_line()],
            "body1": [_mass_box(size=0.5, color="magenta")],
            "body2": [_mass_box(size=0.5, color="green")],
            "body3": [_mass_box(size=0.5, color="blue")],
        }

    def tf_v2(self, x, u, t=0, params=None):
        x1, x2, x3 = x[0] - 2.0, x[1], x[2] + 2.0
        anchor = -4.0
        spring1 = (
            line_between_transform([anchor, 0.0], [x1 - 0.28, 0.0])
            if self.params["k1"] != 0.0
            else empty_transform()
        )
        return {
            "world": identity_matrix(),
            "spring1": spring1,
            "spring2": line_between_transform([x1 + 0.28, 0.0], [x2 - 0.28, 0.0]),
            "spring3": line_between_transform([x2 + 0.28, 0.0], [x3 - 0.28, 0.0]),
            "body1": translation_matrix(x1, 0.0, 0.0),
            "body2": translation_matrix(x2, 0.0, 0.0),
            "body3": translation_matrix(x3, 0.0, 0.0),
            "force": translation_matrix(x3 + 0.32, 0.0, 0.0),
        }

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        return {"force": _force_arrow_v2(u[0])}


class FloatingSingleMass(SingleMass):
    """Single mass with no ground spring."""

    def __init__(self, m=1.0, b=0.0):
        super().__init__(mass=m, k=0.0, b=b)
        self.name = "Floating Single Mass"


class FloatingTwoMass(TwoMass):
    """Two masses with no ground spring on the first mass."""

    def __init__(self, m=1.0, k=1.0, b=0.0, output_mass=2):
        super().__init__(m=m, k=k, b=b, output_mass=output_mass)
        self.name = "Floating Two Mass"
        self.params["k1"] = 0.0


class FloatingThreeMass(ThreeMass):
    """Three masses with no ground spring on the first mass."""

    def __init__(self, m=1.0, k=1.0, b=0.0, output_mass=3):
        super().__init__(m=m, k=k, b=b, output_mass=output_mass)
        self.name = "Floating Three Mass"
        self.params["k1"] = 0.0


if __name__ == "__main__":
    # sys = SingleMass()
    # sys = TwoMass()
    # sys = FloatingThreeMass()
    sys = ThreeMass()

    sys.params["m1"] = 5.0
    sys.params["m2"] = 10.0
    sys.params["m3"] = 1.0
    sys.params["k1"] = 5.0
    sys.params["k2"] = 20.0
    sys.params["k3"] = 20.0
    sys.params["b1"] = 0.0
    sys.params["b2"] = 0.0
    sys.params["b3"] = 0.0

    sys.inputs["u"].nominal_value = np.array([10.0])
    sys.compute_trajectory(tf=15.0)
    sys.animate(time_factor_video=1.0)
