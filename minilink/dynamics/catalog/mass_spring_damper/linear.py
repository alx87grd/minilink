"""Linear mass-spring-damper chains of one, two and three masses, grounded or floating."""

import numpy as np

from minilink.core.kinematics import translation
from minilink.dynamics.abstraction.state_space import StateSpaceSystem
from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    ground_line,
)
from minilink.graphical.catalog.shapes import spring_between

###############################################################################
# Public API
###############################################################################


class SingleMass(StateSpaceSystem):
    """Single linear mass-spring-damper model."""

    def __init__(self, mass=1.0, k=2.0, b=0.0):
        super().__init__(n=2, m=1, p=1, name="Single Mass Spring Damper")

        # Parameters
        self.params = {"mass": float(mass), "k": float(k), "b": float(b)}

        # Signal labels and units
        self.state.labels = ["x", "dx"]
        self.state.units = ["m", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = ["x"]
        self.outputs["y"].units = ["m"]

        # Graphics
        self.camera_scale = 4.0

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params

        mass = params["mass"]
        k = params["k"]
        b = params["b"]

        # m ẍ + b ẋ + k x = u, on the state [x, ẋ]
        # fmt: off
        A = np.array([
            [      0.0,       1.0],
            [-k / mass, -b / mass],
        ])
        # fmt: on

        return A

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params

        mass = params["mass"]

        # The force enters through the acceleration row
        # fmt: off
        B = np.array([
            [0.0],
            [1.0 / mass],
        ])
        # fmt: on

        return B

    def C(self, t=0.0, params=None):
        # The output is the position of the mass
        C = np.array([[1.0, 0.0]])

        return C

    def D(self, t=0.0, params=None):
        D = np.array([[0.0]])

        return D

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=8.0)],
            "body": [mass_box(size=0.6, color="blue")],
        }

    def tf(self, x, u, t=0, params=None):
        return {"body": translation(x[0], 0.0, 0.0)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        # The spring runs from the wall anchor to the mass's left face
        mass_x = x[0]
        anchor = -2.0
        left_face = mass_x - 0.3

        # The force arrow on the mass, and the spring only when there is one (k ≠ 0)
        geom = {"body": force_arrow(u[0])}
        if params["k"] != 0.0:
            geom["world"] = [spring_between([anchor, 0.0], [left_face, 0.0])]

        return geom


class TwoMass(StateSpaceSystem):
    """Two-mass linear spring-damper chain with force on the second mass.

    The springs couple neighbours: ``k1`` ties the first mass to the wall and ``k2``
    ties the two masses together. Each damper acts on its own mass's absolute
    velocity (``b1`` on the first mass, ``b2`` on the second), not on the relative
    velocity across a spring.
    """

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        super().__init__(n=4, m=1, p=1, name="Two Mass Spring Damper")

        # Parameters: the measured mass, then the chain's masses, springs and dampers
        self.output_mass = int(output_mass)
        self.params = {
            "m1": float(m),
            "m2": float(m),
            "k1": float(k),
            "k2": float(k),
            "b1": float(b),
            "b2": float(b),
        }

        # Signal labels and units: the output is the measured mass's position
        _, output_label = mass_output_matrix(2, self.output_mass)
        self.state.labels = ["x1", "x2", "dx1", "dx2"]
        self.state.units = ["m", "m", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

        # Graphics
        self.camera_scale = 5.0

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params

        m1 = params["m1"]
        m2 = params["m2"]
        k1 = params["k1"]
        k2 = params["k2"]
        b1 = params["b1"]
        b2 = params["b2"]

        # m₁ ẍ₁ = −(k₁ + k₂) x₁ + k₂ x₂ − b₁ ẋ₁
        A31 = -(k1 + k2) / m1
        A32 = k2 / m1
        A33 = -b1 / m1

        # m₂ ẍ₂ = k₂ (x₁ − x₂) − b₂ ẋ₂
        A41 = k2 / m2
        A42 = -k2 / m2
        A44 = -b2 / m2

        # The state [x₁, x₂, ẋ₁, ẋ₂]: the positions move at the velocities
        # fmt: off
        A = np.array([
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
            [A31, A32, A33, 0.0],
            [A41, A42, 0.0, A44],
        ])
        # fmt: on

        return A

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params

        m2 = params["m2"]

        # The force enters on the second mass
        # fmt: off
        B = np.array([
            [0.0],
            [0.0],
            [0.0],
            [1.0 / m2],
        ])
        # fmt: on

        return B

    def C(self, t=0.0, params=None):
        C, _ = mass_output_matrix(2, self.output_mass)

        return C

    def D(self, t=0.0, params=None):
        D = np.array([[0.0]])

        return D

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=10.0)],
            "body1": [mass_box(size=0.55, color="green")],
            "body2": [mass_box(size=0.55, color="blue")],
        }

    def tf(self, x, u, t=0, params=None):
        # The first mass is drawn 2 m left of the second at rest
        x1 = x[0] - 2.0
        x2 = x[1]

        return {
            "body1": translation(x1, 0.0, 0.0),
            "body2": translation(x2, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        x1 = x[0] - 2.0
        x2 = x[1]
        anchor = -4.0

        # The ground spring only when there is one (k₁ ≠ 0), then the coupling spring
        springs = []
        if params["k1"] != 0.0:
            springs.append(spring_between([anchor, 0.0], [x1 - 0.3, 0.0]))
        springs.append(spring_between([x1 + 0.3, 0.0], [x2 - 0.3, 0.0]))

        return {"world": springs, "body2": force_arrow(u[0])}


class ThreeMass(StateSpaceSystem):
    """Three-mass linear spring-damper chain with force on the third mass.

    The springs couple neighbours: ``k1`` ties the first mass to the wall, ``k2``
    and ``k3`` tie each mass to the next. Each damper acts on its own mass's
    absolute velocity (``b1``, ``b2``, ``b3`` on the first, second and third
    mass), not on the relative velocity across a spring.
    """

    def __init__(self, m=1.0, k=2.0, b=0.2, output_mass=2):
        super().__init__(n=6, m=1, p=1, name="Three Mass Spring Damper")

        # Parameters: the measured mass, then the chain's masses, springs and dampers
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

        # Signal labels and units: the output is the measured mass's position
        _, output_label = mass_output_matrix(3, self.output_mass)
        self.state.labels = ["x1", "x2", "x3", "dx1", "dx2", "dx3"]
        self.state.units = ["m", "m", "m", "m/s", "m/s", "m/s"]
        self.inputs["u"].labels = ["force"]
        self.inputs["u"].units = ["N"]
        self.outputs["y"].labels = [output_label]
        self.outputs["y"].units = ["m"]

        # Graphics
        self.camera_scale = 6.0

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params

        m1 = params["m1"]
        m2 = params["m2"]
        m3 = params["m3"]
        k1 = params["k1"]
        k2 = params["k2"]
        k3 = params["k3"]
        b1 = params["b1"]
        b2 = params["b2"]
        b3 = params["b3"]

        # m₁ ẍ₁ = −(k₁ + k₂) x₁ + k₂ x₂ − b₁ ẋ₁
        A41 = -(k1 + k2) / m1
        A42 = k2 / m1
        A44 = -b1 / m1

        # m₂ ẍ₂ = k₂ x₁ − (k₂ + k₃) x₂ + k₃ x₃ − b₂ ẋ₂
        A51 = k2 / m2
        A52 = -(k2 + k3) / m2
        A53 = k3 / m2
        A55 = -b2 / m2

        # m₃ ẍ₃ = k₃ (x₂ − x₃) − b₃ ẋ₃
        A62 = k3 / m3
        A63 = -k3 / m3
        A66 = -b3 / m3

        # The state [x₁, x₂, x₃, ẋ₁, ẋ₂, ẋ₃]: the positions move at the velocities
        # fmt: off
        A = np.array([
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            [A41, A42, 0.0, A44, 0.0, 0.0],
            [A51, A52, A53, 0.0, A55, 0.0],
            [0.0, A62, A63, 0.0, 0.0, A66],
        ])
        # fmt: on

        return A

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params

        m3 = params["m3"]

        # The force enters on the third mass
        # fmt: off
        B = np.array([
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [0.0],
            [1.0 / m3],
        ])
        # fmt: on

        return B

    def C(self, t=0.0, params=None):
        C, _ = mass_output_matrix(3, self.output_mass)

        return C

    def D(self, t=0.0, params=None):
        D = np.array([[0.0]])

        return D

    def get_kinematic_geometry(self):
        return {
            "world": [ground_line(length=12.0)],
            "body1": [mass_box(size=0.5, color="magenta")],
            "body2": [mass_box(size=0.5, color="green")],
            "body3": [mass_box(size=0.5, color="blue")],
        }

    def tf(self, x, u, t=0, params=None):
        # The masses are drawn 2 m apart at rest, the second one at the origin
        x1 = x[0] - 2.0
        x2 = x[1]
        x3 = x[2] + 2.0

        return {
            "body1": translation(x1, 0.0, 0.0),
            "body2": translation(x2, 0.0, 0.0),
            "body3": translation(x3, 0.0, 0.0),
        }

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        x1 = x[0] - 2.0
        x2 = x[1]
        x3 = x[2] + 2.0
        anchor = -4.0

        # The ground spring only when there is one (k₁ ≠ 0), then the two coupling springs
        springs = []
        if params["k1"] != 0.0:
            springs.append(spring_between([anchor, 0.0], [x1 - 0.28, 0.0]))
        springs.extend(
            [
                spring_between([x1 + 0.28, 0.0], [x2 - 0.28, 0.0]),
                spring_between([x2 + 0.28, 0.0], [x3 - 0.28, 0.0]),
            ]
        )

        return {"world": springs, "body3": force_arrow(u[0], base=(0.32, 0.0))}


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

        # Parameters: no ground spring
        self.params["k1"] = 0.0


class FloatingThreeMass(ThreeMass):
    """Three masses with no ground spring on the first mass."""

    def __init__(self, m=1.0, k=1.0, b=0.0, output_mass=3):
        super().__init__(m=m, k=k, b=b, output_mass=output_mass)
        self.name = "Floating Three Mass"

        # Parameters: no ground spring
        self.params["k1"] = 0.0


###############################################################################
# Internal machinery
###############################################################################


def mass_box(size=0.5, color="blue", opacity=0.9):
    """A flat square block of side ``size`` for one mass."""
    return Box(
        length_x=size,
        length_y=size,
        length_z=0.2 * size,
        color=color,
        opacity=opacity,
    )


def force_arrow(force, base=(0.35, 0.0)):
    """The force as an arrow in the body frame, along ±x from ``base``, 0.3 m per newton."""
    # No arrow under 10⁻¹² N
    if abs(force) < 1e-12:
        theta = 0.0
        length = 0.0
    else:
        theta = 0.0 if force >= 0.0 else np.pi
        length = 0.3 * abs(force)

    d = np.array([np.cos(theta), np.sin(theta)])

    return [Arrow(base=base, vector=d, scale=length, color="red", linewidth=2)]


def mass_output_matrix(count, output_mass):
    """The row ``C`` measuring the position of mass ``output_mass`` (from 1), and its label."""
    require_output_mass(count, output_mass)

    C = np.zeros((1, 2 * count))
    C[0, output_mass - 1] = 1.0

    return C, f"x{output_mass}"


def require_output_mass(count, output_mass):
    """Refuse an output mass outside the chain's ``1 … count``."""
    if output_mass < 1 or output_mass > count:
        raise ValueError(
            f"output_mass must be between 1 and {count} (the masses of the chain), "
            f"got {output_mass}"
        )


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
