"""The three-body problem: three point masses under mutual Newtonian gravitation."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.kinematics import translation
from minilink.core.system import DynamicSystem
from minilink.graphical.animation.primitives import Sphere

# Chenciner–Montgomery figure-eight initial condition (equal masses, G = 1).
_FIGURE_EIGHT_X0 = np.array(
    [
        0.97000436,
        -0.24308753,
        0.0,
        0.4662036850,
        0.4323657300,
        0.0,
        -0.97000436,
        0.24308753,
        0.0,
        0.4662036850,
        0.4323657300,
        0.0,
        0.0,
        0.0,
        0.0,
        -0.93240737,
        -0.86473146,
        0.0,
    ]
)


###############################################################################
# Public API
###############################################################################


class ThreeBodyProblem(DynamicSystem):
    """Mutual Newtonian gravitation for three point masses in 3-D.

    State layout (18):

        x = [r1, v1, r2, v2, r3, v3]

    with ``r_i, v_i in R^3``. Each mass follows::

        v_i_dot = sum_{j != i} G m_j (r_j - r_i) / |r_j - r_i|^3

    with optional softening ``eps`` added inside the distance norm.
    """

    def __init__(self, preset="figure_eight", masses=None, G=1.0, softening=0.0):
        super().__init__(n=18, output_dim=18, expose_state=True)

        # Parameters: three unit masses unless given
        if masses is None:
            masses = np.ones(3)
        masses = np.asarray(masses, dtype=float)
        require_three_masses(masses)
        self.params = {
            "G": float(G),
            "masses": masses,
            "softening": float(softening),
            "body_radius": 0.05,
        }

        # Signal labels and units
        self.state.labels = [
            "r1x",
            "r1y",
            "r1z",
            "v1x",
            "v1y",
            "v1z",
            "r2x",
            "r2y",
            "r2z",
            "v2x",
            "v2y",
            "v2z",
            "r3x",
            "r3y",
            "r3z",
            "v3x",
            "v3y",
            "v3z",
        ]
        self.state.units = (["m"] * 3 + ["m/s"] * 3) * 3
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Graphics: the camera looks down on the x-y plane
        self.camera_target = np.zeros(3)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 2.5

        # Solver hints: the fastest time scale, and smooth dynamics
        self.solver_info = {
            "smallest_time_constant": 0.01,
            "discontinuous_behavior": False,
        }

        # Initial state: the figure-eight orbit, or all bodies at rest at the origin
        self.x0 = initial_state_of(preset, self.n)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        G = params["G"]
        masses = params["masses"]
        eps = params["softening"]

        r = self.positions(x)
        v = self.velocities(x)

        # v̇_i = Σ_{j≠i} G m_j (r_j − r_i) / (|r_j − r_i|² + ε²)^{3/2}
        a = []
        for i in range(3):
            acc = xp.zeros(3)
            for j in range(3):
                if i == j:
                    continue
                dr = r[j] - r[i]
                dist_sq = xp.dot(dr, dr) + eps**2
                acc = acc + G * masses[j] * dr / (dist_sq * xp.sqrt(dist_sq))
            a.append(acc)

        # ẋ = [v₁, v̇₁, v₂, v̇₂, v₃, v̇₃]
        dx = xp.concatenate([v[0], a[0], v[1], a[1], v[2], a[2]])

        return dx

    def h(self, x, u, t=0, params=None):
        return x

    def kinetic_energy(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        masses = params["masses"]

        v1, v2, v3 = self.velocities(x)

        # T = Σ ½ m_i |v_i|²
        T = 0.5 * (
            masses[0] * xp.dot(v1, v1)
            + masses[1] * xp.dot(v2, v2)
            + masses[2] * xp.dot(v3, v3)
        )

        return T

    def potential_energy(self, x, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        G = params["G"]
        masses = params["masses"]
        eps = params["softening"]

        r1, r2, r3 = self.positions(x)
        pairs = ((0, 1, r1, r2), (0, 2, r1, r3), (1, 2, r2, r3))

        # V = −Σ_{i<j} G m_i m_j / √(|r_j − r_i|² + ε²)
        V = xp.zeros(())
        for i, j, ri, rj in pairs:
            dr = rj - ri
            dist = xp.sqrt(xp.dot(dr, dr) + eps**2)
            V -= G * masses[i] * masses[j] / dist

        return V

    def total_energy(self, x, params=None):
        T = self.kinetic_energy(x, params)
        V = self.potential_energy(x, params)

        E = T + V

        return E

    def get_kinematic_geometry(self):
        radius = self.params["body_radius"]
        colors = ("#e45756", "#4c78a8", "#72b7b2")

        return {
            "body1": [Sphere(radius=radius, color=colors[0])],
            "body2": [Sphere(radius=radius, color=colors[1])],
            "body3": [Sphere(radius=radius, color=colors[2])],
        }

    def tf(self, x, u, t=0, params=None):
        r1, r2, r3 = self.positions(x)

        return {
            "body1": translation(r1[0], r1[1], r1[2]),
            "body2": translation(r2[0], r2[1], r2[2]),
            "body3": translation(r3[0], r3[1], r3[2]),
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def positions(self, x):
        """Return ``(r1, r2, r3)`` each shape ``(3,)`` from a state vector."""
        return x[0:3], x[6:9], x[12:15]

    def velocities(self, x):
        """Return ``(v1, v2, v3)`` each shape ``(3,)`` from a state vector."""
        return x[3:6], x[9:12], x[15:18]


###############################################################################
# Internal machinery
###############################################################################


def require_three_masses(masses):
    """Refuse a mass vector that is not one mass per body."""
    if masses.shape != (3,):
        raise ValueError("masses must be a length-3 vector")


def initial_state_of(preset, n):
    """The initial state a preset names: the figure-eight orbit, or zeros."""
    if preset == "figure_eight":
        return _FIGURE_EIGHT_X0.copy()

    if preset is None:
        return np.zeros(n)

    raise ValueError("preset must be 'figure_eight' or None")


if __name__ == "__main__":
    sys = ThreeBodyProblem()
    sys.compute_trajectory(tf=6.3, n_steps=630, show=False, verbose=False)
    sys.animate(show=False)
