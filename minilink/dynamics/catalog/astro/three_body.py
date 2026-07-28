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


class ThreeBodyProblem(DynamicSystem):
    """Mutual Newtonian gravitation for three point masses in 3-D.

    State layout (18):

        x = [r1, v1, r2, v2, r3, v3]

    with ``r_i, v_i in R^3``. Each mass follows

        v_i_dot = sum_{j != i} G m_j (r_j - r_i) / |r_j - r_i|^3

    with optional softening ``eps`` added inside the distance norm.
    """

    def __init__(self, preset="figure_eight", masses=None, G=1.0, softening=0.0):
        super().__init__(n=18, output_dim=18, expose_state=True)
        self.name = "Three-Body Problem"
        if masses is None:
            masses = np.ones(3)
        masses = np.asarray(masses, dtype=float)
        if masses.shape != (3,):
            raise ValueError("masses must be a length-3 vector")
        self.params = {
            "G": float(G),
            "masses": masses,
            "softening": float(softening),
            "body_radius": 0.05,
        }
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

        self.camera_target = np.zeros(3)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 2.5
        self.solver_info = {
            "smallest_time_constant": 0.01,
            "discontinuous_behavior": False,
        }

        if preset == "figure_eight":
            self.x0 = _FIGURE_EIGHT_X0.copy()
        elif preset is None:
            self.x0 = np.zeros(self.n)
        else:
            raise ValueError("preset must be 'figure_eight' or None")

    def positions(self, x):
        """Return ``(r1, r2, r3)`` each shape ``(3,)`` from a state vector."""
        return x[0:3], x[6:9], x[12:15]

    def velocities(self, x):
        """Return ``(v1, v2, v3)`` each shape ``(3,)`` from a state vector."""
        return x[3:6], x[9:12], x[15:18]

    def kinetic_energy(self, x, params=None):
        params = self.params if params is None else params
        masses = params["masses"]
        v1, v2, v3 = self.velocities(x)
        xp = array_module(x)
        return 0.5 * (
            masses[0] * xp.dot(v1, v1)
            + masses[1] * xp.dot(v2, v2)
            + masses[2] * xp.dot(v3, v3)
        )

    def potential_energy(self, x, params=None):
        params = self.params if params is None else params
        G = params["G"]
        masses = params["masses"]
        eps = params["softening"]
        r1, r2, r3 = self.positions(x)
        pairs = ((0, 1, r1, r2), (0, 2, r1, r3), (1, 2, r2, r3))
        xp = array_module(x)
        phi = xp.zeros(())
        for i, j, ri, rj in pairs:
            dr = rj - ri
            dist = xp.sqrt(xp.dot(dr, dr) + eps**2)
            phi -= G * masses[i] * masses[j] / dist
        return phi

    def total_energy(self, x, params=None):
        return self.kinetic_energy(x, params) + self.potential_energy(x, params)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        G = params["G"]
        masses = params["masses"]
        eps = params["softening"]
        xp = array_module(x)

        r = (x[0:3], x[6:9], x[12:15])
        v = (x[3:6], x[9:12], x[15:18])
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

        dx = xp.zeros(18)
        dx[0:3] = v[0]
        dx[3:6] = a[0]
        dx[6:9] = v[1]
        dx[9:12] = a[1]
        dx[12:15] = v[2]
        dx[15:18] = a[2]
        return dx

    def h(self, x, u, t=0, params=None):
        return x

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


if __name__ == "__main__":
    sys = ThreeBodyProblem()
    sys.compute_trajectory(tf=6.3, n_steps=630, show=False, verbose=False)
    sys.animate(show=False)
