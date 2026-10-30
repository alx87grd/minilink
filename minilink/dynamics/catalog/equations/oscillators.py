"""Canonical nonlinear oscillators: Van der Pol and Lorenz."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


class VanderPol(DynamicSystem):
    """Van der Pol oscillator."""

    def __init__(self, mu=0.5):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)

        # Parameters
        self.params = {"mu": float(mu)}

        # Signal labels, units and bounds
        self.state.labels = ["y", "dy"]
        self.state.units = ["", "1/s"]
        self.state.lower_bound[:] = [-3.0, -3.0]
        self.state.upper_bound[:] = [3.0, 3.0]
        self.inputs["u"].labels = ["unused"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        mu = params["mu"]

        y, dy = x

        # ÿ − μ (1 − y²) ẏ + y = 0: negative damping, pumping energy, for |y| < 1
        ddy = -y + mu * dy * (1.0 - y**2)
        dx = xp.array([dy, ddy])

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x


class Lorenz(DynamicSystem):
    """Lorenz chaotic attractor."""

    def __init__(self, sigma=10.0, rho=28.0, beta=8.0 / 3.0):
        super().__init__(n=3, output_dim=3, expose_state=True)

        # Parameters
        self.params = {
            "sigma": float(sigma),
            "rho": float(rho),
            "beta": float(beta),
        }

        # Signal labels and units
        self.state.labels = ["x", "y", "z"]
        self.state.units = ["", "", ""]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

        # Solver hints: the fastest time scale, and smooth dynamics
        self.solver_info = {
            "smallest_time_constant": 0.01,
            "discontinuous_behavior": False,
        }

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        sigma = params["sigma"]
        rho = params["rho"]
        beta = params["beta"]

        x_val, y, z = x

        # The Lorenz equations, with x_val the first coordinate (x names the state)
        dx = sigma * (y - x_val)
        dy = x_val * (rho - z) - y
        dz = x_val * y - beta * z
        dx_dt = xp.array([dx, dy, dz])

        return dx_dt

    def h(self, x, u, t=0.0, params=None):
        return x


if __name__ == "__main__":
    sys = VanderPol()
    sys.x0 = np.array([0.0, 1.0])
    sys.compute_trajectory(tf=20.0, n_steps=400, show=True, verbose=False)
    sys.plot_phase_plane(traj=sys.traj)
