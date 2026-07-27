import numpy as np

from minilink.core.system import DynamicSystem


class VanderPol(DynamicSystem):
    """Van der Pol oscillator."""

    def __init__(self, mu=0.5):
        super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
        self.name = "Van der Pol Oscillator"
        self.params = {"mu": float(mu)}
        self.state.labels = ["y", "dy"]
        self.state.units = ["", "1/s"]
        self.state.lower_bound[:] = [-3.0, -3.0]
        self.state.upper_bound[:] = [3.0, 3.0]
        self.inputs["u"].labels = ["unused"]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        mu = params["mu"]
        y, dy = x

        # Van der Pol oscillator: damping is negative (pumps energy) for |y| < 1
        ddy = -y + mu * dy * (1.0 - y**2)
        return np.array([dy, ddy])

    def h(self, x, u, t=0.0, params=None):
        return x


class Lorenz(DynamicSystem):
    """Lorenz chaotic attractor."""

    def __init__(self, sigma=10.0, rho=28.0, beta=8.0 / 3.0):
        super().__init__(n=3, output_dim=3, expose_state=True)
        self.name = "Lorenz Attractor"
        self.params = {
            "sigma": float(sigma),
            "rho": float(rho),
            "beta": float(beta),
        }
        self.state.labels = ["x", "y", "z"]
        self.state.units = ["", "", ""]
        self.outputs["y"].labels = list(self.state.labels)
        self.outputs["y"].units = list(self.state.units)
        self.solver_info = {
            "smallest_time_constant": 0.01,
            "discontinuous_behavior": False,
        }

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        sigma = params["sigma"]
        rho = params["rho"]
        beta = params["beta"]
        x_val, y, z = x

        # Lorenz equations
        dx = sigma * (y - x_val)
        dy = x_val * (rho - z) - y
        dz = x_val * y - beta * z
        return np.array([dx, dy, dz])

    def h(self, x, u, t=0.0, params=None):
        return x


if __name__ == "__main__":
    sys = VanderPol()
    sys.x0 = np.array([0.0, 1.0])
    sys.compute_trajectory(tf=20.0, n_steps=400, show=True, verbose=False)
    sys.plot_phase_plane(traj=sys.traj)
