"""Static nonlinearities: saturation, dead zone, relay, and the stateful rate limiter."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, System


class Saturation(System):
    """Symmetric or asymmetric clip ``y = clip(u, lower, upper)``."""

    def __init__(self, lower=-1.0, upper=1.0, dim=1):
        super().__init__()
        self.name = "Saturation"
        self.dim = int(dim)
        self.params = {"lower": float(lower), "upper": float(upper)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        lower = params["lower"]
        upper = params["upper"]

        xp = array_module(u)

        y = xp.clip(u, lower, upper)

        return y


class DeadZone(System):
    """Dead zone of half-width ``width``: zero inside, shifted outside.

    ``y = u - width`` for ``u > width``, ``y = u + width`` for ``u < -width``,
    and ``y = 0`` in between — the standard backlash/stiction model.
    """

    def __init__(self, width=1.0, dim=1):
        super().__init__()
        self.name = "DeadZone"
        self.dim = int(dim)
        self.params = {"width": float(width)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        width = params["width"]

        xp = array_module(u)
        above = xp.where(u > width, u - width, 0.0)
        below = xp.where(u < -width, u + width, 0.0)
        y = above + below

        return y


class Relay(System):
    """Bang-bang relay ``y = amplitude · sign(u)`` (sign(0) = 0)."""

    def __init__(self, amplitude=1.0, dim=1):
        super().__init__()
        self.name = "Relay"
        self.dim = int(dim)
        self.params = {"amplitude": float(amplitude)}

        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        amplitude = params["amplitude"]

        xp = array_module(u)

        y = amplitude * xp.sign(u)

        return y


class RateLimiter(DynamicSystem):
    """Pass a command through, limited to ``rate_max`` per second and to the end stops.

    Parameters
    ----------
    rate_max : float
        Largest rate of change of the output [unit/s]; must be positive.
    tau : float
        Time constant of the lag the block shows for small steps [s]; must be positive.
        The knee between the two regimes sits at a step of about ``tau * rate_max``.
    lower, upper : float
        End stops on the output [unit]; infinite by default, i.e. a rate limit only.
    dim : int
        Number of independent channels.
    x0 : float or array
        Output at ``t = 0`` [unit], one value or one per channel.
    """

    def __init__(
        self,
        rate_max: float = 1.0,
        tau: float = 0.05,
        lower: float = -np.inf,
        upper: float = np.inf,
        dim: int = 1,
        x0=0.0,
    ):
        if rate_max <= 0.0:
            raise ValueError("rate_max must be positive (it scales the tanh)")
        if tau <= 0.0:
            raise ValueError("tau must be positive (it divides the lag rate)")
        if upper < lower:
            raise ValueError("the upper end stop must sit above the lower one")

        super().__init__(n=int(dim), input_dim=int(dim), output_dim=int(dim))

        self.name = "Rate Limiter"
        self.params = {
            "rate_max": float(rate_max),
            "tau": float(tau),
            "lower": float(lower),
            "upper": float(upper),
        }
        self.state.labels = [f"u_lim{i}" for i in range(self.n)]
        self.x0 = np.clip(
            np.broadcast_to(np.asarray(x0, dtype=float), (self.n,)).astype(float),
            lower,
            upper,
        )
        self.solver_info["smallest_time_constant"] = float(tau)

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        rate_max, tau = params["rate_max"], params["tau"]
        lower, upper = params["lower"], params["upper"]
        xp = array_module(x, u)

        # a first-order lag toward the clipped command, its slope limited to rate_max
        x_ref = xp.clip(u, lower, upper)
        dx = rate_max * xp.tanh((x_ref - x) / (tau * rate_max))

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x


if __name__ == "__main__":
    grid = np.linspace(-2.0, 2.0, 9)
    print("input:     ", grid)
    print(
        "saturation:",
        np.array([Saturation(-1, 1).compute(None, np.array([v]))[0] for v in grid]),
    )
    print(
        "dead zone: ",
        np.array([DeadZone(0.5).compute(None, np.array([v]))[0] for v in grid]),
    )
    print(
        "relay:     ",
        np.array([Relay(2.0).compute(None, np.array([v]))[0] for v in grid]),
    )
