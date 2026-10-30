"""Static nonlinearities: saturation, dead zone, relay, and the stateful rate limiter."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class Saturation(System):
    """Symmetric or asymmetric clip ``y = clip(u, lower, upper)``."""

    def __init__(self, lower=-1.0, upper=1.0, dim=1):
        require_ordered_end_stops(lower, upper)

        super().__init__()

        # Parameters: the number of channels and the clip limits
        self.dim = int(dim)
        self.params = {"lower": float(lower), "upper": float(upper)}

        # Ports
        self.add_input_port("u", dim=self.dim)
        self.add_output_port(
            "y", dim=self.dim, function=self.compute, dependencies="all"
        )

    def compute(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        lower = params["lower"]
        upper = params["upper"]

        y = xp.clip(u, lower, upper)

        return y


###############################################################################
# Internal machinery
###############################################################################


def require_ordered_end_stops(lower, upper):
    """Refuse an upper end stop below the lower one."""
    if upper < lower:
        raise ValueError("the upper end stop must sit above the lower one")


if __name__ == "__main__":
    grid = np.linspace(-2.0, 2.0, 9)
    print("input:     ", grid)
    print(
        "saturation:",
        np.array([Saturation(-1, 1).compute(None, np.array([v]))[0] for v in grid]),
    )
