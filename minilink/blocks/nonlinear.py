"""Static nonlinearities: saturation, dead zone, relay, and the stateful rate limiter."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class Saturation(System):
    """Symmetric or asymmetric clip ``y = clip(u, lower, upper)``.

    The end stops are scalars, shared by the ``dim`` channels (default 1), or
    vectors, one stop per channel: ``Saturation([-1.0, -2.0], [1.0, 2.0])``
    clips two channels, its ``dim`` the bounds' size.
    """

    def __init__(self, lower=-1.0, upper=1.0, dim=None):
        dim = channels_of(lower, upper, dim)
        require_ordered_end_stops(lower, upper)

        super().__init__()

        # Parameters: the number of channels and the clip limits (a float, or one per channel)
        self.dim = dim
        self.params = {"lower": end_stop(lower), "upper": end_stop(upper)}

        # Ports
        self.add_input_port("u", dim=self.dim)
        self.add_output_port("y", dim=self.dim, function=self.h, dependencies="all")

    def h(self, x, u, t=0, params=None):
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
    if np.any(np.asarray(upper) < np.asarray(lower)):
        raise ValueError("the upper end stop must sit above the lower one")


def end_stop(bound):
    """A scalar end stop as a float, a vector one as a flat float array."""
    bound = np.asarray(bound, dtype=float)
    if bound.ndim == 0:
        return float(bound)

    return bound.reshape(-1).copy()


def channels_of(lower, upper, dim):
    """The channel count: ``dim`` when given, else the bounds' size (1 for scalars); a vector bound of another size is refused."""
    sizes = {np.size(bound) for bound in (lower, upper) if np.ndim(bound) > 0}
    if len(sizes) > 1:
        raise ValueError(
            f"lower and upper must have one size, got sizes {sorted(sizes)}"
        )
    size = sizes.pop() if sizes else None
    if dim is None:
        return 1 if size is None else int(size)
    if size is not None and size != int(dim):
        raise ValueError(f"dim={dim} but the end stops have {size} entries")

    return int(dim)


if __name__ == "__main__":
    grid = np.linspace(-2.0, 2.0, 9)
    print("input:     ", grid)
    print(
        "saturation:",
        np.array([Saturation(-1, 1).h(None, np.array([v]))[0] for v in grid]),
    )
