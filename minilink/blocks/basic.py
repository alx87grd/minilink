"""Basic dynamic blocks: the integrator."""

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


class Integrator(DynamicSystem):
    """Single integrator block: ``dx = k * u``, ``y = x``.

    The gain ``k`` defaults to 1.0 (a pure integrator); it also serves as the
    canonical scalar block parameter in tests and demos.
    """

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "Integrator"

        # Parameters: the integrator gain
        self.params = {"k": 1.0}

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        k = params["k"]

        dx = xp.array([k * u[0]])

        return dx

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)

        y = xp.array([x[0]])

        return y
