"""Basic dynamic blocks: the integrator."""

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, fastest_time_constant

###############################################################################
# Public API
###############################################################################


class Integrator(DynamicSystem):
    """Single integrator block: ``dx = k * u``, ``y = x``, on ``dim`` channels.

    The gain ``k`` defaults to 1.0 (a pure integrator); it also serves as the
    canonical scalar block parameter in tests and demos. ``dim`` (default 1)
    integrates each entry of a vector input, as ``UnitDelay(dim)`` delays it.
    """

    def __init__(self, dim=1):
        super().__init__(
            state_dim=dim, input_dim=dim, output_dim=dim, y_dependencies=()
        )

        # Parameters: the integrator gain
        self.params = {"k": 1.0}

    @property
    def solver_info(self) -> dict:
        """The solver hints: a pole at the origin, no time constant (``∞``), so the integrator bounds no step."""
        info = self.own_solver_info
        info["smallest_time_constant"] = fastest_time_constant([0.0])

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        k = params["k"]

        # ẋ = k u
        dx = k * xp.asarray(u)

        return dx

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)

        # y = x, a copy of the state
        y = xp.array(x)

        return y
