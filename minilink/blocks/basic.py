from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem


class Integrator(DynamicSystem):
    """Single integrator block: ``dx = k * u``, ``y = x``.

    The gain ``k`` defaults to 1.0 (a pure integrator); it also serves as the
    canonical scalar block parameter in tests and demos.
    """

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())

        self.params = {"k": 1.0}

        self.name = "Integrator"

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        k = params["k"]
        xp = array_module(u)
        return xp.array([k * u[0]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])
