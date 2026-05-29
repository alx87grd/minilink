from minilink.compile.jax_utils import array_module
from minilink.core.system import DynamicSystem, StaticSystem


class Integrator(DynamicSystem):
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


class PropController(StaticSystem):
    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", nominal_value=0.0)
        self.add_output_port(
            "u", dim=1, function=self.ctl, dependencies=("r", "y")
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]

        r, y = self.get_port_values_from_u(u, "r", "y")

        u_cmd = Kp * (r[0] - y[0])
        xp = array_module(u)
        return xp.array([u_cmd])
