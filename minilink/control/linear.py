"""Generic linear control-law blocks.

Plant-agnostic feedback laws following the standard port naming: ``r``
reference, ``y`` measurement, ``u`` control command.
"""

from minilink.core.backends import array_module
from minilink.core.system import StaticSystem


class PController(StaticSystem):
    """Proportional controller ``u = Kp * (r - y)`` on scalar ports."""

    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", nominal_value=0.0)
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]

        r, y = self.get_port_values_from_u(u, "r", "y")

        u_cmd = Kp * (r[0] - y[0])
        xp = array_module(u)
        return xp.array([u_cmd])


class PDController(StaticSystem):
    """PD controller on a (position, rate) measurement.

    The measurement port ``y`` carries ``[position, rate]`` (for example a
    mechanical system exposing ``[q, dq]``); the reference ``r`` targets the
    position only.

    Parameters
    ----------
    y_labels, y_units : sequence of str, optional
        Display metadata for the measurement port (defaults are generic;
        pass e.g. ``("theta", "theta_dot")`` / ``("rad", "rad/s")`` for a
        pendulum).
    u_labels, u_units : sequence of str, optional
        Display metadata for the command port.
    """

    def __init__(
        self,
        y_labels=("position", "rate"),
        y_units=("", ""),
        u_labels=("force",),
        u_units=("",),
    ):
        super().__init__()

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port(
            "y",
            nominal_value=[0.0, 0.0],
            labels=list(y_labels),
            units=list(y_units),
        )
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=list(u_labels),
            units=list(u_units),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]
        Kd = params["Kd"]

        r, y = self.get_port_values_from_u(u, "r", "y")
        position = y[0]
        rate = y[1]

        u_cmd = Kp * (r[0] - position) - Kd * rate
        xp = array_module(u)
        return xp.array([u_cmd])
