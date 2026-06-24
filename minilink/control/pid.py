"""PID with filtered derivative and anti-windup.

For a PID that takes an explicit rate on the measurement port ``y =
[y, dy_dt]``, use :class:`~minilink.control.linear.PIDController`.

:class:`FilteredPIDController` is for scalar ``y`` only: derivative action
comes from a first-order filtered measurement (dirty derivative) rather than a
separate rate input.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem


class FilteredPIDController(DynamicSystem):
    """Continuous-time PID with filtered derivative and anti-windup.

    States are the integral of the tracking error and a first-order filtered
    copy of the measurement. The derivative acts on the filtered measurement:

        u = kp e + ki e_int - kd dy_filt,   e = r - y,   e_int = ∫ e dt

    Integration stops when the unsaturated command would exceed ``u_min`` /
    ``u_max`` in the direction of the error, or when the integral hits
    ``e_int_min`` / ``e_int_max``.

    Parameters
    ----------
    kp, ki, kd : float
        Proportional, integral, and derivative gains.
    tau : float
        First-order filter time constant on the measurement [s].
    y_filt0 : float
        Initial value of the filtered measurement state ``y_filt``.
    u_min, u_max : float
        Output command saturation limits.
    e_int_min, e_int_max : float
        Integrator state clamp limits.
    name : str
        Block display name.
    """

    def __init__(
        self,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        tau: float = 0.1,
        y_filt0: float = 0.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        e_int_min: float = -np.inf,
        e_int_max: float = np.inf,
    ):
        super().__init__(n=2)
        self.name = "Filtered PID"

        self.params = {
            "kp": kp,
            "ki": ki,
            "kd": kd,
            "tau": tau,
            "u_min": u_min,
            "u_max": u_max,
            "e_int_min": e_int_min,
            "e_int_max": e_int_max,
        }
        self.state.labels = ["e_int", "y_filt"]
        self.x0 = np.array([0.0, y_filt0], dtype=float)

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", nominal_value=0.0)
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        kp = params["kp"]
        ki = params["ki"]
        kd = params["kd"]
        tau = params["tau"]
        u_min = params["u_min"]
        u_max = params["u_max"]
        e_int_min = params["e_int_min"]
        e_int_max = params["e_int_max"]

        # internal states
        e_int = x[0]
        y_filt = x[1]

        # inputs ports
        r = u[0]
        y = u[1]

        # error
        e = r - y

        # filter state derivative
        dy_filt = (y - y_filt) / tau

        # Unsaturated command of the PID controller
        u = kp * e + ki * e_int - kd * dy_filt

        # stop integrator when the output is saturated
        stop_hi = xp.logical_and(u >= u_max, e > 0.0)
        stop_lo = xp.logical_and(u <= u_min, e < 0.0)
        stop_sat = xp.logical_or(stop_hi, stop_lo)
        de_int = xp.where(stop_sat, 0.0, e)

        # stop integrator when the integral is saturated
        stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
        stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
        stop_int = xp.logical_or(stop_int_hi, stop_int_lo)
        de_int = xp.where(stop_int, 0.0, de_int)

        return xp.array([de_int, dy_filt])

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        kp = params["kp"]
        ki = params["ki"]
        kd = params["kd"]
        tau = xp.maximum(params["tau"], 1e-3)
        u_min = params["u_min"]
        u_max = params["u_max"]

        # internal states
        e_int = x[0]
        y_filt = x[1]

        # inputs ports
        r = u[0]
        y = u[1]

        # error
        e = r - y

        # filtered derivative
        dy_filt = (y - y_filt) / tau

        # pid control law
        u = kp * e + ki * e_int - kd * dy_filt

        # saturate the output
        u = xp.clip(u, u_min, u_max)

        return xp.array([u])

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, x, u, t):
        return []
