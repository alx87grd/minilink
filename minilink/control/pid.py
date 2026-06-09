"""Generic summing junction and PID controller blocks."""

import numpy as np

from minilink.core.system import DynamicSystem, System


class Sum(System):
    """Two-input summing junction with optional output saturation.

    Parameters
    ----------
    max : float, optional
        Upper saturation limit. Saturation is applied only when both ``max``
        and ``min`` are provided.
    min : float, optional
        Lower saturation limit.
    name : str, optional
        Display name of the block.
    """

    def __init__(self, max=None, min=None, name: str = "Sum"):
        super().__init__(0)

        self.name = name

        self.max = max
        self.min = min

        self.inputs = {}
        self.add_input_port("1", nominal_value=np.array([0.0]))
        self.add_input_port("2", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            "result",
            dim=1,
            function=self.h_sum,
            dependencies=["1", "2"],
        )

    def h_sum(self, x, u, t=0.0, params=None):
        a, b = self.get_port_values_from_u(u, "1", "2")
        result = a[0] + b[0]

        if self.max is not None and self.min is not None:
            result = np.clip(result, self.min, self.max)

        return np.array([result])


class PID(DynamicSystem):
    """Generic scalar PID controller with filtered derivative.

    The controller has two states: the error integral and a first-order
    filtered measurement used for the derivative term.

    .. math::

        e = ref - meas

        cmd = K_p \\, e + K_i \\int e \\, dt - K_d \\, \\dot{meas}_{filt}

    Anti-windup clamps the integrator when the command saturates against
    ``cmd_min`` / ``cmd_max`` or when the integral reaches ``i_min`` / ``i_max``.

    Parameters
    ----------
    Kp, Ki, Kd : float
        Proportional, integral, and derivative gains.
    tau : float
        Time constant of the measurement derivative filter.
    meas0 : float
        Initial filtered measurement.
    cmd_min, cmd_max : float
        Command saturation limits.
    i_min, i_max : float
        Integrator saturation limits.
    name : str
        Display name of the block.
    """

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        tau: float = 0.1,
        meas0: float = 0.0,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        name: str = "PID",
    ):
        super().__init__(2)
        self.name = name

        self.params = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "tau": tau,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "i_min": i_min,
            "i_max": i_max,
        }
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        self.inputs = {}
        self.add_input_port("ref", nominal_value=np.array([0.0]), labels=["ref"])
        self.add_input_port("meas", nominal_value=np.array([0.0]), labels=["meas"])

        self.outputs = {}
        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_cmd,
            dependencies=["ref", "meas"],
            labels=["cmd"],
        )
        self.add_output_port(
            "logs",
            dim=2,
            function=self.h_logs,
            dependencies=["ref", "meas"],
            labels=["ref", "meas"],
        )
        self.add_output_port(
            "pid_int_value",
            dim=3,
            function=self.h_internals,
            dependencies=[],
            labels=["error", "d_meas", "int_e"],
        )

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")
        ref, meas = ref[0], meas[0]

        e = ref - meas
        tau = max(p["tau"], 1e-3)
        d_meas_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_meas_filt

        # Anti-windup: stop integrating when the command saturates against
        # the error direction, or when the integral hits its own limits.
        stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
        stop_lo = (cmd <= p["cmd_min"]) and (e < 0)
        d_int_e = 0.0 if (stop_hi or stop_lo) else e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        return np.array([d_int_e, d_meas_filt])

    def h_cmd(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")
        ref, meas = ref[0], meas[0]

        e = ref - meas
        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd])

    def h_logs(self, x, u, t=0.0, params=None):
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")
        return np.array([ref[0], meas[0]])

    def h_internals(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "ref", "meas")
        ref, meas = ref[0], meas[0]

        e = ref - meas
        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        return np.array([e, d_filt, int_e])
