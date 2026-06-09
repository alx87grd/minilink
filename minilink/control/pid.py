"""Generic PID controller block."""

import numpy as np

from minilink.core.system import DynamicSystem


class PID(DynamicSystem):
    """Generic scalar PID controller with filtered derivative.

    The controller has two states: the error integral and a first-order
    filtered measurement used for the derivative term. Its two input ports form
    the difference junction, so no separate summing block is needed.

    .. math::

        e = r - y

        u = K_p \\, e + K_i \\int e \\, dt - K_d \\, \\dot{y}_{filt}

    Anti-windup stops the integrator when the command saturates against
    ``cmd_min`` / ``cmd_max`` in the direction of the error.

    Ports follow the control-naming convention: input ``r`` (reference), input
    ``y`` (measurement), output ``u`` (command).

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
        }
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        self.inputs = {}
        self.add_input_port("r", nominal_value=np.array([0.0]), labels=["ref"])
        self.add_input_port("y", nominal_value=np.array([0.0]), labels=["meas"])

        self.outputs = {}
        self.add_output_port(
            "u",
            dim=1,
            function=self.h_u,
            dependencies=["r", "y"],
            labels=["cmd"],
        )

    def control_law(self, int_e, meas_filt, ref, meas, params):
        """Return ``(error, filtered_derivative, command)`` for the given state.

        Shared by the state derivative and the command output so the two stay
        consistent.
        """
        e = ref - meas
        tau = max(params["tau"], 1e-3)
        d_meas_filt = (meas - meas_filt) / tau
        cmd = params["Kp"] * e + params["Ki"] * int_e - params["Kd"] * d_meas_filt
        return e, d_meas_filt, cmd

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "r", "y")

        e, d_meas_filt, cmd = self.control_law(int_e, meas_filt, ref[0], meas[0], p)

        # Anti-windup: stop integrating when the command saturates against the
        # error direction.
        stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
        stop_lo = (cmd <= p["cmd_min"]) and (e < 0)
        d_int_e = 0.0 if (stop_hi or stop_lo) else e

        return np.array([d_int_e, d_meas_filt])

    def h_u(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = x[0], x[1]
        ref, meas = self.get_port_values_from_u(u, "r", "y")

        _, _, cmd = self.control_law(int_e, meas_filt, ref[0], meas[0], p)
        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd])
