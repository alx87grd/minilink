"""Feedback profile: siso — decoupled PID loops, compensator or controller form."""

import numpy as np

from minilink.control.impedance import _as_dof_vector
from minilink.core.backends import array_module
from minilink.core.feedback import DynamicController, ErrorDriven


class PID(ErrorDriven, DynamicController):
    """Decoupled PID with filtered derivative and anti-windup.

    ``ports="error"`` (default) is the compensator form: one input ``e``, so
    ``PID(...) @ plant`` inserts the Error block ``e = r - y`` and
    ``PID(...) >> plant`` is the loop gain. ``ports="reference"`` is the
    controller form with inputs ``r`` and ``y``. Per axis,

        u_i = Kp_i e_i + Ki_i e_int_i + Kd_i d_i,

    where ``d_i`` is the filtered derivative of ``e_i`` (error form) or of
    ``-y_i`` (reference form: no derivative kick on reference steps); the
    integrator stops while the command saturates.

    Parameters
    ----------
    Kp, Ki, Kd : float or vector
        Per-axis gains (a scalar broadcasts to all axes).
    tau : float or vector
        Derivative filter time constant per axis [s].
    dof : int
        Number of independent scalar loops (default 1).
    ports : {"error", "reference"}
        Input layout, see above.
    y_filt0 : float or vector
        Initial filtered state per axis (the filtered ``y``, or ``-e``).
    u_min, u_max, e_int_min, e_int_max : float or vector
        Saturation limits per axis.
    """

    feedback_profile = "siso"

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        tau: float = 0.1,
        *,
        dof: int = 1,
        ports: str = "error",
        y_filt0=0.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        e_int_min: float = -np.inf,
        e_int_max: float = np.inf,
    ):
        n = int(dof)
        if n <= 0:
            raise ValueError("dof must be positive")

        super().__init__(n=2 * n)
        self.dof = n
        self.name = "PID"

        self.params = {
            "Kp": _as_dof_vector(Kp, n),
            "Ki": _as_dof_vector(Ki, n),
            "Kd": _as_dof_vector(Kd, n),
            "tau": _as_dof_vector(tau, n),
            "u_min": _as_dof_vector(u_min, n),
            "u_max": _as_dof_vector(u_max, n),
            "e_int_min": _as_dof_vector(e_int_min, n),
            "e_int_max": _as_dof_vector(e_int_max, n),
        }
        self.state.labels = [f"e_int{i}" for i in range(n)] + [
            f"d_filt{i}" for i in range(n)
        ]
        self.x0 = np.concatenate([np.zeros(n), _as_dof_vector(y_filt0, n)])

        self.add_error_ports(ports, n)
        self.add_output_port(
            "u", dim=n, function=self.ctl, dependencies=self.error_dependencies
        )

    def derivative_signal(self, u):
        """The signal the filtered derivative acts on: ``y`` (reference form) or ``-e``."""
        if self.port_layout == "error":
            return -self.error(u)
        return u[self.dof :]

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        n = self.dof

        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"])
        Kd = xp.asarray(params["Kd"])
        tau = xp.asarray(params["tau"])
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])
        e_int_min = xp.asarray(params["e_int_min"])
        e_int_max = xp.asarray(params["e_int_max"])

        e_int, m_filt = x[:n], x[n:]
        e = self.error(u)
        m = self.derivative_signal(u)
        dm_filt = (m - m_filt) / tau

        u_unsat = Kp * e + Ki * e_int - Kd * dm_filt

        # anti-windup: the integrator stops while the command is saturated
        stop_hi = xp.logical_and(u_unsat >= u_max, e > 0.0)
        stop_lo = xp.logical_and(u_unsat <= u_min, e < 0.0)
        stop_sat = xp.logical_or(stop_hi, stop_lo)
        de_int = xp.where(stop_sat, 0.0, e)

        stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
        stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
        stop_int = xp.logical_or(stop_int_hi, stop_int_lo)
        de_int = xp.where(stop_int, 0.0, de_int)

        return xp.concatenate([de_int, dm_filt])

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        n = self.dof

        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"])
        Kd = xp.asarray(params["Kd"])
        tau = xp.maximum(xp.asarray(params["tau"]), 1e-3)
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])

        e_int, m_filt = x[:n], x[n:]
        e = self.error(u)
        dm_filt = (self.derivative_signal(u) - m_filt) / tau

        u_cmd = Kp * e + Ki * e_int - Kd * dm_filt
        return xp.clip(u_cmd, u_min, u_max)
