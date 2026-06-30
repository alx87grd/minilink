"""Feedback profile: siso — decoupled dynamic loops on scalar measurements."""

import numpy as np

from minilink.control.impedance import _as_dof_vector
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem


class FilteredController(DynamicSystem):
    """Decoupled SISO PID with filtered derivative and anti-windup.

    Each axis has its own integrator and filtered-measurement state. The
    derivative acts on the filtered measurement:

        u_i = kp_i e_i + ki_i e_int_i - kd_i dy_filt_i,   e_i = r_i - y_i

    Parameters
    ----------
    dof : int
        Number of independent scalar loops (default 1).
    kp, ki, kd : float or vector
        Per-axis gains (scalar broadcasts to all axes).
    tau : float or vector
        Filter time constant per axis [s].
    y_filt0 : float or vector
        Initial filtered measurement state(s).
    u_min, u_max, e_int_min, e_int_max : float or vector
        Saturation limits per axis.
    """

    feedback_profile = "siso"

    def __init__(
        self,
        dof: int = 1,
        kp: float = 1.0,
        ki: float = 0.0,
        kd: float = 0.0,
        tau: float = 0.1,
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
        self.name = "Filtered Controller"

        self.params = {
            "kp": _as_dof_vector(kp, n),
            "ki": _as_dof_vector(ki, n),
            "kd": _as_dof_vector(kd, n),
            "tau": _as_dof_vector(tau, n),
            "u_min": _as_dof_vector(u_min, n),
            "u_max": _as_dof_vector(u_max, n),
            "e_int_min": _as_dof_vector(e_int_min, n),
            "e_int_max": _as_dof_vector(e_int_max, n),
        }
        self.state.labels = [f"e_int{i}" for i in range(n)] + [
            f"y_filt{i}" for i in range(n)
        ]
        y0 = _as_dof_vector(y_filt0, n)
        self.x0 = np.concatenate([np.zeros(n), y0])

        self.add_input_port("r", dim=n, nominal_value=np.zeros(n))
        self.add_input_port("y", dim=n, nominal_value=np.zeros(n))
        self.add_output_port("u", dim=n, function=self.ctl, dependencies=("r", "y"))

    def _split_state(self, x):
        n = self.dof
        return x[:n], x[n:]

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        n = self.dof

        kp = xp.asarray(params["kp"])
        ki = xp.asarray(params["ki"])
        kd = xp.asarray(params["kd"])
        tau = xp.asarray(params["tau"])
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])
        e_int_min = xp.asarray(params["e_int_min"])
        e_int_max = xp.asarray(params["e_int_max"])

        e_int, y_filt = self._split_state(x)
        r = u[:n]
        y = u[n:]

        e = r - y
        dy_filt = (y - y_filt) / tau

        u_unsat = kp * e + ki * e_int - kd * dy_filt

        stop_hi = xp.logical_and(u_unsat >= u_max, e > 0.0)
        stop_lo = xp.logical_and(u_unsat <= u_min, e < 0.0)
        stop_sat = xp.logical_or(stop_hi, stop_lo)
        de_int = xp.where(stop_sat, 0.0, e)

        stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
        stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
        stop_int = xp.logical_or(stop_int_hi, stop_int_lo)
        de_int = xp.where(stop_int, 0.0, de_int)

        return xp.concatenate([de_int, dy_filt])

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        n = self.dof

        kp = xp.asarray(params["kp"])
        ki = xp.asarray(params["ki"])
        kd = xp.asarray(params["kd"])
        tau = xp.maximum(xp.asarray(params["tau"]), 1e-3)
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])

        e_int, y_filt = self._split_state(x)
        r = u[:n]
        y = u[n:]

        e = r - y
        dy_filt = (y - y_filt) / tau
        u_cmd = kp * e + ki * e_int - kd * dy_filt
        return xp.clip(u_cmd, u_min, u_max)
