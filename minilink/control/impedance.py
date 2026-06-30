"""Feedback profile: impedance — virtual spring-damper on [position; rate]."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, StaticSystem


def _as_dof_vector(value, dof: int):
    arr = np.asarray(value, dtype=float).reshape(-1)
    if arr.size == 1:
        return np.full(dof, float(arr[0]))
    if arr.size != dof:
        raise ValueError(
            f"expected scalar or length-{dof} vector, got shape {arr.shape}"
        )
    return arr


class ImpedanceController(StaticSystem):
    """Virtual spring-damper on ``[position; rate]``.

    The measurement port ``y`` carries ``[pos; rate]`` (dim ``2·dof``). Reference
    ``r`` has dim ``dof`` for regulation (zero rate reference) or ``2·dof`` for
    stacked ``[pos_d; vel_d]`` tracking.

    Parameters
    ----------
    dof : int
        Number of axes.
    tracking_ref : bool
        If ``True``, ``r`` port dim is ``2·dof`` (tracking). Otherwise ``dof``.
    y_labels, y_units, u_labels, u_units : sequence of str, optional
        Display metadata for ports.
    """

    feedback_profile = "impedance"

    def __init__(
        self,
        dof: int = 1,
        *,
        tracking_ref: bool = False,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        super().__init__()
        self.dof = int(dof)
        if self.dof <= 0:
            raise ValueError("dof must be positive")

        n = self.dof
        ref_dim = 2 * n if tracking_ref else n

        if y_labels is None:
            y_labels = [f"pos{i}" for i in range(n)] + [f"rate{i}" for i in range(n)]
        if y_units is None:
            y_units = [""] * (2 * n)
        if u_labels is None:
            u_labels = [f"u{i}" for i in range(n)]
        if u_units is None:
            u_units = [""] * n

        self.params = {
            "Kp": np.full(n, 10.0),
            "Kd": np.full(n, 1.0),
        }

        self.name = "Impedance Controller"

        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * n,
            nominal_value=np.zeros(2 * n),
            labels=list(y_labels),
            units=list(y_units),
        )
        self.add_output_port(
            "u",
            dim=n,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=list(u_labels),
            units=list(u_units),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        n = self.dof
        ref_dim = self.inputs["r"].dim
        r = u[:ref_dim]
        pos = u[ref_dim : ref_dim + n]
        rate = u[ref_dim + n : ref_dim + 2 * n]

        Kp = xp.asarray(_as_dof_vector(params["Kp"], n))
        Kd = xp.asarray(_as_dof_vector(params["Kd"], n))

        if ref_dim == n:
            u_cmd = Kp * (r - pos) - Kd * rate
        elif ref_dim == 2 * n:
            pos_d = r[:n]
            vel_d = r[n:]
            u_cmd = Kp * (pos_d - pos) + Kd * (vel_d - rate)
        else:
            raise ValueError(
                f"ImpedanceController r dim must be {n} or {2 * n}, got {ref_dim}"
            )

        return u_cmd


class ImpedanceIntegralController(DynamicSystem):
    """Impedance control with integral action on position error.

    Measurement ``y = [pos; rate]`` (dim ``2·dof``). Integrator acts on
    ``pos_d - pos`` (regulation) or the position component of tracking error.
    No anti-windup in this version.
    """

    feedback_profile = "impedance"

    def __init__(
        self,
        dof: int = 1,
        *,
        tracking_ref: bool = False,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        n = int(dof)
        if n <= 0:
            raise ValueError("dof must be positive")
        ref_dim = 2 * n if tracking_ref else n

        super().__init__(n=n)
        self.dof = n
        self.name = "Impedance Integral Controller"
        self.params = {
            "kp": np.full(n, 10.0),
            "ki": np.full(n, 1.0),
            "kd": np.full(n, 1.0),
        }
        self.state.labels = [f"e_int{i}" for i in range(n)]

        if y_labels is None:
            y_labels = [f"pos{i}" for i in range(n)] + [f"rate{i}" for i in range(n)]
        if y_units is None:
            y_units = [""] * (2 * n)
        if u_labels is None:
            u_labels = [f"u{i}" for i in range(n)]
        if u_units is None:
            u_units = [""] * n

        self.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
        self.add_input_port(
            "y",
            dim=2 * n,
            nominal_value=np.zeros(2 * n),
            labels=list(y_labels),
            units=list(y_units),
        )
        self.add_output_port(
            "u",
            dim=n,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=list(u_labels),
            units=list(u_units),
        )

    def _split_measurement(self, u):
        n = self.dof
        ref_dim = self.inputs["r"].dim
        r = u[:ref_dim]
        pos = u[ref_dim : ref_dim + n]
        rate = u[ref_dim + n : ref_dim + 2 * n]
        return ref_dim, r, pos, rate

    def _position_error(self, ref_dim, r, pos):
        n = self.dof
        if ref_dim == n:
            return r - pos
        if ref_dim == 2 * n:
            return r[:n] - pos
        raise ValueError(
            f"ImpedanceIntegralController r dim must be {n} or {2 * n}, got {ref_dim}"
        )

    def f(self, x, u, t=0, params=None):
        ref_dim, r, pos, _rate = self._split_measurement(u)
        e_pos = self._position_error(ref_dim, r, pos)
        return e_pos

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n = self.dof
        kp = xp.asarray(_as_dof_vector(params["kp"], n))
        ki = xp.asarray(_as_dof_vector(params["ki"], n))
        kd = xp.asarray(_as_dof_vector(params["kd"], n))

        e_int = x
        ref_dim, r, pos, rate = self._split_measurement(u)
        e_pos = self._position_error(ref_dim, r, pos)

        if ref_dim == n:
            u_cmd = kp * e_pos + ki * e_int - kd * rate
        else:
            vel_d = r[n:]
            u_cmd = kp * e_pos + ki * e_int + kd * (vel_d - rate)

        return u_cmd
