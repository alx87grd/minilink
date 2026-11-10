"""Virtual spring-damper laws on a measured ``[q; dq]``: the rate-measured members of the classical family."""

import numpy as np

from minilink.control.siso import (
    as_axis_vector,
    as_gain_matrix,
    clipped,
    held_integral,
    require_positive_dim,
)
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, System

###############################################################################
# Public API
###############################################################################


class Impedance(System):
    """Virtual spring-damper ``u = Kp (q_d − q) + Kd (dq_d − dq)`` on a measured ``[q; dq]``.

    The measurement port ``y`` carries ``[q; dq]`` (dim ``2·dim``). The reference
    ``r`` is ``q_d`` (dim ``dim``, regulation: ``dq_d = 0``) or ``[q_d; dq_d]``
    (dim ``2·dim``, ``tracking_ref=True``). No state: the loop adds no pole.
    The signature is the PID family's: the gains first, ``Impedance(10.0, 1.0)``,
    the channel count ``dim`` a keyword.

    What is measured decides the derivative: this block reads the rate, so it
    carries no filter state, where :class:`~minilink.control.siso.PD` and
    :class:`~minilink.control.siso.PID` filter the derivative of their one
    measurement.

    Parameters
    ----------
    Kp, Kd : float, vector or (dim, dim) array
        Stiffness and damping: a scalar or a per-axis vector is a diagonal
        matrix, a full matrix couples the axes. Stored in ``self.params`` as
        ``(dim, dim)`` arrays, the shape a value assigned there later must have.
    dim : int
        Number of axes.
    tracking_ref : bool
        If ``True``, ``r`` is ``[q_d; dq_d]``; otherwise ``q_d`` alone.
    u_min, u_max : float or vector
        Actuator limits per axis.
    y_labels, y_units, u_labels, u_units : sequence of str, optional
        Display metadata for ports.
    """

    plot_space = "error_qdq"  # plot_control_law sweeps the (e, de) plane

    def __init__(
        self,
        Kp=10.0,
        Kd=1.0,
        *,
        dim: int = 1,
        tracking_ref: bool = False,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        n = int(dim)
        require_positive_dim(n)

        super().__init__()
        self.dim = n

        # Parameters: stiffness and damping as (n, n) matrices, the limits per axis
        self.params = {
            "Kp": as_gain_matrix(Kp, n),
            "Kd": as_gain_matrix(Kd, n),
            "u_min": as_axis_vector(u_min, n),
            "u_max": as_axis_vector(u_max, n),
        }

        # Ports: the reference and the measurement [q; dq] in, the command out
        declare_impedance_ports(
            self, n, tracking_ref, y_labels, y_units, u_labels, u_units
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])
        n = self.dim
        ref_dim = self.inputs["r"].dim

        e, de = tracking_errors(u, n, ref_dim)

        # u = Kp e + Kd ė
        u_cmd = Kp @ e + Kd @ de

        # Actuator limits: u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u


class IntegralImpedance(DynamicSystem):
    """Spring-damper with integral action, ``u = Kp e + Ki ∫e + Kd ė``, on a measured ``[q; dq]``.

    :class:`Impedance` plus one integral state per axis on the
    position error ``e = q_d − q``, held while the command or the integral
    state sits on a limit (the anti-windup of
    :class:`~minilink.control.siso.PID`). Same ports and references.

    Parameters
    ----------
    Kp, Kd, dim, tracking_ref, u_min, u_max : as :class:`Impedance`.
    Ki : float or vector
        Integral gain per axis, never a matrix: the anti-windup hold pairs a
        command with its own error.
    e_int_min, e_int_max : float or vector
        End stops of the integral state per axis.
    y_labels, y_units, u_labels, u_units : sequence of str, optional
        Display metadata for ports.
    """

    plot_space = "error_qdq"  # plot_control_law sweeps the (e, de) plane

    def __init__(
        self,
        Kp=10.0,
        Ki=1.0,
        Kd=1.0,
        *,
        dim: int = 1,
        tracking_ref: bool = False,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        e_int_min: float = -np.inf,
        e_int_max: float = np.inf,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        n = int(dim)
        require_positive_dim(n)

        super().__init__(state_dim=n)
        self.dim = n

        # Parameters: stiffness and damping as (n, n) matrices, the integral gain,
        # the limits and the end stops per axis
        self.params = {
            "Kp": as_gain_matrix(Kp, n),
            "Ki": as_axis_vector(Ki, n),
            "Kd": as_gain_matrix(Kd, n),
            "u_min": as_axis_vector(u_min, n),
            "u_max": as_axis_vector(u_max, n),
            "e_int_min": as_axis_vector(e_int_min, n),
            "e_int_max": as_axis_vector(e_int_max, n),
        }

        # Signal labels: one integral state per axis
        self.state.labels = [f"e_int{i}" for i in range(n)]

        # Ports: the reference and the measurement [q; dq] in, the command out
        declare_impedance_ports(
            self, n, tracking_ref, y_labels, y_units, u_labels, u_units
        )

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n = self.dim
        ref_dim = self.inputs["r"].dim
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])
        Ki = xp.asarray(params["Ki"])

        e_int = x
        e, de = tracking_errors(u, n, ref_dim)

        # u = Kp e + Ki e_int + Kd ė, unsaturated: the anti-windup reads it
        u_cmd = Kp @ e + Ki * e_int + Kd @ de

        # ė_int = e, held while the command or the integral state sits on a limit
        de_int = held_integral(e, e_int, u_cmd, params, xp)

        return de_int

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n = self.dim
        ref_dim = self.inputs["r"].dim
        Kp = xp.asarray(params["Kp"])
        Kd = xp.asarray(params["Kd"])
        Ki = xp.asarray(params["Ki"])

        e_int = x
        e, de = tracking_errors(u, n, ref_dim)

        # u = Kp e + Ki e_int + Kd ė
        u_cmd = Kp @ e + Ki * e_int + Kd @ de

        # Actuator limits: u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u


###############################################################################
# Internal machinery
###############################################################################


def declare_impedance_ports(
    block, n, tracking_ref, y_labels, y_units, u_labels, u_units
):
    """The ports ``r`` (``q_d``, or ``[q_d; dq_d]``), ``y = [q; dq]`` and ``u`` of a spring-damper law."""
    ref_dim = 2 * n if tracking_ref else n

    # Default display metadata: a position and a rate per axis, a command per axis
    if y_labels is None:
        y_labels = [f"pos{i}" for i in range(n)] + [f"rate{i}" for i in range(n)]
    if y_units is None:
        y_units = [""] * (2 * n)
    if u_labels is None:
        u_labels = [f"u{i}" for i in range(n)]
    if u_units is None:
        u_units = [""] * n

    block.add_input_port("r", dim=ref_dim, nominal_value=np.zeros(ref_dim))
    block.add_input_port(
        "y",
        dim=2 * n,
        nominal_value=np.zeros(2 * n),
        labels=list(y_labels),
        units=list(y_units),
    )
    block.add_output_port(
        "u",
        dim=n,
        function=block.ctl,
        dependencies=("r", "y"),
        labels=list(u_labels),
        units=list(u_units),
    )


def tracking_errors(u, n, ref_dim):
    """``(e, ė)`` from the flat input ``[r; q; dq]``, as :func:`reference_errors`."""
    r = u[:ref_dim]
    q = u[ref_dim : ref_dim + n]
    dq = u[ref_dim + n : ref_dim + 2 * n]

    e, de = reference_errors(r, q, dq)

    return e, de


def reference_errors(r, q, dq):
    """``(e, ė) = (q_d − q, dq_d − dq)`` of a measured ``(q, dq)``, joint or task coordinates; ``r`` is ``q_d`` (``dq_d = 0``) or ``[q_d; dq_d]``."""
    n = q.shape[0]

    # The reference [q_d; dq_d], or q_d alone with dq_d = 0 (regulation)
    q_d = r[:n]
    dq_d = r[n:] if r.shape[0] == 2 * n else 0.0

    e = q_d - q
    de = dq_d - dq

    return e, de
