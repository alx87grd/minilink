"""Virtual spring-damper laws on a measured ``[q; dq]``: the rate-measured members of the classical family.

What is measured decides the derivative: these blocks read the rate, so they carry
no filter state, where :class:`~minilink.control.siso.PD` and
:class:`~minilink.control.siso.PID` filter the derivative of their one measurement.
The stiffness ``Kp`` and damping ``Kd`` are ``n×n`` matrices (a scalar or a per-axis
vector is diagonal), so axes may couple; the integral gain stays per axis, since the
anti-windup hold pairs a command with its own error.
"""

import numpy as np

from minilink.control.siso import as_dof_vector, as_gain_matrix, clipped, held_integral
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, System


class ImpedanceController(System):
    """Virtual spring-damper ``u = Kp (q_d − q) + Kd (dq_d − dq)`` on a measured ``[q; dq]``.

    The measurement port ``y`` carries ``[q; dq]`` (dim ``2·dof``). The reference
    ``r`` is ``q_d`` (dim ``dof``, regulation: ``dq_d = 0``) or ``[q_d; dq_d]``
    (dim ``2·dof``, ``tracking_ref=True``). No state: the loop adds no pole.

    Parameters
    ----------
    dof : int
        Number of axes.
    tracking_ref : bool
        If ``True``, ``r`` is ``[q_d; dq_d]``; otherwise ``q_d`` alone.
    Kp, Kd : float, vector or (dof, dof) array
        Stiffness and damping: a scalar or a per-axis vector is a diagonal
        matrix, a full matrix couples the axes. Stored in ``self.params`` as
        ``(dof, dof)`` arrays, the shape a value assigned there later must have.
    u_min, u_max : float or vector
        Actuator limits per axis.
    y_labels, y_units, u_labels, u_units : sequence of str, optional
        Display metadata for ports.
    """

    plot_space = "error_qdq"  # plot_control_law sweeps the (e, de) plane

    def __init__(
        self,
        dof: int = 1,
        *,
        tracking_ref: bool = False,
        Kp=10.0,
        Kd=1.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        n = int(dof)
        if n <= 0:
            raise ValueError("dof must be positive")
        super().__init__()
        self.dof = n
        self.name = "Impedance Controller"
        self.params = {
            "Kp": as_gain_matrix(Kp, n),
            "Kd": as_gain_matrix(Kd, n),
            "u_min": as_dof_vector(u_min, n),
            "u_max": as_dof_vector(u_max, n),
        }

        declare_impedance_ports(
            self, n, tracking_ref, y_labels, y_units, u_labels, u_units
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)
        Kp, Kd = xp.asarray(params["Kp"]), xp.asarray(params["Kd"])
        e, de = tracking_errors(u, self.dof, self.inputs["r"].dim)

        # u = Kp e + Kd ė, then the actuator limits
        u_cmd = Kp @ e + Kd @ de
        u = clipped(u_cmd, params, xp)

        return u


class ImpedanceIntegralController(DynamicSystem):
    """Spring-damper with integral action, ``u = Kp e + Ki ∫e + Kd ė``, on a measured ``[q; dq]``.

    :class:`ImpedanceController` plus one integral state per axis on the
    position error ``e = q_d − q``, held while the command or the integral
    state sits on a limit (the anti-windup of
    :class:`~minilink.control.siso.PID`). Same ports and references.

    Parameters
    ----------
    dof, tracking_ref, Kp, Kd, u_min, u_max : as :class:`ImpedanceController`.
    Ki : float or vector
        Integral gain per axis.
    e_int_min, e_int_max : float or vector
        End stops of the integral state per axis.
    y_labels, y_units, u_labels, u_units : sequence of str, optional
        Display metadata for ports.
    """

    plot_space = "error_qdq"  # plot_control_law sweeps the (e, de) plane

    def __init__(
        self,
        dof: int = 1,
        *,
        tracking_ref: bool = False,
        Kp=10.0,
        Ki=1.0,
        Kd=1.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
        e_int_min: float = -np.inf,
        e_int_max: float = np.inf,
        y_labels=None,
        y_units=None,
        u_labels=None,
        u_units=None,
    ):
        n = int(dof)
        if n <= 0:
            raise ValueError("dof must be positive")
        super().__init__(n=n)
        self.dof = n
        self.name = "Impedance Integral Controller"
        self.params = {
            "Kp": as_gain_matrix(Kp, n),
            "Ki": as_dof_vector(Ki, n),
            "Kd": as_gain_matrix(Kd, n),
            "u_min": as_dof_vector(u_min, n),
            "u_max": as_dof_vector(u_max, n),
            "e_int_min": as_dof_vector(e_int_min, n),
            "e_int_max": as_dof_vector(e_int_max, n),
        }
        self.state.labels = [f"e_int{i}" for i in range(n)]

        declare_impedance_ports(
            self, n, tracking_ref, y_labels, y_units, u_labels, u_units
        )

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        e_int = x
        e, de = tracking_errors(u, self.dof, self.inputs["r"].dim)

        # ė_int = e, except while the unsaturated command holds it
        u_cmd = self.command(e, e_int, de, params, xp)
        de_int = held_integral(e, e_int, u_cmd, params, xp)

        return de_int

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        e_int = x
        e, de = tracking_errors(u, self.dof, self.inputs["r"].dim)

        # u = Kp e + Ki e_int + Kd ė, then the actuator limits
        u_cmd = self.command(e, e_int, de, params, xp)
        u = clipped(u_cmd, params, xp)

        return u

    def command(self, e, e_int, de, params, xp):
        """The unsaturated command ``Kp e + Ki e_int + Kd ė``."""
        Kp, Kd = xp.asarray(params["Kp"]), xp.asarray(params["Kd"])
        Ki = xp.asarray(params["Ki"])

        u_cmd = Kp @ e + Ki * e_int + Kd @ de

        return u_cmd


# =============================================================================
# Internal machinery
# =============================================================================


def tracking_errors(u, n, ref_dim):
    """``(e, ė)`` from the flat input ``[r; q; dq]``: ``q_d − q`` and ``dq_d − dq``, ``dq_d = 0`` unless ``r`` carries it."""
    r = u[:ref_dim]
    q = u[ref_dim : ref_dim + n]
    dq = u[ref_dim + n : ref_dim + 2 * n]
    q_d = r[:n]
    dq_d = r[n:] if ref_dim == 2 * n else 0.0

    e = q_d - q
    de = dq_d - dq

    return e, de


def declare_impedance_ports(
    block, n, tracking_ref, y_labels, y_units, u_labels, u_units
):
    """The ports ``r`` (``q_d``, or ``[q_d; dq_d]``), ``y = [q; dq]`` and ``u`` of a spring-damper law."""
    ref_dim = 2 * n if tracking_ref else n
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
