"""Classical compensators: the decoupled ``P``, ``PI``, ``PD`` and ``PID`` laws on a tracking error."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.feedback import ErrorDriven
from minilink.core.system import DynamicSystem, System

###############################################################################
# Public API
###############################################################################


class P(ErrorDriven, System):
    """Decoupled proportional compensator ``u_i = Kp_i e_i``, the static member of the family.

    ``ports="error"`` (default) is the compensator form: one input ``e``, so
    ``P(...) @ plant`` inserts the Error block ``e = r - y`` and
    ``P(...) >> plant`` is the loop gain, with no pole added.
    ``ports="reference"`` is the controller form with inputs ``r`` and ``y``,
    the one a hand-wired diagram and the sampled loop (``P(...) % dt``) read.
    A coupled static gain on an output error is ``Gain(K) @ plant``.

    Parameters
    ----------
    Kp : float or vector
        Per-axis gain (a scalar broadcasts to all axes).
    dof : int
        Number of independent scalar loops (default 1).
    ports : {"error", "reference"}
        Input layout, see above.
    u_min, u_max : float or vector
        Actuator limits per axis, infinite by default.
    """

    def __init__(
        self,
        Kp: float = 1.0,
        *,
        dof: int = 1,
        ports: str = "error",
        u_min: float = -np.inf,
        u_max: float = np.inf,
    ):
        n = int(dof)
        require_positive_dof(n)

        super().__init__()
        self.dof = n
        self.name = "P"

        # Parameters
        self.params = {
            "Kp": as_dof_vector(Kp, n),
            "u_min": as_dof_vector(u_min, n),
            "u_max": as_dof_vector(u_max, n),
        }

        # Ports: the error (or the reference and the measurement) in, the command out
        self.port_layout = self.add_error_ports(ports, n)
        self.add_output_port(
            "u", dim=n, function=self.ctl, dependencies=self.error_dependencies
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        Kp = xp.asarray(params["Kp"])

        e = self.error(u)

        u_cmd = Kp * e

        # Actuator limits: u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u


class PID(ErrorDriven, DynamicSystem):
    """Decoupled PID with filtered derivative and anti-windup.

    ``ports="error"`` (default) is the compensator form: one input ``e``, so
    ``PID(...) @ plant`` inserts the Error block ``e = r - y`` and
    ``PID(...) >> plant`` is the loop gain. ``ports="reference"`` is the
    controller form with inputs ``r`` and ``y``. Per axis,

        u_i = Kp_i e_i + Ki_i e_int_i + Kd_i d_i,

    where ``d_i`` is the filtered derivative of ``e_i`` (error form) or of
    ``-y_i`` (reference form: no derivative kick on reference steps); the
    integrator stops while the command saturates.

    One measurement per axis, so the derivative is filtered (a pole at
    ``-1/tau``); the rate-measured members of the same family are in
    :mod:`minilink.control.impedance`.

    The block carries the states its terms need — ``2 dof`` here. Setting
    ``Ki=0`` does **not** remove the integrator: the gain is tunable, so the
    state dimension cannot depend on it, and the dead integrator then shows
    up as an unobservable pole at the origin in every linearization. Use
    :class:`PI`, :class:`PD` or :class:`P` when a term is structurally
    absent, so poles and zeros match the hand calculation.

    Parameters
    ----------
    Kp, Ki, Kd : float or vector
        Per-axis gains (a scalar broadcasts to all axes).
    tau : float or vector
        Derivative filter time constant per axis [s]; must be positive.
    dof : int
        Number of independent scalar loops (default 1).
    ports : {"error", "reference"}
        Input layout, see above.
    y_filt0 : float or vector
        Initial filtered state per axis (the filtered ``y``, or ``-e``).
    u_min, u_max, e_int_min, e_int_max : float or vector
        Saturation limits per axis, infinite by default.
    """

    #: Which terms this form carries — the state layout follows.
    has_integrator = True
    has_filter = True

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
        require_positive_dof(n)
        if self.has_filter:
            require_positive_tau(tau, n)

        # One integrator state and one filter state per axis, for the terms carried
        self.n_int = n if self.has_integrator else 0
        self.n_filt = n if self.has_filter else 0
        super().__init__(n=self.n_int + self.n_filt)
        self.dof = n
        self.name = type(self).__name__

        # Parameters, with the label and the initial value of each carried state
        self.params = {"Kp": as_dof_vector(Kp, n)}
        labels = []
        x0 = []
        if self.has_integrator:
            self.params["Ki"] = as_dof_vector(Ki, n)
            self.params["e_int_min"] = as_dof_vector(e_int_min, n)
            self.params["e_int_max"] = as_dof_vector(e_int_max, n)
            labels += [f"e_int{i}" for i in range(n)]
            x0.append(np.zeros(n))
        if self.has_filter:
            self.params["Kd"] = as_dof_vector(Kd, n)
            self.params["tau"] = as_dof_vector(tau, n)
            labels += [f"d_filt{i}" for i in range(n)]
            x0.append(as_dof_vector(y_filt0, n))
        self.params["u_min"] = as_dof_vector(u_min, n)
        self.params["u_max"] = as_dof_vector(u_max, n)

        # Signal labels and initial state
        self.state.labels = labels
        self.x0 = np.concatenate(x0) if x0 else np.zeros(0)

        # Ports: the error (or the reference and the measurement) in, the command out
        self.port_layout = self.add_error_ports(ports, n)
        self.add_output_port(
            "u", dim=n, function=self.ctl, dependencies=self.error_dependencies
        )

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n_int = self.n_int
        has_integrator = self.has_integrator
        has_filter = self.has_filter
        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"]) if has_integrator else None
        Kd = xp.asarray(params["Kd"]) if has_filter else None

        e = self.error(u)
        e_int = x[:n_int]
        m_filt = x[n_int:]

        # τ ṁ_filt = m − m_filt, on the signal m the derivative acts on
        dm_filt = None
        if has_filter:
            m = self.derivative_signal(u)
            dm_filt = filtered_rate(m, m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, unsaturated: the anti-windup reads it
        u_cmd = Kp * e
        if has_integrator:
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            u_cmd = u_cmd - Kd * dm_filt

        # ė_int = e, held while the command or the integral state sits on a limit
        de_int = None
        if has_integrator:
            de_int = held_integral(e, e_int, u_cmd, params, xp)

        # dx = [ė_int; ṁ_filt], each part only when the block carries that state
        rates = []
        if has_integrator:
            rates.append(de_int)
        if has_filter:
            rates.append(dm_filt)
        dx = xp.concatenate(rates) if rates else xp.zeros(0)

        return dx

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        n_int = self.n_int
        has_integrator = self.has_integrator
        has_filter = self.has_filter
        Kp = xp.asarray(params["Kp"])
        Ki = xp.asarray(params["Ki"]) if has_integrator else None
        Kd = xp.asarray(params["Kd"]) if has_filter else None

        e = self.error(u)
        e_int = x[:n_int]
        m_filt = x[n_int:]

        # τ ṁ_filt = m − m_filt, on the signal m the derivative acts on
        dm_filt = None
        if has_filter:
            m = self.derivative_signal(u)
            dm_filt = filtered_rate(m, m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, each term only when the block carries its state
        u_cmd = Kp * e
        if has_integrator:
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            u_cmd = u_cmd - Kd * dm_filt

        # Actuator limits: u_min ≤ u ≤ u_max
        u = clipped(u_cmd, params, xp)

        return u

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def derivative_signal(self, u):
        """The signal the filtered derivative acts on: ``y`` (reference form) or ``-e``."""
        if self.port_layout == "error":
            return -self.error(u)
        return u[self.dof :]


class PI(PID):
    """Decoupled PI compensator ``u_i = Kp_i e_i + Ki_i e_int_i``.

    :class:`PID` without the derivative term *and without its filter state*,
    so a PI loop has exactly one pole per axis at the origin and no filter
    pole. Same ports and anti-windup as :class:`PID`.
    """

    has_filter = False

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        *,
        dof: int = 1,
        ports: str = "error",
        u_min: float = -np.inf,
        u_max: float = np.inf,
        e_int_min: float = -np.inf,
        e_int_max: float = np.inf,
    ):
        super().__init__(
            Kp=Kp,
            Ki=Ki,
            dof=dof,
            ports=ports,
            u_min=u_min,
            u_max=u_max,
            e_int_min=e_int_min,
            e_int_max=e_int_max,
        )


class PD(PID):
    """Decoupled PD compensator ``u_i = Kp_i e_i + Kd_i d_i``, ``d`` filtered.

    :class:`PID` without the integrator, so a PD loop carries only the
    derivative-filter pole at ``-1/tau`` per axis and no pole at the origin.
    Without integral action the loop keeps a static error; that is the point
    of the comparison. An unfiltered ``Kp + Kd s`` is improper and has no
    block: filter the derivative (this block, or its frequency-domain twin
    :class:`~minilink.blocks.transfer_function.Lead`) or measure the rate
    (:class:`~minilink.control.impedance.Impedance`). For the
    proportional law alone use :class:`P`.
    """

    has_integrator = False

    def __init__(
        self,
        Kp: float = 1.0,
        Kd: float = 0.0,
        tau: float = 0.1,
        *,
        dof: int = 1,
        ports: str = "error",
        y_filt0=0.0,
        u_min: float = -np.inf,
        u_max: float = np.inf,
    ):
        super().__init__(
            Kp=Kp,
            Kd=Kd,
            tau=tau,
            dof=dof,
            ports=ports,
            y_filt0=y_filt0,
            u_min=u_min,
            u_max=u_max,
        )


###############################################################################
# Internal machinery
###############################################################################


# The building blocks every member of the classical family shares, the rate-measured
# laws of minilink.control.impedance included


def filtered_rate(m, m_filt, params, xp):
    """``tau ṁ_filt = m − m_filt``: the first-order filter whose state approximates ``ṁ``."""
    tau = xp.asarray(params["tau"])

    dm_filt = (m - m_filt) / tau

    return dm_filt


def held_integral(e, e_int, u_cmd, params, xp):
    """``ė_int = e``, held at zero on an axis whose command or integral state sits on a limit."""
    u_min = xp.asarray(params["u_min"])
    u_max = xp.asarray(params["u_max"])
    e_int_min = xp.asarray(params["e_int_min"])
    e_int_max = xp.asarray(params["e_int_max"])

    # Anti-windup: the integrator stops while the command is saturated
    stop_hi = xp.logical_and(u_cmd >= u_max, e > 0.0)
    stop_lo = xp.logical_and(u_cmd <= u_min, e < 0.0)
    stop_sat = xp.logical_or(stop_hi, stop_lo)

    # And while the integral state itself sits on one of its end stops
    stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
    stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
    stop_int = xp.logical_or(stop_int_hi, stop_int_lo)

    # ė_int = e, except when held
    held = xp.logical_or(stop_sat, stop_int)
    de_int = xp.where(held, 0.0, e)

    return de_int


def clipped(u_cmd, params, xp):
    """The command inside the actuator limits ``u_min ≤ u ≤ u_max``."""
    u_min = xp.asarray(params["u_min"])
    u_max = xp.asarray(params["u_max"])

    u = xp.clip(u_cmd, u_min, u_max)

    return u


# The refusals and the per-axis shapes of the family's parameters


def require_positive_dof(n):
    """Refuse a block without at least one axis."""
    if n <= 0:
        raise ValueError("dof must be positive")


def require_positive_tau(tau, n):
    """Refuse a derivative filter whose time constant is not positive on every axis."""
    if np.any(as_dof_vector(tau, n) <= 0.0):
        raise ValueError("tau must be positive (it divides the filter rate)")


def as_dof_vector(value, dof: int):
    """A scalar broadcast to ``dof`` axes, or a length-``dof`` vector as given."""
    given = np.asarray(value, dtype=float)
    arr = given.reshape(-1)
    if arr.size == 1:
        return np.full(dof, float(arr[0]))
    if arr.size != dof:
        raise ValueError(
            f"expected scalar or length-{dof} vector, got shape {given.shape}"
        )
    return arr


def as_gain_matrix(value, n: int):
    """A scalar or a per-axis vector as the diagonal ``(n, n)`` gain, or an ``(n, n)`` matrix as given."""
    arr = np.asarray(value, dtype=float)
    if arr.ndim <= 1:
        return np.diag(as_dof_vector(arr, n))
    if arr.shape != (n, n):
        raise ValueError(
            f"expected a scalar, a length-{n} vector or an ({n}, {n}) matrix, got shape {arr.shape}"
        )
    return arr
