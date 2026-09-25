"""Feedback profile: siso — decoupled PID loops, compensator or controller form."""

import numpy as np

from minilink.control.impedance import as_dof_vector
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

    The block carries the states its terms need — ``2 dof`` here. Setting
    ``Ki=0`` does **not** remove the integrator: the gain is tunable, so the
    state dimension cannot depend on it, and the dead integrator then shows
    up as an unobservable pole at the origin in every linearization. Use
    :class:`PI`, :class:`PD` or
    :class:`~minilink.control.output.ProportionalController` when a term is
    structurally absent, so poles and zeros match the hand calculation.

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
        Saturation limits per axis.
    """

    feedback_profile = "siso"

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
        if n <= 0:
            raise ValueError("dof must be positive")
        if self.has_filter and np.any(as_dof_vector(tau, n) <= 0.0):
            raise ValueError("tau must be positive (it divides the filter rate)")

        self.n_int = n if self.has_integrator else 0
        self.n_filt = n if self.has_filter else 0
        super().__init__(n=self.n_int + self.n_filt)
        self.dof = n
        self.name = type(self).__name__

        self.params = {"Kp": as_dof_vector(Kp, n)}
        labels, x0 = [], []
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

        self.state.labels = labels
        self.x0 = np.concatenate(x0) if x0 else np.zeros(0)

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
        n_int = self.n_int
        has_integrator, has_filter = self.has_integrator, self.has_filter
        e = self.error(u)
        e_int, m_filt = x[:n_int], x[n_int:]

        # The filtered-derivative rate and the unsaturated command it feeds
        dm_filt = self.filter_rate(u, m_filt, params, xp)
        u_unsat = self.command(e, e_int, dm_filt, params, xp)

        # dx = [ė_int; ṁ_filt], each term only when the block carries that state
        rates = []
        if has_integrator:
            rates.append(self.integrator_rate(e, e_int, u_unsat, params, xp))
        if has_filter:
            rates.append(dm_filt)
        dx = xp.concatenate(rates) if rates else xp.zeros(0)

        return dx

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        u_min, u_max = xp.asarray(params["u_min"]), xp.asarray(params["u_max"])
        n_int = self.n_int
        e = self.error(u)
        e_int, m_filt = x[:n_int], x[n_int:]
        dm_filt = self.filter_rate(u, m_filt, params, xp)

        # u = Kp e + Ki e_int − Kd ṁ_filt, then the actuator limits
        u_cmd = self.command(e, e_int, dm_filt, params, xp)
        u = xp.clip(u_cmd, u_min, u_max)

        return u

    # Internal machinery: the law, one term per carried state

    def filter_rate(self, u, m_filt, params, xp):
        """Rate of the filtered derivative state, or ``None`` without a filter."""
        if not self.has_filter:
            return None
        m = self.derivative_signal(u)
        tau = xp.asarray(params["tau"])

        # first-order filter on the measured signal: tau ṁ_filt = m − m_filt
        dm_filt = (m - m_filt) / tau

        return dm_filt

    def command(self, e, e_int, dm_filt, params, xp):
        """The unsaturated command, dropping the terms the block does not carry."""
        has_integrator, has_filter = self.has_integrator, self.has_filter
        Kp = xp.asarray(params["Kp"])

        # u = Kp e + Ki e_int − Kd ṁ_filt
        u_cmd = Kp * e
        if has_integrator:
            Ki = xp.asarray(params["Ki"])
            u_cmd = u_cmd + Ki * e_int
        if has_filter:
            Kd = xp.asarray(params["Kd"])
            u_cmd = u_cmd - Kd * dm_filt

        return u_cmd

    def integrator_rate(self, e, e_int, u_unsat, params, xp):
        """Rate of the integral state: the error, held at zero while saturated."""
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])
        e_int_min = xp.asarray(params["e_int_min"])
        e_int_max = xp.asarray(params["e_int_max"])

        # anti-windup: the integrator stops while the command is saturated
        stop_hi = xp.logical_and(u_unsat >= u_max, e > 0.0)
        stop_lo = xp.logical_and(u_unsat <= u_min, e < 0.0)
        stop_sat = xp.logical_or(stop_hi, stop_lo)

        # and while the integral state itself sits on one of its end stops
        stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
        stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
        stop_int = xp.logical_or(stop_int_hi, stop_int_lo)

        # ė_int = e, except when held
        de_int = xp.where(xp.logical_or(stop_sat, stop_int), 0.0, e)

        return de_int


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
    of the comparison. For an unfiltered proportional law use
    :class:`~minilink.control.output.ProportionalController`.
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
