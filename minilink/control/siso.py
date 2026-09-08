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
        if self.has_filter and np.any(_as_dof_vector(tau, n) <= 0.0):
            raise ValueError("tau must be positive (it divides the filter rate)")

        self.n_int = n if self.has_integrator else 0
        self.n_filt = n if self.has_filter else 0
        super().__init__(n=self.n_int + self.n_filt)
        self.dof = n
        self.name = type(self).__name__

        self.params = {"Kp": _as_dof_vector(Kp, n)}
        labels, x0 = [], []
        if self.has_integrator:
            self.params["Ki"] = _as_dof_vector(Ki, n)
            self.params["e_int_min"] = _as_dof_vector(e_int_min, n)
            self.params["e_int_max"] = _as_dof_vector(e_int_max, n)
            labels += [f"e_int{i}" for i in range(n)]
            x0.append(np.zeros(n))
        if self.has_filter:
            self.params["Kd"] = _as_dof_vector(Kd, n)
            self.params["tau"] = _as_dof_vector(tau, n)
            labels += [f"d_filt{i}" for i in range(n)]
            x0.append(_as_dof_vector(y_filt0, n))
        self.params["u_min"] = _as_dof_vector(u_min, n)
        self.params["u_max"] = _as_dof_vector(u_max, n)

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

        e = self.error(u)
        e_int, m_filt = x[: self.n_int], x[self.n_int :]
        dm_filt = self._filter_rate(u, m_filt, params, xp)
        u_unsat = self._command(e, e_int, dm_filt, params, xp)

        rates = []
        if self.has_integrator:
            rates.append(self._integrator_rate(e, e_int, u_unsat, params, xp))
        if self.has_filter:
            rates.append(dm_filt)
        return xp.concatenate(rates) if rates else xp.zeros(0)

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)

        e = self.error(u)
        e_int, m_filt = x[: self.n_int], x[self.n_int :]
        dm_filt = self._filter_rate(u, m_filt, params, xp)
        u_cmd = self._command(e, e_int, dm_filt, params, xp)
        return xp.clip(u_cmd, xp.asarray(params["u_min"]), xp.asarray(params["u_max"]))

    # -- the law, one term per carried state -------------------------------

    def _filter_rate(self, u, m_filt, params, xp):
        """``d(m_filt)/dt = (m - m_filt) / tau``, or ``None`` without a filter."""
        if not self.has_filter:
            return None
        return (self.derivative_signal(u) - m_filt) / xp.asarray(params["tau"])

    def _command(self, e, e_int, dm_filt, params, xp):
        """``u = Kp e + Ki e_int - Kd dm_filt``, dropping the absent terms."""
        u_cmd = xp.asarray(params["Kp"]) * e
        if self.has_integrator:
            u_cmd = u_cmd + xp.asarray(params["Ki"]) * e_int
        if self.has_filter:
            u_cmd = u_cmd - xp.asarray(params["Kd"]) * dm_filt
        return u_cmd

    def _integrator_rate(self, e, e_int, u_unsat, params, xp):
        """``de_int = e``, held at zero while the command or the integrator saturates."""
        u_min = xp.asarray(params["u_min"])
        u_max = xp.asarray(params["u_max"])
        e_int_min = xp.asarray(params["e_int_min"])
        e_int_max = xp.asarray(params["e_int_max"])

        # anti-windup: the integrator stops while the command is saturated
        stop_hi = xp.logical_and(u_unsat >= u_max, e > 0.0)
        stop_lo = xp.logical_and(u_unsat <= u_min, e < 0.0)
        stop_sat = xp.logical_or(stop_hi, stop_lo)

        stop_int_hi = xp.logical_and(e_int >= e_int_max, e > 0.0)
        stop_int_lo = xp.logical_and(e_int <= e_int_min, e < 0.0)
        stop_int = xp.logical_or(stop_int_hi, stop_int_lo)

        return xp.where(xp.logical_or(stop_sat, stop_int), 0.0, e)


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
