"""Generic linear control-law blocks.

Plant-agnostic feedback laws following the standard port naming: ``r``
reference, ``y`` measurement, ``u`` control command.

- :class:`ProportionalController` — output-error proportional ``u = K (r - y)`` (SISO or MIMO).
- :class:`PDController` — scalar PD law on a (position, rate) measurement.
- :class:`PIDController` — continuous-time PID with an integral state on ``[y, dy_dt]``.
- :class:`LinearStateFeedbackController` — full-state feedback ``u = ubar - K (x - r)``
  (the block returned by :func:`minilink.control.lqr.lqr`).
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, StaticSystem


class PDController(StaticSystem):
    """PD controller on a (position, rate) measurement.

    The measurement port ``y`` carries ``[position, rate]`` (for example a
    mechanical system exposing ``[q, dq]``); the reference ``r`` targets the
    position only.

    Parameters
    ----------
    y_labels, y_units : sequence of str, optional
        Display metadata for the measurement port (defaults are generic;
        pass e.g. ``("theta", "theta_dot")`` / ``("rad", "rad/s")`` for a
        pendulum).
    u_labels, u_units : sequence of str, optional
        Display metadata for the command port.
    """

    def __init__(
        self,
        y_labels=("position", "rate"),
        y_units=("", ""),
        u_labels=("force",),
        u_units=("",),
    ):
        super().__init__()

        self.params = {
            "Kp": 10.0,
            "Kd": 1.0,
        }

        self.name = "PD Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port(
            "y",
            nominal_value=[0.0, 0.0],
            labels=list(y_labels),
            units=list(y_units),
        )
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=list(u_labels),
            units=list(u_units),
        )

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]
        Kd = params["Kd"]

        xp = array_module(u)

        r = u[0]
        y = u[1]
        dy_dt = u[2]

        u = Kp * (r - y) - Kd * dy_dt

        return xp.array([u])


class ProportionalController(StaticSystem):
    """Output-error proportional control ``u = K (r - y)`` (SISO or MIMO).

    ``K`` is a scalar (SISO, a ``1×1`` gain) or an ``(m, p)`` matrix mapping a
    ``p``-vector tracking error to an ``m``-vector command; reference ``r`` and
    measurement ``y`` share dimension ``p``. The gain lives in ``params["K"]``
    (always stored as a matrix) so it can be tuned or differentiated.

    This is *output* feedback on the measured ``y`` port (it composes with the
    ``@`` operator). For *state* feedback with a feedforward offset — the form
    LQR produces — use :class:`LinearStateFeedbackController`.
    """

    def __init__(self, K=1.0):
        super().__init__()
        self.name = "P Controller"

        K = np.atleast_2d(np.asarray(K, dtype=float))
        self.params = {"K": K}
        m, p = K.shape

        self.add_input_port("r", dim=p)
        self.add_input_port("y", dim=p)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]

        # input ports concatenate into u = [r, y]
        p = K.shape[1]
        r = u[:p]
        y = u[p:]

        return K @ (r - y)


class PIDController(DynamicSystem):
    """Continuous-time PID on a ``[y, dy_dt]`` measurement.

    The measurement port ``y`` carries ``[y, dy_dt]`` and ``r`` targets the
    measured variable. The single state is the integral of the tracking error:

        u = kp e + ki e_int - kd dy_dt,   e = r - y,   e_int = ∫ e dt

    No anti-windup in this version.
    """

    def __init__(self):
        super().__init__(n=1)
        self.name = "PID Controller"
        self.params = {"kp": 10.0, "ki": 1.0, "kd": 1.0}
        self.state.labels = ["e_int"]

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", dim=2, nominal_value=[0.0, 0.0])
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def f(self, x, u, t=0, params=None):
        xp = array_module(u)

        # inputs ports
        r = u[0]
        y = u[1]

        # error
        e = r - y

        # integral state derivative
        de_int = e

        return xp.array([de_int])

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        kp = params["kp"]
        ki = params["ki"]
        kd = params["kd"]

        xp = array_module(x)

        # internal states
        e_int = x[0]

        # inputs ports
        r = u[0]
        y = u[1]
        dy_dt = u[2]

        # error
        e = r - y

        # pid control law
        u = kp * e + ki * e_int - kd * dy_dt

        return xp.array([u])


class LinearStateFeedbackController(StaticSystem):
    """Full-state feedback ``u = ubar - K (x - r)``.

    Ports: the plant state ``x`` (dimension ``n``) and a reference ``r`` (also
    ``n``); when ``r`` is left unconnected it holds ``xbar``, so the law
    regulates to ``xbar``. ``K`` has shape ``(m, n)``; ``ubar`` is the
    feedforward command. This is the block :func:`minilink.control.lqr.lqr`
    returns.

    This is *state* feedback on the full ``x`` port. For *output* feedback on a
    measured ``y`` (and ``@``-operator wiring), use :class:`ProportionalController`.
    """

    def __init__(self, K, xbar=None, ubar=None):
        super().__init__()
        self.name = "ctl"

        K = np.atleast_2d(np.asarray(K, dtype=float))
        m, n = K.shape
        xbar = (
            np.zeros(n) if xbar is None else np.asarray(xbar, dtype=float).reshape(-1)
        )
        ubar = (
            np.zeros(m) if ubar is None else np.asarray(ubar, dtype=float).reshape(-1)
        )
        self.params = {"K": K, "ubar": ubar}

        self.add_input_port("x", dim=n)
        self.add_input_port("r", dim=n, nominal_value=xbar)
        self.add_output_port("u", dim=m, function=self.ctl, dependencies=("x", "r"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        K = params["K"]
        ubar = params["ubar"]

        # input ports concatenate into u = [x, r]
        n = K.shape[1]
        x_meas = u[:n]
        r = u[n:]

        return ubar - K @ (x_meas - r)
