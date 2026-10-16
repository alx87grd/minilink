"""Servos: a continuous PID loop with feedforward, and its heading twin that wraps the error."""

import numpy as np

from examples.projects.pathtracking.bicycle_los.los import wrap_pi
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


class Servo(DynamicSystem):
    """Scalar continuous PID-like servo with feedforward.

    State ``x = [int_e, meas_filt]``. Ports ``ref``, ``meas``,
    ``feedforward`` → ``cmd``.
    """

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        tau: float = 0.1,
        meas0: float = 0.0,
        name: str = "Servo",
    ):
        super().__init__(2)
        self.name = name

        # Parameters: the gains, the command and integrator limits, and the time
        # constant of the measurement filter [s]
        self.params = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "i_min": i_min,
            "i_max": i_max,
            "tau": tau,
        }

        # Signal labels and the initial state: no error integral, the filter at meas0
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        # Ports: the reference, the measurement and the feedforward in, the command out
        self.add_input_port("ref", nominal_value=np.array([0.0]))
        self.add_input_port("meas", nominal_value=np.array([0.0]))
        self.add_input_port("feedforward", nominal_value=np.array([0.0]))
        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_cmd,
            dependencies=("ref", "meas", "feedforward"),
        )

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]
        Ki = params["Ki"]
        Kd = params["Kd"]
        cmd_min = params["cmd_min"]
        cmd_max = params["cmd_max"]
        i_min = params["i_min"]
        i_max = params["i_max"]
        tau = params["tau"]

        int_e = float(x[0])
        meas_filt = float(x[1])
        ref = float(u[0])
        meas = float(u[1])
        feedforward = float(u[2])
        e = self.calculate_error(ref, meas, u)

        # ṁ_filt = (m − m_filt) / τ, with τ ≥ 10⁻³
        tau = max(tau, 1e-3)
        d_meas_filt = (meas - meas_filt) / tau

        # cmd = Kp e + Ki ∫e − Kd ṁ_filt + u_ff, before the limits
        cmd = Kp * e + Ki * int_e - Kd * d_meas_filt
        cmd += feedforward

        # Anti-windup: the integral holds while the command saturates in the error's
        # direction, and at its own limits i_min ≤ ∫e ≤ i_max
        stop_hi = (cmd >= cmd_max) and (e > 0)
        stop_lo = (cmd <= cmd_min) and (e < 0)
        d_int_e = 0.0 if (stop_hi or stop_lo) else e
        if int_e >= i_max and e > 0.0:
            d_int_e = 0.0
        elif int_e <= i_min and e < 0.0:
            d_int_e = 0.0

        dx = np.array([d_int_e, d_meas_filt], dtype=float)

        return dx

    def h_cmd(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        Kp = params["Kp"]
        Ki = params["Ki"]
        Kd = params["Kd"]
        cmd_min = params["cmd_min"]
        cmd_max = params["cmd_max"]
        tau = params["tau"]

        int_e = float(x[0])
        meas_filt = float(x[1])
        ref = float(u[0])
        meas = float(u[1])
        feedforward = float(u[2])
        e = self.calculate_error(ref, meas, u)

        # ṁ_filt = (m − m_filt) / τ, with τ ≥ 10⁻³
        tau = max(tau, 1e-3)
        d_meas_filt = (meas - meas_filt) / tau

        # cmd = Kp e + Ki ∫e − Kd ṁ_filt + u_ff
        cmd = Kp * e + Ki * int_e - Kd * d_meas_filt
        cmd += feedforward

        # Command limits: cmd_min ≤ cmd ≤ cmd_max
        cmd = np.clip(cmd, cmd_min, cmd_max)

        cmd_out = np.array([cmd], dtype=float)

        return cmd_out

    def calculate_error(self, ref: float, meas: float, u) -> float:
        """The tracking error ``e = ref − meas``; ``u``, the full input, serves a subclass."""
        e = float(ref - meas)

        return e


class HeadingServo(Servo):
    """Servo with heading error wrapped to ``[-pi, pi]``."""

    def calculate_error(self, ref: float, meas: float, u) -> float:
        # e = wrap(ref − meas): the short way round
        e = float(wrap_pi(ref - meas))

        return e
