"""Continuous servo blocks copied from Simon's PID logic."""

import numpy as np

from examples.projects.simon_bicycle_los.los import wrap_pi
from minilink.core.system import DynamicSystem


class Servo(DynamicSystem):
    """Scalar continuous PID-like servo with feedforward.

    State ``x = [int_e, meas_filt]``. Ports are ``ref``, ``meas``,
    ``feedforward`` -> ``cmd``.
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
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        self.add_input_port("ref", nominal_value=np.array([0.0]))
        self.add_input_port("meas", nominal_value=np.array([0.0]))
        self.add_input_port("feedforward", nominal_value=np.array([0.0]))

        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_cmd,
            dependencies=("ref", "meas", "feedforward"),
        )
        self.add_output_port(
            "logs",
            dim=2,
            function=self.data_signal,
            dependencies=("ref", "meas", "feedforward"),
        )
        self.add_output_port(
            "servo_internal",
            dim=4,
            function=self.internal_values,
            dependencies=("ref", "meas", "feedforward"),
        )

    def data_signal(self, x, u, t=0.0, params=None):
        ref = float(u[0])
        meas = float(u[1])
        return np.array([ref, meas], dtype=float)

    def calculate_error(self, ref: float, meas: float, u) -> float:
        e = ref - meas
        return float(e)

    def compute_cmd(self, e, int_e, d_filt, p, feedforward=0.0):
        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
        cmd += feedforward
        return cmd

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas, feedforward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)
        tau = max(p["tau"], 1e-3)
        d_meas_filt = (meas - meas_filt) / tau

        cmd = self.compute_cmd(e, int_e, d_meas_filt, p=p, feedforward=feedforward)
        stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
        stop_lo = (cmd <= p["cmd_min"]) and (e < 0)
        d_int_e = 0.0 if (stop_hi or stop_lo) else e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        return np.array([d_int_e, d_meas_filt], dtype=float)

    def h_cmd(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas, feedforward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)
        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau
        cmd = self.compute_cmd(e, int_e, d_filt, p=p, feedforward=feedforward)
        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd], dtype=float)

    def internal_values(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas, feedforward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)
        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau
        cmd = self.compute_cmd(e, int_e, d_filt, p=p, feedforward=feedforward)
        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([e, d_filt, int_e, cmd], dtype=float)


class AngleWrappedServo(Servo):
    """Servo variant for heading errors."""

    def calculate_error(self, ref: float, meas: float, u) -> float:
        e = wrap_pi(ref - meas)
        return float(e)
