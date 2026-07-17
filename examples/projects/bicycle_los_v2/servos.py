"""Fused velocity / heading / yaw-rate servos for the bicycle cascade."""

import numpy as np

from examples.projects.bicycle_los_v2.path_tracking import wrap_pi
from minilink.core.system import DynamicSystem


def _pid_gains(Kp, Ki, Kd, cmd_min, cmd_max, i_min, i_max, tau=0.1):
    return {
        "Kp": float(Kp),
        "Ki": float(Ki),
        "Kd": float(Kd),
        "cmd_min": float(cmd_min),
        "cmd_max": float(cmd_max),
        "i_min": float(i_min),
        "i_max": float(i_max),
        "tau": float(tau),
    }


def _pid_cmd(e, int_e, d_filt, p, feedforward=0.0):
    cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
    cmd += feedforward
    return cmd


def _pid_deriv(e, int_e, meas, meas_filt, p, feedforward=0.0):
    tau = max(p["tau"], 1e-3)
    d_meas_filt = (meas - meas_filt) / tau
    cmd = _pid_cmd(e, int_e, d_meas_filt, p, feedforward=feedforward)
    stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
    stop_lo = (cmd <= p["cmd_min"]) and (e < 0)
    d_int_e = 0.0 if (stop_hi or stop_lo) else e
    if int_e >= p["i_max"] and e > 0.0:
        d_int_e = 0.0
    elif int_e <= p["i_min"] and e < 0.0:
        d_int_e = 0.0
    return d_int_e, d_meas_filt


def _pid_output(e, int_e, meas, meas_filt, p, feedforward=0.0):
    tau = max(p["tau"], 1e-3)
    d_filt = (meas - meas_filt) / tau
    cmd = _pid_cmd(e, int_e, d_filt, p, feedforward=feedforward)
    return float(np.clip(cmd, p["cmd_min"], p["cmd_max"]))


class Servos(DynamicSystem):
    """Velocity, heading, and yaw-rate cascade ending at force and steer.

    State ``x = [int_v, filt_v, int_h, filt_h, int_r, filt_r]``.

    Internal cascade::

        accel_ref   = velocity_PID(vx_ref, vx)
        yawrate_ref = heading_PID(heading_ref, theta)   # angle wrap
        delta_fb    = yawrate_PID(yawrate_ref, yawrate)
        F_rear      = mass * accel_ref
        delta_ff    = arctan(yawrate_ref * L / vx)
        delta       = clip(delta_ff + delta_fb)

    Ports ``heading_ref``, ``vx_ref``, ``y`` → ``F_rear``, ``delta``.
    """

    def __init__(
        self,
        mass: float,
        length_vehicle: float,
        max_steer: float,
        min_steer: float,
        velocity=None,
        heading=None,
        yawrate=None,
        name: str = "Servos",
    ):
        super().__init__(6)
        self.name = name
        self.mass = float(mass)
        self.L = float(length_vehicle)
        self.max_steer = float(max_steer)
        self.min_steer = float(min_steer)

        if velocity is None:
            velocity = _pid_gains(0.8, 0.01, 0.0, -10.0, 10.0, -0.0, 5.0)
        if heading is None:
            heading = _pid_gains(5.0, 0.0, 0.0, -10.0, 10.0, -1.0, 1.0)
        if yawrate is None:
            yawrate = _pid_gains(
                0.3, 0.0, 0.1, -np.pi / 4.0, np.pi / 4.0, -np.pi / 4.0, np.pi / 4.0
            )

        self.params = {
            "velocity": velocity,
            "heading": heading,
            "yawrate": yawrate,
        }
        self.state.labels = [
            "int_v",
            "filt_v",
            "int_h",
            "filt_h",
            "int_r",
            "filt_r",
        ]
        self.x0 = np.zeros(6, dtype=float)

        self.add_input_port("heading_ref", nominal_value=np.array([0.0]))
        self.add_input_port("vx_ref", nominal_value=np.array([0.0]))
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "F_rear",
            dim=1,
            function=self.h_F_rear,
            dependencies=("heading_ref", "vx_ref", "y"),
        )
        self.add_output_port(
            "delta",
            dim=1,
            function=self.h_delta,
            dependencies=("heading_ref", "vx_ref", "y"),
        )

    def _unpack(self, x, u):
        heading_ref = float(u[0])
        vx_ref = float(u[1])
        theta = float(u[4])
        vx = float(u[5])
        yawrate = float(u[7])
        int_v, filt_v = float(x[0]), float(x[1])
        int_h, filt_h = float(x[2]), float(x[3])
        int_r, filt_r = float(x[4]), float(x[5])
        return (
            heading_ref,
            vx_ref,
            theta,
            vx,
            yawrate,
            int_v,
            filt_v,
            int_h,
            filt_h,
            int_r,
            filt_r,
        )

    def _cascade(self, x, u, params=None):
        p = self.params if params is None else params
        (
            heading_ref,
            vx_ref,
            theta,
            vx,
            yawrate,
            int_v,
            filt_v,
            int_h,
            filt_h,
            int_r,
            filt_r,
        ) = self._unpack(x, u)

        e_v = vx_ref - vx
        accel_ref = _pid_output(e_v, int_v, vx, filt_v, p["velocity"])

        e_h = wrap_pi(heading_ref - theta)
        yawrate_ref = _pid_output(e_h, int_h, theta, filt_h, p["heading"])

        e_r = yawrate_ref - yawrate
        delta_fb = _pid_output(e_r, int_r, yawrate, filt_r, p["yawrate"])

        F_rear = self.mass * accel_ref
        vx_num = max(vx, 1e-6)
        delta_ff = float(np.arctan(yawrate_ref * self.L / vx_num))
        delta = float(np.clip(delta_ff + delta_fb, self.min_steer, self.max_steer))
        return F_rear, delta, e_v, e_h, e_r, yawrate_ref

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        (
            heading_ref,
            vx_ref,
            theta,
            vx,
            yawrate,
            int_v,
            filt_v,
            int_h,
            filt_h,
            int_r,
            filt_r,
        ) = self._unpack(x, u)

        e_v = vx_ref - vx
        d_int_v, d_filt_v = _pid_deriv(e_v, int_v, vx, filt_v, p["velocity"])

        e_h = wrap_pi(heading_ref - theta)
        d_int_h, d_filt_h = _pid_deriv(e_h, int_h, theta, filt_h, p["heading"])

        yawrate_ref = _pid_output(e_h, int_h, theta, filt_h, p["heading"])
        e_r = yawrate_ref - yawrate
        d_int_r, d_filt_r = _pid_deriv(e_r, int_r, yawrate, filt_r, p["yawrate"])

        return np.array(
            [d_int_v, d_filt_v, d_int_h, d_filt_h, d_int_r, d_filt_r],
            dtype=float,
        )

    def h_F_rear(self, x, u, t=0.0, params=None):
        F_rear, _, *_ = self._cascade(x, u, params=params)
        return np.array([F_rear], dtype=float)

    def h_delta(self, x, u, t=0.0, params=None):
        _, delta, *_ = self._cascade(x, u, params=params)
        return np.array([delta], dtype=float)
