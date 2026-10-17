"""Servos: the velocity, heading and yaw-rate PID cascade, to rear force and steer."""

import numpy as np

from examples.projects.pathtracking.bicycle_los_v2.path_tracking import wrap_pi
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


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

        # Vehicle: the mass the force scales, the wheelbase of the steer
        # feedforward, and the steer limits
        self.mass = float(mass)
        self.L = float(length_vehicle)
        self.max_steer = float(max_steer)
        self.min_steer = float(min_steer)

        # Parameters: each loop's PID gains, the defaults when none are given
        if velocity is None:
            velocity = pid_gains(0.8, 0.01, 0.0, -10.0, 10.0, -0.0, 5.0)
        if heading is None:
            heading = pid_gains(5.0, 0.0, 0.0, -10.0, 10.0, -1.0, 1.0)
        if yawrate is None:
            yawrate = pid_gains(
                0.3, 0.0, 0.1, -np.pi / 4.0, np.pi / 4.0, -np.pi / 4.0, np.pi / 4.0
            )

        self.params = {
            "velocity": velocity,
            "heading": heading,
            "yawrate": yawrate,
        }

        # State: each loop's integral and filtered measurement
        self.state.labels = [
            "int_v",
            "filt_v",
            "int_h",
            "filt_h",
            "int_r",
            "filt_r",
        ]
        self.x0 = np.zeros(6, dtype=float)

        # Ports: the two references and the plant output in, force and steer out
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

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        velocity_gains = params["velocity"]
        heading_gains = params["heading"]
        yawrate_gains = params["yawrate"]

        # u stacks heading_ref, vx_ref, then y = [X, Y, θ, vx, vy, r, ...]
        heading_ref = float(u[0])
        vx_ref = float(u[1])
        theta = float(u[4])
        vx = float(u[5])
        yawrate = float(u[7])

        # Each loop's integral and filtered measurement
        int_v = float(x[0])
        filt_v = float(x[1])
        int_h = float(x[2])
        filt_h = float(x[3])
        int_r = float(x[4])
        filt_r = float(x[5])

        # Velocity loop: e_v = vx_ref − vx
        e_v = vx_ref - vx
        d_int_v, d_filt_v = pid_deriv(e_v, int_v, vx, filt_v, velocity_gains)

        # Heading loop: e_h = wrap(θ_ref − θ)
        e_h = wrap_pi(heading_ref - theta)
        d_int_h, d_filt_h = pid_deriv(e_h, int_h, theta, filt_h, heading_gains)

        # Yaw-rate loop, on the heading loop's output: e_r = r_ref − r
        yawrate_ref = pid_output(e_h, int_h, theta, filt_h, heading_gains)
        e_r = yawrate_ref - yawrate
        d_int_r, d_filt_r = pid_deriv(e_r, int_r, yawrate, filt_r, yawrate_gains)

        # ẋ: each loop's integral rate and filter rate
        dx = np.array(
            [d_int_v, d_filt_v, d_int_h, d_filt_h, d_int_r, d_filt_r],
            dtype=float,
        )

        return dx

    def h_F_rear(self, x, u, t=0.0, params=None):
        # The cascade's rear force, from the velocity loop
        F_rear, _, *_ = self.cascade(x, u, params=params)

        F_rear_out = np.array([F_rear], dtype=float)

        return F_rear_out

    def h_delta(self, x, u, t=0.0, params=None):
        # The cascade's steer, feedforward plus yaw-rate feedback
        _, delta, *_ = self.cascade(x, u, params=params)

        delta_out = np.array([delta], dtype=float)

        return delta_out

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def cascade(self, x, u, params=None):
        """Force and steer, the three loop errors, and the yaw-rate reference."""
        params = self.params if params is None else params

        mass = self.mass
        L = self.L
        max_steer = self.max_steer
        min_steer = self.min_steer
        velocity_gains = params["velocity"]
        heading_gains = params["heading"]
        yawrate_gains = params["yawrate"]

        # u stacks heading_ref, vx_ref, then y = [X, Y, θ, vx, vy, r, ...]
        heading_ref = float(u[0])
        vx_ref = float(u[1])
        theta = float(u[4])
        vx = float(u[5])
        yawrate = float(u[7])

        # Each loop's integral and filtered measurement
        int_v = float(x[0])
        filt_v = float(x[1])
        int_h = float(x[2])
        filt_h = float(x[3])
        int_r = float(x[4])
        filt_r = float(x[5])

        # Velocity loop: e_v = vx_ref − vx, its PID output the acceleration reference
        e_v = vx_ref - vx
        accel_ref = pid_output(e_v, int_v, vx, filt_v, velocity_gains)

        # Heading loop: e_h = wrap(θ_ref − θ), its PID output the yaw-rate reference
        e_h = wrap_pi(heading_ref - theta)
        yawrate_ref = pid_output(e_h, int_h, theta, filt_h, heading_gains)

        # Yaw-rate loop: e_r = r_ref − r, its PID output the steer feedback
        e_r = yawrate_ref - yawrate
        delta_fb = pid_output(e_r, int_r, yawrate, filt_r, yawrate_gains)

        # The rear force that gives the acceleration reference: F = m a_ref
        F_rear = mass * accel_ref

        # Kinematic steer feedforward: δ_ff = arctan(r_ref L / vx), with vx ≥ 10⁻⁶
        vx_num = max(vx, 1e-6)
        delta_ff = float(np.arctan(yawrate_ref * L / vx_num))

        # δ = clip(δ_ff + δ_fb, δ_min, δ_max)
        delta = float(np.clip(delta_ff + delta_fb, min_steer, max_steer))

        return F_rear, delta, e_v, e_h, e_r, yawrate_ref


###############################################################################
# Internal machinery
###############################################################################


# The PID building blocks every loop of the cascade shares


def pid_gains(Kp, Ki, Kd, cmd_min, cmd_max, i_min, i_max, tau=0.1):
    """One loop's gains, command limits, integral limits and filter time constant."""
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


def pid_deriv(e, int_e, meas, meas_filt, p, feedforward=0.0):
    """The loop's state rates: the integral's, held by anti-windup, and the filter's."""
    tau = max(p["tau"], 1e-3)

    # Filtered measurement: ṁ_filt = (m − m_filt) / τ, with τ ≥ 10⁻³
    d_meas_filt = (meas - meas_filt) / tau

    # The unsaturated command the integral feeds
    cmd = pid_cmd(e, int_e, d_meas_filt, p, feedforward=feedforward)

    # Anti-windup: the integral holds while the command saturates the error's way
    stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
    stop_lo = (cmd <= p["cmd_min"]) and (e < 0)
    d_int_e = 0.0 if (stop_hi or stop_lo) else e

    # It also holds at its own limits, i_min ≤ ∫e ≤ i_max, when the error pushes past
    if int_e >= p["i_max"] and e > 0.0:
        d_int_e = 0.0
    elif int_e <= p["i_min"] and e < 0.0:
        d_int_e = 0.0

    return d_int_e, d_meas_filt


def pid_output(e, int_e, meas, meas_filt, p, feedforward=0.0):
    """The loop's command, clipped to its limits."""
    tau = max(p["tau"], 1e-3)

    # Filtered measurement rate: ṁ_filt = (m − m_filt) / τ, with τ ≥ 10⁻³
    d_filt = (meas - meas_filt) / tau

    cmd = pid_cmd(e, int_e, d_filt, p, feedforward=feedforward)

    # cmd_min ≤ u ≤ cmd_max
    cmd_clipped = float(np.clip(cmd, p["cmd_min"], p["cmd_max"]))

    return cmd_clipped


def pid_cmd(e, int_e, d_filt, p, feedforward=0.0):
    """The unsaturated PID command, plus the feedforward."""
    Kp = p["Kp"]
    Ki = p["Ki"]
    Kd = p["Kd"]

    # u = Kp e + Ki ∫e − Kd ṁ_filt + u_ff
    cmd = Kp * e + Ki * int_e - Kd * d_filt
    cmd += feedforward

    return cmd
