"""Cascade path tracking: pure pursuit, heading / yaw-rate loops, and velocity PID.

Tracks a sinusoidal lane ``y(x) = A sin(2 pi x / lambda)`` with
:class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.DynamicBicycleCar3D`.
Diagram: PathPlanner → ``path`` → Tracking + ``y`` ``→ theta_ref``; planner ``u_ref``
→ velocity PID ``→ w_rear``; heading / yaw-rate loops ``→ delta``. Run from repo root::

    conda run -n dev-h26 python \\
        examples/scripts/diagrams/demo_dynamic_bicycle_cascade_path_tracking.py

For meshcat instead of the default renderer, call ``diagram.animate(renderer="meshcat")``.

Animation overlays (world frame): reference sinusoid, pure-pursuit chord, heading-error
arc, and steer arc (velocity loop has no drawing primitives).
"""

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.kinematics import SE2, identity
from minilink.core.system import DynamicSystem, StaticSystem, System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D
from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    TorqueArrow,
)

# Path and motion setpoints (shared by pursuit law and XY plot)
A = 2.0
LAMBDA = 20.0
# Pure-pursuit lookahead along +x [m]; Tracking block parameter (not on path message).
LD = 7.0
U_REF = 5.0

# Reference path polyline extent for animation (world x, meters)
_PATH_X0 = -8.0
_PATH_X1 = 95.0


class PathPlanner(System):
    """Hardcoded sinusoid lane definition and cruise speed broadcast.

    Output ``path`` is ``[A, lambda]'`` for ``y(x) = A sin(2 pi x / lambda)``.
    Pure-pursuit lookahead is a separate parameter on :class:`Tracking`.

    Outputs
    -------
    u_ref
        Reference body surge speed [m/s] for the velocity loop (single scalar).
    path
        ``[amplitude, wavelength]'`` [m].
    """

    def __init__(
        self,
        u_ref: float = U_REF,
        *,
        path_a: float = A,
        path_lambda: float = LAMBDA,
    ):
        super().__init__(0)
        self.name = "Path planner"
        self._u_ref = float(u_ref)
        self.path_a = float(path_a)
        self.path_lambda = float(path_lambda)
        self.add_output_port("u_ref", dim=1, function=self.h_u_ref, dependencies=())
        self.add_output_port("path", dim=2, function=self.h_path, dependencies=())

    def h_u_ref(self, x, u, t=0.0, params=None):
        return np.array([self._u_ref], dtype=float)

    def h_path(self, x, u, t=0.0, params=None):
        return np.array([self.path_a, self.path_lambda], dtype=float)

    def get_kinematic_geometry(self):
        xs = np.linspace(_PATH_X0, _PATH_X1, 320)
        ys = self.path_a * np.sin(2.0 * np.pi * xs / self.path_lambda)
        pts = np.column_stack([xs, ys, np.zeros_like(xs)])
        return {"world": [CustomLine(pts, color="seagreen", linewidth=2.2, style="--")]}

    def tf(self, x, u, t=0, params=None):
        return {"world": identity()}


class Tracking(StaticSystem):
    """Pure pursuit targeting ``y(x) = A sin(2 pi x / lambda)`` from planner ``path``.

    Parameters
    ----------
    ld :
        Look-ahead distance along world +x [m].

    Notes
    -----
    Port order in ``u``: ``path`` (2 entries : ``[A, lambda]``), then plant ``y``
    (6).

    Uses a forward look-ahead ``(px + Ld, y(px + Ld))``. The output is ``theta_ref``
    [rad] for the heading loop.

    Animation: chord :class:`~minilink.graphical.animation.primitives.Arrow` from pose to lookahead.
    """

    def __init__(self, ld: float = LD):
        super().__init__()
        self.name = "Tracking"
        self.params = {"Ld": float(ld)}
        self.add_input_port(
            "path",
            dim=2,
            nominal_value=np.array([A, LAMBDA], dtype=float),
        )
        self.add_input_port("y", dim=6, nominal_value=np.zeros(6))
        self.add_output_port(
            "theta_ref",
            dim=1,
            function=self.guidance,
            dependencies=("path", "y"),
        )

    def guidance(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        ld = float(p["Ld"])
        amp, wavelength = float(u[0]), float(u[1])
        px, py = float(u[2]), float(u[3])
        x_la = px + ld
        y_la = amp * np.sin(2.0 * np.pi * x_la / wavelength)
        theta_ref = np.arctan2(y_la - py, x_la - px)
        return np.array([theta_ref], dtype=float)

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        return {"world": identity()}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        p = self.params
        ld = float(p["Ld"])
        amp, wavelength = float(u[0]), float(u[1])
        px, py = float(u[2]), float(u[3])
        x_la = px + ld
        y_la = amp * np.sin(2.0 * np.pi * x_la / wavelength)
        dx = x_la - px
        dy = y_la - py
        return {
            "world": [
                Arrow(
                    base=(px, py),
                    vector=(dx, dy),
                    scale=1.0,
                    color="darkorange",
                    linewidth=2.5,
                    head_ratio=0.15,
                )
            ]
        }


class HeadingLoop(StaticSystem):
    """P loop: heading error → yaw-rate reference ``r_ref`` [rad/s].

    Parameters
    ----------
    K_psi :
        Heading error gain [1/s].
    r_max :
        Saturation on ``r_ref`` [rad/s].

    Notes
    -----
    Port order in ``u``: ``theta_ref`` (1), then plant ``y`` (6).

    Animation: :class:`~minilink.graphical.animation.primitives.TorqueArrow` at the
    vehicle CG with sweep ``theta_ref - theta`` (wrapped).
    """

    def __init__(self, K_psi: float = 1.85, r_max: float = 0.68):
        super().__init__()
        self.name = "Heading loop"
        self.params = {"K_psi": K_psi, "r_max": r_max}
        self.add_input_port("theta_ref", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("y", dim=6, nominal_value=np.zeros(6))
        self.add_output_port(
            "r_ref",
            dim=1,
            function=self.heading_to_r,
            dependencies=("theta_ref", "y"),
        )

    def heading_to_r(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        theta_ref, theta = u[0], u[3]
        e_psi = theta_ref - theta
        e_psi = (e_psi + np.pi) % (2.0 * np.pi) - np.pi
        r_ref = p["K_psi"] * e_psi
        r_ref = np.clip(r_ref, -p["r_max"], p["r_max"])
        return np.array([r_ref], dtype=float)

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        px, py, theta = float(u[1]), float(u[2]), float(u[3])
        return {"heading": SE2(px, py, theta)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        theta_ref, theta = float(u[0]), float(u[3])
        e_psi = theta_ref - theta
        e_psi = (e_psi + np.pi) % (2.0 * np.pi) - np.pi
        return {
            "heading": [
                TorqueArrow(
                    sweep=e_psi,
                    radius=1.15,
                    head_ratio=0.4,
                    color="mediumpurple",
                    linewidth=2.0,
                )
            ]
        }


class YawRateLoop(StaticSystem):
    """P loop: yaw-rate error → steer angle ``delta`` [rad].

    Parameters
    ----------
    K_r :
        Yaw-rate error gain [s].
    delta_max :
        Steering magnitude limit [rad].

    Notes
    -----
    Port order in ``u``: ``r_ref`` (1), then plant ``y`` (6).

    Uses ``delta = K_r (r_ref - r)`` so that when ``r_ref < 0`` (need slower /
    negative yaw rate) the command moves opposite to the case ``r_ref > 0``, matching
    the bicycle convention in this catalog (same structural sign as the yaw term in
    :class:`SimpleLineTracker` in ``demo_dynamic_bicycle_line_tracking.py``).
    """

    def __init__(self, K_r: float = 0.52, delta_max: float = 0.52):
        super().__init__()
        self.name = "Yaw-rate loop"
        self.params = {"K_r": K_r, "delta_max": delta_max}
        self.add_input_port("r_ref", dim=1, nominal_value=np.array([0.0]))
        self.add_input_port("y", dim=6, nominal_value=np.zeros(6))
        self.add_output_port(
            "delta",
            dim=1,
            function=self.r_to_delta,
            dependencies=("r_ref", "y"),
        )

    def r_to_delta(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        r_ref, r = u[0], u[6]
        delta = p["K_r"] * (r_ref - r)
        delta = np.clip(delta, -p["delta_max"], p["delta_max"])
        return np.array([delta], dtype=float)

    def get_kinematic_geometry(self):
        return {}

    def tf(self, x, u, t=0, params=None):
        px, py, theta = float(u[1]), float(u[2]), float(u[3])
        return {"steer": SE2(px, py, theta)}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        delta = float(self.r_to_delta(x, u, t)[0])
        return {
            "steer": [
                TorqueArrow(
                    sweep=delta,
                    radius=0.85,
                    head_ratio=0.4,
                    color="coral",
                    linewidth=2.0,
                )
            ]
        }


class VelocityPID(DynamicSystem):
    """PI + derivative-on-measurement for ``w_rear`` (rear wheel rate).

    State ``x = [int_e, u_filt]'``: integral of speed error and filtered speed.

    Parameters
    ----------
    r_rear :
        Rear wheel radius [m].
    Kp, Ki, Kd :
        PID gains on speed error ``e = u_ref - u`` (body longitudinal).
    tau :
        First-order filter time constant for ``u`` [s].
    w_max :
        Upper clip on ``w_rear`` [rad/s].
    i_max :
        Absolute clamp on ``int_e`` for anti-windup bookkeeping [m].

    Notes
    -----
    Port order in ``u``: ``u_ref`` (1), plant ``y`` (6).

    Feed-forward ``u_ref / r_rear`` is added to the PID correction. Integration
    stops increasing when the unconstrained command is saturated and ``e`` would
    deepen saturation.
    """

    def __init__(self, r_rear: float):
        super().__init__(n=2)
        self.r_rear = float(r_rear)
        self.name = "Velocity PID"
        self.params = {
            "Kp": 2.0,
            "Ki": 1.0,
            "Kd": 0.05,
            "tau": 0.1,
            "w_max": 120.0,
            "i_max": 25.0,
        }
        self.state.labels = ["int_e", "u_filt"]
        self.x0 = np.array([0.0, U_REF], dtype=float)
        self.add_input_port("u_ref", dim=1, nominal_value=np.array([U_REF]))
        self.add_input_port("y", dim=6, nominal_value=np.zeros(6))
        self.add_output_port(
            "w_rear",
            dim=1,
            function=self.h_w,
            dependencies=("u_ref", "y"),
        )
        self.add_output_port("x", dim=2, function=self.compute_state, dependencies=())

    def _speed_error(self, u):
        u_ref, u_b = u[0], u[4]
        return u_ref - u_b

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, u_filt = x[0], x[1]
        e = self._speed_error(u)
        u_b = u[4]
        tau = max(p["tau"], 1e-6)

        w_ff = u[0] / self.r_rear
        d_filt = (u_b - u_filt) / tau
        w_pred = w_ff + p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
        stop_hi = (w_pred >= p["w_max"]) and (e > 0)
        stop_lo = (w_pred <= 0.0) and (e < 0)
        d_int = 0.0 if (stop_hi or stop_lo) else e
        if int_e >= p["i_max"] and e > 0:
            d_int = 0.0
        elif int_e <= -p["i_max"] and e < 0:
            d_int = 0.0

        d_u_filt = (u_b - u_filt) / tau
        return np.array([d_int, d_u_filt], dtype=float)

    def h_w(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, u_filt = x[0], x[1]
        e = self._speed_error(u)
        u_b = u[4]
        tau = max(p["tau"], 1e-6)
        d_filt = (u_b - u_filt) / tau
        w_cmd = u[0] / self.r_rear + p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
        w_cmd = np.clip(w_cmd, 0.0, p["w_max"])
        return np.array([w_cmd], dtype=float)

