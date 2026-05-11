"""Cascade path tracking: pure pursuit, heading / yaw-rate loops, and velocity PID.

Tracks a sinusoidal lane ``y(x) = A sin(2 pi x / lambda)`` with
:class:`~minilink.dynamics.catalog.vehicles.dynamic_bicycle.DynamicBicycleCar3D`.
Diagram: cruise task → velocity PID → ``w_rear``; plant ``y`` feeds pure pursuit
→ heading reference → yaw-rate reference → steering ``delta``. Run from repo root::

    conda run -n dev-h26 python \\
        examples/scripts/diagrams/demo_dynamic_bicycle_cascade_path_tracking.py

For meshcat instead of the default renderer, call ``diagram.animate(renderer="meshcat")``.

Animation overlays (world frame): reference sinusoid, pure-pursuit chord, heading-error
arc, steer arc, and paired speed arrows for the velocity loop.
"""

import numpy as np

from minilink.core.blocks.sources import Source
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D
from minilink.graphical.primitives import (
    Arrow,
    CustomLine,
    TorqueArrow,
    scale_pose2d_matrix,
    torque_pose2d_matrix,
)

# Path and motion setpoints (shared by pursuit law and XY plot)
A = 2.0
LAMBDA = 20.0
LD = 7.0
U_REF = 5.0

# Reference path polyline extent for animation (world x, meters)
_PATH_X0 = -8.0
_PATH_X1 = 95.0


class SinusoidalCruiseTask(Source):
    """Constant longitudinal speed reference and animated reference path polyline."""

    def __init__(
        self,
        u_ref: float = U_REF,
        path_a: float = A,
        path_lambda: float = LAMBDA,
    ):
        super().__init__(1)
        self.name = "Sinusoidal cruise task"
        self.params["value"] = np.array([u_ref], dtype=float)
        self.path_a = float(path_a)
        self.path_lambda = float(path_lambda)

    def get_kinematic_geometry(self):
        xs = np.linspace(_PATH_X0, _PATH_X1, 320)
        ys = self.path_a * np.sin(2.0 * np.pi * xs / self.path_lambda)
        pts = np.column_stack([xs, ys, np.zeros_like(xs)])
        return [CustomLine(pts, color="seagreen", linewidth=2.2, style="--")]

    def get_kinematic_transforms(self, x, u, t):
        return [np.eye(4)]


class SinusoidalPursuit(StaticSystem):
    """Kinematic pure pursuit on ``y(x) = A sin(2 pi x / lambda)``.

    Parameters
    ----------
    path_a :
        Sinusoid amplitude [m].
    path_lambda :
        Wavelength [m].
    Ld :
        Look-ahead distance along +x [m].

    Notes
    -----
    Uses a single forward look-ahead point at ``(px + Ld, y(px + Ld))``. The
    output is the heading of the vehicle-to-point ray, ``theta_ref`` [rad].

    Animation: unit :class:`~minilink.graphical.primitives.Arrow` along the
    vehicle–lookahead chord (scaled by chord length).
    """

    def __init__(self, path_a: float, path_lambda: float, ld: float):
        super().__init__(6, 1)
        self.name = "Pure pursuit (sinusoid)"
        self.params = {"A": path_a, "lambda": path_lambda, "Ld": ld}
        self.inputs = {}
        self.add_input_port(6, "y", nominal_value=np.zeros(6))
        self.outputs = {}
        self.add_output_port(
            1,
            "theta_ref",
            function=self.guidance,
            dependencies=["y"],
        )

    def guidance(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        px, py = u[0], u[1]
        x_la = px + p["Ld"]
        y_la = p["A"] * np.sin(2.0 * np.pi * x_la / p["lambda"])
        theta_ref = np.arctan2(y_la - py, x_la - px)
        return np.array([theta_ref], dtype=float)

    def get_kinematic_geometry(self):
        return [Arrow(color="darkorange", linewidth=2.5, origin="base")]

    def get_kinematic_transforms(self, x, u, t):
        px, py = float(u[0]), float(u[1])
        p = self.params
        x_la = px + p["Ld"]
        y_la = p["A"] * np.sin(2.0 * np.pi * x_la / p["lambda"])
        dx = x_la - px
        dy = y_la - py
        L = float(np.hypot(dx, dy))
        L = max(L, 1e-3)
        th = float(np.arctan2(dy, dx))
        return [scale_pose2d_matrix(px, py, th, L)]


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

    Animation: :class:`~minilink.graphical.primitives.TorqueArrow` at the
    vehicle CG with sweep ``theta_ref - theta`` (wrapped).
    """

    def __init__(self, K_psi: float = 1.85, r_max: float = 0.68):
        super().__init__(7, 1)
        self.name = "Heading loop"
        self.params = {"K_psi": K_psi, "r_max": r_max}
        self.inputs = {}
        self.add_input_port(1, "theta_ref", nominal_value=np.array([0.0]))
        self.add_input_port(6, "y", nominal_value=np.zeros(6))
        self.outputs = {}
        self.add_output_port(
            1,
            "r_ref",
            function=self.heading_to_r,
            dependencies=["theta_ref", "y"],
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
        return [TorqueArrow(radius=1.15, color="mediumpurple", linewidth=2.0)]

    def get_kinematic_transforms(self, x, u, t):
        theta_ref, theta = float(u[0]), float(u[3])
        px, py = float(u[1]), float(u[2])
        e_psi = theta_ref - theta
        e_psi = (e_psi + np.pi) % (2.0 * np.pi) - np.pi
        return [torque_pose2d_matrix(px, py, theta, e_psi)]


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
        super().__init__(7, 1)
        self.name = "Yaw-rate loop"
        self.params = {"K_r": K_r, "delta_max": delta_max}
        self.inputs = {}
        self.add_input_port(1, "r_ref", nominal_value=np.array([0.0]))
        self.add_input_port(6, "y", nominal_value=np.zeros(6))
        self.outputs = {}
        self.add_output_port(
            1,
            "delta",
            function=self.r_to_delta,
            dependencies=["r_ref", "y"],
        )

    def r_to_delta(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        r_ref, r = u[0], u[6]
        delta = p["K_r"] * (r_ref - r)
        delta = np.clip(delta, -p["delta_max"], p["delta_max"])
        return np.array([delta], dtype=float)

    def get_kinematic_geometry(self):
        return [TorqueArrow(radius=0.85, color="coral", linewidth=2.0)]

    def get_kinematic_transforms(self, x, u, t):
        px, py, theta = float(u[1]), float(u[2]), float(u[3])
        delta = float(self.r_to_delta(x, u, t)[0])
        return [torque_pose2d_matrix(px, py, theta, delta)]


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

    Animation: two body-aligned arrows at the CG, lengths tied to ``u_ref`` and
    measured ``u`` (offset laterally to separate them).
    """

    def __init__(self, r_rear: float):
        super().__init__(2, 7, 1)
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
        self.inputs = {}
        self.add_input_port(1, "u_ref", nominal_value=np.array([U_REF]))
        self.add_input_port(6, "y", nominal_value=np.zeros(6))
        self.outputs = {}
        self.add_output_port(
            1,
            "w_rear",
            function=self.h_w,
            dependencies=["u_ref", "y"],
        )
        self.add_output_port(2, "x", function=self.compute_state, dependencies=[])

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

    def get_kinematic_geometry(self):
        return [
            Arrow(color="steelblue", linewidth=2.2, origin="base"),
            Arrow(color="dimgray", linewidth=2.0, origin="base"),
        ]

    def get_kinematic_transforms(self, x, u, t):
        px, py, theta = float(u[1]), float(u[2]), float(u[3])
        u_ref, u_b = float(u[0]), float(u[4])
        s_ref = float(np.clip(u_ref / 4.0, 0.35, 5.0))
        s_b = float(np.clip(abs(u_b) / 4.0, 0.35, 5.0))
        ox = 0.45 * float(-np.sin(theta))
        oy = 0.45 * float(np.cos(theta))
        return [
            scale_pose2d_matrix(px, py, theta, s_ref),
            scale_pose2d_matrix(px + ox, py + oy, theta, s_b),
        ]


def plot_xy_vs_path(px, py, t, a_amp: float, wavelength: float):
    """Overlay the simulated track (vehicle ``x,y``) and the analytic path."""
    import matplotlib.pyplot as plt

    t_final = float(t[-1])
    x0, x1 = float(px[0]), float(px[-1])
    pad = 5.0
    xs = np.linspace(min(x0, x1) - pad, max(x0, x1) + pad, 600)
    ys = a_amp * np.sin(2.0 * np.pi * xs / wavelength)

    fig, ax = plt.subplots(1, 1, figsize=(9, 4))
    ax.plot(xs, ys, "k--", linewidth=1.2, alpha=0.85, label="reference path")
    ax.plot(px, py, "b-", linewidth=1.5, label="vehicle (x, y)")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.35)
    ax.legend(loc="upper right")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(f"Cascade path tracking (t_final = {t_final:.2f} s)")
    fig.tight_layout()
    plt.show()


vehicle = DynamicBicycleCar3D()
vehicle.x0 = np.array([-10.0, 0.4, 0.05, 0.0, 0.0, 0.0], dtype=float)

task = SinusoidalCruiseTask(U_REF, A, LAMBDA)
pure_pursuit = SinusoidalPursuit(A, LAMBDA, LD)
heading_loop = HeadingLoop()
yaw_rate_loop = YawRateLoop()
vel_pid = VelocityPID(vehicle.r_r)

diagram = DiagramSystem()
diagram.name = "Cascade sinusoid tracking"
diagram.add_subsystem(task, "task")
diagram.add_subsystem(pure_pursuit, "pure_pursuit")
diagram.add_subsystem(heading_loop, "heading_loop")
diagram.add_subsystem(yaw_rate_loop, "yaw_rate_loop")
diagram.add_subsystem(vel_pid, "vel_pid")
diagram.add_subsystem(vehicle, "vehicle")

diagram.connect("task", "y", "vel_pid", "u_ref")
diagram.connect("vehicle", "y", "vel_pid", "y")
diagram.connect("vehicle", "y", "pure_pursuit", "y")
diagram.connect("pure_pursuit", "theta_ref", "heading_loop", "theta_ref")
diagram.connect("vehicle", "y", "heading_loop", "y")
diagram.connect("heading_loop", "r_ref", "yaw_rate_loop", "r_ref")
diagram.connect("vehicle", "y", "yaw_rate_loop", "y")
diagram.connect("vel_pid", "w_rear", "vehicle", "w_rear")
diagram.connect("yaw_rate_loop", "delta", "vehicle", "delta")

diagram.plot_graphe()
diagram.compute_trajectory(tf=20.0, dt=0.02, show=False, verbose=False)
diagram.plot_trajectory()

i0, i1 = diagram.state_index["vehicle"]
xv = diagram.traj.x[i0:i1, :]
px, py = xv[0, :], xv[1, :]
y_des = A * np.sin(2.0 * np.pi * px / LAMBDA)
print(
    f"Max |lateral - sinusoid(path x)| along track: {np.max(np.abs(py - y_des)):.4f} m"
)

plot_xy_vs_path(px, py, diagram.traj.t, A, LAMBDA)

# diagram.animate()
# diagram.animate(renderer="meshcat")
