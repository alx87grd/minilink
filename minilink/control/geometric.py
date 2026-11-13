"""Geometric path tracking: pure pursuit, Stanley, line of sight, and a heading loop."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.geometry import ReferencePath
from minilink.core.system import System

#: [m] floor on the distance to the pursuit target, so the arc stays finite when the
#: target sits on the rear axle (the end of an open path, reached).
MIN_REACH = 1.0e-6

# The drawing: points along a drawn arc, and the colour of the MPC's receding horizon
ARC_POINTS = 40
GUIDANCE_COLOR = "#ef6c00"


###############################################################################
# Public API
###############################################################################


class PurePursuit(System):
    """Steer the rear axle along the arc that reaches the path one lookahead ahead.

    The rear axle is projected on the path, and the target is the point one lookahead
    further along it. Of the circles tangent to the car at its rear axle, exactly one
    passes through the target, and the kinematic bicycle drives that circle at one
    steer angle: the command. The lookahead grows with speed, so a fast car aims further
    ahead and weaves less. The law needs no path heading and no curvature, and the
    lookahead is its only tuning. The law drives forward.

    Ports: the vehicle state ``y`` = ``[x, y, theta, vx, ...]`` in, the steer command
    ``u`` [rad] out.

    Parameters
    ----------
    path : ReferencePath
        The path to follow, in the plane [m]; its numbers are ``params["path"]``.
    wheelbase : float
        Distance between the axles [m].
    lookahead : float
        Lookahead at standstill [m].
    lookahead_gain : float
        Added lookahead per unit speed [s].
    delta_max : float
        Steer limit [rad].
    rear_offset : float
        Distance from the state's point back to the rear axle [m]: zero for a model
        whose state sits at the rear axle, the centre-of-gravity-to-rear-axle distance
        for one whose state sits at the centre of gravity.
    state_dim : int
        Dimension of the state the block reads.

    References
    ----------
    Coulter, *Implementation of the Pure Pursuit Path Tracking Algorithm*,
    CMU-RI-TR-92-01, 1992.
    """

    def __init__(
        self,
        path: ReferencePath,
        wheelbase: float = 0.34,
        lookahead: float = 0.6,
        lookahead_gain: float = 0.25,
        delta_max: float = 0.52,
        rear_offset: float = 0.0,
        state_dim: int = 9,
    ):
        require_positive("wheelbase", wheelbase)
        require_positive("lookahead", lookahead)

        super().__init__()

        # The path the law reads: its shape is structure, its numbers a part of params
        self.path = path

        # Parameters
        self.params = {
            "path": path.params,
            "wheelbase": float(wheelbase),  # [m]
            "lookahead": float(lookahead),  # [m]
            "lookahead_gain": float(lookahead_gain),  # [s]
            "delta_max": float(delta_max),  # [rad]
            "rear_offset": float(rear_offset),  # [m]
        }

        # Ports: the vehicle state y in, the steer command u out
        self.add_input_port("y", dim=int(state_dim), nominal_value=np.zeros(state_dim))
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("y",),
            labels=["delta_cmd"],
            units=["rad"],
        )

    def ctl(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        path = self.path
        path_params = params["path"]
        L = params["wheelbase"]
        L_0 = params["lookahead"]
        k_v = params["lookahead_gain"]
        delta_max = params["delta_max"]
        b = params["rear_offset"]

        X = u[0]
        Y = u[1]
        theta = u[2]
        vx = u[3]
        c_t = xp.cos(theta)
        s_t = xp.sin(theta)

        # The rear axle, b behind the state's point along the heading
        p = xp.array([X - b * c_t, Y - b * s_t])

        # Its projection on the path at arc length s, and the target one lookahead
        # L_d = L₀ + k_v |vx| further along it
        s, _ = path.project(p, params=path_params)
        L_d = L_0 + k_v * xp.abs(vx)
        p_t = path.at(s + L_d, params=path_params)

        # The target in the car's frame at the rear axle, (x_t, y_t) = Rᵀ(θ) (p_t − p)
        x_t = c_t * (p_t[0] - p[0]) + s_t * (p_t[1] - p[1])
        y_t = -s_t * (p_t[0] - p[0]) + c_t * (p_t[1] - p[1])

        # κ = 2 y_t / d²: the circle tangent to the car through the target, with d²
        # floored at MIN_REACH²
        kappa = 2.0 * y_t / xp.maximum(x_t**2 + y_t**2, MIN_REACH**2)

        # δ = arctan(L κ): the kinematic bicycle on that circle, within the end stops
        delta = xp.clip(xp.arctan(L * kappa), -delta_max, delta_max)
        delta_cmd = xp.array([delta])

        return delta_cmd

    def target(self, x, u, t=0.0, params=None):
        """The point the law aims at, ``[x, y]`` [m]: one lookahead along the path past the rear axle."""
        params = self.params if params is None else params
        xp = array_module(u)

        path = self.path
        path_params = params["path"]
        L_0 = params["lookahead"]
        k_v = params["lookahead_gain"]
        b = params["rear_offset"]

        X = u[0]
        Y = u[1]
        theta = u[2]
        vx = u[3]

        # The rear axle, its projection at arc length s, and the point L_d further along
        p = xp.array([X - b * xp.cos(theta), Y - b * xp.sin(theta)])
        s, _ = path.project(p, params=path_params)
        L_d = L_0 + k_v * xp.abs(vx)
        p_t = path.at(s + L_d, params=path_params)

        return p_t

    # Graphics

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """The target, and the arc the commanded steer drives toward it, in the world.

        The arc is the commanded one: while the steer sits on an end stop it bends less
        than the pursuit arc and falls short of the target.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import CustomLine, Point
        from minilink.graphical.animation.visualization import WORLD

        params = self.params if params is None else params

        L = params["wheelbase"]
        b = params["rear_offset"]

        X = float(u[0])
        Y = float(u[1])
        theta = float(u[2])

        # The rear axle, the target and the commanded steer
        p = np.array([X - b * np.cos(theta), Y - b * np.sin(theta)])
        p_t = np.asarray(self.target(x, u, t, params), dtype=float)
        delta_cmd = float(self.ctl(x, u, t, params)[0])

        # The arc of curvature tan(δ) / L from the rear axle toward the target
        arc = arc_points(p, theta, np.tan(delta_cmd) / L, p_t)

        geometry = {
            WORLD: [
                CustomLine(arc, color=GUIDANCE_COLOR, linewidth=2.0, style="--"),
                Point(pt=(p_t[0], p_t[1], 0.0), color=GUIDANCE_COLOR, size=3),
            ]
        }

        return geometry


class Stanley(System):
    """Steer the front wheels along the path, and back onto it.

    The front axle is projected on the path. The steer is the heading error, the angle
    from the car's heading to the path's there, plus a cross-track correction, the
    angle that points the front axle back at the path: wider the further off it is,
    narrower the faster the car goes. On the kinematic bicycle the cross-track error
    then decays exponentially at a rate set by the gain, at every speed. The softening
    speed keeps the correction finite at rest. The law drives forward.

    Ports: the vehicle state ``y`` = ``[x, y, theta, vx, ...]`` in, the steer command
    ``u`` [rad] out.

    Parameters
    ----------
    path : ReferencePath
        The path to follow, in the plane [m]; its numbers are ``params["path"]``.
    gain : float
        Cross-track gain [1/s].
    softening : float
        Speed added to the car's in the correction [m/s].
    delta_max : float
        Steer limit [rad].
    front_offset : float
        Distance from the state's point forward to the front axle [m]: the
        centre-of-gravity-to-front-axle distance for a model whose state sits at the
        centre of gravity.
    state_dim : int
        Dimension of the state the block reads.

    References
    ----------
    Hoffmann, Tomlin, Montemerlo and Thrun, *Autonomous automobile trajectory tracking
    for off-road driving: controller design, experimental validation and racing*,
    American Control Conference, 2007.
    """

    def __init__(
        self,
        path: ReferencePath,
        gain: float = 2.0,
        softening: float = 0.5,
        delta_max: float = 0.52,
        front_offset: float = 0.0,
        state_dim: int = 9,
    ):
        require_positive("gain", gain)
        require_positive("softening", softening)

        super().__init__()

        # The path the law reads: its shape is structure, its numbers a part of params
        self.path = path

        # Parameters
        self.params = {
            "path": path.params,
            "gain": float(gain),  # [1/s]
            "softening": float(softening),  # [m/s]
            "delta_max": float(delta_max),  # [rad]
            "front_offset": float(front_offset),  # [m]
        }

        # Ports: the vehicle state y in, the steer command u out
        self.add_input_port("y", dim=int(state_dim), nominal_value=np.zeros(state_dim))
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("y",),
            labels=["delta_cmd"],
            units=["rad"],
        )

    def ctl(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        path = self.path
        path_params = params["path"]
        k = params["gain"]
        v_s = params["softening"]
        delta_max = params["delta_max"]
        a = params["front_offset"]

        X = u[0]
        Y = u[1]
        theta = u[2]
        vx = u[3]
        c_t = xp.cos(theta)
        s_t = xp.sin(theta)

        # The front axle, a ahead of the state's point along the heading
        p = xp.array([X + a * c_t, Y + a * s_t])

        # Its projection c on the path at arc length s, and the path's heading ψ_p there
        s, c = path.project(p, params=path_params)
        T = path.tangent(s, params=path_params)
        psi_p = xp.arctan2(T[1], T[0])

        # The signed cross-track error, e = (p − c) · n with n = [−T_y, T_x] to the left
        e = -(p[0] - c[0]) * T[1] + (p[1] - c[1]) * T[0]

        # The heading error ψ_e = ψ_p − θ, wrapped to (−π, π]
        psi_e = xp.arctan2(xp.sin(psi_p - theta), xp.cos(psi_p - theta))

        # δ = ψ_e + arctan(−k e / (v_s + |vx|)), within the end stops
        delta = psi_e + xp.arctan(-k * e / (v_s + xp.abs(vx)))
        delta = xp.clip(delta, -delta_max, delta_max)
        delta_cmd = xp.array([delta])

        return delta_cmd

    # Graphics

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """The front axle, its projection on the path, and the cross-track error between them."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import CustomLine, Point
        from minilink.graphical.animation.visualization import WORLD

        params = self.params if params is None else params

        path = self.path
        path_params = params["path"]
        a = params["front_offset"]

        X = float(u[0])
        Y = float(u[1])
        theta = float(u[2])

        # The front axle and its projection, as the law builds them
        p = np.array([X + a * np.cos(theta), Y + a * np.sin(theta)])
        _, c = path.project(p, params=path_params)
        c = np.asarray(c)

        # The cross-track segment from the front axle to the path
        segment = np.array([[p[0], p[1], 0.0], [c[0], c[1], 0.0]])

        geometry = {
            WORLD: [
                CustomLine(segment, color=GUIDANCE_COLOR, linewidth=2.0, style="--"),
                Point(pt=(c[0], c[1], 0.0), color=GUIDANCE_COLOR, size=3),
            ]
        }

        return geometry


class LineOfSight(System):
    """Head for a point on the path one lookahead ahead of the closest one: the course to steer.

    A control point on the car, on its heading at a fixed distance ahead of the state's
    point, is projected on the path. Standing at that projection and looking a
    lookahead distance down the path's tangent, the line of sight from the control
    point to that spot is the heading to hold: the path's own heading, turned toward
    the path by the angle the cross-track error subtends over the lookahead. A short
    lookahead returns to the path steeply, a long one gently. The output is a heading
    reference; a heading loop such as :class:`HeadingController` turns it into a steer.

    Ports: the vehicle state ``y`` = ``[x, y, theta, vx, ...]`` in, the heading
    reference ``psi_ref`` [rad] out, in (−π, π].

    Parameters
    ----------
    path : ReferencePath
        The path to follow, in the plane [m]; its numbers are ``params["path"]``.
    lookahead : float
        Distance down the path's tangent to the point of sight [m].
    control_point : float
        Distance from the state's point forward to the control point [m].
    state_dim : int
        Dimension of the state the block reads.

    References
    ----------
    Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, Wiley, 2011,
    ch. 10, lookahead-based steering.
    """

    def __init__(
        self,
        path: ReferencePath,
        lookahead: float = 1.0,
        control_point: float = 0.0,
        state_dim: int = 9,
    ):
        require_positive("lookahead", lookahead)

        super().__init__()

        # The path the law reads: its shape is structure, its numbers a part of params
        self.path = path

        # Parameters
        self.params = {
            "path": path.params,
            "lookahead": float(lookahead),  # [m]
            "control_point": float(control_point),  # [m]
        }

        # Ports: the vehicle state y in, the heading reference out
        self.add_input_port("y", dim=int(state_dim), nominal_value=np.zeros(state_dim))
        self.add_output_port(
            "psi_ref",
            dim=1,
            function=self.ctl,
            dependencies=("y",),
            labels=["psi_ref"],
            units=["rad"],
        )

    def ctl(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        path = self.path
        path_params = params["path"]
        Delta = params["lookahead"]
        ell = params["control_point"]

        X = u[0]
        Y = u[1]
        theta = u[2]
        c_t = xp.cos(theta)
        s_t = xp.sin(theta)

        # The control point, ℓ ahead of the state's point along the heading
        p = xp.array([X + ell * c_t, Y + ell * s_t])

        # Its projection c on the path at arc length s, and the path's heading ψ_p there
        s, c = path.project(p, params=path_params)
        T = path.tangent(s, params=path_params)
        psi_p = xp.arctan2(T[1], T[0])

        # The signed cross-track error, e = (p − c) · n with n = [−T_y, T_x] to the left
        e = -(p[0] - c[0]) * T[1] + (p[1] - c[1]) * T[0]

        # ψ_ref = ψ_p − arctan(e / Δ): the line of sight to the point Δ down the path,
        # wrapped to (−π, π]
        psi_ref = psi_p - xp.arctan(e / Delta)
        psi_ref = xp.arctan2(xp.sin(psi_ref), xp.cos(psi_ref))
        reference = xp.array([psi_ref])

        return reference

    # Graphics

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """The control point's projection, the point of sight down the path, and the line of sight to it."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import CustomLine, Point
        from minilink.graphical.animation.visualization import WORLD

        params = self.params if params is None else params

        path = self.path
        path_params = params["path"]
        Delta = params["lookahead"]
        ell = params["control_point"]

        X = float(u[0])
        Y = float(u[1])
        theta = float(u[2])

        # The control point, its projection, and the point Δ down the path's tangent
        p = np.array([X + ell * np.cos(theta), Y + ell * np.sin(theta)])
        s, c = path.project(p, params=path_params)
        T = np.asarray(path.tangent(s, params=path_params))
        q = np.asarray(c) + Delta * T

        # The line of sight from the control point to that point
        sight = np.array([[p[0], p[1], 0.0], [q[0], q[1], 0.0]])

        geometry = {
            WORLD: [
                CustomLine(sight, color=GUIDANCE_COLOR, linewidth=2.0, style="--"),
                Point(pt=(q[0], q[1], 0.0), color=GUIDANCE_COLOR, size=3),
            ]
        }

        return geometry


class HeadingController(System):
    """Steer a car to a heading: a proportional heading loop through the kinematic bicycle.

    The heading error, wrapped to a half turn either way, asks for a yaw rate in
    proportion to it. The kinematic bicycle turns at that rate at one steer angle for
    the current speed, and that angle is the command. While the steer stays off its end
    stops, the heading error then decays exponentially at the gain's rate, at every
    speed. The softening speed keeps the steer finite at rest. The law drives forward.

    Ports: the heading reference ``r`` [rad] and the vehicle state ``y`` =
    ``[x, y, theta, vx, ...]`` in, the steer command ``u`` [rad] out. With
    :class:`LineOfSight` on ``r`` it steers a car along a path.

    Parameters
    ----------
    gain : float
        Heading gain [1/s], the rate at which the heading error decays. A turn at yaw
        rate ``θ̇`` holds a standing heading error ``θ̇ / gain``, so a gentle gain cuts
        every corner.
    wheelbase : float
        Distance between the axles [m].
    softening : float
        Speed added to the car's when the yaw rate is turned into a steer [m/s].
    delta_max : float
        Steer limit [rad].
    state_dim : int
        Dimension of the state the block reads.
    """

    def __init__(
        self,
        gain: float = 10.0,
        wheelbase: float = 0.34,
        softening: float = 0.5,
        delta_max: float = 0.52,
        state_dim: int = 9,
    ):
        require_positive("gain", gain)
        require_positive("wheelbase", wheelbase)
        require_positive("softening", softening)

        super().__init__()

        # Parameters
        self.params = {
            "gain": float(gain),  # [1/s]
            "wheelbase": float(wheelbase),  # [m]
            "softening": float(softening),  # [m/s]
            "delta_max": float(delta_max),  # [rad]
        }

        # Ports: the heading reference r and the vehicle state y in, the steer u out
        self.add_input_port("r", dim=1, nominal_value=np.zeros(1))
        self.add_input_port("y", dim=int(state_dim), nominal_value=np.zeros(state_dim))
        self.add_output_port(
            "u",
            dim=1,
            function=self.ctl,
            dependencies=("r", "y"),
            labels=["delta_cmd"],
            units=["rad"],
        )

    def ctl(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        k_psi = params["gain"]
        L = params["wheelbase"]
        v_s = params["softening"]
        delta_max = params["delta_max"]

        psi_ref = u[0]
        y = u[1:]
        theta = y[2]
        vx = y[3]

        # The heading error e_ψ = ψ_ref − θ, wrapped to (−π, π]
        e_psi = xp.arctan2(xp.sin(psi_ref - theta), xp.cos(psi_ref - theta))

        # The yaw rate it asks for
        r_d = k_psi * e_psi

        # θ̇ = vx tan(δ) / L solved for δ at θ̇ = r_d, with |vx| softened by v_s, within
        # the end stops
        delta = xp.arctan(L * r_d / (xp.abs(vx) + v_s))
        delta = xp.clip(delta, -delta_max, delta_max)
        delta_cmd = xp.array([delta])

        return delta_cmd


###############################################################################
# Internal machinery
###############################################################################


def arc_points(p, theta, kappa, p_t):
    """Points ``(n, 3)`` of the arc of curvature ``kappa`` leaving ``p`` along ``theta``.

    The arc runs for the pursuit arc's length to the target ``p_t``: ``s = d α / sin α``
    for a target at distance ``d`` and bearing ``α``, at most ``π d`` for a target far
    round to the side. The points use sinc, so a straight arc needs no case of its own.
    """
    c_t = np.cos(theta)
    s_t = np.sin(theta)

    # The target's distance and bearing from the start, in the car's frame
    x_t = c_t * (p_t[0] - p[0]) + s_t * (p_t[1] - p[1])
    y_t = -s_t * (p_t[0] - p[0]) + c_t * (p_t[1] - p[1])
    d = np.hypot(x_t, y_t)
    alpha = np.arctan2(y_t, x_t)

    # The arc lengths, and x = sin(κ s) / κ, y = (1 − cos(κ s)) / κ in the car's frame
    s_end = d / max(np.sinc(alpha / np.pi), 1.0 / np.pi)
    s = np.linspace(0.0, s_end, ARC_POINTS)
    x_b = s * np.sinc(kappa * s / np.pi)
    y_b = 0.5 * kappa * s**2 * np.sinc(kappa * s / (2.0 * np.pi)) ** 2

    # The points in the world frame
    X = p[0] + c_t * x_b - s_t * y_b
    Y = p[1] + s_t * x_b + c_t * y_b
    points = np.column_stack([X, Y, np.zeros(ARC_POINTS)])

    return points


def require_positive(name, value):
    """Refuse a zero or negative number where the law divides by it or reads its sign."""
    if not value > 0.0:
        raise ValueError(f"{name} must be positive, got {value}")
