"""Pure pursuit: steer along the arc through a look-ahead point of a waypoint path."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import System

#: [m] floor on the distance to the target, so the arc stays finite if the target ever
#: lands on the axle itself (a path shorter than one lookahead, driven to its end).
MIN_REACH = 1.0e-6

# The drawing: points along the arc, and the colour of the MPC's receding horizon
ARC_POINTS = 40
PURSUIT_COLOR = "#ef6c00"


###############################################################################
# Public API
###############################################################################


class PurePursuit(System):
    """Steer toward the point of the path one lookahead distance ahead.

    The law is geometry only. Write the target in the body frame at the rear axle,
    ``(x_t, y_t)``, at distance ``d``; the circle through the axle and that point,
    tangent to the car, has curvature ``2 y_t / d^2``, and the kinematic bicycle holds
    that curvature at ``delta = arctan(wheelbase * curvature)``. No path derivative and
    no gain to tune apart from the lookahead — which grows with speed, so a fast car
    aims further ahead and stops weaving.

    Input port ``y``: the vehicle state in the minilink vehicle layout, i.e.
    ``[x, y, theta, vx, ...]``. Output port ``u``: the steer command [rad].

    The target is the first waypoint **at least one lookahead ahead of the closest one,
    counted along the path**. Counting along the path rather than by array index is what
    makes a closed circuit work: at the seam the waypoints ahead of the car are the ones
    at the start of the array, and an index comparison finds none of them and falls back
    to the last waypoint — which sits under the car, where the arc is undefined.

    Parameters
    ----------
    waypoints : array of shape (N, 2)
        Path samples [m], in the order they are driven. A closed loop ends one spacing
        short of its first point (:func:`~minilink.planning.spatial.paths.circuit_waypoints`); it is not closed by
        repeating that point, which would leave a zero-length segment behind.
    wheelbase : float
        Distance between the axles [m].
    lookahead : float
        Lookahead distance at standstill [m].
    lookahead_gain : float
        Added lookahead per unit speed [s], so ``L_d = lookahead + gain * |vx|``.
    delta_max : float
        Steer command limit [rad].
    rear_offset : float
        Distance from the point the state locates back to the rear axle [m], where the
        pursuit geometry is written: zero when the state already sits at the rear axle,
        and the centre-of-gravity-to-rear-axle distance for a model that carries its
        state at the centre of gravity.
    closed : bool or None
        Whether the path loops. ``None`` decides from the waypoints themselves: the path
        is closed when the gap from its last point back to its first is no wider than
        the spacing between neighbours (:func:`is_closed`).
    state_dim : int
        Dimension of the state the block reads (the plant's ``y`` port).
    """

    def __init__(
        self,
        waypoints,
        wheelbase: float = 0.34,
        lookahead: float = 0.6,
        lookahead_gain: float = 0.25,
        delta_max: float = 0.52,
        rear_offset: float = 0.0,
        closed: bool | None = None,
        state_dim: int = 9,
    ):
        super().__init__()

        # Parameters. ``waypoints`` and ``closed`` are not gains: they say what the
        # block tracks. They live in ``params`` all the same, because the search reads
        # them the way it reads the lookahead, and a swept path is then one params dict
        # away.
        path = np.asarray(waypoints, dtype=float)
        require_waypoints(path)
        self.params = {
            "waypoints": path,
            "closed": is_closed(path) if closed is None else bool(closed),
            "wheelbase": float(wheelbase),
            "lookahead": float(lookahead),
            "lookahead_gain": float(lookahead_gain),
            "delta_max": float(delta_max),
            "rear_offset": float(rear_offset),
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

        wheelbase = params["wheelbase"]
        delta_max = params["delta_max"]
        rear_offset = params["rear_offset"]

        # The pose, moved back from the state's point to the rear axle
        X = u[0]
        Y = u[1]
        theta = u[2]
        c_t = xp.cos(theta)
        s_t = xp.sin(theta)
        X_r = X - rear_offset * c_t
        Y_r = Y - rear_offset * s_t
        target = self.target(x, u, t, params)

        # The target in the body frame at the rear axle, (x_t, y_t)
        x_t = c_t * (target[0] - X_r) + s_t * (target[1] - Y_r)
        y_t = -s_t * (target[0] - X_r) + c_t * (target[1] - Y_r)

        # κ = 2 y_t / d², with d² floored at MIN_REACH², and the bicycle's δ = arctan(L κ)
        curvature = 2.0 * y_t / xp.maximum(x_t**2 + y_t**2, MIN_REACH**2)
        delta = xp.arctan(wheelbase * curvature)

        # The steer command, held within the servo end stops: |δ| ≤ δ_max
        delta_cmd = xp.clip(xp.array([delta]), -delta_max, delta_max)

        return delta_cmd

    def target(self, x, u, t=0.0, params=None):
        """The waypoint the law aims at, ``[x, y]`` [m]: the first one a lookahead ahead along the path."""
        params = self.params if params is None else params
        xp = array_module(u)

        lookahead = params["lookahead"]
        lookahead_gain = params["lookahead_gain"]
        rear_offset = params["rear_offset"]
        path = xp.asarray(params["waypoints"])
        # The point count stays a static Python int under JAX tracing
        n_points = xp.shape(params["waypoints"])[0]

        # The pose, moved back from the state's point to the rear axle
        X = u[0]
        Y = u[1]
        theta = u[2]
        vx = u[3]
        X_r = X - rear_offset * xp.cos(theta)
        Y_r = Y - rear_offset * xp.sin(theta)

        # The target: the first waypoint at least one lookahead L_d = L₀ + k_v |vx| ahead
        # of the closest one, counted along the path, so the search steps over the seam
        # of a closed loop
        L_d = lookahead + lookahead_gain * xp.abs(vx)
        reach = xp.sqrt((path[:, 0] - X_r) ** 2 + (path[:, 1] - Y_r) ** 2)
        index = xp.arange(n_points)
        closest = xp.argmin(reach)
        if params["closed"]:
            forward = (index - closest) % n_points
        else:  # An open path stops at its end: nothing wraps back round to the start
            forward = xp.where(index >= closest, index - closest, n_points)
        ahead = (reach >= L_d) & (forward < n_points)
        rank = xp.where(ahead, forward, n_points)
        # Nothing a lookahead away (a path shorter than L_d) means aim at its far end
        chosen = xp.where(xp.any(ahead), xp.argmin(rank), xp.argmax(reach))
        target = path[chosen]

        return target

    # Graphics

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """The target waypoint and the arc the commanded steer drives toward it, in the world.

        The arc is the commanded one, ``κ = tan(δ_cmd) / L``: while the steer sits on
        an end stop it bends less than the pursuit arc and misses the target.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.animation.primitives import CustomLine, Point
        from minilink.graphical.animation.visualization import WORLD

        params = self.params if params is None else params

        wheelbase = params["wheelbase"]
        rear_offset = params["rear_offset"]

        X = float(u[0])
        Y = float(u[1])
        theta = float(u[2])
        c_t = np.cos(theta)
        s_t = np.sin(theta)
        X_r = X - rear_offset * c_t
        Y_r = Y - rear_offset * s_t
        target = np.asarray(self.target(x, u, t, params), dtype=float)
        delta_cmd = float(self.ctl(x, u, t, params)[0])

        # The target in the body frame, its distance d and bearing α from the heading
        x_t = c_t * (target[0] - X_r) + s_t * (target[1] - Y_r)
        y_t = -s_t * (target[0] - X_r) + c_t * (target[1] - Y_r)
        d = np.hypot(x_t, y_t)
        alpha = np.arctan2(y_t, x_t)

        # The commanded arc, κ = tan(δ_cmd) / L, run for the pursuit arc's length to the
        # target, s = d α / sin α, at most π d for a target far round to the side
        kappa = np.tan(delta_cmd) / wheelbase
        s_end = d / max(np.sinc(alpha / np.pi), 1.0 / np.pi)
        s = np.linspace(0.0, s_end, ARC_POINTS)

        # Points on the arc, x = sin(κ s) / κ and y = (1 − cos(κ s)) / κ, written with
        # sinc so the straight line κ = 0 needs no case of its own
        x_b = s * np.sinc(kappa * s / np.pi)
        y_b = 0.5 * kappa * s**2 * np.sinc(kappa * s / (2.0 * np.pi)) ** 2

        # The arc and the target in the world frame
        arc_X = X_r + c_t * x_b - s_t * y_b
        arc_Y = Y_r + s_t * x_b + c_t * y_b
        arc = np.column_stack([arc_X, arc_Y, np.zeros(ARC_POINTS)])
        aim = (target[0], target[1], 0.0)

        geometry = {
            WORLD: [
                CustomLine(arc, color=PURSUIT_COLOR, linewidth=2.0, style="--"),
                Point(pt=aim, color=PURSUIT_COLOR, marker="o", size=3),
            ]
        }

        return geometry


def is_closed(waypoints):
    """True when the polyline loops back on itself within one waypoint spacing.

    A closed path is written without repeating its first point at the end — the repeat
    would leave a zero-length segment with no tangent — so the loop shows up as a
    last-to-first gap no wider than the other steps.
    """
    path = np.asarray(waypoints, dtype=float)

    # The steps between neighbours, and the gap from the last point back to the first
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    gap = float(np.linalg.norm(path[0] - path[-1]))

    # Closed when the gap is no wider than one and a half median steps
    closed = bool(gap <= 1.5 * float(np.median(steps)))

    return closed


###############################################################################
# Internal machinery
###############################################################################


def require_waypoints(path):
    """Refuse a path that is not an ``(N, 2)`` array of at least two points."""
    if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] < 2:
        raise ValueError("waypoints must be an (N, 2) array with N >= 2")
