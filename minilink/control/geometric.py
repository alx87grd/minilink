"""Pure pursuit: the geometric path tracker.

RULES 3.2 keeps ``minilink.control`` from importing ``minilink.planning``, so the tracker
takes the path as a plain ``(N, 2)`` array of waypoints, not a path object. The same array
also builds a ``ReferenceTrack`` on the planning side when a demo wants one.

Reference: R. C. Coulter, "Implementation of the Pure Pursuit Path Tracking Algorithm",
CMU-RI-TR-92-01 (1992).
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.feedback import Controller

#: [m] floor on the distance to the target, so the arc stays finite if the target ever
#: lands on the axle itself (a path shorter than one lookahead, driven to its end).
MIN_REACH = 1.0e-6

# Public API


class PurePursuit(Controller):
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

    feedback_profile = "state"
    measurement_port = "y"
    ref_port = None
    control_port = "u"
    plot_space = "measurement"

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
        self.name = "Pure Pursuit"

        path = np.asarray(waypoints, dtype=float)
        if path.ndim != 2 or path.shape[1] != 2 or path.shape[0] < 2:
            raise ValueError("waypoints must be an (N, 2) array with N >= 2")

        # ``waypoints`` and ``closed`` are not gains: they say what the block tracks.
        # They live in ``params`` all the same, because the search below reads them the
        # way it reads the lookahead, and a swept path is then one params dict away.
        self.params = {
            "waypoints": path,
            "closed": is_closed(path) if closed is None else bool(closed),
            "wheelbase": float(wheelbase),
            "lookahead": float(lookahead),
            "lookahead_gain": float(lookahead_gain),
            "delta_max": float(delta_max),
            "rear_offset": float(rear_offset),
        }

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
        wheelbase, delta_max = params["wheelbase"], params["delta_max"]
        lookahead, lookahead_gain = params["lookahead"], params["lookahead_gain"]
        rear_offset = params["rear_offset"]
        xp = array_module(u)
        path = xp.asarray(params["waypoints"])
        n_points = np.shape(params["waypoints"])[0]

        X, Y, theta, vx = u[0], u[1], u[2], u[3]
        c_t, s_t = xp.cos(theta), xp.sin(theta)
        X_r, Y_r = X - rear_offset * c_t, Y - rear_offset * s_t

        # target: the first waypoint at least one lookahead ahead of the closest one,
        # counted along the path — so the search steps over the seam of a closed loop
        L_d = lookahead + lookahead_gain * xp.abs(vx)
        reach = xp.sqrt((path[:, 0] - X_r) ** 2 + (path[:, 1] - Y_r) ** 2)
        index = xp.arange(n_points)
        closest = xp.argmin(reach)
        if params["closed"]:
            forward = (index - closest) % n_points
        else:  # an open path stops at its end: nothing wraps back round to the start
            forward = xp.where(index >= closest, index - closest, n_points)
        ahead = (reach >= L_d) & (forward < n_points)
        rank = xp.where(ahead, forward, n_points)
        # nothing a lookahead away — a path shorter than L_d — means aim at its far end
        chosen = xp.where(xp.any(ahead), xp.argmin(rank), xp.argmax(reach))
        target = path[chosen]

        # arc through the target: curvature 2 y_t / d^2 in the body frame
        x_t = c_t * (target[0] - X_r) + s_t * (target[1] - Y_r)
        y_t = -s_t * (target[0] - X_r) + c_t * (target[1] - Y_r)
        curvature = 2.0 * y_t / xp.maximum(x_t**2 + y_t**2, MIN_REACH**2)
        delta = xp.arctan(wheelbase * curvature)

        return xp.clip(xp.array([delta]), -delta_max, delta_max)


def is_closed(waypoints):
    """True when the polyline loops back on itself within one waypoint spacing.

    A closed path is written without repeating its first point at the end — the repeat
    would leave a zero-length segment with no tangent — so the loop shows up as a
    last-to-first gap no wider than the other steps.
    """
    path = np.asarray(waypoints, dtype=float)
    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)
    gap = float(np.linalg.norm(path[0] - path[-1]))

    return bool(gap <= 1.5 * float(np.median(steps)))
