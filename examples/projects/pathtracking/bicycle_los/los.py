"""Line-of-sight guidance: the heading toward a lookahead point on a polyline path."""

import math

import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import System
from minilink.graphical.animation.primitives import CustomLine, Point

# Below it, a length (or a squared length) counts as zero: a degenerate segment
_EPS = 1e-12


###############################################################################
# Public API
###############################################################################


class LOS(System):
    """Line-of-sight heading reference from full plant output ``y``.

    Input ``y`` is the plant boundary output
    ``[X, Y, theta, vx, vy, yawrate, ...]``. Output ``heading_ref`` [rad].
    Draws the reference path in the world frame for animation.
    """

    def __init__(
        self,
        path_pts,
        lookahead=1.0,
        control_point_ahead=0.5,
        closed=False,
    ):
        super().__init__(0)
        self.name = "LOS"

        # Parameters: the path's points, x and y in the first two columns
        self.path_pts = np.asarray(path_pts, dtype=float)
        require_xy_columns(self.path_pts)
        self.path_xy = self.path_pts[:, :2]

        # The line-of-sight law along the path
        self.controller = LOSController(
            path_xy=self.path_xy,
            lookahead=lookahead,
            control_point_ahead=control_point_ahead,
            closed=closed,
        )

        # Ports: the plant output in, the heading reference out
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "heading_ref",
            dim=1,
            function=self.h_heading_ref,
            dependencies=("y",),
        )

    def h_heading_ref(self, x, u, t=0.0, params=None):
        px = float(u[0])
        py = float(u[1])
        theta = float(u[2])

        # ψ_ref at the plant's pose (X, Y, θ)
        heading_ref, _ = self.controller.compute(px, py, theta)

        heading_ref_out = np.array([heading_ref], dtype=float)

        return heading_ref_out

    # Graphics

    def get_kinematic_geometry(self):
        path_xy = self.path_xy

        # The path on the ground plane z = 0
        pts = np.column_stack([path_xy, np.zeros(path_xy.shape[0], dtype=float)])

        return {
            "world": [CustomLine(pts, color="salmon", linewidth=2, style="-")],
            "control_point": [
                Point(pt=[0.0, 0.0, 0.0], color="orange", marker="o", size=4)
            ],
            "lookahead": [Point(pt=[0.0, 0.0, 0.0], color="red", marker="o", size=4)],
        }

    def tf(self, x, u, t=0.0, params=None):
        px = float(u[0])
        py = float(u[1])
        theta = float(u[2])

        # The control point the law steers from, and the lookahead point it aims at
        _, info = self.controller.compute(px, py, theta)

        return {
            "world": np.eye(4),
            "control_point": SE2(info["x_ctrl"], info["y_ctrl"], 0.0),
            "lookahead": SE2(info["ax"], info["ay"], 0.0),
        }


class LOSController:
    """Plain Python line-of-sight controller.

    The control point sits ``control_point_ahead`` ahead of the body along its
    heading; the reference is the course from it toward the point
    ``lookahead`` further along the path than its projection.
    """

    def __init__(
        self,
        path_xy,
        lookahead=4.0,
        control_point_ahead=4.0,
        closed=False,
    ):
        # Parameters: the path, the lookahead distance along it [m], the control
        # point's distance ahead of the body [m], and whether the path loops
        self.path = np.asarray(path_xy, dtype=float)
        self.lookahead = float(lookahead)
        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

    def compute(self, x, y, theta, vx=None, vy=None):
        """Return ``(heading_ref, info)``: the LOS course at the pose ``(x, y, theta)``, and the guidance's working values."""
        path = self.path
        lookahead = self.lookahead
        control_point_ahead = self.control_point_ahead
        closed = self.closed

        # The control point c = p + d (cos θ, sin θ), ahead of the body
        if control_point_ahead != 0.0:
            x_ctrl = x + control_point_ahead * math.cos(theta)
            y_ctrl = y + control_point_ahead * math.sin(theta)
        else:
            x_ctrl = x
            y_ctrl = y

        # Its projection on the path, then the lookahead point a further along it
        idx, t, qx, qy, e_perp, path_heading = project_on_path_with_t(
            path,
            x_ctrl,
            y_ctrl,
            closed=closed,
        )
        ax, ay, _, _, _ = point_ahead_along_path(
            path,
            idx,
            t,
            lookahead,
            closed=closed,
        )

        # LOS course angle toward the lookahead point: ψ_ref = atan2(a_y − c_y, a_x − c_x)
        heading_ref = math.atan2(ay - y_ctrl, ax - x_ctrl)

        # The measured course: θ, plus the sideslip β = atan2(v_y, max(10⁻⁶, v_x))
        # when the body velocities are given
        heading_meas = theta
        if vx is not None and vy is not None:
            beta = math.atan2(vy, max(1e-6, vx))
            heading_meas = wrap_pi(theta + beta)

        # e_ψ = wrap(ψ_ref − ψ)
        err_heading = wrap_pi(heading_ref - heading_meas)

        info = {
            "e_perp": e_perp,
            "heading_ref": heading_ref,
            "idx": idx,
            "ax": ax,
            "ay": ay,
            "x_ctrl": x_ctrl,
            "y_ctrl": y_ctrl,
            "err_heading": err_heading,
            "path_heading": path_heading,
        }

        return heading_ref, info


# Path geometry


def project_on_path_with_t(path_xy, x, y, closed=True):
    """Project ``(x, y)`` onto a polyline.

    Returns ``(idx, t, qx, qy, e_perp, path_heading)`` on the nearest segment:
    its index, the fraction ``t`` along it, the closest point ``q``, the signed
    cross-track error (positive on the left) and the segment's heading.
    """
    best = None

    # For all segments
    for i, j in iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        # The closest point q, at the fraction t of the segment
        qx, qy, t = closest_point_on_segment(x, y, x0, y0, x1, y1)

        # A degenerate segment has no direction
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            continue

        # Unit tangent τ, and the left normal n = (−τ_y, τ_x)
        tx = dx / seg_len
        ty = dy / seg_len
        nx = -ty
        ny = tx

        # e⊥ = (p − q)·n, and the squared distance |p − q|²
        e_perp = (x - qx) * nx + (y - qy) * ny
        d2 = (x - qx) ** 2 + (y - qy) ** 2

        # Keep the nearest segment
        if best is None or d2 < best[0]:
            path_heading = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e_perp, path_heading)

    require_projection(best)

    _, idx, t, qx, qy, e_perp, path_heading = best

    return idx, t, qx, qy, e_perp, path_heading


def point_ahead_along_path(path_xy, idx, t, ds, closed=True):
    """Advance ``ds`` meters along the polyline from a projection point.

    Starts at the fraction ``t`` of segment ``idx``, or at ``t = 0`` of the next
    non-degenerate segment when that one is degenerate. Returns
    ``(ax, ay, i, lam, heading)``, floats but the int segment ``i``: the point
    reached, its segment, its fraction along it and the segment's heading. An open
    path ends with its last segment: it stops at that segment's end point with its
    heading, and refuses a negative ``ds``.
    """
    n = len(path_xy)
    require_two_points(n)

    # The segments: the closing one only on a closed path, where the distance wraps
    # around the loop; an open path only goes forward
    if closed:
        n_seg = n
        Lt = path_total_length(path_xy, closed=True)
        if Lt > _EPS:
            ds = ds % Lt
    else:
        n_seg = n - 1
        require_forward_distance(ds)

    # The projection's segment, at most the last one of an open path
    i = idx % n if closed else min(idx, n_seg - 1)
    j = (i + 1) % n

    x0, y0 = path_xy[i]
    x1, y1 = path_xy[j]

    dx = x1 - x0
    dy = y1 - y0
    seg_len = math.hypot(dx, dy)

    # A degenerate segment hands over to t = 0 of the next non-degenerate one
    handovers = 0
    while seg_len <= _EPS and handovers < n_seg - 1 and (closed or i < n_seg - 1):
        t = 0.0
        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        handovers += 1

    # The start c = p_i + t (p_j − p_i), and the length left to the segment's end
    cx = x0 + t * dx
    cy = y0 + t * dy
    rem = (1.0 - t) * seg_len

    # Walk whole segments while the distance left exceeds the current remainder,
    # at most n + 2 of them
    steps = 0
    max_steps = len(path_xy) + 2

    while ds > rem and steps < max_steps:
        # An open path stops at the end point of its last segment, with its heading
        if not closed and i == n_seg - 1:
            heading_end = math.atan2(dy, dx)
            return float(x1), float(y1), int(i), 1.0, float(heading_end)

        ds -= rem

        # The next segment
        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)

        # A degenerate segment adds no length
        if seg_len <= _EPS:
            rem = 0.0
            steps += 1
            continue

        cx = x0
        cy = y0
        rem = seg_len
        steps += 1

    # a = c + λ (p_j − p_i), λ = min(1, ds / ℓ) the fraction of the segment still to cover
    lam = 0.0 if seg_len <= _EPS else min(1.0, ds / max(_EPS, seg_len))
    ax = cx + lam * dx
    ay = cy + lam * dy
    heading_a = math.atan2(dy, dx)

    return float(ax), float(ay), int(i), float(lam), float(heading_a)


def path_total_length(path_xy, closed=True):
    """The polyline's length, the closing segment included when ``closed``."""
    L = 0.0

    # L = Σ |p_j − p_i| over the segments
    for i, j in iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        L += math.hypot(x1 - x0, y1 - y0)

    return L


def wrap_pi(angle):
    """Wrap an angle to ``[-pi, pi]``."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi

    return wrapped


###############################################################################
# Internal machinery
###############################################################################


def require_xy_columns(path_pts):
    """Refuse path points that are not an ``(N, >=2)`` array with x and y columns."""
    if path_pts.ndim != 2 or path_pts.shape[1] < 2:
        raise ValueError("path_pts must be (N, >=2) with x and y columns")


def iter_segments(path_xy, closed: bool):
    """Yield the ``(i, j)`` point indices of each segment, the closing one when ``closed``."""
    n = len(path_xy)
    if n < 2:
        return

    if closed:
        for i in range(n):
            j = (i + 1) % n
            yield i, j
    else:
        for i in range(n - 1):
            yield i, i + 1


def closest_point_on_segment(px, py, x0, y0, x1, y1):
    """Return ``(qx, qy, t)``: the point of the segment nearest ``p``, at the fraction ``t`` along it."""
    vx = x1 - x0
    vy = y1 - y0
    wx = px - x0
    wy = py - y0

    # A degenerate segment is its first point
    den = vx * vx + vy * vy
    if den <= _EPS:
        return x0, y0, 0.0

    # t = clip(w·v / |v|², 0, 1), then q = p₀ + t v
    t = (wx * vx + wy * vy) / den
    t = max(0.0, min(1.0, t))

    qx = x0 + t * vx
    qy = y0 + t * vy

    return qx, qy, t


def require_projection(best):
    """Refuse a path with no segment to project on."""
    if best is None:
        raise ValueError("invalid path for projection")


def require_two_points(n):
    """Refuse a path of fewer than two points."""
    if n < 2:
        raise ValueError("path is too short")


def require_forward_distance(ds):
    """Refuse a negative distance along an open path, which has no loop to wrap it."""
    if ds < 0.0:
        raise ValueError(f"ds must be non-negative on an open path, got {ds}")
