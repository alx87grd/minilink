"""Path tracking: line-of-sight heading guidance and a speed plan along a polyline."""

import math

import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import System
from minilink.graphical.animation.primitives import CustomLine, Point

# Below this, a length or a squared length counts as zero (a degenerate segment)
_EPS = 1e-12


###############################################################################
# Public API
###############################################################################


class PathTracking(System):
    """Path tracking: LOS heading reference plus speed plan.

    Input ``y`` is the plant boundary output
    ``[X, Y, theta, vx, vy, yawrate, ...]``. Outputs ``heading_ref`` [rad]
    and constant ``vx_ref`` [m/s]. Draws the reference path in the world frame.
    """

    def __init__(
        self,
        path_pts,
        lookahead=1.0,
        control_point_ahead=0.5,
        closed=False,
        vx_ref=10.0,
    ):
        super().__init__(0)
        self.name = "Path tracking"

        # Parameters: the constant speed plan
        self.vx_ref_value = float(vx_ref)

        # The reference path: its waypoints, and their x and y columns
        self.path_pts = np.asarray(path_pts, dtype=float)
        require_path_points(self.path_pts)
        self.path_xy = self.path_pts[:, :2]

        # The line-of-sight law along the path
        self.controller = LOSController(
            path_xy=self.path_xy,
            lookahead=lookahead,
            control_point_ahead=control_point_ahead,
            closed=closed,
        )

        # Ports: the plant output in, the heading and speed references out
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port(
            "heading_ref",
            dim=1,
            function=self.h_heading_ref,
            dependencies=("y",),
        )
        self.add_output_port(
            "vx_ref",
            dim=1,
            function=self.h_vx_ref,
            dependencies=(),
        )

    def h_heading_ref(self, x, u, t=0.0, params=None):
        controller = self.controller

        px = float(u[0])
        py = float(u[1])
        theta = float(u[2])

        # The LOS heading toward the lookahead point
        heading_ref, _ = controller.compute(px, py, theta)

        heading_ref_out = np.array([heading_ref], dtype=float)

        return heading_ref_out

    def h_vx_ref(self, x, u, t=0.0, params=None):
        vx_ref = self.vx_ref_value

        # The speed plan is constant: vx_ref
        vx_ref_out = np.array([vx_ref], dtype=float)

        return vx_ref_out

    # Graphics

    def get_kinematic_geometry(self):
        path_xy = self.path_xy

        # The path at z = 0 in the world frame, and the two guidance points
        pts = np.column_stack([path_xy, np.zeros(path_xy.shape[0], dtype=float)])

        return {
            "world": [CustomLine(pts, color="salmon", linewidth=2, style="-")],
            "control_point": [
                Point(pt=[0.0, 0.0, 0.0], color="orange", marker="o", size=4)
            ],
            "lookahead": [Point(pt=[0.0, 0.0, 0.0], color="red", marker="o", size=4)],
        }

    def tf(self, x, u, t=0.0, params=None):
        controller = self.controller

        px = float(u[0])
        py = float(u[1])
        theta = float(u[2])

        # The control point and the lookahead point the law steers by
        _, info = controller.compute(px, py, theta)

        return {
            "world": np.eye(4),
            "control_point": SE2(info["x_ctrl"], info["y_ctrl"], 0.0),
            "lookahead": SE2(info["ax"], info["ay"], 0.0),
        }


class LOSController:
    """Plain Python line-of-sight controller.

    Projects a control point, ``control_point_ahead`` metres ahead of the vehicle,
    on the path, and steers toward the point ``lookahead`` metres further along it.
    """

    def __init__(
        self,
        path_xy,
        lookahead=4.0,
        control_point_ahead=4.0,
        closed=False,
    ):
        # Parameters: the path, the lookahead distance along it, the control point's
        # offset ahead of the vehicle, and whether the path closes on itself
        self.path = np.asarray(path_xy, dtype=float)
        self.lookahead = float(lookahead)
        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

    def compute(self, x, y, theta, vx=None, vy=None):
        """The LOS heading reference at ``(x, y, theta)``, and the guidance ``info``.

        With ``vx`` and ``vy`` given, the heading error is measured from the course
        (the heading plus the sideslip) rather than from the heading.
        """
        path = self.path
        lookahead = self.lookahead
        control_point_ahead = self.control_point_ahead
        closed = self.closed

        # The control point, control_point_ahead metres ahead along the heading θ
        if control_point_ahead != 0.0:
            x_ctrl = x + control_point_ahead * math.cos(theta)
            y_ctrl = y + control_point_ahead * math.sin(theta)
        else:
            x_ctrl = x
            y_ctrl = y

        # Its projection on the path, then the point lookahead metres further along
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

        # LOS course toward the lookahead point: ψ_ref = atan2(a_y − y_c, a_x − x_c)
        heading_ref = math.atan2(ay - y_ctrl, ax - x_ctrl)

        # Measured course: θ, or θ + β with the sideslip β = atan2(vy, max(vx, 10⁻⁶))
        heading_meas = theta
        if vx is not None and vy is not None:
            beta = math.atan2(vy, max(1e-6, vx))
            heading_meas = wrap_pi(theta + beta)

        # Heading error, wrapped to [−π, π)
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


# Path geometry on a polyline


def wrap_pi(angle):
    """``angle`` wrapped to ``[-π, π)``."""
    wrapped = (angle + math.pi) % (2.0 * math.pi) - math.pi

    return wrapped


def project_on_path_with_t(path_xy, x, y, closed=True):
    """Project ``(x, y)`` onto a polyline.

    Returns ``(idx, t, qx, qy, e_perp, path_heading)`` on the nearest segment of
    positive length: its index, the fraction ``t`` along it, the projection, the
    signed cross-track error (positive to the left), and the segment's heading.
    """
    best = None

    # The nearest point over every segment
    for i, j in iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        # The closest point q of the segment, at the fraction t along it
        qx, qy, t = closest_point_on_segment(x, y, x0, y0, x1, y1)

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            continue

        # The segment's unit tangent, and its left normal
        tx = dx / seg_len
        ty = dy / seg_len

        nx = -ty
        ny = tx

        # Signed cross-track error e⊥ = (p − q)·n, and the squared distance |p − q|²
        e_perp = (x - qx) * nx + (y - qy) * ny
        d2 = (x - qx) ** 2 + (y - qy) ** 2

        if best is None or d2 < best[0]:
            path_heading = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e_perp, path_heading)

    require_projection(best)

    _, idx, t, qx, qy, e_perp, path_heading = best
    return idx, t, qx, qy, e_perp, path_heading


def path_total_length(path_xy, closed=True):
    """The polyline's length, the closing segment included when ``closed``."""
    L = 0.0

    # L = Σ |p_j − p_i| over the segments
    for i, j in iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        L += math.hypot(x1 - x0, y1 - y0)

    return L


def point_ahead_along_path(path_xy, idx, t, ds, closed=True):
    """Advance ``ds`` meters along the polyline from a projection point.

    Returns ``(ax, ay, i, lam, heading)``: the point reached, its segment index, its
    fraction along that segment, and the segment's heading. An open path stops at
    its last waypoint.
    """
    n = len(path_xy)
    require_two_waypoints(n)

    # On a closed path, the distance wraps around the loop
    if closed:
        Lt = path_total_length(path_xy, closed=True)
        if Lt > _EPS:
            ds = ds % Lt

    # The projection's segment, or the next one when it is degenerate
    i = idx % n
    j = (i + 1) % n

    x0, y0 = path_xy[i]
    x1, y1 = path_xy[j]

    dx = x1 - x0
    dy = y1 - y0
    seg_len = math.hypot(dx, dy)

    if seg_len <= _EPS:
        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)

    # The start point c = p_i + t (p_j − p_i), and the length left on its segment
    cx = x0 + t * dx
    cy = y0 + t * dy
    rem = (1.0 - t) * seg_len

    steps = 0
    max_steps = len(path_xy) + 2

    # Walk whole segments while ds runs past the end of the current one
    while ds > rem and steps < max_steps:
        ds -= rem

        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)

        if seg_len <= _EPS:
            rem = 0.0
            steps += 1
            continue

        cx = x0
        cy = y0
        rem = seg_len
        steps += 1

        # An open path stops at its last waypoint
        if not closed and i == n - 1:
            heading_end = math.atan2(dy, dx)
            return x0, y0, i, 0.0, heading_end

    # The fraction λ of the segment the remaining ds covers, at most 1
    lam = 0.0 if seg_len <= _EPS else min(1.0, ds / max(_EPS, seg_len))

    # a = c + λ (p_j − p_i), with the segment's heading
    ax = cx + lam * dx
    ay = cy + lam * dy
    heading_a = math.atan2(dy, dx)

    return float(ax), float(ay), int(i), float(lam), float(heading_a)


###############################################################################
# Internal machinery
###############################################################################


def require_path_points(path_pts):
    """Refuse path points that are not an ``(N, ≥2)`` array with x and y columns."""
    if path_pts.ndim != 2 or path_pts.shape[1] < 2:
        raise ValueError("path_pts must be (N, >=2) with x and y columns")


def iter_segments(path_xy, closed: bool):
    """Each segment's ``(i, j)`` waypoint indices; the closing one when ``closed``."""
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
    """The point of segment ``p₀p₁`` nearest ``(px, py)``, and its fraction ``t``."""
    # v = p₁ − p₀ along the segment, w = p − p₀ toward the point
    vx = x1 - x0
    vy = y1 - y0
    wx = px - x0
    wy = py - y0

    den = vx * vx + vy * vy
    if den <= _EPS:
        return x0, y0, 0.0

    # t = clamp(w·v / |v|², 0, 1)
    t = (wx * vx + wy * vy) / den
    t = max(0.0, min(1.0, t))

    # q = p₀ + t v
    qx = x0 + t * vx
    qy = y0 + t * vy

    return qx, qy, t


def require_projection(best):
    """Refuse a projection that found no segment of positive length."""
    if best is None:
        raise ValueError("invalid path for projection")


def require_two_waypoints(n):
    """Refuse a path of fewer than two waypoints."""
    if n < 2:
        raise ValueError("path is too short")
