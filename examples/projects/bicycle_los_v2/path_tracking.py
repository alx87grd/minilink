"""Path tracking: line-of-sight heading guidance plus speed plan."""

import math

import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import System
from minilink.graphical.animation.primitives import CustomLine, Point

_EPS = 1e-12


def wrap_pi(angle):
    """Wrap an angle to ``[-pi, pi]``."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _iter_segments(path_xy, closed: bool):
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


def _closest_point_on_segment(px, py, x0, y0, x1, y1):
    vx, vy = x1 - x0, y1 - y0
    wx, wy = px - x0, py - y0

    den = vx * vx + vy * vy
    if den <= _EPS:
        return x0, y0, 0.0

    t = (wx * vx + wy * vy) / den
    t = max(0.0, min(1.0, t))

    qx = x0 + t * vx
    qy = y0 + t * vy
    return qx, qy, t


def project_on_path_with_t(path_xy, x, y, closed=True):
    """Project ``(x, y)`` onto a polyline."""
    best = None

    for i, j in _iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        qx, qy, t = _closest_point_on_segment(x, y, x0, y0, x1, y1)

        dx = x1 - x0
        dy = y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            continue

        tx = dx / seg_len
        ty = dy / seg_len

        nx = -ty
        ny = tx

        e_perp = (x - qx) * nx + (y - qy) * ny
        d2 = (x - qx) ** 2 + (y - qy) ** 2

        if best is None or d2 < best[0]:
            path_heading = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e_perp, path_heading)

    if best is None:
        raise ValueError("invalid path for projection")

    _, idx, t, qx, qy, e_perp, path_heading = best
    return idx, t, qx, qy, e_perp, path_heading


def path_total_length(path_xy, closed=True):
    L = 0.0
    for i, j in _iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        L += math.hypot(x1 - x0, y1 - y0)
    return L


def point_ahead_along_path(path_xy, idx, t, ds, closed=True):
    """Advance ``ds`` meters along the polyline from a projection point."""
    n = len(path_xy)
    if n < 2:
        raise ValueError("path is too short")

    if closed:
        Lt = path_total_length(path_xy, closed=True)
        if Lt > _EPS:
            ds = ds % Lt

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

    cx = x0 + t * dx
    cy = y0 + t * dy
    rem = (1.0 - t) * seg_len

    steps = 0
    max_steps = len(path_xy) + 2

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

        if not closed and i == n - 1:
            return x0, y0, i, 0.0, math.atan2(dy, dx)

    lam = 0.0 if seg_len <= _EPS else min(1.0, ds / max(_EPS, seg_len))

    ax = cx + lam * dx
    ay = cy + lam * dy
    heading_a = math.atan2(dy, dx)
    return float(ax), float(ay), int(i), float(lam), float(heading_a)


class LOSController:
    """Plain Python line-of-sight controller."""

    def __init__(
        self,
        path_xy,
        lookahead=4.0,
        control_point_ahead=4.0,
        closed=False,
    ):
        self.path = np.asarray(path_xy, dtype=float)
        self.lookahead = float(lookahead)
        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

    def compute(self, x, y, theta, vx=None, vy=None):
        if self.control_point_ahead != 0.0:
            x_ctrl = x + self.control_point_ahead * math.cos(theta)
            y_ctrl = y + self.control_point_ahead * math.sin(theta)
        else:
            x_ctrl = x
            y_ctrl = y

        idx, t, qx, qy, e_perp, path_heading = project_on_path_with_t(
            self.path,
            x_ctrl,
            y_ctrl,
            closed=self.closed,
        )

        ax, ay, _, _, _ = point_ahead_along_path(
            self.path,
            idx,
            t,
            self.lookahead,
            closed=self.closed,
        )

        # LOS course angle toward the lookahead point.
        heading_ref = math.atan2(ay - y_ctrl, ax - x_ctrl)
        heading_meas = theta

        if vx is not None and vy is not None:
            beta = math.atan2(vy, max(1e-6, vx))
            heading_meas = wrap_pi(theta + beta)

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
        self.vx_ref_value = float(vx_ref)

        self.path_pts = np.asarray(path_pts, dtype=float)
        if self.path_pts.ndim != 2 or self.path_pts.shape[1] < 2:
            raise ValueError("path_pts must be (N, >=2) with x and y columns")
        self.path_xy = self.path_pts[:, :2]

        self.controller = LOSController(
            path_xy=self.path_xy,
            lookahead=lookahead,
            control_point_ahead=control_point_ahead,
            closed=closed,
        )

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
        px = float(u[0])
        py = float(u[1])
        theta = float(u[2])
        heading_ref, _ = self.controller.compute(px, py, theta)
        return np.array([heading_ref], dtype=float)

    def h_vx_ref(self, x, u, t=0.0, params=None):
        return np.array([self.vx_ref_value], dtype=float)

    def get_kinematic_geometry(self):
        pts = np.column_stack(
            [self.path_xy, np.zeros(self.path_xy.shape[0], dtype=float)]
        )
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
        _, info = self.controller.compute(px, py, theta)
        return {
            "world": np.eye(4),
            "control_point": SE2(info["x_ctrl"], info["y_ctrl"], 0.0),
            "lookahead": SE2(info["ax"], info["ay"], 0.0),
        }
