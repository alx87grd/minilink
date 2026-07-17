"""Line-of-sight heading guidance for Simon's bicycle path follower."""

import math

import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import System
from minilink.graphical.animation.primitives import Point

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
            psi_path = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e_perp, psi_path)

    if best is None:
        raise ValueError("invalid path for projection")

    _, idx, t, qx, qy, e_perp, psi_path = best
    return idx, t, qx, qy, e_perp, psi_path


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
    psi_a = math.atan2(dy, dx)
    return float(ax), float(ay), int(i), float(lam), float(psi_a)


class LOSController:
    """Plain Python line-of-sight controller."""

    def __init__(
        self,
        path_xy,
        Delta=4.0,
        i_lim=10.0,
        control_point_ahead=4.0,
        closed=False,
    ):
        self.path = np.asarray(path_xy, dtype=float)
        self.Delta = float(Delta)
        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

    def compute(self, x, y, psi, u_body=None, v_body=None):
        if self.control_point_ahead != 0.0:
            x_ctrl = x + self.control_point_ahead * math.cos(psi)
            y_ctrl = y + self.control_point_ahead * math.sin(psi)
        else:
            x_ctrl = x
            y_ctrl = y

        idx, t, qx, qy, e_perp, psi_path = project_on_path_with_t(
            self.path,
            x_ctrl,
            y_ctrl,
            closed=self.closed,
        )

        ax, ay, _, _, _ = point_ahead_along_path(
            self.path,
            idx,
            t,
            self.Delta,
            closed=self.closed,
        )

        chi_d = math.atan2(ay - y_ctrl, ax - x_ctrl)
        chi_meas = psi

        if u_body is not None and v_body is not None:
            beta = math.atan2(v_body, max(1e-6, u_body))
            chi_meas = wrap_pi(psi + beta)

        err_chi = wrap_pi(chi_d - chi_meas)

        info = {
            "e_perp": e_perp,
            "chi_d": chi_d,
            "idx": idx,
            "ax": ax,
            "ay": ay,
            "x_ctrl": x_ctrl,
            "y_ctrl": y_ctrl,
            "err_chi": err_chi,
            "psi_path": psi_path,
        }

        return chi_d, info


class Los(System):
    """Minilink wrapper around Simon's LOS controller."""

    def __init__(
        self,
        path_pts,
        Delta=1.0,
        zeta=0.7,
        omega_n=1.2,
        control_point_ahead=0.5,
        closed=False,
    ):
        super().__init__(0)
        self.name = "LOS"

        self.path_pts = np.asarray(path_pts, dtype=float)
        if self.path_pts.shape[1] < 2:
            raise ValueError("path_pts must contain at least x and y columns")
        self.path_xy = self.path_pts[:, :2]

        self.controller = LOSController(
            path_xy=self.path_xy,
            Delta=Delta,
            control_point_ahead=control_point_ahead,
            closed=closed,
        )

        self.add_input_port("x", nominal_value=np.array([0.0]))
        self.add_input_port("y", nominal_value=np.array([0.0]))
        self.add_input_port("psi", nominal_value=np.array([0.0]))
        self.add_output_port(
            "theta",
            dim=1,
            function=self.los,
            dependencies=("x", "y", "psi"),
        )
        self.add_output_port(
            "logs",
            dim=1,
            function=self.logs,
            dependencies=("x", "y", "psi"),
        )

    def los(self, x, u, t=0.0, params=None):
        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        chi_ref, _ = self.controller.compute(px, py, psi)
        return np.array([chi_ref], dtype=float)

    def logs(self, x, u, t=0.0, params=None):
        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        _, info = self.controller.compute(px, py, psi)
        idx = info["idx"]
        return np.array([idx], dtype=float)

    def get_kinematic_geometry(self):
        return {
            "control_point": [
                Point(pt=[0.0, 0.0, 0.0], color="orange", marker="o", size=4)
            ],
            "lookahead": [Point(pt=[0.0, 0.0, 0.0], color="red", marker="o", size=4)],
        }

    def tf(self, x, u, t=0.0, params=None):
        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        _, info = self.controller.compute(px, py, psi)
        return {
            "control_point": SE2(info["x_ctrl"], info["y_ctrl"], 0.0),
            "lookahead": SE2(info["ax"], info["ay"], 0.0),
        }
