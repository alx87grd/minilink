"""Rounded-rectangle circuit for MPC lap tuning."""

from __future__ import annotations

import numpy as np

from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.track import ReferenceTrack

CIRCUIT_LX = 30.0
CIRCUIT_LY = 18.0
CIRCUIT_R = 10.0
CORRIDOR_HALF_WIDTH = 3.0


def rounded_rectangle_path(Lx, Ly, R, *, nseg=4, narc=8):
    """CCW rounded-rectangle waypoints centered at the origin."""
    R = min(float(R), float(Lx), float(Ly))
    c_tr = (Lx - R, Ly - R)
    c_tl = (-(Lx - R), Ly - R)
    c_bl = (-(Lx - R), -(Ly - R))
    c_br = (Lx - R, -(Ly - R))

    def arc(cx, cy, a0_deg, a1_deg):
        thetas = np.linspace(np.deg2rad(a0_deg), np.deg2rad(a1_deg), narc)
        return [(cx + R * np.cos(t), cy + R * np.sin(t)) for t in thetas]

    pts = []
    pts.extend([(x, -Ly) for x in np.linspace(0.0, Lx - R, nseg, endpoint=True)])
    pts.extend(arc(*c_br, 270, 360))
    pts.extend([(Lx, y) for y in np.linspace(-(Ly - R), Ly - R, nseg, endpoint=True)])
    pts.extend(arc(*c_tr, 0, 90))
    pts.extend([(x, Ly) for x in np.linspace(Lx - R, -(Lx - R), nseg, endpoint=True)])
    pts.extend(arc(*c_tl, 90, 180))
    pts.extend([(-Lx, y) for y in np.linspace(Ly - R, -(Ly - R), nseg, endpoint=True)])
    pts.extend(arc(*c_bl, 180, 270))
    pts.extend([(x, -Ly) for x in np.linspace(-(Lx - R), 0.0, nseg, endpoint=True)])
    return np.asarray(pts, dtype=float)


LOOP_XY = rounded_rectangle_path(CIRCUIT_LX, CIRCUIT_LY, CIRCUIT_R)


def make_track(*, half_width: float = CORRIDOR_HALF_WIDTH) -> ReferenceTrack:
    return ReferenceTrack(from_waypoints(LOOP_XY), half_width=float(half_width))


def start_pose(track: ReferenceTrack) -> tuple[np.ndarray, float]:
    start_xy = LOOP_XY[0].copy()
    s_start, _ = track.path.project(start_xy)
    tangent = track.path.tangent(s_start)
    theta0 = float(np.arctan2(tangent[1], tangent[0]))
    if abs(np.cos(2.0 * theta0)) > 1.0 - 1e-9:
        theta0 += 1e-4
    return start_xy, theta0
