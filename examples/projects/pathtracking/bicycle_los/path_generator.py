"""Path construction helpers for the bicycle LOS demo."""

import math

import numpy as np


def rounded_rectangle_path(
    Lx=40.0, Ly=20.0, R=5.0, nseg=8, narc=12, min_ds=0.10, closed=True
):
    """Rounded rectangle waypoints (half-extents ``Lx``, ``Ly``, corner radius ``R``)."""
    eps = 1e-12
    nseg = max(2, int(nseg))
    narc = max(2, int(narc))
    min_ds = float(min_ds)

    c_TR = (Lx - R, Ly - R)
    c_TL = (-(Lx - R), Ly - R)
    c_BL = (-(Lx - R), -(Ly - R))
    c_BR = (Lx - R, -(Ly - R))

    def add_seq(path, pts):
        for x, y in pts:
            if not path:
                path.append((float(x), float(y)))
            else:
                lx, ly = path[-1]
                if math.hypot(x - lx, y - ly) >= max(min_ds - eps, 0.0):
                    path.append((float(x), float(y)))
        return path

    def arc_pts(cx, cy, a0_deg, a1_deg, n):
        thetas = np.linspace(
            math.radians(a0_deg), math.radians(a1_deg), n, endpoint=True
        )
        return [(cx + R * math.cos(t), cy + R * math.sin(t)) for t in thetas]

    path = []

    xs = np.linspace(Lx - R, -(Lx - R), nseg, endpoint=True)
    add_seq(path, [(x, Ly) for x in xs])
    add_seq(path, arc_pts(*c_TL, 90, 180, narc))

    ys = np.linspace(Ly - R, -(Ly - R), nseg, endpoint=True)
    add_seq(path, [(-Lx, y) for y in ys])
    add_seq(path, arc_pts(*c_BL, 180, 270, narc))

    xs = np.linspace(-(Lx - R), (Lx - R), nseg, endpoint=True)
    add_seq(path, [(x, -Ly) for x in xs])
    add_seq(path, arc_pts(*c_BR, 270, 360, narc))

    ys = np.linspace(-(Ly - R), (Ly - R), nseg, endpoint=True)
    add_seq(path, [(Lx, y) for y in ys])
    add_seq(path, arc_pts(*c_TR, 0, 90, narc))

    if path and not closed:
        x0, y0 = path[0]
        xN, yN = path[-1]
        if math.hypot(xN - x0, yN - y0) < max(min_ds - eps, 0.0):
            path.pop()

    return np.array(path, dtype=float)
