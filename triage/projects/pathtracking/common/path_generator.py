"""Reference paths: a rounded rectangle's waypoints for the path-tracking demos."""

import math

import numpy as np

###############################################################################
# Public API
###############################################################################


def rounded_rectangle_path(
    Lx=40.0, Ly=20.0, R=5.0, nseg=8, narc=12, min_ds=0.10, closed=True
):
    """Rounded rectangle waypoints (half-extents ``Lx``, ``Ly``, corner radius ``R``).

    Counter-clockwise from the right end of the top edge, as an ``(N, 2)`` array:
    ``nseg`` points per edge and ``narc`` per corner arc (at least 2 each), a
    waypoint closer than ``min_ds`` to the previous one dropped. The last arc ends
    where the top edge starts: ``closed=True`` drops that last waypoint when it lies
    within ``min_ds`` of the first, since the closing segment (``iter_segments(...,
    closed=True)``) returns there; ``closed=False`` keeps it, so the open path runs
    the whole loop.
    """
    nseg = max(2, int(nseg))
    narc = max(2, int(narc))
    min_ds = float(min_ds)

    # Corner arc centres, each R inside its corner
    c_TR = (Lx - R, Ly - R)
    c_TL = (-(Lx - R), Ly - R)
    c_BL = (-(Lx - R), -(Ly - R))
    c_BR = (Lx - R, -(Ly - R))

    # The least spacing between waypoints, min_ds less 10⁻¹² of round-off slack
    spacing = max(min_ds - 1e-12, 0.0)

    path = []

    # Top edge, right to left, then the top-left arc
    xs = np.linspace(Lx - R, -(Lx - R), nseg, endpoint=True)
    append_spaced(path, [(x, Ly) for x in xs], spacing)
    append_spaced(path, arc_points(c_TL, R, 90, 180, narc), spacing)

    # Left edge, top to bottom, then the bottom-left arc
    ys = np.linspace(Ly - R, -(Ly - R), nseg, endpoint=True)
    append_spaced(path, [(-Lx, y) for y in ys], spacing)
    append_spaced(path, arc_points(c_BL, R, 180, 270, narc), spacing)

    # Bottom edge, left to right, then the bottom-right arc
    xs = np.linspace(-(Lx - R), (Lx - R), nseg, endpoint=True)
    append_spaced(path, [(x, -Ly) for x in xs], spacing)
    append_spaced(path, arc_points(c_BR, R, 270, 360, narc), spacing)

    # Right edge, bottom to top, then the top-right arc back to the start
    ys = np.linspace(-(Ly - R), (Ly - R), nseg, endpoint=True)
    append_spaced(path, [(Lx, y) for y in ys], spacing)
    append_spaced(path, arc_points(c_TR, R, 0, 90, narc), spacing)

    # A closed path drops its last waypoint when it lands back on the first: the
    # closing segment returns there
    if path and closed:
        x0, y0 = path[0]
        xN, yN = path[-1]
        if math.hypot(xN - x0, yN - y0) < spacing:
            path.pop()

    waypoints = np.array(path, dtype=float)

    return waypoints


###############################################################################
# Internal machinery
###############################################################################


def append_spaced(path, points, spacing):
    """Append ``points`` to ``path`` in place, each one only at ``spacing`` or more from the last."""
    for x, y in points:
        if not path:
            path.append((float(x), float(y)))
        else:
            x_last, y_last = path[-1]
            if math.hypot(x - x_last, y - y_last) >= spacing:
                path.append((float(x), float(y)))


def arc_points(center, R, a0_deg, a1_deg, n):
    """``n`` points of the arc of radius ``R`` about ``center``, from ``a0_deg`` to ``a1_deg`` degrees."""
    cx, cy = center

    thetas = np.linspace(math.radians(a0_deg), math.radians(a1_deg), n, endpoint=True)

    # (x, y) = c + R (cos θ, sin θ)
    points = [(cx + R * math.cos(t), cy + R * math.sin(t)) for t in thetas]

    return points
