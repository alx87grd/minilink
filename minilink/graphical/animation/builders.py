"""
Render-geometry builders (local-frame polylines for dynamic skin).

These are rendering-only helpers; equation-path kinematics live in
:mod:`minilink.core.kinematics`.
"""

import numpy as np

from minilink.graphical.animation.primitives import CustomLine


def arrow_pts(base=(0.0, 0.0), vector=(1.0, 0.0), scale=1.0, head_ratio=0.15):
    """Return Nx3 polyline points for a 2-D arrow in the local frame."""
    base = np.asarray(base, dtype=float).reshape(2)
    vector = np.asarray(vector, dtype=float).reshape(2)
    vx, vy = vector * scale
    length = np.hypot(vx, vy)
    if length < 1e-12:
        return np.zeros((1, 3))
    d = head_ratio
    tip = base + np.array([vx, vy])
    barb = d * length
    direction = np.array([vx, vy]) / length
    normal = np.array([-direction[1], direction[0]])
    return np.array(
        [
            [*base, 0.0],
            [*tip, 0.0],
            [*(tip - direction * barb + normal * barb), 0.0],
            [*tip, 0.0],
            [*(tip - direction * barb - normal * barb), 0.0],
        ]
    )


def torque_arc_pts(
    radius=1.0,
    sweep=0.0,
    head_ratio=0.4,
    n_arc_pts=40,
):
    """Return Nx3 arc + chevron polyline in the local frame (centered at origin)."""
    r = float(radius)
    d = r * head_ratio
    sweep = float(sweep)
    if abs(sweep) < 1e-6:
        return np.zeros((1, 3))
    n_pts = max(3, int(abs(sweep) / (2 * np.pi) * n_arc_pts))
    angles = np.linspace(0, sweep, n_pts)
    arc = np.column_stack([r * np.cos(angles), r * np.sin(angles), np.zeros(n_pts)])
    tip_c = np.cos(sweep)
    tip_s = np.sin(sweep)
    tip = np.array([r * tip_c, r * tip_s, 0.0])
    if sweep > 0:
        barb1 = tip + np.array([-d / 2 * tip_c + d / 2 * tip_s, -d / 2 * tip_s - d / 2 * tip_c, 0.0])
        barb2 = tip + np.array([d / 2 * tip_c + d / 2 * tip_s, d / 2 * tip_s - d / 2 * tip_c, 0.0])
    else:
        barb1 = tip + np.array([-d / 2 * tip_c - d / 2 * tip_s, -d / 2 * tip_s + d / 2 * tip_c, 0.0])
        barb2 = tip + np.array([d / 2 * tip_c - d / 2 * tip_s, d / 2 * tip_s + d / 2 * tip_c, 0.0])
    return np.vstack([arc, np.array([barb1, tip, barb2])])


def arrow_line(vector, scale=1.0, color="red", linewidth=2, head_ratio=0.15):
    """Build a :class:`~minilink.graphical.animation.primitives.CustomLine` arrow."""
    return CustomLine(
        arrow_pts(vector=vector, scale=scale, head_ratio=head_ratio),
        color=color,
        linewidth=linewidth,
    )


def torque_arc_line(
    radius,
    sweep,
    *,
    head_ratio=0.4,
    n_arc_pts=40,
    color="red",
    linewidth=2,
):
    """Build a :class:`~minilink.graphical.animation.primitives.CustomLine` torque arc."""
    return CustomLine(
        torque_arc_pts(radius, sweep, head_ratio=head_ratio, n_arc_pts=n_arc_pts),
        color=color,
        linewidth=linewidth,
    )
