"""Honest arrow primitives (v2): real geometry, no transform-slot hacks.

The legacy :class:`~minilink.graphical.animation.primitives.Arrow` /
:class:`~minilink.graphical.animation.primitives.TorqueArrow` encode their size
through *column-norm scaling* of the placing 4x4 and their torque sweep through
the ``T[3, 3]`` amplitude channel — two conventions the renderers must decode.
The v2 primitives are **honest**: they bake their full point geometry at real
world size in their own local frame (via :func:`arrow_pts` / :func:`torque_arc_pts`),
so the animator just draws them at ``frames[key] @ local_transform`` like any
other polyline — no column-norm scale, no ``T[3, 3]`` side-channel.

These are **separate, internal** classes used only by the v2 pipeline; the legacy
classes stay byte-identical through Phase 4. At the Phase 5 cutover they replace
the legacy classes in ``primitives.py`` under the final names ``Arrow`` /
``TorqueArrow`` (the ``V2`` suffix is dropped). The public catalog already exposes
them under those honest names — see ``graphical/catalog/shapes.py``.
"""

import numpy as np

from minilink.graphical.animation.primitives import GraphicPrimitive

# Render-geometry builders (point geometry, no poses — graphical-band only)


def arrow_pts(base, vector, scale=1.0, head_ratio=0.15):
    """Polyline (Nx3) of a straight arrow from *base* along *vector*, in local XY.

    The arrow is drawn at its **true length** ``scale * |vector|`` — the geometry
    is honest, so no column-norm scaling of the placing transform is needed. A
    near-zero vector collapses to a single point (nothing visible).

    Parameters
    ----------
    base : array-like, length 2
        Tail of the arrow in the local frame.
    vector : array-like, length 2
        Direction (and, with *scale*, length) of the shaft.
    scale : float
        Multiplies *vector* to set the drawn length (e.g. a velocity gain).
    head_ratio : float
        Chevron barb length as a fraction of the shaft length.
    """
    base = np.asarray(base, dtype=float).reshape(2)
    shaft = scale * np.asarray(vector, dtype=float).reshape(2)
    length = float(np.hypot(shaft[0], shaft[1]))
    if length < 1e-12:
        return np.array([[base[0], base[1], 0.0]])

    tip = base + shaft
    direction = shaft / length
    d = head_ratio * length
    back = direction * d
    perp = np.array([-direction[1], direction[0]]) * d

    barb1 = tip - back + perp
    barb2 = tip - back - perp
    return np.array(
        [
            [base[0], base[1], 0.0],
            [tip[0], tip[1], 0.0],
            [barb1[0], barb1[1], 0.0],
            [tip[0], tip[1], 0.0],
            [barb2[0], barb2[1], 0.0],
        ]
    )


def torque_arc_pts(sweep, radius=1.0, head_ratio=0.4, n_arc_pts=40):
    """Polyline (Nx3) of a circular torque arc + chevron, centered at the origin.

    The arc is baked at the given *sweep* (radians, + = CCW) and *radius*, so the
    sweep rides in the geometry rather than the ``T[3, 3]`` slot. Same math as the
    legacy ``TorqueArrow.compute_pts``, lifted to a pure builder.
    """
    sweep = float(sweep)
    r = radius
    d = r * head_ratio

    if abs(sweep) < 1e-6:
        return np.zeros((1, 3))

    n_pts = max(3, int(abs(sweep) / (2 * np.pi) * n_arc_pts))
    angles = np.linspace(0.0, sweep, n_pts)
    arc = np.column_stack([r * np.cos(angles), r * np.sin(angles), np.zeros(n_pts)])

    tip_c, tip_s = np.cos(sweep), np.sin(sweep)
    tip = np.array([r * tip_c, r * tip_s, 0.0])
    if sweep > 0:
        barb1 = tip + np.array(
            [-d / 2 * tip_c + d / 2 * tip_s, -d / 2 * tip_s - d / 2 * tip_c, 0.0]
        )
        barb2 = tip + np.array(
            [d / 2 * tip_c + d / 2 * tip_s, d / 2 * tip_s - d / 2 * tip_c, 0.0]
        )
    else:
        barb1 = tip + np.array(
            [-d / 2 * tip_c - d / 2 * tip_s, -d / 2 * tip_s + d / 2 * tip_c, 0.0]
        )
        barb2 = tip + np.array(
            [d / 2 * tip_c - d / 2 * tip_s, d / 2 * tip_s + d / 2 * tip_c, 0.0]
        )
    return np.vstack([arc, np.array([barb1, tip, barb2])])


# Honest primitives (v2)


class ArrowV2(GraphicPrimitive):
    """A 2-D arrow at honest world size — shaft from *base* along *vector*.

    Unlike the legacy :class:`~minilink.graphical.animation.primitives.Arrow`
    (unit shape + column-norm scaling), the geometry is baked at its true length
    ``scale * |vector|``; the renderer simply draws ``self.pts`` posed by the
    frame. Pass body-frame ``vector`` and key the primitive to that body frame so
    its orientation comes from ``tf`` (no manual ``cos``/``sin`` rotation).
    """

    def __init__(
        self,
        base=(0.0, 0.0),
        vector=(1.0, 0.0),
        scale=1.0,
        head_ratio=0.15,
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.base = np.asarray(base, dtype=float).reshape(2)
        self.vector = np.asarray(vector, dtype=float).reshape(2)
        self.scale = float(scale)
        self.head_ratio = head_ratio
        self.pts = arrow_pts(self.base, self.vector, self.scale, head_ratio)

    def points_at(self, t):
        """Nx3 arrow points at playback time *t* (geometry baked at construction)."""
        return self.pts


class TorqueArrowV2(GraphicPrimitive):
    """A curved torque arc at honest geometry — *sweep* baked, not in ``T[3, 3]``.

    The arc for the given *sweep* and *radius* is built at construction, centered
    at the local origin; the placing frame supplies world position and the start
    orientation. Rebuild it each frame in ``get_dynamic_geometry`` with the
    instantaneous torque.
    """

    def __init__(
        self,
        sweep,
        radius=1.0,
        head_ratio=0.4,
        n_arc_pts=40,
        color="red",
        linewidth=2,
        style="-",
    ):
        super().__init__(color, linewidth, style)
        self.sweep = float(sweep)
        self.radius = radius
        self.head_ratio = head_ratio
        self.n_arc_pts = n_arc_pts
        self.pts = torque_arc_pts(self.sweep, radius, head_ratio, n_arc_pts)

    def points_at(self, t):
        """Nx3 arc points at playback time *t* (geometry baked at construction)."""
        return self.pts
