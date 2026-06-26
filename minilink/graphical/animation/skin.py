"""
Attachable visual appearance for a system.

A :class:`Skin` is the visual counterpart of the kinematic skeleton: it binds
graphic primitives to the named frames returned by
:meth:`minilink.core.system.System.frames`. Each part is either

- **rigid** — a primitive placed by ``frames[frame]`` (chassis, wheel, disc), or
- **data-driven** — a primitive whose pose is rebuilt every frame from
  ``(x, u, t, params)`` (velocity / force arrows).

Two workflows stay one line each:

- *debug, from scratch* — ``sys.skin = Skin().add_arrow(source="u", index=(0, 1))``
  draws an arrow from ``u[0:2]`` with no subclassing;
- *swap an advanced look* — ``sys.skin = bicycle_skin(sys)`` (rich factories live
  next to their plant, which already owns the geometry math).

When ``sys.skin`` is ``None`` the system falls back to
:meth:`~minilink.core.system.System.default_skin` (catalog plants return their
own factory; everything else gets :func:`generic_skin`, one marker per state and
input — the historical default look).
"""

import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    CustomLine,
    Point,
    arrow_transform,
    translation_matrix,
)

# Public API


class Skin:
    """Ordered list of visual parts attached to a system's kinematic frames."""

    def __init__(self):
        self.parts = []

    # Attachment (chainable)

    def add(self, primitive, frame="base"):
        """Attach a rigid *primitive*, placed by ``frames[frame]``."""
        self.parts.append(_FramePart(primitive, frame))
        return self

    def add_line(self, pts, frame="base", color="blue", linewidth=1, style="-"):
        """Attach a :class:`CustomLine` of body-frame *pts* on *frame*."""
        return self.add(
            CustomLine(pts, color=color, linewidth=linewidth, style=style), frame
        )

    def add_arrow(
        self,
        frame="base",
        source="u",
        index=(0, 1),
        scale=0.4,
        color="red",
        linewidth=2,
        vector=None,
    ):
        """Attach a data-driven arrow originating at *frame*.

        The arrow vector is ``(sig[i], sig[j])`` where *sig* is the input ``u``
        (``source="u"``) or state ``x`` (``source="x"``); pass *vector* as a
        callable ``(x, u, t, params) -> (vx, vy)`` for anything richer.
        """
        if vector is None:
            i, j = index

            def vector(x, u, t, params, _i=i, _j=j, _src=source):
                sig = u if _src == "u" else x
                return sig[_i], sig[_j]

        arrow = Arrow(color=color, linewidth=linewidth, origin="base")
        self.parts.append(_ArrowPart(arrow, frame, vector, scale))
        return self

    def add_dynamic(self, primitive, pose):
        """Attach *primitive* with an explicit ``pose(frames, x, u, t, params)``."""
        self.parts.append(_DynamicPart(primitive, pose))
        return self

    # Render assembly

    def primitives(self):
        """Static primitive list, one per part (ordered)."""
        return [part.primitive for part in self.parts]

    def transforms(self, frames, x, u, t, params):
        """4x4 transform per part, aligned with :meth:`primitives`."""
        return [part.transform(frames, x, u, t, params) for part in self.parts]


def generic_skin(sys):
    """Default look for any system: one marker per state and per input.

    Reproduces the historical base-``System`` visualization (a blue ``o`` per
    state at ``(x_i, i)``, a red ``x`` per input at ``(u_j, -j-1)``) so a
    from-scratch plant animates with zero wiring.
    """
    skin = Skin()
    for i in range(sys.n):
        skin.add_dynamic(
            Point(color="blue", marker="o"),
            lambda frames, x, u, t, params, _i=i: translation_matrix(
                dx=x[_i], dy=float(_i)
            ),
        )
    for j in range(sys.m):
        skin.add_dynamic(
            Point(color="red", marker="x"),
            lambda frames, x, u, t, params, _j=j: translation_matrix(
                dx=u[_j], dy=float(-_j - 1)
            ),
        )
    return skin


# Internal parts


class _FramePart:
    """Rigid primitive placed by a named frame pose."""

    def __init__(self, primitive, frame="base"):
        self.primitive = primitive
        self.frame = frame

    def transform(self, frames, x, u, t, params):
        return frames[self.frame]


class _ArrowPart:
    """Arrow originating at a frame, stretched to a data-driven vector."""

    def __init__(self, primitive, frame, vector, scale):
        self.primitive = primitive
        self.frame = frame
        self.vector = vector
        self.scale = scale

    def transform(self, frames, x, u, t, params):
        T = frames[self.frame]
        px, py = float(T[0, 3]), float(T[1, 3])
        vx, vy = self.vector(x, u, t, params)
        return arrow_transform(px, py, vx, vy, self.scale)


class _DynamicPart:
    """Primitive with a fully custom per-frame pose closure."""

    def __init__(self, primitive, pose):
        self.primitive = primitive
        self.pose = pose

    def transform(self, frames, x, u, t, params):
        return self.pose(frames, x, u, t, params)


if __name__ == "__main__":
    skin = Skin().add_arrow(source="u", index=(0, 1))
    print(skin.primitives())
    print(
        np.round(
            skin.transforms(
                {"base": np.eye(4)}, np.zeros(2), np.array([1.0, 0.0]), 0.0, {}
            )[0],
            2,
        )
    )
