"""Public shape primitives — the curated surface students and demos import.

Mirrors how ``dynamics.catalog`` re-exports curated plants: the classes live in
the internal ``graphical/animation/`` band; this module is the friendly, stable
re-export. Two things to note:

- ``Line`` is the public name for the internal ``CustomLine``.
- ``Arrow`` / ``TorqueArrow`` are the **honest** v2 primitives
  (``shapes_v2.ArrowV2`` / ``TorqueArrowV2``) — the public arrows are never the
  legacy column-norm-scaling ones in ``animation/primitives.py``.

Demo import: ``from minilink.graphical.catalog import Box, Circle, Arrow``.
"""

from minilink.graphical.animation.primitives import (
    Box,
    Circle,
    ExtrudedPolygon,
    HorizonPolyline,
    Plane,
    Point,
    Rod,
    Sphere,
    TrajectoryPolyline,
    ground_line,
    spring_line,
    vehicle_body,
    wheel_box,
)
from minilink.graphical.animation.primitives import (
    CustomLine as Line,
)
from minilink.graphical.animation.shapes_v2 import (
    ArrowV2 as Arrow,
)
from minilink.graphical.animation.shapes_v2 import (
    TorqueArrowV2 as TorqueArrow,
)

__all__ = [
    "Arrow",
    "Box",
    "Circle",
    "ExtrudedPolygon",
    "HorizonPolyline",
    "Line",
    "Plane",
    "Point",
    "Rod",
    "Sphere",
    "TorqueArrow",
    "TrajectoryPolyline",
    "ground_line",
    "spring_line",
    "vehicle_body",
    "wheel_box",
]
