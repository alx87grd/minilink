"""Public shape primitives — the curated surface students and demos import.

Mirrors how ``dynamics.catalog`` re-exports curated plants: the classes live in
the internal ``graphical/animation/`` band; this module is the friendly, stable
re-export. Two things to note:

- ``Line`` is the public name for the internal ``CustomLine``.
- ``Arrow`` / ``TorqueArrow`` are the honest, frame-keyed primitives whose
  geometry is baked at construction (no column-norm side-channel scaling).

Demo import: ``from minilink.graphical.catalog import Box, Circle, Arrow``.
"""

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    ExtrudedPolygon,
    HorizonPolyline,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    TrajectoryPolyline,
    ground_line,
    spring_line,
    vehicle_body,
    wheel_box,
)
from minilink.graphical.animation.primitives import (
    CustomLine as Line,
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
