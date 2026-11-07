"""Workspace geometry: solids by their signed distance, and the paths a vehicle follows."""

from minilink.core.geometry.paths import (
    PolylinePath,
    ReferencePath,
    circuit_waypoints,
    from_waypoints,
    is_closed,
)
from minilink.core.geometry.shapes import Box, Inflated, Shape, Sphere, Union

__all__ = [
    "Box",
    "Inflated",
    "PolylinePath",
    "ReferencePath",
    "Shape",
    "Sphere",
    "Union",
    "circuit_waypoints",
    "from_waypoints",
    "is_closed",
]
