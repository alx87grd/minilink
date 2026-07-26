"""Aerial vehicle catalog systems."""

from minilink.dynamics.catalog.aerial.drone import (
    ConstantSpeedHelicopterTunnel,
    Drone2D,
    Drone2DWithSideThruster,
    SpeedControlledDrone2D,
)
from minilink.dynamics.catalog.aerial.plane import Plane2D
from minilink.dynamics.catalog.aerial.rocket import Rocket

__all__ = [
    "ConstantSpeedHelicopterTunnel",
    "Drone2D",
    "Drone2DWithSideThruster",
    "Plane2D",
    "Rocket",
    "SpeedControlledDrone2D",
]
