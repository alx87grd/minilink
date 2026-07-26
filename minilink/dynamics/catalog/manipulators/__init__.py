"""Manipulator catalog systems."""

from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    ThreeLinkManipulator3D,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

__all__ = [
    "FiveLinkPlanarManipulator",
    "OneLinkManipulator",
    "SpeedControlledManipulator",
    "ThreeLinkManipulator3D",
    "TwoLinkManipulator",
    "UR5Manipulator",
]
