"""Manipulator catalog systems."""

from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkManipulator,
    KinematicManipulator,
    OneLinkManipulator,
    ThreeLinkManipulator,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

__all__ = [
    "FiveLinkManipulator",
    "OneLinkManipulator",
    "KinematicManipulator",
    "ThreeLinkManipulator",
    "TwoLinkManipulator",
    "UR5Manipulator",
]
