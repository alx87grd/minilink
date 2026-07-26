"""Teaching alias of ``dynamics.catalog`` — short plant imports.

Ownership of plant math stays under ``minilink.dynamics.catalog``. This package
only re-exports public teaching classes::

    from minilink.catalog import Pendulum, CartPole, Boat2D
"""

from __future__ import annotations

from importlib import import_module
from typing import Any

# name -> (module path, attribute)
_EXPORTS: dict[str, tuple[str, str]] = {
    # pendulum
    "Acrobot": ("minilink.dynamics.catalog.pendulum", "Acrobot"),
    "CartPole": ("minilink.dynamics.catalog.pendulum", "CartPole"),
    "DoublePendulum": ("minilink.dynamics.catalog.pendulum", "DoublePendulum"),
    "InvertedPendulum": ("minilink.dynamics.catalog.pendulum", "InvertedPendulum"),
    "JaxCartPole": ("minilink.dynamics.catalog.pendulum", "JaxCartPole"),
    "Pendulum": ("minilink.dynamics.catalog.pendulum", "Pendulum"),
    "PendulumWithNoisePort": (
        "minilink.dynamics.catalog.pendulum",
        "PendulumWithNoisePort",
    ),
    "RotatingCartPole": ("minilink.dynamics.catalog.pendulum", "RotatingCartPole"),
    "TwoIndependentPendulums": (
        "minilink.dynamics.catalog.pendulum",
        "TwoIndependentPendulums",
    ),
    "UnderactuatedRotatingCartPole": (
        "minilink.dynamics.catalog.pendulum",
        "UnderactuatedRotatingCartPole",
    ),
    # mass-spring-damper
    "FloatingSingleMass": (
        "minilink.dynamics.catalog.mass_spring_damper",
        "FloatingSingleMass",
    ),
    "FloatingThreeMass": (
        "minilink.dynamics.catalog.mass_spring_damper",
        "FloatingThreeMass",
    ),
    "FloatingTwoMass": (
        "minilink.dynamics.catalog.mass_spring_damper",
        "FloatingTwoMass",
    ),
    "SingleMass": ("minilink.dynamics.catalog.mass_spring_damper", "SingleMass"),
    "ThreeMass": ("minilink.dynamics.catalog.mass_spring_damper", "ThreeMass"),
    "TwoMass": ("minilink.dynamics.catalog.mass_spring_damper", "TwoMass"),
    # equations
    "DoubleIntegrator": ("minilink.dynamics.catalog.equations", "DoubleIntegrator"),
    "SimpleIntegrator": ("minilink.dynamics.catalog.equations", "SimpleIntegrator"),
    "TripleIntegrator": ("minilink.dynamics.catalog.equations", "TripleIntegrator"),
    "VanderPol": ("minilink.dynamics.catalog.equations", "VanderPol"),
    # aerial
    "ConstantSpeedHelicopterTunnel": (
        "minilink.dynamics.catalog.aerial",
        "ConstantSpeedHelicopterTunnel",
    ),
    "Drone2D": ("minilink.dynamics.catalog.aerial", "Drone2D"),
    "Drone2DWithSideThruster": (
        "minilink.dynamics.catalog.aerial",
        "Drone2DWithSideThruster",
    ),
    "Plane2D": ("minilink.dynamics.catalog.aerial", "Plane2D"),
    "Rocket": ("minilink.dynamics.catalog.aerial", "Rocket"),
    "SpeedControlledDrone2D": (
        "minilink.dynamics.catalog.aerial",
        "SpeedControlledDrone2D",
    ),
    # marine
    "Boat2D": ("minilink.dynamics.catalog.marine", "Boat2D"),
    "Boat2DWithCurrent": ("minilink.dynamics.catalog.marine", "Boat2DWithCurrent"),
    # vehicles
    "ConstantSpeedKinematicCar": (
        "minilink.dynamics.catalog.vehicles",
        "ConstantSpeedKinematicCar",
    ),
    "DynamicBicycle": ("minilink.dynamics.catalog.vehicles", "DynamicBicycle"),
    "DynamicBicycleCar3D": (
        "minilink.dynamics.catalog.vehicles",
        "DynamicBicycleCar3D",
    ),
    "DynamicHolonomicMobileRobot": (
        "minilink.dynamics.catalog.vehicles",
        "DynamicHolonomicMobileRobot",
    ),
    "HolonomicMobileRobot": (
        "minilink.dynamics.catalog.vehicles",
        "HolonomicMobileRobot",
    ),
    "HolonomicMobileRobot3D": (
        "minilink.dynamics.catalog.vehicles",
        "HolonomicMobileRobot3D",
    ),
    "KinematicBicycle": ("minilink.dynamics.catalog.vehicles", "KinematicBicycle"),
    "KinematicCar": ("minilink.dynamics.catalog.vehicles", "KinematicCar"),
    "LinearTire": ("minilink.dynamics.catalog.vehicles", "LinearTire"),
    "LongitudinalFrontWheelDriveCarWithTorqueInput": (
        "minilink.dynamics.catalog.vehicles",
        "LongitudinalFrontWheelDriveCarWithTorqueInput",
    ),
    "LongitudinalFrontWheelDriveCarWithWheelSlipInput": (
        "minilink.dynamics.catalog.vehicles",
        "LongitudinalFrontWheelDriveCarWithWheelSlipInput",
    ),
    "MountainCar": ("minilink.dynamics.catalog.vehicles", "MountainCar"),
    "QuarterCarOnRoughTerrain": (
        "minilink.dynamics.catalog.vehicles",
        "QuarterCarOnRoughTerrain",
    ),
    "UdeSRacecar": ("minilink.dynamics.catalog.vehicles", "UdeSRacecar"),
    # manipulators
    "FiveLinkPlanarManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "FiveLinkPlanarManipulator",
    ),
    "OneLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "OneLinkManipulator",
    ),
    "SpeedControlledManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "SpeedControlledManipulator",
    ),
    "ThreeLinkManipulator3D": (
        "minilink.dynamics.catalog.manipulators",
        "ThreeLinkManipulator3D",
    ),
    "TwoLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "TwoLinkManipulator",
    ),
    "UR5Manipulator": ("minilink.dynamics.catalog.manipulators", "UR5Manipulator"),
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str) -> Any:
    try:
        module_path, attr = _EXPORTS[name]
    except KeyError as exc:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}") from exc
    value = getattr(import_module(module_path), attr)
    globals()[name] = value
    return value


def __dir__() -> list[str]:
    return sorted(set(globals()) | set(__all__))
