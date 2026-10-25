"""Catalog: the plant models of every domain, for short teaching imports."""

from __future__ import annotations

from minilink.core.exports import lazy_facade

# The teaching alias of ``dynamics.catalog``, for short plant imports, e.g.
# ``from minilink.catalog import Pendulum, CartPole, Boat2D``. The plant math stays
# under ``minilink.dynamics.catalog``; this package only re-exports the public
# teaching classes. Each exported name maps to its (module path, attribute).
_EXPORTS: dict[str, tuple[str, str]] = {
    # Pendulum
    "Acrobot": ("minilink.dynamics.catalog.pendulum", "Acrobot"),
    "CartPole": ("minilink.dynamics.catalog.pendulum", "CartPole"),
    "NoisyCartPole": (
        "minilink.dynamics.catalog.pendulum",
        "NoisyCartPole",
    ),
    "DoublePendulum": ("minilink.dynamics.catalog.pendulum", "DoublePendulum"),
    "InvertedPendulum": ("minilink.dynamics.catalog.pendulum", "InvertedPendulum"),
    "Pendulum": ("minilink.dynamics.catalog.pendulum", "Pendulum"),
    "NoisyPendulum": (
        "minilink.dynamics.catalog.pendulum",
        "NoisyPendulum",
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
    # Mass-spring-damper
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
    # Equations
    "DoubleIntegrator": ("minilink.dynamics.catalog.equations", "DoubleIntegrator"),
    "TripleIntegrator": ("minilink.dynamics.catalog.equations", "TripleIntegrator"),
    "VanderPol": ("minilink.dynamics.catalog.equations", "VanderPol"),
    "Lorenz": ("minilink.dynamics.catalog.equations", "Lorenz"),
    # Astro
    "ThreeBodyProblem": ("minilink.dynamics.catalog.astro", "ThreeBodyProblem"),
    # Aerial
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
    "Plane3D": ("minilink.dynamics.catalog.aerial", "Plane3D"),
    "Rocket": ("minilink.dynamics.catalog.aerial", "Rocket"),
    "SpeedControlledDrone2D": (
        "minilink.dynamics.catalog.aerial",
        "SpeedControlledDrone2D",
    ),
    # Marine
    "Boat2D": ("minilink.dynamics.catalog.marine", "Boat2D"),
    "Boat2DWithCurrent": ("minilink.dynamics.catalog.marine", "Boat2DWithCurrent"),
    # Vehicles
    "BicycleDynRate": ("minilink.dynamics.catalog.vehicles", "BicycleDynRate"),
    "DynamicBicycle": ("minilink.dynamics.catalog.vehicles", "DynamicBicycle"),
    "DynamicBicycleCar3D": (
        "minilink.dynamics.catalog.vehicles",
        "DynamicBicycleCar3D",
    ),
    "HolonomicMobileRobot": (
        "minilink.dynamics.catalog.vehicles",
        "HolonomicMobileRobot",
    ),
    "KinematicBicycle": ("minilink.dynamics.catalog.vehicles", "KinematicBicycle"),
    "KinematicCar": ("minilink.dynamics.catalog.vehicles", "KinematicCar"),
    "UdeSRacecar": ("minilink.dynamics.catalog.vehicles", "UdeSRacecar"),
    "UdeSRacecarDyn": ("minilink.dynamics.catalog.vehicles", "UdeSRacecarDyn"),
    "UdeSRacecarDyn3D": (
        "minilink.dynamics.catalog.vehicles",
        "UdeSRacecarDyn3D",
    ),
    "LongitudinalTorqueCar": (
        "minilink.dynamics.catalog.vehicles",
        "LongitudinalTorqueCar",
    ),
    "LongitudinalSlipCar": (
        "minilink.dynamics.catalog.vehicles",
        "LongitudinalSlipCar",
    ),
    "MountainCar": ("minilink.dynamics.catalog.vehicles", "MountainCar"),
    "QuarterCar": (
        "minilink.dynamics.catalog.vehicles",
        "QuarterCar",
    ),
    # Manipulators
    "FiveLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "FiveLinkManipulator",
    ),
    "OneLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "OneLinkManipulator",
    ),
    "KinematicManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "KinematicManipulator",
    ),
    "ThreeLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "ThreeLinkManipulator",
    ),
    "TwoLinkManipulator": (
        "minilink.dynamics.catalog.manipulators",
        "TwoLinkManipulator",
    ),
    "UR5Manipulator": ("minilink.dynamics.catalog.manipulators", "UR5Manipulator"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
