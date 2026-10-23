"""Catalog plant registry for demo-check runners and kinematic render manifests.

Repo-only (under ``tests/demo_checks/``) — not part of the shipped library.
Single source of truth for which catalog classes get headless demo-check coverage.
Import factories only — no matplotlib or simulator imports here.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from minilink.blocks.transfer_function import TransferFunction
from minilink.catalog import (
    DoublePendulum,
    DynamicBicycle,
    DynamicBicycleCar3D,
    NoisyPendulum,
    Plane3D,
    UdeSRacecar,
    UdeSRacecarDyn,
    UdeSRacecarDyn3D,
)
from minilink.dynamics.catalog.aerial.drone import (
    ConstantSpeedHelicopterTunnel,
    Drone2D,
    Drone2DWithSideThruster,
    SpeedControlledDrone2D,
)
from minilink.dynamics.catalog.aerial.plane import Plane2D
from minilink.dynamics.catalog.aerial.rocket import Rocket
from minilink.dynamics.catalog.astro.three_body import ThreeBodyProblem
from minilink.dynamics.catalog.equations.integrators import (
    DoubleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import Lorenz, VanderPol
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkManipulator,
    KinematicManipulator,
    OneLinkManipulator,
    ThreeLinkManipulator,
    TwoLinkManipulator,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator
from minilink.dynamics.catalog.marine.boat import Boat2D, Boat2DWithCurrent
from minilink.dynamics.catalog.mass_spring_damper.linear import (
    FloatingSingleMass,
    FloatingThreeMass,
    FloatingTwoMass,
    SingleMass,
    ThreeMass,
    TwoMass,
)
from minilink.dynamics.catalog.pendulum.cartpole import (
    CartPole,
    NoisyCartPole,
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot
from minilink.dynamics.catalog.pendulum.pendulum import (
    InvertedPendulum,
    Pendulum,
    TwoIndependentPendulums,
)
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import BicycleDynRate
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalSlipCar,
    LongitudinalTorqueCar,
)
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
    KinematicBicycle,
    KinematicCar,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCar

PlantFactory = Callable[[], Any]


@dataclass(frozen=True)
class CatalogCheckEntry:
    """One catalog class exercised by catalog demo checks."""

    id: str
    factory: PlantFactory
    requires_jax: bool = False


@dataclass(frozen=True)
class KinematicRenderPlant:
    """Subset of catalog plants with committed kinematic render manifest entries."""

    name: str
    module: str
    cls: str
    requires: tuple[str, ...] = ()


CATALOG_CHECK_ENTRIES: tuple[CatalogCheckEntry, ...] = (
    CatalogCheckEntry("DoubleIntegrator", DoubleIntegrator),
    CatalogCheckEntry("TripleIntegrator", TripleIntegrator),
    CatalogCheckEntry("VanderPol", lambda: VanderPol(mu=0.5)),
    CatalogCheckEntry("Lorenz", Lorenz),
    CatalogCheckEntry("ThreeBodyProblem", ThreeBodyProblem),
    CatalogCheckEntry(
        "TransferFunction",
        lambda: TransferFunction([1.0], [1.0, 1.0]),
    ),
    CatalogCheckEntry("SingleMass", SingleMass),
    CatalogCheckEntry("TwoMass", TwoMass),
    CatalogCheckEntry("ThreeMass", ThreeMass),
    CatalogCheckEntry("FloatingSingleMass", FloatingSingleMass),
    CatalogCheckEntry("FloatingTwoMass", FloatingTwoMass),
    CatalogCheckEntry("FloatingThreeMass", FloatingThreeMass),
    CatalogCheckEntry("Pendulum", Pendulum),
    CatalogCheckEntry("InvertedPendulum", InvertedPendulum),
    CatalogCheckEntry("TwoIndependentPendulums", TwoIndependentPendulums),
    CatalogCheckEntry("Acrobot", Acrobot),
    CatalogCheckEntry("RotatingCartPole", RotatingCartPole),
    CatalogCheckEntry("UnderactuatedRotatingCartPole", UnderactuatedRotatingCartPole),
    CatalogCheckEntry("CartPole", CartPole),
    CatalogCheckEntry("NoisyCartPole", NoisyCartPole),
    CatalogCheckEntry("KinematicBicycle", KinematicBicycle),
    CatalogCheckEntry("KinematicCar", KinematicCar),
    CatalogCheckEntry("HolonomicMobileRobot", HolonomicMobileRobot),
    CatalogCheckEntry("BicycleDynRate", BicycleDynRate),
    CatalogCheckEntry(
        "LongitudinalSlipCar",
        LongitudinalSlipCar,
    ),
    CatalogCheckEntry(
        "LongitudinalTorqueCar",
        LongitudinalTorqueCar,
    ),
    CatalogCheckEntry("QuarterCar", QuarterCar),
    CatalogCheckEntry("MountainCar", MountainCar),
    CatalogCheckEntry("Drone2D", Drone2D),
    CatalogCheckEntry("Drone2DWithSideThruster", Drone2DWithSideThruster),
    CatalogCheckEntry("SpeedControlledDrone2D", SpeedControlledDrone2D),
    CatalogCheckEntry("ConstantSpeedHelicopterTunnel", ConstantSpeedHelicopterTunnel),
    CatalogCheckEntry("Rocket", Rocket),
    CatalogCheckEntry("Plane2D", Plane2D),
    CatalogCheckEntry("Plane3D", Plane3D),
    CatalogCheckEntry("DoublePendulum", DoublePendulum),
    CatalogCheckEntry("NoisyPendulum", NoisyPendulum),
    CatalogCheckEntry("DynamicBicycle", DynamicBicycle),
    CatalogCheckEntry("UdeSRacecar", UdeSRacecar),
    CatalogCheckEntry("UdeSRacecarDyn", UdeSRacecarDyn),
    CatalogCheckEntry("UdeSRacecarDyn3D", UdeSRacecarDyn3D),
    CatalogCheckEntry("DynamicBicycleCar3D", DynamicBicycleCar3D),
    CatalogCheckEntry("Boat2D", Boat2D),
    CatalogCheckEntry("Boat2DWithCurrent", Boat2DWithCurrent),
    CatalogCheckEntry(
        "KinematicManipulator",
        lambda: KinematicManipulator(2, 2),
    ),
    CatalogCheckEntry("OneLinkManipulator", OneLinkManipulator),
    CatalogCheckEntry("TwoLinkManipulator", TwoLinkManipulator),
    CatalogCheckEntry("ThreeLinkManipulator", ThreeLinkManipulator),
    CatalogCheckEntry("FiveLinkManipulator", FiveLinkManipulator),
    CatalogCheckEntry("UR5Manipulator", UR5Manipulator),
)

_VEHICLES = "minilink.dynamics.catalog.vehicles"
_PENDULUM = "minilink.dynamics.catalog.pendulum"

KINEMATIC_RENDER_PLANTS: tuple[KinematicRenderPlant, ...] = (
    KinematicRenderPlant(
        "dynamic_bicycle", f"{_VEHICLES}.dynamic_bicycle", "DynamicBicycle"
    ),
    KinematicRenderPlant(
        "dynamic_bicycle_car3d", f"{_VEHICLES}.dynamic_bicycle", "DynamicBicycleCar3D"
    ),
    KinematicRenderPlant(
        "kinematic_bicycle", f"{_VEHICLES}.steering", "KinematicBicycle"
    ),
    KinematicRenderPlant(
        "holonomic_mobile_robot", f"{_VEHICLES}.steering", "HolonomicMobileRobot"
    ),
    KinematicRenderPlant("pendulum", f"{_PENDULUM}.pendulum", "Pendulum"),
    KinematicRenderPlant("cartpole", f"{_PENDULUM}.cartpole", "CartPole"),
    KinematicRenderPlant(
        "two_link_manipulator",
        "minilink.dynamics.catalog.manipulators.arms",
        "TwoLinkManipulator",
    ),
    KinematicRenderPlant(
        "five_link_planar_manipulator",
        "minilink.dynamics.catalog.manipulators.arms",
        "FiveLinkManipulator",
    ),
    KinematicRenderPlant(
        "drone2d", "minilink.dynamics.catalog.aerial.drone", "Drone2D"
    ),
    KinematicRenderPlant(
        "single_mass",
        "minilink.dynamics.catalog.mass_spring_damper.linear",
        "SingleMass",
    ),
)
