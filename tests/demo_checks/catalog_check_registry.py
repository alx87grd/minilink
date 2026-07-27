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
    SimpleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import VanderPol
from minilink.dynamics.catalog.manipulators.arms import (
    FiveLinkPlanarManipulator,
    OneLinkManipulator,
    SpeedControlledManipulator,
    ThreeLinkManipulator3D,
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
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot
from minilink.dynamics.catalog.pendulum.pendulum import (
    InvertedPendulum,
    Pendulum,
    TwoIndependentPendulums,
)
from minilink.dynamics.catalog.vehicles.jax_vehicles import (
    BicycleAcc,
    BicycleKin,
    Holonomic,
    HolonomicAccel,
)
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalFrontWheelDriveCarWithTorqueInput,
    LongitudinalFrontWheelDriveCarWithWheelSlipInput,
)
from minilink.dynamics.catalog.vehicles.steering import (
    ConstantSpeedKinematicCar,
    DynamicHolonomicMobileRobot,
    HolonomicMobileRobot,
    HolonomicMobileRobot3D,
    KinematicBicycle,
    KinematicCar,
    UdeSRacecar,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain

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
    CatalogCheckEntry("SimpleIntegrator", SimpleIntegrator),
    CatalogCheckEntry("DoubleIntegrator", DoubleIntegrator),
    CatalogCheckEntry("TripleIntegrator", TripleIntegrator),
    CatalogCheckEntry("VanderPol", lambda: VanderPol(mu=0.5)),
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
    CatalogCheckEntry("KinematicBicycle", KinematicBicycle),
    CatalogCheckEntry("BicycleKin", BicycleKin, requires_jax=True),
    CatalogCheckEntry(
        "BicycleAcc",
        BicycleAcc,
        requires_jax=True,
    ),
    CatalogCheckEntry("KinematicCar", KinematicCar),
    CatalogCheckEntry("ConstantSpeedKinematicCar", ConstantSpeedKinematicCar),
    CatalogCheckEntry("HolonomicMobileRobot", HolonomicMobileRobot),
    CatalogCheckEntry("DynamicHolonomicMobileRobot", DynamicHolonomicMobileRobot),
    CatalogCheckEntry("Holonomic", Holonomic, requires_jax=True),
    CatalogCheckEntry(
        "HolonomicAccel",
        HolonomicAccel,
        requires_jax=True,
    ),
    CatalogCheckEntry("HolonomicMobileRobot3D", HolonomicMobileRobot3D),
    CatalogCheckEntry("UdeSRacecar", UdeSRacecar),
    CatalogCheckEntry(
        "LongitudinalFrontWheelDriveCarWithWheelSlipInput",
        LongitudinalFrontWheelDriveCarWithWheelSlipInput,
    ),
    CatalogCheckEntry(
        "LongitudinalFrontWheelDriveCarWithTorqueInput",
        LongitudinalFrontWheelDriveCarWithTorqueInput,
    ),
    CatalogCheckEntry("QuarterCarOnRoughTerrain", QuarterCarOnRoughTerrain),
    CatalogCheckEntry("MountainCar", MountainCar),
    CatalogCheckEntry("Drone2D", Drone2D),
    CatalogCheckEntry("Drone2DWithSideThruster", Drone2DWithSideThruster),
    CatalogCheckEntry("SpeedControlledDrone2D", SpeedControlledDrone2D),
    CatalogCheckEntry("ConstantSpeedHelicopterTunnel", ConstantSpeedHelicopterTunnel),
    CatalogCheckEntry("Rocket", Rocket),
    CatalogCheckEntry("Plane2D", Plane2D),
    CatalogCheckEntry("Boat2D", Boat2D),
    CatalogCheckEntry("Boat2DWithCurrent", Boat2DWithCurrent),
    CatalogCheckEntry(
        "SpeedControlledManipulator",
        lambda: SpeedControlledManipulator(2, 2),
    ),
    CatalogCheckEntry("OneLinkManipulator", OneLinkManipulator),
    CatalogCheckEntry("TwoLinkManipulator", TwoLinkManipulator),
    CatalogCheckEntry("ThreeLinkManipulator3D", ThreeLinkManipulator3D),
    CatalogCheckEntry("FiveLinkPlanarManipulator", FiveLinkPlanarManipulator),
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
        "FiveLinkPlanarManipulator",
    ),
    KinematicRenderPlant(
        "drone2d", "minilink.dynamics.catalog.aerial.drone", "Drone2D"
    ),
    KinematicRenderPlant(
        "simple_integrator",
        "minilink.dynamics.catalog.equations.integrators",
        "SimpleIntegrator",
    ),
    KinematicRenderPlant(
        "single_mass",
        "minilink.dynamics.catalog.mass_spring_damper.linear",
        "SingleMass",
    ),
)
