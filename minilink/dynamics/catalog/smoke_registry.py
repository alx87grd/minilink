"""Catalog plant registry for L6 smoke runners and kinematic render manifests.

Single source of truth for which catalog classes get headless smoke coverage.
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
    JaxDynamicHolonomicMobileRobot,
    JaxHolonomicMobileRobot,
    JaxKinematicBicycle,
    JaxKinematicBicycleRateInputs,
    KinematicBicycle,
    KinematicCar,
    UdeSRacecar,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain

PlantFactory = Callable[[], Any]


@dataclass(frozen=True)
class CatalogSmokeEntry:
    """One catalog class exercised by L6 smokes."""

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


CATALOG_SMOKE_ENTRIES: tuple[CatalogSmokeEntry, ...] = (
    CatalogSmokeEntry("SimpleIntegrator", SimpleIntegrator),
    CatalogSmokeEntry("DoubleIntegrator", DoubleIntegrator),
    CatalogSmokeEntry("TripleIntegrator", TripleIntegrator),
    CatalogSmokeEntry("VanderPol", lambda: VanderPol(mu=0.5)),
    CatalogSmokeEntry(
        "TransferFunction",
        lambda: TransferFunction([1.0], [1.0, 1.0]),
    ),
    CatalogSmokeEntry("SingleMass", SingleMass),
    CatalogSmokeEntry("TwoMass", TwoMass),
    CatalogSmokeEntry("ThreeMass", ThreeMass),
    CatalogSmokeEntry("FloatingSingleMass", FloatingSingleMass),
    CatalogSmokeEntry("FloatingTwoMass", FloatingTwoMass),
    CatalogSmokeEntry("FloatingThreeMass", FloatingThreeMass),
    CatalogSmokeEntry("Pendulum", Pendulum),
    CatalogSmokeEntry("InvertedPendulum", InvertedPendulum),
    CatalogSmokeEntry("TwoIndependentPendulums", TwoIndependentPendulums),
    CatalogSmokeEntry("Acrobot", Acrobot),
    CatalogSmokeEntry("RotatingCartPole", RotatingCartPole),
    CatalogSmokeEntry("UnderactuatedRotatingCartPole", UnderactuatedRotatingCartPole),
    CatalogSmokeEntry("CartPole", CartPole),
    CatalogSmokeEntry("KinematicBicycle", KinematicBicycle),
    CatalogSmokeEntry("JaxKinematicBicycle", JaxKinematicBicycle, requires_jax=True),
    CatalogSmokeEntry(
        "JaxKinematicBicycleRateInputs",
        JaxKinematicBicycleRateInputs,
        requires_jax=True,
    ),
    CatalogSmokeEntry("KinematicCar", KinematicCar),
    CatalogSmokeEntry("ConstantSpeedKinematicCar", ConstantSpeedKinematicCar),
    CatalogSmokeEntry("HolonomicMobileRobot", HolonomicMobileRobot),
    CatalogSmokeEntry("DynamicHolonomicMobileRobot", DynamicHolonomicMobileRobot),
    CatalogSmokeEntry(
        "JaxHolonomicMobileRobot", JaxHolonomicMobileRobot, requires_jax=True
    ),
    CatalogSmokeEntry(
        "JaxDynamicHolonomicMobileRobot",
        JaxDynamicHolonomicMobileRobot,
        requires_jax=True,
    ),
    CatalogSmokeEntry("HolonomicMobileRobot3D", HolonomicMobileRobot3D),
    CatalogSmokeEntry("UdeSRacecar", UdeSRacecar),
    CatalogSmokeEntry(
        "LongitudinalFrontWheelDriveCarWithWheelSlipInput",
        LongitudinalFrontWheelDriveCarWithWheelSlipInput,
    ),
    CatalogSmokeEntry(
        "LongitudinalFrontWheelDriveCarWithTorqueInput",
        LongitudinalFrontWheelDriveCarWithTorqueInput,
    ),
    CatalogSmokeEntry("QuarterCarOnRoughTerrain", QuarterCarOnRoughTerrain),
    CatalogSmokeEntry("MountainCar", MountainCar),
    CatalogSmokeEntry("Drone2D", Drone2D),
    CatalogSmokeEntry("Drone2DWithSideThruster", Drone2DWithSideThruster),
    CatalogSmokeEntry("SpeedControlledDrone2D", SpeedControlledDrone2D),
    CatalogSmokeEntry("ConstantSpeedHelicopterTunnel", ConstantSpeedHelicopterTunnel),
    CatalogSmokeEntry("Rocket", Rocket),
    CatalogSmokeEntry("Plane2D", Plane2D),
    CatalogSmokeEntry("Boat2D", Boat2D),
    CatalogSmokeEntry("Boat2DWithCurrent", Boat2DWithCurrent),
    CatalogSmokeEntry(
        "SpeedControlledManipulator",
        lambda: SpeedControlledManipulator(2, 2),
    ),
    CatalogSmokeEntry("OneLinkManipulator", OneLinkManipulator),
    CatalogSmokeEntry("TwoLinkManipulator", TwoLinkManipulator),
    CatalogSmokeEntry("ThreeLinkManipulator3D", ThreeLinkManipulator3D),
    CatalogSmokeEntry("FiveLinkPlanarManipulator", FiveLinkPlanarManipulator),
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
