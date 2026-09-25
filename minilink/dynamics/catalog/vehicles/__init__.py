"""Vehicle dynamics: the teaching ladder from the holonomic point to the UdeS racecar, plus propulsion and suspension."""

# Tire math (``tire_slip`` / ``linear_tire_forces`` in dynamic_bicycle, the brush pair
# in tires) and parameter sets stay in their modules — plants only in __all__.
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    BicycleDynRate,
    DynamicBicycle,
    DynamicBicycleCar3D,
)
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalFrontWheelDriveCarWithTorqueInput,
    LongitudinalFrontWheelDriveCarWithWheelSlipInput,
)
from minilink.dynamics.catalog.vehicles.racecar import (
    UdeSRacecar,
    UdeSRacecarDyn,
    UdeSRacecarDyn3D,
)
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
    KinematicBicycle,
    KinematicCar,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain

__all__ = [
    "BicycleDynRate",
    "DynamicBicycle",
    "DynamicBicycleCar3D",
    "HolonomicMobileRobot",
    "KinematicBicycle",
    "KinematicCar",
    "LongitudinalFrontWheelDriveCarWithTorqueInput",
    "LongitudinalFrontWheelDriveCarWithWheelSlipInput",
    "MountainCar",
    "QuarterCarOnRoughTerrain",
    "UdeSRacecar",
    "UdeSRacecarDyn",
    "UdeSRacecarDyn3D",
]
