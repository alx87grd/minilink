"""Vehicle dynamics: the teaching ladder from the holonomic point to the UdeS racecar, plus propulsion and suspension."""

# The tire laws and slips (``tires``) and the parameter sets stay in their modules:
# plants only in __all__.
from minilink.dynamics.catalog.vehicles.drivetrain_bicycle import DrivetrainBicycle
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    BicycleDynRate,
    DynamicBicycle,
)
from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
from minilink.dynamics.catalog.vehicles.propulsion import (
    LongitudinalSlipCar,
    LongitudinalTorqueCar,
)
from minilink.dynamics.catalog.vehicles.racecar import (
    UdeSRacecar,
    UdeSRacecarDyn,
)
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
    KinematicBicycle,
)
from minilink.dynamics.catalog.vehicles.suspension import QuarterCar

__all__ = [
    "BicycleDynRate",
    "DrivetrainBicycle",
    "DynamicBicycle",
    "HolonomicMobileRobot",
    "KinematicBicycle",
    "LongitudinalTorqueCar",
    "LongitudinalSlipCar",
    "MountainCar",
    "QuarterCar",
    "UdeSRacecar",
    "UdeSRacecarDyn",
]
