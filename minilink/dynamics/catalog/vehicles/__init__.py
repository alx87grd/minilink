"""Vehicle dynamics — the four-rung teaching ladder plus propulsion / suspension.

``HolonomicMobileRobot`` → ``KinematicBicycle`` / ``KinematicCar`` →
``DynamicBicycle`` (linear tires, named ``w_rear`` / ``delta`` ports) →
``BicycleDynRate`` (wheel-rate / steer-rate inputs, the MPC plant). Research
rungs live in ``examples/projects/car_trajopt/vehicles/``.
"""

# Tire math (pure functions ``tire_slip`` / ``linear_tire_forces``) stays in
# minilink.dynamics.catalog.vehicles.dynamic_bicycle — plants only in __all__.
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
]
