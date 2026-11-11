"""Two presets of the 3-D car: an open-wheel formula car and an off-road side-by-side."""

from minilink.dynamics.catalog.vehicles.suspension_car import (
    SpringDamper,
    SuspensionCar3D,
)
from minilink.dynamics.catalog.vehicles.tires import BrushTire

# The racer's numbers
FORMULA_CAR_PARAMS = {
    "mass": 800.0,  # [kg] with its driver
    "inertia": 1100.0,  # [kg m^2] yaw
    "Ixx": 250.0,  # [kg m^2] roll
    "Iyy": 1000.0,  # [kg m^2] pitch
    "a": 1.9,  # [m]
    "b": 1.6,  # [m]
    "track": 1.6,  # [m]
    "h_cg": 0.30,  # [m]
    "r_f": 0.33,  # [m]
    "r_r": 0.34,  # [m]
    "CdA": 1.2,  # [m^2]
    "C_rr": 0.012,  # [-]
    "P_max": 600000.0,  # [W]
    "tau_sat": 5000.0,  # [N m]
    "engine_tau": 0.05,  # [s]
    "Jw_rear": 3.0,  # [kg m^2]
    "bw_drive": 0.2,  # [N m s]
    "tau_fric": 20.0,  # [N m]
    "delta_max": 0.35,  # [rad]
    "steering_tau": 0.05,  # [s]
    "steer_rate_max": 3.0,  # [rad/s]
}

# The buggy's numbers
OFFROAD_CAR_PARAMS = {
    "mass": 800.0,  # [kg] with its driver
    "inertia": 900.0,  # [kg m^2] yaw
    "Ixx": 300.0,  # [kg m^2] roll
    "Iyy": 800.0,  # [kg m^2] pitch
    "a": 1.15,  # [m]
    "b": 1.25,  # [m]
    "track": 1.6,  # [m]
    "h_cg": 0.65,  # [m]
    "r_f": 0.37,  # [m]
    "r_r": 0.37,  # [m]
    "CdA": 1.6,  # [m^2]
    "C_rr": 0.03,  # [-] rolling on dirt
    "P_max": 130000.0,  # [W]
    "tau_sat": 3000.0,  # [N m]
    "engine_tau": 0.15,  # [s]
    "Jw_rear": 2.5,  # [kg m^2]
    "bw_drive": 0.3,  # [N m s]
    "tau_fric": 15.0,  # [N m]
    "delta_max": 0.6,  # [rad]
    "steering_tau": 0.08,  # [s]
    "steer_rate_max": 2.0,  # [rad/s]
}

###############################################################################
# Public API
###############################################################################


class FormulaCar3D(SuspensionCar3D):
    """A generic open-wheel formula car: light, low, stiff, powerful, on grippy slicks.

    The 3-D car with a racer's numbers and its look
    (:func:`~minilink.graphical.catalog.car_skins_3d.formula_car_skin_3d`): 800 kg on
    a 3.5 m wheelbase, the centre of gravity 0.30 m high, 600 kW, slicks of grip 1.6
    and a stiff suspension (a heave mode near 3.5 Hz). No aerodynamic downforce. A
    plausible cartoon, not a model of any real car; a ``tire`` or ``suspension``
    given replaces the preset's, its own numbers included.
    """

    def __init__(
        self,
        named_ports=False,
        tire=None,
        suspension=None,
        terrain=None,
        wheel_angles=False,
    ):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.car_skins_3d import formula_car_skin_3d

        # The racer's parts: slicks and stiff springs
        if tire is None:
            tire = BrushTire(mu=1.6, c_alpha=18.0, c_kappa=22.0, v_eps=0.5)
        if suspension is None:
            suspension = SpringDamper(k=96700.0, c=3500.0, F_eps=50.0)
        super().__init__(
            named_ports=named_ports,
            tire=tire,
            suspension=suspension,
            terrain=terrain,
            wheel_angles=wheel_angles,
        )

        # Parameters: the racer's numbers, the command limits and the rest state following
        self.set_numbers(FORMULA_CAR_PARAMS)

        # Graphics: the racer's look
        self.skin = formula_car_skin_3d
        self.camera_scale = 9.0


class OffRoadCar3D(SuspensionCar3D):
    """A generic off-road side-by-side: tall, short, softly sprung, on big knobby tires.

    The 3-D car with a utility buggy's numbers and its look
    (:func:`~minilink.graphical.catalog.car_skins_3d.offroad_car_skin_3d`): 800 kg on
    a 2.4 m wheelbase, the centre of gravity 0.65 m high (a car that can roll over),
    130 kW, tires of grip 0.9 and a soft, long-travel suspension (a heave mode near
    1.3 Hz). A plausible cartoon, not a model of any real vehicle; a ``tire`` or
    ``suspension`` given replaces the preset's, its own numbers included.
    """

    def __init__(
        self,
        named_ports=False,
        tire=None,
        suspension=None,
        terrain=None,
        wheel_angles=False,
    ):
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.catalog.car_skins_3d import offroad_car_skin_3d

        # The buggy's parts: knobby tires and soft springs
        if tire is None:
            tire = BrushTire(mu=0.9, c_alpha=8.0, c_kappa=10.0, v_eps=0.5)
        if suspension is None:
            suspension = SpringDamper(k=13350.0, c=1150.0, F_eps=50.0)
        super().__init__(
            named_ports=named_ports,
            tire=tire,
            suspension=suspension,
            terrain=terrain,
            wheel_angles=wheel_angles,
        )

        # Parameters: the buggy's numbers, the command limits and the rest state following
        self.set_numbers(OFFROAD_CAR_PARAMS)

        # Graphics: the buggy's look
        self.skin = offroad_car_skin_3d
        self.camera_scale = 7.0
