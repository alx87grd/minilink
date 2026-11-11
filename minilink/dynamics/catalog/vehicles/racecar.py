"""The UdeS 1/10-scale racecar: its numbers and its look on the kinematic and drivetrain bicycles."""

import copy
from functools import partial

import numpy as np

from minilink.core.kinematics import SE2, translation
from minilink.dynamics.catalog.vehicles.drivetrain_bicycle import DrivetrainBicycle
from minilink.dynamics.catalog.vehicles.steering import KinematicBicycle
from minilink.dynamics.catalog.vehicles.tires import BrushTire
from minilink.graphical.catalog.skins import car_skin_2d

# Geometry and actuators of the 1/10 car.
PUBLIC_RACECAR_PARAMS = {
    # Chassis
    "mass": 5.0,  # [kg]
    "inertia": 0.06,  # [kg m^2]
    "a": 0.17,  # [m]    CG to front axle (wheelbase 0.34 m, mass centred)
    "b": 0.17,  # [m]    CG to rear axle
    "r_f": 0.05,  # [m]    front tire radius
    "r_r": 0.05,  # [m]    rear tire radius
    "gravity": 9.81,  # [m/s^2]
    # Aero and rolling resistance
    "rho": 1.2,  # [kg/m^3] air at room temperature
    "CdA": 0.03,  # [m^2]  frontal area at Cd ~ 1 for an open 1/10 body
    "C_rr": 0.02,  # [-]   rolling resistance of a rubber tire on a hard floor
    # Propulsion (what the drive can deliver, not how it is built)
    "P_max": 80.0,  # [W]
    "tau_sat": 1.5,  # [N m] torque the drivetrain holds at the rear axle at rest
    "engine_tau": 0.1,  # [s] lag from the command to the power on the ground
    "Jw_rear": 0.002,  # [kg m^2] rear wheels plus the drivetrain
    "bw_drive": 0.001,  # [N m s] viscous drag of the drivetrain
    "bw_rear": 0.0,  # [N m s] the base class's own wheel drag: f() adds bw_drive instead
    "tau_fric": 0.02,  # [N m] dry friction of the drivetrain
    # Steering
    "delta_max": 0.52,  # [rad] 30 deg
    "steering_tau": 0.08,  # [s] hobby servo closing its own position loop
    "steer_rate_max": 5.0,  # [rad/s] ~0.2 s per 60 deg
    # Smoothing scales: they keep the equations finite and differentiable near rest
    "w_min_epsilon": 1.0,  # [rad/s] floor of the power-to-torque denominator
    "v_c": 0.05,  # [m/s]   creep speed of the rolling resistance
    "w_c": 1.0,  # [rad/s]  creep rate of the drivetrain dry friction
}


###############################################################################
# Public API
###############################################################################


class UdeSRacecar(KinematicBicycle):
    """Kinematic bicycle at the 1/10 racecar scale.

    Same equations as :class:`KinematicBicycle`, with ``a = b`` from
    :data:`PUBLIC_RACECAR_PARAMS`. ``tf`` publishes the link frames, so the 3-D
    look attaches with ``car.skin = racecar_skin_3d``. There is no rolling angle
    in the state: the wheels steer, they do not spin.
    """

    def __init__(self):
        super().__init__()

        # Parameters: the 1/10 car's wheelbase and rear wheel radius
        a = PUBLIC_RACECAR_PARAMS["a"]
        b = PUBLIC_RACECAR_PARAMS["b"]
        r = PUBLIC_RACECAR_PARAMS["r_r"]
        self.params["a"] = a
        self.params["b"] = b
        self.params["r_r"] = r

        # Graphics: 1/10-scale wheels, and a camera that follows the body
        self.wheel_len = 2.0 * r
        self.wheel_width = 0.045
        self.wheel_radius = r
        self.track = 0.20
        self.camera_follow_frame = "body"
        self.camera_scale = 2.0 * (a + b)

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        return link_frames(x[0], x[1], x[2], u[1], params, self.track)


class UdeSRacecarDyn(DrivetrainBicycle):
    """1/10-scale racecar: the drivetrain bicycle with the UdeS racecar's numbers.

    The equations, inputs, outputs and options (``named_ports``, ``tire``,
    ``wheel_angles``) are :class:`DrivetrainBicycle`'s; this preset sets
    :data:`PUBLIC_RACECAR_PARAMS`, its two tires and the 1/10-scale look. A proxy
    that behaves roughly like the real car, for testing ideas, not an identified
    model of it. Its tires are brush tires on soft rubber (``mu`` 1 on a smooth hard
    floor), 4 /rad of cornering stiffness per unit load in front and 5 /rad behind,
    so the car understeers at the limit, 10 of longitudinal stiffness, and a slip
    floor of 0.2 m/s; a ``tire`` given replaces them, its own numbers included.

    ``tf`` publishes the link frames, so ``car.skin = racecar_skin_3d`` attaches the
    3-D look; its wheels spin when ``wheel_angles=True`` puts the rolling angles in
    the state.
    """

    def __init__(self, named_ports=False, tire=None, wheel_angles=False):
        # The 1/10 car's tires: the rear stiffer than the front
        if tire is None:
            tire = {
                "f": BrushTire(mu=1.0, c_alpha=4.0, c_kappa=10.0, v_eps=0.2),
                "r": BrushTire(mu=1.0, c_alpha=5.0, c_kappa=10.0, v_eps=0.2),
            }
        super().__init__(named_ports=named_ports, tire=tire, wheel_angles=wheel_angles)

        # Parameters: the 1/10 car's numbers, its tires' held beside them
        self.params.update(PUBLIC_RACECAR_PARAMS)

        # Ports: the 1/10 car's command limits
        self.bound_commands()

        # Graphics: a 1/10-scale car and a camera that follows it
        self.wheel_len = 2.0 * self.params["r_r"]
        self.wheel_width = 0.045
        self.wheel_radius = self.params["r_r"]
        self.track = 0.20
        self.skin = partial(car_skin_2d, color="#1a1a1a")
        self.camera_follow_frame = "body"
        self.camera_scale = 1.5

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        if self.wheel_angles:
            return link_frames(x[0], x[1], x[2], x[7], params, self.track, x[9], x[10])

        return link_frames(x[0], x[1], x[2], x[7], params, self.track)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """The axle arrows of the 2-D look; none on the car whose wheels turn."""
        if self.wheel_angles:
            return {}

        return super().get_dynamic_geometry(x, u, t, params)


def stiff_tires(params):
    """The same car with rigid tires: the no-slip limit, for tests and comparisons.

    Multiplies every slip stiffness and the friction coefficient so that the axles hold
    their contact conditions, which turns the dynamic bicycle into a kinematic one.
    """
    rigid = copy.deepcopy(params)
    for tire in rigid["tire"].values():
        tire["c_alpha"] = 1.0e4
        tire["c_kappa"] = 1.0e4
        tire["mu"] = 1.0e3
    rigid["C_rr"] = 0.0

    return rigid


###############################################################################
# Internal machinery
###############################################################################


def link_frames(X, Y, psi, delta, params, track, phi_rear=0.0, phi_front=None):
    """Link poses for either racecar plant, and the front wheels every car look reads."""
    # Lazy import: the graphics load on first use, not with the core
    from minilink.graphical.catalog.racecar_skin import racecar_frames

    b = params["b"]
    a = params["a"]
    r_r = params["r_r"]
    r_f = params.get("r_f", r_r)  # The kinematic racecar has one wheel radius

    wheelbase = a + b
    T_body = SE2(X, Y, psi)
    R_steer = SE2(0.0, 0.0, delta)

    frames = racecar_frames(
        X,
        Y,
        psi,
        delta,
        phi_rear=phi_rear,
        phi_front=phi_front,
        ref_to_rear_axle=b,
        wheelbase=wheelbase,
        wheel_radius=r_r,
    )
    frames["wheel_fl"] = T_body @ translation(a, 0.5 * track, r_f) @ R_steer
    frames["wheel_fr"] = T_body @ translation(a, -0.5 * track, r_f) @ R_steer

    return frames


if __name__ == "__main__":
    from minilink.graphical.catalog.racecar_skin import racecar_skin_3d

    car = UdeSRacecar()  # or UdeSRacecarDyn()
    car.skin = racecar_skin_3d

    car.inputs["u"].nominal_value = np.array([1.0, 0.2])

    car.plot_trajectory()
    car.animate(renderer="meshcat")

    car = UdeSRacecarDyn(wheel_angles=True)
    car.skin = racecar_skin_3d

    car.inputs["u"].nominal_value = np.array([150.0, 0.2])

    car.compute_trajectory(tf=10.0)

    car.plot_trajectory()
    car.animate(renderer="meshcat")
