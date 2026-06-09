import matplotlib.pyplot as plt
import numpy as np

from projects.bicycle.control.measurement import Measurement
from minilink.control.pid import Sum
from projects.bicycle.control.pid import InstrumentedPID as PID
from minilink.control.references import ConstantReference
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.graphical.animation.primitives import (
    Point,
    pose2d_matrix,
)
from projects.bicycle.control.motor_map import AccToRearForce, ThrMap
from projects.bicycle.control.steering_map import AngularSpeedToSteeringMap
from projects.bicycle.models.dynamic_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from projects.bicycle.simulations.vehicule_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)

VX_REF = 2.0  # m/s
R_REF = 0.0  # rad/s

SIMULATION_TIME = 10  # seconds


class Trajectory(System):
    """Hardcoded sinusoid lane definition and cruise speed broadcast.

    Output ``path`` is ``[A, lambda]'`` for ``y(x) = A sin(2 pi x / lambda)``.
    Pure-pursuit lookahead is a separate parameter on :class:`Tracking`.

    Outputs
    -------
    u_ref
        Reference body surge speed [m/s] for the velocity loop (single scalar).
    path
        ``[amplitude, wavelength]'`` [m].
    """

    def __init__(
        self,
        x_traj=None,
        y_traj=None,
    ):
        super().__init__(0)
        self.name = "Path planner"
        # Correctly store provided trajectory callables
        self.x_traj = x_traj
        self.y_traj = y_traj

        self.add_output_port("x_targ", dim=1, function=self.x_targ, dependencies=())
        self.add_output_port("y_targ", dim=1, function=self.y_targ, dependencies=())

    def x_targ(self, x, u, t=0.0, params=None):
        x_targ = 0.0

        if self.x_traj is not None:
            x_targ = self.x_traj(t)
        return np.array([x_targ], dtype=float)

    def y_targ(self, x, u, t=0.0, params=None):
        y_targ = 0.0

        if self.y_traj is not None:
            y_targ = self.y_traj(t)
        return np.array([y_targ], dtype=float)

    def get_kinematic_geometry(self):
        # Only show a single point primitive; the renderer will position it
        # using the transform returned by get_kinematic_transforms.
        return [Point(pt=[0.0, 0.0, 0.0], color="orange", marker="o", size=8)]

    def get_kinematic_transforms(self, x, u, t):
        # Compute the current target point position and return a single transform
        # for the Point primitive.
        x_t = 0.0
        y_t = 0.0
        if self.x_traj is not None:
            # Trajectory may return scalar or array; ensure scalar for time t
            x_val = self.x_traj(t)
            x_t = float(np.asarray(x_val).reshape(1)[0])
        if self.y_traj is not None:
            y_val = self.y_traj(t)
            y_t = float(np.asarray(y_val).reshape(1)[0])
        return [pose2d_matrix(x=x_t, y=y_t, theta=0.0)]


def x_pos(t):
    return VX_REF * t


def y_pos(t):
    ys = VX_REF * np.sin(0.2 * np.pi * t / 1.0)
    return ys


def create_diagram(
    vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF, r_ref=R_REF
):

    r_to_steering = AngularSpeedToSteeringMap(vehicle)
    steering = ConstantReference(ref=r_ref, name="Constant angular speed")
    speed_const = ConstantReference(ref=vx_ref, name="Constant linear speed")

    speed_meas = Measurement(name="Speed measurement", y_size=12, index=5, show=False)
    x_pos_meas = Measurement(name="Speed measurement", y_size=12, index=0, show=False)

    ang_speed_meas = Measurement(
        name="Angular speed measurement", y_size=12, index=7, show=False
    )

    rear_speed_meas = Measurement(
        name="Rear wheel speed",
        y_size=12,
        index=8,
    )

    x_pid = PID(
        Kp=2.0,
        Ki=0.0,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-5.0,
        i_max=5.0,
        name="X pos PID",
    )

    v_pid = PID(
        Kp=0.8,
        Ki=0.01,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-0.0,
        i_max=5.0,
        name="Speed PID",
    )

    thr_map = ThrMap(vehicle)
    acc_to_force = AccToRearForce(vehicle)

    r_pid = PID(
        Kp=3.2,
        Ki=1.5,
        Kd=0.1,
        cmd_min=-np.pi / 2.0,
        cmd_max=np.pi / 2.0,
        i_min=-np.pi / 2.0,
        i_max=np.pi / 2.0,
        name="Yaw rate PID",
    )

    sum_bloc = Sum(max=np.pi / 2.0, min=-np.pi / 2.0)

    trajectory = Trajectory(x_traj=x_pos, y_traj=y_pos)

    diagram = DiagramSystem()
    diagram.name = "Cascade PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(r_to_steering, "r_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")
    diagram.add_subsystem(v_pid, "v_pid")
    # diagram.add_subsystem(x_pid, "x_pid")
    diagram.add_subsystem(rear_speed_meas, "rear_speed_meas")
    diagram.add_subsystem(speed_meas, "speed_meas")
    diagram.add_subsystem(x_pos_meas, "x_pos_meas")
    diagram.add_subsystem(ang_speed_meas, "ang_speed_meas")
    diagram.add_subsystem(sum_bloc, "sum_bloc")

    diagram.add_subsystem(trajectory, "trajectory")
    # diagram.add_subsystem(int_traj, "int_traj")

    diagram.add_subsystem(speed_const, "speed_const")

    diagram.connect("speed_const", "ref", "v_pid", "r")
    # diagram.connect("trajectory", "x_targ", "int_traj", "goal_x")
    # diagram.connect("trajectory", "x_targ", "x_pid", "r")
    # diagram.connect("int_traj", "x_targ", "x_pid", "r")

    # diagram.connect("x_pos_meas", "meas", "x_pid", "y")
    # diagram.connect("int_traj", "x_targ", "v_pid", "r")

    # diagram.connect("x_pid", "u", "v_pid", "r")

    diagram.connect("speed_meas", "meas", "v_pid", "y")
    diagram.connect("v_pid", "u", "acc_to_force", "acc_targ")

    # diagram.connect("vehicle", "y", "x_pos_meas", "y")
    diagram.connect("vehicle", "y", "speed_meas", "y")
    diagram.connect("vehicle", "y", "ang_speed_meas", "y")
    diagram.connect("vehicle", "y", "rear_speed_meas", "y")

    diagram.connect("steering", "ref", "r_pid", "r")
    diagram.connect("steering", "ref", "r_to_steering", "r_targ")

    diagram.connect("speed_meas", "meas", "r_to_steering", "vx_meas")
    diagram.connect("ang_speed_meas", "meas", "r_pid", "y")

    diagram.connect("r_pid", "u", "sum_bloc", "1")
    diagram.connect("r_to_steering", "delta", "sum_bloc", "2")

    diagram.connect("sum_bloc", "result", "vehicle", "delta")
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")
    diagram.connect("rear_speed_meas", "meas", "thr_map", "w_rear")

    return diagram


def main():
    vx = VX_REF

    vehicle = create_vehicle(vx=VX_REF)

    diagram = create_diagram(vehicle, vx_ref=vx)

    diagram.plot_diagram()

    print("Starting trajectory computation...")

    diagram.compute_trajectory(
        tf=SIMULATION_TIME,
        dt=0.02,
        show=False,
        verbose=False,
    )

    print("Trajectory computation done.")

    # diagram.plot_trajectory(
    #     signals=("x", "u"),
    #     backend="matplotlib",
    # )

    # traj = diagram.reconstruct_internal_signals(diagram.traj)
    # pid_logs = traj.get_signal("x_pid:logs")

    # ref = pid_logs[0, :]
    # meas = pid_logs[1, :]

    # t = traj.t

    # plt.figure()
    # plt.plot(t, ref, label="Goal position")
    # plt.plot(t, meas, label="Measured position")
    # plt.xlabel("Time [s]")
    # plt.ylabel("Pos [m/s]")
    # plt.title("Position PID - Reference vs Measured")
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("v_pid:logs")

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    t = traj.t

    plt.figure()
    plt.plot(t, ref, label="Goal speed")
    plt.plot(t, meas, label="Measured speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("speed PID - Reference vs Measured")
    plt.legend()
    plt.grid(True)
    plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
