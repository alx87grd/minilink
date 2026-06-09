import matplotlib.pyplot as plt
import numpy as np

from minilink.control.pid import PID, Sum
from minilink.control.references import ConstantReference
from minilink.core.diagram import DiagramSystem
from minilink.core.system import System
from minilink.graphical.animation.primitives import (
    Point,
    pose2d_matrix,
)
from projects.bicycle.control.full_bicycle_meas import BicycleMeasurement
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


class IntermediateTrajectory(System):
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
        self.add_input_port(
            "goal_x",
            nominal_value=np.array([0.0]),
        )

        self.add_output_port(
            "x_targ", dim=1, function=self.x_targ, dependencies=(["goal_x"])
        )
        self.add_output_port("y_targ", dim=1, function=self.y_targ, dependencies=())

    def x_targ(self, x, u, t=0.0, params=None):
        t_f = 5.0
        delta = 1.0  # durée de transition

        tau = t / t_f

        # polynôme degré 5
        s_poly = t_f * (10 * tau**3 - 15 * tau**4 + 6 * tau**5)

        if t <= t_f:
            s = s_poly

        elif t <= t_f + delta:
            theta = (t - t_f) / delta
            alpha = 3 * theta**2 - 2 * theta**3  # transition douce

            s = (1 - alpha) * s_poly + alpha * t

        else:
            s = t
        return np.array([s], dtype=float)

    def y_targ(self, x, u, t=0.0, params=None):
        y_targ = 0.0

        if self.y_traj is not None:
            y_targ = self.y_traj(t)
        return np.array([y_targ], dtype=float)

    # def get_kinematic_geometry(self):
    #     # Only show a single point primitive; the renderer will position it
    #     # using the transform returned by get_kinematic_transforms.
    #     return [Circle(radius=0.5, center=[0.0, 0.0, 0.0], color="red", fill=False)]

    # def get_kinematic_transforms(self, x, u, t):
    #     # Compute the current target point position and return a single transform
    #     # for the Point primitive.
    #     y_t = 0.0
    #     x_targ = float(u[0])

    #     self.a = 1.0  # m/s^2 (v = at)

    #     t_f = 5.0

    #     a_2 = 3.0 / t_f**2
    #     a_3 = -2.0 / t_f**3

    #     self.a = 1.0  # m/s^2 (v = at)

    #     # vx_targ_int = self.a * t
    #     x_t = (a_2 * t**2 + 3 * a_3 * t**3) * VX_REF

    #     if t >= t_f:
    #         x_t = x_targ
    #     if self.y_traj is not None:
    #         y_val = self.y_traj(t)
    #         y_t = float(np.asarray(y_val).reshape(1)[0])
    #     return [pose2d_matrix(x=x_t, y=y_t, theta=0.0)]


class Vx2V(System):
    """Constant scalar reference signal."""

    def __init__(self, ref: float = 1.0, name: str | None = None):
        super().__init__(0)

        self.name = name if name is not None else "Not named :("

        self.add_input_port("theta", dim=1, nominal_value=np.array([0.0]))

        self.ref = float(ref)

        self.add_output_port(
            "ref",
            dim=1,
            function=self.h_ref,
            dependencies=["theta"],
        )

    def h_ref(self, x, u, t=0.0, params=None):
        theta = float(u[0])
        V = self.ref / np.cos(theta)
        return np.array([V], dtype=float)


def x_pos(t):
    return VX_REF * t


def y_pos(t):
    # ys = 1.0 * np.sin(np.sin(0.5 * np.pi * t / 1.0))
    return t


def sin_traj(t, wavelen=3.0, amp=2.0):
    return amp * np.sin(0.5 * np.pi * t / wavelen)


def create_diagram(
    vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=VX_REF, r_ref=R_REF
):

    r_to_steering = AngularSpeedToSteeringMap(vehicle)
    steering = ConstantReference(ref=r_ref, name="Constant angular speed")
    # speed_const = ConstantReference(ref=vx_ref, name="Constant linear speed")
    speed_goal = Vx2V(ref=vx_ref, name="Constant xspeed")

    full_state_meas = BicycleMeasurement(name="Meas states", y_size=10)

    # speed_meas = Measurement(name="Speed measurement", y_size=10, index=3, show=False)
    # x_pos_meas = Measurement(name="X pos measurement", y_size=10, index=0, show=False)

    # y_pos_meas = Measurement(name="y pos measurement", y_size=10, index=1, show=False)

    # ang_speed_meas = Measurement(
    #     name="Angular speed measurement", y_size=10, index=5, show=False
    # )

    # rear_speed_meas = Measurement(
    #     name="Rear wheel speed",
    #     y_size=10,
    #     index=6,
    # )

    x_pid = PID(
        Kp=2.0,
        Ki=0.0,
        Kd=1.0,
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

    theta_pid = PID(
        Kp=1.0,
        Ki=0.0,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-10.0,
        i_max=10.0,
        name="Yaw rate PID",
    )

    r_pid = PID(
        Kp=0.0,
        Ki=0.2,
        Kd=0.0,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        i_min=-np.pi / 4.0,
        i_max=np.pi / 4.0,
        name="Yaw rate PID",
    )

    sum_bloc = Sum(max=np.pi / 2.0, min=-np.pi / 2.0)

    trajectory = Trajectory(x_traj=x_pos, y_traj=sin_traj)
    int_traj = IntermediateTrajectory()

    diagram = DiagramSystem()
    diagram.name = "Cascade PID - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(steering, "steering")
    diagram.add_subsystem(r_to_steering, "r_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(r_pid, "r_pid")
    diagram.add_subsystem(theta_pid, "theta_pid")

    diagram.add_subsystem(acc_to_force, "acc_to_force")
    diagram.add_subsystem(thr_map, "thr_map")
    diagram.add_subsystem(v_pid, "v_pid")

    diagram.add_subsystem(full_state_meas, "full_state_meas")
    diagram.add_subsystem(sum_bloc, "sum_bloc")

    diagram.add_subsystem(trajectory, "trajectory")
    diagram.add_subsystem(speed_goal, "speed_goal")

    diagram.connect("speed_goal", "ref", "v_pid", "ref")
    diagram.connect("trajectory", "y_targ", "theta_pid", "ref")

    # diagram.connect("speed_meas", "meas", "v_pid", "meas")
    diagram.connect("full_state_meas", "vx_meas", "v_pid", "meas")

    diagram.connect("v_pid", "cmd", "acc_to_force", "acc_targ")

    diagram.connect("vehicle", "y", "full_state_meas", "y")
    diagram.connect("theta_pid", "cmd", "r_pid", "ref")
    diagram.connect("theta_pid", "cmd", "r_to_steering", "r_targ")

    diagram.connect("full_state_meas", "vx_meas", "r_to_steering", "vx_meas")
    diagram.connect("full_state_meas", "r_meas", "r_pid", "meas")

    diagram.connect("full_state_meas", "r_meas", "speed_goal", "theta")

    diagram.connect("full_state_meas", "y_meas", "theta_pid", "meas")

    diagram.connect("r_pid", "cmd", "sum_bloc", "1")
    diagram.connect("r_to_steering", "delta", "sum_bloc", "2")

    diagram.connect("sum_bloc", "result", "vehicle", "delta")
    diagram.connect("acc_to_force", "F_rear", "thr_map", "F_rear")
    diagram.connect("thr_map", "thr", "vehicle", "thr")
    diagram.connect("full_state_meas", "w_r_meas", "thr_map", "w_rear")

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

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("v_pid:logs")

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    t = traj.t

    plt.figure()
    plt.plot(t, ref, label="Goal Speed")
    plt.plot(t, meas, label="Measured Speed")
    plt.xlabel("Time [s]")
    plt.ylabel("Speed [m/s]")
    plt.title("Speed PID - Reference vs Measured")
    plt.legend()
    plt.grid(True)
    plt.show()

    traj = diagram.reconstruct_internal_signals(diagram.traj)
    pid_logs = traj.get_signal("theta_pid:logs")

    ref = pid_logs[0, :]
    meas = pid_logs[1, :]

    t = traj.t

    plt.figure()
    plt.plot(t, ref, label="y meas")
    plt.plot(t, meas, label="y goal")
    plt.xlabel("Time [s]")
    plt.ylabel("Y pas [m]")
    plt.title("Theta PIS - Reference vs Measured")
    plt.legend()
    plt.grid(True)
    plt.show()

    attach_vehicle_centered_diagram_camera(diagram, vehicle)

    diagram.animate(renderer="matplotlib")


if __name__ == "__main__":
    main()
