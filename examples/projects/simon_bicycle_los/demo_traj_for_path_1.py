"""Simon bicycle LOS cascade on the current minilink API.

Run from repo root:

    PYTHONPATH=. python examples/projects/simon_bicycle_los/demo_traj_for_path_1.py

For a non-interactive smoke test:

    MPLBACKEND=Agg PYTHONPATH=. python examples/projects/simon_bicycle_los/demo_traj_for_path_1.py --no-show --no-animate
"""

import argparse
import math

import matplotlib.pyplot as plt
import numpy as np

from examples.projects.simon_bicycle_los.engine_bicycle import (
    DynamicBicycleRearWheelDriveEngine,
)
from examples.projects.simon_bicycle_los.los import Los
from examples.projects.simon_bicycle_los.maps import (
    AccToThr,
    AngularSpeedToSteeringMap,
)
from examples.projects.simon_bicycle_los.measurement import BicycleMeasurement
from examples.projects.simon_bicycle_los.path_lines import Lines
from examples.projects.simon_bicycle_los.path_segments import (
    make_rectangle_path,
    make_rounded_rectangle_from_path,
)
from examples.projects.simon_bicycle_los.servo import AngleWrappedServo, Servo
from examples.projects.simon_bicycle_los.vehicle_helper import (
    attach_vehicle_centered_diagram_camera,
    create_vehicle,
)
from minilink.blocks.sources import Source
from minilink.core.diagram import DiagramSystem

path_raw = make_rectangle_path(Lx=40.0, Ly=20.0)
path = make_rounded_rectangle_from_path(
    path_raw,
    R=7.0,
    nseg=2,
    narc=4,
    min_ds=0.1,
    closed=True,
)


def constant_source(value: float, name: str) -> Source:
    source = Source(1)
    source.name = name
    source.params["value"] = np.array([value], dtype=float)
    return source


def create_diagram(vehicle: DynamicBicycleRearWheelDriveEngine, vx_ref=1.0):
    path_raw_lines = Lines(pts=path_raw, name="Raw path", color="gray", linewidth=1)
    los_path = Lines(pts=path, name="LOS path", color="salmon")

    los_system = Los(
        path_pts=path,
        Delta=8.0,
        omega_n=1.2,
        control_point_ahead=vehicle.a + 0.5,
        closed=True,
    )

    velocity_ref = constant_source(vx_ref, name="Constant speed")

    yawrate_to_steering = AngularSpeedToSteeringMap(
        max_steer=vehicle.max_steer,
        min_steer=vehicle.min_steer,
        length_vehicle=vehicle.L,
    )

    measurement = BicycleMeasurement(name="Measured states", y_size=10)

    heading_servo = AngleWrappedServo(
        Kp=5.0,
        Ki=0.0,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-1.0,
        i_max=1.0,
        name="Heading servo",
    )

    yawrate_servo = Servo(
        Kp=0.3,
        Ki=0.0,
        Kd=0.1,
        cmd_min=-np.pi / 4.0,
        cmd_max=np.pi / 4.0,
        i_min=-np.pi / 4.0,
        i_max=np.pi / 4.0,
        name="Yaw-rate servo",
    )

    acc_to_thr = AccToThr(
        r_r=vehicle.r_r,
        engine_power_peak=vehicle.engine_power_peak,
        mass=vehicle.mass,
    )

    velocity_servo = Servo(
        Kp=0.8,
        Ki=0.01,
        Kd=0.0,
        cmd_min=-10.0,
        cmd_max=10.0,
        i_min=-0.0,
        i_max=5.0,
        name="Velocity servo",
    )

    diagram = DiagramSystem()
    diagram.name = "Cascade Servo - DynamicBicycleRearWheelDriveEngine"

    diagram.add_subsystem(yawrate_to_steering, "yawrate_to_steering")
    diagram.add_subsystem(vehicle, "vehicle")
    diagram.add_subsystem(yawrate_servo, "yawrate_servo")
    diagram.add_subsystem(heading_servo, "heading_servo")
    diagram.add_subsystem(acc_to_thr, "acc_to_thr")
    diagram.add_subsystem(velocity_servo, "velocity_servo")
    diagram.add_subsystem(measurement, "measurement")
    diagram.add_subsystem(path_raw_lines, "path_raw")
    diagram.add_subsystem(los_path, "los_path")
    diagram.add_subsystem(los_system, "los_system")
    diagram.add_subsystem(velocity_ref, "velocity_ref")

    diagram.connect("velocity_ref", "y", "velocity_servo", "ref")

    diagram.connect("los_system", "theta", "heading_servo", "ref")

    diagram.connect("heading_servo", "cmd", "yawrate_servo", "ref")
    diagram.connect("heading_servo", "cmd", "yawrate_to_steering", "r_targ")

    diagram.connect("vehicle", "y", "measurement", "y")
    diagram.connect("measurement", "u_meas", "yawrate_to_steering", "vx_meas")
    diagram.connect("measurement", "r_meas", "yawrate_servo", "meas")
    diagram.connect("measurement", "u_meas", "velocity_servo", "meas")
    diagram.connect("measurement", "theta_meas", "heading_servo", "meas")
    diagram.connect("measurement", "theta_meas", "los_system", "psi")
    diagram.connect("measurement", "y_meas", "los_system", "y")
    diagram.connect("measurement", "x_meas", "los_system", "x")

    diagram.connect("yawrate_to_steering", "delta", "yawrate_servo", "feedforward")
    diagram.connect("yawrate_servo", "cmd", "vehicle", "delta")

    diagram.connect("velocity_servo", "cmd", "acc_to_thr", "acc_targ")
    diagram.connect("acc_to_thr", "thr", "vehicle", "thr")
    diagram.connect("measurement", "w_r_meas", "acc_to_thr", "w_motor")

    attach_vehicle_centered_diagram_camera(diagram, vehicle)
    return diagram


def plot_path_preview(vehicle, show=True):
    x0 = float(vehicle.x0[0])
    y0 = float(vehicle.x0[1])
    theta0 = float(vehicle.x0[2])

    los_system = Los(
        path_pts=path,
        Delta=8.0,
        omega_n=1.2,
        control_point_ahead=vehicle.a + 0.5,
        closed=True,
    )
    _, info = los_system.controller.compute(x0, y0, theta0)

    plt.figure()
    plt.plot(
        path_raw[:, 0],
        path_raw[:, 1],
        "-o",
        color="gray",
        linewidth=2,
        label="Raw path",
    )
    plt.plot(
        path[:, 0],
        path[:, 1],
        "-o",
        color="salmon",
        linewidth=1,
        label="LOS path",
    )
    plt.plot(
        info["ax"],
        info["ay"],
        "-o",
        color="red",
        linewidth=1,
        label="LOS first control point",
    )

    arrow_length = vehicle.L * 2
    dx = arrow_length * math.cos(theta0)
    dy = arrow_length * math.sin(theta0)
    plt.arrow(
        x0,
        y0,
        dx,
        dy,
        head_width=0.6,
        head_length=1.0,
        fc="blue",
        ec="blue",
        linewidth=3,
        length_includes_head=True,
        label="Vehicle initial pose",
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Raw path vs LOS path")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    if show:
        plt.show()


def plot_control_point_trajectory(diagram, vehicle, dt, show=True):
    x0 = float(vehicle.x0[0])
    y0 = float(vehicle.x0[1])
    theta0 = float(vehicle.x0[2])

    try:
        i0, i1 = diagram.state_index["vehicle"]
        xv = diagram.traj.x[i0:i1, :]
        px_t = np.asarray(xv[0, :]).flatten()
        py_t = np.asarray(xv[1, :]).flatten()
        psi_t = np.asarray(xv[2, :]).flatten()
    except Exception:
        px_t = np.array([x0])
        py_t = np.array([y0])
        psi_t = np.array([theta0])

    los_system = Los(
        path_pts=path,
        Delta=8.0,
        omega_n=1.2,
        control_point_ahead=vehicle.a + 0.5,
        closed=True,
    )

    x_ctrls = []
    y_ctrls = []
    for k in range(px_t.shape[0]):
        _px = float(px_t[k])
        _py = float(py_t[k])
        _psi = float(psi_t[k])
        _, info = los_system.controller.compute(_px, _py, _psi)
        x_ctrls.append(info["x_ctrl"])
        y_ctrls.append(info["y_ctrl"])

    x_ctrls = np.array(x_ctrls)
    y_ctrls = np.array(y_ctrls)

    try:
        t_ctrl = diagram.traj.t
        if t_ctrl.shape[0] != x_ctrls.shape[0]:
            t_ctrl = np.linspace(0.0, float(t_ctrl[-1]), num=x_ctrls.shape[0])
    except Exception:
        t_ctrl = np.arange(0, x_ctrls.shape[0]) * dt

    plt.figure()
    plt.plot(
        path[:, 0], path[:, 1], "-o", color="salmon", linewidth=1, label="LOS path"
    )
    plt.plot(
        x_ctrls,
        y_ctrls,
        "-.",
        color="orange",
        linewidth=1.2,
        label="LOS control point over time",
    )
    if x_ctrls.size > 0:
        plt.scatter(
            x_ctrls[0], y_ctrls[0], color="green", s=36, label="start ctrl point"
        )
        plt.scatter(x_ctrls[-1], y_ctrls[-1], color="red", s=36, label="end ctrl point")

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("LOS V1 control point trajectory over time")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    if show:
        plt.show()

    plt.figure()
    plt.plot(t_ctrl, x_ctrls, label="x_ctrl (m)", color="tab:blue")
    plt.plot(t_ctrl, y_ctrls, label="y_ctrl (m)", color="tab:orange")
    plt.xlabel("Time [s]")
    plt.ylabel("Position [m]")
    plt.title("LOS control point: x and y vs time")
    plt.legend()
    plt.grid(True)
    if show:
        plt.show()


def main(tf=20.0, dt=0.01, show=True, animate=True, plot_diagram=True):
    vx = 10.0
    vehicle = create_vehicle(Y=0.0, vx=0.0, theta=np.pi, tire_slip_mode=None)
    diagram = create_diagram(vehicle, vx_ref=vx)

    if plot_diagram and show:
        diagram.plot_diagram()

    diagram.compute_trajectory(tf=tf, dt=dt, solver="euler")
    plot_path_preview(vehicle, show=show)
    plot_control_point_trajectory(diagram, vehicle, dt=dt, show=show)

    if animate:
        diagram.animate(renderer="matplotlib")

    return diagram


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tf", type=float, default=20.0)
    parser.add_argument("--dt", type=float, default=0.01)
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--no-animate", action="store_true")
    parser.add_argument("--no-diagram", action="store_true")
    return parser.parse_args()


if __name__ == "__main__":
    args = _parse_args()
    main(
        tf=args.tf,
        dt=args.dt,
        show=not args.no_show,
        animate=not args.no_animate,
        plot_diagram=not args.no_diagram,
    )
