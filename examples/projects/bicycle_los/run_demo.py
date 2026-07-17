"""Bicycle LOS cascade path-following demo.

Run from repo root:

    PYTHONPATH=. python examples/projects/bicycle_los/run_demo.py
"""

import math

import matplotlib.pyplot as plt
import numpy as np

from examples.projects.bicycle_los.los import LOS
from examples.projects.bicycle_los.motor_map import (
    AccelerationToThrottle,
    YawRateToSteering,
)
from examples.projects.bicycle_los.path_generator import rounded_rectangle_path
from examples.projects.bicycle_los.servo import HeadingServo, Servo
from examples.projects.bicycle_los.vehicle import VehicleMeasurement, create_vehicle
from minilink.blocks.sources import Source
from minilink.core.diagram import DiagramSystem

path = rounded_rectangle_path(Lx=40.0, Ly=20.0, R=7.0, nseg=2, narc=4, closed=True)

vehicle = create_vehicle(Y=0.0, vx=0.0, theta=np.pi, tire_slip_mode=None)
vx_ref = 10.0

los = LOS(
    path_pts=path,
    lookahead=8.0,
    control_point_ahead=vehicle.a + 0.5,
    closed=True,
)

velocity_ref = Source(1)
velocity_ref.name = "Velocity reference"
velocity_ref.params["value"] = np.array([vx_ref], dtype=float)

yawrate_to_steering = YawRateToSteering(
    max_steer=vehicle.max_steer,
    min_steer=vehicle.min_steer,
    length_vehicle=vehicle.L,
)

measurement = VehicleMeasurement()

heading_servo = HeadingServo(
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

throttle_map = AccelerationToThrottle(
    r_r=vehicle.r_r,
    engine_power_peak=vehicle.engine_power_peak,
    mass=vehicle.mass,
    transmission_ratio=vehicle.transmission_ratio,
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
diagram.name = "Cascade servo — engine bicycle"

diagram.add_subsystem(yawrate_to_steering, "yawrate_to_steering")
diagram.add_subsystem(vehicle, "vehicle")
diagram.add_subsystem(yawrate_servo, "yawrate_servo")
diagram.add_subsystem(heading_servo, "heading_servo")
diagram.add_subsystem(throttle_map, "throttle_map")
diagram.add_subsystem(velocity_servo, "velocity_servo")
diagram.add_subsystem(measurement, "measurement")
diagram.add_subsystem(los, "los")
diagram.add_subsystem(velocity_ref, "velocity_ref")

diagram.connect("velocity_ref", "y", "velocity_servo", "ref")

diagram.connect("vehicle", "y", "los", "y")
diagram.connect("los", "heading_ref", "heading_servo", "ref")

diagram.connect("heading_servo", "cmd", "yawrate_servo", "ref")
diagram.connect("heading_servo", "cmd", "yawrate_to_steering", "yawrate_ref")

diagram.connect("vehicle", "y", "measurement", "y")
diagram.connect("measurement", "vx", "yawrate_to_steering", "vx")
diagram.connect("measurement", "yawrate", "yawrate_servo", "meas")
diagram.connect("measurement", "vx", "velocity_servo", "meas")
diagram.connect("measurement", "theta", "heading_servo", "meas")

diagram.connect("yawrate_to_steering", "delta", "yawrate_servo", "feedforward")
diagram.connect("yawrate_servo", "cmd", "vehicle", "delta")

diagram.connect("velocity_servo", "cmd", "throttle_map", "accel_ref")
diagram.connect("throttle_map", "throttle", "vehicle", "throttle")
diagram.connect("measurement", "w_rear", "throttle_map", "w_rear")

diagram.camera_follow_frame = "vehicle:body"
diagram.camera_scale = 15.0

diagram.plot_diagram()
diagram.compute_trajectory(tf=20.0, dt=0.01, solver="euler")

x0 = float(vehicle.x0[0])
y0 = float(vehicle.x0[1])
theta0 = float(vehicle.x0[2])
_, info = los.controller.compute(x0, y0, theta0)

plt.figure()
plt.plot(path[:, 0], path[:, 1], "-o", color="salmon", linewidth=1, label="Path")
plt.plot(info["ax"], info["ay"], "o", color="red", label="Lookahead")

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
    label="Initial pose",
)

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Path and initial LOS lookahead")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.show()

diagram.animate(renderer="matplotlib")
