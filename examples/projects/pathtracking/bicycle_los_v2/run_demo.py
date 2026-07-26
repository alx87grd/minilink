"""Bicycle LOS three-layer path-following demo (PathTracking → Servos → Allocation).

Run from repo root:

    PYTHONPATH=. python examples/projects/pathtracking/bicycle_los_v2/run_demo.py
"""

import math

import matplotlib.pyplot as plt
import numpy as np

from examples.projects.pathtracking.bicycle_los_v2.allocation import Allocation
from examples.projects.pathtracking.bicycle_los_v2.path_generator import (
    rounded_rectangle_path,
)
from examples.projects.pathtracking.bicycle_los_v2.path_tracking import PathTracking
from examples.projects.pathtracking.bicycle_los_v2.servos import Servos
from examples.projects.pathtracking.bicycle_los_v2.vehicle import create_vehicle
from minilink.core.diagram import DiagramSystem

path = rounded_rectangle_path(Lx=40.0, Ly=20.0, R=7.0, nseg=2, narc=4, closed=True)

vehicle = create_vehicle(Y=0.0, vx=0.0, theta=np.pi, tire_slip_mode=None)

path_tracking = PathTracking(
    path_pts=path,
    lookahead=8.0,
    control_point_ahead=vehicle.a + 0.5,
    closed=True,
    vx_ref=10.0,
)

servos = Servos(
    mass=vehicle.mass,
    length_vehicle=vehicle.L,
    max_steer=vehicle.max_steer,
    min_steer=vehicle.min_steer,
)

allocation = Allocation(
    r_r=vehicle.r_r,
    engine_power_peak=vehicle.engine_power_peak,
    transmission_ratio=vehicle.transmission_ratio,
)

diagram = DiagramSystem()
diagram.name = "PathTracking → Servos → Allocation"

diagram.add_subsystem(vehicle, "vehicle")
diagram.add_subsystem(path_tracking, "path_tracking")
diagram.add_subsystem(servos, "servos")
diagram.add_subsystem(allocation, "allocation")

diagram.connect("vehicle", "y", "path_tracking", "y")
diagram.connect("path_tracking", "heading_ref", "servos", "heading_ref")
diagram.connect("path_tracking", "vx_ref", "servos", "vx_ref")
diagram.connect("vehicle", "y", "servos", "y")

diagram.connect("servos", "F_rear", "allocation", "F_rear")
diagram.connect("servos", "delta", "allocation", "delta")
diagram.connect("vehicle", "y", "allocation", "y")

diagram.connect("allocation", "throttle", "vehicle", "throttle")
diagram.connect("allocation", "delta", "vehicle", "delta")

diagram.camera_follow_frame = "vehicle:body"
diagram.camera_scale = 15.0

diagram.plot_diagram()
diagram.compute_trajectory(tf=20.0, dt=0.01, solver="euler")

x0 = float(vehicle.x0[0])
y0 = float(vehicle.x0[1])
theta0 = float(vehicle.x0[2])
_, info = path_tracking.controller.compute(x0, y0, theta0)

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
