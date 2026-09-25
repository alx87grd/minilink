"""Model predictive control round a closed circuit, past two cones on the far straight."""

import numpy as np

from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
from minilink.catalog import UdeSRacecarDyn
from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    bind,
    car_outline,
    circuit_waypoints,
    from_waypoints,
    point_probe,
    quadratic_excess,
    quadratic_hinge,
)

LENGTH, WIDTH, RADIUS = 12.0, 8.0, 2.0  # [m] the circuit: a rounded rectangle, 36.6 m
V_TARGET = 4.0  # [m/s] asked for everywhere; the corners are what take it back
P_CRUISE = 15.0  # [W] about what holds V_TARGET on a straight
TF_SIM = 11.0  # [s] a little more than one lap
SIM_DT = 0.002  # [s] inner plant step
N_FRAMES = 111  # animation frames; the GIF is written at 250 dpi, so keep it short

MPC_DT = 0.1  # [s] between two solves
MPC_HORIZON = 1.2  # [s] ~4.5 m ahead at V_TARGET: one corner's worth
MPC_STEPS = 12  # knots, 0.1 s apart: a cone must not fall between two of them
SLSQP_MAXITER = 60
SLSQP_FTOL = 1.0
# 110 warm-started solves at ~0.15 s each; the whole script takes about half a minute.

BODY_LENGTH, BODY_WIDTH, BODY_MARGIN = 0.34, 0.20, 0.02  # [m] footprint the plan keeps
CORRIDOR_HALF_WIDTH = 0.6  # [m] the lane the car has to keep its body inside
PATH_WEIGHT = 60.0  # [1/m^2] on the distance from the centre line
PATH_DEAD_BAND = 0.05  # [m] deviation the path term ignores
CORRIDOR_WEIGHT = 200.0  # [1/m^2] on the depth by which the body leaves the lane
CONE_RADIUS = 0.10  # [m]
CONE_MARGIN = 0.15  # [m] keep-out painted round each cone
CONE_X = (2.0, -2.0)  # [m] where the two cones stand along the far straight
CONE_LEAN = (-0.13, 0.13)  # [m] how far each leans off the line, one way then the other
CONE_CLEARANCE = 0.30  # [m] clearance the plan is charged for losing
CONE_WEIGHT = 400.0  # [1/m^2]
PLANNER_MU = 1.1  # [-] the plan believes in a little more grip than the floor gives

LATERAL_START = 0.15  # [m] the car sets off beside the line, and the plan pulls it in
HEADING_NUDGE = 1.0e-4  # [rad] off the axis of the straight: see the start, below

path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS)
track = ReferenceTrack(from_waypoints(path), half_width=CORRIDOR_HALF_WIDTH)
# Two cones on the far straight, each leaning just far enough over the line that the
# car has to pick a side, and leaning opposite ways so it has to cross between them.
cones = [(x, 0.5 * WIDTH + lean) for x, lean in zip(CONE_X, CONE_LEAN)]
scene = Scene(obstacles=[Sphere(c, CONE_RADIUS + CONE_MARGIN) for c in cones])

# The model the plan runs on: the same car with grip to spare. Nothing else changes, so
# what the plan gets wrong at the limit is the plant's own behaviour, not other physics.
design = UdeSRacecarDyn(named_ports=False)
design.params["mu"] = PLANNER_MU
design.state.lower_bound[6] = 0.0
design.state.upper_bound[6] = 3.0 * V_TARGET / design.params["r_r"]
design.state.lower_bound[7] = -design.params["delta_max"]
design.state.upper_bound[7] = design.params["delta_max"]
design.state.lower_bound[8] = -design.params["P_max"]
design.state.upper_bound[8] = design.params["P_max"]

# What the car is asked for: hold the target speed, keep its centre on the line, keep
# its whole footprint inside the lane and clear of the cones. The quadratic part says
# nothing about where the car is on the map — the three spatial fields do all of that.
centre = bind(design, point_probe())
body = bind(
    design, car_outline(length=BODY_LENGTH, width=BODY_WIDTH, margin=BODY_MARGIN)
)
x_cruise = np.zeros(design.n)
x_cruise[3] = V_TARGET
x_cruise[6] = V_TARGET / design.params["r_r"]
x_cruise[8] = P_CRUISE
cost = (
    QuadraticCost.from_system(
        design,
        Q=np.diag([0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 2.0, 0.0]),
        R=np.diag([0.0005, 60.0]),
        S=np.diag([0.0, 0.0, 0.0, 8.0, 2.0, 0.0, 0.0, 2.0, 0.0]),
        xbar=x_cruise,
        ubar=np.array([P_CRUISE, 0.0]),
    )
    + track.distance_field(centre).as_cost(
        weight=PATH_WEIGHT, shaping=quadratic_excess(threshold=PATH_DEAD_BAND)
    )
    + track.corridor_field(body).as_cost(
        weight=CORRIDOR_WEIGHT, shaping=quadratic_hinge(threshold=0.0)
    )
    + scene.clearance_field(body).as_cost(
        weight=CONE_WEIGHT, shaping=quadratic_hinge(threshold=CONE_CLEARANCE)
    )
)

# Set off halfway along the near straight, beside the line and a hair off the axis of
# the straight: the distance to a polyline has no derivative where it is zero, so a
# probe sitting exactly on a segment would hand the optimizer a NaN gradient.
first = int(np.argmin(np.abs(path[:, 0]) + np.abs(path[:, 1] + 0.5 * WIDTH)))
step = path[(first + 1) % path.shape[0]] - path[first]
heading = np.arctan2(step[1], step[0]) + HEADING_NUDGE
start = path[first] + LATERAL_START * np.array([-np.sin(heading), np.cos(heading)])
x0 = np.array(
    [
        start[0],
        start[1],
        heading,
        V_TARGET,
        0.0,
        0.0,
        V_TARGET / design.params["r_r"],
        0.0,
        P_CRUISE,
    ]
)

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        sys=design, x_start=x0, cost=cost, tf=MPC_HORIZON, X=design.state.box
    ),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": SLSQP_MAXITER, "ftol": SLSQP_FTOL},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, verbose=True)

car = UdeSRacecarDyn(named_ports=False)
car.x0 = x0.copy()  # the whole circuit, not the car
car.camera_scale = 2.0

lap = mpc @ car
lap.plot_diagram()
result = lap.compute_trajectory(
    tf=TF_SIM, x0_plant=x0, plant_dt_inner=SIM_DT, compile_backend="jax"
)
lap.plot_trajectory()

lap.animate(
    result.plant.resample(n_samples=N_FRAMES),
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track),
)


car.skin = racecar_skin_3d
lap.animate(
    result.plant.resample(n_samples=N_FRAMES),
    overlays=mpc_animation_overlays(result, planner, scene=scene, track=track),
    renderer="meshcat",
)
