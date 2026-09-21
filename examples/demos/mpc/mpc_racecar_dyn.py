"""Same kinematic MPC, now over a PID speed loop on the dynamic 1/10 racecar."""

import numpy as np

from minilink import (
    PID,
    Demux,
    DiagramSystem,
    PlanningProblem,
    QuadraticCost,
    TrajectoryOptimizationPlanner,
    UdeSRacecar,
    UdeSRacecarDyn3D,
)
from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.graphical.catalog.racecar_skin import racecar_skin_2d, racecar_skin_3d
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    bind,
    circuit_waypoints,
    from_waypoints,
    point_probe,
    quadratic_hinge,
)

MU = 0.5  # [-] floor grip; drop toward 0.4 and the rear saturates in the corners
V_REF = 2.0  # [m/s] the 1 m corners hold this at MU=1, not at MU=0.4
TF = 10.0
MPC_DT = 0.05
MPC_HORIZON = 1.8
KP, KI, KD, TAU = 40.0, 20.0, 5.0, 0.05  # PID on drive power
P_CRUISE = 10.0  # [W] initial throttle

# --- circuit: same rounded rectangle and cone as the kinematic demo ---
path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
scene = Scene(
    obstacles=[
        Sphere((0.0, 1.8), 0.15),
        Sphere((1.0, 2.0), 0.15),
        Sphere((2.0, 2.2), 0.15),
        Sphere((-2.2, 2.3), 0.35),
    ]
)

# --- design model: the kinematic bicycle the MPC plans on ---
design = UdeSRacecar()
design.inputs["u"].lower_bound = np.array([0.0, -0.52])
design.inputs["u"].upper_bound = np.array([5.0, 0.52])

start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])
x0 = np.array(
    [start[0] - 0.1 * np.sin(heading), start[1] + 0.1 * np.cos(heading), heading]
)
design.x0 = x0

# --- plant: 11-state car (rolling angles spin the wheels); mu is the grip ---
car = UdeSRacecarDyn3D()
car.params["mu"] = MU
car.camera_follow_frame = None
car.camera_scale = 4.0
car.x0 = np.array(
    [
        x0[0],
        x0[1],
        x0[2],
        V_REF,
        0.0,
        0.0,
        V_REF / car.params["r_r"],
        0.0,
        P_CRUISE,
        0.0,
        0.0,
    ]
)

# --- inner loop: MPC [v, delta] in; steer is direct, speed goes through a PID ---
inner = DiagramSystem()
inner.add_subsystem(Demux((1, 1)), "cmd")
inner.add_subsystem(
    PID(
        Kp=KP,
        Ki=KI,
        Kd=KD,
        tau=TAU,
        ports="reference",
        u_min=-car.params["P_max"],
        u_max=car.params["P_max"],
    ),
    "speed",
)
inner.add_subsystem(car, "car")
inner.add_subsystem(Demux((3, 8), port="y"), "pose")
inner.add_input_port("u", dim=2)
inner.connect("input", "u", "cmd", "u")
inner.connect("cmd", "u[0]", "speed", "r")
inner.connect("car", "speed", "speed", "y")
inner.connect("speed", "u", "car", "P_cmd")
inner.connect("cmd", "u[1]", "car", "delta_cmd")
inner.connect("car", "y", "pose", "y")
inner.connect_new_output_port("pose", "y[0:3]", "y")

inner.plot_diagram()

# --- cost: same three terms as the kinematic demo ---
probe = bind(design, point_probe())
quad = QuadraticCost.from_system(
    design, Q=np.zeros((3, 3)), R=np.diag([2.0, 1.0]), ubar=np.array([V_REF, 0.0])
)
corridor = track.corridor_field(probe).as_cost(weight=20.0, shaping=quadratic_hinge())
obstacle = scene.clearance_field(probe).as_cost(
    weight=40.0, shaping=quadratic_hinge(threshold=0.2)
)
cost = quad + corridor + obstacle

# --- MPC: receding-horizon collocation on the kinematic design ---
planner = TrajectoryOptimizationPlanner(
    PlanningProblem(sys=design, x_start=x0, cost=cost, tf=MPC_HORIZON),
    n_steps=20,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 40, "ftol": 0.05},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, verbose=True)

# --- closed loop: hybrid, because the inner plant is a diagram ---
computer = mpc.export_to_computer()
diagram = hybrid_closed_loop(
    computer.diagram,
    inner,
    schedule=computer.schedule,
    computer=computer,
    computer_out="u_ff",
    computer_in="y",
    plant_in="u",
    plant_out="y",
)

# lap.plot_diagram()
result = diagram.compute_trajectory(tf=TF, plant_dt_inner=0.002, compile_backend="jax")
# trail from the car pose: result.plant is the inner diagram, whose first
# states are the PID, not (x, y)
car_run = inner.trajectory_of(car, result.plant)
overlays = mpc_animation_overlays(
    result, planner, scene=scene, track=track, traj=car_run
)
# diagram.plot_trajectory(signals=("car:speed", "car:grip", "speed:u"))
car.skin = racecar_skin_2d
diagram.animate(overlays=overlays)

# --- 3-D look: same lap; the extra states are the wheel rolling angles ---
car.skin = racecar_skin_3d
diagram.animate(
    overlays=overlays,
    renderer="meshcat",
    is_3d=True,
)
