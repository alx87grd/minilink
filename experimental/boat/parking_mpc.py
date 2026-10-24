"""Boat parking by model predictive control: receding-horizon trajectory optimization on the parking cost."""

from parking import CURRENT, X_START, ParkingBoat, parking_cost

from minilink import PlanningProblem, TrajectoryOptimizationPlanner
from minilink.graphical.catalog.boat_skin import boat_skin_3d
from minilink.planning.trajectory_optimization.mpc import (
    ModelPredictiveController,
    mpc_animation_overlays,
)

TF_SIM = 60.0  # [s]
SIM_DT = 0.05  # [s] inner plant step
MPC_DT = 0.5  # [s] between two solves
MPC_HORIZON = 15.0  # [s] a boat this heavy needs the dock in sight to slow down for it
MPC_STEPS = 15  # knots, 1 s apart
MPC_TERMINAL = 10.0  # weight of the state cost at the end of the horizon
SLSQP_MAXITER = 40
SLSQP_FTOL = 1e-3
N_FRAMES = 300

boat = ParkingBoat(CURRENT)

planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        sys=boat,
        x_start=X_START,
        cost=parking_cost(boat, terminal=MPC_TERMINAL),
        tf=MPC_HORIZON,
        X=boat.state.box,
        U=boat.input_box,
    ),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    record_solve_time=True,
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": SLSQP_MAXITER, "ftol": SLSQP_FTOL},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True, verbose=True)

loop = mpc @ boat
result = loop.compute_trajectory(
    tf=TF_SIM, x0_plant=X_START, plant_dt_inner=SIM_DT, compile_backend="jax"
)
loop.plot_trajectory()
loop.animate(
    result.plant.resample(n_samples=N_FRAMES),
    overlays=mpc_animation_overlays(result, planner),
)

boat.skin = boat_skin_3d  # the hull on the water, in the browser
loop.animate(
    result.plant.resample(n_samples=N_FRAMES),
    overlays=mpc_animation_overlays(result, planner),
    renderer="meshcat",
    native=False,
)
