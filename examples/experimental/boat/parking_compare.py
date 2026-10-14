"""Boat parking in a current: the cascade law, MPC and the PPO law from one start, side by side."""

import matplotlib.pyplot as plt
import numpy as np
from parking import (
    CURRENT,
    X_DOCK,
    X_START,
    CascadeParkingController,
    ParkingBoat,
    dock_features,
    parking_cost,
    wrap_angle,
)

from minilink import (
    Gaussian,
    PlanningProblem,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Trajectory,
    TrajectoryOptimizationPlanner,
)
from minilink.control.mpc import ModelPredictiveController
from minilink.planning.evaluation import score_trajectory

TF = 60.0  # [s]
DT = 0.05  # [s] plant step
MPC_DT, MPC_HORIZON, MPC_STEPS, MPC_TERMINAL = 0.5, 15.0, 15, 10.0
PPO_DT, PPO_TIMESTEPS, PPO_GAMMA, INFEASIBLE_COST = 0.1, 15_000_000, 0.998, 2000.0

boat = ParkingBoat(CURRENT)
problem = PlanningProblem(
    sys=boat, x_start=X_START, cost=parking_cost(boat), tf=TF, X=boat.state.box
)
runs = {}

# The cascade law: a position loop over an LQR velocity loop
cascade = CascadeParkingController(boat)
loop = cascade @ boat
traj = loop.reconstruct_internal_signals(loop.compute_trajectory(tf=TF, dt=DT))
runs["Cascade"] = Trajectory(traj.t, traj.x, traj.get_signal("controller:u"))

# MPC: the parking cost over a receding horizon, replanned every MPC_DT
planner = TrajectoryOptimizationPlanner(
    PlanningProblem(
        sys=boat,
        x_start=X_START,
        cost=parking_cost(boat, terminal=MPC_TERMINAL),
        tf=MPC_HORIZON,
        X=boat.state.box,
    ),
    n_steps=MPC_STEPS,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 40, "ftol": 1e-3},
)
mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
result = (mpc @ boat).compute_trajectory(
    tf=TF, x0_plant=X_START, plant_dt_inner=DT, compile_backend="jax"
)
runs["MPC"] = result.plant

# PPO: the same cost as a stochastic problem over starts round the dock
stochastic = StochasticPlanningProblem(
    boat,
    cost=parking_cost(boat),
    tf=np.inf,
    x0_distribution=Gaussian(
        [-10.0, -5.0, X_DOCK[2], 0.0, 0.0, 0.0], [12.0, 12.0, 1.5, 0.5, 0.5, 0.2]
    ),
    X=boat.state.box,
    infeasible_cost=INFEASIBLE_COST,
)
rl = ReinforcementLearningPlanner(
    stochastic,
    dt=PPO_DT,
    features=dock_features,
    hidden=(64, 64),
    algorithm="ppo",
    n_envs=64,
    n_steps=128,
    batch_size=1024,
    learning_rate=3e-4,
    gamma=PPO_GAMMA,
    episode_length=TF,
    log_std_init=-0.5,
)
rl.solve(timesteps=PPO_TIMESTEPS)
loop = rl.get_controller() @ boat
traj = loop.reconstruct_internal_signals(loop.compute_trajectory(tf=TF, dt=DT))
runs["PPO"] = Trajectory(traj.t, traj.x, traj.get_signal("controller:u"))

# --- the comparison: paths over the water, distance and heading to the dock, thrust ---
fig, ax = plt.subplot_mosaic(
    [["path", "distance"], ["path", "heading"], ["path", "thrust"]],
    figsize=(13, 7),
    constrained_layout=True,
)
for name, run in runs.items():
    J, failed = score_trajectory(problem, run)
    e = run.x[:3] - X_DOCK[:3, None]
    (line,) = ax["path"].plot(run.x[0], run.x[1], label=f"{name}: J = {J:.0f}")
    ax["distance"].plot(run.t, np.hypot(e[0], e[1]), color=line.get_color())
    ax["heading"].plot(run.t, wrap_angle(e[2], np), color=line.get_color())
    ax["thrust"].plot(run.t, run.u[0], color=line.get_color(), label=f"{name} surge")
    ax["thrust"].plot(
        run.t, run.u[1], "--", color=line.get_color(), label=f"{name} sway"
    )
bow = 4.0 * np.array([np.cos(X_DOCK[2]), np.sin(X_DOCK[2])])
ax["path"].annotate(
    "", X_DOCK[:2] + bow, X_DOCK[:2], arrowprops=dict(color="k", width=2)
)
ax["path"].plot(*X_DOCK[:2], "k*", markersize=12, label="dock, bow into the current")
ax["path"].plot(*X_START[:2], "ko", label="start")
corner = np.array([-28.0, 4.0])
ax["path"].annotate(
    "", corner + 5.0 * CURRENT, corner, arrowprops=dict(color="tab:green", width=2)
)
ax["path"].text(*corner, " current", color="tab:green", va="bottom")
ax["path"].set(xlabel="x [m]", ylabel="y [m]", aspect="equal")
ax["path"].legend(loc="lower right")
ax["distance"].set(ylabel="distance to dock [m]")
ax["heading"].set(ylabel="heading error [rad]")
ax["thrust"].set(ylabel="thrust [-1, 1]", xlabel="time [s]")
ax["thrust"].legend(ncol=3, fontsize="small")
for key in ("distance", "heading", "thrust"):
    ax[key].grid(True)
plt.show()
