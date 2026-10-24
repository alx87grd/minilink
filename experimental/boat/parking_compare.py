"""Boat parking: the cascade law, MPC and the learned law from one start, in calm water and in the current."""

import matplotlib.pyplot as plt
import numpy as np
from parking import (
    CURRENT,
    POLICY_HIDDEN,
    POLICY_WEIGHTS,
    X_DOCK,
    X_START,
    CascadeParkingController,
    ParkingBoat,
    dock_features,
    parking_cost,
    wrap_angle,
)

from minilink import (
    NeuralPolicyController,
    PlanningProblem,
    Trajectory,
    TrajectoryOptimizationPlanner,
)
from minilink.planning import evaluate_trajectory
from minilink.planning.trajectory_optimization.mpc import ModelPredictiveController

TF = 60.0  # [s]
DT = 0.05  # [s] plant step
MPC_DT, MPC_HORIZON, MPC_STEPS, MPC_TERMINAL = 0.5, 15.0, 15, 10.0
WATERS = {"calm water": np.zeros(2), "current": CURRENT}

runs = {}
for water, current in WATERS.items():
    boat = ParkingBoat(current)
    problem = PlanningProblem(
        sys=boat,
        x_start=X_START,
        cost=parking_cost(boat),
        tf=TF,
        X=boat.state.box,
        U=boat.input_box,
    )

    # The two state-feedback laws: the cascade, and PPO's weights from parking_ppo.py
    cascade = CascadeParkingController(boat)
    ppo = NeuralPolicyController(boat, features=dock_features, hidden=POLICY_HIDDEN)
    ppo.params["mlp"] = dict(np.load(POLICY_WEIGHTS))
    for name, ctl in (("Cascade", cascade), ("PPO", ppo)):
        loop = ctl @ boat
        traj = loop.reconstruct_internal_signals(loop.compute_trajectory(tf=TF, dt=DT))
        runs[name, water] = Trajectory(traj.t, traj.x, traj.get_signal("controller:u"))

    # MPC: the parking cost over a receding horizon; its program carries this water
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
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 40, "ftol": 1e-3},
    )
    mpc = ModelPredictiveController(planner, dt_mpc=MPC_DT, warm_start=True)
    result = (mpc @ boat).compute_trajectory(
        tf=TF, x0_plant=X_START, plant_dt_inner=DT, compile_backend="jax"
    )
    runs["MPC", water] = result.plant

    for name in ("Cascade", "MPC", "PPO"):
        runs[name, water] = (
            runs[name, water],
            float(evaluate_trajectory(problem, runs[name, water]).J[0]),
        )

# --- the comparison: paths over the water, distance and heading to the dock, thrust ---
fig, ax = plt.subplot_mosaic(
    [[f"path {w}", f"distance {w}", f"heading {w}", f"thrust {w}"] for w in WATERS],
    figsize=(17, 8),
    constrained_layout=True,
)
for water, current in WATERS.items():
    for name in ("Cascade", "MPC", "PPO"):
        run, J = runs[name, water]
        e = run.x[:3] - X_DOCK[:3, None]
        (line,) = ax[f"path {water}"].plot(
            run.x[0], run.x[1], label=f"{name}: J = {J:.0f}"
        )
        ax[f"distance {water}"].plot(
            run.t, np.hypot(e[0], e[1]), color=line.get_color()
        )
        ax[f"heading {water}"].plot(run.t, wrap_angle(e[2], np), color=line.get_color())
        ax[f"thrust {water}"].plot(
            run.t, run.u[0], color=line.get_color(), label=f"{name} surge"
        )
        ax[f"thrust {water}"].plot(
            run.t, run.u[1], "--", color=line.get_color(), label=f"{name} sway"
        )
    path = ax[f"path {water}"]
    bow = 4.0 * np.array([np.cos(X_DOCK[2]), np.sin(X_DOCK[2])])
    path.annotate("", X_DOCK[:2] + bow, X_DOCK[:2], arrowprops=dict(color="k", width=2))
    path.plot(*X_DOCK[:2], "k*", markersize=12, label="dock")
    path.plot(*X_START[:2], "ko", label="start")
    if current.any():
        corner = np.array([-28.0, 4.0])
        path.annotate(
            "",
            corner + 5.0 * current,
            corner,
            arrowprops=dict(color="tab:green", width=2),
        )
        path.text(*corner, " current", color="tab:green", va="bottom")
    path.set(title=water, xlabel="x [m]", ylabel="y [m]", aspect="equal")
    path.legend(loc="lower right", fontsize="small")
    ax[f"distance {water}"].set(ylabel="distance to dock [m]")
    ax[f"heading {water}"].set(ylabel="heading error [rad]")
    ax[f"thrust {water}"].set(ylabel="thrust [-1, 1]", xlabel="time [s]")
    ax[f"thrust {water}"].legend(ncol=3, fontsize="x-small")
    for key in ("distance", "heading", "thrust"):
        ax[f"{key} {water}"].grid(True)
plt.show()
