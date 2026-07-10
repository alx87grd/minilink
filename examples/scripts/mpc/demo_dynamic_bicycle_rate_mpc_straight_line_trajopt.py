"""Legacy straight-line MPC — per-step ``TrajectoryOptimizationPlanner`` recompile.

Reference for the pre-compile-once workflow: each MPC tick builds a new
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationPlanner`
and re-JIT-compiles the NLP. For production-style closed loop use
``demo_dynamic_bicycle_rate_mpc_straight_line.py`` (:class:`~minilink.planning.mpc.planner.MPCPlanner`).

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line_trajopt.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.catalog import SceneHistory
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

# --- Task ---
# Target surge speed and closed-loop simulation horizon.
U_TARGET = 4.0
TF_SIM = 5.0

# --- MPC ---
# MPC_HZ: trajopt re-solve rate; SIM_HZ: plant integration rate between solves.
# MPC_HORIZON / MPC_STEPS: receding-horizon direct collocation grid.
# SUBSTEPS: RK4 steps per MPC tick at held input (MPC_DT / SIM_DT).
MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 150
MPC_FTOL = 1e-2
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

# --- Constraints ---
# w_rear and delta are states; w_rear_dot and delta_dot are rate inputs.
W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

# --- Animation ---
REF_X_PAD = 20.0
CAMERA_SCALE = 12.0

configure_jax(enable_x64=True)

# --- Model ---
# sys_mpc: dynamics inside direct collocation. sys_sim: closed-loop plant rollout.
sys_mpc = JaxDynamicBicycleRateInputs()
sys_sim = JaxDynamicBicycleRateInputs()

# Slight model mismatch: planner uses nominal mass/inertia; plant is a bit heavier.
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

for sys in (sys_mpc, sys_sim):
    sys.state.lower_bound[6] = 0.0
    sys.state.upper_bound[6] = W_REAR_MAX
    sys.state.lower_bound[7] = -DELTA_MAX
    sys.state.upper_bound[7] = DELTA_MAX
    sys.inputs["w_rear_dot"].lower_bound[0] = -W_REAR_DOT_MAX
    sys.inputs["w_rear_dot"].upper_bound[0] = W_REAR_DOT_MAX
    sys.inputs["delta_dot"].lower_bound[0] = -DELTA_DOT_MAX
    sys.inputs["delta_dot"].upper_bound[0] = DELTA_DOT_MAX


# --- Cost ---
# Track straight line x_ref; penalize rate inputs toward zero at steady cruise.
r_r = sys_mpc.params["r_r"]
w_rear_ref = U_TARGET / r_r
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
ubar = np.array([0.0, 0.0])
cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    R=np.diag([1.0, 25.0]),
    S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
    xbar=x_ref,
    ubar=ubar,
)

# --- Initial condition and plant evaluator ---
x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0, (U_TARGET * 0.8) / r_r, 0.0])
sim_evaluator = sys_sim.compile(backend="jax", verbose=False)

# --- Closed-loop buffers ---
t_hist = [0.0]
x_hist = [x0.copy()]
u_hist = [np.zeros(sys_sim.m)]
mpc_plans = []
x = x0.copy()
t = 0.0
u_hold = np.zeros(sys_sim.m)
prev_plan = None
next_mpc_t = 0.0

print("MPC straight-line tracking (rate inputs)")
print(f"  mpc_hz={MPC_HZ}, sim_hz={SIM_HZ}, horizon={MPC_HORIZON}s")

# --- MPC closed-loop simulation ---
while t < TF_SIM - 1e-12:
    # Re-solve direct collocation at MPC rate; warm-start from shifted prev plan.
    if t >= next_mpc_t - 1e-12:
        problem = PlanningProblem(sys=sys_mpc, x_start=x, cost=cost)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                solve_disp=False,
                record_solve_time=True,
                optimizer_method="scipy_slsqp",
                optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
            ),
        )

        # Shift previous horizon forward by one MPC tick; pin x[:, 0] to measured x.
        guess = None
        if prev_plan is not None and prev_plan.n_samples >= 3:
            t_shift = prev_plan.t + MPC_DT
            mask = t_shift <= t + MPC_HORIZON + 1e-9
            if np.count_nonzero(mask) >= 3:
                x_guess = prev_plan.x[:, mask].copy()
                x_guess[:, 0] = x
                guess = Trajectory(
                    t=t_shift[mask] - t,
                    x=x_guess,
                    u=prev_plan.u[:, mask],
                )

        plan = planner.compute_solution(initial_guess=guess)
        res = planner.last_optimization_result
        print(f"MPC @ t={t:.2f}s  success={res.success}  solve={res.solve_time_s:.3f}s")
        prev_plan = plan
        u_hold = plan.u[:, 0].copy()
        mpc_plans.append(
            (t, Trajectory(t=plan.t + t, x=plan.x.copy(), u=plan.u.copy()))
        )
        next_mpc_t += MPC_DT

    # ZOH plant step: hold first planned rate input and integrate at SIM_HZ.
    for _ in range(SUBSTEPS):
        if t >= TF_SIM:
            break
        x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
        t += SIM_DT
        t_hist.append(t)
        x_hist.append(x.copy())
        u_hist.append(u_hold.copy())

traj = Trajectory(
    t=np.asarray(t_hist),
    x=np.asarray(x_hist).T,
    u=np.asarray(u_hist).T,
)


# --- Animation ---
x0_ref = float(traj.x[0, 0]) - REF_X_PAD
x1_ref = float(traj.x[0, -1]) + REF_X_PAD
history = SceneHistory(
    reference=CustomLine(
        np.array([[x0_ref, 0.0, 0.0], [x1_ref, 0.0, 0.0]]),
        color="k",
        linewidth=1.0,
        style="--",
    ),
    trail=TrajectoryPolyline(
        traj,
        window="prefix",
        color="b",
        style="--",
        linewidth=1.0,
    ),
    horizon=HorizonPolyline(
        mpc_plans,
        color="tab:orange",
        linewidth=2.0,
        style="--",
    ),
)

sys_sim.params = dict(sys_sim.params)
sys_sim.camera_scale = CAMERA_SCALE
sys_sim.traj = traj
sys_sim.plot_trajectory(signals=("x", "u"))
sys_sim.animate(traj, overlays=[history])
