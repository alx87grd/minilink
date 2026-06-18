"""Receding-horizon MPC for straight-line tracking on the dynamic bicycle.

MPC re-solves direct collocation at ``MPC_HZ``; the plant integrates with RK4 at
``SIM_HZ`` between ticks (ZOH on the first planned input). Animation uses the
native minilink pipeline with reference, executed-trail, and MPC-plan overlays.

Run from repo root::

    python examples/scripts/trajectory_optimization/demo_dynamic_bicycle_mpc_straight_line.py
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycle,
    JaxDynamicBicycle,
)
from minilink.graphical.animation.animator import Animator
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
    time_channel_matrix,
)
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
# MPC_BACKEND: "jax" (JIT program evaluator) or "scipy_slsqp" (NumPy + SciPy SLSQP).
MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 150
MPC_FTOL = 1e-2
MPC_BACKEND = "jax"
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

# --- Constraints ---
# Input bounds applied to both sys_mpc and sys_sim.
W_REAR_MAX = 90.0
DELTA_MAX = 0.55

# --- Animation ---
REF_X_PAD = 20.0
CAMERA_SCALE = 12.0
TIME_FACTOR_VIDEO = 1.0
SAVE_ANIMATION = False

# --- Model ---
# sys_mpc: dynamics inside direct collocation. sys_sim: closed-loop plant rollout.
sys_mpc = JaxDynamicBicycle()
sys_sim = JaxDynamicBicycle()

# Slight model mismatch: planner uses nominal mass/inertia; plant is a bit heavier.
sys_sim.params["mass"] = 1.03 * sys_mpc.params["mass"]
sys_sim.params["inertia"] = 1.02 * sys_mpc.params["inertia"]

for sys in (sys_mpc, sys_sim):
    sys.inputs["w_rear"].lower_bound[0] = 0.0
    sys.inputs["w_rear"].upper_bound[0] = W_REAR_MAX
    sys.inputs["delta"].lower_bound[0] = -DELTA_MAX
    sys.inputs["delta"].upper_bound[0] = DELTA_MAX


# --- Cost ---
# Track straight line x_ref with quadratic state/input/terminal weights.
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0])
ubar = np.array([U_TARGET / sys_mpc.params["r_r"], 0.0])
cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0]),
    R=np.diag([0.05, 35.0]),
    S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0]),
    xbar=x_ref,
    ubar=ubar,
)

# --- Initial condition and plant evaluator ---
# Numpy backend for fast RK4 rollout of sys_sim between MPC solves.
x0 = np.array([0.0, 3.0, 0.0, U_TARGET * 0.8, 0.0, 0.0])
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

print("MPC straight-line tracking")
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
                # optimizer_method="ipopt",
                optimizer_options={"maxiter": 150, "ftol": 1e-2},
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
        print(
            f"MPC @ t={t:.2f}s  success={res.success}  "
            f"solve={res.solve_time_s:.3f}s"
        )
        prev_plan = plan
        u_hold = plan.u[:, 0].copy()
        mpc_plans.append(
            (t, Trajectory(t=plan.t + t, x=plan.x.copy(), u=plan.u.copy()))
        )
        next_mpc_t += MPC_DT

    # ZOH plant step: hold first planned input and integrate at SIM_HZ.
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


class MpcPlanBicycle(DynamicBicycle):
    """Dynamic bicycle with reference, executed trail, and MPC plan overlays."""

    def __init__(self, mpc_plans, executed_traj, *, x_pad=REF_X_PAD):
        super().__init__()
        x0 = float(executed_traj.x[0, 0]) - x_pad
        x1 = float(executed_traj.x[0, -1]) + x_pad
        self._ref = CustomLine(
            np.array([[x0, 0.0, 0.0], [x1, 0.0, 0.0]]),
            color="k",
            linewidth=1.0,
            style="--",
        )
        self._executed = TrajectoryPolyline(
            executed_traj,
            window="prefix",
            color="b",
            style="--",
            linewidth=1.0,
        )
        self._mpc_plan = HorizonPolyline(
            mpc_plans,
            color="tab:orange",
            linewidth=2.0,
            style="--",
        )

    def get_kinematic_geometry(self):
        vehicle = super().get_kinematic_geometry()
        return [self._ref, self._executed] + vehicle + [self._mpc_plan]

    def get_kinematic_transforms(self, x, u, t):
        vehicle = super().get_kinematic_transforms(x, u, t)
        return [
            np.eye(4),
            time_channel_matrix(t),
            *vehicle,
            time_channel_matrix(t),
        ]


mpc_anim_sys = MpcPlanBicycle(mpc_plans, traj, x_pad=REF_X_PAD)
mpc_anim_sys.params = dict(sys_sim.params)
mpc_anim_sys.camera_scale = CAMERA_SCALE

mpc_anim_sys.traj = traj
mpc_anim_sys.plot_trajectory(signals=("x", "u"))
mpc_anim_sys.animate()

# Animator(mpc_anim_sys).animate_simulation(
#     traj,
#     time_factor_video=TIME_FACTOR_VIDEO,
#     renderer="matplotlib",
#     save=SAVE_ANIMATION,
#     file_name="mpc_straight_line",
#     scene_title="MPC dynamic bicycle",
# )
