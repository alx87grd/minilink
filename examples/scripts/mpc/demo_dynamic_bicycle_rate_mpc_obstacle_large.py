"""Rate-input MPC with a large circular obstacle (wide keep-out disk).

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_obstacle_large.py
"""

import numpy as np

from minilink.core.backends import array_module, configure_jax
from minilink.core.costs import CostFunction, QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import JaxDynamicBicycleRateInputs
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.catalog import SceneHistory
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.scene import Scene
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
)

U_TARGET = 8.0
TF_SIM = 5.0
VX0 = 6.4

OBSTACLE_CENTER = (20.0, 0.0)
OBSTACLE_RADIUS = 5.0
OBSTACLE_MARGIN = 0.2
OBSTACLE_REPULSION_WEIGHT = 50.0
OBSTACLE_REPULSION_EPS = 0.08

MPC_HZ = 5.0
SIM_HZ = 200.0
MPC_HORIZON = 2.0
MPC_STEPS = 20
MPC_MAXITER = 100
MPC_FTOL = 1e-1
MPC_DT = 1.0 / MPC_HZ
SIM_DT = 1.0 / SIM_HZ
SUBSTEPS = max(1, int(round(MPC_DT / SIM_DT)))

W_REAR_MAX = 90.0
DELTA_MAX = 0.55
W_REAR_DOT_MAX = 80.0
DELTA_DOT_MAX = 2.0

REF_X_PAD = 20.0
CAMERA_SCALE = 12.0


class TrackingWithObstacleCost(CostFunction):
    """Quadratic tracking plus ``weight / max(clearance, eps)²`` in ``(x, y)``."""

    def __init__(self, tracking, *, center, radius, weight, eps):
        self.tracking = tracking
        self.center = center
        self.radius = float(radius)
        self.weight = float(weight)
        self.eps = float(eps)

    def _repulsion(self, x):
        xp = array_module(x)
        cx, cy = self.center
        clearance = xp.hypot(x[0] - cx, x[1] - cy) - self.radius
        return self.weight / xp.maximum(clearance, self.eps) ** 2

    def g(self, x, u, t=0.0, params=None):
        return self.tracking.g(x, u, t, params=params) + self._repulsion(x)

    def h(self, x, t=0.0, params=None):
        return self.tracking.h(x, t, params=params) + self._repulsion(x)


configure_jax(enable_x64=True)

sys_mpc = JaxDynamicBicycleRateInputs()
sys_sim = JaxDynamicBicycleRateInputs()
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

keepout_radius = OBSTACLE_RADIUS + OBSTACLE_MARGIN
r_r = sys_mpc.params["r_r"]
w_rear_ref = U_TARGET / r_r
x_ref = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, w_rear_ref, 0.0])
ubar = np.array([0.0, 0.0])
tracking_cost = QuadraticCost.from_system(
    sys_mpc,
    Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    R=np.diag([1.0, 25.0]),
    S=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
    xbar=x_ref,
    ubar=ubar,
)
cost = TrackingWithObstacleCost(
    tracking_cost,
    center=OBSTACLE_CENTER,
    radius=keepout_radius,
    weight=OBSTACLE_REPULSION_WEIGHT,
    eps=OBSTACLE_REPULSION_EPS,
)

x0 = np.array([0.0, 3.0, 0.0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])
sim_evaluator = sys_sim.compile(backend="jax", verbose=False)

transcription = DirectCollocationTranscription(
    DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
)
trajopt_options = TrajectoryOptimizationOptions(
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    solve_disp=False,
    record_solve_time=True,
    optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
)

t_hist = [0.0]
x_hist = [x0.copy()]
u_hist = [np.zeros(sys_sim.m)]
mpc_plans = []
x = x0.copy()
t = 0.0
u_hold = np.zeros(sys_sim.m)
prev_plan = None
next_mpc_t = 0.0

print("MPC large obstacle (rate inputs, soft repulsion)")
print(
    f"  u_target={U_TARGET} m/s, obstacle={OBSTACLE_CENTER} "
    f"R={OBSTACLE_RADIUS}+{OBSTACLE_MARGIN}"
)

while t < TF_SIM - 1e-12:
    if t >= next_mpc_t - 1e-12:
        problem = PlanningProblem(sys=sys_mpc, x_start=x, cost=cost)
        planner = TrajectoryOptimizationPlanner(
            problem,
            transcription=transcription,
            options=trajopt_options,
        )

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
        else:
            guess = default_initial_trajectory(
                problem,
                transcription.initial_guess_time_grid(problem),
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

    for _ in range(SUBSTEPS):
        if t >= TF_SIM:
            break
        x = sim_evaluator.rk4_step(x, u_hold, t, SIM_DT)
        t += SIM_DT
        t_hist.append(t)
        x_hist.append(x.copy())
        u_hist.append(u_hold.copy())

traj = Trajectory(t=np.asarray(t_hist), x=np.asarray(x_hist).T, u=np.asarray(u_hist).T)

cx, cy = OBSTACLE_CENTER
clearance = np.hypot(traj.x[0, :] - cx, traj.x[1, :] - cy) - OBSTACLE_RADIUS
print(
    f"done: min clearance={float(clearance.min()):.2f} m, "
    f"final y={traj.x[1, -1]:.2f} vx={traj.x[3, -1]:.1f}"
)


# --- Animation ---
scene = Scene(obstacles=[Sphere(OBSTACLE_CENTER, keepout_radius)])
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
sys_sim.animate(traj, overlays=[scene.as_visualizer(color="tab:red", opacity=0.45), history])
