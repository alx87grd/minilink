"""Receding-horizon MPC with a slalom of scene-pipeline obstacle costs.

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_multi_obstacle_scene.py

A dry run of the ``planning/spatial`` pipeline against a real workflow: the
hand-written ``TrackingWithObstacleCost`` of ``demo_dynamic_bicycle_rate_mpc_obstacle``
is replaced by

    cost = tracking + w * scene.clearance_field(body).as_cost(shaping=inverse_barrier(...))

so the obstacle term is built from a :class:`~minilink.planning.spatial.scene.Scene`
and a collision body (``point_probe``) and composed onto the tracking cost with ``+``.
Several keepout spheres are staggered along the reference path so the vehicle
must weave while converging to the lane center.

After the closed-loop run, ``scene.plot()`` shows all obstacles with the
executed ``(x, y)`` trajectory overlaid (matplotlib imported lazily).
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
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
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.mpc import (
    MPCDirectCollocationTranscription,
    MPCOptions,
    MPCPlanner,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.collision import bind, point_probe
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
)

U_TARGET = 10.0
TF_SIM = 10.0
VX0 = 6.4

OBSTACLE_RADIUS = 0.4
OBSTACLE_MARGIN = 0.2
OBSTACLE_CENTERS = (
    (12.0, 0.5),
    (20.0, -0.4),
    (28.0, 0.6),
    (36.0, -0.3),
)
OBSTACLE_REPULSION_WEIGHT = 20.0
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

scene = Scene(
    obstacles=tuple(Sphere(center, keepout_radius) for center in OBSTACLE_CENTERS)
)
obstacle_cost = scene.clearance_field(bind(sys_mpc, point_probe())).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = tracking_cost + obstacle_cost

x0 = np.array([0.0, 3.0, 0.0, VX0, 0.0, 0.0, VX0 / r_r, 0.0])
sim_evaluator = sys_sim.compile(backend="jax", verbose=False)

template_problem = PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost)
mpc_planner = MPCPlanner(
    template_problem,
    transcription=MPCDirectCollocationTranscription(
        DirectCollocationOptions(tf=MPC_HORIZON, n_steps=MPC_STEPS)
    ),
    options=MPCOptions(
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        record_solve_time=True,
        optimizer_options={"maxiter": MPC_MAXITER, "ftol": MPC_FTOL},
    ),
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

print("MPC multi-obstacle avoidance (rate inputs, spatial-scene cost)")
print(f"  compile={mpc_planner.compile_time_s:.3f}s (once)")
print(
    f"  u_target={U_TARGET} m/s, horizon={MPC_HORIZON}s, "
    f"{len(OBSTACLE_CENTERS)} obstacles R={OBSTACLE_RADIUS}+{OBSTACLE_MARGIN}"
)

while t < TF_SIM - 1e-12:
    if t >= next_mpc_t - 1e-12:
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
                template_problem,
                mpc_planner.transcription.initial_guess_time_grid(template_problem),
            )

        plan = mpc_planner.step(x, initial_guess=guess)
        res = mpc_planner.last_optimization_result
        print(
            f"MPC @ t={t:.2f}s  success={res.success}  "
            f"solve={res.solve_time_s:.3f}s  step={mpc_planner.last_step_time_s:.3f}s"
        )
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

clearances = [
    np.hypot(traj.x[0, :] - cx, traj.x[1, :] - cy) - OBSTACLE_RADIUS
    for cx, cy in OBSTACLE_CENTERS
]
min_clearance = float(np.min(clearances))
print(
    f"done: min clearance={min_clearance:.2f} m, "
    f"final y={traj.x[1, -1]:.2f} vx={traj.x[3, -1]:.1f}"
)

print("Plotting scene with executed trajectory...")
_, ax = scene.plot(
    show=False,
    show_clearance_contour=True,
    title=f"MPC slalom ({len(OBSTACLE_CENTERS)} keepouts, R={keepout_radius:.2f})",
)
ax.plot(traj.x[0, :], traj.x[1, :], color="tab:blue", linewidth=1.5, label="executed")
ax.legend(loc="upper left")

plt.show()


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
sys_sim.animate(
    traj, overlays=[scene.as_visualizer(color="tab:red", opacity=0.45), history]
)
