"""Receding-horizon MPC with a slalom of scene-pipeline obstacle costs.

Run from repo root::

    python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_multi_obstacle_scene.py

A dry run of the ``planning/spatial`` pipeline against a real workflow: the
hand-written ``TrackingWithObstacleCost`` of ``demo_dynamic_bicycle_rate_mpc_obstacle``
is replaced by

    cost = tracking + w * scene.clearance_field(robot).as_cost(shaping=inverse_barrier(...))

so the obstacle term is built from a :class:`~minilink.planning.spatial.scene.Scene`
and a robot body (``point``) and composed onto the tracking cost with ``+``.
Several keepout spheres are staggered along the reference path so the vehicle
must weave while converging to the lane center.

After the closed-loop run, ``scene.plot()`` shows all obstacles with the
executed ``(x, y)`` trajectory overlaid (matplotlib imported lazily).
"""

import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.costs import QuadraticCost
from minilink.core.geometry import Sphere
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    JaxDynamicBicycleRateInputs,
)
from minilink.graphical.animation.primitives import (
    Circle,
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
    time_channel_matrix,
)
from minilink.planning.initial_guess import default_initial_trajectory
from minilink.planning.problems import PlanningProblem
from minilink.planning.spatial.robot import point
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import inverse_barrier
from minilink.planning.trajectory_optimization.direct_collocation import (
    DirectCollocationOptions,
    DirectCollocationTranscription,
)
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationOptions,
    TrajectoryOptimizationPlanner,
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
obstacle_cost = scene.clearance_field(point(position=(0, 1))).as_cost(
    weight=OBSTACLE_REPULSION_WEIGHT,
    shaping=inverse_barrier(epsilon=OBSTACLE_REPULSION_EPS),
)
cost = tracking_cost + obstacle_cost

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

print("MPC multi-obstacle avoidance (rate inputs, spatial-scene cost)")
print(
    f"  u_target={U_TARGET} m/s, horizon={MPC_HORIZON}s, "
    f"{len(OBSTACLE_CENTERS)} obstacles R={OBSTACLE_RADIUS}+{OBSTACLE_MARGIN}"
)
for index, center in enumerate(OBSTACLE_CENTERS, start=1):
    print(f"    {index}: center={center}")

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
import matplotlib.pyplot as plt

plt.show()


class MpcObstacleBicycleRate(JaxDynamicBicycleRateInputs):
    def __init__(
        self, mpc_plans, executed_traj, *, obstacle_centers, keepout_radius, x_pad
    ):
        super().__init__()
        x_lo = float(executed_traj.x[0, 0]) - x_pad
        x_hi = float(executed_traj.x[0, -1]) + x_pad
        self._ref = CustomLine(
            np.array([[x_lo, 0.0, 0.0], [x_hi, 0.0, 0.0]]),
            color="k",
            linewidth=1.0,
            style="--",
        )
        self._obstacles = [
            Circle(
                radius=keepout_radius,
                center=(cx, cy, 0.0),
                color="tab:red",
                fill=True,
            )
            for cx, cy in obstacle_centers
        ]
        self._executed = TrajectoryPolyline(
            executed_traj, window="prefix", color="b", style="--", linewidth=1.0
        )
        self._mpc_plan = HorizonPolyline(
            mpc_plans, color="tab:orange", linewidth=2.0, style="--"
        )

    def get_kinematic_geometry(self):
        vehicle = super().get_kinematic_geometry()
        return (
            [self._ref]
            + self._obstacles
            + [self._executed]
            + vehicle
            + [self._mpc_plan]
        )

    def get_kinematic_transforms(self, x, u, t):
        vehicle = super().get_kinematic_transforms(x, u, t)
        n_obstacles = len(self._obstacles)
        return (
            [np.eye(4)]
            + [np.eye(4)] * n_obstacles
            + [time_channel_matrix(t)]
            + list(vehicle)
            + [time_channel_matrix(t)]
        )

    # === v2 frame-keyed visualization contract ===========================
    # Overlays are honest dynamic geometry: each frame bakes the world-frame
    # polyline from ``points_at(t)`` into a CustomLine (no T[3,3] time channel).

    def get_kinematic_geometry_v2(self):
        geometry = super().get_kinematic_geometry_v2()
        geometry.setdefault("world", [])
        geometry["world"] = [self._ref, *self._obstacles, *geometry["world"]]
        return geometry

    def tf_v2(self, x, u, t=0, params=None):
        frames = super().tf_v2(x, u, t)
        frames.setdefault("world", np.eye(4))
        return frames

    def get_dynamic_geometry_v2(self, x, u, t=0, params=None):
        dynamic = super().get_dynamic_geometry_v2(x, u, t)
        dynamic.setdefault("world", [])
        dynamic["world"] = [
            *dynamic["world"],
            CustomLine(
                self._executed.points_at(t), color="b", style="--", linewidth=1.0
            ),
            CustomLine(
                self._mpc_plan.points_at(t),
                color="tab:orange",
                style="--",
                linewidth=2.0,
            ),
        ]
        return dynamic


mpc_anim_sys = MpcObstacleBicycleRate(
    mpc_plans,
    traj,
    obstacle_centers=OBSTACLE_CENTERS,
    keepout_radius=keepout_radius,
    x_pad=REF_X_PAD,
)
mpc_anim_sys.params = dict(sys_sim.params)
mpc_anim_sys.camera_scale = CAMERA_SCALE
mpc_anim_sys.traj = traj
mpc_anim_sys.plot_trajectory(signals=("x", "u"))
mpc_anim_sys.animate()
