"""MPPI on the UdeS racecar circuit with a cone, against the collocation MPC on the same problem."""

import time

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import (
    PlanningProblem,
    QuadraticCost,
    Trajectory,
    TrajectoryOptimizationPlanner,
    UdeSRacecar,
)
from minilink.control.mpc import ModelPredictiveController
from minilink.graphical.animation.drawables import Overlay, SceneHistory
from minilink.graphical.animation.primitives import CustomLine, TrajectoryPolyline
from minilink.graphical.animation.visualization import WORLD
from minilink.planning import (
    ReferenceTrack,
    Scene,
    Sphere,
    TrackCorridorOverlay,
    bind,
    circuit_waypoints,
    from_waypoints,
    point_probe,
    quadratic_hinge,
)
from minilink.planning.evaluation import score_trajectory

V_REF = 3.5  # [m/s]
TF = 10.0  # closed-loop run, s
DT = 0.1  # control period of both controllers, s
N = 10  # MPPI horizon in control periods (1 s, the MPC horizon)
SIM_DT = 0.01  # plant integration step inside one control period
K = 1024  # sampled input sequences per MPPI tick
LAMBDA = 0.05  # temperature of the softmin
SIGMA = (0.5, 0.2)  # std of the input noise: speed [m/s], steer [rad]
ALPHA = 1.0  # 1 turns the information-theoretic control term off: it scales with the
# nominal input, and a 3.5 m/s cruise speed makes it dominate the path cost
SEED = 0
FAN_TICKS = (5, 20, 40, 60)  # ticks whose sampled futures the track figure shows
FAN_SAMPLES = 48
ANIMATION_SAMPLES = 32  # sampled futures drawn at every tick of the animation

# --- the same problem as demos/udes_racecar/mpc_racecar_kinematic.py ---
path = circuit_waypoints(length=6.0, width=4.0, radius=1.0)
track = ReferenceTrack(from_waypoints(path), half_width=0.6)
cone = Sphere((0.0, 2.0), 0.15)
scene = Scene(obstacles=[cone])

car = UdeSRacecar()
car.camera_follow_frame = None
car.camera_scale = 4.0
car.inputs["u"].lower_bound = np.array([0.0, -0.52])
car.inputs["u"].upper_bound = np.array([5.0, 0.52])

start = path[0]
heading = np.arctan2(path[1, 1] - start[1], path[1, 0] - start[0])
x0 = np.array(
    [start[0] - 0.1 * np.sin(heading), start[1] + 0.1 * np.cos(heading), heading]
)
car.x0 = x0

probe = bind(car, point_probe())
quad = QuadraticCost.from_system(
    car, Q=np.zeros((3, 3)), R=np.diag([2.0, 1.0]), ubar=np.array([V_REF, 0.0])
)
corridor = track.corridor_field(probe).as_cost(weight=20.0, shaping=quadratic_hinge())
obstacle = scene.clearance_field(probe).as_cost(
    weight=40.0, shaping=quadratic_hinge(threshold=0.2)
)
cost = quad + corridor + obstacle
problem = PlanningProblem(sys=car, x_start=x0, cost=cost, tf=N * DT)

# --- MPPI: the pieces it reads, and one jitted tick ---
ev = car.compile(backend="jax")
u_lower, u_upper = problem.U.bounding_box().lower, problem.U.bounding_box().upper
n, m = car.n, car.m
t_grid = jnp.arange(N) * DT
Sigma = jnp.diag(jnp.array(SIGMA) ** 2)
Sigma_inv = jnp.linalg.inv(Sigma)
sub_steps = int(round(DT / SIM_DT))


def path_cost(x_path, v):
    """S = sum_t g(x_t, v_t) dt + h(x_N) along one rollout of a held sequence."""
    g = jax.vmap(cost.g)(x_path[:-1], v, t_grid)
    return jnp.sum(g) * DT + cost.h(x_path[-1], N * DT)


@jax.jit
def mppi_tick(U, x0, key):
    """One path-integral update of the nominal sequence U from the measured x0."""
    # Sample: V^k = U + eps^k, eps^k_t ~ N(0, Sigma), clipped to the input box
    eps = jax.random.normal(key, (K, N, m)) @ jnp.linalg.cholesky(Sigma).T
    V = jnp.clip(U + eps, u_lower, u_upper)

    # Roll out: x^k_{t+1} = Phi(x^k_t, v^k_t) over dt, input held, from x0
    X = jax.vmap(ev.rk4_integrate_zoh_trace, in_axes=(None, 0, None, None))(
        x0, V, 0.0, DT
    )

    # Score: the path cost plus lam (1 - alpha) sum_t u_t^T Sigma^-1 eps^k_t
    S = jax.vmap(path_cost)(X, V)
    S = S + LAMBDA * (1.0 - ALPHA) * jnp.einsum("tm,mn,ktn->k", U, Sigma_inv, eps)

    # Weigh: w^k = exp(-(S^k - S_min) / lam), normalized
    S_min = jnp.min(S)
    w = jnp.exp(-(S - S_min) / LAMBDA)
    w = w / jnp.sum(w)

    # Update: U <- U + sum_k w^k eps^k
    U_new = U + jnp.einsum("k,ktm->tm", w, eps)

    x_plan = ev.rk4_integrate_zoh_trace(x0, U_new, 0.0, DT)
    return U_new, x_plan, X, S, w, 1.0 / jnp.sum(w**2)


@jax.jit
def plant_period(x, u, t):
    """The true plant over one control period: RK4 at SIM_DT with the input held."""
    for i in range(sub_steps):
        x = ev.rk4_step_trace(x, u, t + i * SIM_DT, SIM_DT)
    return x


# --- MPPI closed loop by hand: solve, apply u_0, step the plant, shift ---
key = jax.random.PRNGKey(SEED)
U = jnp.tile(jnp.array([V_REF, 0.0]), (N, 1))
x = jnp.array(x0)
n_ticks = int(round(TF / DT))
ts, xs, us, fans, futures, tick_times = [], [], [], {}, [], []
for k in range(n_ticks):
    key, k_tick = jax.random.split(key)
    t0 = time.perf_counter()
    U, x_plan, X, S, w, ess = mppi_tick(U, x, k_tick)
    U.block_until_ready()
    tick_times.append(time.perf_counter() - t0)
    shown = jnp.argsort(w)[-ANIMATION_SAMPLES:]
    futures.append(
        (k * DT, np.asarray(X[shown]), np.asarray(w[shown]), np.asarray(x_plan))
    )
    if k in FAN_TICKS:
        best = jnp.argsort(w)[-FAN_SAMPLES:]
        fans[k] = (np.asarray(X[best]), np.asarray(w[best]), np.asarray(x_plan))
        print(
            f"MPPI tick {k:3d}  t={k * DT:.1f}s  S_min={float(jnp.min(S)):.3f}  "
            f"effective samples={float(ess):.1f} / {K}  tick={tick_times[-1] * 1e3:.1f} ms"
        )
    ts.append(k * DT)
    xs.append(np.asarray(x))
    us.append(np.asarray(U[0]))
    x = plant_period(x, U[0], k * DT)
    U = jnp.concatenate([U[1:], U[-1:]])
traj_mppi = Trajectory(np.array(ts), np.array(xs).T, np.array(us).T)

# --- the collocation MPC of the demo on the same problem, same plant, same run ---
planner = TrajectoryOptimizationPlanner(
    problem,
    n_steps=8,
    transcription="direct_collocation",
    compile_backend="jax",
    optimizer_method="scipy_slsqp",
    optimizer_options={"maxiter": 40, "ftol": 0.05},
)
mpc = ModelPredictiveController(planner, dt_mpc=DT, warm_start=True)
t0 = time.perf_counter()
result = (mpc @ car).compute_trajectory(
    tf=TF, x0_plant=x0, plant_dt_inner=SIM_DT, compile_backend="jax"
)
mpc_wall = time.perf_counter() - t0
traj_mpc = result.plant


# --- the benchmark: closed-loop cost, clearance to the cone, corridor margin, tick time ---
def report(name, traj, tick_ms):
    J, _ = score_trajectory(problem, traj)
    xy = traj.x[:2].T
    clearance = np.min(np.linalg.norm(xy - cone.center, axis=1)) - cone.radius
    margin = min(float(track.corridor_field(probe).value(xk)) for xk in traj.x.T)
    driven = np.sum(np.linalg.norm(np.diff(xy, axis=0), axis=1))
    print(
        f"{name:5s}  J={J:7.2f}  min cone clearance={clearance:5.2f} m  "
        f"min corridor margin={margin:5.2f} m  driven={driven:5.1f} m  "
        f"tick={tick_ms:5.1f} ms"
    )


print()
report("MPPI", traj_mppi, 1e3 * float(np.median(tick_times[1:])))
report("MPC", traj_mpc, 1e3 * mpc_wall / n_ticks)

# --- the two laps on the track, the cone, and MPPI's sampled futures at four ticks ---
fig, ax = track.plot(show=False)
scene.plot(ax=ax, show_density=False, show=False)
for k, (X_fan, w_fan, x_plan) in fans.items():
    for x_path, w_path in zip(X_fan, w_fan / w_fan.max()):
        ax.plot(
            x_path[:, 0],
            x_path[:, 1],
            color="tab:blue",
            alpha=0.08 + 0.5 * w_path,
            lw=0.7,
        )
    ax.plot(x_plan[:, 0], x_plan[:, 1], color="black", lw=1.5)
ax.plot(
    traj_mppi.x[0], traj_mppi.x[1], color="tab:blue", lw=2, label="MPPI closed loop"
)
ax.plot(
    traj_mpc.x[0],
    traj_mpc.x[1],
    color="tab:red",
    lw=2,
    ls="--",
    label="MPC closed loop",
)
ax.set_xlim(path[:, 0].min() - 1.0, path[:, 0].max() + 1.0)
ax.set_ylim(path[:, 1].min() - 1.0, path[:, 1].max() + 1.0)
ax.legend(loc="upper right", fontsize=8)
ax.set_title(f"UdeS racecar circuit: MPPI ({K} samples) vs collocation MPC")
plt.show()


# --- the animation: at each tick, the sampled futures shaded by weight and the plan ---
class SampledFutures(Overlay):
    """The latest tick's sampled rollouts (alpha = weight) and its updated plan, in the world frame."""

    def __init__(self, futures, *, color=(0.12, 0.47, 0.71)):
        self.futures = futures
        self.color = color

    def get_dynamic_geometry(self, t=0.0, params=None):
        t_solve, X_fan, w_fan, x_plan = max(
            (f for f in self.futures if f[0] <= t + 1e-9), key=lambda f: f[0]
        )
        lines = [
            CustomLine(
                x_path[:, :2], color=(*self.color, 0.08 + 0.6 * w_path), linewidth=0.8
            )
            for x_path, w_path in zip(X_fan, w_fan / w_fan.max())
        ]
        lines.append(CustomLine(x_plan[:, :2], color="black", linewidth=2.0))
        return {WORLD: lines}


overlays = [
    TrackCorridorOverlay(track),
    scene.as_visualizer(),
    SampledFutures(futures),
    SceneHistory(
        trail=TrajectoryPolyline(
            traj_mppi, window="prefix", color="#1565c0", style="--", linewidth=1.0
        )
    ),
]
car.animate(traj_mppi, overlays=overlays)
