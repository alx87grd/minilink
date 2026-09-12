"""Car on the wide technical circuit: the MPC demo's task solved by the RL planner."""

import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import BicycleDynRate, CostFunction, QuadraticCost
from minilink.planning import (
    MonteCarloEvaluator,
    ReferenceTrack,
    ReinforcementLearningPlanner,
    Sampler,
    StochasticPlanningProblem,
    TrackCorridorOverlay,
    bind,
    car_outline,
    from_waypoints,
    plot_track,
    quadratic_excess,
    quadratic_hinge,
)

TRAINING_TIMESTEPS = 1_500_000  # about a minute on a laptop CPU
DT = 0.05
U_TARGET = (
    12.0  # m/s cruise target: 6 gives 17 s laps, 12 about 9.5 s, 15 leaves the track
)
TF_SIM = 30.0
CORRIDOR_HALF_WIDTH = 2.5

# Same wide CCW loop as examples/demos/mpc/mpc_car_circuit.py (no obstacles)
# fmt: off
loop_xy = np.array([
    (-14.0, -10.0), (-9.3333, -10.0), (-7.0, -10.0), (-3.0, -8.2), (-1.0, -10.0),
    (10.5, -10.0), (13.7101, -9.6985), (14.5, -9.3301), (15.2139, -8.8302),
    (15.8302, -8.2139), (16.3301, -7.5), (16.6985, -6.7101), (16.9240, -5.8682),
    (17.0, -5.0), (17.0, 0.0), (17.0, 2.5), (16.6985, 6.7101), (16.3301, 7.5),
    (15.8302, 8.2139), (15.2139, 8.8302), (14.5, 9.3301), (13.7101, 9.6985),
    (12.8682, 9.9240), (12.0, 10.0), (0.75, 10.0), (-4.875, 10.0),
    (-14.6971, 9.7889), (-15.25, 9.5311), (-15.7498, 9.1812), (-16.1812, 8.7498),
    (-16.5311, 8.25), (-16.7889, 7.6971), (-16.9468, 7.1078), (-17.0, 6.5),
    (-17.0, 0.75), (-17.0, -2.125), (-16.6985, -6.7101), (-16.3301, -7.5),
    (-15.8302, -8.2139), (-15.2139, -8.8302), (-14.5, -9.3301),
    (-13.7101, -9.6985), (-12.8682, -9.9240), (-12.0, -10.0),
])
# fmt: on
track = ReferenceTrack(from_waypoints(loop_xy), half_width=CORRIDOR_HALF_WIDTH)

# Polyline constants of a JAX-traceable projection (the policy's track sensing)
A = jnp.asarray(loop_xy[:-1])
AB = jnp.asarray(np.diff(loop_xy, axis=0))
SEG = jnp.linalg.norm(AB, axis=1)
S_KNOTS = jnp.concatenate([jnp.zeros(1), jnp.cumsum(SEG)])
L_TOTAL = float(S_KNOTS[-1])


def project(p_xy):
    """Closest point on the polyline: (arc length, point, unit tangent)."""
    tau = jnp.clip(jnp.sum((p_xy - A) * AB, axis=1) / SEG**2, 0.0, 1.0)
    closest = A + AB * tau[:, None]
    i = jnp.argmin(jnp.linalg.norm(p_xy - closest, axis=1))
    return S_KNOTS[i] + tau[i] * SEG[i], closest[i], AB[i] / SEG[i]


def point_at(s):
    """Point and unit tangent at arc length ``s`` (periodic on the loop)."""
    s = jnp.mod(s, L_TOTAL)
    i = jnp.clip(jnp.searchsorted(S_KNOTS, s, side="right") - 1, 0, len(loop_xy) - 2)
    tau = (s - S_KNOTS[i]) / SEG[i]
    return A[i] + tau * AB[i], AB[i] / SEG[i]


# Plant: the MPC demo's dynamic bicycle. The box is wide on speeds and wheel
# rate (stopping or reversing is not an exit); the steering limit and the map
# edges are exits, and an exit terminates the episode with the exit cost.
sys = BicycleDynRate()
r_r = sys.params["r_r"]
sys.inputs["u"].lower_bound = np.array([-80.0, -2.0])
sys.inputs["u"].upper_bound = np.array([80.0, 2.0])
sys.state.lower_bound = np.array(
    [-25.0, -25.0, -1e3, -20.0, -20.0, -10.0, -200.0, -0.55]
)
sys.state.upper_bound = np.array([25.0, 25.0, 1e3, 40.0, 20.0, 10.0, 200.0, 0.55])
sys.camera_scale = 18.0

# Cost: the MPC demo's terms with the input weights scaled for a per-step reward
x_cruise = np.array([0.0, 0.0, 0.0, U_TARGET, 0.0, 0.0, U_TARGET / r_r, 0.0])
body = bind(sys, car_outline(length=2.4, width=0.2, margin=0.05))
running = (
    QuadraticCost.from_system(
        sys,
        Q=np.diag([0.0, 0.0, 0.0, 0.15, 4.0, 6.0, 0.1, 80.0]),
        R=np.diag([0.001, 1.0]),
        xbar=x_cruise,
    )
    + track.distance_field(body).as_cost(
        weight=20.0, shaping=quadratic_excess(threshold=0.1)
    )
    + track.corridor_field(body).as_cost(
        weight=25.0, shaping=quadratic_hinge(threshold=0.0)
    )
)


class TrackCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        return running.g(x, u, t, params)

    def h(self, x, t=0.0, params=None):
        return 0.0


# Initial states: anywhere along the loop, near the centerline, roughly aligned
def start_anywhere(key):
    import jax

    k1, k2, k3, k4, k5 = jax.random.split(key, 5)
    s = jax.random.uniform(k1, (), minval=0.0, maxval=L_TOTAL)
    q, tangent = point_at(s)
    normal = jnp.array([-tangent[1], tangent[0]])
    p_xy = q + normal * jax.random.uniform(k2, (), minval=-1.0, maxval=1.0)
    theta = jnp.arctan2(tangent[1], tangent[0]) + jax.random.uniform(
        k3, (), minval=-0.3, maxval=0.3
    )
    vx = jax.random.uniform(k4, (), minval=2.0, maxval=U_TARGET + 2.0)
    delta = jax.random.uniform(k5, (), minval=-0.1, maxval=0.1)
    return jnp.array([p_xy[0], p_xy[1], theta, vx, 0.0, 0.0, vx / r_r, delta])


s0, _ = track.path.project(loop_xy[0])
t0v = track.path.tangent(s0)
x_start = np.array(
    [*loop_xy[0], float(np.arctan2(t0v[1], t0v[0])), 5.0, 0.0, 0.0, 5.0 / r_r, 0.0]
)

problem = StochasticPlanningProblem(
    sys,
    cost=TrackCost(),
    tf=np.inf,
    x0_distribution=Sampler(start_anywhere, mean=x_start),
    on_exit="terminate",
    exit_cost=200.0,
)

# Policy features: what a localized car senses — signed lateral offset,
# heading error, scaled body speeds and steering, and the body-frame
# direction to three lookahead points on the centerline
LOOKAHEAD = (3.0, 6.0, 12.0)


def features(x):
    p_xy, theta = x[:2], x[2]
    s, closest, tangent = project(p_xy)
    d_lat = tangent[0] * (p_xy[1] - closest[1]) - tangent[1] * (p_xy[0] - closest[0])
    e_psi = theta - jnp.arctan2(tangent[1], tangent[0])
    c, sn = jnp.cos(theta), jnp.sin(theta)
    ahead = []
    for L in LOOKAHEAD:
        r = point_at(s + L)[0] - p_xy
        ahead += [(c * r[0] + sn * r[1]) / L, (-sn * r[0] + c * r[1]) / L]
    body_state = [x[3] / 10.0, x[4] / 5.0, x[5] / 2.0, x[6] * r_r / 10.0, x[7] / 0.5]
    return jnp.array(
        [d_lat / CORRIDOR_HALF_WIDTH, jnp.cos(e_psi), jnp.sin(e_psi)]
        + body_state
        + ahead
    )


planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=features,
    hidden=(64, 64),
    n_envs=16,
    n_steps=128,
    batch_size=256,
    gamma=0.98,
)
plan = planner.solve(timesteps=TRAINING_TIMESTEPS)
print(f"\n{plan.metadata.message} in {plan.metadata.solve_time_s:.1f} s")
planner.plot_learning_curve()
ppo_ctl = planner.get_controller()

report = MonteCarloEvaluator(problem, dt=DT, n_trials=50, seed=1).evaluate(ppo_ctl)
print("Monte Carlo over random starts on the loop (failure = left the box):", report)

# Closed loop from the start line, more than one lap
sys.x0 = x_start
cl_sys = ppo_ctl @ sys
cl_sys.name = "Car with the learned law"
traj = cl_sys.compute_trajectory(tf=TF_SIM, dt=0.01)
cl_sys.plot_trajectory(traj)

fig, ax = plot_track(track, show=False, title="RL policy on the circuit")
ax.plot(traj.x[0], traj.x[1], color="tab:red", linewidth=1.5, label="closed loop")
ax.legend()

distance = np.array([float(track.path.distance(q)) for q in traj.x[:2].T[::10]])
arc = np.array([track.path.project(q)[0] for q in traj.x[:2].T[::10]])
ds = np.diff(arc)
ds[ds < -L_TOTAL / 2] += L_TOTAL  # unwrap the lap counter across the seam
ds[ds > L_TOTAL / 2] -= L_TOTAL
laps = float(ds.sum() / L_TOTAL)
print(
    "Laps in", TF_SIM, "s:", round(laps, 2), "-> lap time", round(TF_SIM / laps, 1), "s"
)
print("Max distance to the centerline:", round(float(distance.max()), 2), "m")
print("Mean speed:", round(float(traj.x[3].mean()), 2), "m/s")

plt.show()
cl_sys.animate(traj, overlays=[TrackCorridorOverlay(track)])
