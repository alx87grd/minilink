"""Car on the wide technical circuit: a PPO policy learned in JAX instead of MPC."""

import time

import matplotlib.pyplot as plt
import numpy as np

from minilink import BicycleDynRate, CostFunction, QuadraticCost
from minilink.core.backends import array_module
from minilink.experimental.ppo_jax import PPO
from minilink.planning.spatial.collision import bind, car_outline
from minilink.planning.spatial.overlays import TrackCorridorOverlay
from minilink.planning.spatial.paths import from_waypoints
from minilink.planning.spatial.plotting import plot_track
from minilink.planning.spatial.shaping import quadratic_excess, quadratic_hinge
from minilink.planning.spatial.track import ReferenceTrack

TRAINING_TIMESTEPS = 1_500_000  # laps by ~400k steps; lap time keeps improving to ~1.5M
DT = 0.05  # control period, one RK4 step per period
TF = 10.0  # episode duration
U_TARGET = (
    12.0  # m/s cruise target: 6 gives 17 s laps, 12 about 9.5 s, 15 leaves the track
)
TF_SIM = 30.0  # closed-loop test: more than one lap
CORRIDOR_HALF_WIDTH = 2.5

# Same wide CCW loop as examples/demos/mpc/mpc_car_circuit.py (no obstacles).
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

# Polyline constants for the JAX-traceable projection used by the features
A = loop_xy[:-1]
AB = np.diff(loop_xy, axis=0)
SEG = np.linalg.norm(AB, axis=1)
S_KNOTS = np.concatenate([[0.0], np.cumsum(SEG)])
L_TOTAL = float(S_KNOTS[-1])


def project(p_xy):
    """Closest point on the polyline: (arc length, point, unit tangent)."""
    xp = array_module(p_xy)
    a, ab, seg, s_knots = (xp.asarray(v) for v in (A, AB, SEG, S_KNOTS))
    tau = xp.clip(xp.sum((p_xy - a) * ab, axis=1) / seg**2, 0.0, 1.0)
    closest = a + ab * tau[:, None]
    i = xp.argmin(xp.linalg.norm(p_xy - closest, axis=1))
    return s_knots[i] + tau[i] * seg[i], closest[i], ab[i] / seg[i]


def point_at(s):
    """Point and unit tangent at arc length ``s`` (periodic on the loop)."""
    xp = array_module(s)
    a, ab, seg, s_knots = (xp.asarray(v) for v in (A, AB, SEG, S_KNOTS))
    s = xp.mod(s, L_TOTAL)
    i = xp.clip(xp.searchsorted(s_knots, s, side="right") - 1, 0, len(SEG) - 1)
    tau = (s - s_knots[i]) / seg[i]
    return a[i] + tau * ab[i], ab[i] / seg[i]


# Plant: the MPC demo's dynamic bicycle with wheel-acceleration and steer-rate
# inputs. The state box is wide on speeds and wheel rate (stopping or reversing
# is not an exit); the steering limit and the map edges are, and leaving the box
# terminates the episode with the terminal cost h.
sys = BicycleDynRate()
r_r = sys.params["r_r"]
sys.inputs["u"].lower_bound = np.array([-80.0, -2.0])
sys.inputs["u"].upper_bound = np.array([80.0, 2.0])
sys.state.lower_bound = np.array(
    [-25.0, -25.0, -1e3, -20.0, -20.0, -10.0, -200.0, -0.55]
)
sys.state.upper_bound = np.array([25.0, 25.0, 1e3, 40.0, 20.0, 10.0, 200.0, 0.55])
sys.camera_scale = 18.0

# Cost: the MPC demo's terms (cruise speed, small slip and yaw rate, small
# steering, path distance, corridor) with the input weights scaled down for a
# per-step reward, plus a fixed terminal penalty when the car leaves the box.
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
    """Running cost of the track problem, terminal penalty for leaving the box."""

    def g(self, x, u, t=0.0, params=None):
        return running.g(x, u, t, params)

    def h(self, x, t=0.0, params=None):
        return 200.0


cost = TrackCost()

# Observation features: what a localized car senses about the track, not the
# absolute pose. Signed lateral offset, heading error, scaled body speeds and
# steering, and the body-frame direction to three lookahead points on the
# centerline. The learned law is still u = pi(x).
LOOKAHEAD = (3.0, 6.0, 12.0)


def features(x):
    xp = array_module(x)
    p_xy, theta = x[:2], x[2]
    s, closest, tangent = project(p_xy)
    d_lat = tangent[0] * (p_xy[1] - closest[1]) - tangent[1] * (p_xy[0] - closest[0])
    e_psi = theta - xp.arctan2(tangent[1], tangent[0])
    c, sn = xp.cos(theta), xp.sin(theta)
    ahead = []
    for L in LOOKAHEAD:
        r = point_at(s + L)[0] - p_xy
        ahead += [(c * r[0] + sn * r[1]) / L, (-sn * r[0] + c * r[1]) / L]
    body_state = [x[3] / 10.0, x[4] / 5.0, x[5] / 2.0, x[6] * r_r / 10.0, x[7] / 0.5]
    return xp.array(
        [d_lat / CORRIDOR_HALF_WIDTH, xp.cos(e_psi), xp.sin(e_psi)] + body_state + ahead
    )


# Initial states: anywhere along the loop, within a metre of the centerline,
# roughly aligned with it, between 2 m/s and the cruise speed plus 2 m/s.
def reset(key):
    import jax

    k1, k2, k3, k4, k5 = jax.random.split(key, 5)
    s = jax.random.uniform(k1, (), minval=0.0, maxval=L_TOTAL)
    q, tangent = point_at(s)
    normal = array_module(q).array([-tangent[1], tangent[0]])
    p_xy = q + normal * jax.random.uniform(k2, (), minval=-1.0, maxval=1.0)
    theta = array_module(q).arctan2(tangent[1], tangent[0])
    theta = theta + jax.random.uniform(k3, (), minval=-0.3, maxval=0.3)
    vx = jax.random.uniform(k4, (), minval=2.0, maxval=U_TARGET + 2.0)
    delta = jax.random.uniform(k5, (), minval=-0.1, maxval=0.1)
    return array_module(q).array(
        [p_xy[0], p_xy[1], theta, vx, 0.0, 0.0, vx / r_r, delta]
    )


ppo = PPO(
    sys,
    cost,
    dt=DT,
    tf=TF,
    reset_mode=reset,
    domain_exit="terminate",
    features=features,
    n_envs=16,
    n_steps=128,
    batch_size=256,
    gamma=0.98,
    seed=0,
)
ppo_ctl = ppo.controller

t0 = time.time()
ppo.learn(TRAINING_TIMESTEPS)
print(f"\nTrained {ppo.num_timesteps} steps in {time.time() - t0:.1f} s")

# Learning curve: mean return of the exploration episodes
steps = [h["timesteps"] for h in ppo.history]
ep_return = [h["ep_return_mean"] for h in ppo.history]
fig, ax = plt.subplots(figsize=(8, 3))
ax.plot(steps, ep_return)
ax.set_xlabel("timesteps")
ax.set_ylabel("mean episode return")
ax.grid(True, alpha=0.3)

# Closed loop from the start line at 5 m/s, heading along the first straight
s0, _ = track.path.project(loop_xy[0])
t0v = track.path.tangent(s0)
theta0 = float(np.arctan2(t0v[1], t0v[0]))
sys.x0 = np.array([loop_xy[0, 0], loop_xy[0, 1], theta0, 5.0, 0.0, 0.0, 5.0 / r_r, 0.0])
cl_sys = ppo_ctl @ sys
cl_sys.name = "Car with PPO controller (JAX)"
traj = cl_sys.compute_trajectory(tf=TF_SIM, dt=0.01)
cl_sys.plot_trajectory(traj)

# The driven line on the track
fig, ax = plot_track(track, show=False, title="PPO policy on the circuit")
ax.plot(traj.x[0], traj.x[1], color="tab:red", linewidth=1.5, label="closed loop")
ax.legend()

# Lap statistics
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
