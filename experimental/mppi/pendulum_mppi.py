"""MPPI on the pendulum swing-up: the path-integral planner as one standalone script (research lane)."""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, Pendulum, PlanningProblem, Trajectory

K = 1024  # sampled input sequences per tick
N = 40  # control periods in the horizon
DT = 0.05  # control period, s (horizon N * DT = 2 s)
TF_SIM = 6.0  # closed-loop run, s
LAMBDA = 0.1  # temperature of the softmin
SIGMA = 3.0  # std of the input noise, Nm
ALPHA = 0.0  # 0 charges the information-theoretic control term, 1 turns it off
TORQUE = 4.0  # Nm, below m g l: the pendulum must pump energy
SEED = 0
FAN_TICKS = (0, 24, 48)  # ticks whose sampled futures the figure shows
FAN_SAMPLES = 64  # samples drawn in the fan

# Plant: theta = 0 hanging, theta = pi upright, torque limited
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])


# Cost: the wrapped angle error to upright, a rate term, a small effort term; no
# terminal cost. (1 + cos theta is flat at the bottom: input noise around a zero
# sequence then changes the cost by nothing and the softmin has no signal.)
class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        e = jnp.arctan2(jnp.sin(theta - jnp.pi), jnp.cos(theta - jnp.pi))
        return e**2 + 0.1 * dtheta**2 + 0.001 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = PlanningProblem(plant, cost=SwingUpCost(), tf=N * DT, x_start=[0.0, 0.0])

# The pieces the planner reads: the plant's held-input RK4 step, the cost, the input box
ev = plant.compile(backend="jax")
cost = problem.cost
u_lower, u_upper = problem.U.bounding_box().lower, problem.U.bounding_box().upper
n, m = plant.n, plant.m
t_grid = jnp.arange(N) * DT
Sigma = SIGMA**2 * jnp.eye(m)
Sigma_inv = jnp.linalg.inv(Sigma)


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

    # The planner's evidence: the nominal rollout of U_new, and the tick's diagnostics
    x_plan = ev.rk4_integrate_zoh_trace(x0, U_new, 0.0, DT)
    effective_samples = 1.0 / jnp.sum(w**2)
    return U_new, x_plan, X, S, w, effective_samples


# Receding horizon by hand: solve, apply u_0, step the plant, shift the sequence
key = jax.random.PRNGKey(SEED)
U = jnp.zeros((N, m))
x = jnp.array(problem.x_start)
n_ticks = int(round(TF_SIM / DT))
ts, xs, us, fans = [], [], [], {}
for k in range(n_ticks):
    key, k_tick = jax.random.split(key)
    U, x_plan, X, S, w, ess = mppi_tick(U, x, k_tick)
    u = U[0]
    if k in FAN_TICKS:
        best = jnp.argsort(w)[-FAN_SAMPLES:]  # the heaviest samples of the fan
        fans[k] = (k * DT, np.asarray(X[best]), np.asarray(w[best]), np.asarray(x_plan))
        print(
            f"tick {k:3d}  t={k * DT:.2f}s  S_min={float(jnp.min(S)):.3f}  "
            f"effective samples={float(ess):.1f} / {K}"
        )
    ts.append(k * DT)
    xs.append(np.asarray(x))
    us.append(np.asarray(u))
    x = ev.rk4_step(x, u, k * DT, DT)
    U = jnp.concatenate([U[1:], U[-1:]])  # shift: (u_1, ..., u_{N-1}, u_{N-1})
traj = Trajectory(np.array(ts), np.array(xs).T, np.array(us).T)

plant.plot_trajectory(traj)

# The fan of sampled futures at three ticks: the heaviest samples shaded by weight,
# the path-integral update in black, the closed-loop past in grey
fig, axes = plt.subplots(
    1, len(FAN_TICKS), figsize=(4 * len(FAN_TICKS), 3.5), sharey=True
)
for ax, k in zip(axes, FAN_TICKS):
    t_fire, X_fan, w_fan, x_plan = fans[k]
    t_future = t_fire + np.arange(N + 1) * DT
    past = traj.t <= t_fire
    for x_path, w_path in zip(X_fan, w_fan / w_fan.max()):
        ax.plot(
            t_future, x_path[:, 0], color="tab:blue", alpha=0.1 + 0.6 * w_path, lw=0.8
        )
    ax.plot(t_future, x_plan[:, 0], color="black", lw=2, label="U after the update")
    ax.plot(
        traj.t[past], traj.x[0, past], color="grey", lw=2, label="closed loop so far"
    )
    for upright in (np.pi, -np.pi):
        ax.axhline(upright, color="tab:green", ls="--", lw=1)
    ax.set_title(f"t = {t_fire:.1f} s")
    ax.set_xlabel("t [s]")
axes[0].set_ylabel("theta [rad]")
axes[0].legend(loc="lower right", fontsize=8)
fig.suptitle(f"MPPI: {FAN_SAMPLES} of {K} sampled futures, shaded by weight")
fig.tight_layout()
plt.show()

plant.animate(traj)
