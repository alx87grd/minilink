"""Certify a learned policy: find its equilibrium, linearize it, and prove a region of attraction."""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np
from scipy.linalg import solve_continuous_lyapunov

from minilink import CostFunction, Pendulum
from minilink.control import angle_features
from minilink.planning import (
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

TORQUE = 4.0  # Nm, below m g l = 9.81 Nm: the pendulum must pump to get up
DT = 0.05
TRAINING_TIMESTEPS = 120_000

# The task: swing up and stay up, learned by PPO from starts all around the circle
plant = Pendulum()
plant.inputs["u"].lower_bound = np.array([-TORQUE])
plant.inputs["u"].upper_bound = np.array([TORQUE])
plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
plant.state.upper_bound = np.array([4 * np.pi, 20.0])


class SwingUpCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        return (1.0 + jnp.cos(theta)) + 0.01 * dtheta**2 + 0.01 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


problem = StochasticPlanningProblem(
    plant,
    cost=SwingUpCost(),
    tf=np.inf,
    x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
)
planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=angle_features(angles=[0], scales={1: 0.1}),
    hidden=(32, 32),
    n_envs=64,
    n_steps=32,
    batch_size=256,
    learning_rate=3e-3,
    gamma=0.97,
    verbose=0,
)
planner.solve(timesteps=TRAINING_TIMESTEPS, n_trials=32)
policy = planner.get_controller()
loop = policy @ plant  # a System like any other: the network is inside it now
loop.name = "Pendulum with the learned law"

# ---------------------------------------------------------------------------
# 1. Where does the learned law actually come to rest?
# ---------------------------------------------------------------------------
# Training rewarded being near the top; it never promised zero torque there.
X_UP = np.array([np.pi, 0.0])  # the upright we asked for
x_eq = loop.find_equilibrium(X_UP)  # a root find through the network
print(
    f"Torque the policy commands at the exact upright: {float(policy.action(X_UP)[0]):+.3f} Nm"
)
print(
    f"Equilibrium it really settles on:  theta = {x_eq[0]:.4f} rad, {x_eq[0] - np.pi:+.4f} from upright"
)

# ---------------------------------------------------------------------------
# 2. Linearize the closed loop: autodiff straight through the neural network
# ---------------------------------------------------------------------------
A = loop.linearize(x_eq).A()
poles = np.linalg.eigvals(A)
gain = np.asarray(
    jax.jacobian(policy.action)(jnp.asarray(x_eq))
)  # d(pi)/dx: the gain it learned
print(f"\nClosed-loop poles at that equilibrium: {np.round(poles, 3)}")
print(
    f"The network's local feedback gain:      {np.round(gain[0], 2)} (Nm/rad, Nm s/rad)"
)

# ---------------------------------------------------------------------------
# 3. A Lyapunov certificate for the nonlinear loop
# ---------------------------------------------------------------------------
# Solve A'P + PA = -I for the linearization, then use V(x) = (x - x_eq)' P (x - x_eq)
# as a candidate for the *nonlinear* closed loop. V decreases wherever
# Vdot = grad V . f_cl(x) < 0, and f_cl here is the real dynamics with the
# network inside, saturation and all.
P = solve_continuous_lyapunov(A.T, -np.eye(2))
evaluator = loop.compile(backend="jax", verbose=False)
u_none = jnp.zeros(0)  # the closed loop has no boundary input
P_jax, x_eq_jax = jnp.asarray(P), jnp.asarray(x_eq)


def V(x):
    d = x - x_eq_jax
    return d @ P_jax @ d


def V_dot(x):
    return jax.grad(V)(x) @ evaluator.f_trace(x, u_none, 0.0)


# Sweep the phase plane: the certified region is the largest sublevel set
# {V <= c} that contains no point where V stops decreasing. This is a *gridded*
# check, so it is a sharp estimate rather than a proof; a rigorous version
# bounds Vdot between the samples (interval arithmetic, or sum-of-squares on a
# polynomial surrogate of the network). Everything else here is exact.
theta = np.linspace(np.pi - 1.6, np.pi + 1.6, 301)
dtheta = np.linspace(-4.5, 4.5, 301)
TH, DTH = np.meshgrid(theta, dtheta)
grid = jnp.asarray(np.stack([TH.ravel(), DTH.ravel()], axis=1))

v = np.asarray(jax.jit(jax.vmap(V))(grid))
v_dot = np.asarray(jax.jit(jax.vmap(V_dot))(grid))
not_decreasing = (v_dot >= 0.0) & (v > 1e-4)  # the equilibrium itself has Vdot = 0
c = float(v[not_decreasing].min())
certified = v < c

# What stops it: the nearest failure sits on the torque limit
x_limit = np.asarray(grid)[int(np.argmin(np.where(not_decreasing, v, np.inf)))]
u_limit = float(policy.action(x_limit)[0])
theta_static = float(
    np.arcsin(
        TORQUE / (plant.params["m"] * plant.params["gravity"] * plant.params["l"])
    )
)
print(f"\nCertified level set: V <= {c:.4f}")
print(
    f"  angle error up to {np.abs(np.asarray(grid)[certified][:, 0] - x_eq[0]).max():.3f} rad, "
    f"rate up to {np.abs(np.asarray(grid)[certified][:, 1]).max():.2f} rad/s"
)
print(
    f"  it stops at a state where the policy commands {u_limit:+.2f} Nm "
    f"of the {TORQUE:.0f} available: the certificate ends where the actuator does"
)
print(
    f"  no fixed point can exist past {theta_static:.2f} rad of error with this torque "
    "(a moving pendulum can still swing through and be caught)"
)


# ---------------------------------------------------------------------------
# 4. How conservative is the certificate? Simulate the true basin.
# ---------------------------------------------------------------------------
@jax.jit
def converges(x0):
    def step(x, _):
        return evaluator.rk4_step_trace(x, u_none, 0.0, 0.02), None

    x_final = jax.lax.scan(step, x0, None, length=300)[0]  # 6 s
    return (jnp.abs(x_final[0] - x_eq[0]) < 0.05) & (jnp.abs(x_final[1]) < 0.2)


basin = np.asarray(jax.jit(jax.vmap(converges))(grid))
print(
    f"\nOf this window the simulated basin covers {100 * basin.mean():.0f}%, the certificate "
    f"{100 * certified.mean():.1f}% — the certificate holds {100 * certified.sum() / basin.sum():.0f}% of the basin"
)
print(f"Every certified state converges: {bool(np.all(basin[certified]))}")


# ---------------------------------------------------------------------------
# 5. One picture: what is proven, what is observed, what is impossible
# ---------------------------------------------------------------------------
def closed_loop_trajectory(x0, tf=6.0):
    plant.x0 = np.array(x0)
    return loop.compute_trajectory(tf=tf, dt=0.01, verbose=False)


# Two states at the same Lyapunov level, just outside the certified set: one
# converges, one does not. A quadratic V cannot tell them apart, which is
# exactly why the certified set has to stop where it does.
def state_at_level(mask, level):
    candidates = np.where(mask)[0]
    return np.asarray(grid)[candidates[np.argmin(np.abs(v[candidates] - level))]]


level = 4.0 * c
x_lucky = state_at_level(basin & ~certified, level)
x_lost = state_at_level(~basin, level)

fig, (wide, zoom) = plt.subplots(1, 2, figsize=(12.0, 5.0))

# Left: the whole neighbourhood of the top
wide.contourf(TH, DTH, basin.reshape(TH.shape), levels=[0.5, 1.5], colors=["#cfe3f7"])
wide.contour(TH, DTH, v.reshape(TH.shape), levels=[c], colors="tab:red", linewidths=2.0)
wide.contour(
    TH,
    DTH,
    v.reshape(TH.shape),
    levels=[level],
    colors="0.6",
    linewidths=0.8,
    linestyles="--",
)
for sign in (-1.0, 1.0):
    wide.axvline(np.pi + sign * theta_static, color="0.4", linestyle=":", linewidth=1.0)
for x0, label in ((x_lucky, "converges"), (x_lost, "does not")):
    traj = closed_loop_trajectory(x0)
    wide.plot(traj.x[0], traj.x[1], color="0.3", linewidth=1.0)
    wide.plot(*x0, "o", color="0.3", markersize=5)
    wide.annotate(
        label, x0, textcoords="offset points", xytext=(8, 8), fontsize=9, color="0.3"
    )
wide.plot(x_eq[0], x_eq[1], "k*", markersize=12)
wide.set_title("Simulated basin (blue), certified region (red), same $V$ (grey)")

# Right: the certificate itself, and the state that pins it
zoom.contour(TH, DTH, v.reshape(TH.shape), levels=[c], colors="tab:red", linewidths=2.0)
zoom.contour(
    TH,
    DTH,
    v_dot.reshape(TH.shape),
    levels=[0.0],
    colors="tab:orange",
    linewidths=1.2,
    linestyles="--",
)
for x0 in ([x_eq[0] + 0.12, 0.35], [x_eq[0] - 0.13, -0.30]):
    traj = closed_loop_trajectory(x0, tf=4.0)
    zoom.plot(traj.x[0], traj.x[1], color="tab:red", linewidth=1.2)
    zoom.plot(*x0, "o", color="tab:red", markersize=5)
zoom.plot(*x_limit, "s", color="tab:orange", markersize=7)
zoom.annotate(
    f"V stops decreasing here,\nwhere the policy saturates at {u_limit:+.1f} Nm",
    x_limit,
    textcoords="offset points",
    xytext=(10, -6),
    fontsize=8,
    color="tab:orange",
)
zoom.plot(x_eq[0], x_eq[1], "k*", markersize=12)
half = 1.35 * np.abs(np.asarray(grid)[certified][:, 0] - x_eq[0]).max()
zoom.set_xlim(x_eq[0] - half, x_eq[0] + half)
zoom.set_ylim(
    -1.35 * np.abs(np.asarray(grid)[certified][:, 1]).max(),
    1.35 * np.abs(np.asarray(grid)[certified][:, 1]).max(),
)
zoom.set_title(r"Certified set $\{V \leq c\}$ and the curve $\dot V = 0$")

for ax in (wide, zoom):
    ax.set_xlabel(r"$\theta$ [rad]")
    ax.set_ylabel(r"$\dot\theta$ [rad/s]")
wide.set_xlim(theta[0], theta[-1])
wide.set_ylim(dtheta[0], dtheta[-1])
plt.tight_layout()
plt.show()
