"""Differentiate through a CartPole rollout: U-only vs U+params co-design."""

# Teaching twin: examples/teaching/optimal_control/cartpole_rollout_gradients.ipynb

import jax
import jax.numpy as jnp
import numpy as np

from minilink import CartPole, Trajectory

# Demo controls.
ANIMATE = True

plant = CartPole()
evaluator = plant.compile(backend="jax")
rk4_step_p = evaluator.rk4_step_p

# Shared swing-up task (same horizon / cost for a fair (a) vs (b) comparison).
H = 50
dt = 0.05  # 5.0 s
x0 = jnp.array([0.0, 0.05, 0.0, 0.0])  # hanging, tiny tip to break symmetry
theta_target = 1.0  # float(np.pi)  # upright
# p = (lcg, m1, m2); gravity fixed
p_nom = jnp.array([0.5, 1.0, 0.1])
p_lo = jnp.array([0.2, 0.4, 0.05])
p_hi = jnp.array([2.5, 3.0, 1.0])
w_p = 1e-2  # soft preference for staying near nominal physics in (b)
u_bound = 10.0


def params_dict(p_vec):
    return {
        "lcg": p_vec[0],
        "m1": p_vec[1],
        "m2": p_vec[2],
        "gravity": 9.81,
    }


def rollout(u_seq, p_vec):
    p = params_dict(p_vec)

    def step(x, u_scalar):
        x_next = rk4_step_p(x, jnp.array([u_scalar]), 0.0, dt, p)
        return x_next, x_next

    x_final, traj = jax.lax.scan(step, x0, u_seq)
    return x_final, traj


def task_cost(x_final, u_seq):
    # J_task = 5 (θ_N - π)² + light cart / rate / effort penalties
    return (
        5.0 * (x_final[1] - theta_target) ** 2
        + 0.2 * x_final[0] ** 2
        + 0.5 * jnp.sum(x_final[2:] ** 2)
        + 2e-4 * jnp.sum(u_seq**2)
    )


def print_result(tag, x_final, u_seq, p_vec, J):
    u_rms = float(np.sqrt(np.mean(np.asarray(u_seq) ** 2)))
    p = np.asarray(p_vec)
    print(
        f"{tag}:  J = {J:.3e}   theta_N = {float(x_final[1]):.3f} "
        f"(target {theta_target:.3f})   u_rms = {u_rms:.3f} N   "
        f"p = (lcg {p[0]:.3f} m, m1 {p[1]:.3f} kg, m2 {p[2]:.3f} kg)"
    )


def make_trajectory(x0_vec, states, u_seq, dt_step):
    """Wrap a JAX scan rollout as a minilink ``Trajectory`` (includes x0)."""
    x = np.vstack(
        [np.asarray(x0_vec, dtype=float), np.asarray(states, dtype=float)]
    )  # (H+1, n)
    u = np.asarray(u_seq, dtype=float).reshape(-1, 1)
    u = np.vstack([u, u[-1:]])  # hold last force at tf → shape (H+1, m)
    t_grid = dt_step * np.arange(x.shape[0])
    return Trajectory(t=t_grid, x=x.T, u=u.T)


def plant_from_params(p_vec, name):
    """Catalog plant with EoM params applied (ready to animate a traj)."""
    sys = CartPole()
    sys.name = name
    for key, val in params_dict(p_vec).items():
        sys.params[key] = float(val)
    # Scale the graphic rod with CoM length so co-design is visible.
    sys.pole_length = max(1.5, 6.0 * float(p_vec[0]))
    return sys


def pump_init():
    # mild open-loop pump — breaks the all-zero local basin for swing-up
    return 0.5 * jnp.sin(2.0 * jnp.pi * jnp.arange(H) / H)


# ---------------------------------------------------------------------------
# (a) Optimize U only — physics fixed at catalog nominal
# ---------------------------------------------------------------------------


def cost_u(u_seq):
    x_final, _ = rollout(u_seq, p_nom)
    return task_cost(x_final, u_seq)


value_and_grad_u = jax.jit(jax.value_and_grad(cost_u))
u_a = pump_init()
for lr in (0.4, 0.2, 0.1, 0.05):
    for _ in range(1200):
        _, g = value_and_grad_u(u_a)
        u_a = jnp.clip(u_a - lr * g, -u_bound, u_bound)

x_final_a, traj_a = rollout(u_a, p_nom)
J_a = float(cost_u(u_a))
print_result("(a) U-only", x_final_a, u_a, p_nom, J_a)

# The rollout as a minilink Trajectory: the catalog plant plots and animates it
traj_a_ml = make_trajectory(x0, traj_a, u_a, dt)
sys_a = plant_from_params(p_nom, "CartPole (a) U-only")
sys_a.plot_trajectory(traj_a_ml)
if ANIMATE:
    sys_a.animate(traj_a_ml)


# ---------------------------------------------------------------------------
# (b) Co-optimize U and (lcg, m1, m2)
# ---------------------------------------------------------------------------


def cost_co(decision):
    u_seq, p_vec = decision
    x_final, _ = rollout(u_seq, p_vec)
    # soft design regularizer: prefer staying near nominal physics
    return task_cost(x_final, u_seq) + w_p * jnp.sum((p_vec - p_nom) ** 2)


value_and_grad_co = jax.jit(jax.value_and_grad(cost_co))
u_b = pump_init()
p_b = p_nom
for lr_u, lr_p in ((0.4, 0.03), (0.2, 0.015), (0.1, 0.008), (0.05, 0.003)):
    for _ in range(1200):
        _, (g_u, g_p) = value_and_grad_co((u_b, p_b))
        u_b = jnp.clip(u_b - lr_u * g_u, -u_bound, u_bound)
        p_b = jnp.clip(p_b - lr_p * g_p, p_lo, p_hi)

x_final_b, traj_b = rollout(u_b, p_b)
J_b = float(cost_co((u_b, p_b)))
print_result("(b) co-opt", x_final_b, u_b, p_b, J_b)
print(f"(b) task cost without the regularizer: {float(task_cost(x_final_b, u_b)):.3e}")

traj_b_ml = make_trajectory(x0, traj_b, u_b, dt)
sys_b = plant_from_params(p_b, "CartPole (b) co-opt")
sys_b.plot_trajectory(traj_b_ml)
if ANIMATE:
    sys_b.animate(traj_b_ml)
