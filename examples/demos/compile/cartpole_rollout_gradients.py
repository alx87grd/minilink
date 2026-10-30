"""Differentiate through a CartPole rollout: U-only vs U+params co-design, solved by Optimizer."""

# Teaching twin: examples/teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb

import jax
import jax.numpy as jnp
import numpy as np

from minilink import CartPole, MathematicalProgram, Optimizer, Trajectory

# Demo controls.
ANIMATE = True

plant = CartPole()
evaluator = plant.compile(backend="jax")

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
        x_next = evaluator.step(x, jnp.array([u_scalar]), 0.0, p, dt=dt)
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


# A mild open-loop pump breaks the all-zero local basin for swing-up
pump = 0.5 * jnp.sin(2.0 * jnp.pi * jnp.arange(H) / H)

# (a) Optimize U only — physics fixed at catalog nominal


def cost_u(u_seq):
    x_final, _ = rollout(u_seq, p_nom)
    return task_cost(x_final, u_seq)


program_u = MathematicalProgram(
    n_z=H,
    J=cost_u,
    lower=np.full(H, -u_bound),
    upper=np.full(H, u_bound),
    backend="jax",
)
result_u = Optimizer(program_u, z0=pump, method="l-bfgs").solve()
print("(a) U-only:", result_u)

# (b) Co-optimize U and (lcg, m1, m2): the decision z = [u_0 … u_{H−1}, lcg, m1, m2]


def cost_co(z):
    u_seq, p_vec = z[:H], z[H:]
    x_final, _ = rollout(u_seq, p_vec)
    # soft design regularizer: prefer staying near nominal physics
    return task_cost(x_final, u_seq) + w_p * jnp.sum((p_vec - p_nom) ** 2)


program_co = MathematicalProgram(
    n_z=H + 3,
    J=cost_co,
    lower=np.concatenate([np.full(H, -u_bound), p_lo]),
    upper=np.concatenate([np.full(H, u_bound), p_hi]),
    backend="jax",
)
result_co = Optimizer(
    program_co, z0=jnp.concatenate([pump, p_nom]), method="l-bfgs"
).solve()
print("(b) co-opt:", result_co)

# The co-designed swing-up: the catalog plant on the found numbers, its input held
u_b, p_b = result_co.z[:H], result_co.z[H:]
plant.params.update(params_dict(p_b))
plant.pole_length = max(1.5, 6.0 * float(p_b[0]))  # the drawn rod follows lcg
x_b = evaluator.rollout(x0, u_b.reshape(-1, 1), 0.0, dt, plant.params)
traj_b = Trajectory(
    t=dt * np.arange(H + 1),
    x=np.asarray(x_b).T,
    u=np.append(u_b, u_b[-1]).reshape(1, -1),
)
plant.plot_trajectory(traj_b)
if ANIMATE:
    plant.animate(traj_b)
