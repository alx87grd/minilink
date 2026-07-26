"""Differentiate through a CartPole rollout: U-only vs U+params co-design.

Teaching twin (math-first):
``examples/notebooks/applications/cartpole_rollout_gradients.ipynb``.
Related: ``examples/notebooks/showcase/jax.ipynb`` §7.

Full swing-up on catalog ``JaxCartPole`` (hanging → upright, θ★ = π):

  (a) Optimize the force sequence U with physics p held at the catalog nominal.
  (b) Co-optimize U together with (lcg, m1, m2) — design the plant while planning.

Each case: custom result plots, then one ``sys.animate(Trajectory)``.

Run from the repo root::

    python examples/scripts/trajopt/trajopt_cartpole_rollout_gradients.py
"""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink.core.backends import configure_jax
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole

# Demo controls.
ANIMATE = True

configure_jax(enable_x64=True)

plant = JaxCartPole()
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
    u_np = np.asarray(u_seq)
    print(f"J({tag})               = {J:.3e}")
    print(f"target  final θ     : {theta_target:.3f} rad  (π = upright)")
    print(f"reached final θ     : {float(x_final[1]):.3f} rad")
    print(f"angle error         : {float(x_final[1]) - theta_target:+.3e} rad")
    print(
        f"final state x_N     : "
        f"x={float(x_final[0]):+.3f} m, "
        f"θ={float(x_final[1]):+.3f} rad, "
        f"dx={float(x_final[2]):+.3f}, "
        f"dθ={float(x_final[3]):+.3f}"
    )
    print(
        f"u*  min / max / rms : "
        f"{u_np.min():+.3f} / {u_np.max():+.3f} / "
        f"{np.sqrt(np.mean(u_np**2)):.3f} N"
    )
    print(
        f"params (lcg, m1, m2): "
        f"{float(p_vec[0]):.3f} m, {float(p_vec[1]):.3f} kg, {float(p_vec[2]):.3f} kg"
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
    sys = JaxCartPole()
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
print("=" * 70)
print("(a) Optimize U only — full swing-up (physics fixed at nominal p)")
print("=" * 70)


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
traj_a = np.asarray(traj_a)
u_a_np = np.asarray(u_a)
t = dt * np.arange(H)
J_a = float(cost_u(u_a))
print_result("U*", x_final_a, u_a, p_nom, J_a)

fig, ax = plt.subplots(1, 2, figsize=(10, 3.2))
ax[0].plot(t, traj_a[:, 0], label=r"$x(t)$ [m]")
ax[0].plot(t, traj_a[:, 1], label=r"$\theta(t)$ [rad]")
ax[0].axhline(theta_target, ls="--", color="k", lw=0.8, label=r"$\theta^\star=\pi$")
ax[0].set_xlabel("time [s]")
ax[0].set_title("(a) U-only swing-up trajectory")
ax[0].legend()
ax[1].step(t, u_a_np, where="post", color="C1", label=r"$U^\star$ (a)")
ax[1].axhline(0.0, color="k", lw=0.5)
ax[1].set_xlabel("time [s]")
ax[1].set_ylabel("force [N]")
ax[1].set_title(r"(a) optimized force sequence")
ax[1].legend()
fig.tight_layout()
plt.show()

traj_a_ml = make_trajectory(x0, traj_a, u_a, dt)
sys_a = plant_from_params(p_nom, "CartPole (a) U-only")
if ANIMATE:
    sys_a.animate(traj_a_ml)


# ---------------------------------------------------------------------------
# (b) Co-optimize U and (lcg, m1, m2)
# ---------------------------------------------------------------------------
print()
print("=" * 70)
print("(b) Co-optimize U and params (lcg, m1, m2) — full swing-up")
print("=" * 70)


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
traj_b = np.asarray(traj_b)
u_b_np = np.asarray(u_b)
J_b = float(cost_co((u_b, p_b)))
J_b_task = float(task_cost(x_final_b, u_b))
print_result("U*,p*", x_final_b, u_b, p_b, J_b)
print(f"task cost (no reg)  : {J_b_task:.3e}")
print(
    f"Δparams vs nominal  : "
    f"Δlcg={float(p_b[0] - p_nom[0]):+.3f} m, "
    f"Δm1={float(p_b[1] - p_nom[1]):+.3f} kg, "
    f"Δm2={float(p_b[2] - p_nom[2]):+.3f} kg"
)

fig, ax = plt.subplots(1, 3, figsize=(12, 3.2))
ax[0].plot(t, traj_a[:, 1], ls=":", color="C0", label=r"(a) $\theta(t)$")
ax[0].plot(t, traj_b[:, 1], color="C0", label=r"(b) $\theta(t)$")
ax[0].axhline(theta_target, ls="--", color="k", lw=0.8, label=r"$\theta^\star=\pi$")
ax[0].set_xlabel("time [s]")
ax[0].set_ylabel("theta [rad]")
ax[0].set_title("(a) vs (b) pole angle")
ax[0].legend()
ax[1].step(t, u_a_np, where="post", ls=":", color="C1", label=r"(a) $U^\star$")
ax[1].step(t, u_b_np, where="post", color="C1", label=r"(b) $U^\star$")
ax[1].axhline(0.0, color="k", lw=0.5)
ax[1].set_xlabel("time [s]")
ax[1].set_ylabel("force [N]")
ax[1].set_title("(a) vs (b) force sequence")
ax[1].legend()
labels = [r"$\ell_{\mathrm{cg}}$", r"$m_1$", r"$m_2$"]
x_bar = np.arange(3)
ax[2].bar(x_bar - 0.15, np.asarray(p_nom), width=0.3, label="nominal", color="0.7")
ax[2].bar(x_bar + 0.15, np.asarray(p_b), width=0.3, label=r"$p^\star$ (b)", color="C2")
ax[2].set_xticks(x_bar, labels)
ax[2].set_title("(b) co-designed params")
ax[2].legend()
fig.tight_layout()
plt.show()

traj_b_ml = make_trajectory(x0, traj_b, u_b, dt)
sys_b = plant_from_params(p_b, "CartPole (b) co-opt")
if ANIMATE:
    sys_b.animate(traj_b_ml)

# Side-by-side summary
print()
print("=" * 70)
print("Comparison (same full swing-up task)")
print("=" * 70)
print(f"{'':22} {'(a) U-only':>14} {'(b) co-opt':>14}")
print(f"{'J':22} {J_a:14.3e} {J_b:14.3e}")
print(f"{'task cost':22} {J_a:14.3e} {J_b_task:14.3e}")
print(
    f"{'u rms [N]':22} "
    f"{np.sqrt(np.mean(u_a_np**2)):14.3f} "
    f"{np.sqrt(np.mean(u_b_np**2)):14.3f}"
)
print(
    f"{'|θ_N - π| [rad]':22} "
    f"{abs(float(x_final_a[1]) - theta_target):14.3e} "
    f"{abs(float(x_final_b[1]) - theta_target):14.3e}"
)
