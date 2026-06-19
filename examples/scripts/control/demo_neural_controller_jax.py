"""Demo: Train a neural controller over batched closed-loop rollouts.

The neural network is a normal static block: weights live in ``params`` and the
closed-loop diagram supplies the controller input. JAX differentiates through a
small RK4 rollout and updates only the network parameters.
"""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink.blocks.basic import Integrator
from minilink.blocks.neural import NeuralNetwork
from minilink.blocks.routing import Mux
from minilink.core.backends import configure_jax
from minilink.core.diagram import DiagramSystem
from minilink.graphical.common.environment import is_blocking_needed
from minilink.graphical.common.matplotlib_style import DPI_FIGURE, FONT_SIZE

configure_jax(enable_x64=True)


print("=" * 70)
print("Part 1 - Build a neural closed-loop controller")
print("=" * 70)

plant = Integrator()
mux = Mux(dims=(1, 1))
controller = NeuralNetwork(input_dim=2, output_dim=1, hidden_dim=8, seed=2, scale=0.2)

diagram = DiagramSystem()
diagram.connection_verbose = False
diagram.add_subsystem(mux, "mux")
diagram.add_subsystem(controller, "nn")
diagram.add_subsystem(plant, "plant")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "mux", "in0")
diagram.connect("plant", "y", "mux", "in1")
diagram.connect("mux", "y", "nn", "u")
diagram.connect("nn", "y", "plant", "u")
diagram.connect_new_output_port("plant", "y", "y")

diagram.plot_diagram()

diagram.x0[0] = 2.0
diagram.compute_trajectory()
diagram.plot_trajectory()

evaluator = diagram.compile(backend="jax")
f_p = evaluator.get_f_p_jit()

print("Compiled diagram with state order:", list(diagram.subsystems.keys()))


print("\n" + "=" * 70)
print("Part 2 - Train on several initial conditions and references")
print("=" * 70)

dt = 0.05
n_steps = 120
t_span = dt * jnp.arange(n_steps)
x0_batch = jnp.array([[-2.0], [-1.0], [0.0], [2.0], [-1.5], [1.5]])
r_batch = jnp.array([[0.0], [0.2], [0.5], [0.8], [-0.2], [1.0]])


def rk4_step(x, t, r, nn_params):
    params = {"nn": nn_params}
    k1 = f_p(x, r, t, params)
    k2 = f_p(x + 0.5 * dt * k1, r, t + 0.5 * dt, params)
    k3 = f_p(x + 0.5 * dt * k2, r, t + 0.5 * dt, params)
    k4 = f_p(x + dt * k3, r, t + dt, params)

    return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


def rollout_loss(x0, r, nn_params):
    def step(x, t):
        x_next = rk4_step(x, t, r, nn_params)
        e = x_next[0] - r[0]
        return x_next, e**2

    _, costs = jax.lax.scan(step, x0, t_span)
    return jnp.mean(costs)


def l2_norm(params):
    leaves = jax.tree_util.tree_leaves(params)
    return sum(jnp.sum(leaf**2) for leaf in leaves)


def batch_loss(nn_params):
    losses = jax.vmap(lambda x0, r: rollout_loss(x0, r, nn_params))(x0_batch, r_batch)
    return jnp.mean(losses) + 1e-8 * l2_norm(nn_params)


loss_and_grad = jax.jit(jax.value_and_grad(batch_loss))
nn_params = {key: jnp.asarray(value) for key, value in controller.params.items()}
initial_params = jax.tree_util.tree_map(lambda param: param.copy(), nn_params)

learning_rate = 0.8

initial_loss = batch_loss(nn_params)

print(f"{'iter':>6} {'loss':>12}")
for i in range(10000):
    value, grad = loss_and_grad(nn_params)
    if i % 20 == 0:
        print(f"{i:>6} {float(value):>12.5f}")
    nn_params = jax.tree_util.tree_map(
        lambda param, dparam: param - learning_rate * dparam,
        nn_params,
        grad,
    )

final_loss = batch_loss(nn_params)
learned_params = {key: jax.device_get(value) for key, value in nn_params.items()}

print()
print(f"Initial batch loss: {float(initial_loss):.5f}")
print(f"Final batch loss:   {float(final_loss):.5f}")
print(
    "Learned controller output at [r=1, y=0]:",
    controller.compute([], jnp.array([1.0, 0.0]), params=learned_params),
)

assert final_loss < initial_loss, "training did not reduce the batched rollout loss"


print("\n" + "=" * 70)
print("Part 3 - Control-law heatmap u = f(r, y)")
print("=" * 70)

R_MIN, R_MAX = -0.5, 1.0
Y_MIN, Y_MAX = -3.0, 3.0
N_GRID = 100
r_vals = jnp.linspace(R_MIN, R_MAX, N_GRID)
y_vals = jnp.linspace(Y_MIN, Y_MAX, N_GRID)


@jax.jit
def control_surface(r_grid, y_grid, nn_params):
    def row(y_val):
        def cell(r_val):
            return controller.compute([], jnp.array([r_val, y_val]), params=nn_params)[0]

        return jax.vmap(cell)(r_grid)

    return jax.vmap(row)(y_grid)


def plot_control_law_heatmaps(before_params, after_params):
    u_before = np.asarray(control_surface(r_vals, y_vals, before_params))
    u_after = np.asarray(control_surface(r_vals, y_vals, after_params))
    u_lim = max(
        float(np.max(np.abs(u_before))),
        float(np.max(np.abs(u_after))),
        1e-6,
    )

    fig, axes = plt.subplots(
        1,
        2,
        figsize=(10.0, 4.2),
        dpi=DPI_FIGURE,
        sharey=True,
        constrained_layout=True,
    )
    manager = getattr(fig.canvas, "manager", None)
    set_window_title = getattr(manager, "set_window_title", None)
    if callable(set_window_title):
        set_window_title("Neural controller: u = f(r, y)")

    for ax, u_map, title in zip(
        axes,
        (u_before, u_after),
        ("Before training", "After training"),
    ):
        mesh = ax.pcolormesh(
            np.asarray(r_vals),
            np.asarray(y_vals),
            u_map,
            shading="auto",
            cmap="RdBu_r",
            vmin=-u_lim,
            vmax=u_lim,
        )
        for r_train in np.unique(np.asarray(r_batch)):
            ax.axvline(float(r_train), color="k", ls="--", lw=0.8, alpha=0.7)
        ax.set_xlabel(r"$r$ (reference)", fontsize=FONT_SIZE)
        ax.set_title(title, fontsize=FONT_SIZE)
        fig.colorbar(mesh, ax=ax, label=r"$u$")

    axes[0].set_ylabel(r"$y$ (plant output)", fontsize=FONT_SIZE)
    plt.show(block=is_blocking_needed())


plot_control_law_heatmaps(initial_params, nn_params)


print("\n" + "=" * 70)
print("Part 4 - Simulate with the learned controller")
print("=" * 70)

diagram.params = {"nn": learned_params}
diagram.compute_trajectory()
diagram.plot_trajectory()
