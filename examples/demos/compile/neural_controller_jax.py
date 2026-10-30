"""Train a neural controller over batched closed-loop rollouts."""

import jax
import jax.numpy as jnp
from jax.flatten_util import ravel_pytree

from minilink import DiagramSystem, Integrator, MathematicalProgram, Mux, Optimizer
from minilink.blocks import MLP

# Part 1: build a neural closed-loop controller
plant = Integrator()
mux = Mux(dims=(1, 1))
controller = MLP(input_dim=2, output_dim=1, hidden=(8,), seed=2)

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

plant.x0[0] = 2.0

evaluator = diagram.compile(backend="jax")
f = evaluator.f


# Part 2: train on several initial conditions and references
dt = 0.05
n_steps = 120
t_span = dt * jnp.arange(n_steps)
x0_batch = jnp.array([[-2.0], [-1.0], [0.0], [2.0], [-1.5], [1.5]])
r_batch = jnp.array([[0.0], [0.2], [0.5], [0.8], [-0.2], [1.0]])


def rk4_step(x, t, r, nn_params):
    # The full parameter tree, the network's branch replaced
    params = {**evaluator.params, "nn": nn_params}
    k1 = f(x, r, t, params)
    k2 = f(x + 0.5 * dt * k1, r, t + 0.5 * dt, params)
    k3 = f(x + 0.5 * dt * k2, r, t + 0.5 * dt, params)
    k4 = f(x + dt * k3, r, t + dt, params)

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


# The network's weights as one decision vector z, and the loss as its program
nn_params = {key: jnp.asarray(value) for key, value in controller.params.items()}
z0, weights_of = ravel_pytree(nn_params)
program = MathematicalProgram(
    n_z=z0.size, J=lambda z: batch_loss(weights_of(z)), backend="jax"
)
result = Optimizer(
    program, z0=z0, method="gd", iterations=10000, options={"learning_rate": 0.8}
).solve()
print(result)

assert result.cost < batch_loss(nn_params), "training did not reduce the loss"


# Part 3: simulate with the learned controller
learned_params = jax.device_get(weights_of(result.z))
diagram.params = {**diagram.params, "nn": learned_params}
diagram.compute_trajectory()
diagram.plot_trajectory()
