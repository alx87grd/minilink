"""Demo: Autotuning PID parameters using JAX.

This script demonstrates how to autotune the parameters of a PID controller
by differentiating the integral of the closed-loop tracking error with
respect to the controller's parameters.

A JAX-compiled diagram is rolled out over a time horizon using a custom
RK4 scan loop, and ``jax.grad`` computes the gradients of the rollout loss
directly through the solver steps.
"""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink.control.linear import PIDController
from minilink.core.backends import configure_jax
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

configure_jax(enable_x64=True)


print("=" * 70)
print("Part 1 — Setup closed-loop diagram and compile to JAX")
print("=" * 70)

plant = Pendulum()
ctl = PIDController()
# Initialize PID params to something stable but suboptimal
ctl.params = {"kp": 1.0, "ki": 0.0, "kd": 1.0}

diagram = DiagramSystem()
diagram.add_subsystem(ctl, "ctl")
diagram.add_subsystem(plant, "plant")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "ctl", "r")
diagram.connect("plant", "y", "ctl", "y")
diagram.connect("ctl", "u", "plant", "u")
diagram.inputs["r"].nominal_value = np.array([1.0])

diagram.plot_diagram(show_pdf=False, show_inline=False)
diagram.compute_trajectory(tf=5.0, show=False)
diagram.plot_trajectory(signals=("plant:x"))

evaluator = diagram.compile(backend="jax")
f_p = evaluator.get_f_p_jit()

print("Compiled successfully. Diagram state order:", list(diagram.subsystems.keys()))


print("\n" + "=" * 70)
print("Part 2 — Define rollout loss and compute gradients")
print("=" * 70)

dt = 0.05
n_steps = 100
t_span = jnp.arange(n_steps) * dt
u_ref = jnp.array([1.0])  # Step reference to angle 1.0 rad


def rk4_step(x, u, t, params):
    k1 = f_p(x, u, t, params)
    k2 = f_p(x + dt / 2.0 * k1, u, t + dt / 2.0, params)
    k3 = f_p(x + dt / 2.0 * k2, u, t + dt / 2.0, params)
    k4 = f_p(x + dt * k3, u, t + dt, params)
    return x + dt / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


plant_params = plant.params


def rollout_loss(pid_params_array):
    """
    Simulates the closed-loop system and computes the integral tracking error.
    pid_params_array: [kp, ki, kd]
    """
    params = {
        "ctl": {
            "kp": pid_params_array[0],
            "ki": pid_params_array[1],
            "kd": pid_params_array[2],
        },
        "plant": plant_params,
    }

    # State is [e_int, q, dq] because diagram.subsystems is ['ctl', 'plant']
    x0 = jnp.array([0.0, 0.0, 0.0])

    def step_fn(x, t):
        x_next = rk4_step(x, u_ref, t, params)
        q = x_next[1]

        # Loss: quadratic tracking error + small regularization on gains
        error = u_ref[0] - q
        cost = error**2 + 1e-4 * jnp.sum(pid_params_array**2)

        return x_next, cost

    _, costs = jax.lax.scan(step_fn, x0, t_span)
    return jnp.mean(costs)


loss_and_grad = jax.jit(jax.value_and_grad(rollout_loss))

# Initial guess (matches ctl.params above)
pid_params = jnp.array([1.0, 0.0, 1.0])
lr = 10.0  # Learning rate


def evaluate_and_plot(iteration, current_pid_params):
    diagram.inputs["r"].nominal_value = np.array([1.0])
    diagram.params = {
        "ctl": {
            "kp": float(current_pid_params[0]),
            "ki": float(current_pid_params[1]),
            "kd": float(current_pid_params[2]),
        }
    }
    # Simulate using the standard NumPy evaluator (since we just need the forward pass)
    diagram.compute_trajectory(tf=5.0)
    diagram.plot_trajectory(signals=("plant:x"))
    print(
        f"Iteration {iteration}: Kp={float(current_pid_params[0]):.1f}, "
        f"Ki={float(current_pid_params[1]):.1f}, Kd={float(current_pid_params[2]):.1f}"
    )


print(f"{'Iter':>5} {'Loss':>12} {'Kp':>8} {'Ki':>8} {'Kd':>8}")

# Plot initial performance
evaluate_and_plot(0, pid_params)

for i in range(101):
    loss, grad = loss_and_grad(pid_params)
    if i % 20 == 0:
        print(
            f"{i:>5} {float(loss):>12.4f} {float(pid_params[0]):>8.3f} {float(pid_params[1]):>8.3f} {float(pid_params[2]):>8.3f}"
        )
    pid_params -= lr * grad
    if i > 0 and i % 5 == 0:
        evaluate_and_plot(i, pid_params)

print("\nAutotuning converged.")
print(
    f"Final PID parameters: Kp={float(pid_params[0]):.3f}, Ki={float(pid_params[1]):.3f}, Kd={float(pid_params[2]):.3f}"
)

# Plot final performance and block
evaluate_and_plot("Final", pid_params)
plt.show()
