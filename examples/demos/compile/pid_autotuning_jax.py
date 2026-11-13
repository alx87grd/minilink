"""Autotune PID gains by differentiating a closed-loop JAX rollout."""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import (
    DiagramSystem,
    IntegralImpedance,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
)

# Rollout horizon (JAX loss and NumPy plot share dt and tf).
TF = 10.0
DT = 0.05
N_STEPS = 200

# Gradient descent on PID gains.
N_LEARN_STEPS = 1000
LR = 1.0

# Live trajectory plot during tuning.
PLOT_EVERY = 5
PLOT_PAUSE = 0.05
PLOT_BACKEND = "matplotlib"  # use "plotly" if matplotlib live updates stall

plant = Pendulum()
ctl = IntegralImpedance(Kp=1.0, Ki=0.0, Kd=1.0)
eye1 = jnp.eye(1)  # the gains are 1×1 matrices, the integral gain a 1-vector

diagram = DiagramSystem()
diagram.add_subsystem(plant, "plant")
diagram.add_subsystem(ctl, "controller")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "controller", "r")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("controller", "u", "plant", "u")
diagram.inputs["r"].nominal_value = np.array([1.0])

evaluator = diagram.compile(backend="jax")
theta_idx = diagram.state_index["plant"][0]
x0 = jnp.array(diagram.x0)
u_ref = jnp.array([1.0])
plant_params = plant.params
t_span = jnp.arange(N_STEPS) * DT

# The live plot: the closed loop at the current gains, run by the problem's evaluator
# (its cost the tracking error, Q on θ − r, as the loss below)
cost = QuadraticCost.from_system(
    plant, Q=np.diag([1.0, 0.0]), R=np.zeros((1, 1)), x_bar=np.array([1.0, 0.0])
)
pe = PlanningProblem(plant, cost=cost, horizon=TF).compile(diagram, DT)
live_plot = pe.live_plot(
    ["controller.Kp", "controller.Ki", "controller.Kd"],
    every=PLOT_EVERY,
    pause=PLOT_PAUSE,
    renderer=PLOT_BACKEND,
)


def rollout_loss(pid):
    params = {
        "controller": {
            **ctl.params,
            "Kp": pid[0] * eye1,
            "Ki": pid[1:2],
            "Kd": pid[2] * eye1,
        },
        "plant": plant_params,
    }

    def step(x, t):
        x_next = evaluator.step(x, u_ref, t, params, dt=DT)
        err = u_ref[0] - x_next[theta_idx]
        return x_next, err**2 + 1e-4 * jnp.sum(pid**2)

    _, costs = jax.lax.scan(step, x0, t_span)
    return jnp.mean(costs)


loss_and_grad = jax.jit(jax.value_and_grad(rollout_loss))
pid_params = jnp.array([1.0, 0.0, 1.0])

if PLOT_BACKEND == "matplotlib":
    plt.ion()

print(f"{'iter':>5} {'loss':>12} {'Kp':>8} {'Ki':>8} {'Kd':>8}")
for i in range(N_LEARN_STEPS):
    loss, grad = loss_and_grad(pid_params)
    if i % 20 == 0:
        print(
            f"{i:>5} {float(loss):>12.4f} {float(pid_params[0]):>8.3f} "
            f"{float(pid_params[1]):>8.3f} {float(pid_params[2]):>8.3f}"
        )

    live_plot(np.asarray(pid_params), loss, None)

    pid_params = pid_params - LR * grad

print(
    f"\nFinal PID: Kp={float(pid_params[0]):.3f}, "
    f"Ki={float(pid_params[1]):.3f}, Kd={float(pid_params[2]):.3f}, "
    f"loss={float(loss):.4f}"
)
if PLOT_BACKEND == "matplotlib":
    plt.ioff()
    plt.show()
