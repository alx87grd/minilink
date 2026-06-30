"""Demo: autotune PID gains by differentiating a closed-loop JAX rollout."""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink.control.impedance import ImpedanceIntegralController
from minilink.core.backends import configure_jax
from minilink.core.diagram import DiagramSystem
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.signals import open_time_signal_plot, resolve_plot_signals

configure_jax(enable_x64=True)

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
ctl = ImpedanceIntegralController()
ctl.params = {"kp": 1.0, "ki": 0.0, "kd": 1.0}

diagram = DiagramSystem()
diagram.add_subsystem(plant, "plant")
diagram.add_subsystem(ctl, "ctl")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "ctl", "r")
diagram.connect("plant", "y", "ctl", "y")
diagram.connect("ctl", "u", "plant", "u")
diagram.inputs["r"].nominal_value = np.array([1.0])

evaluator = diagram.compile(backend="jax")
np_evaluator = diagram.compile(backend="numpy")
theta_idx = diagram.state_index["plant"][0]
x0 = jnp.array(diagram.x0)
u_ref = jnp.array([1.0])
plant_params = plant.params
t_span = jnp.arange(N_STEPS) * DT

plot_steps = int(TF / DT) + 1
plot_t = np.arange(plot_steps) * DT
plot_u_knots = np.broadcast_to(diagram.inputs["r"].nominal_value, (plot_steps, 1))
plot_signals = resolve_plot_signals(diagram)


def apply_pid_params(pid):
    diagram.params = {
        "ctl": {
            "kp": float(pid[0]),
            "ki": float(pid[1]),
            "kd": float(pid[2]),
        }
    }


def simulate_traj(pid):
    apply_pid_params(pid)
    xs = np_evaluator.rk4_rollout_forced(diagram.x0, plot_u_knots, 0.0, DT)
    return Trajectory(t=plot_t, x=xs.T, u=plot_u_knots.T)


def rollout_loss(pid):
    params = {
        "ctl": {"kp": pid[0], "ki": pid[1], "kd": pid[2]},
        "plant": plant_params,
    }

    def step(x, t):
        x_next = evaluator.rk4_step_p(x, u_ref, t, DT, params)
        err = u_ref[0] - x_next[theta_idx]
        return x_next, err**2 + 1e-4 * jnp.sum(pid**2)

    _, costs = jax.lax.scan(step, x0, t_span)
    return jnp.mean(costs)


loss_and_grad = jax.jit(jax.value_and_grad(rollout_loss))
pid_params = jnp.array([1.0, 0.0, 1.0])

plot_handle = None
if PLOT_BACKEND == "matplotlib":
    plt.ion()

print(f"{'iter':>5} {'loss':>12} {'kp':>8} {'ki':>8} {'kd':>8}")
for i in range(N_LEARN_STEPS):
    loss, grad = loss_and_grad(pid_params)
    if i % 20 == 0:
        print(
            f"{i:>5} {float(loss):>12.4f} {float(pid_params[0]):>8.3f} "
            f"{float(pid_params[1]):>8.3f} {float(pid_params[2]):>8.3f}"
        )

    if i % PLOT_EVERY == 0:
        traj = simulate_traj(pid_params)
        title = (
            f"iteration {i}  cost {float(loss):.4g}  "
            f"kp={float(pid_params[0]):.3f}  ki={float(pid_params[1]):.3f}  "
            f"kd={float(pid_params[2]):.3f}"
        )
        if plot_handle is None:
            plot_handle = open_time_signal_plot(
                diagram,
                traj,
                signals=plot_signals,
                backend=PLOT_BACKEND,
                show=True,
                pause=PLOT_PAUSE,
            )
        plot_handle.update(traj, title=title)
        if PLOT_BACKEND == "matplotlib":
            plt.pause(PLOT_PAUSE)

    pid_params = pid_params - LR * grad

print(
    f"\nFinal PID: kp={float(pid_params[0]):.3f}, "
    f"ki={float(pid_params[1]):.3f}, kd={float(pid_params[2]):.3f}, "
    f"loss={float(loss):.4f}"
)
if PLOT_BACKEND == "matplotlib":
    plt.ioff()
    plt.show()
