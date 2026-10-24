"""Autotune PID gains: gradient descent by hand on a JAX rollout, then the same tuning through the planning pipeline."""

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import (
    DiagramSystem,
    IntegralImpedance,
    Optimizer,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Trajectory,
)
from minilink.graphical.signals import open_time_signal_plot, resolve_plot_signals

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

# The tuning cost, one for both sections: the mean over the run of the squared
# tracking error and of the effort above the torque that holds the setpoint,
#   J = (1/T) ∫₀ᵀ ((r − θ)² + ρ (u − u_ss)²) dt,   u_ss = m g l sin r
R_REF = 1.0  # setpoint, rad
RHO = 0.01  # effort weight

plant = Pendulum()
ctl = IntegralImpedance(Kp=1.0, Ki=0.0, Kd=1.0)
I = jnp.eye(1)  # the gains are 1×1 matrices, the integral gain a 1-vector
U_SS = plant.params["m"] * plant.params["gravity"] * plant.params["l"] * np.sin(R_REF)

diagram = DiagramSystem()
diagram.add_subsystem(plant, "plant")
diagram.add_subsystem(ctl, "controller")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "controller", "r")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("controller", "u", "plant", "u")
diagram.inputs["r"].nominal_value = np.array([R_REF])

# =============================================================================
# 1. By hand: the rollout, the cost and the gradient-descent loop written out
# =============================================================================

evaluator = diagram.compile(backend="jax")
np_evaluator = diagram.compile(backend="numpy")
theta_idx = diagram.state_index["plant"][0]
u_slice = evaluator.plan.output_slices[("controller", "u")]
x0 = jnp.array(diagram.x0)
u_ref = jnp.array([R_REF])
u_seq = jnp.tile(u_ref, (N_STEPS, 1))
t_grid = jnp.arange(N_STEPS + 1) * DT
plant_params = plant.params

plot_steps = int(TF / DT) + 1
plot_t = np.arange(plot_steps) * DT
plot_u_knots = np.broadcast_to(diagram.inputs["r"].nominal_value, (plot_steps, 1))
plot_signals = resolve_plot_signals(diagram)


def apply_pid_params(pid):
    diagram.params = {
        "controller": {
            **ctl.params,
            "Kp": float(pid[0]) * np.eye(1),
            "Ki": np.array([float(pid[1])]),
            "Kd": float(pid[2]) * np.eye(1),
        }
    }


def simulate_traj(pid):
    apply_pid_params(pid)
    xs = np_evaluator.jit.rollout(diagram.x0, plot_u_knots, 0.0, DT, hold="foh")
    return Trajectory(t=plot_t, x=xs.T, u=plot_u_knots.T)


def rollout_loss(pid):
    params = {
        "controller": {
            **ctl.params,
            "Kp": pid[0] * I,
            "Ki": pid[1:2],
            "Kd": pid[2] * I,
        },
        "plant": plant_params,
    }

    # The run: x_{k+1} = Φ(x_k, r, t_k), and the command u_k the controller outputs
    xs = evaluator.jit.rollout(x0, u_seq, 0.0, DT, params)
    us = jax.vmap(lambda x, t: evaluator.signals(x, u_ref, t, params)[u_slice])(
        xs, t_grid
    )

    # J = (1/T) ∫₀ᵀ ((r − θ)² + ρ (u − u_ss)²) dt, by the trapezoid on the samples
    g = (u_ref[0] - xs[:, theta_idx]) ** 2 + RHO * (us[:, 0] - U_SS) ** 2
    J = jnp.trapezoid(g, dx=DT) / TF

    return J


loss_and_grad = jax.jit(jax.value_and_grad(rollout_loss))
pid_start = jnp.array([1.0, 0.0, 1.0])
pid_params = pid_start

plot_handle = None
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

    if i % PLOT_EVERY == 0:
        traj = simulate_traj(pid_params)
        title = (
            f"iteration {i}  cost {float(loss):.4g}  "
            f"Kp={float(pid_params[0]):.3f}  Ki={float(pid_params[1]):.3f}  "
            f"Kd={float(pid_params[2]):.3f}"
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
    f"\nBy hand: Kp={float(pid_params[0]):.3f}, "
    f"Ki={float(pid_params[1]):.3f}, Kd={float(pid_params[2]):.3f}, "
    f"loss={float(rollout_loss(pid_params)):.4f}"
)
if PLOT_BACKEND == "matplotlib":
    plt.ioff()
    plt.show()

# =============================================================================
# 2. The pipeline: the problem stated, compiled, exported, solved
# =============================================================================

# The task: the same cost as a quadratic on the plant, Q on θ − r, R on u − u_ss
cost = QuadraticCost.from_system(
    plant,
    Q=np.diag([1.0 / TF, 0.0]),
    R=RHO / TF * np.eye(1),
    xbar=np.array([R_REF, 0.0]),
    ubar=np.array([U_SS]),
)
problem = PlanningProblem(plant, cost=cost, tf=TF)

# The run compiled once: every parameter of the loop is a leaf of pe.params
pe = problem.compile(diagram, DT)
wrt = ["controller.Kp", "controller.Ki", "controller.Kd"]
start = pe.unpack(np.asarray(pid_start), wrt)

# The program: z = (Kp, Ki, Kd), J(z) the run's cost, ∇J by reverse mode
program = pe.program(wrt, params=start)

# Gradient descent on the program: the hand loop's step, from the same start
descent = Optimizer(
    program,
    method="gd",
    options={"learning_rate": LR, "maxiter": N_LEARN_STEPS},
).solve()
print(
    f"Pipeline, gd: Kp={descent.z[0]:.3f}, Ki={descent.z[1]:.3f}, "
    f"Kd={descent.z[2]:.3f}, J={descent.cost:.4f}  "
    f"(largest gap to the hand loop: {np.max(np.abs(descent.z - pid_params)):.1e})"
)

# The class default (L-BFGS for an unconstrained program) finishes the descent
best = Optimizer(program, method="auto").solve()
print(
    f"Pipeline, {best.stats.get('nit', '?')} L-BFGS iterations: "
    f"Kp={best.z[0]:.3f}, Ki={best.z[1]:.3f}, Kd={best.z[2]:.3f}, J={best.cost:.4f}"
)

# The tuned gains written back, and judged by the same evaluation
tuned = pe.unpack(best.z, wrt, start)
print(pe.evaluate(tuned))

# The two tunings on the plant: the angle and the torque
fig, (ax_theta, ax_u) = plt.subplots(2, 1, sharex=True, figsize=(7, 5))
for label, gains in (("gradient descent", descent.z), ("L-BFGS", best.z)):
    run = pe.trajectory(pe.unpack(gains, wrt, start)).of("plant")
    ax_theta.plot(run.t, run.x[0], label=label)
    ax_u.plot(run.t, run.u[0], label=label)
ax_theta.axhline(R_REF, color="k", ls="--", lw=0.8)
ax_theta.set_ylabel("theta [rad]")
ax_u.axhline(U_SS, color="k", ls="--", lw=0.8)
ax_u.set_ylabel("u [Nm]")
ax_u.set_xlabel("t [s]")
ax_theta.legend()
fig.suptitle("PID tuned by the pipeline")
fig.tight_layout()
plt.show()
