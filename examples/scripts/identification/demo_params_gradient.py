"""Gradients with respect to parameters: sensitivity and identification.

Part 1 — leaf system: compile a pendulum with the JAX backend and evaluate
``jacobian_f_params`` (∂f/∂θ as a pytree matching the params dict), validated
against central finite differences on ``f_p``.

Part 2 — diagram use case: close the loop around the pendulum, roll out the
"true" closed loop to collect samples, then identify the plant's gravity and
damping from data by gradient descent on the equation-error loss

    loss(θ) = mean_k ‖f_p(x_k, u_k, t_k, {"plant": θ}) − dx_k‖²

using ``jax.grad`` through the compiled diagram evaluator. The nested params
dict is partial: only the plant entry is supplied, the controller keeps its
live defaults.
"""

import jax
import jax.numpy as jnp
import numpy as np

from minilink.core.backends import array_module, configure_jax
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem

configure_jax(enable_x64=True)


# JAX-traceable blocks with proper params dicts
class TraceablePendulum(DynamicSystem):
    """Pendulum with all physical constants in ``params`` (JAX-traceable)."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2, y_dependencies=())
        self.name = "TraceablePendulum"
        self.params = {"m": 1.0, "l": 1.0, "gravity": 9.81, "d": 0.2}
        self.state.labels = ["theta", "dtheta"]

    def f(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        m = params["m"]
        l = params["l"]
        gravity = params["gravity"]
        d = params["d"]
        xp = array_module(x)
        q, dq = x[0], x[1]
        tau = u[0]

        # Pendulum about the pivot: (m l^2) ddq = tau - m g l sin(q) - d dq
        ddq = (tau - m * gravity * l * xp.sin(q) - d * dq) / (m * l**2)

        return xp.array([dq, ddq])

    def h(self, x, u, t=0, params=None):
        return x


class TraceablePDController(StaticSystem):
    """u = Kp (r − θ) − Kd dθ, with full-state measurement (JAX-traceable)."""

    def __init__(self):
        super().__init__()
        self.name = "TraceablePDController"
        self.params = {"Kp": 12.0, "Kd": 4.0}
        self.add_input_port("r", dim=1)
        self.add_input_port("y", dim=2)
        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        Kp = params["Kp"]
        Kd = params["Kd"]
        xp = array_module(u)
        r, y = self.get_port_values_from_u(u, "r", "y")

        # PD law on the angle with rate damping: tau = Kp (r - q) - Kd dq
        tau = Kp * (r[0] - y[0]) - Kd * y[1]

        return xp.array([tau])


# Part 1: leaf sensitivity ∂f/∂θ vs finite differences
print("=" * 70)
print("Part 1 — leaf pendulum: jacobian_f_params vs finite differences")
print("=" * 70)

plant = TraceablePendulum()
evaluator = plant.compile(backend="jax")

x = jnp.array([0.8, -0.3])
u = jnp.array([0.5])
t = 0.0
params = dict(plant.params)

jac = evaluator.jacobian_f_params(x, u, t, params)

eps = 1e-6
print(f"{'param':>10} {'autodiff d(ddq)/dθ':>22} {'finite diff':>22}")
for key in params:
    hi = dict(params)
    lo = dict(params)
    hi[key] = params[key] + eps
    lo[key] = params[key] - eps
    fd = (evaluator.f_p(x, u, t, hi) - evaluator.f_p(x, u, t, lo)) / (2 * eps)
    ad = np.asarray(jac[key])
    np.testing.assert_allclose(ad, np.asarray(fd), rtol=1e-5, atol=1e-7)
    print(f"{key:>10} {float(ad[1]):>22.6f} {float(fd[1]):>22.6f}")
print("Finite-difference check passed.")

# Part 2: closed-loop diagram, identify plant params from data
print()
print("=" * 70)
print("Part 2 — closed loop: identify gravity and damping by equation error")
print("=" * 70)

diagram = DiagramSystem()
diagram.connection_verbose = False
diagram.add_subsystem(TraceablePDController(), "ctl")
diagram.add_subsystem(TraceablePendulum(), "plant")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "ctl", "r")
diagram.connect("plant", "y", "ctl", "y")
diagram.connect("ctl", "u", "plant", "u")

true_evaluator = diagram.compile(backend="jax")

# Roll out the true closed loop under a sinusoidal reference to collect data.
dt = 0.02
n_samples = 400
ts = dt * jnp.arange(n_samples)
rs = 1.2 * jnp.sin(0.8 * ts).reshape(-1, 1)
x0 = jnp.array([0.5, 0.0])
xs = true_evaluator.rk4_rollout_forced(x0, rs, 0.0, dt)

# "Measured" state derivatives (in practice: numerical differentiation of
# logged states; here taken from the true model).
f_true = true_evaluator.get_f_jit()
dxs = jax.vmap(f_true, in_axes=(0, 0, 0))(xs, rs, ts)

# Equation-error identification: only the plant entry of the nested params
# dict is supplied; the controller keeps its live defaults.
f_p = true_evaluator.get_f_p_jit()


def loss(theta):
    plant_params = {"m": 1.0, "l": 1.0, "gravity": theta[0], "d": theta[1]}
    dx_hat = jax.vmap(f_p, in_axes=(0, 0, 0, None))(xs, rs, ts, {"plant": plant_params})
    return jnp.mean((dx_hat - dxs) ** 2)


loss_and_grad = jax.jit(jax.value_and_grad(loss))

theta = jnp.array([7.0, 1.0])  # initial guess: gravity=7.0, d=1.0
learning_rate = 2.0

print(f"{'iter':>6} {'loss':>14} {'gravity':>10} {'damping':>10}")
for i in range(151):
    value, grad = loss_and_grad(theta)
    if i % 25 == 0:
        print(
            f"{i:>6} {float(value):>14.3e} {float(theta[0]):>10.4f} "
            f"{float(theta[1]):>10.4f}"
        )
    theta = theta - learning_rate * grad

print()
print(f"Identified gravity = {float(theta[0]):.4f}  (true: 9.81)")
print(f"Identified damping = {float(theta[1]):.4f}  (true: 0.20)")

assert abs(float(theta[0]) - 9.81) < 1e-2, "gravity did not converge"
assert abs(float(theta[1]) - 0.20) < 1e-2, "damping did not converge"
print("Identification converged.")
