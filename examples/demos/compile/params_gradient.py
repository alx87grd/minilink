"""Gradients with respect to parameters: sensitivity and identification."""

import jax
import jax.numpy as jnp
import numpy as np

from minilink import (
    DiagramSystem,
    DynamicSystem,
    MathematicalProgram,
    Optimizer,
    System,
)
from minilink.core.backends import array_module


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


class TraceableImpedanceController(System):
    """u = Kp (r − θ) − Kd dθ, with full-state measurement (JAX-traceable)."""

    def __init__(self):
        super().__init__()
        self.name = "TraceableImpedanceController"
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


# Part 1: leaf sensitivity df/dparams, exact (autodiff) vs finite differences

plant = TraceablePendulum()
x = np.array([0.8, -0.3])
u = np.array([0.5])

exact = plant.jacobian("f", "params", x, u)  # JAX autodiff (the plant traces)
fd = plant.jacobian("f", "params", x, u, method="fd")

print(f"{'param':>10} {'autodiff d(ddq)/dθ':>22} {'finite diff':>22}")
for key in plant.params:
    np.testing.assert_allclose(exact[key], fd[key], rtol=1e-5, atol=1e-7)
    print(f"{key:>10} {exact[key][1]:>22.6f} {fd[key][1]:>22.6f}")
print("Finite-difference check passed.")

# Part 2: closed-loop diagram, identify gravity and damping by equation error

diagram = DiagramSystem()
diagram.connection_verbose = False
diagram.add_subsystem(TraceableImpedanceController(), "controller")
diagram.add_subsystem(TraceablePendulum(), "plant")
diagram.add_input_port("r", dim=1)
diagram.connect("input", "r", "controller", "r")
diagram.connect("plant", "y", "controller", "y")
diagram.connect("controller", "u", "plant", "u")

true_evaluator = diagram.compile(backend="jax")

# Roll out the true closed loop under a sinusoidal reference to collect data.
dt = 0.02
n_samples = 400
ts = dt * jnp.arange(n_samples)
rs = 1.2 * jnp.sin(0.8 * ts).reshape(-1, 1)
x0 = jnp.array([0.5, 0.0])
xs = true_evaluator.rollout(x0, rs, 0.0, dt, hold="foh")

# "Measured" state derivatives (in practice: numerical differentiation of
# logged states; here taken from the true model).
f_true = true_evaluator.f
dxs = jax.vmap(f_true, in_axes=(0, 0, 0))(xs, rs, ts)

# Equation-error identification: copy the compiled snapshot and replace the plant.
f = true_evaluator.f
compiled_params = true_evaluator.params


def loss(theta):
    plant_params = {"m": 1.0, "l": 1.0, "gravity": theta[0], "d": theta[1]}
    p = {**compiled_params, "plant": plant_params}
    dx_hat = jax.vmap(f, in_axes=(0, 0, 0, None))(xs, rs, ts, p)
    return jnp.mean((dx_hat - dxs) ** 2)


program = MathematicalProgram(n_z=2, J=loss, backend="jax")
result = Optimizer(program, z0=jnp.array([7.0, 1.0]), method="l-bfgs").solve()
print(result)  # z* = (gravity, damping), from the guess (7.0, 1.0); true (9.81, 0.20)

theta = result.z
assert abs(float(theta[0]) - 9.81) < 1e-2, "gravity did not converge"
assert abs(float(theta[1]) - 0.20) < 1e-2, "damping did not converge"
