import time

import numpy as np

from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem

# Demo controls.
PRINT_COMPILE_REPORT = True  # Print compile timing diagnostics.

# Custom blocks


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "Integrator"

    def f(self, x, u, t=0, params=None):
        return u

    def h(self, x, u, t=0, params=None):
        return x


class PropController(StaticSystem):
    def __init__(self):
        super().__init__()
        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", dim=1, nominal_value=np.array([0.0]))

        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        r, y = self.get_port_values_from_u(u, "r", "y")

        u_cmd = 10.0 * (r[0] - y[0])
        return [u_cmd]


# Custom diagram

# Plant system
sys1 = Integrator()
sys1.state.labels = ["v"]
sys1.x0[0] = 20.0
sys2 = Integrator()
sys2.state.labels = ["x"]
sys2.x0[0] = 20.0

# Controllers
ctl1 = PropController()
ctl2 = PropController()

# Source input
step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([20.0])
step.params["step_time"] = 10.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl1, "controller1")
diagram.add_subsystem(ctl2, "controller2")
diagram.add_subsystem(sys1, "integrator1")
diagram.add_subsystem(sys2, "integrator2")

diagram.connect("integrator1", "y", "integrator2", "u")
diagram.connect("controller2", "u", "integrator1", "u")
diagram.connect("integrator1", "y", "controller2", "y")
diagram.connect("controller1", "u", "controller2", "r")
diagram.connect("integrator2", "y", "controller1", "y")
diagram.connect("step", "y", "controller1", "r")

diagram.plot_graphe()
# diagram.compute_trajectory(tf=20)


x = np.array([1.0, 2.0])
u = np.array([])

f_baseline = diagram.f

evaluator_numpy = diagram.compile(verbose=PRINT_COMPILE_REPORT)
f_compiled_numpy = evaluator_numpy.f
try:
    evaluator_jax = diagram.compile(backend="jax", verbose=PRINT_COMPILE_REPORT)
except ImportError as exc:
    evaluator_jax = None
    print(f"Skipping JAX compile: {exc}")
else:
    f_compiled_jax = evaluator_jax.f
    f_compiled_jax_jit = evaluator_jax.get_f_jit()

dx_baseline = f_baseline(x, u)
dx_compiled_numpy = f_compiled_numpy(x, u)

print("Computing dx=f(x,u) with baseline and compiled evaluator")
print("Baseline:", dx_baseline)
print("Compiled (numpy):", dx_compiled_numpy)
if evaluator_jax is not None:
    dx_compiled_jax = f_compiled_jax(x, u)
    dx_compiled_jax_jit = f_compiled_jax_jit(x, u)
    print("Compiled (jax):", dx_compiled_jax)
    print("Compiled (jax jit):", dx_compiled_jax_jit)


# Benchmarking
n_iters = 100000
print(f"\nBenchmarking {n_iters} iterations:")

# Baseline (recursive)
t0 = time.perf_counter()
for _ in range(n_iters):
    f_baseline(x, u)
dt = time.perf_counter() - t0
print(f"Baseline:       {dt:.4f} s ({n_iters / dt:.0f} evals/sec)")

# NumPy Compiled
t0 = time.perf_counter()
for _ in range(n_iters):
    f_compiled_numpy(x, u)
dt = time.perf_counter() - t0
print(f"NumPy Compiled: {dt:.4f} s ({n_iters / dt:.0f} evals/sec)")

# Jax (not JIT)

# # warm up
# f_compiled_jax(x, u)

# t0 = time.perf_counter()
# for _ in range(n_iters):
#     f_compiled_jax(x, u)
# dt = time.perf_counter() - t0
# print(f"JAX (not JIT):  {dt:.4f} s ({n_iters/dt:.0f} evals/sec)")


# JAX JIT

if evaluator_jax is not None:
    # Warm up (compilation happens here)
    f_compiled_jax_jit(x, u).block_until_ready()

    t0 = time.perf_counter()
    for _ in range(n_iters):
        f_compiled_jax_jit(x, u).block_until_ready()
    dt = time.perf_counter() - t0
    print(f"JAX JIT:        {dt:.4f} s ({n_iters / dt:.0f} evals/sec)")
