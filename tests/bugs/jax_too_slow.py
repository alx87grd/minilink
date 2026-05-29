import time

import numpy as np

from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem

# Custom blocks


class Integrator(DynamicSystem):
    def __init__(self):

        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "Integrator"
        self.add_output_port("y", dim=1, function=self.h, dependencies=())

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

        r = u[0]
        y = u[1]

        u = 10.0 * (r - y)

        return [u]


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

# # Diagram
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

evaluator_numpy = diagram.compile()
f_compiled_numpy = evaluator_numpy.f
evaluator_jax = diagram.compile(backend="jax")
f_compiled_jax = evaluator_jax.f
f_compiled_jax_jit = evaluator_jax.get_f_jit()


# Benchmarking
n_iters = 10000
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


# JAX JIT

# Warm up (compilation happens here)
f_compiled_jax_jit(x, u).block_until_ready()

t0 = time.perf_counter()
for _ in range(n_iters):
    f_compiled_jax_jit(x, u).block_until_ready()
dt = time.perf_counter() - t0
print(f"JAX JIT:        {dt:.4f} s ({n_iters / dt:.0f} evals/sec)")


# Jax (not JIT)

print("\nJAX (not JIT) is super slow to check!!!!!!")

# # warm up
f_compiled_jax(x, u)

t0 = time.perf_counter()
for _ in range(n_iters):
    f_compiled_jax(x, u)
dt = time.perf_counter() - t0
print(f"JAX (not JIT):  {dt:.4f} s ({n_iters / dt:.0f} evals/sec)")

# Summary: The “error” is really a performance model mismatch: the implementation is intentionally trace-oriented (functional .at[].set, jnp everywhere), which is slow when run eagerly at large len(port_ops) + len(state_ops), and fast when wrapped in jax.jit, exactly as your numbers showed.
