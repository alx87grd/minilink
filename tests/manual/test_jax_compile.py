"""
Tests for the JAX compilation backend using the new Evaluator API.

Covers:
  - JaxEvaluator.compute_dx  (gradient through f)
  - JaxEvaluator.compute_outputs  (gradient through h / output ports)
  - JaxEvaluator.get_jit_compute_dx  (JIT wrapper)

Usage:
    python tests/manual/test_jax_compile.py
"""

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.framework import DynamicSystem, StaticSystem

try:
    import jax
    import jax.numpy as jnp
    JAX_AVAILABLE = True
except ImportError:
    JAX_AVAILABLE = False
    print("Warning: JAX not installed, all tests in this file will be skipped.")


######################################################################
# JAX-compatible subsystem definitions
######################################################################


class JaxGain(StaticSystem):
    def __init__(self, gain=2.0):
        super().__init__(m=1, p=1)
        self.name = "JaxGain"
        self.gain = float(gain)

    def h(self, x, u, t=0, params=None):
        return u * self.gain


class JaxIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, m=1, p=1)
        self.name = "JaxIntegrator"
        self.add_output_port(1, "x_out", function=self.compute_state, dependencies=())

    def f(self, x, u, t=0, params=None):
        # dx/dt = u  (JAX-traceable: no in-place mutation)
        return u

    def compute_state(self, x, u, t=0, params=None):
        return x


######################################################################
# Tests
######################################################################


def test_jax_evaluator_compute_dx():
    """JaxEvaluator.compute_dx returns correct derivative and supports grad."""
    if not JAX_AVAILABLE:
        print("SKIP: test_jax_evaluator_compute_dx (JAX not available)")
        return

    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    diag.add_input_port(1, "u")
    diag.add_subsystem(JaxIntegrator(), "int")
    diag.connect("input", "u", "int", "u")

    evaluator = diag.compile(backend="jax")

    x = jnp.array([1.0], dtype=jnp.float32)
    u = jnp.array([2.5], dtype=jnp.float32)

    # Forward pass
    dx = evaluator.compute_dx(x, u, 0.0)
    assert float(dx[0]) == 2.5, f"Expected dx=2.5, got {dx[0]}"

    # Gradient through compute_dx
    def dx0_of_u(u0):
        return evaluator.compute_dx(x, jnp.array([u0]), 0.0)[0]

    ddx_du = jax.grad(dx0_of_u)(jnp.array(2.5, dtype=jnp.float32))
    assert float(ddx_du) == 1.0, f"Expected grad=1.0, got {ddx_du}"

    print(f"PASS: test_jax_evaluator_compute_dx  dx={dx}, grad={ddx_du}")


def test_jax_evaluator_compute_outputs():
    """JaxEvaluator.compute_outputs returns correct port values and supports grad."""
    if not JAX_AVAILABLE:
        print("SKIP: test_jax_evaluator_compute_outputs (JAX not available)")
        return

    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    diag.add_input_port(1, "u")
    diag.add_subsystem(JaxGain(3.0), "gain")
    diag.connect("input", "u", "gain", "u")

    evaluator = diag.compile(backend="jax")

    x = jnp.zeros((diag.n,), dtype=jnp.float32)
    u = jnp.array([2.0], dtype=jnp.float32)

    y = evaluator.compute_outputs(x, u, 0.0, ports=[("gain", "y")])
    assert float(y[0]) == 6.0, f"Expected y=6.0, got {y[0]}"

    def y_of_u(u0):
        return evaluator.compute_outputs(x, jnp.array([u0]), 0.0, ports=[("gain", "y")])[0]

    dy_du = jax.grad(y_of_u)(jnp.array(2.0, dtype=jnp.float32))
    assert float(dy_du) == 3.0, f"Expected grad=3.0, got {dy_du}"

    print(f"PASS: test_jax_evaluator_compute_outputs  y={y}, grad={dy_du}")


def test_jax_evaluator_jit():
    """JaxEvaluator.get_jit_compute_dx returns a JIT-compiled callable."""
    if not JAX_AVAILABLE:
        print("SKIP: test_jax_evaluator_jit (JAX not available)")
        return

    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    diag.add_input_port(1, "u")
    diag.add_subsystem(JaxIntegrator(), "int")
    diag.connect("input", "u", "int", "u")

    evaluator = diag.compile(backend="jax")
    jit_dx = evaluator.get_jit_compute_dx()

    x = jnp.array([1.0], dtype=jnp.float32)
    u = jnp.array([5.0], dtype=jnp.float32)

    dx = jit_dx(x, u, 0.0)
    assert float(dx[0]) == 5.0, f"Expected dx=5.0, got {dx[0]}"

    print(f"PASS: test_jax_evaluator_jit  dx={dx}")


######################################################################
if __name__ == "__main__":
    test_jax_evaluator_compute_dx()
    test_jax_evaluator_compute_outputs()
    test_jax_evaluator_jit()

    if JAX_AVAILABLE:
        print("\nAll JAX tests passed.")
    else:
        print("\nAll tests skipped (JAX not available).")
