"""
Speed comparison: diagram.f (reference) vs NumpyEvaluator vs JaxEvaluator vs JaxEvaluator+JIT.

Tests correctness (NumpyEvaluator must exactly match reference f) and measures
relative performance across different network topologies and sizes.

Usage:
    python tests/manual/test_f_compare_old.py
"""

import time

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.core.framework import System

try:
    import jax.numpy as jnp
    JAX_AVAILABLE = True
except ImportError:
    JAX_AVAILABLE = False


######################################################################
# Test Subsystems
######################################################################


class SimpleGain(System):
    def __init__(self, id_str, gain=2.0):
        super().__init__(0, 1, 1)
        self.name = id_str
        self.gain = gain
        self.add_input_port(1, "u")
        self.add_output_port(1, "y", function=self.h, dependencies=["u"])

    def h(self, x, u, t=0, params=None):
        return u * self.gain


class SimpleIntegrator(System):
    def __init__(self, id_str):
        super().__init__(1, 1, 1)
        self.name = id_str
        self.add_input_port(1, "u")
        self.add_output_port(1, "x", function=self.compute_state, dependencies="all")

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u


class MultiInputNode(System):
    def __init__(self, id_str, in_ports):
        super().__init__(1, in_ports, 1)
        self.name = id_str
        self.in_ports = in_ports
        for p in range(in_ports):
            self.add_input_port(1, f"u{p}")
        self.add_output_port(1, "x", function=self.compute_state, dependencies="all")

    def compute_state(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return np.sum(u)


######################################################################
# Network Builders
######################################################################


def build_deep_network(depth=50):
    """Builds a long chain of Integrator -> Gain -> Integrator ..."""
    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    for i in range(depth):
        diag.add_subsystem(SimpleIntegrator(f"Int{i}"), f"Int{i}")
        diag.add_subsystem(SimpleGain(f"Gain{i}"), f"Gain{i}")
        diag.connect(f"Int{i}", "x", f"Gain{i}", "u")
        if i > 0:
            diag.connect(f"Gain{i-1}", "y", f"Int{i}", "u")
    return diag


def build_dense_network(num_nodes=50, connections_per_node=3):
    """Builds a dense feed-forward network of nodes."""
    diag = DiagramSystem()
    diag.graphe_building_verbose = False
    for i in range(num_nodes):
        diag.add_subsystem(SimpleIntegrator(f"Node{i}"), f"Node{i}")

    np.random.seed(42)
    for i in range(1, num_nodes):
        num_conn = min(i, connections_per_node)
        sources = np.random.choice(range(i), size=num_conn, replace=False)

        sys_id = f"MultiNode{i}"
        diag.add_subsystem(MultiInputNode(sys_id, num_conn), sys_id)

        for p_idx, src_i in enumerate(sources):
            diag.connect(f"Node{src_i}", "x", sys_id, f"u{p_idx}")

        diag.connect(sys_id, "x", f"Node{i}", "u")

    diag.add_subsystem(SimpleGain("SourceNode"), "SourceNode")
    diag.connect("SourceNode", "y", "Node0", "u")
    return diag


######################################################################
# Benchmark Logic
######################################################################


def benchmark_all_backends(diag, iters=100, label="Network"):
    """
    Compares all backends:
      1. diagram.f           (recursive reference)
      2. NumpyEvaluator
      3. JaxEvaluator        (if JAX available)
      4. JaxEvaluator + JIT  (if JAX available)
    """
    print(f"\n{'='*60}")
    print(f"  {label}")
    print(f"  Subsystems: {len(diag.subsystems)}, States: {diag.n}")
    print(f"{'='*60}")

    x_np = np.ones(diag.n)
    u_np = np.array([])
    t = 0.0

    results = {}

    # ── 1. Reference f ──────────────────────────────────────────────
    orig_f_works = True
    try:
        dx_ref = diag.f(x_np, u_np, t)  # trial call
        start = time.time()
        for _ in range(iters):
            dx_ref = diag.f(x_np, u_np, t)
        results["f()"] = time.time() - start
        print(f"  Reference f()          ({iters} iters): {results['f()']:.5f} s")
    except RecursionError:
        print("  Reference f()          : SKIPPED (RecursionError)")
        orig_f_works = False

    # ── 2. NumpyEvaluator ────────────────────────────────────────────
    np_evaluator = diag.compile(backend="numpy")
    np_evaluator.compute_dx(x_np, u_np, t)  # warmup
    start = time.time()
    for _ in range(iters):
        dx_np = np_evaluator.compute_dx(x_np, u_np, t)
    results["numpy"] = time.time() - start
    print(f"  NumpyEvaluator         ({iters} iters): {results['numpy']:.5f} s")

    if orig_f_works:
        np.testing.assert_allclose(dx_ref, dx_np, atol=1e-8)
        print("  ✓ NumpyEvaluator matches reference f()")

    # ── 3 & 4. JAX backends ─────────────────────────────────────────
    if JAX_AVAILABLE:
        x_jax = jnp.array(x_np)
        u_jax = jnp.array(u_np)

        jax_evaluator = diag.compile(backend="jax")
        jit_compute_dx = jax_evaluator.get_jit_compute_dx()

        # Warmup (triggers XLA compile)
        jax_evaluator.compute_dx(x_jax, u_jax, t).block_until_ready()
        jit_compute_dx(x_jax, u_jax, t).block_until_ready()

        start = time.time()
        for _ in range(iters):
            dx_jax = jax_evaluator.compute_dx(x_jax, u_jax, t)
        dx_jax.block_until_ready()
        results["jax"] = time.time() - start
        print(f"  JaxEvaluator           ({iters} iters): {results['jax']:.5f} s")

        start = time.time()
        for _ in range(iters):
            dx_jit = jit_compute_dx(x_jax, u_jax, t)
        dx_jit.block_until_ready()
        results["jax_jit"] = time.time() - start
        print(f"  JaxEvaluator (JIT)     ({iters} iters): {results['jax_jit']:.5f} s")
    else:
        print("  JAX backends           : SKIPPED (JAX not installed)")

    # ── Speedup Summary ──────────────────────────────────────────────
    base = results["numpy"]
    print(f"\n  Speedups (vs NumpyEvaluator):")
    for name, t_val in results.items():
        if name != "numpy":
            arrow = "↑" if t_val < base else "↓"
            print(f"    {name:22s}: {base / t_val:.2f}x {arrow}")


######################################################################
if __name__ == "__main__":
    benchmark_all_backends(
        build_deep_network(depth=50),
        iters=1000,
        label="Chain Network (depth=50)",
    )

    benchmark_all_backends(
        build_dense_network(num_nodes=20, connections_per_node=5),
        iters=1000,
        label="Dense Network (20 nodes)",
    )
