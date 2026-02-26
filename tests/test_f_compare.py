import time
import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
from minilink.core.framework import System
from minilink.core.diagram import DiagramSystem

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
        self.add_output_port(1, "x", function=self.compute_state, dependencies=())

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
        self.add_output_port(1, "x", function=self.compute_state, dependencies=())

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
# Test Logic
######################################################################


def benchmark_simulation(diag, iters=100, label="Network"):
    """Core benchmarking function for f vs f_fast."""
    print(f"\n=== Benchmarking {label} ===")
    print(f"Subsystems: {len(diag.subsystems)}, States: {diag.n}")

    if not diag.compiled:
        print("Compiling...")
        start = time.time()
        diag.compile()
        print(f"Compiled in {time.time()-start:.4f}s")

    x = np.ones(diag.n)
    u = np.array([])
    t = 0.0

    # Test Original f (if it doesn't crash)
    orig_f_works = True
    try:
        diag.f(x, u, t)
    except RecursionError:
        print("Original f failed (RecursionError). Skipping its benchmark.")
        orig_f_works = False

    # Warmup Fast
    diag.f_fast(x, u, t)
    diag.f_fast(x, u, t)

    # Benchmark Fast
    start_time = time.time()
    print("Executing f_fast...")
    for _ in range(iters):
        dx_fast = diag.f_fast(x, u, t)
    fast_time = time.time() - start_time
    print(f"Fast f time     ({iters} calls): {fast_time:.5f} s")

    # Benchmark Original
    if orig_f_works:
        start_time = time.time()
        print("Executing f...")
        for _ in range(iters):
            dx_orig = diag.f(x, u, t)
        orig_time = time.time() - start_time
        print(f"Original f time ({iters} calls): {orig_time:.5f} s")
        print(f"Speedup vs Original: {orig_time / fast_time:.2f}x")

        # Validation
        np.testing.assert_allclose(dx_orig, dx_fast, atol=1e-8)
        print("Verification: f_fast matches f exactly.")
    else:
        print("Verification: Skipped (Original f too slow/deep)")


def validate_large_network(num_nodes=500, conn_per_node=5, iters=10):
    """Validates that large networks compile and run f_fast without crashing."""

    print(f"\n=== Validating Large Network ({num_nodes} nodes) ===")
    diag = build_dense_network(num_nodes, conn_per_node)
    print(f"Subsystems: {len(diag.subsystems)}, States: {diag.n}")
    diag.plot_graphe()

    print("Compiling...")
    diag.compile()

    x = np.ones(diag.n)
    u = np.array([])
    t = 0.0

    print("Executing f_fast...")
    dx = diag.f_fast(x, u, t)
    print(f"Success! Output vector size: {len(dx)}")

    # Benchmark Fast
    start_time = time.time()
    print("Executing f_fast...")
    for _ in range(iters):
        dx_fast = diag.f_fast(x, u, t)
    fast_time = time.time() - start_time
    print(f"Fast f time     ({iters} calls): {fast_time:.5f} s")


######################################################################
if __name__ == "__main__":
    # import sys
    # sys.setrecursionlimit(2000)

    # 1. Standard chain benchmark
    chain_diag = build_deep_network(depth=50)
    chain_diag.plot_graphe()
    benchmark_simulation(chain_diag, iters=1000, label="Chain Network")

    # 2. Dense network benchmark
    dense_diag = build_dense_network(num_nodes=20, connections_per_node=5)
    dense_diag.plot_graphe()
    benchmark_simulation(dense_diag, iters=1000, label="Dense Network")

    # 3. Large network validation
    validate_large_network(num_nodes=500, conn_per_node=5, iters=1000)
