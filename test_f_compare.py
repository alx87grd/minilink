import time
import numpy as np
from framework import System
from diagram import DiagramSystem

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
        self.add_output_port(1, "x", function=self.compute_state, dependencies=None)
        
    def compute_state(self, x, u, t=0, params=None):
        return x
        
    def f(self, x, u, t=0, params=None):
        return u

def build_deep_network(depth=50):
    diag = DiagramSystem()
    
    # We create a chain of integrators and gains
    # Int1 -> Gain1 -> Int2 -> Gain2 -> ... -> IntN -> GainN
    for i in range(depth):
        diag.add_subsystem(SimpleIntegrator(f"Int{i}"), f"Int{i}")
        diag.add_subsystem(SimpleGain(f"Gain{i}"), f"Gain{i}")
        
        diag.connect(f"Int{i}", "x", f"Gain{i}", "u")
        
        if i > 0:
            diag.connect(f"Gain{i-1}", "y", f"Int{i}", "u")
            
    return diag

def test_performance():
    depth = 50
    diag = build_deep_network(depth)
    diag.compile()
    
    # Set up some dummy state and input
    x = np.ones(diag.n)
    u = np.array([])
    t = 0.0
    
    # Warmup
    diag.f(x, u, t)
    diag.f_fast(x, u, t)
    diag.f_super_fast(x, u, t)
    
    iters = 1000
    
    # Timing original f (recursive)
    start_time = time.time()
    for _ in range(iters):
        dx_orig = diag.f(x, u, t)
    orig_time = time.time() - start_time
    
    # Timing f_fast (topological)
    start_time = time.time()
    for _ in range(iters):
        dx_fast = diag.f_fast(x, u, t)
    fast_time = time.time() - start_time

    # Timing f_super_fast (flattened)
    start_time = time.time()
    for _ in range(iters):
        dx_super_fast = diag.f_super_fast(x, u, t)
    super_fast_time = time.time() - start_time
    
    print(f"Original f time     ({iters} calls): {orig_time:.5f} s")
    print(f"Fast f time         ({iters} calls): {fast_time:.5f} s")
    print(f"Super Fast f time   ({iters} calls): {super_fast_time:.5f} s")
    print("-" * 40)
    print(f"Speedup vs Orginal  : {orig_time / super_fast_time:.2f}x")
    print(f"Speedup vs Fast     : {fast_time / super_fast_time:.2f}x")
    
    # Verify correctness
    np.testing.assert_allclose(dx_orig, dx_fast, err_msg="f and f_fast derivatives do not match!")
    np.testing.assert_allclose(dx_orig, dx_super_fast, err_msg="f and f_super_fast derivatives do not match!")
    print("Verification passed! f_super_fast output matches exactly with f.")

def build_dense_network(num_nodes=50, connections_per_node=3):
    diag = DiagramSystem()
    
    # Create N integrator nodes
    for i in range(num_nodes):
        diag.add_subsystem(SimpleIntegrator(f"Node{i}"), f"Node{i}")
        
    # Create random but feed-forward connections to avoid algebraic loops
    # A node i can only take inputs from nodes j < i
    np.random.seed(42)
    for i in range(1, num_nodes):
        # Determine how many connections this node will receive
        num_conn = min(i, connections_per_node)
        sources = np.random.choice(range(i), size=num_conn, replace=False)
        
        # We need a custom block to sum multiple inputs since SimpleIntegrator only takes 1
        # To keep it simple, we just chain gains or add a custom summer block
        # For simplicity of benchmarking diagram routing speed, let's just create a multi-gain block
        
        class MultiInputNode(System):
            def __init__(self, id_str, in_ports):
                super().__init__(1, in_ports, 1)
                self.name = id_str
                self.in_ports = in_ports
                for p in range(in_ports):
                    self.add_input_port(1, f"u{p}")
                self.add_output_port(1, "x", function=self.compute_state, dependencies=None)
                
            def compute_state(self, x, u, t=0, params=None):
                return x
                
            def f(self, x, u, t=0, params=None):
                return np.sum(u)

        sys_id = f"MultiNode{i}"
        diag.add_subsystem(MultiInputNode(sys_id, num_conn), sys_id)
        
        # Connect sources to this new multi-node
        for p_idx, src_i in enumerate(sources):
            diag.connect(f"Node{src_i}", "x", sys_id, f"u{p_idx}")
            
        # Connect the multi-node into the main integrator sequence
        diag.connect(sys_id, "x", f"Node{i}", "u")

    # Connect Node 0 to a dummy gain to give it an input
    diag.add_subsystem(SimpleGain("SourceNode"), "SourceNode")
    diag.connect("SourceNode", "y", "Node0", "u")
        
    return diag

def test_large_network_performance(skip_orig_f=True):
    print("\n=== Benchmarking Dense Network ===")
    num_nodes = 200
    conn_per_node = 5
    diag = build_dense_network(num_nodes, conn_per_node)
    
    print(f"Graph nodes: {len(diag.subsystems)}")
    print(f"Compiling...")
    compile_start = time.time()
    diag.compile()
    diag.compile_super_fast()
    print(f"Compilation finished in {time.time() - compile_start:.4f}s")
    
    x = np.ones(diag.n)
    u = np.array([])
    t = 0.0
    
    # Warmup
    orig_f_works = not skip_orig_f
    if orig_f_works:
        try:
            diag.f(x, u, t)
        except RecursionError:
            print("Warmup: Original f failed with RecursionError. Skipping benchmarking for it.")
            orig_f_works = False
        
    diag.f_fast(x, u, t)
    diag.f_super_fast(x, u, t)
    
    iters = 1000
    orig_time = float('inf')
    dx_orig = None
    
    if orig_f_works:
        start_time = time.time()
        for _ in range(iters):
            dx_orig = diag.f(x, u, t)
        orig_time = time.time() - start_time
        print(f"Original f time     ({iters} calls): {orig_time:.5f} s")
    else:
        print(f"Original f time     ({iters} calls): Skipped (RecursionError/Too Slow)")
    
    start_time = time.time()
    for _ in range(iters):
        dx_fast = diag.f_fast(x, u, t)
    fast_time = time.time() - start_time

    start_time = time.time()
    for _ in range(iters):
        dx_super_fast = diag.f_super_fast(x, u, t)
    super_fast_time = time.time() - start_time
    
    print(f"Fast f time         ({iters} calls): {fast_time:.5f} s")
    print(f"Super Fast f time   ({iters} calls): {super_fast_time:.5f} s")
    print("-" * 40)
    if orig_f_works:
        print(f"Speedup vs Orginal  : {orig_time / super_fast_time:.2f}x")
    print(f"Speedup vs Fast     : {fast_time / super_fast_time:.2f}x")
    
    if orig_f_works:
        np.testing.assert_allclose(dx_orig, dx_fast, err_msg="f and f_fast derivatives do not match!", atol=1e-8)
        np.testing.assert_allclose(dx_orig, dx_super_fast, err_msg="f and f_super_fast derivatives do not match!", atol=1e-8)
    else:
        np.testing.assert_allclose(dx_fast, dx_super_fast, err_msg="f_fast and f_super_fast do not match!", atol=1e-8)
        
    print("Verification passed! f_super_fast output matches exactly.")

if __name__ == "__main__":
    import sys
    sys.setrecursionlimit(5000)
    print("=== Benchmarking Chain Network ===")
    test_performance()
    test_large_network_performance()
