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
    
    # Timing original f (recursive)
    start_time = time.time()
    for _ in range(100):
        dx_orig = diag.f(x, u, t)
    orig_time = time.time() - start_time
    
    # Timing f_fast (topological)
    start_time = time.time()
    for _ in range(100):
        dx_fast = diag.f_fast(x, u, t)
    fast_time = time.time() - start_time
    
    print(f"Original f time (100 calls): {orig_time:.5f} s")
    print(f"Fast f time     (100 calls): {fast_time:.5f} s")
    print(f"Speedup: {orig_time / fast_time:.2f}x")
    
    # Verify correctness
    np.testing.assert_allclose(dx_orig, dx_fast, err_msg="f and f_fast derivatives do not match!")
    print("Verification passed! f_fast output matches exactly with f.")

if __name__ == "__main__":
    test_performance()
