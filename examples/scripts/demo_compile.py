from minilink.compile import compile
import numpy as np
from minilink.core.framework import System


class SimpleIntegrator(System):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.add_input_port(1, "u")
        self.add_output_port(1, "y", function=self.h, dependencies="")

    def h(self, x, u, t=0, params=None):
        return x

    def f(self, x, u, t=0, params=None):
        return u


sys = SimpleIntegrator()
# ev = compile(sys, backend="numpy")
ev = compile(sys, backend="jax")
x = np.array([0.5, 0.0])
u = np.ones(sys.m)
print("f:", ev.f(x, u, 0))
print("h:", ev.h(x, u, 0))
print("outputs:", ev.outputs(x, u, 0))
print("f_ivp:", ev.f_ivp(x, 0))
print("f_p:", ev.f_p(x, u, 0, {"g": 0.0, "m": 1.0, "l": 1.0}))

ev_np = compile(sys, backend="numpy", verbose=True)
ev_jax = compile(sys, backend="jax", verbose=True)

print("f (numpy):", ev_np.f(x, u, 0))
print("f (jax):", ev_jax.f(x, u, 0))
