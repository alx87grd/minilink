import time

import numpy as np
import jax.numpy as jnp

from minilink.core.framework import DynamicSystem


class Pendulum(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum"

    def f(self, x, u, t=0, params=None):
        # Pure JAX implementation
        gravity = 9.81
        length = 1.0
        damping = 0.5
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * jnp.sin(q) - damping * dq + u[0]
        return jnp.array([dq, ddq])


sys = Pendulum()
x_np = np.array([1.0, 0.0])
u_np = np.array([0.0])
t = 0.0
n_calls = 100000

print("\n=== Evaluator f() speed test ===")
print(f"system={sys.name}  calls={n_calls}")

tic = time.perf_counter()
ev_np = sys.compile(backend="numpy", verbose=False)
t_compile_np = time.perf_counter() - tic

tic = time.perf_counter()
for _ in range(n_calls):
    y_np = ev_np.f(x_np, u_np, t)
t_f_np = time.perf_counter() - tic

print(f"compile[numpy] {t_compile_np:.6f}s")
print(f"f-loop[numpy]  {t_f_np:.6f}s")
t_call_np_us = (t_f_np / n_calls) * 1e6
print(f"f-call[numpy]  {t_call_np_us:.3f} us/call")

x_jax = jnp.array([1.0, 0.0])
u_jax = jnp.array([0.0])

tic = time.perf_counter()
ev_jax = sys.compile(backend="jax", verbose=False)
t_compile_jax = time.perf_counter() - tic

tic = time.perf_counter()
for _ in range(n_calls):
    y_jax = ev_jax.f(x_jax, u_jax, t)
y_jax.block_until_ready()
t_f_jax = time.perf_counter() - tic

print(f"compile[jax]   {t_compile_jax:.6f}s")
print(f"f-loop[jax]    {t_f_jax:.6f}s")
t_call_jax_us = (t_f_jax / n_calls) * 1e6
print(f"f-call[jax]    {t_call_jax_us:.3f} us/call")
if t_f_jax > 0:
    print(f"speedup f-loop (numpy/jax): {t_f_np / t_f_jax:.3f}x")

delta_compile = t_compile_jax - t_compile_np
delta_call = (t_f_np / n_calls) - (t_f_jax / n_calls)
if delta_call > 0:
    n_roi = int(np.ceil(max(delta_compile, 0.0) / delta_call))
    print(f"roi threshold  {n_roi} calls")
else:
    print("roi threshold  never (jax call is not faster)")
