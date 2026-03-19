import time
import numpy as np

try:
    import jax
    import jax.numpy as jnp
except ImportError:
    print("Warning: JAX not installed, some tests will fail.")

from minilink.core.framework import DynamicSystem
from minilink.core.jax_utils import get_f_jax


# System 1: A system that explicitly provides f_jax (Best Practice for complex blocks)
class PendulumFunctional(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum (JAX-Compatible via f_jax)"

    def f(self, x, u, t=0, params=None):
        # Strict Numpy implementation (raises ConcretizationTypeError if duck-typed in JAX)
        gravity = 9.81
        length = 1.0
        damping = 0.5
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * np.sin(q) - damping * dq + u[0]
        return np.array([dq, ddq])

    def f_jax(self, x, u, t=0, params=None):
        # Pure JAX implementation
        gravity = 9.81
        length = 1.0
        damping = 0.5
        q, dq = x[0], x[1]
        ddq = -(gravity / length) * jnp.sin(q) - damping * dq + u[0]
        return jnp.array([dq, ddq])


# System 2: Faulty (in-place mutation with no f_jax fallback)
class PendulumFaulty(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, m=1, p=2)
        self.name = "Pendulum (Incompatible)"

    def f(self, x, u, t=0, params=None):
        gravity = 9.81
        length = 1.0
        damping = 0.5

        dx = np.zeros(2)  # In-place pre-allocation
        dx[0] = x[1]  # BOOM: JAX Tracer cannot handle this item assignment
        dx[1] = -(gravity / length) * np.sin(x[0]) - damping * x[1] + u[0]

        return dx


if __name__ == "__main__":
    print("=" * 60)
    print("TESTING INCOMPATIBLE SYSTEM (DUCK-TYPING ERROR)")
    print("=" * 60)

    sys_faulty = PendulumFaulty()
    f_jax_faulty = get_f_jax(sys_faulty)

    x_test_jax = jnp.array([1.0, 0.0])
    u_test_jax = jnp.array([0.0])

    try:
        # First execution triggers JAX tracing inside XLA, calling sys.f with Tracers
        res = f_jax_faulty(x_test_jax, u_test_jax, 0)
        print("Wait, it succeeded? This shouldn't happen.")
    except Exception as e:
        print("SUCCESSFULLY INTERCEPTED ERROR. User receives friendly message:")
        print(f"--> {e}")

    print("\n" + "=" * 60)
    print("SPEED TEST ON COMPATIBLE SYSTEM")
    print("=" * 60)

    sys_ok = PendulumFunctional()
    f_jax_ok = get_f_jax(sys_ok)  # Picks up sys_ok.f_jax automatically

    x_test_np = np.array([1.0, 0.0])
    u_test_np = np.array([0.0])

    # ---------------------------------------------------------
    # 1. Run in pure Numpy
    # ---------------------------------------------------------
    t0 = time.time()
    for _ in range(10000):
        _ = sys_ok.f(x_test_np, u_test_np, 0)
    t_numpy = time.time() - t0
    print(f"NumPy 10k calls: {t_numpy:.4f} seconds")

    # ---------------------------------------------------------
    # 2. Compile XLA (Warmup)
    # ---------------------------------------------------------
    print("\nCompiling XLA in background...")
    t0_comp = time.time()
    _ = f_jax_ok(x_test_jax, u_test_jax, 0).block_until_ready()
    t_compile = time.time() - t0_comp
    print(f"JAX Compile Time: {t_compile:.4f} seconds")

    # ---------------------------------------------------------
    # 3. Run XLA JAX Loop
    # ---------------------------------------------------------
    t0 = time.time()
    for _ in range(10000):
        res = f_jax_ok(x_test_jax, u_test_jax, 0)
    res.block_until_ready()  # Synchronize device to host
    t_jax = time.time() - t0

    print(f"JAX 10k calls:   {t_jax:.4f} seconds")

    if t_jax > 0:
        print(f"-> Speedup ratio: {t_numpy / t_jax:.1f}x")
