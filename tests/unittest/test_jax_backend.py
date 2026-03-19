import unittest

from minilink.core.diagram import DiagramSystem
from minilink.core.framework import DynamicSystem, StaticSystem


class _JaxImports:
    try:
        import jax
        import jax.numpy as jnp

        jax_available = True
    except ImportError:
        jax_available = False
        jax = None
        jnp = None


@unittest.skipUnless(_JaxImports.jax_available, "jax is not installed")
class TestJaxBackendVariant2(unittest.TestCase):
    def test_compile_jax_outputs_and_grad(self):
        jax = _JaxImports.jax
        jnp = _JaxImports.jnp

        class JaxGain(StaticSystem):
            def __init__(self, gain=2.0):
                super().__init__(m=1, p=1)
                self.name = "JaxGain"
                self.gain = float(gain)

            def h(self, x, u, t=0, params=None):
                # u shape: (1,)
                return u * self.gain

        diag = DiagramSystem()
        diag.graphe_building_verbose = False
        diag.add_input_port(1, "u")
        diag.add_subsystem(JaxGain(3.0), "gain")
        diag.connect("input", "u", "gain", "u")
        diag.compile()

        fn = diag.compile_jax(output_ports=[("gain", "y")], jit=True)

        x = jnp.zeros((diag.n,), dtype=jnp.float32)

        def y_of_u(u0):
            y = fn(x, jnp.array([u0], dtype=jnp.float32), 0.0)
            return y[0]

        u0 = jnp.array(2.0, dtype=jnp.float32)
        y = y_of_u(u0)
        dy_du = jax.grad(lambda uu: y_of_u(uu))(u0)

        self.assertAlmostEqual(float(y), 6.0, places=5)
        self.assertAlmostEqual(float(dy_du), 3.0, places=5)

    def test_f_fast_jax_dx_and_grad(self):
        jax = _JaxImports.jax
        jnp = _JaxImports.jnp

        class JaxIntegrator(DynamicSystem):
            def __init__(self):
                super().__init__(n=1, m=1, p=1)
                self.name = "JaxIntegrator"

                # For clarity, output is the state (y=x).
                self.add_output_port(1, "x_out", function=self.compute_state, dependencies=())

            def f(self, x, u, t=0, params=None):
                # dx/dt = u
                return u

            def h(self, x, u, t=0, params=None):
                # Keep the default `y` output JAX-compatible too.
                return x

            def compute_state(self, x, u, t=0, params=None):
                return x

        diag = DiagramSystem()
        diag.graphe_building_verbose = False
        diag.add_input_port(1, "u")
        diag.add_subsystem(JaxIntegrator(), "int")
        diag.connect("input", "u", "int", "u")
        diag.compile()

        x = jnp.array([1.0], dtype=jnp.float32)

        def dx0_of_u(u0):
            dx = diag.f_fast_jax(x, jnp.array([u0], dtype=jnp.float32), 0.0)
            return dx[0]

        u0 = jnp.array(2.5, dtype=jnp.float32)
        dx0 = dx0_of_u(u0)
        ddx0_du = jax.grad(lambda uu: dx0_of_u(uu))(u0)

        self.assertAlmostEqual(float(dx0), 2.5, places=5)
        self.assertAlmostEqual(float(ddx0_du), 1.0, places=5)


if __name__ == "__main__":
    unittest.main()

