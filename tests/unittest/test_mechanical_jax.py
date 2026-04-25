"""Optional JAX smoke tests (skip if jax is not installed)."""

import unittest

import numpy as np
import pytest

try:
    import jax
    import jax.numpy as jnp

    from minilink.mechanics import JaxMechanicalSystem, MechanicalSystem

    HAS_JAX = True
except ImportError:
    HAS_JAX = False


pytestmark = pytest.mark.jax


@unittest.skipUnless(HAS_JAX, "jax not installed")
class TestMechanicalSystemJax(unittest.TestCase):
    def test_f_is_jaxpr_traceable(self):
        sys = JaxMechanicalSystem(dof=2)
        x = jnp.zeros(4)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_f_matches_numpy_1dof(self):
        sys_j = JaxMechanicalSystem(dof=1)
        sys_n = MechanicalSystem(dof=1)
        x = jnp.array([0.2, -0.1])
        u = jnp.zeros(1)
        dx_j = sys_j.f(x, u)
        xn = np.array([0.2, -0.1])
        un = np.zeros(1)
        dx_n = sys_n.f(xn, un)
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-5, atol=1e-5)


if __name__ == "__main__":
    unittest.main()
