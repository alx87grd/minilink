"""Optional JAX smoke tests (skip if jax is not installed).

The match-NumPy tests follow the canonical pattern documented in
``agent.md`` &sect;4: every JAX twin plant must include both a nominal /
linear regime check and a non-trivial regime check (custom inertia,
damping, gravity, or an off-axis input) so the JAX trace and the NumPy
reference disagree at most by ULP-level rounding.
"""

import unittest

import numpy as np
import pytest

pytest.importorskip("jax")

import jax  # noqa: E402
import jax.numpy as jnp  # noqa: E402

from minilink.compile.jax_utils import configure_jax  # noqa: E402
from minilink.dynamics.abstraction.mechanical import (  # noqa: E402
    JaxMechanicalSystem,
    MechanicalSystem,
)


@pytest.mark.optional
@pytest.mark.jax
class TestMechanicalSystemJax(unittest.TestCase):
    def setUp(self):
        # x64 lets the non-trivial-regime check assert ULP-level agreement
        # with the NumPy reference (float64).
        configure_jax(enable_x64=True)

    def test_f_is_jaxpr_traceable(self):
        sys = JaxMechanicalSystem(dof=2)
        x = jnp.zeros(4)
        u = jnp.zeros(2)
        jax.make_jaxpr(lambda xx, uu: sys.f(xx, uu))(x, u)

    def test_f_matches_numpy_1dof_linear_regime(self):
        # Nominal regime: small q / dq, identity inertia, zero gravity.
        sys_j = JaxMechanicalSystem(dof=1)
        sys_n = MechanicalSystem(dof=1)
        x = jnp.array([0.2, -0.1])
        u = jnp.zeros(1)
        dx_j = sys_j.f(x, u)
        xn = np.array([0.2, -0.1])
        un = np.zeros(1)
        dx_n = sys_n.f(xn, un)
        np.testing.assert_allclose(np.asarray(dx_j), dx_n, rtol=1e-5, atol=1e-5)

    def test_f_matches_numpy_2dof_nontrivial_regime(self):
        # Non-trivial regime: custom inertia (non-identity, non-diagonal),
        # custom gravity / damping, and a non-zero input. The dynamics now
        # exercise C, g, d, and B beyond their default zero / identity.
        class _NumpyTwoLink(MechanicalSystem):
            def H(self, q, params=None):
                m1, m2, l = 1.5, 0.7, 0.4
                c2 = np.cos(q[1])
                return np.array(
                    [
                        [m1 + m2 + 2 * m2 * l * c2, m2 + m2 * l * c2],
                        [m2 + m2 * l * c2, m2],
                    ],
                )

            def C(self, q, dq, params=None):
                m2, l = 0.7, 0.4
                s2 = np.sin(q[1])
                return np.array(
                    [
                        [-m2 * l * s2 * dq[1], -m2 * l * s2 * (dq[0] + dq[1])],
                        [m2 * l * s2 * dq[0], 0.0],
                    ],
                )

            def g(self, q, params=None):
                return np.array([2.0 * np.sin(q[0]), 1.0 * np.sin(q[1])])

            def d(self, q, dq, params=None):
                return 0.05 * np.asarray(dq)

        class _JaxTwoLink(JaxMechanicalSystem):
            def H(self, q, params=None):
                m1, m2, l = 1.5, 0.7, 0.4
                c2 = jnp.cos(q[1])
                return jnp.array(
                    [
                        [m1 + m2 + 2 * m2 * l * c2, m2 + m2 * l * c2],
                        [m2 + m2 * l * c2, m2],
                    ],
                )

            def C(self, q, dq, params=None):
                m2, l = 0.7, 0.4
                s2 = jnp.sin(q[1])
                return jnp.array(
                    [
                        [-m2 * l * s2 * dq[1], -m2 * l * s2 * (dq[0] + dq[1])],
                        [m2 * l * s2 * dq[0], 0.0],
                    ],
                )

            def g(self, q, params=None):
                return jnp.array([2.0 * jnp.sin(q[0]), 1.0 * jnp.sin(q[1])])

            def d(self, q, dq, params=None):
                return 0.05 * dq

        sys_n = _NumpyTwoLink(dof=2)
        sys_j = _JaxTwoLink(dof=2)

        x = np.array([0.5, -0.7, 0.3, -0.2])
        u = np.array([0.4, -0.6])

        dx_n = sys_n.f(x, u)
        dx_j = np.asarray(sys_j.f(jnp.asarray(x), jnp.asarray(u)))
        np.testing.assert_allclose(dx_j, dx_n, rtol=1e-9, atol=1e-9)

    def test_f_uses_explicit_params_in_matrix_hooks(self):
        class MassSystem(JaxMechanicalSystem):
            def __init__(self):
                super().__init__(dof=1)
                self.params = {"mass": 2.0}

            def H(self, q, params=None):
                return jnp.array([[params["mass"]]])

        sys = MassSystem()
        x = jnp.array([0.0, 0.0])
        u = jnp.array([8.0])

        np.testing.assert_allclose(np.asarray(sys.f(x, u)), np.array([0.0, 4.0]))
        np.testing.assert_allclose(
            np.asarray(sys.f(x, u, params={"mass": 4.0})),
            np.array([0.0, 2.0]),
        )


if __name__ == "__main__":
    unittest.main()
