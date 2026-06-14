"""Regression tests for the JaxPendulum / NumpyPendulum subclass relationship."""

import numpy as np
import pytest

from benchmarks.systems.basic import JaxPendulum, NumpyPendulum


def test_jax_pendulum_subclasses_numpy_pendulum():
    assert issubclass(JaxPendulum, NumpyPendulum)


def test_jax_pendulum_module_imports_without_jax_dependency():
    # The class itself must be importable in NumPy-only environments; only
    # calling ``f`` should require JAX. This test does not import jax; it
    # exercises only the constructor and inherited attributes.
    p = JaxPendulum(gravity=9.81, length=1.0, damping=0.05)
    assert p.name == "JaxPendulum"
    assert p.n == 2 and p.m == 1 and p.p == 2
    assert p.gravity == 9.81
    assert p.length == 1.0
    assert p.damping == 0.05


def test_jax_pendulum_f_matches_numpy_pendulum():
    pytest.importorskip("jax")
    import jax.numpy as jnp

    np_sys = NumpyPendulum(gravity=9.81, length=1.0, damping=0.1)
    jx_sys = JaxPendulum(gravity=9.81, length=1.0, damping=0.1)

    x = np.array([0.7, -0.3])
    u = np.array([0.5])

    dx_np = np_sys.f(x, u)
    dx_jx = np.asarray(jx_sys.f(jnp.asarray(x), jnp.asarray(u)))
    np.testing.assert_allclose(dx_jx, dx_np, rtol=1e-9, atol=1e-9)
