"""Basic reusable benchmark systems."""

from __future__ import annotations

import numpy as np

from minilink.core.framework import DynamicSystem

try:
    import jax.numpy as jnp

    _HAS_JAX = True
except ImportError:  # pragma: no cover - exercised in non-jax environments
    jnp = None
    _HAS_JAX = False


class NumpyPendulum(DynamicSystem):
    """Pendulum using NumPy ops in `f`."""

    def __init__(self, *, gravity=9.81, length=1.0, damping=0.0):
        super().__init__(n=2, m=1, p=2)
        self.name = "NumpyPendulum"
        self.gravity = gravity
        self.length = length
        self.damping = damping

    def f(self, x, u, t=0, params=None):
        q, dq = x[0], x[1]
        ddq = -(self.gravity / self.length) * np.sin(q) - self.damping * dq + u[0]
        return np.array([dq, ddq])


class JaxPendulum(DynamicSystem):
    """Pendulum using JAX ops in `f`."""

    def __init__(self, *, gravity=9.81, length=1.0, damping=0.0):
        if not _HAS_JAX:
            raise ModuleNotFoundError(
                "JAX is required for JaxPendulum. Install `minilink[jax]` "
                "or use NumpyPendulum instead."
            )
        super().__init__(n=2, m=1, p=2)
        self.name = "JaxPendulum"
        self.gravity = gravity
        self.length = length
        self.damping = damping

    def f(self, x, u, t=0, params=None):
        q, dq = x[0], x[1]
        ddq = -(self.gravity / self.length) * jnp.sin(q) - self.damping * dq + u[0]
        return jnp.array([dq, ddq])


def make_pendulum(
    *,
    backend="jax",
    initial_angle=2.0,
    damping=0.0,
    gravity=9.81,
    length=1.0,
):
    """Build a pendulum with non-trivial initial conditions."""
    if backend == "jax":
        sys = JaxPendulum(gravity=gravity, length=length, damping=damping)
    elif backend == "numpy":
        sys = NumpyPendulum(gravity=gravity, length=length, damping=damping)
    else:
        raise ValueError(f"Unsupported backend '{backend}'. Use 'jax' or 'numpy'.")
    sys.x0[0] = initial_angle
    return sys
