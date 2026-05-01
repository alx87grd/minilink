"""Basic reusable benchmark systems."""

import numpy as np

from minilink.compile.jax_utils import require_jax_numpy
from minilink.core.system import DynamicSystem


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


class JaxPendulum(NumpyPendulum):
    """Pendulum using JAX ops in `f`.

    Inherits the port layout, parameter attributes, and ``__init__`` of
    :class:`NumpyPendulum`; overrides only the dynamics so ``f`` traces
    through ``jax.numpy``. JAX is loaded lazily on the first ``f`` call so
    importing this module stays free without the ``minilink[jax]`` extra.
    """

    def __init__(self, *, gravity=9.81, length=1.0, damping=0.0):
        super().__init__(gravity=gravity, length=length, damping=damping)
        self.name = "JaxPendulum"

    def f(self, x, u, t=0, params=None):
        jnp = require_jax_numpy()
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
