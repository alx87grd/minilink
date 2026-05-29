"""Exact linear time-invariant state-space systems."""

import numpy as np

from minilink.compile.jax_utils import array_module
from minilink.core.system import DynamicSystem


def _shape(name, matrix):
    shape = getattr(matrix, "shape", None)
    if shape is None:
        shape = np.shape(matrix)
    if len(shape) != 2:
        raise ValueError(f"{name} must be a 2D array")
    return tuple(shape)


def _identity_like(template, n):
    xp = array_module(template)
    dtype = getattr(template, "dtype", None)
    if dtype is None:
        return xp.eye(n)
    return xp.eye(n, dtype=dtype)


def _zeros_like(template, shape):
    xp = array_module(template)
    dtype = getattr(template, "dtype", None)
    if dtype is None:
        return xp.zeros(shape)
    return xp.zeros(shape, dtype=dtype)


def _check_dimensions(A, B, C, D):
    A_shape = _shape("A", A)
    B_shape = _shape("B", B)
    C_shape = _shape("C", C)
    D_shape = _shape("D", D)

    if A_shape[0] != A_shape[1]:
        raise ValueError("A must be square")
    if B_shape[0] != A_shape[0]:
        raise ValueError("B row count must match A row count")
    if C_shape[1] != A_shape[0]:
        raise ValueError("C column count must match A row count")
    if D_shape != (C_shape[0], B_shape[1]):
        raise ValueError("D shape must be (C rows, B columns)")

    return A_shape[0], B_shape[1], C_shape[0]


class StateSpaceSystem(DynamicSystem):
    """Linear time-invariant system.

    The model is exact LTI state space, not a local linearization container::

        dx = A @ x + B @ u
        y = C @ x + D @ u

    ``A``, ``B``, ``C``, and ``D`` are fixed object attributes and are kept as
    passed so NumPy arrays, JAX arrays, or other matrix objects can drive the
    same equation code. The ``params`` argument accepted by :meth:`f` and
    :meth:`h` is ignored and exists only to match the standard
    :class:`~minilink.core.system.System` signature.
    """

    def __init__(self, A, B, C=None, D=None, *, name="State Space System"):
        self.A = A
        self.B = B
        self.C = _identity_like(A, _shape("A", A)[0]) if C is None else C
        self.D = (
            _zeros_like(B, (_shape("C", self.C)[0], _shape("B", B)[1]))
            if D is None
            else D
        )

        n, m, p = self._check_dimensions()
        super().__init__(
            n=n,
            input_dim=m,
            output_dim=p,
            expose_state=True,
            y_dependencies=() if D is None else ("u",),
        )

        self.name = name

    def _check_dimensions(self):
        """Validate state-space matrix dimensions."""
        return _check_dimensions(self.A, self.B, self.C, self.D)

    def f(self, x, u, t=0.0, params=None):
        return self.A @ x + self.B @ u

    def h(self, x, u, t=0.0, params=None):
        return self.C @ x + self.D @ u
