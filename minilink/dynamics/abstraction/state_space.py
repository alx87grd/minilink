"""Linear state-space systems (time-varying / parameter-dependent and LTI)."""

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
    """Linear state-space system, possibly time-varying or parameter-dependent::

        dx = A(t, params) @ x + B(t, params) @ u
        y  = C(t, params) @ x + D(t, params) @ u

    The matrices are built by the methods :meth:`A`, :meth:`B`, :meth:`C`, and
    :meth:`D`, mirroring :meth:`MechanicalSystem.H` ``(q, params)``. Subclasses
    override :meth:`A` and :meth:`B` (and optionally :meth:`C` / :meth:`D`) to
    assemble the matrices from ``params``. For a constant system, use the
    :class:`LTISystem` convenience subclass.
    """

    def __init__(self, *, n, m, p=None, feedthrough=False, name="State Space System"):
        super().__init__(
            n=n,
            input_dim=m,
            output_dim=n if p is None else p,
            expose_state=True,
            y_dependencies=("u",) if feedthrough else (),
        )
        self.name = name

    def A(self, t=0.0, params=None):
        """State matrix ``A`` as a function of time and parameters."""
        raise NotImplementedError

    def B(self, t=0.0, params=None):
        """Input matrix ``B`` as a function of time and parameters."""
        raise NotImplementedError

    def C(self, t=0.0, params=None):
        """Output matrix ``C``; defaults to full-state output (identity)."""
        return _identity_like(self.B(t, params), self.n)

    def D(self, t=0.0, params=None):
        """Feedthrough matrix ``D``; defaults to zero."""
        return _zeros_like(self.B(t, params), (self.p, self.m))

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        A = self.A(t, params)
        B = self.B(t, params)

        # dx = A x + B u
        return A @ x + B @ u

    def h(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        C = self.C(t, params)
        D = self.D(t, params)

        # y = C x + D u
        return C @ x + D @ u


class LTISystem(StateSpaceSystem):
    """Linear time-invariant system with constant matrices::

        dx = A @ x + B @ u
        y  = C @ x + D @ u

    The matrices are stored exactly as passed so NumPy arrays, JAX arrays, or
    other matrix objects can drive the same equation code. Access them through
    the zero-argument method calls, e.g. ``sys.A()`` (useful for introspection
    such as ``numpy.linalg.eigvals(sys.A())``).
    """

    def __init__(self, A, B, C=None, D=None, *, name="LTI System"):
        self._A = A
        self._B = B
        self._C = _identity_like(A, _shape("A", A)[0]) if C is None else C
        self._D = (
            _zeros_like(B, (_shape("C", self._C)[0], _shape("B", B)[1]))
            if D is None
            else D
        )

        n, m, p = _check_dimensions(self._A, self._B, self._C, self._D)
        super().__init__(n=n, m=m, p=p, feedthrough=D is not None, name=name)

    def A(self, t=0.0, params=None):
        return self._A

    def B(self, t=0.0, params=None):
        return self._B

    def C(self, t=0.0, params=None):
        return self._C

    def D(self, t=0.0, params=None):
        return self._D
