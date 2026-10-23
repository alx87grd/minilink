"""Linear state-space systems (time-varying / parameter-dependent and LTI)."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem

###############################################################################
# Public API
###############################################################################


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

    def __init__(
        self,
        *,
        n,
        m,
        p=None,
        feedthrough=False,
        name=None,
        declare_ports=True,
        expose_state=True,
    ):
        super().__init__(
            n=n,
            input_dim=m if declare_ports else None,
            output_dim=(n if p is None else p) if declare_ports else None,
            expose_state=declare_ports and expose_state,
            y_dependencies=("u",) if feedthrough else (),
        )
        if name is not None:
            self.name = name

    def A(self, t=0.0, params=None):
        """State matrix ``A`` as a function of time and parameters."""
        raise NotImplementedError

    def B(self, t=0.0, params=None):
        """Input matrix ``B`` as a function of time and parameters."""
        raise NotImplementedError

    def C(self, t=0.0, params=None):
        """Output matrix ``C``; defaults to full-state output (identity)."""
        return identity_on(self.B(t, params), self.n)

    def D(self, t=0.0, params=None):
        """Feedthrough matrix ``D``; defaults to zero."""
        return zeros_on(self.B(t, params), (self.p, self.m))

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        A = self.A(t, params)
        B = self.B(t, params)

        dx = A @ x + B @ u

        return dx

    def h(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        C = self.C(t, params)
        D = self.D(t, params)

        y = C @ x + D @ u

        return y


class LTISystem(StateSpaceSystem):
    """Linear time-invariant system with constant matrices::

        dx = A @ x + B @ u
        y  = C @ x + D @ u

    The matrices are stored exactly as passed so NumPy arrays, JAX arrays, or
    other matrix objects can drive the same equation code. Access them through
    the zero-argument method calls, e.g. ``sys.A()`` (useful for introspection
    such as ``numpy.linalg.eigvals(sys.A())``). ``declare_ports=False`` leaves
    the ports ``u``, ``y`` and ``x`` undeclared, for a subclass that declares
    its own layout (a compensator ``TransferFunction``); ``expose_state=False``
    declares ``u`` and ``y`` only (a filter is not a plant).

    A model obtained by :func:`~minilink.analysis.linearization.linearize`
    describes deviations about an operating point, which it carries as
    ``xbar``, ``ubar`` and ``ybar`` (the state, the inputs of its ``B``
    columns, and the outputs of its ``C`` rows), so a design made on it knows
    where it applies. A model built from arrays sits at the origin.
    """

    def __init__(
        self,
        A,
        B,
        C=None,
        D=None,
        *,
        name=None,
        declare_ports=True,
        expose_state=True,
        xbar=None,
        ubar=None,
        ybar=None,
    ):
        # The matrices, as passed: C = I and D = 0 when omitted
        self._A = A
        self._B = B
        self._C = identity_on(A, matrix_shape("A", A)[0]) if C is None else C
        self._D = (
            zeros_on(B, (matrix_shape("C", self._C)[0], matrix_shape("B", B)[1]))
            if D is None
            else D
        )

        # The dimensions and the ports. Structural feedthrough only when D has a
        # nonzero entry: an explicit zero D must not create an algebraic y <- u
        # dependency
        n, m, p = dimensions_of(self._A, self._B, self._C, self._D)
        feedthrough = bool(np.any(np.asarray(self._D)))
        super().__init__(
            n=n,
            m=m,
            p=p,
            feedthrough=feedthrough,
            name=name,
            declare_ports=declare_ports,
            expose_state=expose_state,
        )

        # The operating point the deviations are measured from (the origin by
        # default), its output ȳ = C x̄ + D ū unless given
        self.xbar = point_component(xbar, n, "xbar")
        self.ubar = point_component(ubar, m, "ubar")
        self.ybar = (
            np.asarray(self._C, dtype=float) @ self.xbar
            + np.asarray(self._D, dtype=float) @ self.ubar
            if ybar is None
            else point_component(ybar, p, "ybar")
        )

    def A(self, t=0.0, params=None):
        return self._A

    def B(self, t=0.0, params=None):
        return self._B

    def C(self, t=0.0, params=None):
        return self._C

    def D(self, t=0.0, params=None):
        return self._D

    def __str__(self):
        return f"{super().__str__()}\n{matrices_text(self)}"


###############################################################################
# Internal machinery
###############################################################################


def identity_on(template, n):
    """The ``n × n`` identity on the backend (and dtype) of ``template``."""
    xp = array_module(template)
    dtype = getattr(template, "dtype", None)
    if dtype is None:
        return xp.eye(n)
    return xp.eye(n, dtype=dtype)


def zeros_on(template, shape):
    """Zeros of ``shape`` on the backend (and dtype) of ``template``."""
    xp = array_module(template)
    dtype = getattr(template, "dtype", None)
    if dtype is None:
        return xp.zeros(shape)
    return xp.zeros(shape, dtype=dtype)


def matrix_shape(name, matrix):
    """The ``(rows, columns)`` of a matrix of any backend; refuses anything not 2D."""
    shape = getattr(matrix, "shape", None)
    if shape is None:
        shape = np.shape(matrix)
    if len(shape) != 2:
        raise ValueError(f"{name} must be a 2D array")
    return tuple(shape)


def dimensions_of(A, B, C, D):
    """``(n, m, p)`` of a consistent ``(A, B, C, D)``; a mismatched pair raises."""
    A_shape = matrix_shape("A", A)
    B_shape = matrix_shape("B", B)
    C_shape = matrix_shape("C", C)
    D_shape = matrix_shape("D", D)

    if A_shape[0] != A_shape[1]:
        raise ValueError("A must be square")
    if B_shape[0] != A_shape[0]:
        raise ValueError("B row count must match A row count")
    if C_shape[1] != A_shape[0]:
        raise ValueError("C column count must match A row count")
    if D_shape != (C_shape[0], B_shape[1]):
        raise ValueError("D shape must be (C rows, B columns)")

    return A_shape[0], B_shape[1], C_shape[0]


def point_component(value, dim, name):
    """One component of the operating point as a float vector of length ``dim`` (zeros when ``None``)."""
    if value is None:
        return np.zeros(dim)
    point = np.asarray(value, dtype=float).reshape(-1)
    if point.shape != (dim,):
        raise ValueError(f"{name} must have shape ({dim},), got {point.shape}")
    return point


def matrices_text(sys):
    """``A``, ``B``, ``C`` and ``D``, one aligned block each."""
    blocks = []
    for name, M in (("A", sys.A()), ("B", sys.B()), ("C", sys.C()), ("D", sys.D())):
        prefix = f"  {name} = "
        values = np.asarray(M, dtype=float) + 0.0  # -0.0 prints as 0.0
        blocks.append(
            prefix
            + np.array2string(values, precision=4, suppress_small=True, prefix=prefix)
        )
    return "\n".join(blocks)
