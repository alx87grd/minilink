"""Linear state-space systems: time-varying, LTI, and the linearization of a system about an operating point."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem, fastest_time_constant

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
        state_dim,
        input_dim,
        output_dim=None,
        feedthrough=False,
        name=None,
        declare_ports=True,
        expose_state=True,
    ):
        n = state_dim
        p = n if output_dim is None else output_dim
        super().__init__(
            state_dim=n,
            input_dim=input_dim if declare_ports else None,
            output_dim=p if declare_ports else None,
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

    @property
    def solver_info(self) -> dict:
        """The solver hints, the smallest time constant ``1 / max |λᵢ(A)|`` at ``t = 0``, read from the current ``params``."""
        eigenvalues = np.linalg.eigvals(np.asarray(self.A(), dtype=float))

        info = self.own_solver_info
        info["smallest_time_constant"] = fastest_time_constant(eigenvalues)

        return info

    @solver_info.setter
    def solver_info(self, value: dict):
        self.own_solver_info = dict(value)

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


class ConstantMatrices:
    """The matrices ``A``, ``B``, ``C``, ``D`` of a model that stores them in its ``params``.

    Each is a method ``A(t, params)``, as on :class:`StateSpaceSystem`, read
    from the tree passed (the block's own ``params`` when ``None``), so a
    tree passed to ``f`` or ``h`` moves them.
    """

    def A(self, t=0.0, params=None):
        params = self.params if params is None else params
        return params["A"]

    def B(self, t=0.0, params=None):
        params = self.params if params is None else params
        return params["B"]

    def C(self, t=0.0, params=None):
        params = self.params if params is None else params
        return params["C"]

    def D(self, t=0.0, params=None):
        params = self.params if params is None else params
        return params["D"]


class OperatingPoint:
    """The operating point ``(x̄, ū, ȳ)`` of a model that stores it in its ``params``.

    ``x_bar``, ``u_bar`` and ``y_bar`` read ``params``; :meth:`set_nominal_point`
    makes the point the model's nominal state and input.
    """

    @property
    def x_bar(self):
        """The operating point's state ``x̄`` (``params["x_bar"]``)."""
        return self.params["x_bar"]

    @property
    def u_bar(self):
        """The operating point's input ``ū`` (``params["u_bar"]``)."""
        return self.params["u_bar"]

    @property
    def y_bar(self):
        """The system's output at the operating point, ``ȳ = h(x̄, ū)`` (``params["y_bar"]``)."""
        return self.params["y_bar"]

    def set_nominal_point(self):
        """Make ``x̄`` the nominal state and ``ū`` the inputs' nominal values, each input port its share (each signal keeps its own copy)."""
        u_bar = self.params["u_bar"]

        self.state.nominal_value = self.params["x_bar"]
        for port_id, port in self.inputs.items():
            port.nominal_value = u_bar[self.get_input_port_slice(port_id)]


class LTISystem(ConstantMatrices, StateSpaceSystem):
    """Linear time-invariant system with constant matrices::

        dx = A @ x + B @ u
        y  = C @ x + D @ u

    The matrices are the block's ``params`` (``{"A", "B", "C", "D"}``), stored
    exactly as passed so NumPy arrays, JAX arrays, or other matrix objects can
    drive the same equation code; a tree passed to ``f`` or ``h`` moves them.
    Access them through the zero-argument method calls, e.g. ``sys.A()``
    (useful for introspection such as ``numpy.linalg.eigvals(sys.A())``). ``declare_ports=False`` leaves
    the ports ``u``, ``y`` and ``x`` undeclared, for a subclass that declares
    its own layout (a compensator ``TransferFunction``); ``expose_state=False``
    declares ``u`` and ``y`` only (a filter is not a plant).

    The model of a system about an operating point is a
    :class:`LinearizedSystem`, what
    :func:`~minilink.analysis.linearization.linearize` returns; its
    :meth:`~LinearizedSystem.deviation_model` is this class, in the
    deviations from that point.
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
    ):
        # The matrices, as passed: C = I and D = 0 when omitted
        C = identity_on(A, matrix_shape("A", A)[0]) if C is None else C
        D = (
            zeros_on(B, (matrix_shape("C", C)[0], matrix_shape("B", B)[1]))
            if D is None
            else D
        )

        # The dimensions and the ports. Structural feedthrough only when D has a
        # nonzero entry: an explicit zero D must not create an algebraic y <- u
        # dependency
        n, m, p = dimensions_of(A, B, C, D)
        feedthrough = bool(np.any(np.asarray(D)))
        super().__init__(
            state_dim=n,
            input_dim=m,
            output_dim=p,
            feedthrough=feedthrough,
            name=name,
            declare_ports=declare_ports,
            expose_state=expose_state,
        )

        # Parameters: the four matrices
        self.params = {"A": A, "B": B, "C": C, "D": D}

    def __str__(self):
        return f"{super().__str__()}\n{matrices_text(self)}"

    def _repr_latex_(self):
        return lti_latex()


class LinearizedSystem(OperatingPoint, ConstantMatrices, StateSpaceSystem):
    """The first-order model of a system about an operating point ``(x̄, ū)``, in the system's own coordinates::

        dx = f(x̄, ū) + A @ (x - x̄) + B @ (u - ū)
        y  = h(x̄, ū) + C @ (x - x̄) + D @ (u - ū)

    What :func:`~minilink.analysis.linearization.linearize` returns on a
    continuous system. Its state, inputs and outputs are the system's own,
    with its labels and units, so the model closes a loop wherever the system
    does (``lqr(lin, Q, R) @ lin`` regulates to ``x̄``) and a simulation of it
    is the system's linear prediction near ``x̄``. ``A, B, C, D`` are the
    Jacobians, read by every analysis and design verb; the drift
    ``dx_bar = f(x̄, ū)`` is zero at an equilibrium, and the verbs warn when it
    is not. Everything is in ``params`` (``{"A", "B", "C", "D", "x_bar",
    "u_bar", "y_bar", "dx_bar"}``), numbers only, so JAX traces them;
    ``x_bar``, ``u_bar``, ``y_bar`` read them. The nominal state is ``x̄``, the
    input ports' nominal ``ū``. :meth:`deviation_model` is the pure
    ``LTISystem`` in the deviations ``Δx = x − x̄``, ``Δu = u − ū``.
    """

    def __init__(self, A, B, C, D, *, x_bar, u_bar, y_bar, dx_bar, name=None):
        # The dimensions and the ports u -> y, x, fed through only when D ≠ 0
        n, m, p = dimensions_of(A, B, C, D)
        feedthrough = bool(np.any(np.asarray(D)))
        super().__init__(
            state_dim=n,
            input_dim=m,
            output_dim=p,
            feedthrough=feedthrough,
            name=name,
        )

        # Parameters: the Jacobians and the operating point, with the system's
        # output ȳ = h(x̄, ū) and drift f(x̄, ū) there
        self.params = {
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "x_bar": point_component(x_bar, n, "x_bar"),
            "u_bar": point_component(u_bar, m, "u_bar"),
            "y_bar": point_component(y_bar, p, "y_bar"),
            "dx_bar": point_component(dx_bar, n, "dx_bar"),
        }

        # The nominal point: x̄, and ū on the input
        self.set_nominal_point()

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        A = params["A"]
        B = params["B"]
        x_bar = params["x_bar"]
        u_bar = params["u_bar"]
        dx_bar = params["dx_bar"]

        dx = dx_bar + A @ (x - x_bar) + B @ (u - u_bar)

        return dx

    def h(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        C = params["C"]
        D = params["D"]
        x_bar = params["x_bar"]
        u_bar = params["u_bar"]
        y_bar = params["y_bar"]

        y = y_bar + C @ (x - x_bar) + D @ (u - u_bar)

        return y

    def deviation_model(self):
        """The ``LTISystem`` ``(A, B, C, D)`` in the deviations ``Δx = x − x̄``, ``Δu = u − ū``, ``Δy = y − ȳ``, its labels with a Δ."""
        model = LTISystem(
            self.A(), self.B(), self.C(), self.D(), name=f"{self.name} (deviations)"
        )
        relabel_deviations(model, self)

        return model

    def __str__(self):
        return f"{super().__str__()}\n{matrices_text(self)}\n{point_text(self)}"

    def _repr_latex_(self):
        return linearized_latex()


class DiscreteLTISystem(ConstantMatrices, StepSystem):
    """Linear time-invariant difference equation with constant matrices::

        x_{k+1} = A_d @ x_k + B_d @ u_k
        y_k     = C_d @ x_k + D_d @ u_k

    A tick law: its poles are read against the unit circle (``|z| < 1`` is
    stable), its frequency response ``G(e^{jω})`` per tick. Seconds come only
    from a clock: ``model % Δ`` runs it every ``Δ`` seconds, and the analysis
    reads that clock (``G(e^{jωΔ})`` in rad/s). The matrices are its ``params``
    (``{"A", "B", "C", "D"}``). The model of a step or sampled system about an
    operating point is a :class:`LinearizedStepSystem`, and its
    :meth:`~LinearizedStepSystem.deviation_model` is this class.
    """

    def __init__(self, A, B, C=None, D=None, *, name=None):
        # The matrices: C = I and D = 0 when omitted
        C = identity_on(A, matrix_shape("A", A)[0]) if C is None else C
        D = (
            zeros_on(B, (matrix_shape("C", C)[0], matrix_shape("B", B)[1]))
            if D is None
            else D
        )
        n, m, p = dimensions_of(A, B, C, D)
        feedthrough = bool(np.any(np.asarray(D)))
        super().__init__(
            state_dim=n,
            input_dim=m,
            output_dim=p,
            y_dependencies=("u",) if feedthrough else (),
        )
        if name is not None:
            self.name = name

        # Parameters: the four matrices
        self.params = {"A": A, "B": B, "C": C, "D": D}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        A_d = params["A"]
        B_d = params["B"]

        x_next = A_d @ x + B_d @ u

        return x_next

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        C_d = params["C"]
        D_d = params["D"]

        y = C_d @ x + D_d @ u

        return y

    def __str__(self):
        return f"{super().__str__()}\n  per tick\n{matrices_text(self)}"


class LinearizedStepSystem(OperatingPoint, ConstantMatrices, StepSystem):
    """The first-order model of a step or sampled system about an operating point ``(x̄, ū)``, in the system's own coordinates::

        x_{k+1} = F(x̄, ū) + A_d @ (x_k - x̄) + B_d @ (u_k - ū)
        y_k     = h(x̄, ū) + C_d @ (x_k - x̄) + D_d @ (u_k - ū)

    The twin of :class:`LinearizedSystem` that
    :func:`~minilink.analysis.linearization.linearize` returns on a tick law,
    a block on a clock or a sampled loop, ``F`` its step. ``x_next_bar =
    F(x̄, ū)`` equals ``x̄`` at an equilibrium. ``params`` holds ``{"A", "B",
    "C", "D", "x_bar", "u_bar", "y_bar", "x_next_bar"}``; a tick law, as
    :class:`DiscreteLTISystem` (``model % Δ`` puts it on a clock), and
    :meth:`deviation_model` is that pure difference equation in the deviations.
    """

    def __init__(self, A, B, C, D, *, x_bar, u_bar, y_bar, x_next_bar, name=None):
        # The dimensions and the ports u -> y, fed through only when D ≠ 0
        n, m, p = dimensions_of(A, B, C, D)
        feedthrough = bool(np.any(np.asarray(D)))
        super().__init__(
            state_dim=n,
            input_dim=m,
            output_dim=p,
            y_dependencies=("u",) if feedthrough else (),
        )
        if name is not None:
            self.name = name

        # Parameters: the Jacobians and the operating point, with the system's
        # output ȳ = h(x̄, ū) and next state F(x̄, ū) there
        self.params = {
            "A": A,
            "B": B,
            "C": C,
            "D": D,
            "x_bar": point_component(x_bar, n, "x_bar"),
            "u_bar": point_component(u_bar, m, "u_bar"),
            "y_bar": point_component(y_bar, p, "y_bar"),
            "x_next_bar": point_component(x_next_bar, n, "x_next_bar"),
        }

        # The nominal point: x̄, and ū on the input
        self.set_nominal_point()

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        A_d = params["A"]
        B_d = params["B"]
        x_bar = params["x_bar"]
        u_bar = params["u_bar"]
        x_next_bar = params["x_next_bar"]

        x_next = x_next_bar + A_d @ (x - x_bar) + B_d @ (u - u_bar)

        return x_next

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        C_d = params["C"]
        D_d = params["D"]
        x_bar = params["x_bar"]
        u_bar = params["u_bar"]
        y_bar = params["y_bar"]

        y = y_bar + C_d @ (x - x_bar) + D_d @ (u - u_bar)

        return y

    def deviation_model(self):
        """The ``DiscreteLTISystem`` ``(A_d, B_d, C_d, D_d)`` in the deviations from the point, its labels with a Δ."""
        model = DiscreteLTISystem(
            self.A(),
            self.B(),
            self.C(),
            self.D(),
            name=f"{self.name} (deviations)",
        )
        relabel_deviations(model, self)

        return model

    def __str__(self):
        return (
            f"{super().__str__()}\n  per tick\n{matrices_text(self)}\n"
            f"{point_text(self)}"
        )


# The linear models a design reads as they are, and those of them taken about a point
LINEARIZATIONS = (LinearizedSystem, LinearizedStepSystem)
LINEAR_MODELS = (LTISystem, DiscreteLTISystem, *LINEARIZATIONS)


def design_point(model):
    """The point a design on ``model`` holds: ``(x̄, ū, ȳ)`` of a linearization, the origin of a linear model."""
    if isinstance(model, LINEARIZATIONS):
        return model.x_bar, model.u_bar, model.y_bar
    return np.zeros(model.n), np.zeros(model.m), np.zeros(model.p)


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


def require_arrays(**matrices):
    """Refuse a model's matrix method (``sys.A``) given where its matrix (``sys.A()``) goes."""
    for name, matrix in matrices.items():
        if callable(matrix):
            raise TypeError(
                f"{name} is a method of the model, not its matrix: call it, "
                f"sys.{name}() (sys.{name}(t, params) on a time-varying model)"
            )


def matrix_shape(name, matrix):
    """The ``(rows, columns)`` of a matrix of any backend; refuses anything not 2D."""
    require_arrays(**{name: matrix})
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


def point_text(sys):
    """The operating point a linearization carries, one aligned line per vector."""
    lines = []
    for name in ("x_bar", "u_bar", "y_bar", "dx_bar", "x_next_bar"):
        if name not in sys.params:
            continue
        prefix = f"  {name} = "
        values = np.asarray(sys.params[name], dtype=float) + 0.0  # -0.0 prints as 0.0
        lines.append(
            prefix
            + np.array2string(values, precision=4, suppress_small=True, prefix=prefix)
        )
    return "\n".join(lines)


def relabel_deviations(model, linearization):
    """Label ``model``'s state, input and output as the deviations of ``linearization``'s (``Δθ``), the units kept."""
    input_labels, input_units = linearization.get_all_input_labels_and_units()
    output = linearization.outputs["y"]

    model.state.labels = deviation_labels(linearization.state.labels)
    model.state.units = list(linearization.state.units)
    model.inputs["u"].labels = deviation_labels(input_labels)
    model.inputs["u"].units = list(input_units)
    model.outputs["y"].labels = deviation_labels(output.labels)
    model.outputs["y"].units = list(output.units)
    if "x" in model.outputs:
        model.outputs["x"].labels = list(model.state.labels)
        model.outputs["x"].units = list(model.state.units)


def deviation_labels(labels):
    """``θ`` -> ``Δθ``: the label of a deviation from the operating point."""
    return [f"Δ{label}" for label in labels]


def lti_latex():
    """The state-space equations in LaTeX."""
    return (
        r"\begin{align*}"
        r"\dot{x} &= A x + B u \\"
        r"y &= C x + D u"
        r"\end{align*}"
    )


def linearized_latex():
    """The linearized equations about the operating point in LaTeX."""
    return (
        r"\begin{align*}"
        r"\dot{x} &= f(\bar{x}, \bar{u}) + A (x - \bar{x}) + B (u - \bar{u}) \\"
        r"y &= h(\bar{x}, \bar{u}) + C (x - \bar{x}) + D (u - \bar{u})"
        r"\end{align*}"
    )
