"""A system on a clock: ``x(kh + h) = F(x(kh), u(kh), kh)``, the sampled-data kind of the step algebra."""

from __future__ import annotations

import copy

import numpy as np

from minilink.core.backends import array_module, require_jax
from minilink.core.compile.evaluators.stepping import (
    step_body,
    step_path,
    step_with_cost,
)
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import (
    AUTO_DT_SCALE,
    DEFAULT_SMALLEST_TIME_CONSTANT,
    DynamicSystem,
    System,
)

SAMPLED_SOLVERS = ("rk4", "euler", "exact")

# What a clock makes of its source: the computer's law (block % dt), a discrete-time
# model (discretize), or a loop's continuous plant on the base clock
SAMPLED_KINDS = ("law", "model", "plant")


###############################################################################
# Public API
###############################################################################


class SampledSystem(StepSystem):
    """
    A system on a clock of period :attr:`sample_period`, its input held over each period.

    One equation with two faces. The seconds face is the equation, the
    sampled-data model of the textbook::

        x(t + h) = update(x(t), u(t), t),     y(t) = h(x(t), u(t), t)

    at the sampling instants ``t``, every output port a function in seconds as
    on any system. The tick face serves the step algebra:
    ``step(x, u, k) = update(x, u, k h)`` and ``h(x, u, k)`` likewise, the clock's
    origin at zero. A sampled diagram hands the seconds face its own time
    ``t_k = t_start + k Δ``.

    A view of ``source``, never a container: its ``params``, ports and labels
    are the source's own, and ``sample_period`` is structural. A continuous
    source is integrated over the period by ``solver`` (``"rk4"``, ``"euler"``,
    or ``"exact"`` for a linear model, the matrix exponential), ``substeps``
    steps per period; a static source is read at the sampling instants; a
    tick source (a :class:`StepSystem`) keeps its own count, so it has no
    seconds face.

    ``kind`` says what the clock made of the source: ``"law"`` the computer's
    difference equation (``sys % dt``, one Euler step per period, never
    integrated inside it), ``"model"`` a discrete-time model for the discrete
    tools (:func:`~minilink.analysis.discretization.discretize`), ``"plant"``
    a loop's continuous plant on the base clock (``(ctl % dt) @ plant`` wraps
    it; the user never builds one). The plant alone is integrated inside the
    loop's periods: a loop's dense run, substeps and host runner are its.
    """

    def __init__(
        self,
        source: System,
        sample_period: float,
        *,
        solver: str = "rk4",
        substeps: int = 1,
        kind: str = "model",
    ):
        require_sampleable(source, solver, substeps)
        require_kind(kind)

        # The source and the clock
        self.source = source
        self.sample_period = positive_period(sample_period)
        self.solver = solver
        self.substeps = int(substeps)
        self.kind = kind

        # The System shell: a static source has no state (n = 0); the shell's zero
        # start would write through to the source, whose own start is put back
        x0 = np.array(source.x0, dtype=float)
        System.__init__(self, source.n)
        source.x0 = x0
        self.name = f"Discretized({source.name})"
        self.id = source.id
        self.params = None

        # State: the source's (its x0 a view), labels, units, bounds and nominal value
        self.state = source.state

        # Ports: the source's own, their functions included
        self.inputs = source.inputs
        self.outputs = source.outputs

        # Graphics: the source's camera hints (its geometry and poses are read through it)
        self.camera_target = source.camera_target
        self.camera_plot_axes = source.camera_plot_axes
        self.camera_scale = source.camera_scale
        self.camera_follow_frame = source.camera_follow_frame
        self.camera_priority = source.camera_priority

    @property
    def params(self) -> dict:
        """The source's live ``params`` while no override is assigned, else the override."""
        if self.params_override is None:
            return self.source.params

        return self.params_override

    @params.setter
    def params(self, value: dict | None):
        self.params_override = value

    @property
    def x0(self):
        """The source's ``x0``, read live; setting it writes through to the source, which owns it."""
        return np.asarray(self.source.x0, dtype=float)

    @x0.setter
    def x0(self, value):
        self.source.x0 = np.array(value, dtype=float)

    @property
    def solver_info(self) -> dict:
        """The source's solver hints, read live: they describe the source."""
        return self.source.solver_info

    @solver_info.setter
    def solver_info(self, value: dict):
        # The System shell's defaults: the view's hints are its source's
        return

    def refresh(self):
        """Refresh the source, then read again the structure the view took from it: its size, state and ports."""
        source = self.source
        source.refresh()

        self.n = source.n
        self.state = source.state
        self.inputs = source.inputs
        self.outputs = source.outputs

    @property
    def counts(self) -> bool:
        """Whether the block counts its own ticks: a tick source, which reads ``k``, never seconds."""
        return isinstance(self.source, StepSystem)

    @property
    def is_random(self):
        """Whether the source draws random numbers from a seed in its ``params``."""
        return self.source.is_random

    def realize(self, key):
        """The params of one realization: the source's, under ``key``."""
        return self.source.realize(key)

    def update(self, x, u, t, params=None):
        """
        The state one sample period later, ``x(t + h)``, from ``x(t)`` and the held ``u(t)``.

        Parameters
        ----------
        x : array of shape (n,)
        u : array of shape (m,)
        t : float
            The sampling instant, in seconds.
        params : dict, optional
            ``None`` means "use ``self.params``".
        """
        params = self.params if params is None else params

        source = self.source
        dt = self.sample_period
        solver = self.solver
        substeps = self.substeps

        require_seconds_face(self)
        if not isinstance(source, DynamicSystem) or source.n == 0:
            return x

        if solver == "exact":
            return exact_update(source, x, u, t, dt, params)

        def f(x, u, t):
            return source.f(x, u, t, params)

        # x(t + Δt) = Φ(x(t), u(t), t), the input held over [t, t + Δt]
        x_next = step_body(f, x, u, u, t, dt, solver, substeps)

        return x_next

    def update_with_cost(self, x, u, t, params, rate):
        """
        ``(x(t + h), c)``: :meth:`update` with a stage cost integrated beside the state.

        ``rate(x, u, τ)`` is the cost's integrand along the period, the input
        held; the state is :meth:`update`'s to the bit, ``c = ∫ rate dτ`` by the
        same solver and substeps.
        """
        params = self.params if params is None else params

        source = self.source
        dt = self.sample_period
        solver = self.solver
        substeps = self.substeps

        require_resolved(self)

        def f(x, u, t):
            return source.f(x, u, t, params)

        # [x(t + Δt); c] = Φ([x(t); 0], u(t), t), the cost a state of its own
        x_next, c = step_with_cost(f, rate, x, u, t, dt, solver, substeps)

        return x_next, c

    @property
    def resolves(self) -> bool:
        """Whether a continuous source is integrated inside the period, so its substeps can be kept: a model or a plant, never a law."""
        source = self.source
        continuous = isinstance(source, DynamicSystem) and source.n > 0

        return continuous and self.solver != "exact" and self.kind != "law"

    @property
    def is_plant(self) -> bool:
        """Whether this is a loop's continuous plant on the base clock."""
        return self.kind == "plant"

    def with_steps(self, substeps=None, solver=None):
        """This view with another integration step: ``substeps`` per period and ``solver`` (``None`` keeps each)."""
        substeps = self.substeps if substeps is None else int(substeps)
        solver = self.solver if solver is None else solver
        require_sampleable(self.source, solver, substeps)

        stepped = copy.copy(self)
        stepped.substeps = substeps
        stepped.solver = solver

        return stepped

    def update_path(self, x, u, t, params=None):
        """
        The state at the end of every substep of :meth:`update`: ``(substeps, n)``.

        The last row is ``x(t + h)`` from the same steps; the dense run of a
        loop (``compute_trajectory``) keeps them.
        """
        params = self.params if params is None else params

        source = self.source
        dt = self.sample_period
        solver = self.solver
        substeps = self.substeps

        require_resolved(self)

        def f(x, u, t):
            return source.f(x, u, t, params)

        # x(t + j Δt / s) for j = 1 … s, the input held over [t, t + Δt]
        x_path = step_path(f, x, u, u, t, dt, solver, substeps)

        return x_path

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        dt = self.sample_period

        if self.counts:
            return self.source.step(x, u, k, params)

        t_k = k * dt

        # x_{k+1} = update(x_k, u_k, t_k), t_k = k Δt
        x_next = self.update(x, u, t_k, params)

        return x_next

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        h = self.source.h
        dt = self.sample_period

        if self.counts:
            return h(x, u, k, params)

        t_k = k * dt

        # y_k = h(x_k, u_k, t_k), the source's output map sampled at t_k = k Δt
        y = h(x, u, t_k, params)

        return y

    # Display: the source's drawing, its poses at the time it is read

    def get_kinematic_geometry(self):
        return self.source.get_kinematic_geometry()

    def tf(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return self.source.tf(x, u, t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        params = self.params if params is None else params
        return self.source.get_dynamic_geometry(x, u, t, params)


class NativeSampledSystem(SampledSystem):
    """
    A native sampled law: a system on a clock with no source, its equation written in seconds.

    A subclass writes the seconds face, ``update(x, u, t)`` the state one
    period after the sampling instant ``t``, and declares its output ports as
    any system does (``add_output_port(id, function=…)``, each function in
    seconds); the tick face is the base class's, ``step(x, u, k) =
    update(x, u, k h)``. Its ``params``, ``x0`` and solver hints are its own.
    Its equation needs its period, so ``sample_period`` is set at
    construction, and ``law % dt`` is refused: it is already on its clock
    (rule 1 of the loop's grammar).
    """

    def __init__(self, n=0, sample_period=None):
        # No source: the clock alone, set at construction
        self.source = None
        self.solver = None
        self.substeps = 1
        self.kind = "law"
        self.sample_period = positive_period(require_period(sample_period))

        # The System shell: its own state, ports, parameters and hints
        System.__init__(self, n)

    @property
    def params(self) -> dict:
        """The law's own numbers."""
        return self.params_override

    @params.setter
    def params(self, value: dict | None):
        self.params_override = {} if value is None else value

    @property
    def x0(self):
        """The law's own initial state."""
        return self.__dict__["own_x0"]

    @x0.setter
    def x0(self, value):
        self.__dict__["own_x0"] = np.array(value, dtype=float)

    @property
    def solver_info(self) -> dict:
        """The law's own solver hints."""
        return self.__dict__["own_solver_info"]

    @solver_info.setter
    def solver_info(self, value: dict):
        self.__dict__["own_solver_info"] = value

    def refresh(self):
        """Nothing derived to refresh: the law owns its structure."""
        return

    @property
    def counts(self) -> bool:
        """A native law reads seconds, never its tick count."""
        return False

    @property
    def is_random(self):
        """Whether the law draws: a ``seed`` in its ``params`` that is not ``None``."""
        return self.params.get("seed") is not None

    def realize(self, key):
        """The law's ``params``: a native law holds no draw of its own to realize."""
        return self.params

    @property
    def resolves(self) -> bool:
        """A native law has no source integrated inside the period."""
        return False

    def update(self, x, u, t, params=None):
        """The state one sample period after ``t``: the law's own equation."""
        raise NotImplementedError(
            f"{type(self).__name__} is a native sampled law: write update(x, u, t, params)"
        )

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        y_port = next(iter(self.outputs.values()))
        dt = self.sample_period

        # y_k = h(x_k, u_k, t_k), the first output port sampled at t_k = k Δt
        t_k = k * dt
        y = y_port.compute(x, u, t_k, params)

        return y

    def __mod__(self, dt: float):
        """Refuse: a sampled-data law takes its period at construction, it is already on its clock."""
        raise TypeError(
            f"{self.name} is a sampled-data law on its own clock of "
            f"{self.sample_period:g} s: its period is set at construction "
            "(sample_period=), never by %"
        )

    # Display: the System shell's

    def get_kinematic_geometry(self):
        return System.get_kinematic_geometry(self)

    def tf(self, x, u, t=0, params=None):
        return System.tf(self, x, u, t, params)

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return System.get_dynamic_geometry(self, x, u, t, params)


def sampled_substeps(plant, sample_period) -> int:
    """
    The integration steps per period of a plant on a clock: the continuous simulator's step rule.

    ``Δt = AUTO_DT_SCALE · τ_min``, the plant's smallest time constant (the
    catalog default 1 ms when it declares none), so ``n = ⌈Δ / Δt⌉``.
    """
    tau = plant.solver_info.get(
        "smallest_time_constant", DEFAULT_SMALLEST_TIME_CONSTANT
    )

    return steps_per_period(sample_period, AUTO_DT_SCALE * tau)


def steps_per_period(sample_period, dt) -> int:
    """``n = ⌈Δ / Δt⌉``: the steps of at most ``dt`` that make up one period (a multiple's round)."""
    dt = positive_period(dt)

    # n = ⌈Δ / Δt⌉, a multiple of Δt landing on its round despite the division
    n = int(np.ceil(float(sample_period) / dt - 1e-9))

    return max(1, n)


###############################################################################
# Internal machinery
###############################################################################


def exact_update(source, x, u, t, dt, params):
    """``x(t + h) = A_d x + B_d u``, the exact hold of a linear model."""
    xp = array_module(x, u)

    n = source.n
    m = source.m

    A = xp.asarray(source.A(t, params))
    B = xp.asarray(source.B(t, params))

    # [A_d B_d; 0 I] = exp([A B; 0 0] Δt)
    # fmt: off
    M = xp.block([
        [A, B],
        [xp.zeros((m, n)), xp.zeros((m, m))],
    ])
    # fmt: on
    E = expm(M * dt)
    A_d = E[:n, :n]
    B_d = E[:n, n:]

    # x_{k+1} = A_d x_k + B_d u_k
    x_next = A_d @ x + B_d @ u

    return x_next


def expm(M):
    """The matrix exponential on the array's backend."""
    xp = array_module(M)
    if xp is np:
        # Lazy import: SciPy's linear algebra loads with the first exact hold
        import scipy.linalg

        return scipy.linalg.expm(M)

    jax = require_jax()

    return jax.scipy.linalg.expm(M)


def require_sampleable(source, solver, substeps) -> None:
    """Refuse a source that is not a system, a solver the clock does not have, or a bad substep count."""
    if not isinstance(source, System):
        raise TypeError(f"a clock applies to a System, got {type(source).__name__}")
    if isinstance(source, SampledSystem):
        raise TypeError(
            f"{source.name} is already on a clock of {source.sample_period:g} s; "
            "apply the clock to its source"
        )
    if solver not in SAMPLED_SOLVERS:
        raise ValueError(
            f"Unknown solver {solver!r}; expected one of {list(SAMPLED_SOLVERS)!r}."
        )
    if int(substeps) != substeps or substeps < 1:
        raise ValueError(f"substeps must be a positive integer, got {substeps!r}")
    if isinstance(source, StepSystem) and substeps != 1:
        raise ValueError(
            f"{source.name} is a tick law: it takes no integration substeps"
        )
    if solver == "exact" and not (
        callable(getattr(source, "A", None)) and callable(getattr(source, "B", None))
    ):
        raise TypeError(
            f"solver='exact' needs a linear model with A and B, got {source.name}; "
            "pass linearize(system) or use solver='rk4'"
        )
    if "dt" in (source.params or {}):
        raise ValueError(
            f"the params of {source.name} carry 'dt' = {source.params['dt']!r}, but "
            "the sample period is the clock's, which f never receives; remove 'dt' "
            "from the params and call discretize(system, dt)."
        )


def require_kind(kind) -> None:
    """Refuse a kind of sampled view the clock does not make."""
    if kind not in SAMPLED_KINDS:
        raise ValueError(f"kind must be one of {list(SAMPLED_KINDS)!r}, got {kind!r}")


def require_resolved(system) -> None:
    """Refuse the substeps of a block that integrates nothing inside its period."""
    if not system.resolves:
        raise TypeError(
            f"{system.name} integrates no continuous state inside its period: it has "
            "no substeps to keep"
        )


def require_seconds_face(system) -> None:
    """Refuse the seconds face of a tick source: it counts its ticks, it reads no seconds."""
    if system.counts:
        raise TypeError(
            f"{system.source.name} is a tick law: it reads its count k, never seconds; "
            "call step(x, u, k)"
        )


def positive_period(sample_period: float) -> float:
    """``sample_period`` as a float, refused unless positive."""
    sample_period = float(sample_period)
    if sample_period <= 0.0:
        raise ValueError(f"sample_period must be positive, got {sample_period}")
    return sample_period


def require_period(sample_period):
    """Refuse a sampled-data law built with no period: its equation needs it."""
    if sample_period is None:
        raise TypeError(
            "a sampled-data law takes its period at construction: "
            "NativeSampledSystem(n, sample_period=dt)"
        )

    return sample_period
