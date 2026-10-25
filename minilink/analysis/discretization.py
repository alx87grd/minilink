"""Discretization: a continuous-time system as a discrete-time step model."""

from __future__ import annotations

import numpy as np
import scipy.linalg

from minilink.core.backends import array_module, require_jax
from minilink.core.compile.evaluators.stepping import step_body
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem

DISCRETIZE_SOLVERS = ("rk4", "euler", "exact")


###############################################################################
# Public API
###############################################################################


def discretize(
    system: DynamicSystem,
    dt: float,
    *,
    solver: str = "rk4",
    substeps: int = 1,
) -> StepSystem:
    """
    The plant one sample period later: a :class:`StepSystem` ``x_{k+1} = Φ(x_k, u_k, k)``.

    The input is held over each period ``[k dt, (k + 1) dt]``. ``solver`` is
    ``"rk4"`` or ``"euler"``, taking ``substeps`` steps per period, or
    ``"exact"`` for a linear model, the matrix exponential
    ``x_{k+1} = e^{A dt} x_k + ∫₀^{dt} e^{A s} ds B u_k``. The step model reads
    the source's live ``params`` (``step(x, u, k, params)`` overrides them), and
    the sample time is ``dt`` alone: the source's ``params`` may not carry a
    ``"dt"`` key. The wrapper starts from the source's ``x0`` and keeps its
    state, input and ``y`` metadata.
    """
    require_discretizable(system, solver, substeps)
    require_dt_free(system)

    # The exact hold of a linear model: the matrix exponential, once
    if solver == "exact":
        return ExactDiscretization(system, dt)

    return Discretization(system, dt, solver=solver, substeps=substeps)


class Discretization(StepSystem):
    """
    A continuous :class:`DynamicSystem` sampled every :attr:`dt`, its input held.

    ``step`` is the step model's one step body (``rk4`` or ``euler``,
    ``substeps`` per period), the one the compiled evaluator steps, on the source's ``f``; the
    sample time lives here and never reaches ``f``. The wrapper starts from the
    source's ``x0`` and copies its state, input and ``y`` labels, units, bounds
    and nominal values.
    """

    def __init__(
        self,
        source: DynamicSystem,
        dt: float,
        *,
        solver: str = "rk4",
        substeps: int = 1,
    ):
        # The source
        self.source = source

        # The source's y feedthrough, which the standard y port copies
        y_deps = y_dependencies_of(source)

        super().__init__(
            n=source.n,
            input_dim=source.m if source.m else None,
            output_dim=source.p if source.p else None,
            expose_state="x" in source.outputs,
            y_dependencies=y_deps,
        )
        self.name = f"Discretized({source.name})"

        # The sample time and the step model's knobs; the source's params, live
        self.params = None
        self.dt = positive_dt(dt)
        self.solver = solver
        self.substeps = int(substeps)

        # State: the source's initial state, labels, units, bounds and nominal value
        state = source.state
        self.x0 = np.array(source.x0, dtype=float)
        self.state.labels = list(state.labels)
        self.state.units = list(state.units)
        self.state.lower_bound = np.array(state.lower_bound, dtype=float)
        self.state.upper_bound = np.array(state.upper_bound, dtype=float)
        self.state.nominal_value = np.array(state.nominal_value, dtype=float)

        # Input port: each source input port's metadata, stacked in flat-u order
        if source.m:
            ports = source.inputs.values()
            u_port = self.inputs["u"]
            u_port.labels, u_port.units = source.get_all_input_labels_and_units()
            u_port.lower_bound = np.concatenate([port.lower_bound for port in ports])
            u_port.upper_bound = np.concatenate([port.upper_bound for port in ports])
            u_port.nominal_value = source.get_u_from_input_ports()

        # Output port: the source's y metadata
        if source.p:
            y_port = self.outputs["y"]
            source_y = source.outputs["y"]
            y_port.labels = list(source_y.labels)
            y_port.units = list(source_y.units)
            y_port.lower_bound = np.array(source_y.lower_bound, dtype=float)
            y_port.upper_bound = np.array(source_y.upper_bound, dtype=float)
            y_port.nominal_value = np.array(source_y.nominal_value, dtype=float)

    @property
    def params(self) -> dict:
        """The source's live ``params`` while no override is assigned, else the override."""
        if self.params_override is None:
            return self.source.params

        return self.params_override

    @params.setter
    def params(self, value: dict | None):
        self.params_override = value

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        source_f = self.source.f
        dt = self.dt
        solver = self.solver
        substeps = self.substeps

        def f(x, u, t):
            return source_f(x, u, t, params)

        t_k = k * dt

        # x_{k+1} = Φ(x_k, u_k, t_k), the input held over [t_k, t_k + Δt]
        x_next = step_body(f, x, u, u, t_k, dt, solver, substeps)

        return x_next

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        h = self.source.h
        dt = self.dt

        t_k = k * dt

        # y_k = h(x_k, u_k, t_k), the source's output map sampled at t_k = k Δt
        y = h(x, u, t_k, params)

        return y


class ExactDiscretization(Discretization):
    """The exact sampled linear model: ``x_{k+1} = A_d x_k + B_d u_k``, ``y_k = C x_k + D u_k``."""

    def __init__(self, source: DynamicSystem, dt: float):
        super().__init__(source, dt, solver="exact")
        require_linear_model(source)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        source = self.source
        dt = self.dt
        n = source.n
        m = source.m

        t_k = k * dt
        A = xp.asarray(source.A(t_k, params))
        B = xp.asarray(source.B(t_k, params))

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


###############################################################################
# Internal machinery
###############################################################################


def require_discretizable(system, solver: str, substeps: int) -> None:
    """Refuse a source that is not a :class:`DynamicSystem`, an unknown solver or a bad substep count."""
    if not isinstance(system, DynamicSystem):
        raise TypeError(
            f"discretize requires DynamicSystem, got {type(system).__name__}"
        )
    if solver not in DISCRETIZE_SOLVERS:
        raise ValueError(
            f"Unknown solver {solver!r}; expected one of {list(DISCRETIZE_SOLVERS)!r}."
        )
    if int(substeps) != substeps or substeps < 1:
        raise ValueError(f"substeps must be a positive integer, got {substeps!r}")


def require_dt_free(system) -> None:
    """Refuse a source whose ``params`` carry ``"dt"``: the sample time is discretize's ``dt``."""
    if "dt" in system.params:
        raise ValueError(
            f"the params of {system.name} carry 'dt' = {system.params['dt']!r}, but "
            "the sample time is discretize's dt, which f never receives; remove 'dt' "
            "from the params and call discretize(system, dt)."
        )


def require_linear_model(source) -> None:
    """Refuse an exact discretization of a model without ``A`` and ``B`` matrices."""
    if not (
        callable(getattr(source, "A", None)) and callable(getattr(source, "B", None))
    ):
        raise TypeError(
            f"solver='exact' needs a linear model with A and B, got {source.name}; "
            "pass linearize(system) or use solver='rk4'"
        )


def expm(M):
    """The matrix exponential on the array's backend."""
    xp = array_module(M)
    if xp is np:
        return scipy.linalg.expm(M)

    jax = require_jax()

    return jax.scipy.linalg.expm(M)


def y_dependencies_of(source) -> tuple | str:
    """The direct feedthrough of the wrapper's ``y``, read from the source's.

    The wrapper stacks every source input on its one port ``u``: a source ``y``
    on ``"all"`` stays ``"all"``, one that reads some inputs reads ``("u",)``,
    and one that reads none (or no ``y``) reads ``()``.
    """
    y_deps = ()
    if "y" in source.outputs:
        source_deps = source.outputs["y"].dependencies
        if source_deps == "all":
            y_deps = "all"
        elif tuple(source_deps):
            y_deps = ("u",)
    return y_deps


def positive_dt(dt: float) -> float:
    """``dt`` as a float, refused unless positive."""
    dt = float(dt)
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    return dt
