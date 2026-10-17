"""Discretization: a continuous-time system as a discrete-time step model."""

from __future__ import annotations

import numpy as np

from minilink.core.system import DynamicSystem, StepSystem

_INTEGRATORS = frozenset({"rk4", "euler"})


###############################################################################
# Public API
###############################################################################


def discretize(
    system: DynamicSystem,
    dt: float | None = None,
    *,
    integrator: str = "rk4",
    params: dict | None = None,
) -> StepSystem:
    """
    Wrap a :class:`DynamicSystem` as a :class:`StepSystem` with hold interval ``dt``.

    ``x_{k+1} = step(x, u, k; p)`` integrates ``f`` over ``[k dt, (k + 1) dt]``
    with ``integrator`` ``"rk4"`` or ``"euler"`` (the same word as ``Sys2Gym``),
    the input held. ``p`` defaults to the wrapper's :attr:`params`: the
    source's live ``params`` when ``params`` is ``None``, else the dict given,
    which replaces them (RULES 4.4). The hold interval has one owner, the
    wrapper's :attr:`dt`, and never reaches ``f`` or ``h``: when ``dt`` is
    omitted it is read once from ``params["dt"]``, and a ``"dt"`` key is
    dropped from the ``params`` given. Three cases are therefore refused:
    ``dt`` and a ``params["dt"]`` that differ, since one would be dropped
    unseen; a ``params`` that holds only ``"dt"`` while the source has other
    params, since it would replace them with ``{}`` (pass ``dt=`` to keep
    them); and, when ``params`` is ``None``, a source whose own params carry
    ``"dt"``, since they reach ``f`` live beside :attr:`dt` (remove the key
    and pass ``dt=``). The wrapper starts from the source's ``x0`` and keeps
    its state, input and ``y`` metadata.
    """
    require_discretizable(system, integrator)

    # One owner for the sample time: the wrapper's dt, never a key of the params f reads
    dt = hold_interval(system, dt, params)
    params = model_params(system, params, dt)

    # The wrapper that takes one step of the chosen integrator
    if integrator == "euler":
        return DiscretizedEulerDynamicSystem(system, dt, params)

    return DiscretizedRK4DynamicSystem(system, dt, params)


class DiscretizedDynamicSystem(StepSystem):
    """
    Discrete-time wrapper over a continuous :class:`DynamicSystem`.

    The hold interval :attr:`dt` lives on the wrapper and nowhere else:
    :attr:`params` are handed to the source's ``f`` and ``h`` untouched, so
    building the wrapper or assigning :attr:`params` refuses a ``"dt"`` key in
    them (a key added later in place to the source's live params is not
    checked). The wrapper starts from the source's ``x0`` and copies its
    state, input and ``y`` labels, units, bounds and nominal values.
    """

    def __init__(
        self,
        source: DynamicSystem,
        dt: float,
        params: dict | None = None,
        *,
        integrator: str,
    ):
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

        # The source, the hold interval, and the params f receives: the params
        # setter reads the source and dt, so they come first
        self.source = source
        self.dt = positive_dt(dt)
        self.params = params
        self.integrator = integrator

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
        """
        The params ``f`` and ``h`` receive: the source's live ones, or the override.

        The source's live ``params`` while :attr:`params_override` is ``None``
        (the same dict, so an in-place edit reaches the source), else that dict.
        Assigning ``params`` sets :attr:`params_override`, and refuses a
        ``"dt"`` key in the params that would reach ``f``: the value, or the
        source's ``params`` when the value is ``None``.
        """
        if self.params_override is None:
            return self.source.params

        return self.params_override

    @params.setter
    def params(self, value: dict | None):
        # One owner for the sample time: the params that reach f never carry it
        p = self.source.params if value is None else value
        self.require_dt_free(p, value)

        self.params_override = value

    def h(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        h = self.source.h
        dt = self.dt

        t_k = k * dt

        # y_k = h(x_k, u_k, t_k), the source's output map sampled at t_k = k Δt
        y = h(x, u, t_k, params)

        return y

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def require_dt_free(self, p, value):
        """Refuse params that carry ``"dt"``: the sample time is the wrapper's :attr:`dt` alone."""
        if "dt" in p:
            where = f"the params of {self.source.name}" if value is None else "params"
            raise ValueError(
                f"{where} carry 'dt' = {p['dt']!r}, but the sample time of "
                f"{self.name} is its dt = {self.dt} s, which f never receives; "
                f"remove 'dt' from {where} and set the sample time through dt alone."
            )


class DiscretizedEulerDynamicSystem(DiscretizedDynamicSystem):
    """``x_{k+1} = x_k + dt f(x_k, u_k, t_k; p)``."""

    def __init__(self, source: DynamicSystem, dt: float, params: dict | None = None):
        super().__init__(source, dt, params, integrator="euler")

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        f = self.source.f
        dt = self.dt

        t_k = k * dt

        # x_{k+1} = x_k + Δt f(x_k, u_k, t_k), the input held over [t_k, t_k + Δt]
        x_next = x + dt * f(x, u, t_k, params)

        return x_next


class DiscretizedRK4DynamicSystem(DiscretizedDynamicSystem):
    """One RK4 step of ``f`` over ``[t_k, t_k + dt]`` with ZOH on ``u``."""

    def __init__(self, source: DynamicSystem, dt: float, params: dict | None = None):
        super().__init__(source, dt, params, integrator="rk4")

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        f = self.source.f
        dt = self.dt

        t_k = k * dt

        # The four slopes over [t_k, t_k + Δt], the input held
        k1 = f(x, u, t_k, params)
        k2 = f(x + 0.5 * dt * k1, u, t_k + 0.5 * dt, params)
        k3 = f(x + 0.5 * dt * k2, u, t_k + 0.5 * dt, params)
        k4 = f(x + dt * k3, u, t_k + dt, params)

        # x_{k+1} = x_k + Δt/6 (k₁ + 2k₂ + 2k₃ + k₄)
        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        return x_next


###############################################################################
# Internal machinery
###############################################################################


def require_discretizable(system, integrator: str) -> None:
    """Refuse a source that is not a :class:`DynamicSystem`, or an unknown integrator."""
    if not isinstance(system, DynamicSystem):
        raise TypeError(
            f"discretize requires DynamicSystem, got {type(system).__name__}"
        )
    if integrator not in _INTEGRATORS:
        raise ValueError(
            f"Unknown integrator {integrator!r}; expected one of {sorted(_INTEGRATORS)!r}."
        )


def hold_interval(
    system: DynamicSystem,
    dt: float | None,
    params: dict | None,
) -> float:
    """The sample time: ``dt``, or ``params["dt"]`` when ``dt`` is omitted; never two that differ."""
    p = system.params if params is None else params
    if dt is None:
        if "dt" not in p:
            raise ValueError(
                "discretize requires the sample time: call discretize(system, "
                "dt=...) or discretize(system, params={..., 'dt': ...})."
            )
        dt = p["dt"]
    elif params is not None and "dt" in params and params["dt"] != dt:
        raise ValueError(
            f"dt={dt!r} and params['dt'] = {params['dt']!r} are two different "
            f"sample times, and one would be dropped; pass the sample time once."
        )
    return dt


def model_params(
    system: DynamicSystem,
    params: dict | None,
    dt: float,
) -> dict | None:
    """The params ``f`` receives: ``params`` without its ``"dt"`` key, ``None`` for the source's."""
    if params is None:
        return None
    if set(params) == {"dt"} and set(system.params) - {"dt"}:
        raise ValueError(
            f"params={params!r} holds only the sample time, so it would replace "
            f"the params of {system.name} with {{}}; call "
            f"discretize(system, dt={dt!r}) to keep them."
        )
    return {key: value for key, value in params.items() if key != "dt"}


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
