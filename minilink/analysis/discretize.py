"""Convert continuous-time systems to discrete-time step models."""

from __future__ import annotations

import numpy as np

from minilink.core.system import DynamicSystem, StepSystem

_INTEGRATORS = frozenset({"rk4", "euler"})


class DiscretizedDynamicSystem(StepSystem):
    """
    Discrete-time wrapper over a continuous :class:`DynamicSystem`.

    The hold interval :attr:`dt` lives on the wrapper; :attr:`params` are
    handed to the source's ``f`` and ``h`` untouched. The wrapper starts from
    the source's ``x0`` and copies its state, input and ``y`` labels, units,
    bounds and nominal values.
    """

    def __init__(
        self,
        source: DynamicSystem,
        dt: float,
        params: dict | None = None,
        *,
        integrator: str,
    ):
        y_deps = ()
        if "y" in source.outputs:
            y_deps = source.outputs["y"].dependencies
            y_deps = y_deps if y_deps == "all" else tuple(y_deps)
        super().__init__(
            n=source.n,
            input_dim=source.m if source.m else None,
            output_dim=source.p if source.p else None,
            expose_state="x" in source.outputs,
            y_dependencies=y_deps,
        )
        self.name = f"Discretized({source.name})"
        self.source = source
        self.params_override = params
        self.dt = _positive_dt(dt)
        self.integrator = integrator

        # the source's initial state and state metadata
        state = source.state
        self.x0 = np.array(source.x0, dtype=float)
        self.state.labels = list(state.labels)
        self.state.units = list(state.units)
        self.state.lower_bound = np.array(state.lower_bound, dtype=float)
        self.state.upper_bound = np.array(state.upper_bound, dtype=float)
        self.state.nominal_value = np.array(state.nominal_value, dtype=float)

        # each source input port's metadata, stacked in flat-u order into u
        if source.m:
            ports = source.inputs.values()
            u_port = self.inputs["u"]
            u_port.labels, u_port.units = source.get_all_input_labels_and_units()
            u_port.lower_bound = np.concatenate([port.lower_bound for port in ports])
            u_port.upper_bound = np.concatenate([port.upper_bound for port in ports])
            u_port.nominal_value = source.get_u_from_input_ports()

        # the source's y output metadata
        if source.p:
            y_port, source_y = self.outputs["y"], source.outputs["y"]
            y_port.labels = list(source_y.labels)
            y_port.units = list(source_y.units)
            y_port.lower_bound = np.array(source_y.lower_bound, dtype=float)
            y_port.upper_bound = np.array(source_y.upper_bound, dtype=float)
            y_port.nominal_value = np.array(source_y.nominal_value, dtype=float)

    @property
    def params(self) -> dict:
        """
        The source's live ``params`` while :attr:`params_override` is ``None``
        (the same dict, so an in-place edit reaches the source), else that dict.
        Assigning ``params`` sets :attr:`params_override`.
        """
        if self.params_override is None:
            return self.source.params
        return self.params_override

    @params.setter
    def params(self, value: dict | None):
        self.params_override = value

    def h(self, x, u, k=0, params=None):
        h = self.source.h
        p = self.params if params is None else params
        dt = self.dt
        t_k = k * dt

        # the source's output map, sampled at t_k
        y = h(x, u, t_k, p)

        return y


class DiscretizedEulerDynamicSystem(DiscretizedDynamicSystem):
    """``x_{k+1} = x_k + dt f(x_k, u_k, t_k; p)``."""

    def __init__(self, source: DynamicSystem, dt: float, params: dict | None = None):
        super().__init__(source, dt, params, integrator="euler")

    def step(self, x, u, k=0, params=None):
        f = self.source.f
        p = self.params if params is None else params
        dt = self.dt
        t_k = k * dt

        # one forward-Euler step, the input held over [t_k, t_k + dt]
        x_next = x + dt * f(x, u, t_k, p)

        return x_next


class DiscretizedRK4DynamicSystem(DiscretizedDynamicSystem):
    """One RK4 step of ``f`` over ``[t_k, t_k + dt]`` with ZOH on ``u``."""

    def __init__(self, source: DynamicSystem, dt: float, params: dict | None = None):
        super().__init__(source, dt, params, integrator="rk4")

    def step(self, x, u, k=0, params=None):
        f = self.source.f
        p = self.params if params is None else params
        dt = self.dt
        t_k = k * dt

        # the four RK4 slopes over [t_k, t_k + dt], the input held
        k1 = f(x, u, t_k, p)
        k2 = f(x + 0.5 * dt * k1, u, t_k + 0.5 * dt, p)
        k3 = f(x + 0.5 * dt * k2, u, t_k + 0.5 * dt, p)
        k4 = f(x + dt * k3, u, t_k + dt, p)
        x_next = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        return x_next


# Public API


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
    which replaces them (RULES 4.4). The hold interval stays on the wrapper as
    :attr:`dt` and never reaches ``f`` or ``h``: when ``dt`` is omitted it is
    read once from ``params["dt"]``, and a ``"dt"`` key is dropped from the
    ``params`` given. A ``params`` that holds only ``"dt"`` is therefore
    refused while the source has params of its own, since it would replace
    them with ``{}``: pass the sample time as ``dt=`` to keep them. The
    wrapper starts from the source's ``x0`` and keeps its state, input and
    ``y`` metadata.
    """
    if not isinstance(system, DynamicSystem):
        raise TypeError(
            f"discretize requires DynamicSystem, got {type(system).__name__}"
        )
    if integrator not in _INTEGRATORS:
        raise ValueError(
            f"Unknown integrator {integrator!r}; expected one of {sorted(_INTEGRATORS)!r}."
        )

    dt = _hold_interval(system, dt, params)
    params = _model_params(system, params)

    if integrator == "euler":
        return DiscretizedEulerDynamicSystem(system, dt, params)
    return DiscretizedRK4DynamicSystem(system, dt, params)


# Internal machinery


def _hold_interval(
    system: DynamicSystem,
    dt: float | None,
    params: dict | None,
) -> float:
    p = system.params if params is None else params
    if dt is None:
        if "dt" not in p:
            raise ValueError("discretize requires dt=... or params['dt'].")
        dt = p["dt"]
    return dt


def _model_params(system: DynamicSystem, params: dict | None) -> dict | None:
    if params is None:
        return None
    if set(params) == {"dt"} and system.params:
        raise ValueError(
            f"params={params!r} holds only the sample time, so it would replace "
            f"the params of {system.name} with {{}}; call "
            f"discretize(system, dt={params['dt']!r}) to keep them."
        )
    return {key: value for key, value in params.items() if key != "dt"}


def _positive_dt(dt: float) -> float:
    dt = float(dt)
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    return dt
