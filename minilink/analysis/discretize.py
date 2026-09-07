"""Convert continuous-time systems to discrete-time step models."""

from __future__ import annotations

from minilink.core.system import DynamicSystem, StepSystem

_INTEGRATORS = frozenset({"rk4", "euler"})


class DiscretizedDynamicSystem(StepSystem):
    """Discrete-time wrapper over a continuous :class:`DynamicSystem`."""

    def __init__(self, source: DynamicSystem, params: dict, *, integrator: str):
        y_deps = ()
        if "y" in source.outputs:
            y_deps = tuple(source.outputs["y"].dependencies)
        super().__init__(
            n=source.n,
            input_dim=source.m if source.m else None,
            output_dim=source.p if source.p else None,
            expose_state="x" in source.outputs,
            y_dependencies=y_deps,
        )
        self.name = f"Discretized({source.name})"
        self.params = dict(params)
        self.integrator = integrator
        self._source = source

    def h(self, x, u, k=0, params=None):
        h = self._source.h
        p = self.params if params is None else params
        dt = p["dt"]
        t_k = k * dt
        return h(x, u, t_k, p)


class DiscretizedEulerDynamicSystem(DiscretizedDynamicSystem):
    """``x_{k+1} = x_k + dt f(x_k, u_k, t_k; p)``."""

    def __init__(self, source: DynamicSystem, params: dict):
        super().__init__(source, params, integrator="euler")

    def step(self, x, u, k=0, params=None):
        f = self._source.f
        p = self.params if params is None else params
        dt = p["dt"]
        t_k = k * dt
        return x + dt * f(x, u, t_k, p)


class DiscretizedRK4DynamicSystem(DiscretizedDynamicSystem):
    """One RK4 step of ``f`` over ``[t_k, t_k + dt]`` with ZOH on ``u``."""

    def __init__(self, source: DynamicSystem, params: dict):
        super().__init__(source, params, integrator="rk4")

    def step(self, x, u, k=0, params=None):
        f = self._source.f
        p = self.params if params is None else params
        dt = p["dt"]
        t_k = k * dt
        k1 = f(x, u, t_k, p)
        k2 = f(x + 0.5 * dt * k1, u, t_k + 0.5 * dt, p)
        k3 = f(x + 0.5 * dt * k2, u, t_k + 0.5 * dt, p)
        k4 = f(x + dt * k3, u, t_k + dt, p)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)


# Public API


def discretize(
    system: DynamicSystem,
    dt: float | None = None,
    *,
    integrator: str = "rk4",
    params: dict | None = None,
) -> StepSystem:
    """
    Wrap a :class:`DynamicSystem` as a :class:`StepSystem` with sample time in ``params``.

    ``params["dt"]`` is the hold interval (set via ``dt=`` and/or ``params``).
    ``x_{k+1} = step(x, u, k; p)`` integrates ``f`` with ``integrator``
    ``"rk4"`` or ``"euler"`` (the same word as ``Sys2Gym``); ``p`` defaults to
    the wrapper's :attr:`params`.
    """
    if not isinstance(system, DynamicSystem):
        raise TypeError(
            f"discretize requires DynamicSystem, got {type(system).__name__}"
        )
    if integrator not in _INTEGRATORS:
        raise ValueError(
            f"Unknown integrator {integrator!r}; expected one of {sorted(_INTEGRATORS)!r}."
        )

    merged = _merge_discretize_params(system, dt, params)

    if integrator == "euler":
        return DiscretizedEulerDynamicSystem(system, merged)
    return DiscretizedRK4DynamicSystem(system, merged)


# Internal machinery


def _merge_discretize_params(
    system: DynamicSystem,
    dt: float | None,
    params: dict | None,
) -> dict:
    merged = dict(getattr(system, "params", {}))
    if params is not None:
        merged.update(params)
    if dt is not None:
        merged["dt"] = float(dt)
    if "dt" not in merged:
        raise ValueError("discretize requires dt=... or params['dt'].")
    dt_val = float(merged["dt"])
    if dt_val <= 0.0:
        raise ValueError(f"params['dt'] must be positive, got {dt_val}")
    merged["dt"] = dt_val
    return merged
