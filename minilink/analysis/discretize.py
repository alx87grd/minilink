"""Convert continuous-time systems to discrete-time step models."""

from __future__ import annotations

from minilink.core.system import DynamicSystem, StepSystem


def discretize(system: DynamicSystem, dt: float, *, method: str = "rk4") -> StepSystem:
    """
    Wrap a :class:`DynamicSystem` as a :class:`StepSystem` with sample time ``dt``.

    ``x_{k+1} = step(x, u, k)`` integrates ``f`` over one hold interval using
    ``method`` (``rk4`` only in v1). ``dt`` lives in the closure, not in ``step``.
    """
    if not isinstance(system, DynamicSystem):
        raise TypeError(
            f"discretize requires DynamicSystem, got {type(system).__name__}"
        )
    dt = float(dt)
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")
    if method != "rk4":
        raise ValueError(f"Unknown discretization method {method!r}; expected 'rk4'.")

    source = system

    class DiscretizedDynamicSystem(StepSystem):
        def __init__(self):
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
            self.params = dict(getattr(source, "params", {}))
            self.dt = dt
            self.evaluator = source.compile(backend="numpy")

        def step(self, x, u, k=0, params=None):
            t_k = float(k) * self.dt
            return self.evaluator.rk4_step(x, u, t_k, self.dt)

        def h(self, x, u, k=0, params=None):
            t_k = float(k) * self.dt
            return source.h(x, u, t_k, params)

    return DiscretizedDynamicSystem()
