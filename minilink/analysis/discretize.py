"""Convert continuous-time systems to discrete-time step models."""

from __future__ import annotations

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, StepSystem, System


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

    class DiscretizedLeaf(StepSystem):
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
            self._dt = dt
            self._evaluator = source.compile(backend="numpy")

        def step(self, x, u, k=0, params=None):
            xp = array_module(x, u)
            x_arr = xp.asarray(x, dtype=float).reshape(source.n)
            u_arr = xp.asarray(u, dtype=float).reshape(source.m)
            t_k = float(k) * self._dt
            x_new = self._evaluator.rk4_step(x_arr, u_arr, t_k, self._dt)
            return xp.asarray(x_new, dtype=float).reshape(source.n)

        def h(self, x, u, k=0, params=None):
            xp = array_module(x, u)
            x_arr = xp.asarray(x, dtype=float).reshape(source.n)
            u_arr = xp.asarray(u, dtype=float).reshape(source.m)
            t_k = float(k) * self._dt
            y = self._evaluator.outputs(x_arr, u_arr, t_k)
            return xp.asarray(y, dtype=float).reshape(source.p)

    return DiscretizedLeaf()


def sample_static(system: System, dt: float) -> System:
    """
    Wrap a static ``System`` (``n = 0``) for clocked step-diagram / hybrid use.

    The block has no internal state; ``dt`` is stored in ``params`` for metadata.
    Sampling and ZOH are enforced by :class:`~minilink.simulation.computer.Computer`
    or :class:`~minilink.simulation.hybrid_simulator.HybridSimulator`.
    """
    if not isinstance(system, System):
        raise TypeError(f"sample_static requires System, got {type(system).__name__}")
    if system.n != 0:
        raise ValueError(
            f"sample_static requires a static System (n=0), got n={system.n}"
        )
    dt = float(dt)
    if dt <= 0.0:
        raise ValueError(f"dt must be positive, got {dt}")

    source = system

    class SampledStatic(System):
        def __init__(self):
            super().__init__(n=0)
            self.name = f"Sampled({source.name})"
            self.params = dict(getattr(source, "params", {}))
            self.params["dt"] = dt
            for port_id, port in source.inputs.items():
                self.add_input_port(
                    port_id,
                    dim=port.dim,
                    nominal_value=port.nominal_value,
                )
            for port_id, port in source.outputs.items():
                self.add_output_port(
                    port_id,
                    dim=port.dim,
                    function=port.compute,
                    dependencies=port.dependencies,
                    nominal_value=port.nominal_value,
                )

    return SampledStatic()
