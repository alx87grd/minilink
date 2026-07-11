"""Evaluator execution-tier naming contract (fast vs trace)."""

from __future__ import annotations

TRACE_TIER_MSG = "Trace tier requires compile(backend='jax')."

_TRACE_TIER_NAMES = frozenset(
    {
        "f_trace",
        "f_trace_p",
        "f_jit",
        "f_jit_p",
        "outputs_trace",
        "outputs_trace_p",
        "outputs_jit",
        "outputs_jit_p",
        "step_trace",
        "step_trace_p",
        "step_jit",
        "step_jit_p",
    }
)


class NoTraceTierMixin:
    """Mixin for NumPy evaluators — reject JAX-only trace-tier access."""

    def __getattr__(self, name: str):
        if name in _TRACE_TIER_NAMES:
            raise AttributeError(TRACE_TIER_MSG) from None
        raise AttributeError(
            f"{type(self).__name__!r} object has no attribute {name!r}"
        )


class TraceTierMixin:
    """Mixin for JAX evaluators — documents trace-tier availability."""

    @property
    def has_trace_tier(self) -> bool:
        return True


def register_jit_aliases(cls, names: tuple[str, ...]) -> None:
    """Register ``{name}_jit`` / ``{name}_jit_p`` as aliases of fast-tier methods."""
    for name in names:
        fast = getattr(cls, name)
        setattr(cls, f"{name}_jit", fast)
        param = getattr(cls, f"{name}_p")
        setattr(cls, f"{name}_jit_p", param)
