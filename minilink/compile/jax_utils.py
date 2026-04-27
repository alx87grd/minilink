"""Small helpers for optional JAX in otherwise NumPy-first code paths."""

from __future__ import annotations

import functools
import types
from typing import Any

import numpy as np


@functools.lru_cache(maxsize=1)
def require_jax_numpy() -> types.ModuleType:
    """Return ``jax.numpy`` (lazy, cached). Raises if JAX is not installed."""
    try:
        import jax.numpy as jnp
    except ImportError as e:
        raise ImportError(
            "JAX is required for this code path. Install with: pip install jax jaxlib"
        ) from e
    return jnp


def array_module(a: Any) -> types.ModuleType:
    """Return ``jax.numpy`` if *a* is a JAX array/tracer, else ``numpy``."""
    if type(a).__module__.startswith("jax"):
        import jax.numpy as jnp

        return jnp
    return np


def format_benchmark_backend_label(compile_backend: str) -> str:
    """Human-readable backend label for benchmark tables and result structs.

    For ``compile_backend="jax"``, returns ``jax(cpu)``, ``jax(gpu)``, etc., from the
    active JAX default device. Any other value is returned unchanged.
    """
    if compile_backend != "jax":
        return compile_backend
    try:
        import jax
    except ImportError:
        return "jax"
    try:
        plat = str(jax.default_backend()).lower()
    except Exception:
        return "jax"
    return f"jax({plat})"
