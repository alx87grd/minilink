"""Small helpers for optional JAX in otherwise NumPy-first code paths."""

import functools
import types

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


def configure_jax(*, enable_x64: bool | None = None) -> types.ModuleType:
    """Configure process-wide JAX options used by Minilink JAX code paths."""
    try:
        import jax
    except ImportError as e:
        raise ImportError(
            "JAX is required for this code path. Install with: pip install jax jaxlib"
        ) from e

    if enable_x64 is not None:
        jax.config.update("jax_enable_x64", bool(enable_x64))
    return jax


def array_module(a) -> types.ModuleType:
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
