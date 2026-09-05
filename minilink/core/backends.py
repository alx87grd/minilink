"""
NumPy/JAX backend policy and helpers.

Minilink is NumPy-first; JAX is optional (``minilink[jax]``) and always
imported lazily. This module is the single source of truth for both sides of
that policy:

- **Backend vocabulary** — the ``"numpy"`` / ``"jax"`` / ``"auto"`` /
  ``"direct"`` strings accepted by the compiler, simulator, and
  trajectory-optimization transcriptions, validated by
  :func:`normalize_backend`.
- **Runtime helpers** — :func:`array_module` for hybrid equation paths and
  :func:`require_jax_numpy` / :func:`configure_jax` for JAX-only code paths.
- **Precision policy** — JAX runs with 64-bit floats by default
  (:func:`jax_x64_policy`, applied by :func:`ensure_jax_x64` from every
  JAX evaluator and from :func:`require_jax_numpy`); set ``MINILINK_JAX_X64=0``
  to keep JAX's float32 default for GPU / RL workloads.

Backend strings
---------------

================ ============================================================
``"numpy"``      NumPy evaluator. Always available.
``"jax"``        JAX evaluator. Hard-requires the ``minilink[jax]`` extra.
``"auto"``       Try JAX, fall back to NumPy. Used by the simulator.
``"direct"``     Use :meth:`~minilink.core.system.DynamicSystem.f` directly
                 (no compiled evaluator); valid for some trajectory-optimization
                 transcriptions on :class:`~minilink.core.system.DynamicSystem`
                 or :class:`~minilink.core.diagram.DiagramSystem`.
================ ============================================================

``"auto"`` and ``"direct"`` are *simulator* / *transcription* concepts: the
low-level :func:`minilink.core.compile.compiler.compile` only accepts
``"numpy"`` and ``"jax"``. Use :func:`normalize_backend` with ``allow_auto`` /
``allow_direct`` to validate at the right layer.
"""

from __future__ import annotations

import functools
import os
import types

import numpy as np

BACKEND_NUMPY = "numpy"
BACKEND_JAX = "jax"
BACKEND_AUTO = "auto"
BACKEND_DIRECT = "direct"

#: Backends accepted by :func:`minilink.core.compile.compiler.compile`.
COMPILE_BACKENDS: tuple[str, ...] = (BACKEND_NUMPY, BACKEND_JAX)

#: Backends accepted by :class:`~minilink.simulation.simulator.Simulator`.
SIMULATOR_BACKENDS: tuple[str, ...] = (BACKEND_NUMPY, BACKEND_JAX, BACKEND_AUTO)

#: Backends accepted by JAX trajectory-optimization transcriptions.
TRANSCRIPTION_BACKENDS: tuple[str, ...] = (
    BACKEND_NUMPY,
    BACKEND_JAX,
    BACKEND_DIRECT,
)


def normalize_backend(
    name: str | None,
    *,
    allow_auto: bool = False,
    allow_direct: bool = False,
) -> str:
    """Validate and lowercase a compile-backend string.

    Parameters
    ----------
    name : str or None
        Backend identifier. ``None`` is treated as ``"numpy"``.
    allow_auto : bool, optional
        Allow the ``"auto"`` value (Simulator-level concept).
    allow_direct : bool, optional
        Allow the ``"direct"`` value (transcription-level concept).

    Returns
    -------
    str
        Normalized lowercase backend string.
    """
    if name is None:
        return BACKEND_NUMPY
    key = str(name).strip().lower()
    valid = list(COMPILE_BACKENDS)
    if allow_auto:
        valid.append(BACKEND_AUTO)
    if allow_direct:
        valid.append(BACKEND_DIRECT)
    if key not in valid:
        raise ValueError(
            f"Unknown compile backend {name!r}. Expected one of {tuple(valid)}."
        )
    return key


@functools.lru_cache(maxsize=1)
def require_jax_numpy() -> types.ModuleType:
    """Return ``jax.numpy`` (lazy, cached). Raises if JAX is not installed."""
    try:
        import jax.numpy as jnp
    except ImportError as e:
        raise ImportError(
            "This code path requires JAX. "
            "Install with `pip install minilink[jax]` (or `pip install jax jaxlib`)."
        ) from e
    ensure_jax_x64()
    return jnp


def jax_x64_policy() -> bool:
    """Library policy: 64-bit floats in JAX unless ``MINILINK_JAX_X64=0``."""
    flag = os.environ.get("MINILINK_JAX_X64", "1").strip().lower()
    return flag not in ("0", "false", "no", "off")


def ensure_jax_x64() -> types.ModuleType:
    """Apply :func:`jax_x64_policy` to the JAX config (idempotent; JAX required).

    Called by every JAX evaluator constructor and by :func:`require_jax_numpy`,
    so tools built on JAX evaluators never need the caller to enable x64.
    """
    import jax

    if jax_x64_policy():
        jax.config.update("jax_enable_x64", True)
    return jax


def configure_jax(*, enable_x64: bool | None = None) -> types.ModuleType:
    """Configure process-wide JAX options used by Minilink JAX code paths."""
    try:
        import jax
    except ImportError as e:
        raise ImportError(
            "This code path requires JAX. "
            "Install with `pip install minilink[jax]` (or `pip install jax jaxlib`)."
        ) from e

    if enable_x64 is not None:
        jax.config.update("jax_enable_x64", bool(enable_x64))
    return jax


def array_module(*values) -> types.ModuleType:
    """Return ``jax.numpy`` if any value is a JAX array/tracer, else ``numpy``.

    This is the hybrid-backend idiom for equation paths: bind
    ``xp = array_module(x)`` once at the top of ``f`` / ``h``, then write the
    math with ``xp`` so the same code runs under NumPy and traces under JAX.
    """
    for value in values:
        if type(value).__module__.startswith("jax"):
            import jax.numpy as jnp

            return jnp
    return np
