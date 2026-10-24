"""NumPy/JAX backend policy: the backend strings, ``array_module``, and the lazy JAX imports."""

from __future__ import annotations

import functools
import os
import types

import numpy as np

# Minilink is NumPy-first; JAX is optional (``pip install jax jaxlib``) and always imported
# lazily. The backend strings are validated by ``normalize_backend``.
BACKEND_NUMPY = "numpy"
BACKEND_JAX = "jax"
BACKEND_AUTO = "auto"
BACKEND_DIRECT = "direct"

_JAX_REQUIRED = "This code path requires JAX. Install with `pip install jax jaxlib`."

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


###############################################################################
# Public API
###############################################################################


def normalize_backend(
    name: str | None,
    *,
    allow_auto: bool = False,
    allow_direct: bool = False,
) -> str:
    """Validate and lowercase a compile-backend string.

    ================ ============================================================
    ``"numpy"``      NumPy evaluator. Always available.
    ``"jax"``        JAX evaluator. Hard-requires JAX (``pip install jax jaxlib``).
    ``"auto"``       Try JAX, fall back to NumPy. Used by the simulator.
    ``"direct"``     Use :meth:`~minilink.core.system.DynamicSystem.f` directly
                     (no compiled evaluator); valid for some trajectory-optimization
                     transcriptions on :class:`~minilink.core.system.DynamicSystem`
                     or :class:`~minilink.core.diagram.DiagramSystem`.
    ================ ============================================================

    ``"auto"`` and ``"direct"`` are *simulator* / *transcription* concepts: the
    low-level :func:`minilink.core.compile.compiler.compile` only accepts
    ``"numpy"`` and ``"jax"``; ``allow_auto`` / ``allow_direct`` admit them at
    the layer that understands them.

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

    # The compile backends, and the layer-level ones this caller admits
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


def array_module(*values) -> types.ModuleType:
    """Return ``jax.numpy`` if any value is a JAX array/tracer, else ``numpy``.

    This is the hybrid-backend idiom for equation paths: bind
    ``xp = array_module(x)`` once at the top of ``f`` / ``h``, then write the
    math with ``xp`` so the same code runs under NumPy and traces under JAX.
    """
    # A JAX array or tracer is told by the module of its type (``jax…`` or ``jaxlib…``)
    for value in values:
        if type(value).__module__.startswith("jax"):
            import jax.numpy as jnp

            # The core records are pytrees from the first JAX array on, traced or not
            register_jax_pytrees()

            return jnp

    return np


# The lazy JAX imports


@functools.lru_cache(maxsize=1)
def jax_installed() -> bool:
    """Return True when the ``jax`` package is importable (without importing it)."""
    import importlib.util

    return importlib.util.find_spec("jax") is not None


@functools.lru_cache(maxsize=1)
def require_jax() -> types.ModuleType:
    """Return ``jax`` (lazy, cached) under the float64 policy. Raises if JAX is not installed."""
    try:
        import jax
    except ImportError as e:
        raise ImportError(_JAX_REQUIRED) from e

    ensure_jax_x64()

    return jax


@functools.lru_cache(maxsize=1)
def require_jax_numpy() -> types.ModuleType:
    """Return ``jax.numpy`` (lazy, cached). Raises if JAX is not installed."""
    try:
        import jax.numpy as jnp
    except ImportError as e:
        raise ImportError(_JAX_REQUIRED) from e

    ensure_jax_x64()

    return jnp


# The precision policy


def jax_x64_policy() -> bool:
    """Library policy: 64-bit floats in JAX unless ``MINILINK_JAX_X64=0``.

    ``MINILINK_JAX_X64=0`` keeps JAX's float32 default, for GPU or
    reinforcement-learning workloads.
    """
    flag = os.environ.get("MINILINK_JAX_X64", "1").strip().lower()

    return flag not in ("0", "false", "no", "off")


def ensure_jax_x64() -> types.ModuleType:
    """Apply :func:`jax_x64_policy` to the JAX config (idempotent; JAX required).

    Called by every JAX evaluator constructor and by :func:`require_jax_numpy`,
    so tools built on JAX evaluators never need the caller to enable x64.
    Without JAX it raises :func:`require_jax`'s error, with the install hint.
    """
    try:
        import jax
    except ImportError as e:
        raise ImportError(_JAX_REQUIRED) from e

    if jax_x64_policy():
        jax.config.update("jax_enable_x64", True)

    register_jax_pytrees()

    return jax


def configure_jax(*, enable_x64: bool | None = None) -> types.ModuleType:
    """Configure process-wide JAX options used by Minilink JAX code paths."""
    try:
        import jax
    except ImportError as e:
        raise ImportError(_JAX_REQUIRED) from e

    if enable_x64 is not None:
        jax.config.update("jax_enable_x64", bool(enable_x64))

    return jax


# Random keys: a JAX PRNG key, or a NumPy generator


def is_jax_key(key) -> bool:
    """``True`` for a JAX PRNG key (concrete or traced)."""
    return array_module(key) is not np


def numpy_generator(key) -> np.random.Generator:
    """A NumPy generator from a generator, an integer seed, or ``None``."""
    if isinstance(key, np.random.Generator):
        return key

    return np.random.default_rng(key)


@functools.lru_cache(maxsize=1)
def register_jax_pytrees() -> None:
    """Register the core records (:class:`~minilink.core.trajectory.Trajectory`) as JAX pytrees, once."""
    # JAX imported here, not through require_jax: this runs inside its first call
    import jax

    # A real cycle: the trajectory module imports this one
    from minilink.core.trajectory import register_trajectory_pytree

    register_trajectory_pytree(jax)
