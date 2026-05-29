"""
Central vocabulary for compile-backend strings.

One source of truth shared by :class:`~minilink.simulation.simulator.Simulator`,
:class:`~minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationOptions`,
:func:`minilink.compile.compiler.compile`, and the benchmark modules. Avoids
drift in the meaning of ``"numpy"`` / ``"jax"`` / ``"auto"`` / ``"direct"``
across the library.

Backend strings
---------------

================ ============================================================
``"numpy"``      NumPy evaluator. Always available.
``"jax"``        JAX evaluator. Hard-requires the ``minilink[jax]`` extra.
``"auto"``       Try JAX, fall back to NumPy. Used by the simulator.
``"direct"``     Use ``system.f`` directly (no compiled evaluator); valid for
                 some trajectory-optimization transcriptions.
================ ============================================================

Notes
-----
``"auto"`` and ``"direct"`` are *transcription* / *simulator* concepts: the
low-level :func:`minilink.compile.compiler.compile` only accepts ``"numpy"``
and ``"jax"``. Use :func:`normalize_backend` with ``allow_auto`` /
``allow_direct`` to validate at the right layer.
"""

from __future__ import annotations

BACKEND_NUMPY = "numpy"
BACKEND_JAX = "jax"
BACKEND_AUTO = "auto"
BACKEND_DIRECT = "direct"

#: Backends accepted by :func:`minilink.compile.compiler.compile`.
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


def require_jax_backend() -> None:
    """Raise :class:`ImportError` with the canonical install hint if JAX is missing."""
    try:
        import jax  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "This code path requires JAX. "
            "Install with `pip install minilink[jax]` (or `pip install jax jaxlib`)."
        ) from exc
