"""The automatic compile backend: JAX when the system traces, NumPy otherwise."""

import logging

from minilink.core.backends import BACKEND_AUTO, BACKEND_JAX, BACKEND_NUMPY

###############################################################################
# Public API
###############################################################################


def resolve_auto_backend(build, backend):
    """Return ``(backend_name, evaluator)`` using ``build(backend)`` to compile.

    ``backend=BACKEND_AUTO`` tries JAX first (when importable) and
    falls back to NumPy on import or compile failure; any other value compiles
    exactly as requested.
    """
    # A named backend compiles as requested
    if backend != BACKEND_AUTO:
        return backend, build(backend)

    # NumPy when JAX does not import (a bare probe: the evaluators apply the x64 policy)
    try:
        import jax  # noqa: F401
    except ImportError:
        return BACKEND_NUMPY, build(BACKEND_NUMPY)

    # JAX first, NumPy when the system does not compile under JAX
    try:
        return BACKEND_JAX, build(BACKEND_JAX)
    except Exception:
        logging.getLogger(__name__).debug(
            "JAX compile failed, falling back to numpy", exc_info=True
        )
        return BACKEND_NUMPY, build(BACKEND_NUMPY)
