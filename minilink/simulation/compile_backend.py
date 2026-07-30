"""Automatic compile-backend resolution shared by simulators."""

import logging

from minilink.core.backends import BACKEND_AUTO, BACKEND_JAX, BACKEND_NUMPY


def resolve_auto_backend(build, compile_backend):
    """Return ``(backend_name, evaluator)`` using ``build(backend)`` to compile.

    ``compile_backend=BACKEND_AUTO`` tries JAX first (when importable) and
    falls back to NumPy on import or compile failure; any other value compiles
    exactly as requested.
    """
    if compile_backend != BACKEND_AUTO:
        return compile_backend, build(compile_backend)

    try:
        import jax  # noqa: F401
    except ImportError:
        return BACKEND_NUMPY, build(BACKEND_NUMPY)
    try:
        return BACKEND_JAX, build(BACKEND_JAX)
    except Exception:
        logging.getLogger(__name__).debug(
            "JAX compile failed, falling back to numpy", exc_info=True
        )
        return BACKEND_NUMPY, build(BACKEND_NUMPY)
