"""The automatic compile backend: JAX when the system traces, NumPy otherwise."""

from minilink.core.backends import BACKEND_AUTO
from minilink.core.compile.compiler import build_on_auto

###############################################################################
# Public API
###############################################################################


def resolve_auto_backend(build, backend):
    """Return ``(backend_name, evaluator)`` using ``build(backend)`` to compile.

    ``backend=BACKEND_AUTO`` takes the compiler's one rule
    (:func:`~minilink.core.compile.compiler.build_on_auto`): JAX first, NumPy
    when JAX does not import or a block does not trace; any other value
    compiles exactly as requested.
    """
    # A named backend compiles as requested
    if backend != BACKEND_AUTO:
        return backend, build(backend)

    return build_on_auto(build)
