"""
JAX Integration Utilities for Minilink Systems.
This module provides functions to optionally XLA-compile and AutoDiff
standard minilink blocks.
"""

def get_f_jax(sys):
    """
    Returns a JAX-jitted version of the system's dynamic equation f(x,u,t,params).
    If the system explicitly defines a `f_jax` method, it is used.
    Otherwise, it attempts to duck-type trace the standard `f` method.
    """
    try:
        import jax
    except ImportError:
        raise ImportError(
            "JAX is required for this feature but is not installed. "
            "Please install it with `pip install jax jaxlib` or via conda."
        )

    # 1. Check if the block explicitly provides a JAX implementation
    if hasattr(sys, 'f_jax'):
        # Automatically wrap it in jit
        return jax.jit(sys.f_jax)

    # 2. Duck-type the standard numpy implementation
    from jax.errors import ConcretizationTypeError

    # JAX treats 'params' as a static Python config (a dict).
    # If params changes, JAX recompiles; otherwise it's baked into the XLA code.
    @jax.jit(static_argnames=['params'])
    def f_jitted(x, u, t=0, params=None):
        try:
            return sys.f(x, u, t, params)
        except (ConcretizationTypeError, TypeError, Exception) as e:
            raise RuntimeError(
                f"\n\nBlock '{sys.name}' cannot be transparently compiled with JAX.\n"
                f"Its f() likely performs in-place mutations or strict numpy operations.\n"
                f"Please rewrite f() in a purely functional style or define a manual "
                f"`f_jax` method on your class.\n"
                f"Original JAX error: {str(e)}"
            ) from e

    return f_jitted
