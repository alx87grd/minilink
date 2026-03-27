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

    # On dit à JAX de traiter 'params' comme une configuration Python statique (un dict)
    # Si le contenu de 'params' change, JAX recompilera la fonction. Sinon, il l'intègre en dur dans le C++.
    @jax.jit(static_argnames=['params'])
    def f_jitted(x, u, t=0, params=None):
        try:
            return sys.f(x, u, t, params)
        except (ConcretizationTypeError, TypeError, Exception) as e:
            # JAX tracing error when standard numpy code isn't purely functional 
            # (e.g. `dx = np.zeros(2); dx[0] = ...` or `np.array([tracer])`)
            raise RuntimeError(
                f"\n\nLe bloc '{sys.name}' n'est pas compilable avec JAX de façon transparente.\n"
                f"Son équation f() effectue probablement des mutations in-place ou des opérations "
                f"strictes avec numpy.\n"
                f"Veuillez réécrire f() de manière purement fonctionnelle ou définir manuellement "
                f"une méthode `f_jax` dans votre classe.\n"
                f"Erreur d'origine JAX : {str(e)}"
            ) from e

    return f_jitted
