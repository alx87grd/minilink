"""
Pick NumPy vs JAX for dynamics arrays based on the leading state slice.

Legacy helper for blocks that dispatch on array type. For mechanical plants,
use :class:`~minilink.mechanics.mechanical.MechanicalSystem` (NumPy) or
:class:`~minilink.mechanics.mechanical.JaxMechanicalSystem` (JAX) explicitly.
"""

from __future__ import annotations

import numpy as np


def array_module(x):
    """
    Return ``numpy`` or ``jax.numpy`` to match *x*.

    Parameters
    ----------
    x : array-like
        Typically the state vector passed to :meth:`~minilink.core.system.System.f`.

    Returns
    -------
    module
        Either the ``numpy`` module or ``jax.numpy``.
    """
    if isinstance(x, np.ndarray):
        return np
    try:
        import jax.numpy as jnp
    except ImportError:
        return np
    return jnp
