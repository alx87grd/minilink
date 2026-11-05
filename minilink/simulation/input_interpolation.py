"""Input hold: a sampled input signal read at any time of a forced simulation, each sample held over its period."""

from collections.abc import Callable

import numpy as np

###############################################################################
# Public API
###############################################################################


def build_u_at_t(times: np.ndarray, u: np.ndarray) -> Callable[[float], np.ndarray]:
    """
    Return ``u(t)``: the samples ``u``, shape ``(m, n_pts)``, at ``times``, held over each period.

    Zero-order hold, ``u(t) = u_i`` for ``t_i ≤ t < t_{i+1}``; before the first
    sample and after the last, the end samples hold.
    """
    n_pts = u.shape[1]

    def u_of_t(t: float) -> np.ndarray:
        i = int(np.searchsorted(times, t, side="right")) - 1
        i = max(0, min(i, n_pts - 1))
        return u[:, i]

    return u_of_t
