"""Input interpolation: a sampled input signal read at any time of a forced simulation."""

from collections.abc import Callable

import numpy as np
from scipy.interpolate import interp1d

# The solver option that names the forced input's scheme: ``"zoh"`` or an
# ``interp1d`` kind
INPUT_INTERP_KEY = "input_interp"


###############################################################################
# Public API
###############################################################################


def build_u_at_t(
    times: np.ndarray, u: np.ndarray, scheme: str = "linear"
) -> Callable[[float], np.ndarray]:
    """
    Return ``u(t)``: the samples ``u``, shape ``(m, n_pts)``, at ``times``, read at any ``t``.

    ``"zoh"`` holds each sample until the next one; any other ``scheme`` is an
    ``interp1d`` kind, one interpolant per channel. Before the first sample and
    after the last, the end samples hold.
    """
    m, n_pts = u.shape

    # Zero-order hold: u(t) = u_i with t_i ≤ t < t_{i+1}, the index clamped to the samples
    if scheme == "zoh":

        def u_of_t(t: float) -> np.ndarray:
            i = int(np.searchsorted(times, t, side="right")) - 1
            i = max(0, min(i, n_pts - 1))
            return u[:, i]

        return u_of_t

    # One interpolant per channel, the end samples held outside the grid
    interps = [
        interp1d(
            times,
            u[j],
            kind=scheme,
            bounds_error=False,
            fill_value=(u[j, 0], u[j, -1]),
            assume_sorted=True,
        )
        for j in range(m)
    ]

    def u_of_t(t: float) -> np.ndarray:
        u_t = np.array([f(t) for f in interps], dtype=float)
        return u_t

    return u_of_t
