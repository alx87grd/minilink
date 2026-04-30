"""Input interpolation for forced simulation."""

from collections.abc import Callable

import numpy as np
from scipy.interpolate import interp1d

INPUT_INTERP_KEY = "input_interp"


def build_u_at_t(
    times: np.ndarray, u: np.ndarray, scheme: str = "linear"
) -> Callable[[float], np.ndarray]:
    m, n_pts = u.shape

    if scheme == "zoh":

        def u_of_t(t: float) -> np.ndarray:
            i = int(np.searchsorted(times, t, side="right")) - 1
            i = max(0, min(i, n_pts - 1))
            return u[:, i]

        return u_of_t

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
        return np.array([f(t) for f in interps], dtype=float)

    return u_of_t
