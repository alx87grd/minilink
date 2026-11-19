"""Field grids: a scalar field sampled on a 2-D axis-aligned grid, as plain NumPy arrays."""

from collections.abc import Callable

import numpy as np

###############################################################################
# Public API
###############################################################################


def sample_grid(
    fn: Callable,
    bounds,
    *,
    grid_shape: tuple = (120, 120),
    t: float = 0.0,
    params=None,
):
    """
    Sample a scalar query over a 2-D axis-aligned grid: ``(xs, ys, Z)``.

    Scenes and state fields rasterize their clearance and cost-density queries
    with it, for heatmaps. ``Z[j, i]`` is the value at ``(xs[i], ys[j])``,
    shape ``(ny, nx)``.

    Parameters
    ----------
    fn : callable
        Scalar query ``fn(p, t, params) -> float`` over a workspace point ``p``.
    bounds : ((x_min, x_max), (y_min, y_max))
        Grid extent along each axis.
    grid_shape : (nx, ny)
        Number of samples along each axis.
    """
    (x_lo, x_hi), (y_lo, y_hi) = bounds
    nx, ny = grid_shape

    # The nodes along each axis
    xs = np.linspace(x_lo, x_hi, nx)
    ys = np.linspace(y_lo, y_hi, ny)

    # Zⱼᵢ = fn(pᵢⱼ, t, params) at pᵢⱼ = (xᵢ, yⱼ): row j holds yⱼ
    Z = np.empty((ny, nx), dtype=float)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            Z[j, i] = float(fn(np.array([x, y]), t, params))

    return xs, ys, Z


def pad_bounds(bounds, padding: float):
    """Expand axis-aligned ``((x_lo, x_hi), (y_lo, y_hi))`` bounds by ``padding``."""
    (x_lo, x_hi), (y_lo, y_hi) = bounds
    pad = float(padding)

    padded = ((x_lo - pad, x_hi + pad), (y_lo - pad, y_hi + pad))

    return padded
