"""Field grids: a scalar field sampled on a 2-D axis-aligned grid, as plain NumPy arrays."""

from collections.abc import Callable, Sequence
from dataclasses import dataclass

import numpy as np

###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class FieldGrid:
    """
    A scalar field sampled on a 2-D axis-aligned grid.

    Plain NumPy arrays, no Matplotlib: the heatmaps of
    :mod:`minilink.planning.spatial.plotting` draw them.

    Parameters
    ----------
    xs : np.ndarray
        Grid coordinates along the first axis, shape ``(nx,)``.
    ys : np.ndarray
        Grid coordinates along the second axis, shape ``(ny,)``.
    Z : np.ndarray
        Sampled values, shape ``(ny, nx)`` (row ``j`` holds ``ys[j]``).
    """

    xs: np.ndarray
    ys: np.ndarray
    Z: np.ndarray

    @property
    def extent(self) -> tuple:
        """``(x_min, x_max, y_min, y_max)`` for image-style plotting."""
        return (
            float(self.xs[0]),
            float(self.xs[-1]),
            float(self.ys[0]),
            float(self.ys[-1]),
        )


def sample_grid(
    fn: Callable,
    bounds,
    *,
    grid: tuple = (120, 120),
    t: float = 0.0,
    params=None,
) -> FieldGrid:
    """
    Sample a scalar query over a 2-D axis-aligned grid.

    Scenes and state fields rasterize their clearance and cost-density queries
    with it, for heatmaps.

    Parameters
    ----------
    fn : callable
        Scalar query ``fn(p, t, params) -> float`` over a workspace point ``p``.
    bounds : ((x_min, x_max), (y_min, y_max))
        Grid extent along each axis.
    grid : (nx, ny)
        Number of samples along each axis.
    """
    (x_lo, x_hi), (y_lo, y_hi) = bounds
    nx, ny = grid

    # The nodes along each axis
    xs = np.linspace(x_lo, x_hi, nx)
    ys = np.linspace(y_lo, y_hi, ny)

    # Zⱼᵢ = fn(pᵢⱼ, t, params) at pᵢⱼ = (xᵢ, yⱼ): row j holds yⱼ
    Z = np.empty((ny, nx), dtype=float)
    for j, y in enumerate(ys):
        for i, x in enumerate(xs):
            Z[j, i] = float(fn(np.array([x, y]), t, params))

    return FieldGrid(xs, ys, Z)


def sample_field_costs(
    costs: Sequence,
    *,
    bounds,
    grid: tuple = (200, 200),
    state_dim: int = 2,
    u=None,
    t: float = 0.0,
    params=None,
) -> FieldGrid:
    """
    Rasterize shaped ``FieldCost`` terms on a workspace XY grid.

    Each cell is the cost of a **workspace point** ``p = (x, y)`` using a
    **point probe** — build costs with ``mount(point_probe(), on=sys)`` so the
    field does not depend on robot heading or body extent. ``state_dim`` must
    match the planner plant state size (``sys.n``). The MPC tuning plots of
    :mod:`minilink.planning.spatial.plotting` draw the result.
    """
    require_costs(costs)

    # The held input, and the state whose first two entries carry the point
    u_use = np.zeros(1) if u is None else u
    x = np.zeros(int(state_dim))

    def fn(p, t_query=0.0, params_query=None):
        x[0] = p[0]
        x[1] = p[1]

        # g(p) = Σ_c g_c(x(p), u, t), summed in the terms' order
        g = sum(float(c.g(x, u_use, t=t_query, params=params_query)) for c in costs)

        return g

    return sample_grid(fn, bounds, grid=grid, t=t, params=params)


def pad_bounds(bounds, padding: float):
    """Expand axis-aligned ``((x_lo, x_hi), (y_lo, y_hi))`` bounds by ``padding``."""
    (x_lo, x_hi), (y_lo, y_hi) = bounds
    pad = float(padding)

    padded = ((x_lo - pad, x_hi + pad), (y_lo - pad, y_hi + pad))

    return padded


###############################################################################
# Internal machinery
###############################################################################


def require_costs(costs):
    """Refuse a raster with no cost term."""
    if not costs:
        raise ValueError("sample_field_costs requires at least one cost")
