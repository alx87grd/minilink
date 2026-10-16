"""Workspace fields: smooth penalty densities over a workspace point ``p`` in ℝ² or ℝ³."""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module

###############################################################################
# Public API
###############################################################################


class WorkspaceField(ABC):
    """
    Base class for workspace penalty densities.

    A point carries penalty ``density(p) >= 0``. The field may depend on time
    and optional parameters, so the generic signature is ``density(p, t, params)``.

    A field models a smooth penalty over the workspace — mud, slope preference,
    heat maps — not a solid region. Densities sum at the scene level into
    :meth:`~minilink.planning.spatial.scene.Scene.cost_density`.

    Construction is a NumPy/Python boundary; :meth:`density` is a native-array
    math path that stays NumPy under NumPy input and traces under JAX.
    """

    @abstractmethod
    def density(self, p, t=0.0, params=None):
        """Return the nonnegative penalty density at workspace point ``p``."""
        ...


@dataclass(frozen=True)
class GaussianField(WorkspaceField):
    """
    Smooth Gaussian penalty peak centered at ``center``.

    Parameters
    ----------
    center : array_like
        Peak location with shape ``(dim,)``.
    amplitude : float
        Peak density at ``center``.
    sigma : float
        Standard deviation of the Gaussian envelope.
    """

    center: np.ndarray
    amplitude: float
    sigma: float

    def __post_init__(self) -> None:
        # Parameters: a flat float center, a nonnegative peak, a positive width
        center = np.asarray(self.center, dtype=float).reshape(-1).copy()
        amplitude = float(self.amplitude)
        sigma = float(self.sigma)
        require_nonnegative_amplitude(amplitude)
        require_positive_sigma(sigma)

        # Stored on the frozen dataclass
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "amplitude", amplitude)
        object.__setattr__(self, "sigma", sigma)

    def density(self, p, t=0.0, params=None):
        xp = array_module(p)

        center = self.center
        amplitude = self.amplitude
        sigma = self.sigma

        # density(p) = A exp(−½ ‖p − c‖² / σ²)
        d2 = xp.sum((p - center) ** 2)
        density = amplitude * xp.exp(-0.5 * d2 / sigma**2)

        return density


###############################################################################
# Internal machinery
###############################################################################


def require_nonnegative_amplitude(amplitude):
    """Refuse a negative peak density."""
    if amplitude < 0.0:
        raise ValueError("amplitude must be nonnegative")


def require_positive_sigma(sigma):
    """Refuse a Gaussian envelope of zero or negative width."""
    if sigma <= 0.0:
        raise ValueError("sigma must be positive")
