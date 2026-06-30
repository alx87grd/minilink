"""
Workspace penalty fields on ``p in R^2`` or ``R^3``.

A :class:`WorkspaceField` models a smooth penalty density over the workspace —
mud, slope preference, heat maps — not a solid region. Densities sum at the
scene level into :meth:`~minilink.planning.spatial.scene.Scene.cost_density`.

Construction is a NumPy/Python boundary; :meth:`WorkspaceField.density` is a
native-array math path that stays NumPy under NumPy input and traces under JAX.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np

from minilink.core.backends import array_module

# Public API


class WorkspaceField(ABC):
    """
    Base class for workspace penalty densities.

    A point carries penalty ``density(p) >= 0``. The field may depend on time
    and optional parameters, so the generic signature is ``density(p, t, params)``.
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
        center = np.asarray(self.center, dtype=float).reshape(-1).copy()
        amplitude = float(self.amplitude)
        sigma = float(self.sigma)
        if amplitude < 0.0:
            raise ValueError("amplitude must be nonnegative")
        if sigma <= 0.0:
            raise ValueError("sigma must be positive")
        object.__setattr__(self, "center", center)
        object.__setattr__(self, "amplitude", amplitude)
        object.__setattr__(self, "sigma", sigma)

    def density(self, p, t=0.0, params=None):
        xp = array_module(p)
        center = self.center
        amplitude = self.amplitude
        sigma = self.sigma

        d2 = xp.sum((p - center) ** 2)

        # Gaussian penalty envelope
        return amplitude * xp.exp(-0.5 * d2 / sigma**2)
