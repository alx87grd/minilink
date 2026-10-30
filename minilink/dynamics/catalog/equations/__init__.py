"""Equation-level catalog systems."""

from minilink.dynamics.catalog.equations.integrators import (
    DoubleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import Lorenz, VanderPol

__all__ = [
    "DoubleIntegrator",
    "Lorenz",
    "TripleIntegrator",
    "VanderPol",
]
