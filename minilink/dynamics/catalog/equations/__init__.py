"""Equation-level catalog systems."""

from minilink.dynamics.catalog.equations.integrators import (
    DoubleIntegrator,
    TripleIntegrator,
)
from minilink.dynamics.catalog.equations.oscillators import Lorenz, VanDerPol

__all__ = [
    "DoubleIntegrator",
    "Lorenz",
    "TripleIntegrator",
    "VanDerPol",
]
