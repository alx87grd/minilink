"""Reusable synthetic systems for tests and benchmarks.

This package centralizes test/benchmark-only builders that are shared by
manual benchmark scripts and future performance tooling.
"""

from minilink.blocks.testing.basic import JaxPendulum, NumpyPendulum, make_pendulum
from minilink.blocks.testing.engine import make_physics_many_spheres
from minilink.blocks.testing.network import (
    MultiInputNode,
    SimpleGain,
    SimpleIntegrator,
    build_deep_network,
    make_dense_network,
)

__all__ = [
    "JaxPendulum",
    "MultiInputNode",
    "NumpyPendulum",
    "SimpleGain",
    "SimpleIntegrator",
    "build_deep_network",
    "make_dense_network",
    "make_pendulum",
    "make_physics_many_spheres",
]
