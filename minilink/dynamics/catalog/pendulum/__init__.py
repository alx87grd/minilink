"""Pendulum-style rotational plants."""

from minilink.dynamics.catalog.pendulum.cartpole import (
    CartPole,
    NoisyCartPole,
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot, DoublePendulum
from minilink.dynamics.catalog.pendulum.pendulum import (
    InvertedPendulum,
    NoisyPendulum,
    Pendulum,
    TwoIndependentPendulums,
)

__all__ = [
    "Acrobot",
    "CartPole",
    "NoisyCartPole",
    "DoublePendulum",
    "InvertedPendulum",
    "Pendulum",
    "NoisyPendulum",
    "RotatingCartPole",
    "TwoIndependentPendulums",
    "UnderactuatedRotatingCartPole",
]
