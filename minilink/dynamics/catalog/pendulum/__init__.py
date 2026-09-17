"""Pendulum-style rotational plants."""

from minilink.dynamics.catalog.pendulum.cartpole import (
    CartPole,
    CartPoleWithNoisePort,
    RotatingCartPole,
    UnderactuatedRotatingCartPole,
)
from minilink.dynamics.catalog.pendulum.double_pendulum import Acrobot, DoublePendulum
from minilink.dynamics.catalog.pendulum.pendulum import (
    InvertedPendulum,
    Pendulum,
    PendulumWithNoisePort,
    TwoIndependentPendulums,
)

__all__ = [
    "Acrobot",
    "CartPole",
    "CartPoleWithNoisePort",
    "DoublePendulum",
    "InvertedPendulum",
    "Pendulum",
    "PendulumWithNoisePort",
    "RotatingCartPole",
    "TwoIndependentPendulums",
    "UnderactuatedRotatingCartPole",
]
