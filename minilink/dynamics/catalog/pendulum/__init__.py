"""Pendulum-style rotational plants."""

from minilink.dynamics.catalog.pendulum.cartpole import (
    CartPole,
    JaxCartPole,
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
    "DoublePendulum",
    "InvertedPendulum",
    "JaxCartPole",
    "Pendulum",
    "PendulumWithNoisePort",
    "RotatingCartPole",
    "TwoIndependentPendulums",
    "UnderactuatedRotatingCartPole",
]
