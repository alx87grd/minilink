"""Tests for :func:`minilink.core.costs.require_jax_traceable_cost`."""

import numpy as np
import pytest

from minilink.core.costs import (
    CostFunction,
    QuadraticCost,
    require_jax_traceable_cost,
)


def _identity_quadratic() -> QuadraticCost:
    return QuadraticCost(
        Q=np.eye(2),
        R=np.eye(1),
        S=np.eye(2),
        xbar=np.zeros(2),
        ubar=np.zeros(1),
    )


def test_rejects_plain_quadratic_cost():
    with pytest.raises(ValueError, match="JaxQuadraticCost"):
        require_jax_traceable_cost(_identity_quadratic())


def test_accepts_jax_quadratic_cost():
    pytest.importorskip("jax")
    from minilink.core.costs import JaxQuadraticCost

    cost = JaxQuadraticCost(
        Q=np.eye(2),
        R=np.eye(1),
        S=np.eye(2),
        xbar=np.zeros(2),
        ubar=np.zeros(1),
    )
    require_jax_traceable_cost(cost)  # must not raise


def test_accepts_non_quadratic_cost():
    class NoOpCost(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return 0.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    require_jax_traceable_cost(NoOpCost())  # must not raise (rule only gates QuadraticCost)
