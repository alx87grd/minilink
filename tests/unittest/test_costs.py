import numpy as np
import pytest

from minilink.core.costs import (
    CostFunction,
    QuadraticCost,
    ScaledCost,
    SumCost,
    TimeCost,
)
from minilink.core.trajectory import Trajectory


def test_time_cost_is_unity_off_target_zero_on_target():
    cost = TimeCost(xbar=np.zeros(2), eps=0.1)
    assert cost.g(np.array([1.0, 0.0]), np.zeros(1)) == 1.0
    assert cost.g(np.array([0.02, 0.0]), np.zeros(1)) == 0.0
    assert cost.h(np.array([1.0, 0.0])) == 0.0


def test_time_cost_jax_matches_numpy():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    cost = TimeCost(xbar=np.zeros(2), eps=0.1)
    states = jnp.array([[1.0, 0.0], [0.02, 0.0], [0.0, 0.0]])
    u = jnp.zeros(1)

    def g_at(x):
        return cost.g(x, u)

    jax_values = jax.vmap(g_at)(states)
    numpy_values = np.array([cost.g(np.asarray(x), np.zeros(1)) for x in states])
    assert np.allclose(np.asarray(jax_values), numpy_values)


def make_quadratic(n: int = 2, m: int = 1) -> QuadraticCost:
    return QuadraticCost(
        Q=np.eye(n),
        R=np.eye(m),
        S=np.zeros((n, n)),
        xbar=np.zeros(n),
        ubar=np.zeros(m),
    )


class _ParamCost(CostFunction):
    """Running cost that reads ``params['w']`` to check forwarding."""

    def g(self, x, u, t=0.0, params=None):
        return 0.0 if params is None else float(params["w"])

    def h(self, x, t=0.0, params=None):
        return 0.0


def test_sum_cost_is_additive():
    c1 = make_quadratic()
    c2 = make_quadratic()
    s = c1 + c2
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    assert isinstance(s, SumCost)
    assert s.g(x, u) == pytest.approx(c1.g(x, u) + c2.g(x, u))
    assert s.h(x) == pytest.approx(c1.h(x) + c2.h(x))


def test_add_flattens_nested_sums():
    c = make_quadratic()
    s = c + c + c

    assert isinstance(s, SumCost)
    assert len(s.terms) == 3


def test_sum_builtin_uses_radd_zero_seed():
    costs = [make_quadratic() for _ in range(3)]
    s = sum(costs)  # 0 + c0 + c1 + c2
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    assert isinstance(s, SumCost)
    assert s.g(x, u) == pytest.approx(3.0 * costs[0].g(x, u))


def test_scaled_cost_weights():
    c = make_quadratic()
    x = np.array([1.0, 2.0])
    u = np.array([3.0])

    weighted = 2.5 * c
    assert isinstance(weighted, ScaledCost)
    assert weighted.g(x, u) == pytest.approx(2.5 * c.g(x, u))

    combo = c + 2.0 * c  # reads as base + weight * other
    assert combo.g(x, u) == pytest.approx(3.0 * c.g(x, u))


def test_params_forwarded_to_terms():
    s = make_quadratic() + _ParamCost()
    x = np.array([1.0, 2.0])
    u = np.array([3.0])
    base = make_quadratic().g(x, u)

    assert s.g(x, u, params={"w": 5.0}) == pytest.approx(base + 5.0)
    assert (3.0 * _ParamCost()).g(x, u, params={"w": 2.0}) == pytest.approx(6.0)


def test_total_cost_on_trajectory():
    c = make_quadratic()
    s = c + c
    t = np.linspace(0.0, 1.0, 5)
    traj = Trajectory(t=t, x=np.ones((2, 5)), u=np.ones((1, 5)))

    assert s.total_cost(traj) == pytest.approx(2.0 * c.total_cost(traj))


def test_jax_twin_cost_grad():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    s = make_quadratic() + make_quadratic()

    def g(x):
        return s.g(x, jnp.zeros(1))

    gradient = jax.grad(g)(jnp.array([1.0, 2.0]))
    assert np.asarray(gradient).shape == (2,)
