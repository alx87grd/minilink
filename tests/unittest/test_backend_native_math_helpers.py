"""Tests for backend-native set and cost equation helpers."""

import numpy as np
import pytest

from minilink.core.costs import QuadraticCost
from minilink.core.sets import (
    BallSet,
    BoxSet,
    CallableSet,
    IntersectionSet,
    SingletonSet,
)
from minilink.core.trajectory import Trajectory


def _quadratic_cost() -> QuadraticCost:
    return QuadraticCost(
        Q=np.eye(2),
        R=np.eye(1),
        S=2.0 * np.eye(2),
        xbar=np.array([1.0, -1.0]),
        ubar=np.array([0.5]),
    )


def test_set_margins_are_numpy_arrays():
    z = np.array([0.25, -0.5])
    box = BoxSet(lower=np.array([-1.0, -2.0]), upper=np.array([1.0, 2.0]))
    singleton = SingletonSet(np.array([1.0, -1.0]))
    ball = BallSet(center=np.zeros(2), radius=1.0)
    intersection = IntersectionSet([box, ball])

    np.testing.assert_allclose(box.margin(z), [1.25, 1.5, 0.75, 2.5])
    np.testing.assert_allclose(singleton.residual(z), [-0.75, 0.5])
    np.testing.assert_allclose(singleton.margin(z), [-0.75, -0.5])
    np.testing.assert_allclose(ball.margin(z), [1.0 - np.linalg.norm(z)])
    np.testing.assert_allclose(
        intersection.margin(z),
        np.concatenate((box.margin(z), ball.margin(z))),
    )


def test_callable_set_margin_preserves_numpy_output():
    set_ = CallableSet(
        lambda z, t, params: np.array([z[0] + t, params["limit"] - z[1]])
    )

    margin = set_.margin(np.array([1.0, 2.0]), t=0.5, params={"limit": 3.0})

    np.testing.assert_allclose(margin, [1.5, 1.0])


def test_quadratic_cost_numpy_math_and_reporting_helpers():
    cost = _quadratic_cost()
    x = np.array([2.0, 1.0])
    u = np.array([1.5])

    assert np.isclose(cost.g(x, u), 6.0)
    assert np.isclose(cost.h(x), 10.0)

    traj = Trajectory(
        t=np.array([0.0, 1.0]),
        x=np.column_stack((x, x)),
        u=np.column_stack((u, u)),
    )
    evaluated = cost.evaluate_trajectory(traj)
    assert isinstance(cost.terminal_cost(traj), float)
    assert isinstance(cost.total_cost(traj), float)
    np.testing.assert_allclose(evaluated.signals["cost_rate"], [[6.0, 6.0]])
    np.testing.assert_allclose(evaluated.signals["cost"], [[0.0, 6.0]])


def test_set_margins_are_jax_jittable():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from minilink.compile.jax_utils import configure_jax

    configure_jax(enable_x64=True)

    box = BoxSet(lower=np.array([-1.0, -2.0]), upper=np.array([1.0, 2.0]))
    singleton = SingletonSet(np.array([1.0, -1.0]))
    ball = BallSet(center=np.zeros(2), radius=1.0)
    callable_set = CallableSet(lambda z, t, params: jnp.array([z[0] + t]))
    intersection = IntersectionSet([box, ball])
    z = jnp.asarray([0.25, -0.5])

    np.testing.assert_allclose(
        np.asarray(jax.jit(box.margin)(z)),
        box.margin(np.asarray(z)),
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(singleton.residual)(z)),
        singleton.residual(np.asarray(z)),
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(singleton.margin)(z)),
        singleton.margin(np.asarray(z)),
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(ball.margin)(z)),
        ball.margin(np.asarray(z)),
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(callable_set.margin)(z, 0.5, None)),
        [0.75],
    )
    np.testing.assert_allclose(
        np.asarray(jax.jit(intersection.margin)(z)),
        intersection.margin(np.asarray(z)),
    )


def test_quadratic_cost_is_jax_jittable():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from minilink.compile.jax_utils import configure_jax

    configure_jax(enable_x64=True)

    cost = _quadratic_cost()
    x = jnp.asarray([2.0, 1.0])
    u = jnp.asarray([1.5])

    def J(x, u):
        return cost.g(x, u) + cost.h(x)

    assert np.isclose(float(jax.jit(J)(x, u)), 16.0)
