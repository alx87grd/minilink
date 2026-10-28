"""The quadratic field and its ellipsoid: value, gradient, schedule, draw and box."""

import numpy as np
import pytest

from minilink.control.design import lqr_gain_schedule
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.fields import EllipsoidSet, FieldSet, QuadraticField

P = np.array([[4.0, 1.0, 0.0], [1.0, 2.0, 0.5], [0.0, 0.5, 3.0]])
CENTER = np.array([0.5, -1.0, 2.0])


def numpy_value(x, center, P):
    d = np.atleast_2d(x) - center
    return np.einsum("ij,jk,ik->i", d, P, d)


def test_value_and_gradient_match_numpy_on_random_points():
    V = QuadraticField(CENTER, P)
    X = np.random.default_rng(1).normal(size=(50, 3))

    np.testing.assert_array_equal(V.value(X), numpy_value(X, CENTER, P))
    assert V(X[0]) == numpy_value(X[0], CENTER, P)[0]
    np.testing.assert_allclose(V.gradient(X[0]), 2.0 * P @ (X[0] - CENTER))


def test_the_exact_gradient_matches_central_differences():
    V = QuadraticField(CENTER, P)
    x = np.array([0.1, 0.2, -0.3])
    eps = 1e-6
    numeric = np.array(
        [(V(x + eps * e) - V(x - eps * e)) / (2 * eps) for e in np.eye(3)]
    )
    np.testing.assert_allclose(V.gradient(x), numeric, rtol=1e-6)


def test_a_shape_that_does_not_match_the_center_is_refused():
    with pytest.raises(ValueError, match="P must be"):
        QuadraticField(np.zeros(2), P)
    with pytest.raises(ValueError, match="schedule"):
        QuadraticField(np.zeros(3), P, t=np.array([0.0, 1.0]))


def test_a_schedule_reads_s_of_t_and_holds_past_its_ends():
    A = np.array([[0.0, 1.0], [0.0, 0.0]])
    B = np.array([[0.0], [1.0]])
    t, _, S = lqr_gain_schedule(A, B, np.eye(2), np.eye(1), 10 * np.eye(2), 2.0)
    V = QuadraticField(np.zeros(2), S, t=t)
    x = np.array([1.0, -0.5])

    np.testing.assert_allclose(V(x, t=t[3]), x @ S[3] @ x)
    np.testing.assert_allclose(
        V(x, t=0.5 * (t[3] + t[4])), x @ (0.5 * (S[3] + S[4])) @ x
    )
    np.testing.assert_allclose(V(x, t=5.0), x @ S[-1] @ x)
    assert isinstance(V.as_set(upper=1.0), FieldSet)
    assert not isinstance(V.as_set(upper=1.0), EllipsoidSet)


def test_the_sublevel_set_draws_inside_and_its_box_holds_every_draw():
    region = QuadraticField(CENTER, P).as_set(upper=2.0)
    points = region.sample(3, 2000)
    box = region.bounding_box()

    assert isinstance(region, EllipsoidSet)
    assert np.all(numpy_value(points, CENTER, P) <= 2.0 + 1e-12)
    assert np.all(points >= box.lower) and np.all(points <= box.upper)
    assert region.contains(points[0])
    np.testing.assert_array_equal(region.sample(None), CENTER)
    assert region.sample(3).shape == (3,)


def test_a_band_is_a_plain_field_set():
    V = QuadraticField(CENTER, P)
    band = V.as_set(lower=1.0, upper=2.0)

    assert not isinstance(band, EllipsoidSet)
    assert not band.contains(CENTER)


@pytest.mark.optional
@pytest.mark.jax
def test_value_and_gradient_trace_under_jit():
    pytest.importorskip("jax")
    jax = require_jax()
    jnp = require_jax_numpy()
    V = QuadraticField(CENTER, P)
    x = np.array([0.3, 0.1, -0.2])

    value = jax.jit(lambda z: V(z))(jnp.asarray(x))
    grad = jax.jit(jax.grad(lambda z: V(z)))(jnp.asarray(x))

    np.testing.assert_allclose(float(value), V(x), rtol=1e-14)
    np.testing.assert_allclose(np.asarray(grad), V.gradient(x), rtol=1e-14)
