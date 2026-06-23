from dataclasses import dataclass

import numpy as np
import pytest

from minilink.core.backends import array_module
from minilink.core.geometry import Sphere
from minilink.planning.environment.environment import Environment
from minilink.planning.environment.features import GaussianField
from minilink.planning.environment.fields import StateField
from minilink.planning.environment.robot import (
    PlanarBody,
    RobotBody,
    apply_transform,
    disc,
    point,
)


# --- test doubles ----------------------------------------------------------


@dataclass(frozen=True)
class _ConstField(StateField):
    vec: np.ndarray

    def value(self, x, u=None, t=0.0, params=None):
        return np.asarray(self.vec, dtype=float)


class _TwoSphereBody(RobotBody):
    """Two parts: one at the state position, one offset by +2 in x."""

    @property
    def shapes(self):
        return (Sphere(np.zeros(2), 0.1), Sphere(np.zeros(2), 0.2))

    def body_poses(self, x, u=None, t=0.0, params=None):
        def planar(tx, ty):
            return np.array([[1.0, 0.0, tx], [0.0, 1.0, ty], [0.0, 0.0, 1.0]])

        return (planar(x[0], x[1]), planar(x[0] + 2.0, x[1]))


# --- robot body ------------------------------------------------------------


def test_planar_body_translation():
    body = disc(radius=0.3, position=(0, 1))
    (T,) = body.body_poses(np.array([1.5, -2.0]))
    assert T.shape == (3, 3)
    assert apply_transform(T, np.zeros(2)) == pytest.approx([1.5, -2.0])


def test_planar_body_heading_rotates_offset_probe():
    body = PlanarBody(Sphere([1.0, 0.0], 0.1), position=(0, 1), heading=2)
    (T,) = body.body_poses(np.array([0.0, 0.0, np.pi / 2]))
    # a body point at (1, 0) rotates by +90 deg to (0, 1)
    assert apply_transform(T, np.array([1.0, 0.0])) == pytest.approx([0.0, 1.0], abs=1e-9)


def test_multibody_clearance_has_one_component_per_part():
    env = Environment(obstacles=(Sphere([0.0, 0.0], 0.5),))
    value = env.clearance_field(_TwoSphereBody()).value(np.array([1.0, 0.0]))
    # part 0 at (1,0) radius 0.1; part 1 at (3,0) radius 0.2
    assert value.shape == (2,)
    assert value[0] == pytest.approx(1.0 - 0.5 - 0.1)
    assert value[1] == pytest.approx(3.0 - 0.5 - 0.2)


# --- environment + clearance field ----------------------------------------


def test_clearance_value_against_obstacle():
    env = Environment(obstacles=(Sphere([2.0, 2.0], 0.5),))
    field = env.clearance_field(disc(radius=0.3))
    assert field.value(np.array([0.0, 0.0]))[0] == pytest.approx(
        np.hypot(2.0, 2.0) - 0.5 - 0.3
    )
    assert field.value(np.array([2.0, 2.0]))[0] == pytest.approx(-0.5 - 0.3)


def test_empty_environment_raises():
    env = Environment()
    with pytest.raises(ValueError):
        env.clearance(np.zeros(2))
    with pytest.raises(ValueError):
        env.clearance_field(disc(0.3))


# --- soft traversability ---------------------------------------------------


def test_gaussian_field_peak_and_decay():
    field = GaussianField(center=[1.0, 2.0], amplitude=3.0, sigma=0.5)
    assert field.density(np.array([1.0, 2.0])) == pytest.approx(3.0)
    assert field.density(np.array([2.0, 2.0])) == pytest.approx(
        3.0 * np.exp(-0.5 * (1.0 / 0.5) ** 2)
    )


def test_environment_cost_density_sums_fields():
    f1 = GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=1.0)
    f2 = GaussianField(center=[1.0, 0.0], amplitude=2.0, sigma=1.0)
    env = Environment(fields=(f1, f2))
    p = np.array([0.0, 0.0])
    assert env.cost_density(p) == pytest.approx(f1.density(p) + f2.density(p))


def test_cost_field_value_at_robot_position():
    center = np.array([2.0, 2.0])
    env = Environment(fields=(GaussianField(center=center, amplitude=1.5, sigma=0.8),))
    x = np.array([0.0, 0.0])
    assert env.cost_field(disc(radius=0.3)).value(x)[0] == pytest.approx(
        env.cost_density(x)
    )


def test_cost_field_as_cost_weight_and_peak():
    env = Environment(fields=(GaussianField(center=[0.0, 0.0], amplitude=2.0, sigma=0.5),))
    cost = env.cost_field(disc(0.2)).as_cost(weight=4.0)
    at_peak = cost.g(np.array([0.0, 0.0]), np.zeros(1))
    far = cost.g(np.array([5.0, 0.0]), np.zeros(1))
    assert at_peak == pytest.approx(8.0)
    assert at_peak > far


def test_cost_field_as_constraint_keep_out_band():
    env = Environment(fields=(GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=0.5),))
    keep_out = env.cost_field(point()).as_constraint(upper=0.2)
    assert keep_out.contains(np.array([2.0, 0.0]))
    assert not keep_out.contains(np.array([0.0, 0.0]))


def test_empty_fields_return_zero_density_and_cost_field():
    env = Environment()
    assert env.cost_density(np.zeros(2)) == pytest.approx(0.0)
    assert env.cost_field(disc(0.3)).value(np.array([1.0, 2.0]))[0] == pytest.approx(0.0)


# --- FieldSet --------------------------------------------------------------


def test_fieldset_contains_matches_minkowski_sum():
    center, obstacle_r, robot_r = np.array([2.0, 2.0]), 0.5, 0.3
    env = Environment(obstacles=(Sphere(center, obstacle_r),))
    free = env.clearance_field(disc(radius=robot_r)).as_constraint()

    rng = np.random.default_rng(0)
    for x in rng.uniform(-1.0, 5.0, size=(200, 2)):
        expected_free = np.linalg.norm(x - center) >= obstacle_r + robot_r
        assert free.contains(x) == expected_free


def test_disc_radius_shifts_free_boundary():
    env = Environment(obstacles=(Sphere([0.0, 0.0], 0.5),))
    x = np.array([0.65, 0.0])  # 0.5 < dist < 0.5 + 0.3
    assert env.clearance_field(point()).as_constraint().contains(x)
    assert not env.clearance_field(disc(0.3)).as_constraint().contains(x)


def test_fieldset_bound_mechanics():
    f = _ConstField(np.array([0.5]))
    assert f.as_constraint(lower=0.0).contains(np.zeros(1))
    assert not f.as_constraint(lower=None, upper=0.2).contains(np.zeros(1))
    assert f.as_constraint(lower=0.0, upper=1.0).contains(np.zeros(1))


def test_fieldset_requires_a_bound():
    with pytest.raises(ValueError):
        _ConstField(np.array([1.0])).as_constraint(lower=None, upper=None)


# --- FieldCost + cross export ---------------------------------------------


def test_fieldcost_weight_and_sum():
    f = _ConstField(np.array([1.0, 2.0]))
    cost = f.as_cost(weight=3.0)
    assert cost.g(np.zeros(2), np.zeros(1)) == pytest.approx(3.0 * (1.0 + 2.0))


def test_cross_export_constraint_and_barrier_cost():
    env = Environment(obstacles=(Sphere([0.0, 0.0], 0.5),))
    field = env.clearance_field(disc(0.3))
    free = field.as_constraint()

    def barrier(v):
        xp = array_module(v)
        return xp.maximum(0.2 - v, 0.0) ** 2

    cost = field.as_cost(weight=10.0, shaping=barrier)
    inside, far = np.array([0.0, 0.0]), np.array([3.0, 0.0])

    assert not free.contains(inside)
    assert free.contains(far)
    assert cost.g(inside, np.zeros(2)) > 1.0
    assert cost.g(far, np.zeros(2)) == pytest.approx(0.0)


# --- JAX twin --------------------------------------------------------------


def test_jax_twin_margin_matches_and_differentiates():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    env = Environment(obstacles=(Sphere([2.0, 2.0], 0.5),))
    free = env.clearance_field(disc(0.3)).as_constraint()
    x = np.array([0.0, 0.0])

    assert float(free.margin(jnp.array(x))[0]) == pytest.approx(float(free.margin(x)[0]))

    gradient = jax.grad(lambda q: jnp.sum(free.margin(q)))(jnp.array(x))
    assert np.asarray(gradient).shape == (2,)


def test_jax_twin_cost_field_matches_and_differentiates():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    env = Environment(fields=(GaussianField(center=[1.0, 1.0], amplitude=2.0, sigma=0.7),))
    cost = env.cost_field(disc(0.2)).as_cost(weight=1.5)
    x = np.array([0.5, 0.5])

    assert cost.g(jnp.array(x), jnp.zeros(1)) == pytest.approx(cost.g(x, np.zeros(1)))

    gradient = jax.grad(lambda q: cost.g(q, jnp.zeros(1)))(jnp.array(x))
    assert np.asarray(gradient).shape == (2,)
