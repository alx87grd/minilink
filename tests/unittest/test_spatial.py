from dataclasses import dataclass

import numpy as np
import pytest

from minilink.core.backends import array_module
from minilink.core.geometry import Sphere
from minilink.planning.spatial.robot import (
    RobotBody,
    apply_transform,
    point,
    sphere,
)
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import (
    inverse_barrier,
    occupancy,
    quadratic_hinge,
)
from minilink.planning.spatial.state_fields import StateField
from minilink.planning.spatial.workspace_fields import GaussianField

# --- test doubles ----------------------------------------------------------


@dataclass(frozen=True)
class _ConstField(StateField):
    val: float

    def value(self, x, u=None, t=0.0, params=None):
        return float(self.val)


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


def test_translation_body_pose():
    body = sphere(radius=0.3, position=(0, 1))
    (T,) = body.body_poses(np.array([1.5, -2.0]))
    assert T.shape == (3, 3)
    assert apply_transform(T, np.zeros(2)) == pytest.approx([1.5, -2.0])


def test_translation_body_in_3d():
    body = sphere(radius=0.5, position=(0, 1, 2))
    (T,) = body.body_poses(np.array([1.0, 2.0, 3.0]))
    assert T.shape == (4, 4)
    assert apply_transform(T, np.zeros(3)) == pytest.approx([1.0, 2.0, 3.0])


def test_multibody_clearance_is_worst_case_over_parts():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    value = scene.clearance_field(_TwoSphereBody()).value(np.array([1.0, 0.0]))
    # part 0 at (1,0) radius 0.1 -> 0.4; part 1 at (3,0) radius 0.2 -> 2.3; min wins
    assert np.ndim(value) == 0
    assert value == pytest.approx(min(1.0 - 0.5 - 0.1, 3.0 - 0.5 - 0.2))


# --- scene + clearance field -----------------------------------------------


def test_clearance_value_against_obstacle():
    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    field = scene.clearance_field(sphere(radius=0.3))
    assert field.value(np.array([0.0, 0.0])) == pytest.approx(
        np.hypot(2.0, 2.0) - 0.5 - 0.3
    )
    assert field.value(np.array([2.0, 2.0])) == pytest.approx(-0.5 - 0.3)


def test_clearance_pipeline_in_3d():
    scene = Scene(obstacles=(Sphere([0.0, 0.0, 0.0], 1.0),))
    field = scene.clearance_field(sphere(0.5, position=(0, 1, 2)))
    assert field.value(np.array([3.0, 0.0, 0.0])) == pytest.approx(3.0 - 1.0 - 0.5)
    free = field.as_constraint()
    assert free.contains(np.array([3.0, 0.0, 0.0]))
    assert not free.contains(np.array([0.5, 0.0, 0.0]))


def test_position_indices_decouple_state_size():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    robot = sphere(0.3, position=(0, 1))
    x = np.array([2.0, 0.0, 99.0, 99.0, 99.0])  # extra state ignored
    assert scene.clearance_field(robot).value(x) == pytest.approx(2.0 - 0.5 - 0.3)


def test_empty_scene_raises():
    scene = Scene()
    with pytest.raises(ValueError):
        scene.clearance(np.zeros(2))
    with pytest.raises(ValueError):
        scene.clearance_field(sphere(0.3))


# --- workspace fields ------------------------------------------------------


def test_gaussian_field_peak_and_decay():
    field = GaussianField(center=[1.0, 2.0], amplitude=3.0, sigma=0.5)
    assert field.density(np.array([1.0, 2.0])) == pytest.approx(3.0)
    assert field.density(np.array([2.0, 2.0])) == pytest.approx(
        3.0 * np.exp(-0.5 * (1.0 / 0.5) ** 2)
    )


def test_scene_cost_density_sums_workspace_fields():
    f1 = GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=1.0)
    f2 = GaussianField(center=[1.0, 0.0], amplitude=2.0, sigma=1.0)
    scene = Scene(workspace_fields=(f1, f2))
    p = np.array([0.0, 0.0])
    assert scene.cost_density(p) == pytest.approx(f1.density(p) + f2.density(p))


def test_cost_field_value_at_robot_position():
    center = np.array([2.0, 2.0])
    scene = Scene(
        workspace_fields=(GaussianField(center=center, amplitude=1.5, sigma=0.8),)
    )
    x = np.array([0.0, 0.0])
    assert scene.cost_field(sphere(radius=0.3)).value(x) == pytest.approx(
        scene.cost_density(x)
    )


def test_cost_field_as_cost_weight_and_peak():
    scene = Scene(
        workspace_fields=(GaussianField(center=[0.0, 0.0], amplitude=2.0, sigma=0.5),)
    )
    cost = scene.cost_field(sphere(0.2)).as_cost(weight=4.0)
    at_peak = cost.g(np.array([0.0, 0.0]), np.zeros(1))
    far = cost.g(np.array([5.0, 0.0]), np.zeros(1))
    assert at_peak == pytest.approx(8.0)
    assert at_peak > far


def test_cost_field_as_constraint_keep_out_band():
    scene = Scene(
        workspace_fields=(GaussianField(center=[0.0, 0.0], amplitude=1.0, sigma=0.5),)
    )
    keep_out = scene.cost_field(point()).as_constraint(upper=0.2)
    assert keep_out.contains(np.array([2.0, 0.0]))
    assert not keep_out.contains(np.array([0.0, 0.0]))


def test_empty_workspace_fields_return_zero_density_and_cost_field():
    scene = Scene()
    assert scene.cost_density(np.zeros(2)) == pytest.approx(0.0)
    assert scene.cost_field(sphere(0.3)).value(np.array([1.0, 2.0])) == pytest.approx(
        0.0
    )


# --- FieldSet --------------------------------------------------------------


def test_fieldset_contains_matches_minkowski_sum():
    center, obstacle_r, robot_r = np.array([2.0, 2.0]), 0.5, 0.3
    scene = Scene(obstacles=(Sphere(center, obstacle_r),))
    free = scene.clearance_field(sphere(radius=robot_r)).as_constraint()

    rng = np.random.default_rng(0)
    for x in rng.uniform(-1.0, 5.0, size=(200, 2)):
        expected_free = np.linalg.norm(x - center) >= obstacle_r + robot_r
        assert free.contains(x) == expected_free


def test_sphere_radius_shifts_free_boundary():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    x = np.array([0.65, 0.0])  # 0.5 < dist < 0.5 + 0.3
    assert scene.clearance_field(point()).as_constraint().contains(x)
    assert not scene.clearance_field(sphere(0.3)).as_constraint().contains(x)


def test_fieldset_bound_mechanics():
    f = _ConstField(0.5)
    assert f.as_constraint(lower=0.0).contains(np.zeros(1))
    assert not f.as_constraint(lower=None, upper=0.2).contains(np.zeros(1))
    assert f.as_constraint(lower=0.0, upper=1.0).contains(np.zeros(1))


def test_fieldset_requires_a_bound():
    with pytest.raises(ValueError):
        _ConstField(1.0).as_constraint(lower=None, upper=None)


# --- FieldCost + cross export ---------------------------------------------


def test_fieldcost_weight_and_shaping():
    f = _ConstField(2.0)
    assert f.as_cost(weight=3.0).g(np.zeros(2), np.zeros(1)) == pytest.approx(6.0)
    squared = f.as_cost(weight=1.0, shaping=lambda v: v**2)
    assert squared.g(np.zeros(2), np.zeros(1)) == pytest.approx(4.0)


def test_cross_export_constraint_and_barrier_cost():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    field = scene.clearance_field(sphere(0.3))
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


# --- shaping ---------------------------------------------------------------


def test_quadratic_hinge():
    s = quadratic_hinge(threshold=1.0)
    assert s(np.asarray(2.0)) == pytest.approx(0.0)  # free of the margin
    assert s(np.asarray(0.0)) == pytest.approx(1.0)  # (1 - 0)^2
    assert s(np.asarray(-1.0)) == pytest.approx(4.0)  # (1 - (-1))^2


def test_inverse_barrier_blows_up_at_contact():
    s = inverse_barrier(epsilon=0.1)
    assert s(np.asarray(1.0)) == pytest.approx(1.0)
    assert s(np.asarray(0.0)) == pytest.approx(100.0)  # clamped at epsilon
    assert s(np.asarray(-5.0)) == pytest.approx(100.0)


def test_occupancy_is_bounded_unit_interval():
    s = occupancy(scale=0.5)
    assert s(np.asarray(0.0)) == pytest.approx(0.5)  # boundary
    assert s(np.asarray(5.0)) < 1e-3  # free -> ~0
    assert s(np.asarray(-5.0)) > 1.0 - 1e-3  # collision -> ~1


def test_shaping_composes_with_clearance_field():
    scene = Scene(obstacles=(Sphere([0.0, 0.0], 0.5),))
    field = scene.clearance_field(sphere(0.2))
    bounded = field.as_cost(weight=3.0, shaping=occupancy(scale=0.1))
    # bounded occupancy keeps the per-step obstacle cost in [0, weight]
    assert 0.0 <= bounded.g(np.array([0.0, 0.0]), np.zeros(2)) <= 3.0
    assert bounded.g(np.array([5.0, 0.0]), np.zeros(2)) < 1e-2  # ~free


# --- JAX twin --------------------------------------------------------------


def test_jax_twin_margin_matches_and_differentiates():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    scene = Scene(obstacles=(Sphere([2.0, 2.0], 0.5),))
    free = scene.clearance_field(sphere(0.3)).as_constraint()
    x = np.array([0.0, 0.0])

    assert float(free.margin(jnp.array(x))[0]) == pytest.approx(
        float(free.margin(x)[0])
    )

    gradient = jax.grad(lambda q: jnp.sum(free.margin(q)))(jnp.array(x))
    assert np.asarray(gradient).shape == (2,)


# --- plotting --------------------------------------------------------------


def test_scene_plot_smoke():
    import os

    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)

    scene = Scene(
        obstacles=(Sphere([4.0, 0.0], 0.5),),
        workspace_fields=(GaussianField([2.0, 1.5], 2.0, 1.0),),
    )
    fig, ax = scene.plot(show=False)
    assert fig is not None
    assert ax is not None

    fig, ax = scene.plot(show=False, robot=sphere(0.25), x=np.array([2.0, 0.0]))
    assert len(ax.patches) >= 2

    _, ax = scene.plot(show=False, robot=point(), x=np.array([2.0, 0.0]))
    assert len(ax.lines) >= 1


def test_car_body_collision_depends_on_heading():
    from minilink.core.geometry import Box
    from minilink.planning.spatial.robot import car

    # a vertical gap (width 0.8) between two walls
    scene = Scene(
        obstacles=(Box([-3.0, -3.0], [-0.4, 3.0]), Box([0.4, -3.0], [3.0, 3.0]))
    )
    field = scene.clearance_field(car(length=1.6, width=0.5))
    assert field.value(np.array([0.0, 0.0, np.pi / 2])) > 0.0  # aligned: fits the gap
    assert field.value(np.array([0.0, 0.0, 0.0])) < 0.0  # across: spans into the walls
