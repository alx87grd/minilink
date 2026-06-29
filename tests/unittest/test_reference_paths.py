"""Tests for reference paths and corridor tracking fields."""

import os

import numpy as np
import pytest

from minilink.core.backends import array_module
from minilink.core.geometry import Sphere
from minilink.planning.spatial.paths import PolylinePath, from_waypoints
from minilink.planning.spatial.collision import point, sphere
from minilink.planning.spatial.scene import Scene
from minilink.planning.spatial.shaping import quadratic_excess, quadratic_hinge
from minilink.planning.spatial.track import ReferenceTrack


def test_polyline_distance_to_segment():
    path = PolylinePath([[0.0, 0.0], [10.0, 0.0]])
    assert path.distance(np.array([5.0, 2.0])) == pytest.approx(2.0)
    assert path.distance(np.array([-1.0, 0.0])) == pytest.approx(1.0)
    assert path.distance(np.array([11.0, 0.0])) == pytest.approx(1.0)


def test_polyline_project_and_sample_roundtrip():
    path = PolylinePath([[0.0, 0.0], [3.0, 4.0], [3.0, 9.0]])
    s, closest = path.project(np.array([3.0, 4.0]))
    assert s == pytest.approx(5.0)
    assert closest == pytest.approx([3.0, 4.0])
    assert path.sample(5.0) == pytest.approx([3.0, 4.0])
    assert path.total_length == pytest.approx(10.0)


def test_from_waypoints_default_is_polyline():
    path = from_waypoints([[0, 0], [1, 0], [1, 1]])
    assert isinstance(path, PolylinePath)
    assert path.distance(np.array([0.5, 0.5])) == pytest.approx(0.5)


def test_corridor_margin_inside_and_outside():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.5)
    assert track.corridor_margin(np.array([5.0, 1.0])) == pytest.approx(0.5)
    assert track.corridor_margin(np.array([5.0, 2.0])) == pytest.approx(-0.5)


def test_corridor_field_subtracts_robot_radius():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    field = track.corridor_field(sphere(radius=0.3))
    # center at y=1 -> margin 0; body edge at 0.7 -> field value -0.3 + ... 
    # half_width - dist - r = 1 - 1 - 0.3 = -0.3
    assert field.value(np.array([5.0, 1.0])) == pytest.approx(-0.3)


def test_path_distance_field_uses_robot_probes():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=2.0)
    field = track.distance_field(sphere(radius=0.3))
    assert field.value(np.array([5.0, 1.0])) == pytest.approx(0.7)


def test_corridor_field_as_constraint():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    inside = track.corridor_field(point()).as_constraint(lower=0.0)
    assert inside.contains(np.array([5.0, 0.5]))
    assert not inside.contains(np.array([5.0, 2.0]))


def test_path_distance_as_soft_cost():
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    cost = track.distance_field(point()).as_cost(
        weight=2.0, shaping=quadratic_excess(threshold=0.0)
    )
    on_path = np.array([5.0, 0.0])
    off_path = np.array([5.0, 2.0])
    assert cost.g(on_path, np.zeros(1)) == pytest.approx(0.0)
    assert cost.g(off_path, np.zeros(1)) == pytest.approx(2.0 * 2.0**2)


def test_obstacle_and_corridor_compose():
    scene = Scene(obstacles=(Sphere([5.0, 0.0], 0.5),))
    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    body = point()
    free = (
        scene.clearance_field(body).as_constraint()
        & track.corridor_field(body).as_constraint(lower=0.0)
    )
    assert free.contains(np.array([1.0, 0.2]))
    assert not free.contains(np.array([5.0, 0.0]))
    assert not free.contains(np.array([1.0, 2.0]))


def test_jax_path_distance_matches():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    track = ReferenceTrack(from_waypoints([[0, 0], [10, 0]]), half_width=1.0)
    field = track.corridor_field(point())
    x = np.array([4.0, 0.3])
    np_val = float(field.value(x))
    jax_val = float(field.value(jnp.array(x)))
    assert jax_val == pytest.approx(np_val)

    grad = jax.grad(lambda q: field.value(q))(jnp.array(x))
    assert np.asarray(grad).shape == (2,)


def test_plot_track_smoke():
    os.environ.setdefault("MPLBACKEND", "Agg")
    import matplotlib

    matplotlib.use("Agg", force=True)

    track = ReferenceTrack(from_waypoints([[0, 0], [5, 1], [10, 0]]), half_width=0.8)
    fig, ax = track.plot(show=False)
    assert fig is not None
    assert len(ax.lines) >= 1
