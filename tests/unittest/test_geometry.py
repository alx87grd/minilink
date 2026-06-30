import numpy as np
import pytest

from minilink.core.geometry import Box, Inflated, Sphere, Union


def test_sphere_sdf_sign_and_value():
    disc = Sphere(center=[0.0, 0.0], radius=1.0)
    assert disc.dim == 2
    assert disc.sdf(np.array([0.0, 0.0])) == pytest.approx(-1.0)  # center, inside
    assert disc.sdf(np.array([1.0, 0.0])) == pytest.approx(0.0)  # boundary
    assert disc.sdf(np.array([3.0, 0.0])) == pytest.approx(2.0)  # outside


def test_box_sdf_inside_face_and_corner():
    box = Box(lower=[-1.0, -1.0], upper=[1.0, 1.0])
    assert box.sdf(np.array([0.0, 0.0])) == pytest.approx(-1.0)  # center
    assert box.sdf(np.array([1.0, 0.0])) == pytest.approx(0.0)  # face
    assert box.sdf(np.array([4.0, 0.0])) == pytest.approx(3.0)  # outside along x
    assert box.sdf(np.array([4.0, 5.0])) == pytest.approx(5.0)  # corner gap (3, 4) -> 5


def test_union_is_pointwise_min():
    a = Sphere([0.0, 0.0], 1.0)
    b = Sphere([5.0, 0.0], 1.0)
    field = a | b
    p = np.array([5.0, 0.0])

    assert isinstance(field, Union)
    assert field.sdf(p) == pytest.approx(-1.0)
    assert field.sdf(p) == pytest.approx(min(float(a.sdf(p)), float(b.sdf(p))))


def test_inflate_subtracts_radius():
    disc = Sphere([0.0, 0.0], 1.0)
    grown = disc.inflate(0.5)
    p = np.array([2.0, 0.0])

    assert isinstance(grown, Inflated)
    assert grown.sdf(p) == pytest.approx(float(disc.sdf(p)) - 0.5)


def test_contains_is_numpy_boundary():
    disc = Sphere([0.0, 0.0], 1.0)
    assert disc.contains([0.0, 0.0])
    assert not disc.contains([2.0, 0.0])


def test_construction_validation():
    with pytest.raises(ValueError):
        Sphere([0.0, 0.0], -1.0)
    with pytest.raises(ValueError):
        Box(lower=[1.0, 1.0], upper=[0.0, 0.0])
    with pytest.raises(ValueError):
        Union(())


def test_jax_twin_matches_numpy_and_traces():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    disc = Sphere([0.0, 0.0], 1.0)
    wall = Box([2.0, -1.0], [3.0, 1.0])
    field = disc | wall
    p = [0.5, 0.25]

    reference = float(field.sdf(np.array(p)))

    def sdf(q):
        return field.sdf(q)

    # value parity (NumPy vs JAX) and branch-free traceability (jit + grad run)
    assert float(field.sdf(jnp.array(p))) == pytest.approx(reference)
    assert float(jax.jit(sdf)(jnp.array(p))) == pytest.approx(reference)

    gradient = jax.grad(sdf)(jnp.array([0.5, 0.0]))  # interior of disc → smooth
    assert np.asarray(gradient).shape == (2,)
