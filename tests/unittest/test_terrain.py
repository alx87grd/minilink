"""Contract of the terrain: closed-form gradients, both backends, the static block."""

import numpy as np
import pytest

from minilink.core.backends import configure_jax
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem
from minilink.dynamics.catalog.vehicles.terrain import (
    Bumps,
    Slope,
    Terrain,
    TerrainBlock,
)

TERRAINS = [Terrain, Slope, Bumps]

RNG = np.random.default_rng(11)
X = 4.0 * RNG.standard_normal(50)
Y = 4.0 * RNG.standard_normal(50)


@pytest.mark.parametrize("make", TERRAINS, ids=lambda m: m.__name__)
def test_the_gradient_is_the_derivative_of_the_height(make):
    """The closed-form gradient against a central difference of the height."""
    terrain = make()
    h = 1.0e-6

    dz_dx, dz_dy = terrain.gradient(X, Y)
    fd_x = (terrain.height(X + h, Y) - terrain.height(X - h, Y)) / (2.0 * h)
    fd_y = (terrain.height(X, Y + h) - terrain.height(X, Y - h)) / (2.0 * h)

    np.testing.assert_allclose(dz_dx, fd_x, atol=1e-8)
    np.testing.assert_allclose(dz_dy, fd_y, atol=1e-8)


def test_the_flat_ground_is_level_and_grips_fully():
    terrain = Terrain()

    assert np.all(terrain.height(X, Y) == 0.0)
    assert np.all(terrain.friction(X, Y) == 1.0)
    assert np.all(np.concatenate(terrain.gradient(X, Y)) == 0.0)


def test_the_fields_read_the_params_they_are_given():
    """A terrain's numbers live in ``params``; a call with its own tree reads that tree."""
    bumps = Bumps()
    tall = {**bumps.params, "terrain_amplitude": 0.2}
    slope = Slope()
    icy = {**slope.params, "terrain_mu": 0.1}

    np.testing.assert_allclose(bumps.height(X, Y, 0.0, tall), 4.0 * bumps.height(X, Y))
    np.testing.assert_allclose(slope.friction(X, Y, 0.0, icy), 0.1)
    assert slope.height(1.0, 0.0) == pytest.approx(
        np.tan(slope.params["terrain_slope"])
    )


@pytest.mark.jax
@pytest.mark.parametrize("make", TERRAINS, ids=lambda m: m.__name__)
def test_numpy_and_jax_agree(make):
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    jnp = jax.numpy
    terrain = make()

    for field in ("height", "gradient", "friction"):
        on_numpy = np.asarray(getattr(terrain, field)(X, Y))
        on_jax = np.asarray(
            jax.jit(getattr(terrain, field))(jnp.asarray(X), jnp.asarray(Y))
        )
        np.testing.assert_allclose(on_jax, on_numpy, rtol=1e-12, atol=1e-15)


class Points(DynamicSystem):
    """Fixed points in the plane: ``[x_1 … x_n, y_1 … y_n]``, held."""

    def __init__(self, n_points):
        super().__init__(state_dim=2 * n_points, input_dim=1, output_dim=2 * n_points)

    def f(self, x, u, t=0.0, params=None):
        return 0.0 * x

    def h(self, x, u, t=0.0, params=None):
        return x


def test_the_block_in_a_diagram_returns_the_fields_at_its_input_points():
    bumps = Bumps()
    points = Points(4)
    points.x0 = np.array([0.1, 0.7, 1.3, -0.4, 0.2, -0.5, 0.9, 1.6])
    p_x = points.x0[:4]
    p_y = points.x0[4:]
    diagram = DiagramSystem()
    diagram.add_subsystem(points, "wheels")
    diagram.add_subsystem(TerrainBlock(bumps, n_points=4), "terrain")
    diagram.connect("wheels", "y", "terrain", "u")

    run = diagram.compute_trajectory(
        tf=0.1, record=["terrain:height", "terrain:gradient", "terrain:friction"]
    )

    np.testing.assert_allclose(
        run.signals["terrain:height"][:, -1], bumps.height(p_x, p_y)
    )
    np.testing.assert_allclose(
        run.signals["terrain:gradient"][:, -1],
        np.concatenate(bumps.gradient(p_x, p_y)),
    )
    np.testing.assert_allclose(
        run.signals["terrain:friction"][:, -1], bumps.friction(p_x, p_y)
    )
