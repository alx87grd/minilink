"""The grid field: a table on a regular grid read as a field, and the planners' cost-to-go as fields."""

import numpy as np
import pytest
from scipy.interpolate import RegularGridInterpolator

from minilink import DynamicProgrammingPlanner, LQRPlanner
from minilink.core import BoxSet, MarginField
from minilink.core.backends import require_jax, require_jax_numpy
from minilink.core.fields import GridField, QuadraticField
from tests.unittest.test_planner_params import GRID, reach, regulation

LEVELS = [
    np.linspace(-1.0, 2.0, 7),
    np.array([0.0, 0.5, 1.5, 3.0]),
    np.linspace(-2.0, 2.0, 5),
]
TABLE = np.random.default_rng(0).normal(size=(7, 4, 5))


def inside_points(n=60, seed=1):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(level[0], level[-1], n) for level in LEVELS])


def test_linear_and_nearest_read_as_scipy_inside_the_grid():
    X = inside_points()
    for method in ("linear", "nearest"):
        V = GridField(LEVELS, TABLE, method=method)
        reference = RegularGridInterpolator(tuple(LEVELS), TABLE, method=method)(X)

        np.testing.assert_allclose(V.value(X), reference, rtol=1e-12, atol=1e-12)
        assert np.ndim(V(X[0])) == 0
        np.testing.assert_allclose(V(X[0]), reference[0], rtol=1e-12, atol=1e-12)


def test_a_spline_reads_as_scipy():
    X = inside_points()
    V = GridField(LEVELS, TABLE, method="cubic")
    reference = RegularGridInterpolator(tuple(LEVELS), TABLE, method="cubic")(X)

    np.testing.assert_allclose(V.value(X), reference, rtol=1e-12)
    assert np.ndim(V(X[0])) == 0
    assert V(X[0]) == pytest.approx(reference[0], rel=1e-12)


def test_the_nodes_read_their_own_values():
    V = GridField(LEVELS, TABLE)
    i, j, k = 3, 2, 4
    x = np.array([LEVELS[0][i], LEVELS[1][j], LEVELS[2][k]])

    assert V(x) == pytest.approx(TABLE[i, j, k], abs=1e-12)


def test_outside_the_grid_the_edge_holds_or_the_number_given():
    x_out = np.array([5.0, -1.0, 0.3])
    x_edge = np.array([2.0, 0.0, 0.3])

    assert GridField(LEVELS, TABLE)(x_out) == GridField(LEVELS, TABLE)(x_edge)
    assert GridField(LEVELS, TABLE, outside=99.0)(x_out) == 99.0
    assert GridField(LEVELS, TABLE, outside=99.0)(x_edge) != 99.0


def test_the_gradient_is_the_cells_multilinear_one():
    V = GridField(LEVELS, TABLE)
    x = np.array([0.23, 0.71, -0.37])
    eps = 1e-7
    numeric = np.array(
        [(V(x + eps * e) - V(x - eps * e)) / (2 * eps) for e in np.eye(3)]
    )

    np.testing.assert_allclose(V.gradient(x), numeric, rtol=1e-6, atol=1e-8)
    assert V.gradient(inside_points(5)).shape == (5, 3)


def test_the_gradient_is_zero_where_the_field_is_held_or_given():
    x_out = np.array([5.0, 0.71, -0.37])

    assert GridField(LEVELS, TABLE).gradient(x_out)[0] == 0.0
    assert GridField(LEVELS, TABLE).gradient(x_out)[1] != 0.0
    np.testing.assert_array_equal(
        GridField(LEVELS, TABLE, outside=1.0).gradient(x_out), np.zeros(3)
    )


def test_a_time_axis_reads_linearly_and_holds_past_its_ends():
    t = np.array([0.0, 1.0, 3.0])
    values = np.stack([TABLE, 2.0 * TABLE, -TABLE])
    V = GridField(LEVELS, values, t=t)
    x = inside_points(1)[0]
    V0 = GridField(LEVELS, TABLE)(x)

    assert V(x, t=0.0) == pytest.approx(V0)
    assert V(x, t=0.5) == pytest.approx(1.5 * V0)
    assert V(x, t=2.0) == pytest.approx(0.5 * V0)
    assert V(x, t=10.0) == pytest.approx(-V0)
    assert V(x, t=-1.0) == pytest.approx(V0)


def test_params_override_the_table():
    V = GridField(LEVELS, TABLE)
    x = inside_points(1)[0]
    doubled = {**V.params, "values": 2.0 * TABLE}

    assert V.value(x, params=doubled) == pytest.approx(2.0 * V(x))


def test_bad_shapes_and_methods_are_refused():
    with pytest.raises(ValueError, match="shape"):
        GridField(LEVELS, TABLE[:, :, :3])
    with pytest.raises(ValueError, match="increasing"):
        GridField([np.array([0.0, 0.0, 1.0])], np.zeros(3))
    with pytest.raises(ValueError, match="method"):
        GridField(LEVELS, TABLE, method="spline")


@pytest.mark.jax
def test_the_field_traces_and_its_gradient_is_jaxs():
    pytest.importorskip("jax")
    jax = require_jax()
    jnp = require_jax_numpy()
    V = GridField(LEVELS, TABLE)
    X = inside_points(20)

    traced = jax.jit(lambda X, p: V.value(X, params=p))(jnp.asarray(X), V.params)
    np.testing.assert_allclose(np.asarray(traced), V.value(X), rtol=1e-12)

    # Its own table, a constant of the trace
    np.testing.assert_allclose(
        np.asarray(jax.jit(V.value)(jnp.asarray(X))), V.value(X), rtol=1e-12
    )

    x = jnp.asarray(X[0])
    np.testing.assert_allclose(
        np.asarray(jax.grad(V.value)(x)), V.gradient(X[0]), rtol=1e-10
    )
    np.testing.assert_allclose(np.asarray(V.gradient(x)), V.gradient(X[0]), rtol=1e-12)


# The planners' cost-to-go


def test_value_iterations_cost_to_go_is_a_grid_field_charging_the_failure_cost_outside():
    planner = DynamicProgrammingPlanner(regulation(), max_iterations=30, **GRID)
    solution = planner.solve()
    J = solution.cost_to_go
    result = solution.record.tables
    x_in = np.array([0.4, -1.3])

    assert isinstance(J, GridField)
    assert J(x_in) == pytest.approx(result.value_at(x_in), rel=1e-12)
    assert planner.cost_to_go(x_in) == float(J(x_in))
    assert J(np.array([10.0, 0.0])) == planner.out_of_bound_cost
    assert not J.as_set(upper=0.5 * planner.out_of_bound_cost).contains(
        np.array([10.0, 0.0])
    )
    assert J.as_terminal_cost().h(x_in) == pytest.approx(J(x_in))


def test_lqrs_cost_to_go_is_a_quadratic_field_in_both_horizons():
    infinite = LQRPlanner(reach().replace(Xf=None, horizon=None)).solve()
    finite = LQRPlanner(reach().replace(Xf=None)).solve()
    x = np.array([2.5, 0.4])
    t, _, S = finite.record.schedule

    assert isinstance(infinite.cost_to_go, QuadraticField)
    d = x - infinite.cost_to_go.center
    assert infinite.cost_to_go(x) == pytest.approx(d @ infinite.record.P @ d, rel=1e-12)

    assert isinstance(finite.cost_to_go, QuadraticField)
    np.testing.assert_array_equal(finite.cost_to_go.t, t)
    assert finite.cost_to_go(x, t=t[40]) == pytest.approx(d @ S[40] @ d, rel=1e-12)


@pytest.mark.jax
def test_value_iterations_law_closes_a_jax_loop_as_on_numpy():
    pytest.importorskip("jax")
    problem = regulation()
    law = (
        DynamicProgrammingPlanner(problem, max_iterations=30, **GRID).solve().controller
    )
    plant = problem.sys
    plant.x0 = np.array([0.5, 0.0])
    loop = law @ plant

    on_numpy = loop.compute_trajectory(tf=2.0, solver="rk4_fixedsteps")
    on_jax = loop.compute_trajectory(tf=2.0, solver="rk4_fixedsteps", backend="jax")

    np.testing.assert_allclose(np.asarray(on_jax.x), on_numpy.x, rtol=1e-9, atol=1e-12)


def test_a_sets_margin_field_is_its_least_margin():
    box = BoxSet(np.array([-1.0, -2.0]), np.array([1.0, 2.0]))
    V = MarginField(box)

    assert V(np.array([0.5, 0.0])) == pytest.approx(0.5)
    assert V(np.array([0.0, 2.5])) == pytest.approx(-0.5)
    assert V.params is box.params
