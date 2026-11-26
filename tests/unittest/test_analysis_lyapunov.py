"""Region of attraction: the Lyapunov matrix, the level, and the plot."""

import unittest

import numpy as np
import pytest

from minilink import CartPole, InvertedPendulum, VanDerPol
from minilink.analysis.lyapunov import (
    dynamics_of,
    plot_region_of_attraction,
    region_of_attraction,
)
from minilink.control.design import lqr
from minilink.core.backends import array_module
from minilink.core.fields import EllipsoidSet, QuadraticField
from minilink.core.system import DynamicSystem
from minilink.simulation.simulator import Simulator


class StableLinear(DynamicSystem):
    """``dx = A x`` with a Hurwitz ``A``: the region of attraction is the plane."""

    def __init__(self, a=-1.0, b=-2.0):
        super().__init__(state_dim=2, input_dim=1, output_dim=2)
        self.params = {"a": float(a), "b": float(b)}
        self.state.lower = np.array([-10.0, -10.0])
        self.state.upper = np.array([10.0, 10.0])

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        return xp.array([params["a"] * x[0], params["b"] * x[1]])

    def h(self, x, u, t=0.0, params=None):
        return x


class UntraceableSpring(DynamicSystem):
    """``dx = -k(x) x`` with a Python branch: JAX cannot trace it, NumPy can."""

    def __init__(self):
        super().__init__(state_dim=2, input_dim=1, output_dim=2)
        self.state.lower = np.array([-5.0, -5.0])
        self.state.upper = np.array([5.0, 5.0])

    def f(self, x, u, t=0.0, params=None):
        stiffness = 2.0 if float(x[0]) > 0.0 else 1.0  # float() breaks tracing
        return np.array([-stiffness * x[0], -x[1]])

    def h(self, x, u, t=0.0, params=None):
        return x


class ReverseVanDerPol(VanDerPol):
    """Van der Pol run backwards: the origin is stable inside the limit cycle."""

    def f(self, x, u, t=0.0, params=None):
        return -super().f(x, u, t, params)


def saturated_pendulum(torque=2.0):
    """Inverted pendulum whose actuator cannot hold it far from upright."""
    plant = InvertedPendulum()
    plant.inputs["u"].lower = np.array([-torque])
    plant.inputs["u"].upper = np.array([torque])
    return plant


def saturated_loop():
    plant = saturated_pendulum()
    return lqr(plant, Q=np.eye(2), R=np.eye(1), x_bar=np.zeros(2)) @ plant


def converges(sys, region, X0, horizon):
    """Whether each state of ``X0`` ends near the region's center after ``horizon`` seconds."""
    x_bar = region.params["center"]
    finals = [
        Simulator(sys, x0=x0, tf=horizon, verbose=False).solve().x[:, -1] for x0 in X0
    ]
    distance = np.linalg.norm(np.array(finals) - x_bar, axis=1)
    return distance < 0.05 * np.max(region.half_width)


def V_dot_on(sys, region, X):
    """``V̇ = 2 (x − x̄)ᵀ P f(x)`` on a stack of states."""
    x_bar = region.params["center"]
    P = region.params["P"]
    f = dynamics_of(sys, sys.u_nominal, 0.0, None)
    return 2.0 * np.einsum("ij,jk,ik->i", X - x_bar, P, f(X))


class TestRegionOfAttraction(unittest.TestCase):
    def test_the_region_is_the_sublevel_set_of_the_lyapunov_function(self):
        sys = StableLinear()
        region = region_of_attraction(sys)

        self.assertIsInstance(region, EllipsoidSet)
        np.testing.assert_allclose(region.params["center"], [0.0, 0.0], atol=1e-9)
        # AᵀP + PA = −Q with A = diag(a, b) gives P = diag(−1/2a, −1/2b)
        np.testing.assert_allclose(region.params["P"], np.diag([0.5, 0.25]), atol=1e-9)
        self.assertIsInstance(region.field, QuadraticField)
        # Nothing in the plane stops V; the state box does, at |x| = 10
        self.assertLessEqual(region.half_width.max(), 10.0 + 1e-6)
        self.assertTrue(region.contains(np.zeros(2)))

    def test_the_saturated_pendulum_region_lies_inside_the_true_basin(self):
        loop = saturated_loop()
        region = region_of_attraction(loop)

        self.assertGreater(region.level, 0.0)
        # Simulation agrees with the theory: every state drawn inside converges
        X0 = region.sample(1, 40)
        self.assertTrue(np.all(converges(loop, region, X0, horizon=20.0)))

    def test_no_dense_sample_inside_the_region_has_a_growing_v(self):
        # The level is an estimate from samples: a far denser draw inside the
        # region finds V̇ ≥ 0 only at its very edge, if at all
        loop = saturated_loop()
        region = region_of_attraction(loop)
        X = region.sample(7, 100_000)
        V = region.field(X)

        growing = V_dot_on(loop, region, X) >= 0.0
        self.assertTrue(not growing.any() or V[growing].min() > 0.99 * region.level)

    def test_reverse_van_der_pol_stays_inside_its_limit_cycle(self):
        # The region is bounded by the unstable limit cycle, about |x| = 2 for mu = 1
        sys = ReverseVanDerPol()
        sys.state.lower = np.array([-4.0, -4.0])
        sys.state.upper = np.array([4.0, 4.0])
        region = region_of_attraction(sys)

        self.assertLess(region.half_width.max(), 2.5)
        # A state well outside the cycle diverges, so it must not be in the region
        self.assertFalse(region.contains(np.array([3.0, 3.0])))

    def test_an_unstable_equilibrium_has_no_region(self):
        plant = saturated_pendulum()
        with self.assertRaises(ValueError) as raised:
            region_of_attraction(plant, np.zeros(2))  # open loop, upright
        self.assertIn("not stable", str(raised.exception))

    def test_the_equilibrium_is_found_not_assumed(self):
        # A law with a steady offset settles beside the state it was aimed at
        plant = saturated_pendulum()
        ctl = lqr(plant, Q=np.eye(2), R=np.eye(1), x_bar=np.zeros(2))
        ctl.params["u_bar"] = np.array([0.5])  # a bias the design did not intend
        loop = ctl @ plant
        region = region_of_attraction(loop, np.zeros(2))

        x_bar = region.params["center"]
        self.assertGreater(abs(float(x_bar[0])), 1e-3)
        np.testing.assert_allclose(loop.f(x_bar, loop.u_nominal), 0.0, atol=1e-6)

    def test_a_plant_that_cannot_trace_is_analyzed_on_numpy(self):
        from minilink.core.compile.compiler import compile_auto

        sys = UntraceableSpring()
        backend, _ = compile_auto(sys)
        self.assertEqual(backend, "numpy")  # the premise of this test

        region = region_of_attraction(sys, n_samples=2000)  # looped evaluation
        self.assertGreater(region.level, 0.0)

    def test_a_four_state_region_holds_on_a_dense_draw(self):
        # In four states the samples thin out: the shrinking search still lands
        # at the edge where V stops decreasing
        cart = CartPole()
        cart.inputs["u"].lower = np.array([-20.0])
        cart.inputs["u"].upper = np.array([20.0])
        cart.state.lower = np.array([-3.0, -4 * np.pi, -10.0, -10.0])
        cart.state.upper = -cart.state.lower
        x_up = np.array([0.0, np.pi, 0.0, 0.0])
        Q = np.diag([1.0, 10.0, 1.0, 1.0])
        loop = lqr(cart, Q=Q, R=np.eye(1), x_bar=x_up) @ cart
        region = region_of_attraction(loop, x_up)

        self.assertEqual(region.half_width.size, 4)
        X = region.sample(7, 200_000)
        V = region.field(X)
        growing = V_dot_on(loop, region, X) >= 0.0
        self.assertTrue(not growing.any() or V[growing].min() > 0.95 * region.level)

    def test_the_level_repeats(self):
        loop = saturated_loop()
        self.assertEqual(
            region_of_attraction(loop).level, region_of_attraction(loop).level
        )

    def test_the_simulated_region_reaches_past_the_lyapunov_one(self):
        # Same ellipsoid family, sized by convergence instead of V̇ < 0: at least
        # as large, and every start drawn inside converges
        loop = saturated_loop()
        lyapunov_region = region_of_attraction(loop)
        simulated = region_of_attraction(loop, method="simulation")

        self.assertIsInstance(simulated, EllipsoidSet)
        np.testing.assert_allclose(
            simulated.params["P"], lyapunov_region.params["P"], atol=1e-12
        )
        self.assertGreaterEqual(simulated.level, lyapunov_region.level)
        X0 = simulated.sample(3, 30)
        self.assertTrue(np.all(converges(loop, simulated, X0, horizon=20.0)))

    def test_an_unknown_method_is_refused(self):
        with self.assertRaisesRegex(ValueError, "lyapunov"):
            region_of_attraction(StableLinear(), method="sos")


@pytest.mark.optional
@pytest.mark.jax
def test_numpy_and_jax_backends_agree():
    pytest.importorskip("jax")
    from minilink.analysis import lyapunov

    loop = saturated_loop()
    jax_level = region_of_attraction(loop).level

    # The same search with f evaluated state by state on NumPy
    original = lyapunov.dynamics_of
    numpy_evaluator = loop.compile(backend="numpy", verbose=False)
    lyapunov.dynamics_of = lambda sys, u_bar, t, params: (
        lambda X: np.stack([numpy_evaluator.f(x, u_bar, t, params) for x in X])
    )
    try:
        numpy_level = region_of_attraction(loop, n_samples=50_000).level
    finally:
        lyapunov.dynamics_of = original
    np.testing.assert_allclose(jax_level, numpy_level, rtol=1e-6)


@pytest.mark.plotting
def test_the_plot_draws_the_region_and_the_basin():
    import matplotlib

    matplotlib.use("Agg")
    from unittest import mock

    from minilink.analysis import lyapunov

    loop = saturated_loop()
    region = region_of_attraction(loop)

    # A coarse plane: the basin is one simulation per point, state by state on NumPy
    with mock.patch.object(lyapunov, "PLOT_POINTS", 15):
        result = plot_region_of_attraction(loop, region, basin=True, show=False)
    ax = result.axes
    assert "region of attraction" in " ".join(ax.get_title().split())
    assert len(ax.collections) >= 2  # the shaded basin and the level set
    assert ax.get_xlabel() == loop.state.labels[0]


@pytest.mark.plotting
def test_the_plot_draws_both_estimates_side_by_side():
    import matplotlib

    matplotlib.use("Agg")
    loop = saturated_loop()
    regions = {
        "Lyapunov": region_of_attraction(loop),
        "simulation": region_of_attraction(loop, method="simulation"),
    }

    ax = plot_region_of_attraction(loop, regions, show=False).axes
    legend = [text.get_text() for text in ax.get_legend().get_texts()]
    assert "Lyapunov" in legend and "simulation" in legend


@pytest.mark.plotting
def test_plot_overlays_leave_the_system_run_and_start_untouched():
    import matplotlib

    matplotlib.use("Agg")
    sys = StableLinear()
    sys.x0 = np.array([1.0, -1.0])
    region = region_of_attraction(sys)
    traj = sys.compute_trajectory(tf=0.5, verbose=False)

    ax = plot_region_of_attraction(
        sys, region, trajectories=[[2.0, 1.0], [-3.0, 0.5]], show=False
    ).axes
    assert len(ax.lines) >= 4  # each overlay: its path and its start
    assert sys.traj is traj
    np.testing.assert_array_equal(sys.x0, [1.0, -1.0])


@pytest.mark.plotting
def test_the_plot_draws_with_plotly():
    pytest.importorskip("plotly")
    sys = StableLinear()
    region = region_of_attraction(sys)

    drawn = plot_region_of_attraction(
        sys,
        region,
        basin=True,
        trajectories=[[2.0, 1.0]],
        renderer="plotly",
        show=False,
    )
    kinds = [type(trace).__name__ for trace in drawn.figure.data]
    assert kinds[0] == "Heatmap"  # the basin, a filled band
    assert "Contour" in kinds and kinds.count("Scatter") == 4
    names = {trace.name for trace in drawn.figure.data}
    assert {"converges in simulation", "region", "trajectory", "equilibrium"} <= names
