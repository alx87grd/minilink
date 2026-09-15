"""Lyapunov certificates: the level, its verification, and the plot."""

import unittest

import numpy as np
import pytest

from minilink import CartPole, InvertedPendulum, VanderPol
from minilink.analysis.lyapunov import (
    LyapunovCertificate,
    region_of_attraction,
    sample_in_ellipsoid,
)
from minilink.control.lqr import lqr_at_operating_point
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem


class StableLinear(DynamicSystem):
    """``dx = A x`` with a Hurwitz ``A``: the region of attraction is the plane."""

    def __init__(self, a=-1.0, b=-2.0):
        super().__init__(n=2, input_dim=1, output_dim=2)
        self.params = {"a": float(a), "b": float(b)}
        self.state.lower_bound = np.array([-10.0, -10.0])
        self.state.upper_bound = np.array([10.0, 10.0])

    def f(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x)
        return xp.array([params["a"] * x[0], params["b"] * x[1]])

    def h(self, x, u, t=0.0, params=None):
        return x


class UntraceableSpring(DynamicSystem):
    """``dx = -k(x) x`` with a Python branch: JAX cannot trace it, NumPy can."""

    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2)
        self.state.lower_bound = np.array([-5.0, -5.0])
        self.state.upper_bound = np.array([5.0, 5.0])

    def f(self, x, u, t=0.0, params=None):
        stiffness = 2.0 if float(x[0]) > 0.0 else 1.0  # float() breaks tracing
        return np.array([-stiffness * x[0], -x[1]])

    def h(self, x, u, t=0.0, params=None):
        return x


class ReverseVanderPol(VanderPol):
    """Van der Pol run backwards: the origin is stable inside the limit cycle."""

    def f(self, x, u, t=0.0, params=None):
        return -super().f(x, u, t, params)


def saturated_pendulum(torque=2.0):
    """Inverted pendulum whose actuator cannot hold it far from upright."""
    plant = InvertedPendulum()
    plant.inputs["u"].lower_bound = np.array([-torque])
    plant.inputs["u"].upper_bound = np.array([torque])
    return plant


class TestQuadraticCertificate(unittest.TestCase):
    def test_linear_loop_is_limited_only_by_its_domain(self):
        sys = StableLinear()
        roa = region_of_attraction(sys)

        self.assertIsInstance(roa, LyapunovCertificate)
        np.testing.assert_allclose(roa.x_bar, [0.0, 0.0], atol=1e-9)
        # A' P + P A = -Q with A = diag(a, b) gives P = diag(-1/2a, -1/2b)
        np.testing.assert_allclose(roa.P, np.diag([0.5, 0.25]), atol=1e-9)
        self.assertTrue(np.all(roa.poles.real < 0))
        # Nothing in the plane stops V; the state box does, at |x| = 10
        self.assertLessEqual(roa.extent.max(), 10.0 + 1e-6)
        self.assertTrue(roa.contains(np.zeros(2)))
        self.assertTrue(roa.verify(n=50).holds)

    def test_saturated_pendulum_certificate_sits_inside_the_true_basin(self):
        plant = saturated_pendulum()
        loop = (
            lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1)) @ plant
        )
        roa = region_of_attraction(loop)

        self.assertGreater(roa.level, 0.0)
        self.assertTrue(roa.contains(roa.x_bar))
        # Simulation agrees with the theory, which is the whole point
        report = roa.verify(n=100)
        self.assertTrue(report.holds, msg=str(report))
        self.assertIsNone(report.counterexample)
        # The limiting sample is on the boundary of the claim, not inside it
        self.assertAlmostEqual(roa.V(roa.limiting_state), roa.level, places=9)

    def test_reverse_van_der_pol_stays_inside_its_limit_cycle(self):
        # The textbook case: the region of attraction is bounded by the
        # unstable limit cycle, which for mu = 1 passes about |x| = 2.
        sys = ReverseVanderPol()
        sys.state.lower_bound = np.array([-4.0, -4.0])
        sys.state.upper_bound = np.array([4.0, 4.0])
        roa = region_of_attraction(sys)

        self.assertLess(roa.extent.max(), 2.5)
        self.assertTrue(roa.verify(n=100).holds)
        # A state well outside the cycle diverges, so it must not be certified
        self.assertFalse(roa.contains(np.array([3.0, 3.0])))

    def test_unstable_equilibrium_has_nothing_to_certify(self):
        plant = saturated_pendulum()
        with self.assertRaises(ValueError) as raised:
            region_of_attraction(plant, np.zeros(2))  # open loop, upright
        self.assertIn("not stable", str(raised.exception))

    def test_equilibrium_is_found_not_assumed(self):
        # A law with a steady offset settles beside the state it was aimed at.
        plant = saturated_pendulum()
        ctl = lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1))
        ctl.params["ubar"] = np.array([0.5])  # a bias the design did not intend
        roa = region_of_attraction(ctl @ plant, np.zeros(2))

        self.assertGreater(abs(float(roa.x_bar[0])), 1e-3)
        np.testing.assert_allclose(roa.V_dot(roa.x_bar), 0.0, atol=1e-6)

    def test_every_certified_state_of_a_grid_converges(self):
        # The property the plot draws: the certified set is inside the basin.
        plant = saturated_pendulum()
        loop = (
            lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1)) @ plant
        )
        roa = region_of_attraction(loop)

        half = 2.5 * roa.extent
        axes = [np.linspace(-h, h, 61) + roa.x_bar[i] for i, h in enumerate(half)]
        grid = np.stack([m.ravel() for m in np.meshgrid(*axes)], axis=1)
        certified = roa.contains(grid)
        self.assertGreater(certified.sum(), 100)

        final = roa.rollout(10.0 / abs(roa.rate))(grid[certified])
        distance = np.linalg.norm(final - roa.x_bar, axis=1)
        self.assertLess(distance.max(), 0.05 * roa.extent.max())

    def test_a_plant_that_cannot_trace_is_certified_on_numpy(self):
        from minilink.core.compile.compiler import compile_auto

        sys = UntraceableSpring()
        backend, _ = compile_auto(sys)
        self.assertEqual(backend, "numpy")  # the premise of this test

        roa = region_of_attraction(sys)  # finite-difference Jacobian, looped sweep
        self.assertTrue(np.all(roa.poles.real < 0))
        self.assertGreater(roa.level, 0.0)
        self.assertTrue(roa.verify(n=20).holds)

    def test_four_state_loop_certifies_and_its_slice_is_not_its_shadow(self):
        # n > 2: the sweep goes quasi-random, and a phase-plane plot draws the
        # slice through the equilibrium, which coupling makes smaller than the
        # shadow the certified set casts on the two axes.
        arm = CartPole()
        arm.inputs["u"].lower_bound = np.array([-20.0])
        arm.inputs["u"].upper_bound = np.array([20.0])
        arm.state.lower_bound = np.array([-3.0, -4 * np.pi, -10.0, -10.0])
        arm.state.upper_bound = -arm.state.lower_bound
        x_up = np.array([0.0, np.pi, 0.0, 0.0])
        loop = (
            lqr_at_operating_point(
                arm, x_up, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.eye(1)
            )
            @ arm
        )
        roa = region_of_attraction(loop, x_up)

        self.assertEqual(roa.extent.size, 4)
        self.assertTrue(roa.verify(n=50).holds)

        shadow = roa.extent[[0, 1]]
        sliced = roa.slice_extent(0, 1)
        np.testing.assert_array_less(sliced, shadow + 1e-12)
        self.assertLess(
            sliced[1], 0.7 * shadow[1]
        )  # the pole angle is strongly coupled

    def test_the_search_reports_whether_it_had_enough_samples(self):
        # Two states on a grid: the halves agree, so the level is sharp.
        plant = saturated_pendulum()
        loop = (
            lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1)) @ plant
        )
        roa = region_of_attraction(loop)
        self.assertLess(roa.sample_spread, 0.1)
        self.assertFalse(roa.sample_limited)
        self.assertNotIn("sample-limited", str(roa))

        # The flag is what a caller reads, and it shows up in the summary line
        thin = region_of_attraction(loop)
        thin.sample_spread = 0.4
        self.assertTrue(thin.sample_limited)
        self.assertIn("sample-limited", str(thin))

    def test_method_and_search_options(self):
        sys = StableLinear()
        with self.assertRaises(NotImplementedError):
            region_of_attraction(sys, method="sos")
        with self.assertRaises(ValueError):
            region_of_attraction(sys, method="neural")
        with self.assertRaises(ValueError):
            region_of_attraction(sys, search="sobol")

        grid = region_of_attraction(sys, search="grid", samples=2601)
        drawn = region_of_attraction(sys, search="random", samples=4000)
        self.assertAlmostEqual(grid.level, drawn.level, delta=0.4 * grid.level)

    def test_sample_in_ellipsoid_respects_the_level(self):
        P = np.array([[4.0, 1.0], [1.0, 2.0]])
        d = sample_in_ellipsoid(P, level=3.0, n=500)
        self.assertTrue(np.all(np.einsum("ij,jk,ik->i", d, P, d) <= 3.0 + 1e-9))
        np.testing.assert_allclose(
            sample_in_ellipsoid(P, 3.0, 5), sample_in_ellipsoid(P, 3.0, 5)
        )


@pytest.mark.optional
@pytest.mark.jax
def test_numpy_and_jax_backends_agree():
    pytest.importorskip("jax")
    plant = saturated_pendulum()
    loop = lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1)) @ plant

    from minilink.analysis import lyapunov

    jax_level = region_of_attraction(loop).level
    original = lyapunov.compiled
    lyapunov.compiled = lambda sys: (
        "numpy",
        sys.compile(backend="numpy", verbose=False),
    )
    try:
        numpy_level = region_of_attraction(loop).level
    finally:
        lyapunov.compiled = original
    np.testing.assert_allclose(jax_level, numpy_level, rtol=1e-6)


@pytest.mark.plotting
def test_plot_draws_the_slice_and_the_basin():
    import matplotlib

    matplotlib.use("Agg")
    plant = saturated_pendulum()
    loop = lqr_at_operating_point(plant, np.zeros(2), Q=np.eye(2), R=np.eye(1)) @ plant
    roa = region_of_attraction(loop)

    fig, ax = roa.plot(basin=True, n=41, show=False)
    self_titled = ax.get_title()
    assert "region of attraction" in self_titled
    assert len(ax.collections) >= 1  # the shaded basin
    assert ax.get_xlabel() == loop.state.labels[0]

    fig_shortcut, _ = loop.plot_region_of_attraction(n=21, show=False)
    assert fig_shortcut is not None
