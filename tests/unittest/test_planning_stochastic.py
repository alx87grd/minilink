"""Cost horizon/discount, the problem exit rule, distributions, and the stochastic problem."""

import numpy as np
import pytest

from minilink import Pendulum
from minilink.core.costs import CostFunction, QuadraticCost
from minilink.planning.distributions import Gaussian, Particles, Sampler, Uniform
from minilink.planning.policy_synthesis.dp import DynamicProgrammingPlanner
from minilink.planning.problems import PlanningProblem, StochasticPlanningProblem


def pendulum():
    plant = Pendulum()
    plant.state.lower_bound = np.array([-2 * np.pi, -10.0])
    plant.state.upper_bound = np.array([2 * np.pi, 10.0])
    return plant


def quadratic(plant):
    return QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1))


# --- R1: cost horizon and discount, problem exit rule ---


def test_cost_horizon_follows_tf_unless_declared():
    cost = quadratic(pendulum())
    assert cost.horizon_kind(tf=5.0) == "finite"
    assert cost.horizon_kind(tf=np.inf) == "infinite"
    assert cost.horizon_kind(tf=None) == "infinite"

    class Infinite(CostFunction):
        horizon = "infinite"
        discount_rate = 0.5

        def g(self, x, u, t=0.0, params=None):
            return 0.0

        def h(self, x, t=0.0, params=None):
            return 0.0

    inf = Infinite()
    assert inf.horizon_kind(tf=5.0) == "infinite"
    np.testing.assert_allclose(inf.discount_factor(0.1), np.exp(-0.05))
    assert cost.discount_factor(0.1) == 1.0


def test_problem_exit_rule_and_penalty():
    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), tf=np.inf)
    assert problem.on_exit == "infeasible"
    assert problem.exit_penalty(np.zeros(2)) is None
    assert problem.horizon_kind() == "infinite"

    scalar = PlanningProblem(plant, tf=2.0, on_exit="terminate", exit_cost=50)
    assert scalar.exit_cost == 50.0
    assert scalar.exit_penalty(np.zeros(2), 1.0) == 50.0
    assert scalar.horizon_kind() == "finite"

    shaped = PlanningProblem(
        plant, on_exit="terminate", exit_cost=lambda x, t: 10.0 * float(x[0] ** 2)
    )
    assert shaped.exit_penalty(np.array([2.0, 0.0])) == 40.0

    with pytest.raises(ValueError):
        PlanningProblem(plant, on_exit="penalize")


def test_dp_reads_the_problem_exit_cost():
    plant = pendulum()
    problem = PlanningProblem(plant, cost=quadratic(plant), exit_cost=42.0)
    planner = DynamicProgrammingPlanner(problem, x_grid=(5, 5), u_grid=(3,), dt=0.05)
    assert planner.options.out_of_bound_cost == 42.0
    explicit = DynamicProgrammingPlanner(
        problem, x_grid=(5, 5), u_grid=(3,), dt=0.05, out_of_bound_cost=7.0
    )
    assert explicit.options.out_of_bound_cost == 7.0


# --- R2: distributions and the stochastic problem ---


def test_distributions_sample_on_numpy():
    rng = np.random.default_rng(0)
    gauss = Gaussian([1.0, 2.0], 0.1)
    assert gauss.sample(rng).shape == (2,)
    assert gauss.sample(rng, n=5).shape == (5, 2)
    np.testing.assert_allclose(gauss.mean(), [1.0, 2.0])

    box = Uniform([-1.0, 0.0], [1.0, 2.0])
    samples = box.sample(3, n=100)
    assert samples.shape == (100, 2)
    assert box.support.contains(samples[0])
    np.testing.assert_allclose(box.mean(), [0.0, 1.0])

    pts = Particles([[0.0, 0.0], [1.0, 1.0]])
    assert pts.sample(rng, n=4).shape == (4, 2)

    custom = Sampler(lambda key: np.array([1.0, -1.0]), mean=[1.0, -1.0])
    assert custom.sample(rng, n=2).shape == (2, 2)


@pytest.mark.optional
@pytest.mark.jax
def test_distributions_sample_and_trace_on_jax():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    key = jax.random.PRNGKey(0)
    for dist in (
        Gaussian([0.0, 0.0], [1.0, 2.0]),
        Uniform([-1.0, -1.0], [1.0, 1.0]),
        Particles([[0.0, 1.0], [2.0, 3.0]]),
        Sampler(lambda k: jax.random.normal(k, (2,)), mean=[0.0, 0.0]),
    ):
        one = dist.sample(key)
        assert one.shape == (2,)
        many = jax.jit(lambda k: dist.sample(k, n=8))(key)
        assert many.shape == (8, 2)
        assert jnp.all(jnp.isfinite(many))


def test_stochastic_problem_is_a_planning_problem_with_a_nominal_bridge():
    plant = pendulum()
    problem = StochasticPlanningProblem(
        plant,
        cost=quadratic(plant),
        tf=np.inf,
        x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
        params_distribution={"m": Uniform([0.8], [1.2])},
        on_exit="terminate",
        exit_cost=100.0,
    )
    assert isinstance(problem, PlanningProblem)
    np.testing.assert_allclose(problem.x_start, [0.0, 0.0])
    assert problem.X0.contains(np.array([1.0, 0.5]))  # the support of the draw
    assert problem.sample_x0(0, n=4).shape == (4, 2)
    assert set(problem.sample_params(0)) == {"m"}

    nominal = problem.nominal()
    assert type(nominal) is PlanningProblem
    np.testing.assert_allclose(nominal.x_start, [0.0, 0.0])
    assert nominal.exit_cost == 100.0 and nominal.on_exit == "terminate"

    with pytest.raises(ValueError):
        StochasticPlanningProblem(plant, x0_distribution=Gaussian([0.0], 1.0))
    with pytest.raises(ValueError):
        StochasticPlanningProblem(
            plant,
            x0_distribution=Gaussian([0.0, 0.0], 1.0),
            params_distribution={"nope": Uniform([0.0], [1.0])},
        )
