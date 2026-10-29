"""Path integral control (MPPI): the planner, its compiled face, and the MPC on it."""

import numpy as np
import pytest

pytestmark = [pytest.mark.optional, pytest.mark.jax]

jax = pytest.importorskip("jax")
jnp = jax.numpy

from minilink import (  # noqa: E402
    CostFunction,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
)
from minilink.analysis import discretize  # noqa: E402
from minilink.control import ModelPredictiveController  # noqa: E402
from minilink.dynamics.catalog.equations.integrators import (  # noqa: E402
    DoubleIntegrator,
)
from minilink.planning import MPPIPlanner, MPPIRecord  # noqa: E402
from minilink.planning.trajectory_optimization.mpc.predictive import (  # noqa: E402
    DelayedPredictiveController,
    PurePredictiveController,
    WarmStartedPredictiveController,
)

DT = 0.05


class SwingUpCost(CostFunction):
    """The wrapped angle to upright, a little speed and effort."""

    def g(self, x, u, t=0.0, params=None):
        theta, dtheta = x
        e = jnp.arctan2(jnp.sin(theta - jnp.pi), jnp.cos(theta - jnp.pi))
        return e**2 + 0.1 * dtheta**2 + 0.001 * u[0] ** 2

    def h(self, x, t=0.0, params=None):
        return 0.0


def swing_up_planner(**options):
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    problem = PlanningProblem(
        plant, cost=SwingUpCost(), tf=2.0, x_start=[0.0, 0.0], U=plant.input_box
    )
    settings = dict(dt=DT, n_samples=512, temperature=0.1, noise=9.0)
    return MPPIPlanner(problem, **{**settings, **options})


def double_integrator_planner():
    plant = DoubleIntegrator()
    plant.inputs["u"].lower_bound = np.array([-5.0])
    plant.inputs["u"].upper_bound = np.array([5.0])
    cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=0.1 * np.eye(1))
    problem = PlanningProblem(
        plant, cost=cost, tf=1.0, x_start=[1.0, 0.0], U=plant.input_box
    )
    return MPPIPlanner(
        problem, dt=DT, n_samples=2048, temperature=0.01, noise=1.0, n_iterations=8
    )


def test_a_solve_is_a_function_of_its_seed():
    pv = swing_up_planner().compile()
    z_a, _ = pv.solve()
    z_b, _ = pv.solve()
    z_c, _ = pv.solve({"seed": 1})
    np.testing.assert_array_equal(z_a, z_b)
    assert not np.array_equal(z_a, z_c)


def test_the_plan_lowers_the_cost_of_the_default_one():
    planner = double_integrator_planner()
    pv = planner.compile()
    z0 = pv.initial()
    _, status_0 = pv.solve(
        {"method": {"temperature": 1e9}}, z0
    )  # an unweighted average
    z, status = pv.solve(None, z0)
    assert float(status["cost"]) < float(status_0["cost"])
    assert z.shape == (pv.n_z,) and pv.n_z == 20 * 1 + 1


def test_the_first_input_points_the_way_lqr_does():
    # Near the origin the double integrator's optimal first input pushes back
    # toward it, as the LQR law of the same cost does
    from minilink import lqr

    planner = double_integrator_planner()
    solution = planner.solve()
    K = lqr(planner.problem.sys, np.eye(2), 0.1 * np.eye(1))
    u_lqr = K.outputs["u"].compute(np.zeros(0), np.array([1.0, 0.0, 0.0, 0.0]), 0.0)
    u_mppi = solution.trajectory.u[0, 0]
    assert np.sign(u_mppi) == np.sign(u_lqr[0]) and abs(u_mppi) > 0.5


def test_the_solution_is_a_held_schedule_with_its_record():
    solution = swing_up_planner().solve()
    assert isinstance(solution.solver, MPPIRecord)
    assert solution.trajectory.t.size == 41
    source = solution.policy
    u_mid = source.h(np.zeros(0), np.zeros(0), 0.5 * DT)
    np.testing.assert_allclose(u_mid, solution.trajectory.u[:, 0])


def test_shift_holds_the_last_input_and_command_reads_the_held_one():
    pv = swing_up_planner().compile()
    U = np.arange(40.0).reshape(40, 1)
    z = np.concatenate([U.reshape(-1), [1.0]])
    shifted = pv.shift(z, 1.0 + 2 * DT)
    np.testing.assert_allclose(shifted[:3], [2.0, 3.0, 4.0])
    assert shifted[-2] == 39.0 and shifted[-1] == 1.0 + 2 * DT
    np.testing.assert_allclose(pv.command(z, None, 1.0 + 2.5 * DT), [2.0])


@pytest.mark.parametrize(
    "form, kind",
    [
        (dict(delay=False), WarmStartedPredictiveController),
        (dict(delay=True), DelayedPredictiveController),
        (dict(delay=False, warm_start=False), PurePredictiveController),
    ],
)
def test_the_mpc_runs_mppi_in_every_form(form, kind):
    planner = swing_up_planner()
    mpc = ModelPredictiveController(planner, **form)
    assert isinstance(mpc, kind)
    loop = (mpc % DT) @ discretize(planner.problem.sys, DT)
    run = loop.compute_trajectory(tf=0.5)
    assert np.all(np.isfinite(run.x)) and run.has_signal("plant:u")


def test_the_zero_delay_mppi_swings_the_pendulum_up():
    planner = swing_up_planner(n_samples=1024)
    mpc = ModelPredictiveController(planner, delay=False)
    loop = (mpc % DT) @ discretize(planner.problem.sys, DT)
    run = loop.compute_trajectory(tf=6.0)
    theta, dtheta = run.of("plant").x[:, -1]
    upright = np.arctan2(np.sin(theta - np.pi), np.cos(theta - np.pi))
    assert abs(upright) < 0.2 and abs(dtheta) < 0.5


def test_mppi_refuses_an_unboxed_input_and_a_ragged_horizon():
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    problem = PlanningProblem(plant, cost=cost, tf=2.0, x_start=[0.0, 0.0])
    with pytest.raises(ValueError, match="input box"):
        MPPIPlanner(problem)
    boxed = PlanningProblem(
        plant, cost=cost, tf=2.02, x_start=[0.0, 0.0], U=plant.input_box
    )
    plant.inputs["u"].lower_bound = np.array([-1.0])
    plant.inputs["u"].upper_bound = np.array([1.0])
    with pytest.raises(ValueError, match="whole number of periods"):
        MPPIPlanner(boxed, dt=0.05)
