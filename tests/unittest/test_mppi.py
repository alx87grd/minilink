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


# The fan of sampled futures, rebuilt from a run; the verbose line


def zero_delay_run(tf=0.5, verbose=False):
    planner = swing_up_planner()
    mpc = ModelPredictiveController(planner, delay=False, verbose=verbose)
    loop = (mpc % DT) @ discretize(planner.problem.sys, DT)
    run = loop.compute_trajectory(tf=tf)
    return mpc, loop, run


def test_the_rebuilt_fans_are_the_solves_of_the_run():
    from minilink.core.compile.plan import nested_layout
    from minilink.planning.trajectory_optimization.mpc import mpc_sampled_futures

    mpc, loop, run = zero_delay_run()
    fans = mpc_sampled_futures(run, mpc, loop=loop, n_shown=4)
    memory, _ = nested_layout(loop)[("controller", "memory")]
    assert len(fans) == 10 and fans[0]["futures"].shape == (4, 41, 2)
    for k, fan in enumerate(fans[:-1]):
        # The memory holds z*_k at tick k + 1: the rebuilt solve's plan is it, bit for bit
        np.testing.assert_array_equal(
            fan["inputs"].reshape(-1), run.x[memory, k + 1][:-1]
        )
    assert np.all(np.diff(fans[0]["weights"]) <= 0)  # the heaviest first


def test_reading_a_solves_inputs_back_runs_no_solve():
    from unittest import mock

    from minilink.planning.trajectory_optimization.path_integral import (
        CompiledMPPIPlanner,
    )

    mpc, loop, run = zero_delay_run()
    ev = loop.compile()
    with mock.patch.object(CompiledMPPIPlanner, "solve", autospec=True) as solve:
        u = ev.block_input(("controller", "solve"), run.x[:, 3], run.u[:, 3], 3)
    assert solve.call_count == 0
    np.testing.assert_array_equal(u[:2], run.of("plant").x[:, 3])


def test_the_overlays_and_the_figure_draw_mppi():
    import matplotlib

    matplotlib.use("Agg")
    from minilink.planning.trajectory_optimization.mpc import (
        mpc_animation_overlays,
        mpc_plans_from_rollout,
        plot_sampled_futures,
    )
    from minilink.planning.trajectory_optimization.mpc.viz import SampledFutures

    mpc, loop, run = zero_delay_run()
    plans = mpc_plans_from_rollout(run, mpc, loop=loop)
    assert len(plans) == 10 and plans[0][1].x.shape == (2, 41)
    overlays = mpc_animation_overlays(run, mpc, loop=loop, samples=8)
    fan = next(o for o in overlays if isinstance(o, SampledFutures))
    assert len(next(iter(fan.get_dynamic_geometry(0.2).values()))) == 8 + 1
    fig, ax = plot_sampled_futures(
        run, mpc, loop=loop, ticks=[0, 5], axes=("t", 0), show=False
    )
    assert len(ax.lines) == 2 * (32 + 1) + 1


def test_verbose_prints_mppis_account():
    import contextlib
    import io

    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        zero_delay_run(tf=0.1, verbose=True)
    line = out.getvalue().splitlines()[0]
    assert line.startswith("MPC @ t=0.00s") and "S_min=" in line
    assert "effective_samples=" in line and "/512" in line and "exit_rate=" in line


def test_mppis_tree_holds_the_problems_cost_and_a_moved_weight_moves_the_plan():
    planner = double_integrator_planner()
    pv = planner.compile()
    assert {
        "x_start",
        "t_start",
        "plant",
        "cost",
        "discount_rate",
        "method",
        "seed",
    } <= set(pv.params)
    z0 = pv.initial()
    z, _ = pv.solve(None, z0)
    heavy = {"cost": {**pv.params["cost"], "R": 100.0 * np.eye(1)}}
    z_heavy, _ = pv.solve(heavy, z0)
    assert np.max(np.abs(z_heavy[:-1])) < np.max(np.abs(z[:-1]))


def test_the_mppi_mpcs_cost_is_tuned_through_the_solve_block():
    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    cost = QuadraticCost.from_system(plant, R=0.01 * np.eye(1))
    problem = PlanningProblem(
        plant, cost=cost, tf=0.5, x_start=[1.0, 0.0], U=plant.input_box
    )
    planner = MPPIPlanner(problem, dt=DT, n_samples=256, temperature=0.1, noise=4.0)
    mpc = ModelPredictiveController(planner, delay=False)
    pe = problem.replace(tf=0.3).compile(mpc % DT, backend="numpy")
    base = pe.params
    solve = base["controller"]["solve"]
    heavy = {
        **base,
        "controller": {
            **base["controller"],
            "solve": {**solve, "cost": {**solve["cost"], "R": 100.0 * np.eye(1)}},
        },
    }
    u_base = pe.trajectory().of("plant").u
    u_heavy = pe.trajectory(heavy).of("plant").u
    assert not np.allclose(u_heavy, u_base)


def test_a_gradient_through_mppis_temperature_is_finite():
    pv = double_integrator_planner().compile()
    z0 = jnp.asarray(pv.initial())

    def cost(lam):
        _, status = pv.solve_fn(
            {**pv.params, "method": {**pv.params["method"], "temperature": lam}}, z0
        )
        return status["cost"]

    assert np.isfinite(float(jax.grad(cost)(0.01)))
