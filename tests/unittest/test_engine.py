"""The problem evaluator: the run, the scoring formula, the draws, the gradient and the program."""

import numpy as np
import pytest

from minilink import PID, Pendulum, Simulator
from minilink.core.costs import QuadraticCost
from minilink.core.distributions import Gaussian, Uniform
from minilink.core.sets import BallSet, BoxInputSet
from minilink.core.trajectory import Trajectory
from minilink.planning.engine import (
    Evaluation,
    closed_loop_of,
    evaluate,
    reduce_criterion,
    score_run,
)
from minilink.planning.problems import PlanningProblem, StochasticPlanningProblem

UPRIGHT = {"r": np.array([np.pi])}


def plant():
    return Pendulum()


def upright_problem(**fields):
    sys = plant()
    cost = QuadraticCost.from_system(
        sys, xbar=np.array([np.pi, 0.0]), R=0.01 * np.eye(1)
    )
    fields.setdefault("tf", 2.0)
    fields.setdefault("U", BoxInputSet.from_bounds([-50.0], [50.0]))
    return PlanningProblem(sys, cost=cost, **fields)


def pid():
    return PID(Kp=40.0, Ki=1.0, Kd=8.0)


# --- the run: the loop's trajectory, its signals and the plant's role ---


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
def test_the_run_records_every_block_output_and_the_plant_role(backend):
    pe = upright_problem().compile(pid(), dt=0.01, backend=backend)
    traj = pe.trajectory(UPRIGHT)

    # The loop's state on the grid, every block output recorded, the plant's input among them
    assert traj.x.shape == (4, 201)
    assert np.allclose(traj.t[-1], 2.0)
    assert "controller:u" in traj.signals
    assert np.allclose(traj.signals["plant.u"], traj.signals["controller:u"])

    # The plant's run: its state slice and its input
    run = traj.of("plant")
    assert run.x.shape == (2, 201) and run.u.shape == (1, 201)
    assert np.allclose(run.x[:, 0], 0.0)


@pytest.mark.jax
def test_numpy_and_jax_runs_agree():
    problem = upright_problem()
    reports = [
        problem.compile(pid(), dt=0.01, backend=backend).evaluate(UPRIGHT)
        for backend in ("numpy", "jax")
    ]
    assert np.allclose(reports[0].J, reports[1].J, rtol=1e-9)


@pytest.mark.jax
def test_the_fixed_step_run_matches_the_adaptive_simulator():
    problem = upright_problem()
    pe = problem.compile(pid(), dt=0.001, backend="jax")
    traj = pe.trajectory(UPRIGHT)

    loop = pe.loop
    loop.inputs["r"].nominal_value = np.array([np.pi])
    sim = Simulator(loop, tf=2.0, solver="scipy")
    sim.solver_backend_options.update(rtol=1e-10, atol=1e-10)
    reference = sim.solve()
    assert np.allclose(traj.x[:, -1], reference.x[:, -1], rtol=1e-5, atol=1e-6)


# --- the score: one formula for every run ---


def test_the_score_is_the_integral_plus_the_terminal_cost():
    sys = plant()
    cost = QuadraticCost.from_system(sys, S=np.eye(2))
    problem = PlanningProblem(sys, cost=cost, tf=1.0, U=sys.input_box)
    t = np.linspace(0.0, 1.0, 101)
    x = np.vstack([np.ones_like(t), np.zeros_like(t)])
    u = np.zeros((1, t.size))

    row = score_run(problem, Trajectory(t, x, u))

    # ∫₀¹ x₁² dt = 1, plus h = x₁(1)² = 1
    assert row["cost"] == pytest.approx(2.0)
    assert row["J"] == pytest.approx(2.0)
    assert not row["failed"] and row["reached"]


def test_the_price_switch_charges_a_failed_or_missed_run():
    sys = plant()
    cost = QuadraticCost.from_system(sys)
    goal = BallSet(center=[np.pi, 0.0], radius=0.1)
    t = np.linspace(0.0, 1.0, 11)
    run = Trajectory(t, np.zeros((2, t.size)), np.full((1, t.size), 10.0))

    # Off: J is the cost; the run left U (|u| > 5) and missed the goal
    off = score_run(
        PlanningProblem(sys, cost=cost, tf=1.0, Xf=goal, U=sys.input_box), run
    )
    assert off["failed"] and not off["reached"]
    assert off["J"] == pytest.approx(off["cost"])
    assert off["feasibility_margin"] == pytest.approx(-5.0)
    assert off["reachability_margin"] == pytest.approx(0.1 - np.pi)

    # On: the price is added once
    on = score_run(
        PlanningProblem(
            sys, cost=cost, tf=1.0, Xf=goal, infeasible_cost=1000.0, U=sys.input_box
        ),
        run,
    )
    assert on["J"] == pytest.approx(off["cost"] + 1000.0)


def test_a_run_that_is_not_finite_costs_inf_or_the_price():
    sys = plant()
    cost = QuadraticCost.from_system(sys)
    t = np.linspace(0.0, 1.0, 11)
    x = np.zeros((2, t.size))
    x[0, -1] = np.nan
    run = Trajectory(t, x, np.zeros((1, t.size)))

    off = score_run(PlanningProblem(sys, cost=cost, tf=1.0, U=sys.input_box), run)
    assert off["failed"] and off["J"] == np.inf and off["cost"] == np.inf

    on = score_run(
        PlanningProblem(sys, cost=cost, tf=1.0, infeasible_cost=50.0, U=sys.input_box),
        run,
    )
    assert on["J"] == pytest.approx(50.0)


def test_the_criteria_reduce_the_runs():
    J = np.array([1.0, 2.0, 3.0, 10.0])
    assert reduce_criterion(J, "expectation") == pytest.approx(4.0)
    assert reduce_criterion(J, "worst_case") == pytest.approx(10.0)
    assert reduce_criterion(J, ("cvar", 0.5)) == pytest.approx(6.5)


# --- the draws: one run, or n_trials from one key ---


def stochastic_problem(**fields):
    sys = plant()
    cost = QuadraticCost.from_system(sys)
    return StochasticPlanningProblem(
        sys=sys,
        cost=cost,
        tf=1.0,
        U=BoxInputSet.from_bounds([-50.0], [50.0]),
        x0_distribution=Gaussian(mean=np.array([0.5, 0.0]), std=np.array([0.1, 0.1])),
        params_distribution={"m": Uniform(np.array([0.9]), np.array([1.1]))},
        **fields,
    )


def test_a_deterministic_problem_is_one_run():
    pe = upright_problem().compile(pid(), dt=0.01, backend="numpy")
    assert pe.evaluate(UPRIGHT).n_trials == 1
    with pytest.raises(ValueError, match="one run"):
        pe.evaluate(n_trials=5)


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
def test_a_stochastic_problem_draws_its_trials(backend):
    pe = stochastic_problem().compile(pid(), dt=0.01, backend=backend)
    with pytest.raises(ValueError, match="n_trials"):
        pe.evaluate()

    report = pe.evaluate(n_trials=8, key=0, record=True)
    assert report.x0.shape == (8, 2)
    assert len(np.unique(report.x0[:, 0])) == 8
    assert len(report.trajectories) == 8
    assert np.allclose(report.trajectories[3].of("plant").x[:, 0], report.x0[3])

    # One key, the same draws
    again = pe.evaluate(n_trials=8, key=0)
    assert np.allclose(report.J, again.J)


def test_the_facade_is_compile_then_evaluate():
    report = evaluate(stochastic_problem(), pid(), dt=0.01, n_trials=4, backend="numpy")
    assert isinstance(report, Evaluation) and report.n_trials == 4


# --- the gradient and the program ---


@pytest.mark.jax
def test_the_gradient_matches_a_central_difference():
    import jax

    pe = upright_problem().compile(pid(), dt=0.01, backend="jax")
    params = pe.merged(UPRIGHT)
    wrt = ["controller.Kp", "controller.Kd"]

    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    z = pe.pack(params, wrt)
    h = 1e-5
    for i, path in enumerate(wrt):
        dz = np.zeros_like(z)
        dz[i] = h
        J_plus = pe.evaluate(pe.unpack(z + dz, wrt, params)).objective
        J_minus = pe.evaluate(pe.unpack(z - dz, wrt, params)).objective
        central = (float(J_plus) - float(J_minus)) / (2 * h)
        leaf = path.split(".")[1]
        assert float(grad["controller"][leaf][0]) == pytest.approx(central, rel=1e-4)


def test_pack_and_unpack_round_trip():
    pe = upright_problem().compile(pid(), dt=0.01, backend="numpy")
    wrt = ["controller.Kp", "x_start"]
    z = pe.pack(pe.params, wrt)
    assert z.tolist() == [40.0, 0.0, 0.0]

    params = pe.unpack(np.array([3.0, 0.1, 0.2]), wrt)
    assert params["controller"]["Kp"].tolist() == [3.0]
    assert params["x_start"].tolist() == [0.1, 0.2]
    assert params["controller"]["Kd"].tolist() == [8.0]
    with pytest.raises(KeyError, match="no leaf"):
        pe.pack(pe.params, ["controller.K"])


@pytest.mark.jax
def test_the_program_reads_the_same_objective_and_the_declared_rows():
    problem = upright_problem(Xf=BallSet(center=[np.pi, 0.0], radius=0.5))
    pe = problem.compile(pid(), dt=0.01, backend="jax")
    wrt = ["controller.Kp", "controller.Kd"]

    program = pe.program(wrt, bounds={"controller.Kp": (0.0, None)}, params=UPRIGHT)
    z0 = program.metadata["z0"]

    assert program.n_z == 2 and program.backend == "jax"
    assert program.lower.tolist() == [0.0, -np.inf]
    assert program.metadata["rows"] == ["feasibility_margin", "reachability_margin"]
    assert float(program.J(z0)) == pytest.approx(float(pe.evaluate(UPRIGHT).objective))
    assert np.asarray(program.g(z0)).shape == (2,)
    assert np.asarray(program.grad_J(z0)).shape == (2,)


# --- the loop: what the evaluator accepts ---


def test_a_built_loop_is_taken_as_it_is_and_a_step_system_refused():
    problem = upright_problem()
    loop = pid() @ problem.sys
    assert closed_loop_of(problem, loop) is loop

    from minilink.blocks.step import UnitDelay

    with pytest.raises(TypeError, match="StepSystem"):
        closed_loop_of(problem, UnitDelay())


def test_dt_must_divide_the_horizon():
    with pytest.raises(ValueError, match="divide"):
        upright_problem(tf=1.0).compile(pid(), dt=0.3)


# --- a trajectory you already hold ---


def test_evaluate_trajectory_scores_the_plant_run_of_a_loop_or_a_logged_run():
    from minilink.planning.engine import evaluate_trajectory

    problem = upright_problem()
    pe = problem.compile(pid(), dt=0.01, backend="numpy")
    traj = pe.trajectory(UPRIGHT)
    report = pe.evaluate(UPRIGHT)

    # The loop's run, projected by its role, and the plant's run itself: one score
    for held in (traj, traj.of("plant")):
        np.testing.assert_allclose(evaluate_trajectory(problem, held).J, report.J)

    # Signals without a role, or the wrong sizes, are refused
    with pytest.raises(ValueError, match="role"):
        evaluate_trajectory(
            problem, Trajectory(traj.t, traj.x, traj.u, signals={"s": traj.u})
        )
    with pytest.raises(ValueError, match="plant's"):
        evaluate_trajectory(problem, Trajectory(traj.t, traj.x, traj.u))


@pytest.mark.jax
def test_evaluate_trajectory_agrees_on_numpy_and_jax_with_an_exit_and_a_divergence():
    import jax.numpy as jnp

    from minilink.planning.engine import evaluate_trajectory

    sys = plant()
    cost = QuadraticCost.from_system(sys)
    problem = PlanningProblem(
        sys, cost=cost, tf=1.0, U=sys.input_box, infeasible_cost=100.0
    )
    t = np.linspace(0.0, 1.0, 11)
    runs = {
        "inside": np.zeros((1, t.size)),
        "exit": np.full((1, t.size), 10.0),
        "diverges": np.where(t > 0.5, np.inf, 0.0)[None, :],
    }
    for name, u in runs.items():
        x = np.vstack([np.sin(t), np.cos(t)])
        on_numpy = evaluate_trajectory(problem, Trajectory(t, x, u))
        on_jax = evaluate_trajectory(
            problem, Trajectory(jnp.asarray(t), jnp.asarray(x), jnp.asarray(u))
        )
        np.testing.assert_allclose(np.asarray(on_jax.J), on_numpy.J, rtol=1e-6)
        assert bool(on_jax.failed[0]) == bool(on_numpy.failed[0]) == (name != "inside")


# --- the problem's parameters: a live view of the parts, and leaves of the run ---


def test_the_problems_params_are_a_live_view_whose_writes_reach_the_parts():
    problem = upright_problem(discount_rate=0.0)
    sys, cost = problem.sys, problem.cost
    view = problem.params
    assert view["plant"] is sys.params and view["cost"] is cost.params
    assert set(view) == {"plant", "cost", "X", "U", "X0", "x_start", "discount_rate"}

    # In place, and by assignment: the parts change
    problem.params["cost"]["R"][0, 0] = 2.0
    assert cost.R[0, 0] == 2.0
    problem.params = {"plant": {**sys.params, "m": 3.0}, "discount_rate": 0.5}
    assert sys.params["m"] == 3.0 and problem.discount_rate == 0.5
    problem.params = {"x_start": [0.2, 0.0]}
    assert problem.x_start.tolist() == [0.2, 0.0] and problem.X0.point.tolist() == [
        0.2,
        0.0,
    ]

    with pytest.raises(ValueError, match="unknown problem role"):
        problem.params = {"gains": {}}
    with pytest.raises(TypeError, match="holds no parameters"):
        problem.params = {"U": {"lower": 0.0}}


@pytest.mark.jax
def test_the_gradient_reaches_the_cost_weights_and_the_discount():
    import jax

    pe = upright_problem().compile(pid(), dt=0.01, backend="jax")
    params = pe.merged(UPRIGHT)
    assert set(params["cost"]) == {"Q", "R", "S", "xbar", "ubar"}

    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    # dJ/dR₁₁ by a central difference
    h = 1e-5
    plus = pe.unpack(params["cost"]["R"].reshape(-1) + h, ["cost.R"], params)
    minus = pe.unpack(params["cost"]["R"].reshape(-1) - h, ["cost.R"], params)
    central = (
        float(pe.evaluate(plus).objective) - float(pe.evaluate(minus).objective)
    ) / (2 * h)
    assert float(grad["cost"]["R"][0, 0]) == pytest.approx(central, rel=1e-5)
    assert float(grad["discount_rate"]) < 0.0  # discounting lowers the cost


@pytest.mark.jax
@pytest.mark.parametrize("path", ["plant.m", "x_start"])
def test_the_gradient_reaches_the_plants_mass_and_the_start(path):
    import jax

    pe = upright_problem().compile(pid(), dt=0.01, backend="jax")
    params = pe.merged({**UPRIGHT, "x_start": np.array([0.3, 0.0])})
    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    # The derivative along the first entry of the leaf, by a central difference
    z = pe.pack(params, [path])
    dz = np.zeros_like(z)
    dz[0] = 1e-5
    J_plus = float(pe.evaluate(pe.unpack(z + dz, [path], params)).objective)
    J_minus = float(pe.evaluate(pe.unpack(z - dz, [path], params)).objective)
    central = (J_plus - J_minus) / 2e-5
    analytic = np.reshape(pe.pack(grad, [path]), (-1,))[0]
    assert float(analytic) == pytest.approx(central, rel=1e-4)


def test_the_recorded_plant_command_is_the_reconstructed_signal():
    pe = upright_problem().compile(pid(), dt=0.01, backend="numpy")
    traj = pe.trajectory(UPRIGHT)

    loop = pe.loop
    loop.inputs["r"].nominal_value = np.array([np.pi])
    rebuilt = loop.reconstruct_internal_signals(traj)
    np.testing.assert_allclose(
        traj.signals["plant.u"], rebuilt.signals["controller:u"], atol=1e-12
    )


@pytest.mark.jax
def test_a_tuning_that_leaves_X_comes_back_inside_through_the_margin_row():
    from minilink import Optimizer
    from minilink.core.sets import BoxSet

    sys = plant()
    cost = QuadraticCost.from_system(
        sys, xbar=np.array([np.pi, 0.0]), R=0.001 * np.eye(1)
    )
    X = BoxSet([-1.0, -20.0], [np.pi + 0.05, 20.0])  # no overshoot past upright
    problem = PlanningProblem(sys, cost=cost, tf=3.0, X=X)
    pe = problem.compile(PID(Kp=60.0, Ki=0.0, Kd=2.0), dt=0.01, backend="jax")
    assert pe.evaluate(UPRIGHT).failed[0]  # the start overshoots

    wrt = ["controller.Kp", "controller.Kd"]
    bounds = {"controller.Kp": (1.0, 200.0), "controller.Kd": (0.0, 50.0)}
    program = pe.program(wrt, bounds=bounds, params=UPRIGHT)
    assert program.metadata["rows"] == ["feasibility_margin"]
    result = Optimizer(program, method="scipy_slsqp", options={"maxiter": 100}).solve()

    tuned = pe.evaluate(pe.unpack(result.z, wrt, pe.merged(UPRIGHT)))
    assert not tuned.failed[0] and float(tuned.feasibility_margin[0]) >= 0.0


def test_a_problem_realizes_its_draws_from_one_key():
    problem = stochastic_problem()
    x0s, draws = problem.realize(0, 5)
    assert x0s.shape == (5, 2) and draws["params"]["m"].shape == (5,)
    again, _ = problem.realize(0, 5)
    np.testing.assert_array_equal(x0s, again)  # one key, the same draws
    assert np.all((draws["params"]["m"] >= 0.9) & (draws["params"]["m"] <= 1.1))

    # A deterministic problem has one realization: its start, no draws
    one, none = upright_problem().realize()
    assert one.shape == (1, 2) and none == {"params": {}, "disturbances": {}}
    with pytest.raises(ValueError, match="one realization"):
        upright_problem().realize(0, 3)


@pytest.mark.jax
def test_a_sets_membership_traces_on_jax():
    import jax
    import jax.numpy as jnp

    from minilink.core.sets import BallSet

    ball = BallSet(center=[0.0, 0.0], radius=1.0)
    box = BoxInputSet.from_bounds([-1.0], [1.0])
    inside = jax.jit(lambda z: ball.contains(z))
    assert bool(inside(jnp.array([0.5, 0.0]))) and not bool(
        inside(jnp.array([2.0, 0.0]))
    )
    assert bool(jax.jit(lambda u: box.contains(u))(jnp.array([0.5])))
    assert ball.contains(np.array([0.5, 0.0]))  # NumPy in, a NumPy boolean out
