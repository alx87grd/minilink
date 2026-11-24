"""Contract tests for the PlanningProblem shortcuts (evaluate, compare, program) and System.rollout_batch."""

from __future__ import annotations

from importlib.util import find_spec

import numpy as np
import pytest

from minilink import (
    Impedance,
    Optimizer,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Uniform,
)

JAX_AVAILABLE = find_spec("jax") is not None


class TestPlanningProblemShortcuts:
    """PlanningProblem.evaluate, .compare, and .program match their standalone/evaluator twins."""

    def test_evaluate_is_the_compiled_evaluation(self):
        plant = Pendulum()
        cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
        problem = PlanningProblem(plant, x0=[1.0, 0.0], horizon=2.0, cost=cost)
        ctrl = Impedance(Kp=5.0, Kd=2.0)

        via_method = problem.evaluate(ctrl, dt=0.05, backend="numpy")
        via_compile = problem.compile(ctrl, dt=0.05, backend="numpy").evaluate()

        assert np.isclose(via_method.objective, via_compile.objective)
        assert np.allclose(via_method.J, via_compile.J)

    def test_compare_scores_each_entry_as_evaluate_does(self):
        plant = Pendulum()
        cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
        problem = PlanningProblem(
            plant,
            x0=[1.0, 0.0],
            horizon=2.0,
            cost=cost,
            distributions={"plant": {"m": Uniform(0.9, 1.1)}},
        )
        entries = {
            "Soft": Impedance(Kp=2.0, Kd=1.0),
            "Stiff": Impedance(Kp=10.0, Kd=3.0),
        }

        comparison = problem.compare(dt=0.05, n_trials=4, backend="numpy", **entries)

        assert list(comparison.evaluations) == ["Soft", "Stiff"]
        assert comparison.problem is problem
        for name, ctrl in entries.items():
            alone = problem.evaluate(ctrl, dt=0.05, n_trials=4, backend="numpy")
            assert np.allclose(comparison.evaluations[name].J, alone.J)

    def test_program_matches_compiled_evaluator_and_solves(self):
        plant = Pendulum()
        cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
        problem = PlanningProblem(plant, x0=[1.0, 0.0], horizon=2.0, cost=cost)
        ctrl = Impedance(Kp=1.0, Kd=1.0)
        wrt = ["controller.Kp", "controller.Kd"]
        bounds = {"controller.Kp": (0.0, 30.0), "controller.Kd": (0.0, 15.0)}

        prog_method = problem.program(
            ctrl, wrt, bounds=bounds, dt=0.05, backend="numpy"
        )
        prog_compiled = problem.compile(ctrl, dt=0.05, backend="numpy").program(
            wrt, bounds=bounds
        )

        z0 = np.array([1.0, 1.0])
        assert np.isclose(prog_method.J(z0), prog_compiled.J(z0))

        best = Optimizer(prog_method, method="slsqp").solve()
        assert best.success
        assert best.J < prog_method.J(z0)


class TestSystemRolloutBatch:
    """System.rollout_batch and the compiled rollout_batch: the same runs on NumPy and JAX."""

    def test_numpy_rollout_batch_matches_individual_rollouts(self):
        plant = Pendulum()
        starts = np.array([[0.5, 0.0], [1.5, 0.0], [-1.0, 0.5]])

        t, xs = plant.rollout_batch(starts, tf=1.0, dt=0.05, backend="numpy")

        assert t.shape == (21,)
        assert np.isclose(t[0], 0.0)
        assert np.isclose(t[-1], 1.0)
        assert xs.shape == (3, 21, 2)

        ev = plant.compile(backend="numpy")
        for i, x0 in enumerate(starts):
            single = ev.rollout(x0, None, 0.0, 0.05, n_steps=20)
            assert np.allclose(xs[i], single)

    def test_numpy_rollout_batch_with_batched_params_and_inputs(self):
        plant = Pendulum()
        lengths = np.array([0.5, 1.0, 2.0])
        starts = np.tile([1.0, 0.0], (3, 1))
        params = dict(plant.params, l=lengths)
        u_shared = np.full((10, 1), 0.5)

        t, xs = plant.rollout_batch(
            starts,
            u=u_shared,
            dt=0.05,
            n_steps=10,
            params=params,
            solver="euler",
            backend="numpy",
        )

        assert t.shape == (11,)
        assert xs.shape == (3, 11, 2)
        # Shorter pendulum accelerates faster under gravity
        assert abs(xs[0, -1, 0] - xs[2, -1, 0]) > 1e-2

        # Per-member inputs (B, N, m)
        u_batch = np.zeros((3, 10, 1))
        u_batch[1, :, 0] = 2.0
        _, xs_u = plant.rollout_batch(
            starts, u=u_batch, dt=0.05, n_steps=10, backend="numpy"
        )
        assert not np.allclose(xs_u[0], xs_u[1])

    def test_rollout_batch_validation(self):
        plant = Pendulum()
        starts = np.zeros((2, 2))
        with pytest.raises(ValueError, match="not both"):
            plant.rollout_batch(starts, tf=1.0, n_steps=10, dt=0.1, backend="numpy")
        with pytest.raises(ValueError, match="positive"):
            plant.rollout_batch(starts, tf=1.0, dt=0.0, backend="numpy")
        with pytest.raises(ValueError, match="shape"):
            plant.rollout_batch(np.zeros((2, 5)), tf=1.0, dt=0.1, backend="numpy")

    def test_compiled_numpy_evaluator_rollout_batch_loops(self):
        plant = Pendulum()
        starts = np.array([[0.5, 0.0], [1.0, 0.0]])
        ev = plant.compile(backend="numpy")

        xs = ev.rollout_batch(starts, n_steps=5, dt=0.01, solver="euler")

        assert xs.shape == (2, 6, 2)
        for i, x0 in enumerate(starts):
            single = ev.rollout(x0, None, 0.0, 0.01, solver="euler", n_steps=5)
            assert np.allclose(xs[i], single)

    def test_step_evaluator_rollout_batch_needs_jax(self):
        plant = Pendulum()
        step_ev = plant.discretize(0.01).compile(backend="numpy")
        with pytest.raises(TypeError, match="JAX evaluator"):
            step_ev.rollout_batch(np.zeros((2, 2)), n_steps=5)

    @pytest.mark.jax
    @pytest.mark.skipif(not JAX_AVAILABLE, reason="JAX not installed")
    def test_jax_and_numpy_rollout_batch_agree(self):
        plant = Pendulum()
        starts = np.column_stack([np.linspace(-2.0, 2.0, 8), np.zeros(8)])
        lengths = np.linspace(0.6, 1.8, 8)
        params = dict(plant.params, l=lengths)

        t_np, xs_np = plant.rollout_batch(
            starts, tf=1.0, dt=0.02, params=params, backend="numpy"
        )
        t_jax, xs_jax = plant.rollout_batch(
            starts, tf=1.0, dt=0.02, params=params, backend="jax"
        )
        t_auto, xs_auto = plant.rollout_batch(
            starts, tf=1.0, dt=0.02, params=params, backend="auto"
        )

        assert np.allclose(t_np, t_jax)
        assert np.allclose(xs_np, xs_jax, rtol=1e-12, atol=1e-12)
        assert np.allclose(t_np, t_auto)
        assert np.allclose(xs_np, xs_auto, rtol=1e-12, atol=1e-12)

    @pytest.mark.jax
    @pytest.mark.skipif(not JAX_AVAILABLE, reason="JAX not installed")
    def test_jax_rollout_batch_reads_the_solver(self):
        plant = Pendulum()
        starts = np.array([[1.0, 0.0], [2.0, 0.0]])

        _, rk4 = plant.rollout_batch(starts, n_steps=20, dt=0.05, backend="jax")
        _, euler = plant.rollout_batch(
            starts, n_steps=20, dt=0.05, solver="euler", backend="jax"
        )
        _, euler_np = plant.rollout_batch(
            starts, n_steps=20, dt=0.05, solver="euler", backend="numpy"
        )

        assert not np.allclose(rk4, euler)
        assert np.allclose(euler, euler_np, rtol=1e-12, atol=1e-12)


class TestTheCompiledObjective:
    """J = problem.compile(controller): J(p) is the objective, its draws fixed at the compile."""

    def rest_task(self, stochastic):
        plant = Pendulum()
        cost = QuadraticCost.from_system(plant, R=0.1 * np.eye(1))
        task = PlanningProblem(plant, x0=[2.0, 0.0], horizon=2.0, cost=cost)
        if stochastic:
            task = task.replace(distributions={"plant": {"m": Uniform(0.8, 1.2)}})
        return task

    def test_J_of_p_is_the_objective(self):
        task = self.rest_task(stochastic=False)
        J = task.compile(Impedance(Kp=1.0, Kd=1.0), dt=0.05, backend="numpy")

        assert np.isclose(J(), J.evaluate().objective)
        assert np.isclose(J(J.params), J.evaluate(J.params).objective)

    def test_the_draws_are_the_compiles_unless_a_call_names_its_own(self):
        task = self.rest_task(stochastic=True)
        controller = Impedance(Kp=1.0, Kd=1.0)
        J = task.compile(controller, dt=0.05, n_trials=6, key=3, backend="numpy")

        assert J.evaluate().n_trials == 6
        assert np.isclose(J(), J.evaluate(key=3, n_trials=6).objective)
        assert J.evaluate(n_trials=2).n_trials == 2

        unset = task.compile(controller, dt=0.05, backend="numpy")
        with pytest.raises(ValueError, match="needs n_trials"):
            unset()

    @pytest.mark.jax
    @pytest.mark.skipif(not JAX_AVAILABLE, reason="JAX not installed")
    def test_the_gradient_of_J_is_the_gradient_of_its_objective(self):
        import jax

        task = self.rest_task(stochastic=True)
        J = task.compile(Impedance(Kp=1.0, Kd=1.0), dt=0.05, n_trials=4, backend="jax")

        dJ = jax.grad(J)(J.params)
        by_hand = jax.grad(lambda p: J.evaluate(p, n_trials=4).objective)(J.params)

        for leaf in ("Kp", "Kd"):
            np.testing.assert_allclose(
                dJ["controller"][leaf], by_hand["controller"][leaf], rtol=1e-12
            )
