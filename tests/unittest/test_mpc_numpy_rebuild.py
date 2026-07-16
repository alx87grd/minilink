"""NumPy rebuild-mode MPC (no JAX parametric compile)."""

import unittest
import warnings
from unittest.mock import patch

import numpy as np

from minilink.control.mpc import ModelPredictiveController, mpc_default_computer_x0
from minilink.core.costs import QuadraticCost
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.system import DynamicSystem, StepSystem, System
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)


class SingleIntegrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.state.lower_bound = np.array([-10.0])
        self.state.upper_bound = np.array([10.0])
        self.inputs["u"].lower_bound = np.array([-10.0])
        self.inputs["u"].upper_bound = np.array([10.0])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return x


def _make_numpy_planner(x_start=0.0):
    sys = SingleIntegrator()
    cost = QuadraticCost.from_system(
        sys,
        Q=np.zeros((1, 1)),
        R=np.eye(1),
        S=np.zeros((1, 1)),
    )
    problem = PlanningProblem(
        sys=sys,
        x_start=np.array([x_start]),
        cost=cost,
        tf=1.0,
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=5,
        transcription="direct_collocation",
        compile_backend="numpy",
        optimizer_options={"maxiter": 50, "ftol": 1e-4},
    )


class TestMPCNumPyRebuild(unittest.TestCase):
    def test_controller_init_without_parametric_compile(self):
        planner = _make_numpy_planner()
        n_compile = []

        with patch.object(
            planner,
            "compile_parametric_program",
            side_effect=lambda: n_compile.append(1),
        ):
            mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)

        self.assertEqual(len(n_compile), 0)
        self.assertFalse(planner.has_parametric_program)
        self.assertIsInstance(mpc, StepSystem)

    def test_algebraic_numpy_controller_init(self):
        planner = _make_numpy_planner()
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
        self.assertIsInstance(mpc, System)
        self.assertNotIsInstance(mpc, StepSystem)
        self.assertFalse(planner.has_parametric_program)

    def test_compute_command_rebuilds_plan(self):
        planner = _make_numpy_planner(0.1)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)

        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            cmd = mpc.compute_command(np.array([0.1]), k=0)

        rebuild_msgs = [
            w.message
            for w in caught
            if "rebuilding the NLP each call" in str(w.message)
        ]
        self.assertEqual(rebuild_msgs, [])
        self.assertTrue(cmd.success)
        self.assertIsNotNone(planner.last_program)
        np.testing.assert_allclose(cmd.u_ff, cmd.plan.trajectory.u[:, 0])

    def test_rebuild_per_tick_new_program(self):
        planner = _make_numpy_planner(0.0)
        mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)

        programs = []
        original_solve = planner.solve_trajectory

        def solve_and_track(*args, **kwargs):
            plan = original_solve(*args, **kwargs)
            programs.append(planner.last_program)
            return plan

        with patch.object(planner, "solve_trajectory", side_effect=solve_and_track):
            mpc.compute_command(np.array([0.0]), k=0)
            mpc.compute_command(np.array([0.05]), k=1)

        self.assertEqual(len(programs), 2)
        self.assertIsNot(programs[0], programs[1])

    def test_hybrid_closed_loop_smoke(self):
        planner = _make_numpy_planner(0.0)
        plant = SingleIntegrator()
        plant.x0 = np.array([0.0])
        dt_mpc = 0.2
        tf = 0.6
        n_ticks = int(round(tf / dt_mpc))

        mpc = ModelPredictiveController(planner, dt_mpc=dt_mpc, warm_start=True)
        hybrid = mpc @ plant
        self.assertIsInstance(hybrid, HybridDiagram)

        n_solve = []
        n_compile_parametric = []
        _solve = planner.solve_trajectory_from
        _compile_parametric = planner.compile_parametric_program

        def solve_w(*args, **kwargs):
            n_solve.append(1)
            return _solve(*args, **kwargs)

        def compile_parametric_w(*args, **kwargs):
            n_compile_parametric.append(1)
            return _compile_parametric(*args, **kwargs)

        planner.solve_trajectory_from = solve_w
        planner.compile_parametric_program = compile_parametric_w
        try:
            hybrid.compute_trajectory(
                tf=tf,
                x0_plant=plant.x0,
                x0_computer=mpc_default_computer_x0(planner),
                compile_backend="numpy",
                verbose=False,
            )
        finally:
            planner.solve_trajectory_from = _solve
            planner.compile_parametric_program = _compile_parametric

        self.assertEqual(len(n_solve), n_ticks)
        self.assertEqual(len(n_compile_parametric), 0)
