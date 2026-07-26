import unittest
import warnings
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
import pytest
from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem
from minilink.simulation.simulator import (
    COMPILE_BACKEND_AUTO,
    DISCONTINUOUS_AUTO_DT_SCALE,
    Simulator,
)


def _have_jax() -> bool:
    try:
        import jax

        return True
    except ImportError:
        return False


class StableLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "StableLinearSystem"
        self.x0 = np.array([1.0])

    def f(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([-x[0] + u[0]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])


class TwoPortLinearSystem(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, output_dim=1, y_dependencies=())
        self.name = "TwoPortLinearSystem"
        self.add_input_port("left", nominal_value=1.0)
        self.add_input_port("right", nominal_value=2.0)
        self.x0 = np.array([0.0])

    def f(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([-x[0] + u[0] + 2.0 * u[1]])

    def h(self, x, u, t=0, params=None):
        xp = array_module(x)
        return xp.array([x[0]])


class DiscontinuousLinearSystem(StableLinearSystem):
    def __init__(self):
        super().__init__()
        self.solver_info["discontinuous_behavior"] = True


class TestNewSimulator(unittest.TestCase):
    def test_default_solver_auto_selects_euler_for_discontinuous_system(self):
        sim = Simulator(
            DiscontinuousLinearSystem(),
            tf=1.0,
            n_steps=5,
            solver_warnings="ignore",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "euler")

    def test_large_grid_numpy_stays_scipy(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=10000,
            compile_backend="numpy",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "scipy")

    def test_default_compile_backend_is_numpy(self):
        sim = Simulator(StableLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        self.assertEqual(sim.compile_backend, "numpy")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_compile_backend_auto_resolves_to_jax_when_available(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=5,
            compile_backend=COMPILE_BACKEND_AUTO,
            verbose=False,
        )
        self.assertEqual(sim.compile_backend, "jax")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_auto_selects_rk4_for_large_uniform_grid_with_jax(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=10000,
            compile_backend="jax",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "rk4_fixedsteps")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_jax_grid_below_threshold_stays_scipy(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=1.0,
            n_steps=9999,
            compile_backend="jax",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "scipy")

    @pytest.mark.optional
    @pytest.mark.jax
    @unittest.skipUnless(_have_jax(), "jax not installed")
    def test_discontinuous_jax_large_grid_stays_euler(self):
        sim = Simulator(
            DiscontinuousLinearSystem(),
            tf=1.0,
            n_steps=10000,
            compile_backend="jax",
            solver_warnings="ignore",
            verbose=False,
        )
        self.assertEqual(sim.solver_mode, "euler")

    def test_discontinuous_auto_dt_is_finer(self):
        sim = Simulator(
            DiscontinuousLinearSystem(),
            tf=0.01,
            solver_warnings="ignore",
            verbose=False,
        )
        expected_dt = 0.001 * DISCONTINUOUS_AUTO_DT_SCALE
        self.assertAlmostEqual(sim.t[1] - sim.t[0], expected_dt)

    def test_discontinuous_auto_solver_emits_warning(self):
        with pytest.warns(UserWarning, match="Discontinuous feedback"):
            Simulator(
                DiscontinuousLinearSystem(),
                tf=1.0,
                n_steps=5,
                solver_warnings="warn",
                verbose=False,
            )

    def test_discontinuous_forced_rk4_emits_substep_warning(self):
        with pytest.warns(UserWarning, match="sub-step"):
            Simulator(
                DiscontinuousLinearSystem(),
                tf=1.0,
                n_steps=5,
                solver="rk4_fixedsteps",
                solver_warnings="warn",
                verbose=False,
            )

    def test_discontinuous_forced_scipy_stiff_emits_adaptive_warning(self):
        with pytest.warns(UserWarning, match="adaptive stepping"):
            Simulator(
                DiscontinuousLinearSystem(),
                tf=1.0,
                n_steps=5,
                solver="scipy_stiff",
                solver_warnings="warn",
                verbose=False,
            )

    def test_smooth_system_emits_no_discontinuous_warning(self):
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", UserWarning)
            Simulator(StableLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        discontinuous_msgs = [
            w.message for w in caught if "Discontinuous feedback" in str(w.message)
        ]
        self.assertEqual(discontinuous_msgs, [])

    def test_invalid_time_grid_arguments_raise_value_error(self):
        sys = StableLinearSystem()
        with self.assertRaises(ValueError):
            Simulator(sys, tf=0.0, n_steps=5, verbose=False)
        with self.assertRaises(ValueError):
            Simulator(sys, tf=1.0, n_steps=1, verbose=False)
        with self.assertRaises(ValueError):
            Simulator(sys, tf=1.0, dt=0.0, verbose=False)

    def test_invalid_x0_shape_raises_value_error(self):
        with self.assertRaises(ValueError):
            Simulator(
                StableLinearSystem(),
                x0=np.array([[1.0]]),
                tf=1.0,
                n_steps=5,
                verbose=False,
            )

    def test_scipy_failure_raises_and_keeps_debug_information(self):
        failed_solution = SimpleNamespace(
            success=False,
            status=-1,
            message="integration failed",
            nfev=12,
            njev=0,
            nlu=0,
            t=np.array([0.0]),
            y=np.array([[1.0]]),
        )
        sim = Simulator(StableLinearSystem(), tf=1.0, n_steps=5, verbose=False)
        with patch(
            "minilink.simulation.solvers.scipy_ivp.solve_ivp",
            return_value=failed_solution,
        ):
            with self.assertRaises(RuntimeError) as ctx:
                sim.solve()
        self.assertIn("integration failed", str(ctx.exception))
        sb = sim.solver_backend
        self.assertIs(sb.last_solve_ivp_solution, failed_solution)
        self.assertFalse(sb.last_debug["success"])
        self.assertEqual(sb.last_debug["message"], "integration failed")
        self.assertFalse(hasattr(sim, "last_traj"))

    def test_solve_populates_last_traj_and_last_debug(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        traj = sim.solve()
        self.assertIs(sim.last_traj, traj)
        self.assertEqual(sim.last_debug["solver"], "euler")
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))

    def test_solve_forced_validates_shape(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        bad_u = np.zeros((3, 1))
        with self.assertRaises(ValueError):
            sim.solve_forced(bad_u)

    def test_solve_forced_accepts_callable_full_input(self):
        sim = Simulator(
            StableLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        traj = sim.solve_forced(lambda t: 10.0 * t)
        np.testing.assert_allclose(traj.u, np.array([[0.0, 1.0, 2.0]]))

    def test_solve_forced_accepts_constant_vector(self):
        sim = Simulator(
            TwoPortLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        traj = sim.solve_forced(np.array([3.0, 4.0]))
        np.testing.assert_allclose(traj.u, np.array([[3.0, 3.0, 3.0], [4.0, 4.0, 4.0]]))

    def test_solve_forced_accepts_scalar_on_one_named_port(self):
        sim = Simulator(
            TwoPortLinearSystem(), tf=0.2, n_steps=3, solver="euler", verbose=False
        )
        traj = sim.solve_forced(5.0, input_port_id="left")
        np.testing.assert_allclose(traj.u[0, :], np.array([5.0, 5.0, 5.0]))
        np.testing.assert_allclose(traj.u[1, :], np.array([2.0, 2.0, 2.0]))

    def test_solve_forced_supports_fixed_step_rk4(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=0.2,
            n_steps=3,
            solver="rk4_fixedsteps",
            verbose=False,
        )
        u_traj = np.zeros((1, 3))
        traj = sim.solve_forced(u_traj)
        self.assertEqual(sim.last_debug["solver"], "rk4_fixedsteps")
        np.testing.assert_allclose(traj.u, u_traj)
        np.testing.assert_allclose(traj.x[:, 0], [1.0])
        self.assertLess(traj.x[0, -1], 1.0)

    def test_wrapper_compute_trajectory_uses_new_simulator_path(self):
        sys = StableLinearSystem()
        traj = sys.compute_trajectory(
            tf=0.2, n_steps=3, solver="euler", show=False, verbose=False
        )
        self.assertIs(sys.traj, traj)
        self.assertEqual(traj.x.shape, (1, 3))
        self.assertEqual(traj.u.shape, (1, 3))
        self.assertEqual(traj.t.shape, (3,))

    def test_wrapper_compute_forced_accepts_full_input_trajectory(self):
        sys = StableLinearSystem()
        u_traj = np.array([[0.0, 0.5, 1.0]])
        traj = sys.compute_forced(
            u_traj, tf=0.2, n_steps=3, solver="euler", show=False, verbose=False
        )
        self.assertIs(sys.traj, traj)
        np.testing.assert_allclose(traj.u, u_traj)

    def test_wrapper_compute_forced_samples_callable_on_one_named_port(self):
        sys = TwoPortLinearSystem()
        traj = sys.compute_forced(
            lambda t: 10.0 * t,
            input_port_id="left",
            tf=0.2,
            n_steps=3,
            solver="euler",
            show=False,
            verbose=False,
        )
        np.testing.assert_allclose(traj.u[0, :], np.array([0.0, 1.0, 2.0]))
        np.testing.assert_allclose(traj.u[1, :], np.array([2.0, 2.0, 2.0]))


class TestEulerSolverModes(unittest.TestCase):
    def setUp(self):
        self.plant = StableLinearSystem()
        self.ev = self.plant.compile(backend="numpy", verbose=False)

    def test_euler_variable_dt_on_nonuniform_grid(self):
        from minilink.simulation.solvers.euler import EulerSolverBackend

        times = np.array([0.0, 0.05, 0.2, 0.25])
        backend = EulerSolverBackend()
        x_traj = backend.integrate(self.ev, times, self.plant.x0)
        self.assertEqual(x_traj.shape, (1, times.size))
        self.assertEqual(backend.last_debug["solver"], "euler")

    def test_euler_fixedsteps_matches_euler_on_uniform_grid(self):
        from minilink.simulation.solvers.euler import EulerSolverBackend
        from minilink.simulation.solvers.euler_fixed import EulerFixedStepSolverBackend

        times = np.linspace(0.0, 0.2, 4)
        generic = EulerSolverBackend()
        fixed = EulerFixedStepSolverBackend()
        x_generic = generic.integrate(self.ev, times, self.plant.x0)
        x_fixed = fixed.integrate(self.ev, times, self.plant.x0)
        np.testing.assert_allclose(x_generic, x_fixed, rtol=0.0, atol=1e-12)
        self.assertEqual(fixed.last_debug["solver"], "euler_fixedsteps")

    def test_euler_fixedsteps_rejects_nonuniform_grid(self):
        from minilink.simulation.solvers.euler_fixed import EulerFixedStepSolverBackend

        times = np.array([0.0, 0.05, 0.2, 0.25])
        backend = EulerFixedStepSolverBackend()
        with self.assertRaises(ValueError):
            backend.integrate(self.ev, times, self.plant.x0)

    def test_simulator_accepts_euler_fixedsteps(self):
        sim = Simulator(
            StableLinearSystem(),
            tf=0.2,
            n_steps=3,
            solver="euler_fixedsteps",
            verbose=False,
        )
        traj = sim.solve()
        self.assertEqual(sim.solver_mode, "euler_fixedsteps")
        self.assertEqual(sim.last_debug["solver"], "euler_fixedsteps")
        self.assertEqual(traj.x.shape, (1, 3))


from minilink.blocks.routing import Gain
from minilink.blocks.sources import Step
from minilink.simulation.static_simulator import StaticSimulator

try:
    import jax

    _JAX_AVAILABLE = True
except ImportError:
    _JAX_AVAILABLE = False


class TestStaticSimulator(unittest.TestCase):
    def test_gain_compute_trajectory_shape(self):
        gain = Gain(K=2.0, dim=1)
        traj = gain.compute_trajectory(t0=0, tf=1, n_steps=5, verbose=False)
        self.assertEqual(traj.x.shape, (0, 5))
        self.assertEqual(traj.u.shape, (1, 5))
        self.assertIn("y", traj.signals)
        np.testing.assert_allclose(traj.signals["y"], np.zeros((1, 5)))

    def test_step_source_signals_on_grid(self):
        step = Step(
            initial_value=np.array([0.0]), final_value=np.array([1.0]), step_time=0.5
        )
        traj = step.compute_trajectory(t0=0, tf=1, n_steps=3, verbose=False)
        self.assertEqual(traj.x.shape, (0, 3))
        y = traj.signals["y"]
        np.testing.assert_array_equal(y[:, 0], [0.0])
        np.testing.assert_array_equal(y[:, 1], [1.0])
        np.testing.assert_array_equal(y[:, 2], [1.0])

    def test_static_simulator_direct(self):
        gain = Gain(K=3.0, dim=1)
        sim = StaticSimulator(gain, t0=0, tf=0.2, n_steps=3)
        traj = sim.solve_forced(np.array([[1.0, 2.0, 3.0]]))
        np.testing.assert_allclose(traj.signals["y"], [[3.0, 6.0, 9.0]])


@pytest.mark.optional
@pytest.mark.jax
@unittest.skipUnless(_JAX_AVAILABLE, "JAX not installed")
class TestStaticSimulatorJax(unittest.TestCase):
    def test_gain_jax_compile_backend(self):
        gain = Gain(K=2.0, dim=1)
        traj = gain.compute_trajectory(
            t0=0, tf=0.5, n_steps=3, compile_backend="jax", verbose=False
        )
        self.assertEqual(traj.x.shape, (0, 3))
        self.assertIn("y", traj.signals)


from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.simulator import DISCONTINUOUS_AUTO_DT_SCALE, Simulator

TF = 1.0
DT_COARSE = 0.1


def _build_smc_diagram():
    """Match ``examples/scripts/control/demo_sliding_mode_pendulum.py`` setup."""
    plant = Pendulum(length=1.0, mass=1.0)
    model = Pendulum(length=1.0, mass=0.5)
    plant.x0 = np.array([1.0, 0.0])
    ref = Step(
        initial_value=np.array([np.pi, 0.0]),
        final_value=np.array([0.0, 0.0]),
        step_time=0.5,
    )
    smc = SlidingModeController(model, lam=20.0, gain=8.0, nab=0.15)
    return ref >> closed_loop_qdq(smc, plant)


def _max_ddq_consistency_error(diagram, traj):
    """Max |Δdq/Δt − ddq_from_f| on interior grid points."""
    q = traj.x[0]
    dq = traj.x[1]
    dt = np.diff(traj.t)
    ddq_num = np.diff(dq) / dt
    errors = []
    for i in range(len(ddq_num)):
        x_i = np.array([q[i], dq[i]])
        k = diagram.f(x_i, traj.u[:, i], traj.t[i])
        errors.append(abs(ddq_num[i] - float(k[1])))
    return float(np.max(errors)) if errors else 0.0


def _integrate(diagram, *, solver, dt=DT_COARSE, tf=TF):
    return diagram.compute_trajectory(
        tf=tf, dt=dt, solver=solver, solver_warnings="ignore", show=False, verbose=False
    )


class TestDiscontinuousSolvers(unittest.TestCase):
    def setUp(self):
        self.diagram = _build_smc_diagram()
        self.assertTrue(self.diagram.solver_info["discontinuous_behavior"])

    def test_auto_solver_selects_euler(self):
        sim = Simulator(
            self.diagram, tf=TF, dt=DT_COARSE, solver_warnings="ignore", verbose=False
        )
        self.assertEqual(sim.solver_mode, "euler")

    def test_auto_dt_uses_discontinuous_scale(self):
        sim = Simulator(self.diagram, tf=0.01, solver_warnings="ignore", verbose=False)
        expected_dt = 0.001 * DISCONTINUOUS_AUTO_DT_SCALE
        self.assertAlmostEqual(sim.t[1] - sim.t[0], expected_dt)

    def test_euler_matches_f_based_ddq_better_than_rk4_on_coarse_dt(self):
        traj_euler = _integrate(self.diagram, solver="euler")
        traj_rk4 = _integrate(self.diagram, solver="rk4_fixedsteps")
        err_euler = _max_ddq_consistency_error(self.diagram, traj_euler)
        err_rk4 = _max_ddq_consistency_error(self.diagram, traj_rk4)
        self.assertLess(
            err_euler,
            err_rk4,
            "Euler should align Δdq/Δt with f-based ddq better than RK4 sub-steps",
        )

    def test_rk4_coarse_dt_can_freeze_state_while_euler_moves(self):
        traj_euler = _integrate(self.diagram, solver="euler")
        traj_rk4 = _integrate(self.diagram, solver="rk4_fixedsteps")
        dq_motion_euler = float(np.max(np.abs(np.diff(traj_euler.x[1]))))
        dq_motion_rk4 = float(np.max(np.abs(np.diff(traj_rk4.x[1]))))
        self.assertGreater(
            dq_motion_euler,
            dq_motion_rk4,
            "RK4 sub-step cancellation can stall dq updates on a coarse grid",
        )

    @pytest.mark.skip(
        reason="SciPy adaptive solvers on discontinuous SMC closed loops can hang for minutes (Zeno-like refinement near sign(s) switches); not for smoke."
    )
    def test_scipy_adaptive_on_smc_not_for_smoke(self):
        """Documented skip — run manually only with a long timeout if needed."""
        _integrate(self.diagram, solver="scipy_stiff", tf=0.5)


from minilink.blocks.basic import Integrator


class TestIntegrateZoh(unittest.TestCase):
    def test_single_step_matches_integrate(self):
        plant = Integrator()
        evaluator = plant.compile()
        x0 = np.array([0.0])
        u_hold = np.array([2.0])
        x_final = evaluator.integrate_zoh(x0, u_hold, t0=0.0, dt_hold=0.1)
        x_seq = evaluator.rk4_integrate_zoh(x0, u_hold.reshape(1, 1), t0=0.0, dt=0.1)
        np.testing.assert_allclose(x_final, x_seq[-1])

    def test_subdivided_dt_inner(self):
        plant = Integrator()
        evaluator = plant.compile()
        x0 = np.array([0.0])
        u_hold = np.array([1.0])
        dt_hold = 0.1
        dt_inner = 0.02
        x_final = evaluator.integrate_zoh(
            x0, u_hold, t0=0.0, dt_hold=dt_hold, dt_inner=dt_inner
        )
        n_sub = int(round(dt_hold / dt_inner))
        u_sequence = np.ones((n_sub, 1))
        dt_step = dt_hold / n_sub
        x_seq = evaluator.rk4_integrate_zoh(x0, u_sequence, t0=0.0, dt=dt_step)
        np.testing.assert_allclose(x_final, x_seq[-1], rtol=1e-10, atol=1e-10)

    def test_integrate_zoh_rollout_grid(self):
        plant = Integrator()
        evaluator = plant.compile()
        x0 = np.array([0.0])
        u_hold = np.array([1.0])
        dt_hold = 0.1
        dt_inner = 0.02
        t_samples, x_samples = evaluator.integrate_zoh_rollout(
            x0, u_hold, t0=0.0, dt_hold=dt_hold, dt_inner=dt_inner
        )
        n_sub = int(round(dt_hold / dt_inner))
        self.assertEqual(t_samples.shape[0], n_sub + 1)
        self.assertEqual(x_samples.shape, (n_sub + 1, 1))
        np.testing.assert_allclose(t_samples[0], 0.0)
        np.testing.assert_allclose(t_samples[-1], dt_hold, rtol=1e-12)
        x_final = evaluator.integrate_zoh(
            x0, u_hold, t0=0.0, dt_hold=dt_hold, dt_inner=dt_inner
        )
        np.testing.assert_allclose(x_samples[-1], x_final)

    def test_invalid_dt_raises(self):
        plant = Integrator()
        evaluator = plant.compile()
        with self.assertRaises(ValueError):
            evaluator.integrate_zoh(np.array([0.0]), np.array([1.0]), 0.0, 0.0)


from minilink.core.compile.evaluators.step_rollout import gather_u
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem, System
from minilink.simulation.computer import Computer, StepSchedule


class KDependentPort(System):
    """Static block whose output depends on step index ``k`` in the third slot."""

    def __init__(self):
        super().__init__()
        self.add_input_port("u", dim=1)
        self.add_output_port("y", dim=1, function=self.compute, dependencies=("u",))

    def compute(self, x, u, t=0, params=None):
        xp = array_module(u)
        k = int(t)
        return xp.array([u[0] + float(k)])


class Accumulator(StepSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def _build_direct_plant():
    """Boundary input wired directly to the plant — no in-tick serial dependency."""
    diagram = StepDiagramSystem()
    diagram.add_subsystem(Accumulator(), "plant")
    diagram.add_input_port("u")
    diagram.connect("input", "u", "plant", "u")
    diagram.connect_new_output_port("plant", "y", "y")
    return diagram


def _build_fast_slow_series():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(Accumulator(), "fast")
    diagram.add_subsystem(Accumulator(), "slow")
    diagram.add_input_port("u")
    diagram.connect("input", "u", "fast", "u")
    diagram.connect("fast", "y", "slow", "u")
    diagram.connect_new_output_port("slow", "y", "y")
    return diagram


def _hand_tick(computer: Computer, u) -> dict[str, np.ndarray]:
    """Reference one-tick update mirroring Computer double-buffer semantics."""
    plan = computer.plan
    k_tick = computer.k
    read = computer.signal_read
    write = computer.signal_write
    write[:] = read
    x_work = computer.x.copy()
    u_arr = np.asarray(u, dtype=float).reshape(computer.diagram.m)
    for sys_id in computer.all_sys_ids:
        if k_tick % computer.divisor(sys_id) != 0:
            continue
        for op in computer.port_ops_by_sys.get(sys_id, ()):
            local_x = x_work[op.local_x_slice]
            local_u = gather_u(op.gather_sources, op.u_dim, read, u_arr)
            write[op.out_slice] = op.compute_func(
                local_x, local_u, k_tick, op.bound_params
            )
        step_op = computer.step_op_by_sys.get(sys_id)
        if step_op is not None:
            local_x = x_work[step_op.local_x_slice]
            local_u = gather_u(step_op.gather_sources, step_op.u_dim, read, u_arr)
            x_work[step_op.local_x_slice] = step_op.step_func(
                local_x, local_u, k_tick, step_op.bound_params
            )
    computer.signal_read, computer.signal_write = (write, read)
    computer.x = x_work
    outs = {
        port_id: computer.signal_read[sl].copy()
        for port_id, sl in plan.external_output_slices.items()
    }
    computer.k += 1
    return outs


class TestComputer(unittest.TestCase):
    def test_single_rate_matches_rollout(self):
        diagram = _build_direct_plant()
        schedule = StepSchedule(dt_base=0.01)
        computer = Computer(diagram, schedule)
        computer.compile()
        computer.reset()
        n_steps = 20
        u_val = 1.0
        xs = [computer.x.copy()]
        for _ in range(n_steps):
            computer.tick(np.array([u_val]))
            xs.append(computer.x.copy())
        rollout = diagram.compute_rollout(n_steps=n_steps, u=np.array([u_val]))
        np.testing.assert_allclose(xs[-1], rollout.x[:, -1], rtol=1e-10, atol=1e-10)
        for i in range(1, n_steps + 1):
            np.testing.assert_allclose(xs[i], rollout.x[:, i], rtol=1e-10, atol=1e-10)

    def test_k_increments_and_passed_to_leaf(self):
        diagram = StepDiagramSystem()
        diagram.add_subsystem(KDependentPort(), "blk")
        diagram.add_input_port("u")
        diagram.connect("input", "u", "blk", "u")
        diagram.connect_new_output_port("blk", "y", "y")
        computer = Computer(diagram, StepSchedule(dt_base=0.01))
        computer.compile()
        computer.reset()
        self.assertEqual(computer.k, 0)
        outs = computer.tick(np.array([2.0]))
        self.assertAlmostEqual(float(outs["y"][0]), 2.0)
        self.assertEqual(computer.k, 1)
        outs = computer.tick(np.array([2.0]))
        self.assertAlmostEqual(float(outs["y"][0]), 3.0)

    def test_multi_rate_hold(self):
        diagram = _build_fast_slow_series()
        schedule = StepSchedule(dt_base=0.01, fire={"fast": 1, "slow": 2})
        computer = Computer(diagram, schedule)
        computer.compile()
        computer.reset()
        ref = Computer(diagram, schedule)
        ref.compile()
        ref.reset()
        n_steps = 12
        u_val = 1.0
        for _ in range(n_steps):
            outs = computer.tick(np.array([u_val]))
            ref_outs = _hand_tick(ref, np.array([u_val]))
            np.testing.assert_allclose(computer.x, ref.x)
            for key in outs:
                np.testing.assert_allclose(outs[key], ref_outs[key])

    def test_from_rates_divisors(self):
        schedule = StepSchedule.from_rates(
            dt_base=0.01, rates_hz={"a": 100.0, "b": 50.0}
        )
        self.assertEqual(schedule.fire["a"], 1)
        self.assertEqual(schedule.fire["b"], 2)
        with self.assertRaises(ValueError):
            StepSchedule.from_rates(dt_base=0.01, rates_hz={"a": 30.0})
        with self.assertRaises(ValueError):
            StepSchedule.from_rates(dt_base=0.01, rates_hz={"a": 200.0})

    def test_rollout_differs_when_scheduled(self):
        diagram = _build_fast_slow_series()
        sync = diagram.compute_rollout(n_steps=6, u=np.array([1.0]))
        schedule = StepSchedule(dt_base=0.01, fire={"fast": 1, "slow": 2})
        computer = Computer(diagram, schedule)
        computer.compile()
        computer.reset()
        for _ in range(6):
            computer.tick(np.array([1.0]))
        self.assertFalse(np.allclose(computer.x, sync.x[:, -1]))

    def test_reset_replay(self):
        diagram = _build_direct_plant()
        computer = Computer(diagram, StepSchedule(dt_base=0.01))
        computer.compile()
        computer.reset()
        n_steps = 10
        for k in range(n_steps):
            computer.tick(np.array([float(k)]))
        x_final = computer.x.copy()
        k_final = computer.k
        computer.reset()
        for k in range(n_steps):
            computer.tick(np.array([float(k)]))
        np.testing.assert_allclose(computer.x, x_final)
        self.assertEqual(computer.k, k_final)

    def test_unknown_sys_id_in_fire_raises(self):
        diagram = _build_direct_plant()
        schedule = StepSchedule(dt_base=0.01, fire={"missing": 2})
        computer = Computer(diagram, schedule)
        with self.assertRaises(ValueError):
            computer.compile()

    def test_tick_before_compile_raises(self):
        diagram = _build_direct_plant()
        computer = Computer(diagram, StepSchedule(dt_base=0.01))
        with self.assertRaises(RuntimeError):
            computer.tick(np.array([1.0]))


from minilink.control.output import ProportionalController
from minilink.simulation.computer import Computer, StepSchedule, as_computer


def _build_step_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(ProportionalController(0.5), "ctl")
    diagram.add_input_port("r")
    diagram.add_input_port("y")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("input", "y", "ctl", "y")
    diagram.connect_new_output_port("ctl", "u", "u")
    return diagram


class TestAsComputer(unittest.TestCase):
    def test_leaf_mod_float(self):
        computer = ProportionalController(0.3) % 0.02
        self.assertIsInstance(computer, Computer)
        self.assertAlmostEqual(computer.schedule.dt_base, 0.02)
        self.assertIn("ctl", computer.diagram.subsystems)

    def test_step_diagram_as_computer(self):
        diagram = _build_step_diagram()
        computer = as_computer(diagram, StepSchedule(dt_base=0.01))
        self.assertIs(diagram, computer.diagram)

    def test_rejects_continuous_plant(self):
        with self.assertRaises(TypeError):
            Integrator() % 0.01

    def test_mpc_mod_exposes_u_ff(self):
        pytest = __import__("pytest")
        jax = pytest.importorskip("jax")
        del jax
        from minilink.control.mpc import ModelPredictiveController
        from minilink.core.backends import configure_jax
        from minilink.core.costs import QuadraticCost
        from minilink.dynamics.catalog.vehicles.jax_vehicles import (
            BicycleDynRate,
        )
        from minilink.planning.problems import PlanningProblem
        from minilink.planning.trajectory_optimization.direct_collocation import (
            DirectCollocationOptions,
            DirectCollocationTranscription,
        )
        from minilink.planning.trajectory_optimization.planner import (
            TrajectoryOptimizationOptions,
            TrajectoryOptimizationPlanner,
        )

        configure_jax(enable_x64=True)
        sys = BicycleDynRate()
        x0 = sys.x0.copy()
        planner = TrajectoryOptimizationPlanner(
            PlanningProblem(
                sys=sys,
                tf=1.0,
                x_start=x0,
                cost=QuadraticCost.from_system(sys, xbar=x0),
            ),
            transcription=DirectCollocationTranscription(
                DirectCollocationOptions(n_steps=5)
            ),
            options=TrajectoryOptimizationOptions(
                compile_backend="jax",
                record_solve_time=True,
                optimizer_method="scipy_slsqp",
                optimizer_options={"maxiter": 5, "ftol": 0.1},
            ),
        )
        computer = (
            ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False) % 0.2
        )
        self.assertIn("y", computer.diagram.inputs)
        self.assertIn("u_ff", computer.diagram.outputs)


from minilink.simulation.realtime import (
    CallbackInput,
    CallbackOutput,
    RealtimeSimulator,
)


class TestRealtimeSimulator(unittest.TestCase):
    def test_headless_run_returns_trajectory(self):
        sys = StableLinearSystem()
        rt_sim = RealtimeSimulator(
            sys,
            frame_dt=0.01,
            sim_dt=0.005,
            renderer=None,
            input=CallbackInput(lambda t, x: np.array([2.0])),
            max_steps=5,
        )
        traj = rt_sim.run()

        # Initial sample + one sample per frame; locked clock → uniform grid.
        self.assertEqual(traj.t.size, 6)
        self.assertEqual(traj.x.shape, (1, 6))
        self.assertEqual(traj.u.shape, (1, 6))
        np.testing.assert_allclose(np.diff(traj.t), 0.01)
        np.testing.assert_allclose(traj.u[:, 1:], 2.0)
        # dx = -x + u with u=2 pulls the state up from x0=1 toward 2.
        self.assertGreater(traj.x[0, -1], traj.x[0, 0])
        self.assertIs(rt_sim.last_traj, traj)

    def test_nominal_input_and_output_publish(self):
        sys = TwoPortLinearSystem()
        published = []
        rt_sim = RealtimeSimulator(
            sys,
            frame_dt=0.01,
            renderer=None,
            output=CallbackOutput(
                lambda t, x, u, outputs: published.append((t, u.copy(), outputs))
            ),
            max_steps=3,
        )
        traj = rt_sim.run()

        # No input attached → nominal port values are held every frame.
        np.testing.assert_allclose(traj.u[:, -1], [1.0, 2.0])
        self.assertEqual(len(published), 3)
        _, u_pub, outputs_pub = published[0]
        np.testing.assert_allclose(u_pub, [1.0, 2.0])
        self.assertIn("y", outputs_pub)

    def test_tf_stops_the_run(self):
        traj = RealtimeSimulator(
            StableLinearSystem(),
            frame_dt=0.01,
            renderer=None,
            tf=0.03,
        ).run()
        self.assertAlmostEqual(traj.t[-1], 0.03)

    def test_auto_compile_backend_prefers_jax_when_available(self):
        rt_sim = RealtimeSimulator(
            StableLinearSystem(),
            frame_dt=0.01,
            renderer=None,
            compile_backend=None,
            max_steps=1,
        )
        if _have_jax():
            self.assertEqual(rt_sim.compile_backend, "jax")
        else:
            self.assertEqual(rt_sim.compile_backend, "numpy")
        self.assertEqual(rt_sim.evaluator.backend, rt_sim.compile_backend)

    def test_realtime_overrun_emits_warning(self):
        import time
        import warnings

        def slow_u(t, x):
            time.sleep(0.03)
            return np.array([0.0])

        rt_sim = RealtimeSimulator(
            StableLinearSystem(),
            frame_dt=0.005,
            renderer=None,
            compile_backend="numpy",
            input=CallbackInput(slow_u),
            max_steps=2,
        )
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            rt_sim.run()
        messages = [
            str(w.message) for w in caught if issubclass(w.category, UserWarning)
        ]
        self.assertTrue(
            any("behind real time" in msg for msg in messages),
            f"expected real-time overrun warning, got {messages!r}",
        )
        self.assertGreater(rt_sim.n_overruns, 0)
