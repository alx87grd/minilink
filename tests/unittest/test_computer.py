"""Tests for scheduled :class:`~minilink.simulation.computer.Computer` runtime."""

import unittest

import numpy as np

from minilink.core.backends import array_module
from minilink.core.compile.evaluators.numpy_evaluator import _gather_u
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
            local_u = _gather_u(op.gather_sources, op.u_dim, read, u_arr)
            write[op.out_slice] = op.compute_func(
                local_x, local_u, k_tick, op.bound_params
            )
        step_op = computer.step_op_by_sys.get(sys_id)
        if step_op is not None:
            local_x = x_work[step_op.local_x_slice]
            local_u = _gather_u(step_op.gather_sources, step_op.u_dim, read, u_arr)
            x_work[step_op.local_x_slice] = step_op.step_func(
                local_x, local_u, k_tick, step_op.bound_params
            )

    computer.signal_read, computer.signal_write = write, read
    computer.x = x_work
    outs = {
        port_id: computer.signal_read[sl].copy()
        for port_id, sl in plan.external_output_slices.items()
    }
    computer.k += 1
    return outs


class TestComputer(unittest.TestCase):
    def test_single_rate_matches_rollout(self):
        # Direct boundary -> plant wiring: boundary u is external, not a held
        # internal signal, so parallel gather-from-read matches sync rollout.
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
            dt_base=0.01,
            rates_hz={"a": 100.0, "b": 50.0},
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


if __name__ == "__main__":
    unittest.main()
