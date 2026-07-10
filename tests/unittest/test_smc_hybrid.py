"""Hybrid SMC on a pendulum: HybridSimulator vs hand-rolled ZOH loop."""

import unittest

import numpy as np

from minilink.control.modelbased import SlidingModeController
from minilink.core.hybrid_composition import hybrid_closed_loop
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.hybrid_simulator import HybridSimulator


def _build_hybrid(*, ts=0.05):
    plant = Pendulum(length=1.0, mass=1.0)
    plant.x0 = np.array([2.5, -0.2])
    ctl = SlidingModeController(plant, lam=2.0, gain=6.0, nab=0.12)
    return hybrid_closed_loop(
        ctl,
        plant,
        schedule=ts,
        computer_in="y",
        plant_out="y",
    )


class TestSmcHybrid(unittest.TestCase):
    def test_hand_loop_matches_hybrid_simulator(self):
        hybrid = _build_hybrid(ts=0.05)
        dt_base = hybrid.computer.schedule.dt_base
        plant_eval = hybrid.plant.compile()
        computer = hybrid.computer
        computer.compile()
        computer.reset()

        diagram = hybrid.computer.diagram
        slice_r = diagram.get_input_port_slice("r")
        slice_y = diagram.get_input_port_slice("y")
        ref = np.array([0.0, 0.0])

        x_plant = hybrid.plant.x0.copy()
        u_nom = hybrid.plant.get_u_from_input_ports()
        y_sample = plant_eval.outputs(x_plant, u_nom, 0.0)["y"]

        n_steps = 12
        q_hand = []
        for _ in range(n_steps):
            k = computer.k
            u_computer = diagram.get_u_from_input_ports().copy()
            u_computer[slice_r] = ref
            u_computer[slice_y] = y_sample
            outs = computer.tick(u_computer)
            u_plant = np.asarray(outs["u"], dtype=float).reshape(-1)
            x_plant = plant_eval.integrate_zoh(
                x_plant,
                u_plant,
                k * dt_base,
                dt_base,
            )
            y_sample = plant_eval.outputs(
                x_plant,
                u_plant,
                (k + 1) * dt_base,
            )["y"]
            q_hand.append(float(y_sample[0]))

        sim = HybridSimulator(
            hybrid,
            t0=0.0,
            n_steps=n_steps,
            plant_dt_inner=0.002,
        )
        result = sim.solve_forced(ref, input_port_id="r")
        np.testing.assert_allclose(
            result.computer.signals["y"][0, :n_steps],
            q_hand,
            rtol=1e-5,
            atol=1e-5,
        )


if __name__ == "__main__":
    unittest.main()
