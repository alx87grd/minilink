"""README-tier impedance pendulum + mass–spring–damper walkthrough.

Run from repo root::

    python examples/scripts/plots/demo_readme.py
"""

from minilink.control.impedance import ImpedanceController
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


def run_smoke(*, tf: float = 1.0, show: bool = False) -> None:
    """Headless L6 smoke: impedance pendulum only."""
    controller = ImpedanceController()
    plant = Pendulum()
    plant.x0[0] = 2.0
    plant.params["l"] = 5.0
    plant.params["m"] = 1.0
    diagram = controller @ plant
    diagram.compute_trajectory(tf=tf, show=show)
    diagram.plot_trajectory(show=show)


def _run_impedance_pendulum(*, tf: float = 10.0, show: bool = True) -> None:
    controller = ImpedanceController()  # u = Kp * (r - theta) - Kd * theta_dot
    plant = Pendulum()  # theta_ddot = -(g / l) * sin(theta) + tau / (m * l**2)

    plant.x0[0] = 2.0
    plant.params["l"] = 5.0
    plant.params["m"] = 1.0

    diagram = controller @ plant
    diagram.compute_trajectory(tf=tf, show=show)
    diagram.plot_diagram()
    diagram.plot_trajectory(show=show)
    diagram.animate(show=show)


def _run_mass_spring_damper(*, show: bool = True) -> None:
    import numpy as np

    from minilink.blocks.sources import Step
    from minilink.core.system import DynamicSystem

    class MassSpringDamper(DynamicSystem):
        def __init__(self):
            super().__init__(n=2, input_dim=1, expose_state=True)
            self.params = {"m": 1.0, "k": 4.0, "c": 0.3}

        def f(self, x, u, t=0, params=None):
            params = self.params if params is None else params
            m, k, c = params["m"], params["k"], params["c"]

            position, velocity = x
            force = u[0]

            dx = np.zeros(2)
            dx[0] = velocity
            dx[1] = (force - c * velocity - k * position) / m
            return dx

    sys = MassSpringDamper()
    sys.x0[0] = 1.0  # released from rest at position 1
    sys.plot_diagram()
    sys.compute_trajectory(tf=20.0, show=show)
    sys.plot_trajectory(show=show)

    step = Step()
    step.params["final_value"] = np.array([10.0])
    step.params["step_time"] = 2.0

    diagram = step >> sys
    diagram.plot_diagram()
    diagram.compute_trajectory(tf=20.0, show=show)
    diagram.plot_trajectory(signals=("x", (step, "y")), show=show)


def main() -> None:
    _run_impedance_pendulum()
    _run_mass_spring_damper()


if __name__ == "__main__":
    main()
