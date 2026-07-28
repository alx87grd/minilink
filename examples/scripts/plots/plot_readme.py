########################################################
from minilink import ImpedanceController, Pendulum

controller = ImpedanceController()  # u = Kp * (r - theta) - Kd * theta_dot
plant = Pendulum()  # theta_ddot = -(g / l) * sin(theta) + tau / (m * l**2)

plant.x0[0] = 2.0
plant.params["l"] = 5.0
plant.params["m"] = 1.0

diagram = controller @ plant
diagram.compute_trajectory(tf=10.0)
diagram.plot_diagram()
diagram.plot_trajectory()
diagram.animate()


########################################################
import numpy as np  # noqa: E402

from minilink.blocks.sources import Step  # noqa: E402
from minilink.core.system import DynamicSystem  # noqa: E402


class MassSpringDamper(DynamicSystem):
    """m * pddot + c * pdot + k * p = F"""

    def __init__(self):
        super().__init__(n=2, input_dim=1, expose_state=True)

    def f(self, x, u, t=0, params=None):
        # m * pddot + c * pdot + k * p = F
        m = 1.0
        k = 4.0
        c = 0.3

        p, pdot = x
        F = u[0]

        pddot = (F - c * pdot - k * p) / m
        dx = np.array([pdot, pddot])
        return dx


# Standalone system
sys = MassSpringDamper()
sys.x0[0] = 1.0  # released from rest at position 1
sys.plot_diagram()
sys.compute_trajectory(tf=20.0)
sys.plot_trajectory()

# Diagram
step = Step()
step.params["final_value"] = np.array([10.0])
step.params["step_time"] = 2.0

diagram = step >> sys
diagram.plot_diagram()
diagram.compute_trajectory(tf=20.0)
diagram.plot_trajectory(signals=("x", (step, "y")))
