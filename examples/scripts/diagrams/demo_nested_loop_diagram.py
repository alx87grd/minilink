import numpy as np

from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem

# Custom blocks


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.name = "Integrator"

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


class PropController(StaticSystem):
    def __init__(self):
        super().__init__()

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.add_input_port("r", nominal_value=0.0)
        self.add_input_port("y", dim=1, nominal_value=np.array([0.0]))

        self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        r, y = self.get_port_values_from_u(u, "r", "y")
        Kp = params["Kp"]

        u_cmd = Kp * (r[0] - y[0])
        return np.array([u_cmd])


# Custom diagram

# Plant system
sys1 = Integrator()
sys1.state.labels = ["v"]
sys1.x0[0] = 20.0
sys2 = Integrator()
sys2.state.labels = ["x"]
sys2.x0[0] = 20.0

# Controllers
ctl1 = PropController()
ctl1.params["Kp"] = 1.0
ctl2 = PropController()
ctl2.params["Kp"] = 1.0

# Source input
step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([20.0])
step.params["step_time"] = 10.0

# Diagram
diagram = DiagramSystem()

diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl1, "controller1")
diagram.add_subsystem(ctl2, "controller2")
diagram.add_subsystem(sys1, "integrator1")
diagram.add_subsystem(sys2, "integrator2")

diagram.connect("integrator1", "y", "integrator2", "u")
diagram.connect("controller2", "u", "integrator1", "u")
diagram.connect("integrator1", "y", "controller2", "y")
diagram.connect("controller1", "u", "controller2", "r")
diagram.connect("integrator2", "y", "controller1", "y")
diagram.connect("step", "y", "controller1", "r")

diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()
# diagram.animate() # No geometry defined for the blocks
