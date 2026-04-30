import numpy as np

from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem, StaticSystem

# Custom blocks


class Integrator(DynamicSystem):
    def __init__(self):
        super().__init__(1, 1, 1)
        self.name = "Integrator"
        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=[])

    def f(self, x, u, t=0, params=None):
        return np.array([u[0]])

    def h(self, x, u, t=0, params=None):
        return np.array([x[0]])


class PropController(StaticSystem):
    def __init__(self):
        super().__init__(2, 1)

        self.params = {
            "Kp": 10.0,
        }

        self.name = "Controller"

        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(1, "y", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])

    def ctl(self, x, u, t=0, params=None):
        params = self.params if params is None else params

        r = u[0]
        y = u[1]
        Kp = params["Kp"]

        u_cmd = Kp * (r - y)
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
diagram.connect("controller1", "u", "controller2", "ref")
diagram.connect("integrator2", "y", "controller1", "y")
diagram.connect("step", "y", "controller1", "ref")

diagram.plot_graphe()
diagram.compute_trajectory(tf=20)
# diagram.animate() # No geometry defined for the blocks
