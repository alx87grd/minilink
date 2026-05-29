"""Diagram composition shortcut examples.

Run from the repo root:

    python examples/scripts/diagrams/demo_diagram_shortcuts.py
"""

from minilink.control.pendulum_pd import PendulumPDController
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


# Equivalent explicit form:
#
# diagram = DiagramSystem()
# diagram.add_subsystem(step, "step")
# diagram.add_subsystem(ctl, "controller")
# diagram.add_subsystem(plant, "pendulum")
step = Step(final_value=[1.0])
ctl = PendulumPDController()
plant = Pendulum()


diagram = step + ctl + plant
diagram.name = "Shortcut add-only diagram"
diagram.plot_diagram()


# Equivalent explicit form:
#
# chain = DiagramSystem()
# chain.add_subsystem(step, "step")
# chain.add_subsystem(integrator1, "integrator")
# chain.add_subsystem(integrator2, "integrator_2")
# chain.connect("step", "y", "integrator", "u")
# chain.connect("integrator", "y", "integrator_2", "u")
# chain.connect_new_output_port("integrator_2", "y", "y")

chain = Step() >> Integrator() >> Integrator()
chain.name = "Shortcut integrator chain"
chain.plot_diagram()
# chain.compute_trajectory(tf=5.0, show=False)


# Equivalent explicit form:
#
# closed = DiagramSystem()
# closed.add_subsystem(ctl, "controller")
# closed.add_subsystem(plant, "pendulum")
# closed.add_input_port("r", dim=1)
# closed.connect("input", "r", "controller", "r")
# closed.connect("pendulum", "y", "controller", "y")
# closed.connect("controller", "u", "pendulum", "u")
# closed.connect_new_output_port("pendulum", "y", "y")
closed = PendulumPDController() @ Pendulum()
closed.name = "Shortcut closed-loop pendulum"
closed.plot_diagram()
# closed.compute_trajectory(tf=10.0, show=False)


# Equivalent explicit form:
#
# auto = DiagramSystem()
# auto.add_subsystem(step, "step")
# auto.add_subsystem(ctl, "controller")
# auto.add_subsystem(plant, "pendulum")
# auto.connect("step", "y", "controller", "r")
# auto.connect("pendulum", "y", "controller", "y")
# auto.connect("controller", "u", "pendulum", "u")
auto = (Step() + PendulumPDController() + Pendulum()).autowire(strict=True)
auto.name = "Autowired shortcut diagram"
auto.plot_diagram()
