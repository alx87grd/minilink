"""Diagram composition shortcut examples.

Run from the repo root:

    python examples/scripts/diagrams/demo_diagram_shortcuts.py
"""

from minilink.control.pendulum_pd import PendulumPDController
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step, WhiteNoise
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum


def show(diagram, name, operation):
    diagram.name = name
    print(f"{name}: {operation}")
    diagram.plot_diagram()
    return diagram


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
show(diagram, "Shortcut add-only diagram", "step + ctl + plant")


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
show(chain, "Shortcut integrator chain", "Step() >> Integrator() >> Integrator()")
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
show(closed, "Shortcut closed-loop pendulum", "PendulumPDController() @ Pendulum()")
# closed.compute_trajectory(tf=10.0, show=False)


# Python parses this as ``step >> (controller @ plant)``. Shortcut composition
# flattens the closed-loop diagram, so the result has direct ``step``,
# ``controller``, and ``pendulum`` subsystems.
fed_closed = Step(final_value=[1.0]) >> PendulumPDController() @ Pendulum()
show(
    fed_closed,
    "Source into closed-loop shortcut",
    "Step() >> PendulumPDController() @ Pendulum()",
)
# fed_closed.compute_trajectory(tf=10.0, show=False)


# Noise and disturbance ports stay explicit. This keeps ``>>`` from guessing
# between open internal plant inputs such as ``w`` and ``v``.
noisy = Step(final_value=[1.0]) >> PendulumPDController() @ Pendulum()
noise = WhiteNoise()
noisy.add_subsystem(noise, "sensor_noise")
noisy.connect("sensor_noise", "y", "pendulum", "v")
show(
    noisy,
    "Closed loop with explicit sensor noise",
    "Step() >> PendulumPDController() @ Pendulum(), WhiteNoise() -> pendulum.v",
)


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
show(
    auto,
    "Autowired shortcut diagram",
    "(Step() + PendulumPDController() + Pendulum()).autowire(strict=True)",
)
