"""Cascade control wired by hand: an inner velocity loop inside an outer position loop."""

from minilink import DiagramSystem, Integrator, ProportionalController, Step

# Plant: force -> velocity -> position, two integrators in series
velocity = Integrator()
velocity.state.labels = ["v"]
velocity.x0[0] = 20.0
position = Integrator()
position.state.labels = ["x"]
position.x0[0] = 20.0

# One proportional controller per loop
outer = ProportionalController(1.0)  # position error -> velocity reference
inner = ProportionalController(1.0)  # velocity error -> force

step = Step(final_value=20.0, step_time=10.0)

diagram = DiagramSystem()
diagram.add_subsystem(step, "step")
diagram.add_subsystem(outer, "outer")
diagram.add_subsystem(inner, "inner")
diagram.add_subsystem(velocity, "velocity")
diagram.add_subsystem(position, "position")

diagram.connect("step", "y", "outer", "r")
diagram.connect("position", "y", "outer", "y")
diagram.connect("outer", "u", "inner", "r")
diagram.connect("velocity", "y", "inner", "y")
diagram.connect("inner", "u", "velocity", "u")
diagram.connect("velocity", "y", "position", "u")

diagram.plot_diagram()
diagram.compute_trajectory(tf=20)
diagram.plot_trajectory()
# diagram.animate() # No geometry defined for the blocks
