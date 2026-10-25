"""Nested diagram vs :func:`~minilink.core.assembly.flatten_diagram` (before / after)."""

from minilink import DiagramSystem, Gain, Integrator, P
from minilink.core.assembly import flatten_diagram

# A small loop nested inside an outer gain
inner = DiagramSystem()
inner.add_subsystem(P(2.0, ports="reference"), "controller")
inner.add_subsystem(Integrator(), "plant")
inner.add_input_port("r")
inner.connect("input", "r", "controller", "r")
inner.connect("plant", "y", "controller", "y")
inner.connect("controller", "u", "plant", "u")
inner.connect_new_output_port("plant", "y", "y")

nested = DiagramSystem()
nested.add_subsystem(Gain([-1.0]), "gain")
nested.add_subsystem(inner, "loop")
nested.connect("loop", "y", "gain", "u")
nested.connect("gain", "y", "loop", "r")

flat, report = flatten_diagram(nested)


# nested.plot_diagram(expand=False)
nested.plot_diagram(expand=True, show_pdf=True)
flat.plot_diagram(expand=False, show_pdf=True)
# flat.plot_diagram(expand=True, show_pdf=True)
