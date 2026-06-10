import numpy as np

from minilink.core.blocks.sources import Source
from minilink.core.diagram import DiagramSystem

# ============================================================
# Inner diagram
# ============================================================

inner = DiagramSystem()
inner.name = "Inner diagram"

inner_source = Source(1)
inner_source.params["value"] = np.array([4.0])

inner.add_subsystem(inner_source, "inner_source")

# Expose the inner source as an output of the inner diagram
inner.connect_new_output_port("inner_source", "y", "inner_y")


# ============================================================
# Outer diagram
# ============================================================

outer = DiagramSystem()
outer.name = "Outer diagram"

# Add the whole inner diagram as one block
outer.add_subsystem(inner, "nested_diagram")

# Expose the nested source output as an output of the outer diagram
outer.connect_new_output_port("nested_diagram", "inner_y", "outer_y")


# ============================================================
# Visualize
# ============================================================

inner.plot_diagram()
outer.plot_diagram()
