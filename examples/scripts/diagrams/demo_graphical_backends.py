"""Basic graphical backend options.

Run from the repo root:

    python examples/scripts/diagrams/demo_graphical_backends.py
"""

import numpy as np

from minilink.core.blocks.basic import Integrator
from minilink.core.diagram import DiagramSystem
from minilink.graphical.diagram_export import export_diagram_topology


diagram = DiagramSystem()
diagram.graphe_building_verbose = False
diagram.name = "Graphical backend demo"
diagram.add_subsystem(Integrator(), "plant")
diagram.add_input_port(1, "u_cmd", nominal_value=np.array([1.0]))
diagram.connect("input", "u_cmd", "plant", "u")
diagram.connect_new_output_port("plant", "y", "y")

traj = diagram.compute_trajectory(
    tf=5.0,
    dt=0.05,
    solver="euler",
    show=False,
    verbose=True,
)

diagram.plot_trajectory()

diagram.plot_trajectory(
    signals=("x",),
    backend="matplotlib",
    show=True,
)

diagram.plot_trajectory(
    signals=("plant:y",),
    backend="matplotlib",
    show=True,
)
diagram.plot_trajectory(
    signals=("x", "u", "plant:y"),
    backend="plotly",
    show=True,
)
