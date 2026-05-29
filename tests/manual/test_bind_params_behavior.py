"""
Manual demo: ``bind_params=False`` (live ``params``) vs ``bind_params=True`` (snapshot at compile).

Usage from repo root::

    PYTHONPATH=. python tests/manual/test_bind_params_behavior.py

After compile, we change the integrator gain ``plant.params['k']``. The default
evaluator picks up the new gain without recompiling; the bound evaluator keeps
the old gain until you call ``compile()`` again.
"""

import numpy as np

from minilink.compile.compiler import compile_diagram
from minilink.core.blocks.basic import Integrator, PropController
from minilink.core.diagram import DiagramSystem

diag = DiagramSystem()
diag.graphe_building_verbose = False

ctl = PropController()
plant = Integrator()
ctl.params["Kp"] = 2.5

diag.add_subsystem(ctl, "ctl")
diag.add_subsystem(plant, "plant")
diag.add_input_port("r", dim=1)

diag.connect("input", "r", "ctl", "r")
diag.connect("plant", "y", "ctl", "y")
diag.connect("ctl", "u", "plant", "u")
# diag.plot_graphe()


x = np.array([0.3])
u = np.array([1.0])
t = 0.0

ev = compile_diagram(diag, bind_params=False)
# ev = compile_diagram(diag, bind_params=True)
# ctl.params["Kp"] = 100.0
dx_a = ev.f(x, u, t)
print(f"f: {dx_a.ravel()}")
