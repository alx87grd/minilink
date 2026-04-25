"""Manual check for the `native=True` animation path.

Runs each combination of ``renderer in {"matplotlib", "meshcat"}`` and
``native in {False, True}`` on a pendulum trajectory so the legacy Python-loop
path and the new backend-native path can be eyeballed side-by-side.

The matplotlib native path uses ``matplotlib.animation.FuncAnimation``; the
meshcat native path builds a ``meshcat.animation.Animation`` and plays it in
the browser via ``Visualizer.set_animation``. ``TorqueArrow`` on the pendulum
is frozen at ``t=0`` in the meshcat native path by design (see DESIGN.md §4.7).
"""

import numpy as np

from minilink.blocks.examples import Pendulum
from minilink.simulation.simulator import Simulator


sys = Pendulum()
sys.x0[0] = 2.0

sim = Simulator(sys, t0=0.0, tf=8.0)
traj = sim.solve()


print("\n--- matplotlib, native=False (legacy Python-loop path) ---")
sys.animate(traj=traj, renderer="matplotlib", native=False)

print("\n--- matplotlib, native=True (FuncAnimation) ---")
sys.animate(traj=traj, renderer="matplotlib", native=True)


print("\n--- meshcat, native=False (legacy Python-loop path) ---")
try:
    sys.animate(traj=traj, renderer="meshcat", native=False)
except ImportError as e:
    print(f"Skipping meshcat: {e}")

print("\n--- meshcat, native=True (Animation + set_animation) ---")
try:
    sys.animate(traj=traj, renderer="meshcat", native=True)
except ImportError as e:
    print(f"Skipping meshcat: {e}")


print("\n--- meshcat, html=True (inline static_html iframe) ---")
try:
    html_obj = sys.animate(traj=traj, renderer="meshcat", html=True)
    print(f"Returned object type: {type(html_obj).__name__}")
except ImportError as e:
    print(f"Skipping meshcat html: {e}")
