"""Compare ``native=True`` vs ``native=False`` animation backends (headless smoke).

Matplotlib runs with ``show=False``. Meshcat is skipped when not installed.
"""

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.simulator import Simulator

sys = Pendulum()
sys.x0[0] = 2.0
traj = Simulator(sys, t0=0.0, tf=2.0).solve()

sys.animate(traj=traj, renderer="matplotlib", native=False, show=False)
sys.animate(traj=traj, renderer="matplotlib", native=True, show=False)

try:
    sys.animate(traj=traj, renderer="meshcat", native=False, show=False)
    sys.animate(traj=traj, renderer="meshcat", native=True, show=False)
except ImportError:
    pass
