"""Compare ``native=True`` vs ``native=False`` animation backends (headless smoke).

Matplotlib runs with ``show=False``. Meshcat is skipped when not installed.

Run from repo root::

    python examples/scripts/animation/demo_native_comparison.py
"""

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.simulation.simulator import Simulator


def run_smoke(*, tf: float = 0.5, show: bool = False) -> None:
    """Headless L6 smoke: matplotlib native vs non-native one frame each."""
    sys = Pendulum()
    sys.x0[0] = 2.0
    traj = Simulator(sys, t0=0.0, tf=tf).solve()

    sys.animate(traj=traj, renderer="matplotlib", native=False, show=show)
    sys.animate(traj=traj, renderer="matplotlib", native=True, show=show)

    try:
        sys.animate(traj=traj, renderer="meshcat", native=False, show=show)
        sys.animate(traj=traj, renderer="meshcat", native=True, show=show)
    except ImportError:
        pass


def main() -> None:
    run_smoke(tf=2.0, show=False)


if __name__ == "__main__":
    main()
