"""Static simulation: a memoryless ``System`` leaf (``n = 0``) sampled on a time grid."""

import warnings

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.simulation.compile_backend import resolve_auto_backend
from minilink.simulation.input_coercion import coerce_forced_input
from minilink.simulation.simulator import COMPILE_BACKEND_AUTO
from minilink.simulation.time_grid import build_time_grid

__all__ = ["StaticSimulator", "COMPILE_BACKEND_AUTO"]


###############################################################################
# Public API
###############################################################################


class StaticSimulator:
    """
    Evaluate static IO blocks on a time grid (no ODE integration).

    Records boundary outputs in ``Trajectory.signals`` via the compiled
    static evaluator's ``outputs`` method.
    """

    def __init__(
        self,
        sys,
        x0=None,
        t_start=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        verbose=False,
        backend=BACKEND_NUMPY,
    ):
        require_static_system(sys)
        if solver is not None:
            warnings.warn(
                "solver is ignored for StaticSimulator (no ODE integration)",
                stacklevel=2,
            )

        # The system and the options
        self.verbose = verbose
        self.sys = sys

        # Outputs are sampled, not integrated: no dt policy, so the automatic
        # grid is the DEFAULT_N_STEPS reporting grid
        self.t, self.dt, self.n_pts = build_time_grid(
            t_start,
            tf,
            n_steps=n_steps,
            dt=dt,
            verbose=verbose,
        )

        # The compiled system, on backend or, for "auto", JAX then NumPy
        self.backend, self.evaluator = self.resolve_and_build_evaluator(sys, backend)

    def solve(self):
        """Sample the outputs on the grid under the nominal input."""
        u_traj = self.nominal_u_traj()

        return self.build_trajectory(u_traj)

    def solve_forced(self, u, input_port_id=None):
        """Sample the outputs on the grid under a prescribed input, as ``Simulator.solve_forced``."""
        u_traj = coerce_forced_input(self.sys, self.t, u, input_port_id=input_port_id)

        return self.build_trajectory(u_traj)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_and_build_evaluator(self, sys, backend):
        """The backend and its evaluator: ``backend``, or JAX then NumPy for ``"auto"``."""
        return resolve_auto_backend(
            lambda backend: sys.compile(backend=backend), backend
        )

    def nominal_u_traj(self):
        """The nominal input held over the grid, ``(m, n_pts)``."""
        m = self.sys.m
        n_pts = self.n_pts

        u_traj = np.zeros((m, n_pts))
        if m > 0:
            u_bar = self.evaluator._u_nominal
            u_traj[:, :] = np.asarray(u_bar).reshape(m, 1)

        return u_traj

    def build_trajectory(self, u_traj):
        """Evaluate the boundary outputs at every sample and record them in ``signals``."""
        evaluator = self.evaluator
        times = self.t
        n_pts = self.n_pts
        m = self.sys.m

        x_traj = np.zeros((0, n_pts))
        signals = {}
        empty_x = np.array([])

        for i, ti in enumerate(times):
            # y = h(u, t) at the sample
            u_i = u_traj[:, i] if m > 0 else np.array([])
            out = evaluator.outputs(empty_x, u_i, float(ti))

            # Each output channel, one column per sample; x and u are the
            # trajectory's own channels, as in the hybrid simulator
            for key, value in out.items():
                if key in {"x", "u"}:
                    continue
                arr = np.asarray(value, dtype=float).reshape(-1)
                if key not in signals:
                    signals[key] = np.zeros((arr.size, n_pts))
                signals[key][:, i] = arr

        return Trajectory(t=times, x=x_traj, u=u_traj, signals=signals)


###############################################################################
# Internal machinery
###############################################################################


def require_static_system(sys):
    """Refuse a ``DynamicSystem``, and any system with a state."""
    if isinstance(sys, DynamicSystem):
        raise TypeError(
            "DynamicSystem (including DiagramSystem) uses Simulator via "
            "compute_trajectory, or construct Simulator directly."
        )
    if sys.n != 0:
        raise TypeError(f"StaticSimulator requires n=0 static System, got n={sys.n}")
