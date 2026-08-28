"""Time-domain simulation for static ``System`` leaves (``n=0``)."""

import warnings

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.simulation.compile_backend import resolve_auto_backend
from minilink.simulation.input_coercion import coerce_forced_input
from minilink.simulation.simulator import COMPILE_BACKEND_AUTO
from minilink.simulation.time_grid import build_time_grid


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
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        verbose=False,
        compile_backend=BACKEND_NUMPY,
    ):
        if isinstance(sys, DynamicSystem):
            raise TypeError(
                "DynamicSystem (including DiagramSystem) uses Simulator via "
                "compute_trajectory, or construct Simulator directly."
            )
        if sys.n != 0:
            raise TypeError(
                f"StaticSimulator requires n=0 static System, got n={sys.n}"
            )
        if solver is not None:
            warnings.warn(
                "solver is ignored for StaticSimulator (no ODE integration)",
                stacklevel=2,
            )

        self.verbose = verbose
        self.sys = sys
        self.sys.refresh()

        default_dt = sys.solver_info.get("smallest_time_constant", 0.001) * 0.1
        self.t, self.dt, self.n_pts = build_time_grid(
            t0,
            tf,
            n_steps=n_steps,
            dt=dt,
            default_dt=default_dt,
            verbose=verbose,
        )
        self.compile_backend, self.evaluator = self._resolve_and_build_evaluator(
            sys, compile_backend
        )

    def solve(self):
        u_traj = self._nominal_u_traj()
        return self._build_trajectory(u_traj)

    def solve_forced(self, u, input_port_id=None):
        u_traj = coerce_forced_input(self.sys, self.t, u, input_port_id=input_port_id)
        return self._build_trajectory(u_traj)

    def _build_trajectory(self, u_traj):
        n_pts = self.n_pts
        m = self.sys.m
        x_traj = np.zeros((0, n_pts))

        signals = {}
        empty_x = np.array([])
        for i, ti in enumerate(self.t):
            u_i = u_traj[:, i] if m > 0 else np.array([])
            out = self.evaluator.outputs(empty_x, u_i, float(ti))
            for key, value in out.items():
                arr = np.asarray(value, dtype=float).reshape(-1)
                if key not in signals:
                    signals[key] = np.zeros((arr.size, n_pts))
                signals[key][:, i] = arr

        return Trajectory(t=self.t, x=x_traj, u=u_traj, signals=signals)

    def _nominal_u_traj(self):
        m = self.sys.m
        u_traj = np.zeros((m, self.n_pts))
        if m > 0:
            u_bar = self.evaluator._u_nominal
            u_traj[:, :] = np.asarray(u_bar).reshape(m, 1)
        return u_traj

    def _resolve_and_build_evaluator(self, sys, compile_backend):
        return resolve_auto_backend(
            lambda backend: sys.compile(backend=backend), compile_backend
        )


__all__ = ["StaticSimulator", "COMPILE_BACKEND_AUTO"]
