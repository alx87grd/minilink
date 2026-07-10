"""Time-domain simulation for static ``System`` leaves (``n=0``)."""

import warnings

import numpy as np

from minilink.core.backends import (
    BACKEND_AUTO,
    BACKEND_JAX,
    BACKEND_NUMPY,
)
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
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
        self.times = self.t
        self.compile_backend, self.evaluator = self._resolve_and_build_evaluator(
            sys, compile_backend
        )

    def solve(self):
        u_traj = self._nominal_u_traj()
        return self._build_trajectory(u_traj)

    def solve_forced(self, u, input_port_id=None):
        u_traj = self._coerce_forced_input(u, input_port_id=input_port_id)
        return self._build_trajectory(u_traj)

    def _build_trajectory(self, u_traj):
        n_pts = self.n_pts
        m = self.sys.m
        x_traj = np.zeros((0, n_pts))

        signals = {}
        empty_x = np.array([])
        for i, ti in enumerate(self.times):
            u_i = u_traj[:, i] if m > 0 else np.array([])
            out = self.evaluator.outputs(empty_x, u_i, float(ti))
            for key, value in out.items():
                arr = np.asarray(value, dtype=float).reshape(-1)
                if key not in signals:
                    signals[key] = np.zeros((arr.size, n_pts))
                signals[key][:, i] = arr

        return Trajectory(t=self.times, x=x_traj, u=u_traj, signals=signals)

    def _nominal_u_traj(self):
        m = self.sys.m
        u_traj = np.zeros((m, self.n_pts))
        if m > 0:
            u_bar = self.evaluator._u_nominal
            u_traj[:, :] = np.asarray(u_bar).reshape(m, 1)
        return u_traj

    def _resolve_and_build_evaluator(self, sys, compile_backend):
        if compile_backend != BACKEND_AUTO:
            return compile_backend, sys.compile(backend=compile_backend)

        try:
            import jax  # noqa: F401
        except ImportError:
            return BACKEND_NUMPY, sys.compile(backend=BACKEND_NUMPY)
        try:
            return BACKEND_JAX, sys.compile(backend=BACKEND_JAX)
        except Exception:
            return BACKEND_NUMPY, sys.compile(backend=BACKEND_NUMPY)

    def _coerce_forced_input(self, u, input_port_id=None):
        if input_port_id is None:
            return self._coerce_forced_signal(u, self.sys.m, "u")

        port_slice = self.sys.get_input_port_slice(input_port_id)
        u_nominal = self.sys.get_u_from_input_ports().reshape(self.sys.m, 1)
        u_traj = np.repeat(u_nominal, self.n_pts, axis=1)
        u_traj[port_slice, :] = self._coerce_forced_signal(
            u,
            self.sys.inputs[input_port_id].dim,
            f"u for input port '{input_port_id}'",
        )
        return self._validate_forced_u_traj(u_traj)

    def _coerce_forced_signal(self, data, expected_dim, label):
        if callable(data):
            return self._sample_forced_callable(data, expected_dim)
        return self._coerce_forced_array(data, expected_dim, label)

    def _sample_forced_callable(self, fn, expected_dim):
        samples = np.zeros((expected_dim, self.n_pts), dtype=float)
        for i, ti in enumerate(self.times):
            value = np.asarray(fn(float(ti)), dtype=float)
            if expected_dim == 1 and value.ndim == 0:
                samples[0, i] = float(value)
            else:
                samples[:, i] = value.reshape(expected_dim)
        return samples

    def _coerce_forced_array(self, data, expected_dim, label):
        arr = np.asarray(data, dtype=float)
        expected_shape = (expected_dim, self.n_pts)

        if arr.ndim == 0:
            if expected_dim != 1:
                raise ValueError(f"{label} must have shape {expected_shape}")
            return np.full((1, self.n_pts), float(arr), dtype=float)

        if arr.ndim == 1:
            if expected_dim == 1 and arr.shape[0] == self.n_pts:
                return arr.reshape(1, self.n_pts)
            if arr.shape[0] == expected_dim:
                return np.repeat(arr.reshape(expected_dim, 1), self.n_pts, axis=1)
            raise ValueError(f"{label} must have shape {expected_shape}")

        if arr.ndim == 2 and arr.shape == expected_shape:
            return arr

        raise ValueError(f"{label} must have shape {expected_shape}")

    def _validate_forced_u_traj(self, u_traj):
        u = np.asarray(u_traj, dtype=float)
        expected_shape = (self.sys.m, self.n_pts)
        if u.shape != expected_shape:
            raise ValueError(f"u_traj must have shape {expected_shape}")
        return u


__all__ = ["StaticSimulator", "COMPILE_BACKEND_AUTO"]
