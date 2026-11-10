"""Static simulation: a memoryless ``System`` leaf (``n = 0``) sampled on a time grid."""

import warnings

import numpy as np

from minilink.core.backends import BACKEND_JAX, BACKEND_NUMPY
from minilink.core.compile.compiler import build_on_auto
from minilink.core.compile.evaluators.compiled import body_of
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
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
        tf=10,
        *,
        x0=None,
        t0=0,
        n_samples=None,
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
        # grid is the DEFAULT_N_SAMPLES reporting grid
        self.t, self.dt, self.n_samples = build_time_grid(
            t0,
            tf,
            n_samples=n_samples,
            dt=dt,
            verbose=verbose,
        )

        # The compiled system, on backend or, for "auto", JAX then NumPy
        self.backend, self.evaluator = self.resolve_and_build_evaluator(sys, backend)

    def solve(self, *, u=None, port=None):
        """Sample the outputs on the grid: under the nominal input, or ``u`` prescribed as for ``Simulator.solve``."""
        if u is None:
            u_traj = self.nominal_u_traj()
        else:
            u_traj = coerce_forced_input(self.sys, self.t, u, port=port)

        return self.build_trajectory(u_traj)

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def resolve_and_build_evaluator(self, sys, backend):
        """The backend and its evaluator: ``backend``, or JAX then NumPy for ``"auto"``."""
        return build_on_auto(lambda backend: sys.compile(backend=backend), backend)

    def nominal_u_traj(self):
        """The nominal input held over the grid, ``(m, n_samples)``."""
        m = self.sys.m
        n_samples = self.n_samples

        u_traj = np.zeros((m, n_samples))
        if m > 0:
            u_bar = self.evaluator._u_nominal
            u_traj[:, :] = np.asarray(u_bar).reshape(m, 1)

        return u_traj

    def build_trajectory(self, u_traj):
        """Evaluate the boundary outputs at every sample and record them in ``signals``."""
        times = self.t
        n_samples = self.n_samples

        # y = h(u, t) at every sample, one channel a row
        outputs = outputs_on_grid(self.evaluator, self.sys, u_traj, times)

        # The output channels; x and u are the trajectory's own channels, as in
        # the hybrid simulator
        signals = {
            key: values for key, values in outputs.items() if key not in {"x", "u"}
        }

        return Trajectory(
            t=times, x=np.zeros((0, n_samples)), u=u_traj, signals=signals
        )


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


def outputs_on_grid(evaluator, sys, u_traj, times):
    """``{port: (dim, n_samples)}``, the outputs at every sample: one ``vmap`` of the jitted body on JAX, a loop on NumPy."""
    m = sys.m
    n_samples = times.shape[0]
    empty_x = np.array([])

    # NumPy: y = h(u_k, t_k) sample by sample
    if evaluator.backend != BACKEND_JAX:
        columns = {}
        for i, ti in enumerate(times):
            u_i = u_traj[:, i] if m > 0 else empty_x
            for key, value in evaluator.outputs(empty_x, u_i, float(ti)).items():
                arr = np.asarray(value, dtype=float).reshape(-1)
                if key not in columns:
                    columns[key] = np.zeros((arr.size, n_samples))
                columns[key][:, i] = arr
        return columns

    # JAX: the whole grid in one dispatch, the jitted map kept on the evaluator
    forms = evaluator.compiled_forms
    if "outputs_on_grid" not in forms:
        jax, jnp = evaluator.jax, evaluator.jnp
        outputs = body_of(evaluator, "outputs")
        forms["outputs_on_grid"] = jax.jit(
            jax.vmap(lambda u, t: outputs(jnp.zeros(0), u, t, None))
        )
    grid = forms["outputs_on_grid"](np.asarray(u_traj, dtype=float).T, times)

    # The ports in their declared order, one channel a row
    return {
        key: np.asarray(grid[key], dtype=float).reshape(n_samples, -1).T
        for key in sys.outputs
    }
