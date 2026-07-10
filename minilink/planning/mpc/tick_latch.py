"""Per-tick MPC solve latch shared by algebraic and step MPC blocks."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from minilink.core.trajectory import Trajectory
from minilink.planning.mpc.planner import MPCPlanner
from minilink.planning.mpc.warm_start import mpc_warm_start_guess


@dataclass
class MPCTickSolve:
    """One MPC fire: reconstructed plan and feedforward slices."""

    plan: Trajectory
    z: np.ndarray
    u_ff: np.ndarray
    x_ff: np.ndarray


class MPCTickLatch:
    """
    Memoize ``planner.step`` once per integer tick ``k``.

    Port ``compute`` paths on MPC blocks call :meth:`solve_for_tick`; the first
    call at a new ``k`` runs the NLP, later calls at the same ``k`` read the latch.
    """

    def __init__(
        self,
        planner: MPCPlanner,
        *,
        step_disp: bool = False,
        dt_mpc: float | None = None,
        t0: float = 0.0,
    ) -> None:
        self._planner = planner
        self._step_disp = bool(step_disp)
        self._dt_mpc = None if dt_mpc is None else float(dt_mpc)
        self._t0 = float(t0)
        self._latch_k: int | None = None
        self._latch: MPCTickSolve | None = None

    def solve_for_tick(
        self,
        k,
        y,
        *,
        z_warm=None,
        dt_mpc=None,
        initial_guess=None,
    ) -> MPCTickSolve:
        k_int = int(k)
        if self._latch_k == k_int and self._latch is not None:
            return self._latch

        y_arr = np.asarray(y, dtype=float).reshape(-1)
        guess = initial_guess
        if guess is None and z_warm is not None:
            if dt_mpc is None:
                raise ValueError("dt_mpc is required when z_warm is provided")
            guess = mpc_warm_start_guess(
                z_warm,
                y_arr,
                self._planner,
                dt_mpc=float(dt_mpc),
                k=k_int,
            )

        plan = self._planner.step(y_arr, initial_guess=guess)
        result = self._planner.last_optimization_result
        if result is None:
            raise RuntimeError("MPCPlanner.step did not store last_optimization_result")

        n = int(self._planner.problem.sys.n)
        m = int(self._planner.problem.sys.m)
        if plan.n_samples < 2:
            raise RuntimeError(
                "MPC plan must have at least two samples for x_ff = plan.x[:, 1]"
            )

        latch = MPCTickSolve(
            plan=plan,
            z=np.asarray(result.z, dtype=float).reshape(-1),
            u_ff=np.asarray(plan.u[:, 0], dtype=float).reshape(m),
            x_ff=np.asarray(plan.x[:, 1], dtype=float).reshape(n),
        )
        self._latch_k = k_int
        self._latch = latch
        if self._step_disp:
            self._print_step_disp(k_int, result)
        return latch

    def _print_step_disp(self, k_int, result) -> None:
        solve_s = result.solve_time_s
        if solve_s is None:
            solve_s = self._planner.last_solve_time_s
        step_s = self._planner.last_step_time_s
        if self._dt_mpc is not None:
            t_fire = self._t0 + k_int * self._dt_mpc
            print(
                f"MPC @ t={t_fire:.2f}s  success={result.success}  "
                f"solve={solve_s:.3f}s  step={step_s:.3f}s"
            )
        else:
            print(
                f"MPC @ k={k_int}  success={result.success}  "
                f"solve={solve_s:.3f}s  step={step_s:.3f}s"
            )

    def reset_latch(self) -> None:
        """Clear tick memo (e.g. after ``Computer.reset``)."""
        self._latch_k = None
        self._latch = None
