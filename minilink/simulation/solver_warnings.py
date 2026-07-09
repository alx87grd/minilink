"""User-facing warnings for discontinuous closed-loop simulation."""

from __future__ import annotations

import warnings

_DISCONTINUOUS_AUTO_DT_SCALE = 0.1


def collect_discontinuous_solver_notes(
    *,
    solver_mode: str,
    solver_info: dict,
    dt: float | None,
    user_solver: str | None,
    user_specified_dt: bool,
    verbose: bool = False,
) -> list[str]:
    """
    Build discontinuous-loop advisory notes for panels or warnings.

    Returns an empty list when ``discontinuous_behavior`` is false.
    """
    if not solver_info.get("discontinuous_behavior", False):
        return []

    notes = [
        "Discontinuous feedback detected (e.g. sliding-mode sign(s)). "
        "Prefer explicit Euler with a small dt; logged port torques match "
        "one evaluation per step. RK4 sub-steps and SciPy adaptive stepping "
        "can produce misleading or unstable results. For digital sample-and-hold "
        "semantics, consider HybridSimulator with a sampled computer model and "
        "an explicit sample time (hybrid_closed_loop / sample_static)."
    ]

    if user_solver == "rk4_fixedsteps":
        notes.append(
            "Forced rk4_fixedsteps: sub-step torques may oscillate and cancel; "
            "ctl:u on the output grid may not match the effective integrated torque."
        )
    elif user_solver is not None and (
        user_solver == "scipy" or user_solver.startswith("scipy_")
    ):
        notes.append(
            f"Forced {user_solver}: adaptive stepping may stall or take extreme "
            "substeps near switching surfaces."
        )

    smallest = solver_info.get("smallest_time_constant", 0.001)
    recommended_dt = smallest * _DISCONTINUOUS_AUTO_DT_SCALE
    if user_solver == "euler" and user_specified_dt and dt is not None:
        if dt > recommended_dt:
            notes.append(
                f"Euler with dt={dt:g} may be too coarse for discontinuous feedback; "
                f"consider dt <= {recommended_dt:g}, HybridSimulator with an explicit "
                "computer sample time, or a hybrid_closed_loop path."
            )
    elif user_solver is None and solver_mode == "euler" and verbose and dt is not None:
        notes.append(
            f"Auto-selected Euler with dt={dt:g} (discontinuous default scale "
            f"{_DISCONTINUOUS_AUTO_DT_SCALE:g} × smallest_time_constant)."
        )

    return notes


def emit_discontinuous_solver_warnings(
    *,
    solver_mode: str,
    solver_info: dict,
    dt: float | None,
    user_solver: str | None,
    user_specified_dt: bool,
    solver_warnings: str = "warn",
    verbose: bool = False,
) -> list[str]:
    """
    Emit :class:`UserWarning` messages for discontinuous closed loops.

    When ``verbose`` is true, returns notes for the setup panel and does **not**
    call :func:`warnings.warn` (avoids duplicate output after the preamble).
    """
    notes = collect_discontinuous_solver_notes(
        solver_mode=solver_mode,
        solver_info=solver_info,
        dt=dt,
        user_solver=user_solver,
        user_specified_dt=user_specified_dt,
        verbose=verbose,
    )
    if not notes:
        return notes
    if solver_warnings == "ignore" or verbose:
        return notes

    text = " ".join(notes)
    if solver_warnings == "error":
        raise UserWarning(text)
    warnings.warn(text, category=UserWarning, stacklevel=3)
    return notes
