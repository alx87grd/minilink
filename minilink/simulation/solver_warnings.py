"""User-facing warnings for discontinuous closed loops and held signals."""

from __future__ import annotations

import warnings

from minilink.core.system import DEFAULT_SMALLEST_TIME_CONSTANT

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
        "an explicit sample time (hybrid_closed_loop)."
    ]

    if user_solver == "rk4_fixedsteps":
        notes.append(
            "Forced rk4_fixedsteps: sub-step torques may oscillate and cancel; "
            "ctl:u on the output grid may not match the effective integrated torque."
        )
    elif user_solver == "euler_fixedsteps":
        notes.append(
            "Forced euler_fixedsteps: uses uniform-dt compiled rollouts; prefer "
            "solver='euler' when the time grid is non-uniform."
        )
    elif user_solver is not None and (
        user_solver == "scipy" or user_solver.startswith("scipy_")
    ):
        notes.append(
            f"Forced {user_solver}: adaptive stepping may stall or take extreme "
            "substeps near switching surfaces."
        )

    smallest = solver_info.get("smallest_time_constant", DEFAULT_SMALLEST_TIME_CONSTANT)
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


def collect_held_signal_notes(
    *,
    solver_mode: str,
    solver_info: dict,
    dt: float | None,
    user_solver: str | None,
    user_specified_dt: bool,
) -> list[str]:
    """
    Build advisory notes when a block holds a signal over a sample period.

    Returns an empty list when no subsystem publishes ``sample_period``.
    """
    period = solver_info.get("sample_period")
    if period is None:
        return []

    notes = []
    if user_solver is not None and (
        user_solver == "scipy" or user_solver.startswith("scipy_")
    ):
        notes.append(
            f"Forced {user_solver} on a signal held over {period:g} s: adaptive "
            "stepping fights the jump at every sample; rk4_fixedsteps with a dt "
            "that divides the sample period is faster and more accurate."
        )
    if user_specified_dt and dt is not None and not solver_mode.startswith("scipy"):
        steps_per_sample = period / dt
        if abs(steps_per_sample - round(steps_per_sample)) > 1e-9 * max(
            1.0, steps_per_sample
        ):
            notes.append(
                f"dt={dt:g} does not divide the sample period {period:g} s: a step "
                "straddles a sample boundary, or holds one sample over several, and "
                "the plant feels an intensity scaled by dt / sample_period; use "
                "dt = sample_period / n."
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
    Emit :class:`UserWarning` messages for discontinuous closed loops and held signals.

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
    notes += collect_held_signal_notes(
        solver_mode=solver_mode,
        solver_info=solver_info,
        dt=dt,
        user_solver=user_solver,
        user_specified_dt=user_specified_dt,
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
