"""Human-readable ``verbose`` panels for time-domain simulators."""

from __future__ import annotations

from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)


def print_simulation_preamble(
    *,
    title: str,
    system_name: str,
    n: int,
    m: int,
    x0,
    t0: float,
    tf: float,
    n_pts: int,
    dt: float,
    solver_mode: str,
    user_solver: str | None,
    compile_backend: str,
    auto_time_grid: bool,
    solver_options: dict | None = None,
    notes: list[str] | None = None,
) -> None:
    """Framed setup panel printed before integration starts."""
    print()
    print(DISP_RULE_MAIN)
    print(title)
    print(DISP_RULE_MAIN)
    print(f"system: {system_name!r}")
    print(f"n={n}, m={m}")
    print(f"x0: {preview_vector(x0)}")
    print(f"interval: [{t0:g}, {tf:g}]")
    grid_note = " (auto dt)" if auto_time_grid else ""
    print(f"n_pts={n_pts}, dt={dt:g}{grid_note}")
    if user_solver is None:
        print(f"solver: {solver_mode!r} (auto-selected)")
    else:
        print(f"solver: {solver_mode!r} (user={user_solver!r})")
    print(f"compile_backend={compile_backend!r}")
    if solver_options:
        print("solver_options:", solver_options)
    if notes:
        print(DISP_RULE_DIV)
        print("notes:")
        for note in notes:
            print(f"  - {note}")
    print(DISP_RULE_DIV)
    print("Running integration...")


def print_simulation_report(
    *,
    elapsed_s: float,
    n_samples: int,
    x_final=None,
    last_debug: dict | None = None,
) -> None:
    """Framed completion panel printed after integration finishes."""
    print(f"Completed in {elapsed_s:.3f} seconds")
    print(DISP_RULE_DIV)
    print(f"n_samples: {n_samples}")
    if x_final is not None and len(x_final) > 0:
        print(f"x_final: {preview_vector(x_final)}")
    if last_debug:
        stats = {
            key: last_debug[key]
            for key in ("solver", "mode", "nfev", "njev", "nlu", "success", "status")
            if key in last_debug
        }
        if stats:
            print("integration_stats:", stats)
    print(DISP_RULE_MAIN)
