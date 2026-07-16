"""Deprecated: use ``python benchmarks/run_study.py --preset trajopt --mode solvers``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_trajopt_solver_presets.py is deprecated; use "
        "python benchmarks/run_study.py --preset trajopt --mode solvers",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "trajopt", "--mode", "solvers"]))
