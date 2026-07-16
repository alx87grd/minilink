"""Deprecated: use ``python benchmarks/run_study.py --preset sim --mode matrix``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_simulator_speed_matrix.py is deprecated; use "
        "python benchmarks/run_study.py --preset sim --mode matrix",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "sim", "--mode", "matrix"]))
