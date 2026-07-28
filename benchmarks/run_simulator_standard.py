"""Deprecated: use ``python benchmarks/run_study.py --preset sim --mode standard``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_simulator_standard.py is deprecated; use "
        "python benchmarks/run_study.py --preset sim --mode standard",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "sim", "--mode", "standard"]))
