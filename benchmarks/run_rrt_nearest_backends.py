"""Deprecated: use ``python benchmarks/run_study.py --preset rrt_nearest``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_rrt_nearest_backends.py is deprecated; use "
        "python benchmarks/run_study.py --preset rrt_nearest",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "rrt_nearest"]))
