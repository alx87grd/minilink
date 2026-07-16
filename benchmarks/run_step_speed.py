"""Deprecated: use ``python benchmarks/run_study.py --preset step_eval --plant leaf``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_step_speed.py is deprecated; use "
        "python benchmarks/run_study.py --preset step_eval --plant leaf",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "step_eval", "--plant", "leaf"]))
