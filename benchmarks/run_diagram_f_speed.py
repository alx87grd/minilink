"""Deprecated: use ``python benchmarks/run_study.py --preset f_eval --plant diagram_dense``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_diagram_f_speed.py is deprecated; use "
        "python benchmarks/run_study.py --preset f_eval --plant diagram_dense",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "f_eval", "--plant", "diagram_dense"]))
