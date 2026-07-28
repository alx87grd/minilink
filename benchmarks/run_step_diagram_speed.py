"""Deprecated: use ``python benchmarks/run_study.py --preset step_eval --plant step_diagram``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_step_diagram_speed.py is deprecated; use "
        "python benchmarks/run_study.py --preset step_eval --plant step_diagram",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "step_eval", "--plant", "step_diagram"]))
