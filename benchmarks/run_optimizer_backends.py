"""Deprecated: use ``python benchmarks/run_study.py --preset optimizer``."""

from __future__ import annotations

from benchmarks.run_study import main

if __name__ == "__main__":
    import warnings

    warnings.warn(
        "run_optimizer_backends.py is deprecated; use "
        "python benchmarks/run_study.py --preset optimizer",
        DeprecationWarning,
        stacklevel=1,
    )
    raise SystemExit(main(["--preset", "optimizer"]))
