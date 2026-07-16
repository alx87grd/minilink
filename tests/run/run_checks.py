"""Check launcher — pick a check, click **Run**.

**Human (IDE):** change ``CHECK`` below, then Run this file.

| CHECK | What |
| --- | --- |
| ``l1`` | L1 pytest (daily default) |
| ``l1_minimal`` | L1 without optional extras |
| ``l1_headless`` | L1 with headless pygame |
| ``pre_push`` | ruff + pytest (CI test job) |
| ``l2`` | L2 regression (full local) |
| ``l2_ci`` | L2 regression (CI flags) |
| ``l6`` | L6 catalog + flagship smokes |
| ``l3_list`` | L3 benchmark preset list |
| ``l3_f_eval`` | L3 native/NumPy/JAX f() on pendulum |
"""

from __future__ import annotations

import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[2]
_CHECKS = Path(__file__).resolve().parent
for path in (_ROOT, _CHECKS):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import _common  # noqa: E402

CHECK = "l1"


def main() -> int:
    match CHECK:
        case "l1":
            return _common.run_pytest()
        case "l1_minimal":
            return _common.run_pytest(marker="not optional")
        case "l1_headless":
            return _common.run_pytest(headless_pygame=True)
        case "pre_push":
            for step in (_common.run_ruff_check, _common.run_ruff_format_check):
                code = step()
                if code != 0:
                    return code
            return _common.run_pytest()
        case "l2":
            return _common.run_regression(ci_mode=False)
        case "l2_ci":
            return _common.run_regression(ci_mode=True)
        case "l6":
            code = _common.run_smoke_script(
                "tests/smoke/run_catalog_smokes.py",
                ["--fast"],
            )
            if code != 0:
                return code
            return _common.run_smoke_script("tests/smoke/run_flagship_demos.py")
        case "l3_list":
            return _common.run_study_script(["--list"])
        case "l3_f_eval":
            return _common.run_study_script(
                ["--preset", "f_eval", "--plant", "pendulum"]
            )
        case other:
            print(f"Unknown CHECK={other!r}. See docstring for options.")
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
