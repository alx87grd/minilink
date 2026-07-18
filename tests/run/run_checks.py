"""Check launcher — pick a check, click **Run**.

**Human (IDE):** change ``CHECK`` below, then Run this file.

| CHECK | What |
| --- | --- |
| ``contract_tests`` | Contract tests via pytest (daily default) |
| ``contract_tests_minimal`` | Contract tests without optional extras |
| ``contract_tests_headless`` | Contract tests with headless pygame |
| ``pre_push`` | ruff + pytest (CI test job) |
| ``regression_gates`` | Regression gates (full local) |
| ``regression_gates_ci`` | Regression gates (CI flags) |
| ``demo_checks`` | Catalog + flagship demo checks |
| ``notebook_checks`` | Teaching notebook smoke (execute code cells) |
| ``benchmark_study_list`` | Benchmark study preset list |
| ``benchmark_study_f_eval`` | Benchmark study: native/NumPy/JAX f() on pendulum |
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

CHECK = "contract_tests"


def main() -> int:
    match CHECK:
        case "contract_tests":
            return _common.run_pytest()
        case "contract_tests_minimal":
            return _common.run_pytest(marker="not optional")
        case "contract_tests_headless":
            return _common.run_pytest(headless_pygame=True)
        case "pre_push":
            for step in (_common.run_ruff_check, _common.run_ruff_format_check):
                code = step()
                if code != 0:
                    return code
            return _common.run_pytest()
        case "regression_gates":
            return _common.run_regression(ci_mode=False)
        case "regression_gates_ci":
            return _common.run_regression(ci_mode=True)
        case "demo_checks":
            code = _common.run_demo_check_script(
                "tests/demo_checks/run_catalog_checks.py",
                ["--fast"],
            )
            if code != 0:
                return code
            return _common.run_demo_check_script(
                "tests/demo_checks/run_flagship_demos.py"
            )
        case "notebook_checks":
            return _common.run_demo_check_script(
                "tests/demo_checks/run_notebook_checks.py"
            )
        case "benchmark_study_list":
            return _common.run_study_script(["--list"])
        case "benchmark_study_f_eval":
            return _common.run_study_script(
                ["--preset", "f_eval", "--plant", "pendulum"]
            )
        case other:
            print(f"Unknown CHECK={other!r}. See docstring for options.")
            return 2


if __name__ == "__main__":
    raise SystemExit(main())
