"""Check launcher — set ``CHECK`` below and click **Run**; the checks are the Human table of tests/README.md."""

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
