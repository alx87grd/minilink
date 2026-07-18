"""Notebook smoke checks — execute teaching notebooks; fail on cell errors.

**Human (IDE):** open this file and click **Run**.
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


def main() -> int:
    return _common.run_demo_check_script("tests/demo_checks/run_notebook_checks.py")


if __name__ == "__main__":
    raise SystemExit(main())
