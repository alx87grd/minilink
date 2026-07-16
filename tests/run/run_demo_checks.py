"""Demo checks — catalog plants + flagship demos must not throw.

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

CATALOG_FAST = True  # False → full catalog sweep (slower)


def main() -> int:
    catalog_args = ["--fast"] if CATALOG_FAST else []
    code = _common.run_demo_check_script(
        "tests/demo_checks/run_catalog_checks.py",
        catalog_args,
    )
    if code != 0:
        return code
    return _common.run_demo_check_script("tests/demo_checks/run_flagship_demos.py")


if __name__ == "__main__":
    raise SystemExit(main())
