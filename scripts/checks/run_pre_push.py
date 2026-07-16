"""Pre-push gate — ruff + pytest (mirrors CI ``test`` job).

**Human (IDE):** open this file and click **Run** before pushing.
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
    for step in (_common.run_ruff_check, _common.run_ruff_format_check):
        code = step()
        if code != 0:
            return code
    return _common.run_pytest()


if __name__ == "__main__":
    raise SystemExit(main())
