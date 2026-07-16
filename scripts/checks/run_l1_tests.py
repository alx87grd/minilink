"""L1 — library contract tests (pytest).

**Human (IDE):** open this file and click **Run** — no terminal needed.

Includes L4 graphics contract and L6 smoke bridge via ``test_smoke_runners.py``.
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

# --- optional IDE toggles (defaults are fine for daily use) ---
HEADLESS_PYGAME = False  # True → SDL_VIDEODRIVER=dummy for pygame smokes
MARKER = None  # e.g. "not optional" for minimal-deps-only run
EXTRA_ARGS: list[str] = []  # e.g. ["tests/unittest/test_mpc.py"] for one module


def main() -> int:
    return _common.run_pytest(
        EXTRA_ARGS or None,
        marker=MARKER,
        headless_pygame=HEADLESS_PYGAME,
    )


if __name__ == "__main__":
    raise SystemExit(main())
