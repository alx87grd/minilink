"""Helpers shared by the benchmark modules.

Everything here is performance-reporting plumbing used by at least two
benchmark modules: backend labels, optional-dependency probes, and ANSI
table coloring.
"""

from __future__ import annotations

import os
import sys


def format_benchmark_backend_label(compile_backend: str) -> str:
    """Human-readable backend label for benchmark tables and result structs.

    For ``compile_backend="jax"``, returns ``jax(cpu)``, ``jax(gpu)``, etc., from the
    active JAX default device. Any other value is returned unchanged.
    """
    if compile_backend != "jax":
        return compile_backend
    try:
        import jax
    except ImportError:
        return "jax"
    try:
        plat = str(jax.default_backend()).lower()
    except Exception:
        return "jax"
    return f"jax({plat})"


def ipopt_available() -> bool:
    """Return ``True`` when the ``cyipopt`` solver backend can be imported."""
    try:
        import cyipopt  # noqa: F401

        return True
    except ImportError:
        return False


def stdout_color() -> bool:
    """Whether table rows may use ANSI colors (NO_COLOR/FORCE_COLOR aware)."""
    if os.environ.get("NO_COLOR", "").strip():
        return False
    if os.environ.get("FORCE_COLOR", "").strip():
        return True
    if os.environ.get("CLICOLOR_FORCE", "").strip():
        return True
    return sys.stdout.isatty()


def ansi(text: str, *codes: int) -> str:
    """Wrap ``text`` in ANSI escape codes when stdout supports color."""
    if not stdout_color() or not codes:
        return text
    return f"\033[{';'.join(str(code) for code in codes)}m{text}\033[0m"
