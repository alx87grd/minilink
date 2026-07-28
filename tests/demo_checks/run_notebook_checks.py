"""Execute teaching notebooks via nbclient (demo-check layer).

Auto-discovers every ``.ipynb`` under ``examples/notebooks/`` (including
subfolders). Code cells must not raise; outputs are discarded (notebooks are
not rewritten). Uses ``MPLBACKEND=Agg`` so matplotlib stays headless.

Defaults: ``timeout=180``, ``requires=[]``. Non-default deps / timeouts live
in ``notebook_overrides.json`` keyed by repo-relative path. Notebooks under
``examples/projects/`` or ``examples/experimental/`` are not smoked here.

Usage (from repo root)::

    python tests/demo_checks/run_notebook_checks.py
    python tests/demo_checks/run_notebook_checks.py --notebook showcase_minilink
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
_CHECKS_DIR = Path(__file__).resolve().parent
OVERRIDES_PATH = _CHECKS_DIR / "notebook_overrides.json"
NOTEBOOK_DIR = REPO_ROOT / "examples" / "notebooks"
DEFAULT_TIMEOUT = 180.0


@dataclass(frozen=True)
class NotebookRow:
    notebook_id: str
    status: str
    message: str = ""


def _notebook_id(rel_path: str) -> str:
    """``examples/notebooks/intro/00_core.ipynb`` → ``intro_00_core``."""
    stem = Path(rel_path).relative_to("examples/notebooks").with_suffix("")
    return "_".join(stem.parts)


def _load_overrides() -> dict[str, dict]:
    if not OVERRIDES_PATH.is_file():
        return {}
    raw = json.loads(OVERRIDES_PATH.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError(f"{OVERRIDES_PATH.name} must be a JSON object keyed by path")
    return raw


def _discover_notebooks() -> list[str]:
    """Repo-relative paths of every ``.ipynb`` under ``examples/notebooks/``."""
    return sorted(
        p.relative_to(REPO_ROOT).as_posix() for p in NOTEBOOK_DIR.rglob("*.ipynb")
    )


def _execute_notebook(path: Path, *, timeout: float) -> tuple[str, str]:
    """Execute ``path`` with nbclient; return (status, message)."""
    try:
        import nbformat
        from nbclient import NotebookClient
        from nbclient.exceptions import CellExecutionError, CellTimeoutError
    except ImportError as exc:
        return "fail", f"missing notebook executor deps: {exc}"

    os.environ.setdefault("PYTHONPATH", str(REPO_ROOT))
    os.environ["MPLBACKEND"] = "Agg"
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    os.environ.setdefault("MINILINK_NOTEBOOK_SMOKE", "1")

    try:
        nb = nbformat.read(path, as_version=4)
        client = NotebookClient(
            nb,
            timeout=int(timeout),
            kernel_name="python3",
            resources={"metadata": {"path": str(REPO_ROOT)}},
        )
        client.execute()
    except CellTimeoutError:
        return "fail", f"timeout after {timeout:.0f}s"
    except CellExecutionError as exc:
        # Last lines of the traceback are the useful bit.
        tail = str(exc).strip()[-500:]
        return "fail", tail
    except OSError as exc:
        return "fail", str(exc)
    except Exception as exc:
        return "fail", f"{type(exc).__name__}: {exc}"

    return "pass", ""


def run_notebook_checks(
    *,
    notebook_filter: str | None = None,
    timeout_override: float | None = None,
) -> list[NotebookRow]:
    overrides = _load_overrides()
    rows: list[NotebookRow] = []

    for rel in _discover_notebooks():
        notebook_id = _notebook_id(rel)
        if notebook_filter is not None and notebook_id != notebook_filter:
            continue

        entry = overrides.get(rel) or {}
        requires = tuple(entry.get("requires") or ())
        missing = False
        for extra in requires:
            if importlib.util.find_spec(extra) is None:
                rows.append(NotebookRow(notebook_id, "skip", f"missing {extra}"))
                missing = True
                break
        if missing:
            continue

        path = REPO_ROOT / rel
        if not path.is_file():
            rows.append(NotebookRow(notebook_id, "fail", f"missing file {rel}"))
            continue

        timeout = (
            float(timeout_override)
            if timeout_override is not None
            else float(entry.get("timeout", DEFAULT_TIMEOUT))
        )
        status, message = _execute_notebook(path, timeout=timeout)
        rows.append(NotebookRow(notebook_id, status, message))
    return rows


def _print_report(rows: list[NotebookRow]) -> int:
    width = max((len(row.notebook_id) for row in rows), default=4)
    for row in rows:
        note = f"  ({row.message})" if row.message else ""
        print(f"  {row.notebook_id:<{width}}  {row.status}{note}")
    failed = sum(row.status == "fail" for row in rows)
    skipped = sum(row.status == "skip" for row in rows)
    passed = sum(row.status == "pass" for row in rows)
    print(f"\n{passed} passed, {failed} failed, {skipped} skipped")
    return 1 if failed else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Notebook smoke checks: execute each teaching notebook's code cells "
            "(outputs discarded; fail on error)"
        )
    )
    parser.add_argument(
        "--notebook",
        default=None,
        help="Run one notebook id (e.g. showcase_minilink, intro_00_core)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=None,
        help="Override per-notebook timeout seconds (default: override or 180)",
    )
    args = parser.parse_args(argv)
    rows = run_notebook_checks(
        notebook_filter=args.notebook,
        timeout_override=args.timeout,
    )
    return _print_report(rows)


if __name__ == "__main__":
    raise SystemExit(main())
