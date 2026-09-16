"""Assert a built minilink wheel or sdist ships the library and not the research lane."""

from __future__ import annotations

import argparse
import sys
import tarfile
import zipfile
from pathlib import Path


def _leaked_research_lane(names: list[str]) -> list[str]:
    leaked: list[str] = []
    for name in names:
        path = name.replace("\\", "/")
        if (
            "/minilink/experimental/" in path
            or path.endswith("/minilink/experimental")
            or "/examples/experimental/" in path
            or path.endswith("/examples/experimental")
            or "/examples/projects/" in path
            or path.endswith("/examples/projects")
        ):
            leaked.append(name)
    return leaked


def check_wheel(path: Path) -> list[str]:
    names = zipfile.ZipFile(path).namelist()
    errors: list[str] = []
    if not any(name.endswith("minilink/__init__.py") for name in names):
        errors.append(f"{path.name} has no minilink/__init__.py")
    leaked = _leaked_research_lane(names)
    if leaked:
        errors.append(f"{path.name} ships research-lane paths: {leaked[:8]}")
    return errors


def check_sdist(path: Path) -> list[str]:
    with tarfile.open(path, "r:gz") as archive:
        names = archive.getnames()
    errors: list[str] = []
    if not any(name.endswith("minilink/__init__.py") for name in names):
        errors.append(f"{path.name} has no minilink/__init__.py")
    leaked = _leaked_research_lane(names)
    if leaked:
        errors.append(f"{path.name} ships research-lane paths: {leaked[:8]}")
    return errors


def check_artifact(path: Path) -> list[str]:
    suffix = "".join(path.suffixes)
    if suffix.endswith(".whl"):
        return check_wheel(path)
    if suffix.endswith(".tar.gz"):
        return check_sdist(path)
    return [f"unsupported artifact {path.name}"]


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifacts", nargs="+", type=Path)
    args = parser.parse_args(argv)
    errors: list[str] = []
    for artifact in args.artifacts:
        if not artifact.is_file():
            errors.append(f"missing artifact {artifact}")
            continue
        errors.extend(check_artifact(artifact))
    if errors:
        print("\n".join(errors), file=sys.stderr)
        return 1
    for artifact in args.artifacts:
        print(f"ok {artifact.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
