"""Recorded host speed snapshots for regression context (not pass/fail gates)."""

from __future__ import annotations

import json
import platform
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from benchmarks.baseline import MetricRecord
from benchmarks.common import ansi

PROFILES_DIR = Path(__file__).resolve().parent / "host_profiles"


@dataclass(frozen=True)
class HostProfile:
    """Speed metrics recorded on one machine/workload."""

    profile_id: str
    label: str
    recorded_at: str
    host_hint: str
    workload: str
    notes: str
    metrics: dict[str, float]


def load_host_profiles(directory: Path | None = None) -> tuple[HostProfile, ...]:
    root = PROFILES_DIR if directory is None else directory
    if not root.is_dir():
        return ()
    profiles: list[HostProfile] = []
    for path in sorted(root.glob("*.json")):
        profiles.append(
            _profile_from_dict(json.loads(path.read_text(encoding="utf-8")))
        )
    return tuple(profiles)


def save_host_profile(
    path: Path,
    *,
    profile_id: str,
    label: str,
    recorded: list[MetricRecord],
    workload: str,
    host_hint: str | None = None,
    notes: str = "",
) -> None:
    """Write a profile from a regression run (speed metrics only)."""
    metrics = {
        metric.id: float(metric.value)
        for metric in recorded
        if metric.gate == "speed" and isinstance(metric.value, (int, float))
    }
    payload = {
        "schema_version": 1,
        "profile_id": profile_id,
        "label": label,
        "recorded_at": _today_iso(),
        "host_hint": host_hint or platform.platform(),
        "workload": workload,
        "notes": notes,
        "metrics": metrics,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def print_host_speed_context(
    *,
    recorded: list[MetricRecord],
    baseline_host_hint: str,
    current_host_hint: str | None = None,
    profiles: tuple[HostProfile, ...] | None = None,
    metric_ids: tuple[str, ...] | None = None,
) -> None:
    """Print how current speed metrics compare to other recorded hosts."""
    profiles = load_host_profiles() if profiles is None else profiles
    if not profiles:
        return

    current_host_hint = current_host_hint or platform.platform()
    speed_current = {
        metric.id: float(metric.value)
        for metric in recorded
        if metric.gate == "speed" and isinstance(metric.value, (int, float))
    }
    if not speed_current:
        return

    if metric_ids is None:
        metric_ids = tuple(sorted(speed_current))

    print()
    print(ansi("=== Host speed context (informational) ===", 1, 36))
    print(f"current host: {current_host_hint}")
    if baseline_host_hint:
        print(f"gate baseline host: {baseline_host_hint}")
    print(
        "Recorded profiles help interpret speed failures on slow VMs — "
        "they do not change pass/fail."
    )
    print()
    print(
        f"{'metric':<42} {'current':>10}  "
        + "  ".join(f"{profile.profile_id:>16}" for profile in profiles)
    )
    print("-" * (54 + 18 * len(profiles)))

    for metric_id in metric_ids:
        if metric_id not in speed_current:
            continue
        current = speed_current[metric_id]
        cells = []
        for profile in profiles:
            ref = profile.metrics.get(metric_id)
            if ref is None:
                cells.append(f"{'—':>16}")
                continue
            ratio = current / ref if ref > 0 else float("inf")
            cells.append(f"{ref:>10.4g} ({ratio:.1f}×)")
        print(f"{metric_id:<42} {current:>10.4g}  " + "  ".join(cells))

    print()
    for profile in profiles:
        note = f" — {profile.notes}" if profile.notes else ""
        print(
            f"  {profile.profile_id}: {profile.label} "
            f"({profile.workload}, {profile.recorded_at}){note}"
        )


def _profile_from_dict(data: dict[str, Any]) -> HostProfile:
    metrics_raw = data.get("metrics", {})
    metrics = {str(key): float(value) for key, value in metrics_raw.items()}
    return HostProfile(
        profile_id=str(data["profile_id"]),
        label=str(data.get("label", data["profile_id"])),
        recorded_at=str(data.get("recorded_at", "")),
        host_hint=str(data.get("host_hint", "")),
        workload=str(data.get("workload", "default")),
        notes=str(data.get("notes", "")),
        metrics=metrics,
    )


def _today_iso() -> str:
    from datetime import date

    return date.today().isoformat()
