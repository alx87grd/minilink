"""Committed benchmark baselines: load, compare, and report regression metrics."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Literal

import numpy as np

from benchmarks.common import ansi

GateKind = Literal["speed", "accuracy"]
DirectionKind = Literal["higher_better", "lower_better", "vector_match"]
CompareStatus = Literal["pass", "fail", "skip", "missing"]

DEFAULT_REGRESSION_FACTOR = 4.0


@dataclass(frozen=True)
class MetricRecord:
    """One scalar or vector metric from a benchmark run or baseline file."""

    id: str
    gate: GateKind
    direction: DirectionKind
    value: float | list[float]
    unit: str = "ratio"
    max_allowed: float | None = None
    atol: float | None = None
    rtol: float | None = None
    notes: str = ""


@dataclass
class BaselineFile:
    """Parsed baseline JSON."""

    schema_version: int
    suite: str
    description: str
    regression_factor: float
    recorded_at: str
    host_hint: str
    metrics: tuple[MetricRecord, ...]


@dataclass(frozen=True)
class ComparisonRow:
    """One metric compared against the baseline."""

    metric_id: str
    gate: GateKind
    status: CompareStatus
    baseline_value: float | list[float] | None
    current_value: float | list[float] | None
    message: str = ""


@dataclass
class ComparisonResult:
    """Full comparison output."""

    rows: tuple[ComparisonRow, ...]

    @property
    def failed(self) -> bool:
        return any(row.status == "fail" for row in self.rows)

    @property
    def failures(self) -> tuple[ComparisonRow, ...]:
        return tuple(row for row in self.rows if row.status == "fail")


def filter_speed_gate_failures(
    result: ComparisonResult,
    *,
    suffixes: tuple[str, ...],
) -> ComparisonResult:
    """Downgrade speed failures whose metric id does not end with a suffix."""
    if not suffixes:
        return result
    rows: list[ComparisonRow] = []
    for row in result.rows:
        if row.status != "fail" or row.gate != "speed":
            rows.append(row)
            continue
        if any(row.metric_id.endswith(suffix) for suffix in suffixes):
            rows.append(row)
            continue
        rows.append(
            ComparisonRow(
                metric_id=row.metric_id,
                gate=row.gate,
                status="skip",
                baseline_value=row.baseline_value,
                current_value=row.current_value,
                message=f"ignored (not in speed gate suffixes: {row.message})",
            )
        )
    return ComparisonResult(rows=tuple(rows))


def default_regression_factor() -> float:
    raw = os.environ.get("MINILINK_BENCH_REGRESSION_FACTOR", "").strip()
    if not raw:
        return DEFAULT_REGRESSION_FACTOR
    return float(raw)


def load_baseline(path: Path) -> BaselineFile:
    """Load a baseline JSON file."""
    data = json.loads(path.read_text(encoding="utf-8"))
    metrics = tuple(_metric_from_dict(item) for item in data["metrics"])
    return BaselineFile(
        schema_version=int(data["schema_version"]),
        suite=str(data["suite"]),
        description=str(data.get("description", "")),
        regression_factor=float(
            data.get("regression_factor", DEFAULT_REGRESSION_FACTOR)
        ),
        recorded_at=str(data.get("recorded_at", "")),
        host_hint=str(data.get("host_hint", "")),
        metrics=metrics,
    )


def save_baseline(path: Path, baseline: BaselineFile) -> None:
    """Write a baseline JSON file."""
    payload = {
        "schema_version": baseline.schema_version,
        "suite": baseline.suite,
        "description": baseline.description,
        "regression_factor": baseline.regression_factor,
        "recorded_at": baseline.recorded_at,
        "host_hint": baseline.host_hint,
        "metrics": [_metric_to_dict(metric) for metric in baseline.metrics],
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def compare_metrics(
    recorded: list[MetricRecord],
    baseline: BaselineFile,
    *,
    factor: float | None = None,
) -> ComparisonResult:
    """Compare recorded metrics against a baseline file."""
    factor = baseline.regression_factor if factor is None else factor
    baseline_by_id = {metric.id: metric for metric in baseline.metrics}
    rows: list[ComparisonRow] = []

    for current in recorded:
        ref = baseline_by_id.get(current.id)
        if ref is None:
            rows.append(
                ComparisonRow(
                    metric_id=current.id,
                    gate=current.gate,
                    status="skip",
                    baseline_value=None,
                    current_value=current.value,
                    message="no baseline entry",
                )
            )
            continue
        status, message = _compare_one(current, ref, factor=factor)
        rows.append(
            ComparisonRow(
                metric_id=current.id,
                gate=ref.gate,
                status=status,
                baseline_value=ref.value,
                current_value=current.value,
                message=message,
            )
        )

    return ComparisonResult(rows=tuple(rows))


def merge_recorded_into_baseline(
    baseline: BaselineFile,
    recorded: list[MetricRecord],
    *,
    relax_accuracy: bool = False,
) -> BaselineFile:
    """Return a new baseline with ``value`` fields updated from a recorded run."""
    recorded_by_id = {metric.id: metric for metric in recorded}
    merged: list[MetricRecord] = []
    for metric in baseline.metrics:
        new = recorded_by_id.get(metric.id)
        if new is None:
            merged.append(metric)
            continue
        merged.append(
            MetricRecord(
                id=metric.id,
                gate=metric.gate,
                direction=metric.direction,
                value=new.value,
                unit=new.unit or metric.unit,
                max_allowed=new.max_allowed if relax_accuracy else metric.max_allowed,
                atol=new.atol if relax_accuracy else metric.atol,
                rtol=new.rtol if relax_accuracy else metric.rtol,
                notes=metric.notes or new.notes,
            )
        )
    for metric_id, new in recorded_by_id.items():
        if metric_id not in {metric.id for metric in baseline.metrics}:
            merged.append(new)
    return BaselineFile(
        schema_version=baseline.schema_version,
        suite=baseline.suite,
        description=baseline.description,
        regression_factor=baseline.regression_factor,
        recorded_at=baseline.recorded_at,
        host_hint=baseline.host_hint,
        metrics=tuple(merged),
    )


def print_comparison_report(result: ComparisonResult) -> None:
    """Print speed and accuracy comparison tables."""
    speed_rows = [row for row in result.rows if row.gate == "speed"]
    accuracy_rows = [row for row in result.rows if row.gate == "accuracy"]

    if speed_rows:
        print()
        print("=== Speed regression ===")
        _print_table(speed_rows)
    if accuracy_rows:
        print()
        print("=== Accuracy regression ===")
        _print_table(accuracy_rows)

    if result.failed:
        print()
        print(ansi("REGRESSION FAILED", 1, 31))
    else:
        print()
        print(ansi("All regression checks passed", 1, 32))


def _print_table(rows: list[ComparisonRow]) -> None:
    print(f"{'metric':<42} {'status':<8} {'baseline':>14} {'current':>14}  note")
    print("-" * 95)
    for row in rows:
        status = row.status
        if status == "pass":
            status = ansi(status, 32)
        elif status == "fail":
            status = ansi(status, 31)
        print(
            f"{row.metric_id:<42} {status:<17} "
            f"{_format_value(row.baseline_value):>14} "
            f"{_format_value(row.current_value):>14}  {row.message}"
        )


def _compare_one(
    current: MetricRecord,
    baseline: MetricRecord,
    *,
    factor: float,
) -> tuple[CompareStatus, str]:
    if current.direction == "vector_match":
        return _compare_vector_match(current, baseline)
    if not isinstance(current.value, (int, float)) or isinstance(current.value, bool):
        return "fail", "expected scalar metric"
    current_f = float(current.value)
    if baseline.direction == "vector_match":
        return "fail", "baseline/current direction mismatch"

    baseline_f = float(baseline.value)  # type: ignore[arg-type]

    if baseline.gate == "accuracy" and baseline.direction == "lower_better":
        ceiling = baseline.max_allowed
        if ceiling is None:
            ceiling = baseline_f
        if current_f > ceiling:
            return "fail", f"above max_allowed {ceiling:g}"
        return "pass", ""

    if baseline.direction == "higher_better":
        floor = baseline_f / factor
        if current_f < floor:
            return "fail", f"below baseline/{factor:g} ({floor:g})"
        return "pass", ""

    if baseline.direction == "lower_better":
        ceiling = baseline_f * factor
        if current_f > ceiling:
            return "fail", f"above baseline*{factor:g} ({ceiling:g})"
        return "pass", ""

    return "fail", f"unknown direction {baseline.direction!r}"


def _compare_vector_match(
    current: MetricRecord,
    baseline: MetricRecord,
) -> tuple[CompareStatus, str]:
    if not isinstance(current.value, list) or not isinstance(baseline.value, list):
        return "fail", "vector_match requires list values"
    current_v = np.asarray(current.value, dtype=float)
    baseline_v = np.asarray(baseline.value, dtype=float)
    atol = 0.05 if baseline.atol is None else baseline.atol
    rtol = 0.05 if baseline.rtol is None else baseline.rtol
    if np.allclose(current_v, baseline_v, atol=atol, rtol=rtol):
        return "pass", ""
    delta = float(np.max(np.abs(current_v - baseline_v)))
    return "fail", f"max |delta|={delta:.4g} (atol={atol}, rtol={rtol})"


def _metric_from_dict(data: dict[str, Any]) -> MetricRecord:
    value = data["value"]
    if isinstance(value, list):
        value = [float(v) for v in value]
    else:
        value = float(value)
    return MetricRecord(
        id=str(data["id"]),
        gate=data["gate"],
        direction=data["direction"],
        value=value,
        unit=str(data.get("unit", "ratio")),
        max_allowed=_optional_float(data.get("max_allowed")),
        atol=_optional_float(data.get("atol")),
        rtol=_optional_float(data.get("rtol")),
        notes=str(data.get("notes", "")),
    )


def _metric_to_dict(metric: MetricRecord) -> dict[str, Any]:
    payload: dict[str, Any] = {
        "id": metric.id,
        "gate": metric.gate,
        "direction": metric.direction,
        "value": metric.value,
        "unit": metric.unit,
    }
    if metric.max_allowed is not None:
        payload["max_allowed"] = metric.max_allowed
    if metric.atol is not None:
        payload["atol"] = metric.atol
    if metric.rtol is not None:
        payload["rtol"] = metric.rtol
    if metric.notes:
        payload["notes"] = metric.notes
    return payload


def _optional_float(value: Any) -> float | None:
    if value is None:
        return None
    return float(value)


def _format_value(value: float | list[float] | None) -> str:
    if value is None:
        return "-"
    if isinstance(value, list):
        if len(value) <= 4:
            return "[" + ", ".join(f"{v:.4g}" for v in value) + "]"
        head = ", ".join(f"{v:.4g}" for v in value[:3])
        return f"[{head}, ...]({len(value)})"
    return f"{value:.4g}"
