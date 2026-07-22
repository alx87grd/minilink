"""Closed-loop one-lap harness for MPC tuning."""

from __future__ import annotations

import json
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import numpy as np

from examples.projects.mpc_tuning.models import build_plant, corridor_half_width
from examples.projects.mpc_tuning.track import make_track, start_pose
from minilink.control.mpc import ModelPredictiveController
from minilink.core.backends import configure_jax
from minilink.planning.problems import PlanningProblem
from minilink.planning.trajectory_optimization.planner import (
    TrajectoryOptimizationPlanner,
)

configure_jax(enable_x64=True)

PROJECT_ROOT = Path(__file__).resolve().parent
RESULTS_DIR = PROJECT_ROOT / "results"
CONFIGS_DIR = PROJECT_ROOT / "configs"


@dataclass
class LapResult:
    completed: bool
    lap_time: float | None
    mean_path_error: float
    max_path_error: float
    median_solve_s: float | None
    p95_solve_s: float | None
    nlp_success_rate: float
    n_replans: int
    t_end: float
    progress_s: float
    valid: bool
    realtime_ok: bool
    error: str | None = None

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


def _project_s_dist(path, p_xy: np.ndarray) -> tuple[float, float]:
    """Arc-length and distance-to-centerline (``Path.project`` returns closest point)."""
    s, closest = path.project(p_xy)
    d = float(np.linalg.norm(np.asarray(p_xy, dtype=float).reshape(-1) - closest))
    return float(s), d


def _lap_progress_and_errors(
    xy: np.ndarray, path, *, max_dist_for_progress: float | None = None
) -> tuple[float, np.ndarray]:
    """Cumulative forward arc progress and per-sample path distances.

    Progress only advances when the sample is within ``max_dist_for_progress``
    of the centerline (avoids false laps from off-track projection jumps).
    """
    L = float(path.total_length)
    n = xy.shape[1]
    dists = np.empty(n, dtype=float)
    s_prev, dists[0] = _project_s_dist(path, xy[:, 0])
    progress = 0.0
    for i in range(1, n):
        s, d = _project_s_dist(path, xy[:, i])
        dists[i] = d
        ds = float(s) - float(s_prev)
        if ds < -0.5 * L:
            ds += L
        on_track = max_dist_for_progress is None or d <= max_dist_for_progress
        if on_track and ds > 0.0:
            progress += ds
        s_prev = s
    return float(progress), dists


def _path_ok(mean_e: float, max_e: float, half_w: float) -> bool:
    return mean_e < half_w and max_e < 2.0 * half_w


def run_lap(model_key: str, config: dict[str, Any]) -> LapResult:
    """Simulate one lap from rest; return metrics (never raises for NLP issues)."""
    cfg = dict(config)
    cfg["model"] = model_key
    tf_max = float(cfg.get("TF_SIM_MAX", 90.0))
    mpc_dt = float(cfg.get("MPC_DT", 0.2))
    horizon = float(cfg["MPC_HORIZON"])
    n_steps = int(cfg["MPC_STEPS"])
    plant_dt = float(cfg.get("plant_dt_inner", 0.01))
    half_w = corridor_half_width()

    track = make_track()
    start_xy, theta0 = start_pose(track)
    solve_times: list[float] = []
    successes: list[bool] = []

    try:
        sys_mpc, sys_sim, x0, cost = build_plant(
            model_key, track, cfg, start_xy, theta0
        )
        sys_sim.x0 = x0.copy()
        planner = TrajectoryOptimizationPlanner(
            PlanningProblem(sys=sys_mpc, x_start=x0, cost=cost, tf=horizon),
            n_steps=n_steps,
            transcription=str(cfg.get("transcription", "direct_collocation")),
            compile_backend=str(cfg.get("compile_backend", "jax")),
            record_solve_time=True,
            optimizer_method=str(cfg.get("optimizer_method", "scipy_slsqp")),
            optimizer_options={
                "maxiter": int(cfg.get("maxiter", 100)),
                "ftol": float(cfg.get("ftol", 0.1)),
            },
        )
        mpc = ModelPredictiveController(
            planner,
            dt_mpc=mpc_dt,
            warm_start=bool(cfg.get("warm_start", True)),
            step_disp=False,
        )

        def _on_solve() -> None:
            opt = planner.last_optimization_result
            if opt is None:
                return
            successes.append(bool(opt.success))
            st = opt.solve_time_s
            if st is None:
                st = planner.last_solve_time_s
            if st is not None:
                solve_times.append(float(st))

        # export_to_computer clears after_solve — set hook after ``mpc @ plant``.
        hybrid = mpc @ sys_sim
        mpc._latch.set_after_solve(_on_solve)

        # Cap sim length from cruise speed so failed runs don't always burn TF_SIM_MAX.
        L_track = float(track.path.total_length)
        u_tgt = float(cfg.get("U_TARGET", 10.0))
        tf_est = min(tf_max, max(30.0, 1.6 * L_track / max(u_tgt, 1.0) + 15.0))

        t0 = time.perf_counter()
        result = hybrid.compute_trajectory(
            tf=tf_est,
            x0_plant=x0,
            plant_dt_inner=plant_dt,
            compile_backend=str(cfg.get("compile_backend", "jax")),
            verbose=False,
        )
        wall = time.perf_counter() - t0
        traj = result.plant
        xy = np.vstack([traj.x[0, :], traj.x[1, :]])
        # Subsample for progress / error (every ~0.05 s).
        stride = max(1, int(round(0.05 / max(plant_dt, 1e-6))))
        idx = np.arange(0, xy.shape[1], stride)
        if idx[-1] != xy.shape[1] - 1:
            idx = np.append(idx, xy.shape[1] - 1)
        xy_s = xy[:, idx]
        t_s = traj.t[idx]
        progress, dists = _lap_progress_and_errors(
            xy_s, track.path, max_dist_for_progress=2.0 * half_w
        )
        L = L_track
        completed = progress >= L - 1e-3
        lap_time = None
        lap_i = len(dists) - 1
        if completed:
            prog = 0.0
            s_prev, _ = _project_s_dist(track.path, xy_s[:, 0])
            lap_time = float(t_s[-1])
            for i in range(1, xy_s.shape[1]):
                s, d = _project_s_dist(track.path, xy_s[:, i])
                ds = float(s) - float(s_prev)
                if ds < -0.5 * L:
                    ds += L
                if d <= 2.0 * half_w and ds > 0.0:
                    prog += ds
                s_prev = s
                if prog >= L - 1e-3:
                    lap_time = float(t_s[i])
                    lap_i = i
                    break

        dists_lap = dists[: lap_i + 1]
        mean_e = float(np.mean(dists_lap))
        max_e = float(np.max(dists_lap))
        # Exclude first replan from timing stats (JIT / cold start).
        timing = solve_times[1:] if len(solve_times) > 1 else list(solve_times)
        med = float(np.median(timing)) if timing else None
        p95 = float(np.percentile(timing, 95)) if timing else None
        n_ok = sum(1 for s in successes if s)
        rate = float(n_ok / len(successes)) if successes else 0.0
        path_ok = _path_ok(mean_e, max_e, half_w)
        valid = bool(completed and path_ok)
        realtime_ok = med is not None and med <= 0.2
        _ = wall  # available for logging callers
        return LapResult(
            completed=completed,
            lap_time=lap_time,
            mean_path_error=mean_e,
            max_path_error=max_e,
            median_solve_s=med,
            p95_solve_s=p95,
            nlp_success_rate=rate,
            n_replans=len(successes),
            t_end=float(traj.t[-1]),
            progress_s=progress,
            valid=valid,
            realtime_ok=bool(realtime_ok),
        )
    except Exception as exc:  # noqa: BLE001 — trials must not abort the search
        return LapResult(
            completed=False,
            lap_time=None,
            mean_path_error=float("nan"),
            max_path_error=float("nan"),
            median_solve_s=None,
            p95_solve_s=None,
            nlp_success_rate=0.0,
            n_replans=len(successes),
            t_end=0.0,
            progress_s=0.0,
            valid=False,
            realtime_ok=False,
            error=f"{type(exc).__name__}: {exc}",
        )


def append_jsonl(model_key: str, record: dict[str, Any]) -> Path:
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    path = RESULTS_DIR / f"{model_key}.jsonl"
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, default=float) + "\n")
    return path


def load_json(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def save_json(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, sort_keys=True)
        f.write("\n")


def is_valid_lap(lap: LapResult, *, require_realtime: bool = False) -> bool:
    if not lap.valid:
        return False
    if require_realtime and not lap.realtime_ok:
        return False
    return True
