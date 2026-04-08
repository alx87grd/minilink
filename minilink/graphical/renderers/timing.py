"""Shared trajectory sampling for animation backends."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class AnimationFrameSchedule:
    """Timing derived from a trajectory and playback speed."""

    nsteps: int
    skip_steps: int
    interval_ms: float
    n_frames: int
    target_fps: float


def trajectory_frame_schedule(
    traj,
    time_factor_video: float,
    *,
    target_fps: float = 30.0,
) -> AnimationFrameSchedule:
    """
    Match matplotlib's FuncAnimation timing: subsample simulation steps and
    compute wall-clock interval between displayed frames.
    """
    nsteps = int(traj.t.size)
    sim_dt = (traj.t[-1] - traj.t[0]) / (nsteps - 1)
    frame_dt = 1.0 / target_fps
    video_dt = frame_dt * time_factor_video
    skip_steps = max(1, int(np.round(video_dt / sim_dt)))
    interval_ms = (sim_dt * skip_steps / time_factor_video) * 1000.0
    n_frames = int(nsteps / skip_steps)
    return AnimationFrameSchedule(
        nsteps=nsteps,
        skip_steps=skip_steps,
        interval_ms=interval_ms,
        n_frames=n_frames,
        target_fps=target_fps,
    )


def sim_index_for_frame(frame_idx: int, schedule: AnimationFrameSchedule) -> int:
    return int(min(frame_idx * schedule.skip_steps, schedule.nsteps - 1))
