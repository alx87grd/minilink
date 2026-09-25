"""Shared trajectory sampling for animation backends."""

import math
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

    Frames sit on every ``skip_steps``-th sample and the last frame is always
    the final sample. A trajectory with no duration (one sample, or every sample
    at one instant) shows each sample for one frame.
    """
    nsteps = int(traj.t.size)
    frame_dt = 1.0 / target_fps
    if nsteps < 2 or traj.t[-1] == traj.t[0]:
        return AnimationFrameSchedule(
            nsteps=nsteps,
            skip_steps=1,
            interval_ms=frame_dt * 1000.0,
            n_frames=nsteps,
            target_fps=target_fps,
        )
    sim_dt = (traj.t[-1] - traj.t[0]) / (nsteps - 1)
    video_dt = frame_dt * time_factor_video
    skip_steps = max(1, int(np.round(video_dt / sim_dt)))
    interval_ms = (sim_dt * skip_steps / time_factor_video) * 1000.0
    n_frames = math.ceil((nsteps - 1) / skip_steps) + 1
    return AnimationFrameSchedule(
        nsteps=nsteps,
        skip_steps=skip_steps,
        interval_ms=interval_ms,
        n_frames=n_frames,
        target_fps=target_fps,
    )


def sim_index_for_frame(frame_idx: int, schedule: AnimationFrameSchedule) -> int:
    return int(min(frame_idx * schedule.skip_steps, schedule.nsteps - 1))
