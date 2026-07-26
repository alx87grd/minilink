"""
Real-time simulation orchestrator.

:class:`RealtimeSimulator` advances a continuous system in sync with the
wall clock: poll a live :class:`~minilink.simulation.realtime.io.RealtimeInput`,
hold ``u`` for one frame (ZOH), advance with the same compiled evaluator
stack as offline simulation, draw the frame live through the existing
:class:`~minilink.graphical.animation.animator.Animator` pipeline, publish to
an optional :class:`~minilink.simulation.realtime.io.RealtimeOutput`, and
return a :class:`~minilink.core.trajectory.Trajectory` when the session stops.

Sibling orchestrators: :class:`~minilink.simulation.simulator.Simulator`
(offline time grid) and
:class:`~minilink.simulation.hybrid_simulator.HybridSimulator` (computer
ticks). The stepping math lives in the evaluators; this module owns only the
loop, the clock policy, and the recording.

TODO: User Architectural Review — first realtime slice (game-mode parity);
the loop structure and I/O contracts may still move.
"""

from __future__ import annotations

import logging
import time
import warnings

import numpy as np

from minilink.core.backends import BACKEND_AUTO, BACKEND_JAX, BACKEND_NUMPY
from minilink.core.system import DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.simulation.realtime.io import RealtimeInput, RealtimeOutput
from minilink.simulation.simulator import (
    DISCONTINUOUS_AUTO_DT_SCALE,
    SMOOTH_AUTO_DT_SCALE,
)

# In ``sync="realtime"`` the measured wall elapsed is clamped to this many
# frame periods so a stall (window drag, GC pause) cannot inject a huge step.
MAX_DT_HOLD_FRAMES = 4

# Minimum wall time between real-time overrun warnings (avoid per-frame spam).
OVERRUN_WARN_INTERVAL_S = 2.0


class RealtimeSimulator:
    """
    Advance a continuous system live, one rendered frame at a time.

    Each frame: ``u, stop = input.poll(t, x)`` (nominal ``u`` when no input),
    then ``x = evaluator.integrate_zoh(x, u, t, dt_hold, dt_inner=sim_dt)``,
    live draw, optional external publish, then sleep out the frame budget.

    Parameters
    ----------
    sys : DynamicSystem
        Continuous system or diagram to simulate.
    frame_dt : float
        Target wall time between frames in seconds (default ``1/30``).
    sim_dt : float, optional
        Internal integration step for the ZOH hold. ``None`` derives it from
        ``sys.solver_info`` with the same heuristic as the offline
        :class:`~minilink.simulation.simulator.Simulator`, clamped to
        ``frame_dt``.
    sync : str
        ``"locked"`` (default): advance exactly ``frame_dt`` of simulation
        time per frame — reproducible and JAX-friendly (constant ZOH shape).
        ``"realtime"``: advance by the measured wall elapsed, quantized to a
        multiple of ``sim_dt`` and clamped to ``MAX_DT_HOLD_FRAMES``
        frame periods.
    renderer : str or None
        Live view backend name for
        :func:`~minilink.graphical.animation.animator.make_renderer`
        (default ``"pygame"``); ``None`` runs headless.
    is_3d : bool
        Open the live scene in 3-D (renderer dependent).
    input : RealtimeInput, optional
        Live input source; ``None`` holds the nominal port values.
    output : RealtimeOutput, optional
        External per-frame consumer; ``None`` publishes nothing.
    compile_backend : str or None
        Evaluator backend. ``None`` or ``"auto"`` (default ``None``): try JAX
        when available and the system is JAX-compatible, otherwise NumPy.
        Pass ``"numpy"`` or ``"jax"`` to force a backend.
    max_steps : int, optional
        Stop after this many frames (headless smoke / CI).
    tf : float, optional
        Stop once the simulation time reaches ``tf``.

    Notes
    -----
    One sample is recorded per frame: sample ``k > 0`` stores the state at
    ``t_k`` and the input held over the frame ending at ``t_k``; sample 0 is
    the initial state with the nominal input.

    When a frame's wall cost exceeds ``frame_dt``, a :class:`UserWarning` is
    raised (throttled) so lagging sessions are visible without flooding the
    console.
    """

    def __init__(
        self,
        sys,
        *,
        frame_dt=1 / 30.0,
        sim_dt=None,
        sync="locked",
        renderer="pygame",
        is_3d=False,
        input=None,
        output=None,
        compile_backend=None,
        max_steps=None,
        tf=None,
    ):
        if not isinstance(sys, DynamicSystem):
            raise TypeError(
                "RealtimeSimulator requires a DynamicSystem (continuous evolution); "
                f"got {type(sys).__name__}"
            )
        if sync not in ("locked", "realtime"):
            raise ValueError(f"sync must be 'locked' or 'realtime', got {sync!r}")
        if float(frame_dt) <= 0.0:
            raise ValueError(f"frame_dt must be positive, got {frame_dt}")
        if input is not None and not isinstance(input, RealtimeInput):
            raise TypeError("input must be a RealtimeInput or None")
        if output is not None and not isinstance(output, RealtimeOutput):
            raise TypeError("output must be a RealtimeOutput or None")

        self.sys = sys
        self.frame_dt = float(frame_dt)
        self.sim_dt = self._resolve_sim_dt(sim_dt)
        self.sync = sync
        self.renderer = renderer
        self.is_3d = is_3d
        self.input = input
        self.output = output
        self.max_steps = max_steps
        self.tf = tf
        self.last_traj = None
        self.n_overruns = 0

        self.compile_backend, self.evaluator = self._resolve_and_build_evaluator(
            compile_backend
        )

        # Build the live view up front so bad renderer names fail fast; the
        # window itself opens only inside run(). Lazy graphics import keeps
        # headless use dependency-free.
        self._animator = None
        self._backend = None
        if renderer is not None:
            from minilink.graphical.animation.animator import Animator, make_renderer

            self._animator = Animator(sys)
            self._backend = make_renderer(renderer, self._animator)
            if not self._backend.supports_interactive:
                raise ValueError(
                    f"renderer={renderer!r} does not support interactive loops; "
                    "use it with render() or animate() on a precomputed trajectory."
                )

    # Public API

    def run(self, x0=None, t0=0.0):
        """
        Run the live loop until stop; return the recorded :class:`Trajectory`.

        The run stops on the first of: input ``should_stop``, renderer quit
        event, ``max_steps`` frames, or simulation time reaching ``tf``.

        Parameters
        ----------
        x0 : np.ndarray, optional
            Initial state (defaults to ``sys.x0``).
        t0 : float
            Initial simulation time.
        """
        sys = self.sys
        evaluator = self.evaluator
        frame_dt = self.frame_dt
        sim_dt = self.sim_dt

        x = np.asarray(sys.x0 if x0 is None else x0, dtype=float).reshape(evaluator.n)
        t = float(t0)
        u = np.asarray(sys.get_u_from_input_ports(), dtype=float)

        self._warm_up(x, u, t)

        ts, xs, us = [t], [x.copy()], [u.copy()]

        backend = self._backend
        kinematic = None
        if backend is not None:
            kinematic = sys.get_kinematic_geometry()
            self._animator.open_live_scene(
                backend,
                x,
                u,
                t,
                is_3d=self.is_3d,
                kinematic=kinematic,
                title=f"Realtime: {sys.name}",
            )
        if self.input is not None:
            self.input.open(sys)
        if self.output is not None:
            self.output.open(sys)

        step_idx = 0
        last_frame_start = time.perf_counter()
        last_overrun_warn_s = None
        self.n_overruns = 0
        try:
            while True:
                frame_start = time.perf_counter()
                if self.sync == "realtime" and step_idx > 0:
                    dt_hold = self._quantize_dt_hold(frame_start - last_frame_start)
                else:
                    dt_hold = frame_dt
                last_frame_start = frame_start

                should_stop = False
                if self.input is not None:
                    u, should_stop = self.input.poll(t, x)
                    u = np.asarray(u, dtype=float).reshape(evaluator.m)

                x = evaluator.integrate_zoh(x, u, t, dt_hold, dt_inner=sim_dt)
                x = np.asarray(x, dtype=float)  # native array → NumPy boundary
                t += dt_hold
                step_idx += 1

                ts.append(t)
                xs.append(x.copy())
                us.append(u.copy())

                if backend is not None:
                    events = self._animator.update_live_frame(
                        backend, x, u, t, kinematic=kinematic
                    )
                    should_stop = should_stop or events.get("quit", False)
                if self.output is not None:
                    outputs = evaluator.outputs(x, u, t)
                    self.output.publish(t, x, u, outputs)

                wall_frame_s = time.perf_counter() - frame_start
                if wall_frame_s > frame_dt:
                    self.n_overruns += 1
                    last_overrun_warn_s = self._warn_realtime_overrun(
                        wall_frame_s, last_overrun_warn_s
                    )

                if should_stop:
                    break
                if self.max_steps is not None and step_idx >= self.max_steps:
                    break
                if self.tf is not None and t >= self.tf - 1e-12:
                    break

                sleep_s = frame_dt - wall_frame_s
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        finally:
            if backend is not None:
                backend.close_scene()
            if self.input is not None:
                self.input.close()
            if self.output is not None:
                self.output.close()

        traj = Trajectory(
            t=np.asarray(ts), x=np.column_stack(xs), u=np.column_stack(us)
        )
        self.last_traj = traj
        return traj

    # Internal machinery

    def _resolve_and_build_evaluator(self, compile_backend):
        """Compile with an explicit backend, or auto-try JAX then NumPy.

        ``None`` and ``"auto"`` share the same policy: prefer JAX when the
        package is installed and the system is JAX-compatible.
        """
        if compile_backend is None or compile_backend == BACKEND_AUTO:
            try:
                import jax  # noqa: F401
            except ImportError:
                return BACKEND_NUMPY, self.sys.compile(backend=BACKEND_NUMPY)
            try:
                return BACKEND_JAX, self.sys.compile(backend=BACKEND_JAX)
            except Exception:
                logging.getLogger(__name__).debug(
                    "JAX compile failed, falling back to numpy", exc_info=True
                )
                return BACKEND_NUMPY, self.sys.compile(backend=BACKEND_NUMPY)
        return compile_backend, self.sys.compile(backend=compile_backend)

    def _resolve_sim_dt(self, sim_dt):
        """Explicit ``sim_dt``, or the offline auto-``dt`` heuristic clamped to a frame."""
        if sim_dt is not None:
            sim_dt = float(sim_dt)
            if sim_dt <= 0.0:
                raise ValueError(f"sim_dt must be positive, got {sim_dt}")
            return sim_dt
        info = self.sys.solver_info
        if info.get("discontinuous_behavior", False):
            scale = DISCONTINUOUS_AUTO_DT_SCALE
        else:
            scale = SMOOTH_AUTO_DT_SCALE
        return min(float(info["smallest_time_constant"]) * scale, self.frame_dt)

    def _quantize_dt_hold(self, wall_elapsed):
        """Snap the measured elapsed to a multiple of ``sim_dt`` and clamp it.

        The ZOH substep count is part of the traced shape on JAX, so keeping
        ``dt_hold`` on the ``sim_dt`` grid avoids retracing every frame.
        """
        max_hold = MAX_DT_HOLD_FRAMES * self.frame_dt
        clamped = min(max(wall_elapsed, self.sim_dt), max_hold)
        n_sub = max(1, int(round(clamped / self.sim_dt)))
        return n_sub * self.sim_dt

    def _warn_realtime_overrun(self, wall_frame_s, last_warn_s):
        """Emit a throttled warning when a frame misses the real-time budget."""
        now = time.perf_counter()
        if last_warn_s is not None and (now - last_warn_s) < OVERRUN_WARN_INTERVAL_S:
            return last_warn_s
        fps_budget = 1.0 / self.frame_dt
        fps_actual = 1.0 / wall_frame_s if wall_frame_s > 0.0 else float("inf")
        tip = (
            "try compile_backend='jax' or a larger frame_dt / sim_dt"
            if self.compile_backend == BACKEND_NUMPY
            else "try a larger frame_dt / sim_dt, or a lighter renderer"
        )
        warnings.warn(
            (
                f"RealtimeSimulator is behind real time: frame took "
                f"{wall_frame_s * 1000:.1f} ms "
                f"(budget {self.frame_dt * 1000:.1f} ms, "
                f"~{fps_actual:.1f} Hz vs {fps_budget:.1f} Hz target, "
                f"backend={self.compile_backend!r}, overruns={self.n_overruns}). "
                f"Suggestion: {tip}."
            ),
            UserWarning,
            stacklevel=3,
        )
        return now

    def _warm_up(self, x, u, t):
        """Trigger evaluator JIT compilation before the wall clock matters.

        On JAX the ZOH rollout traces and compiles on first call; doing that
        inside the loop would stall the first frame. The warm result is
        discarded.
        """
        x_warm = self.evaluator.integrate_zoh(
            x, u, t, self.frame_dt, dt_inner=self.sim_dt
        )
        outputs_warm = None
        if self.output is not None:
            outputs_warm = self.evaluator.outputs(x, u, t)
        if getattr(self.evaluator, "backend", "numpy") == "jax":
            import jax

            jax.block_until_ready(x_warm)
            if outputs_warm is not None:
                for value in outputs_warm.values():
                    jax.block_until_ready(value)
