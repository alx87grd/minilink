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
:class:`~minilink.simulation.discrete.hybrid_simulator.HybridSimulator` (computer
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
from minilink.core.compile.evaluators.stepping import substeps_of
from minilink.core.system import AUTO_DT_SCALE, DynamicSystem
from minilink.core.trajectory import Trajectory
from minilink.simulation.realtime.io import RealtimeInput, RealtimeOutput

# In ``sync="realtime"`` the measured wall elapsed is clamped to this many
# frame periods so a stall (window drag, GC pause) cannot inject a huge step.
MAX_DT_HOLD_FRAMES = 4

# Live ceiling on ZOH substeps per rendered frame during auto ``sim_dt``
# (prefer not lagging even when the offline auto-``dt`` is tiny).
DEFAULT_SIM_STEPS_PER_FRAME = 10

# Fraction of ``frame_dt`` reserved for integration during auto ``sim_dt``
# calibration (leave most of the frame for draw / I/O — prefer not lagging).
AUTO_SIM_DT_BUDGET_FRAC = 0.25

# Timed probe frames after a discard warm-up when choosing auto ``sim_dt``.
AUTO_SIM_DT_PROBE_REPEATS = 3

# Minimum wall time between real-time overrun warnings (avoid per-frame spam).
OVERRUN_WARN_INTERVAL_S = 2.0


class RealtimeSimulator:
    """
    Advance a continuous system live, one rendered frame at a time.

    Each frame: ``u, stop = input.poll(t, x)`` (nominal ``u`` when no input),
    then ``x = evaluator.step(x, u, t, dt=dt_hold, substeps=…)`` (steps of about ``sim_dt``),
    live draw, optional external publish, then sleep out the frame budget.

    Parameters
    ----------
    sys : DynamicSystem
        Continuous system or diagram to simulate.
    frame_dt : float
        Target wall time between frames in seconds (default ``1/30``).
    sim_dt : float, optional
        Internal integration step for the ZOH hold. ``None`` (default) picks
        a live-friendly step at the start of :meth:`run`: time a short ZOH
        probe, then choose as many substeps per ``frame_dt`` as fit in
        ``AUTO_SIM_DT_BUDGET_FRAC`` of the frame budget (default 25%, so most
        of the frame is left for draw / I/O). The fine-end cap is the coarser
        of ``frame_dt / DEFAULT_SIM_STEPS_PER_FRAME`` and the offline
        :class:`~minilink.simulation.simulator.Simulator` auto-``dt``
        (``smallest_time_constant`` × scale) — never finer than offline would
        use, and never more than the live substep ceiling. Pass an explicit
        value to skip calibration.
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
    plot : bool or sequence of str or (System, str)
        Open a live rolling-window time-signal plot during the run (default
        ``False``): ``True`` shows the signals of
        :func:`~minilink.graphical.signals.resolve_plot_signals`, a sequence
        names the signals to show. Uses
        :class:`~minilink.simulation.realtime.live_plot.LiveSignalPlotOutput`,
        published beside ``output`` when both are given.
    plot_backend : str
        Signal plotting backend of the live plot (``"matplotlib"`` or
        ``"plotly"``), as on every plotting verb.
    backend : str or None
        Evaluator backend. ``None`` or ``"auto"`` (default ``None``): try JAX
        when available and the system is JAX-compatible, otherwise NumPy.
        Pass ``"numpy"`` or ``"jax"`` to force a backend.
    max_steps : int, optional
        Stop after this many frames (headless smoke / CI).
    tf : float, optional
        Stop once the simulation time reaches ``tf``.
    verbose : bool
        Print framed setup / completion panels (default quiet), matching the
        offline :class:`~minilink.simulation.simulator.Simulator` style, plus a
        live in-place per-frame line of compute time vs ``frame_dt`` budget
        (``ok`` / ``LATE``).

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
        plot=False,
        plot_backend="matplotlib",
        backend=None,
        max_steps=None,
        tf=None,
        verbose=False,
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
        self._auto_sim_dt = sim_dt is None
        # Provisional until :meth:`run` calibrates when ``sim_dt`` was omitted.
        self.sim_dt = self._resolve_sim_dt(sim_dt)
        self.sync = sync
        self.renderer = renderer
        self.is_3d = is_3d
        self.input = input
        require_plot_flag(plot)
        self.plot = plot
        self.plot_backend = plot_backend
        self.output = realtime_output_of(output, plot, plot_backend=plot_backend)
        self.max_steps = max_steps
        self.tf = tf
        self.verbose = bool(verbose)
        self.last_traj = None
        self.n_overruns = 0
        self._compile_s = 0.0

        t_compile = time.perf_counter()
        self.backend, self.evaluator = self._resolve_and_build_evaluator(backend)
        self._compile_s = time.perf_counter() - t_compile

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

    def run(self, x0=None, t_start=0.0):
        """
        Run the live loop until stop; return the recorded :class:`Trajectory`.

        The run stops on the first of: input ``should_stop``, renderer quit
        event, ``max_steps`` frames, or simulation time reaching ``tf``.

        Parameters
        ----------
        x0 : np.ndarray, optional
            Initial state (defaults to ``sys.x0``).
        t_start : float
            Initial simulation time.
        """
        sys = self.sys
        evaluator = self.evaluator
        frame_dt = self.frame_dt

        x = np.asarray(sys.x0 if x0 is None else x0, dtype=float).reshape(evaluator.n)
        t = float(t_start)
        u = np.asarray(sys.get_u_from_input_ports(), dtype=float)

        t_warm = time.perf_counter()
        if self._auto_sim_dt:
            self.sim_dt = self._calibrate_sim_dt(x, u, t)
        self._warm_up(x, u, t)
        warm_up_s = time.perf_counter() - t_warm
        if self.verbose:
            from minilink.simulation.sim_reporting import print_realtime_preamble

            print_realtime_preamble(
                system_name=sys.name,
                n=evaluator.n,
                m=evaluator.m,
                x0=x,
                t_start=t,
                frame_dt=frame_dt,
                sim_dt=self.sim_dt,
                sim_dt_auto=self._auto_sim_dt,
                sync=self.sync,
                backend=self.backend,
                renderer=self.renderer,
                compile_s=self._compile_s,
                warm_up_s=warm_up_s,
                tf=self.tf,
                max_steps=self.max_steps,
            )

        sim_dt = self.sim_dt

        ts, xs, us = [t], [x.copy()], [u.copy()]
        backend = self._backend
        kinematic = None
        output_opened = False
        scene_opened = False
        input_opened = False
        step_idx = 0
        self.n_overruns = 0
        status_open = False
        try:
            # Open output first (e.g. a live Matplotlib signal plot) so the
            # interactive scene / PygameInput window opens last and keeps OS focus.
            if self.output is not None:
                self.output.open(sys, x0=x, u0=u, t0=t)
                output_opened = True

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
                scene_opened = True
            if self.input is not None:
                self.input.open(sys)
                input_opened = True

            last_frame_start = time.perf_counter()
            last_overrun_warn_s = None
            run_wall_start = last_frame_start
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

                x = evaluator.step(
                    x, u, t, dt=dt_hold, substeps=substeps_of(dt_hold, sim_dt)
                )
                x = np.asarray(x, dtype=float)  # native array → NumPy boundary
                t += dt_hold
                step_idx += 1

                ts.append(t)
                xs.append(x.copy())
                us.append(u.copy())

                if backend is not None:
                    if self.input is not None:
                        d_az, d_el = self.input.poll_camera(dt_hold)
                        if d_az != 0.0 or d_el != 0.0:
                            backend.orbit_camera(d_az, d_el)
                    events = self._animator.update_live_frame(
                        backend, x, u, t, kinematic=kinematic
                    )
                    should_stop = should_stop or events.get("quit", False)
                if self.output is not None:
                    outputs = (
                        evaluator.outputs(x, u, t)
                        if self.output.needs_boundary_outputs
                        else {}
                    )
                    self.output.publish(t, x, u, outputs)

                wall_frame_s = time.perf_counter() - frame_start
                if wall_frame_s > frame_dt:
                    self.n_overruns += 1
                    last_overrun_warn_s = self._warn_realtime_overrun(
                        wall_frame_s, last_overrun_warn_s
                    )

                if self.verbose:
                    tag = "LATE" if wall_frame_s > frame_dt else "ok"
                    msg = (
                        f"t={t:.2f}s  compute={wall_frame_s * 1000:.1f}ms / "
                        f"budget={frame_dt * 1000:.1f}ms  {tag}"
                    )
                    print(f"\r{msg:<72}", end="", flush=True)
                    status_open = True

                if should_stop:
                    break
                if self.max_steps is not None and step_idx >= self.max_steps:
                    break
                if self.tf is not None and t >= self.tf - 1e-12:
                    break

                sleep_s = frame_dt - wall_frame_s
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
        except KeyboardInterrupt:
            pass
        finally:
            if self.verbose and status_open:
                print()
            if backend is not None and scene_opened:
                backend.close_scene()
            if self.input is not None and input_opened:
                self.input.close()
            if self.output is not None and output_opened:
                self.output.close()

        if self.verbose:
            from minilink.simulation.sim_reporting import print_realtime_report

            wall_s = time.perf_counter() - run_wall_start
            print_realtime_report(
                elapsed_s=wall_s,
                n_frames=step_idx,
                n_overruns=self.n_overruns,
                t_final=t,
                x_final=x,
            )

        traj = Trajectory(
            t=np.asarray(ts), x=np.column_stack(xs), u=np.column_stack(us)
        )
        self.last_traj = traj
        return traj

    # Internal machinery

    def _resolve_and_build_evaluator(self, backend):
        """Compile with an explicit backend, or auto-try JAX then NumPy.

        ``None`` and ``"auto"`` share the same policy: prefer JAX when the
        package is installed and the system is JAX-compatible.
        """
        if backend is None or backend == BACKEND_AUTO:
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
        return backend, self.sys.compile(backend=backend)

    def _offline_auto_dt(self):
        """Same automatic ``dt`` the offline :class:`Simulator` would pick."""
        info = self.sys.solver_info
        return float(info["smallest_time_constant"]) * AUTO_DT_SCALE

    def _auto_n_sub_max(self):
        """Max ZOH substeps/frame: offline auto-``dt`` floor, live ceiling.

        Never refine past the offline Simulator auto-``dt``, and never take
        more than ``DEFAULT_SIM_STEPS_PER_FRAME`` substeps (prefer not lagging).
        """
        offline_dt = max(self._offline_auto_dt(), np.finfo(float).eps)
        n_from_offline = max(1, int(self.frame_dt // offline_dt))
        return max(1, min(DEFAULT_SIM_STEPS_PER_FRAME, n_from_offline))

    def _resolve_sim_dt(self, sim_dt):
        """Explicit ``sim_dt``, or provisional finest auto grid until calibrate."""
        if sim_dt is not None:
            sim_dt = float(sim_dt)
            if sim_dt <= 0.0:
                raise ValueError(f"sim_dt must be positive, got {sim_dt}")
            return sim_dt
        return self.frame_dt / self._auto_n_sub_max()

    def _calibrate_sim_dt(self, x, u, t):
        """Choose ``sim_dt`` so a frame of ZOH work fits under a real-time budget.

        Probes the finest allowed grid (``_auto_n_sub_max`` substeps — offline
        auto-``dt`` floor capped by the live substep ceiling), estimates cost
        per substep, then picks the largest ``n_sub`` that fits in
        ``AUTO_SIM_DT_BUDGET_FRAC * frame_dt``.
        """
        frame_dt = self.frame_dt
        n_probe = self._auto_n_sub_max()
        sim_dt_probe = frame_dt / n_probe
        evaluator = self.evaluator

        def _one_frame():
            x_out = evaluator.step(
                x, u, t, dt=frame_dt, substeps=substeps_of(frame_dt, sim_dt_probe)
            )
            if getattr(evaluator, "backend", "numpy") == "jax":
                import jax

                jax.block_until_ready(x_out)
            return x_out

        # Discard warm-up (JIT / cache) before timing.
        _one_frame()

        times = []
        for _ in range(AUTO_SIM_DT_PROBE_REPEATS):
            t0 = time.perf_counter()
            _one_frame()
            times.append(time.perf_counter() - t0)
        times.sort()
        probe_s = times[len(times) // 2]

        cost_per_sub = probe_s / n_probe
        budget_s = AUTO_SIM_DT_BUDGET_FRAC * frame_dt
        if cost_per_sub <= 0.0:
            n_sub = n_probe
        else:
            n_sub = int(budget_s // cost_per_sub)
        n_sub = max(1, min(n_probe, n_sub))
        return frame_dt / n_sub

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
            "try backend='jax' or a larger frame_dt / sim_dt"
            if self.backend == BACKEND_NUMPY
            else "try a larger frame_dt / sim_dt, or a lighter renderer"
        )
        warnings.warn(
            (
                f"RealtimeSimulator is behind real time: frame took "
                f"{wall_frame_s * 1000:.1f} ms "
                f"(budget {self.frame_dt * 1000:.1f} ms, "
                f"~{fps_actual:.1f} Hz vs {fps_budget:.1f} Hz target, "
                f"backend={self.backend!r}, overruns={self.n_overruns}). "
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
        x_warm = self.evaluator.step(
            x, u, t, dt=self.frame_dt, substeps=substeps_of(self.frame_dt, self.sim_dt)
        )
        outputs_warm = None
        if self.output is not None and self.output.needs_boundary_outputs:
            outputs_warm = self.evaluator.outputs(x, u, t)
        if getattr(self.evaluator, "backend", "numpy") == "jax":
            import jax

            jax.block_until_ready(x_warm)
            if outputs_warm is not None:
                for value in outputs_warm.values():
                    jax.block_until_ready(value)


###############################################################################
# Internal machinery
###############################################################################


class CompositeOutput(RealtimeOutput):
    """Fan out lifecycle and per-frame samples to multiple :class:`RealtimeOutput`s."""

    def __init__(self, outputs):
        self.outputs = tuple(outputs)
        self.needs_boundary_outputs = any(
            out.needs_boundary_outputs for out in self.outputs
        )

    def open(self, sys, *, x0=None, u0=None, t0=0.0):
        for out in self.outputs:
            out.open(sys, x0=x0, u0=u0, t0=t0)

    def publish(self, t, x, u, outputs):
        for out in self.outputs:
            out.publish(t, x, u, outputs)

    def close(self):
        for out in self.outputs:
            out.close()


def require_plot_flag(plot) -> None:
    """Refuse a backend name as ``plot``: the backend is ``plot_backend``, ``plot`` is ``True`` or the signals."""
    if isinstance(plot, str):
        raise ValueError(
            f"plot={plot!r}: pass plot=True or a sequence of signal names, and the "
            "backend as plot_backend='matplotlib' or 'plotly'"
        )


def realtime_output_of(output, plot, *, plot_backend: str = "matplotlib"):
    """The user's ``RealtimeOutput``, with a ``LiveSignalPlotOutput`` beside it when ``plot`` asks for one."""
    if plot is False or plot is None:
        return output

    from minilink.simulation.realtime.live_plot import LiveSignalPlotOutput

    signals = None if plot is True else tuple(plot)
    plot_out = LiveSignalPlotOutput(signals=signals, backend=plot_backend)
    if output is None:
        return plot_out
    return CompositeOutput((plot_out, output))
