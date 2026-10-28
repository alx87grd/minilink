"""Hybrid simulation: a computer ticking on its schedule, the plant held between ticks."""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.compile.evaluators.stepping import substeps_of
from minilink.core.discrete.hybrid_diagram import HybridDiagram
from minilink.core.discrete.step_rollout import StepRollout
from minilink.core.system import System
from minilink.core.trajectory import Trajectory
from minilink.simulation.discrete.computer import Computer
from minilink.simulation.input_coercion import coerce_forced_input, require_x0

###############################################################################
# Public API
###############################################################################


class HybridSimulator:
    """
    Advance a :class:`HybridDiagram` on the computer base tick grid.

    The API mirrors :class:`~minilink.simulation.simulator.Simulator`:
    construct with ``t0`` / ``tf``, then :meth:`solve` or :meth:`solve_forced`.

    Parameters
    ----------
    hybrid : HybridDiagram
        Computer + plant + boundary connections.
    x0_plant, x0_computer : array_like, optional
        Initial plant / computer states (defaults from diagram ``x0``).
    t0, tf : float
        Simulation interval in seconds. Duration defaults to ``tf - t0``.
    n_steps : int, optional
        Override number of base ticks (advanced).
    plant_dt_inner : float, optional
        RK4 sub-step inside each base tick for the plant; also sets plant
        :class:`~minilink.core.trajectory.Trajectory` sample spacing.
    backend : str
        Backend for plant compile (default ``numpy``).
    verbose : bool
        Print setup information.
    """

    def __init__(
        self,
        hybrid: HybridDiagram,
        *,
        x0_plant=None,
        x0_computer=None,
        t0=0,
        tf=10,
        n_steps=None,
        plant_dt_inner=None,
        backend=BACKEND_NUMPY,
        verbose=False,
    ):
        require_hybrid_diagram(hybrid)

        # The diagram, its computer and its plant, and the options
        self.verbose = verbose
        self.hybrid = hybrid
        self.computer: Computer = hybrid.computer
        self.plant = hybrid.plant
        self.plant_dt_inner = plant_dt_inner

        # The base tick grid on [t0, tf], at the schedule's base period
        self.t0 = float(t0)
        self.tf = float(tf)
        self.dt_base, self.n_ticks, self.times = select_tick_grid(
            t0=self.t0,
            tf=self.tf,
            dt_base=self.computer.schedule.dt_base,
            n_steps=n_steps,
        )

        # The compiled computer and plant
        self.computer.compile()
        self.plant_eval = self.plant.compile(backend=backend)

        # The initial states, each diagram's own x0 unless given
        self.x0_plant = require_x0(
            self.plant.x0 if x0_plant is None else x0_plant,
            self.plant.n,
            label="x0_plant",
        )
        self.x0_computer = require_x0(
            self.computer.diagram.x0 if x0_computer is None else x0_computer,
            self.computer.diagram.n,
            label="x0_computer",
        )

        # The boundary links, computer to plant and plant to computer, as port pairs
        self.c2p, self.p2c = index_boundary_connections(hybrid)

        # The latest run, filled by solve and solve_forced
        self.last_result = None

        if self.verbose:
            print(
                f"HybridSimulator: dt_base={self.dt_base}, n_ticks={self.n_ticks}, "
                f"t=[{self.t0}, {self.tf}]"
            )

    def solve(self):
        """Simulate with nominal computer-boundary inputs."""
        diagram = self.computer.diagram
        n_ticks = self.n_ticks

        # The nominal boundary input, held on every tick
        u_nominal = np.asarray(diagram.get_u_from_input_ports(), dtype=float)
        if diagram.m == 0:
            u_forced = np.zeros((0, n_ticks))
        else:
            u_forced = np.repeat(u_nominal.reshape(-1, 1), n_ticks, axis=1)

        return self.run(u_forced)

    def solve_forced(self, u, *, input_port_id=None):
        """
        Simulate with prescribed computer-boundary input.

        ``u`` may be a callable ``u(t)``, constant vector, or sampled matrix
        on the tick grid (``m × n_ticks``). With ``input_port_id``, ``u``
        drives that one input port and the other ports hold their nominal
        values.
        """
        diagram = self.computer.diagram

        # The input on the ticks; a computer without inputs has none to force
        if diagram.m == 0 and input_port_id is None:
            u_traj = np.zeros((0, self.n_ticks))
        else:
            u_traj = coerce_forced_input(
                diagram, self.times, u, input_port_id=input_port_id
            )

        return self.run(u_traj)

    def run(self, u_traj: np.ndarray) -> HybridSimResult:
        """Tick the computer on ``u_traj`` (``m × n_ticks``), the plant held between ticks."""
        computer = self.computer
        diagram = computer.diagram
        plant = self.plant
        plant_eval = self.plant_eval
        c2p = self.c2p
        p2c = self.p2c
        t0 = self.t0
        dt_base = self.dt_base
        dt_inner = self.plant_dt_inner
        n_ticks = self.n_ticks
        n_computer = diagram.n
        plant_m = plant.m

        # The initial states, and the plant outputs the first tick samples
        computer.reset(self.x0_computer)
        x_plant = self.x0_plant.copy()
        u_plant_nominal = np.asarray(plant.get_u_from_input_ports(), dtype=float)
        zoh_buffers: dict[str, np.ndarray] = {}
        sample_buffers = sample_plant_outputs(
            plant_eval,
            x_plant,
            u_plant_nominal,
            t0,
            p2c,
        )

        # The computer's history, one column per tick
        k_hist = np.zeros(n_ticks, dtype=float)
        x_computer_hist = np.zeros((n_computer, n_ticks))
        computer_signal_hist = allocate_signal_hist(diagram, p2c, n_ticks)
        plant_port_aliases = {
            plant_port: computer_port for plant_port, computer_port in p2c
        }

        # The plant's history, one segment of fine samples per tick
        plant_signal_names = collect_plant_signal_names(diagram, p2c)
        t_plant_parts: list[np.ndarray] = []
        x_plant_parts: list[np.ndarray] = []
        u_plant_parts: list[np.ndarray] = []
        plant_signal_parts = {name: [] for name in plant_signal_names}

        for tick_idx in range(n_ticks):
            # Tick the computer on its forced input and the plant outputs it sampled
            k = computer.k
            t_k = t0 + k * dt_base
            u_computer = assemble_computer_u(
                diagram,
                u_traj[:, tick_idx],
                sample_buffers,
                p2c,
            )
            outs = computer.tick(u_computer)

            # Zero-order hold: the commands stay on the plant input until the next tick
            for computer_port, plant_port in c2p:
                zoh_buffers[plant_port] = np.asarray(
                    outs[computer_port], dtype=float
                ).copy()
            u_plant = assemble_plant_u(
                plant,
                u_plant_nominal,
                zoh_buffers,
                c2p,
            )

            # Integrate the plant over [t_k, t_k + Δt], then sample its outputs
            n_sub = substeps_of(dt_base, dt_inner)
            h = dt_base / n_sub
            u_held = np.tile(np.asarray(u_plant, dtype=float), (n_sub, 1))
            x_seg = np.asarray(plant_eval.rollout(x_plant, u_held, t_k, h))
            t_seg = t_k + h * np.arange(n_sub + 1)
            x_plant = x_seg[-1]
            sample_buffers = sample_plant_outputs(
                plant_eval,
                x_plant,
                u_plant,
                t_k + dt_base,
                p2c,
            )

            # A segment opens on the last tick's final sample: the first tick keeps it
            if tick_idx == 0:
                t_append = t_seg
                x_append = x_seg
            else:
                t_append = t_seg[1:]
                x_append = x_seg[1:]

            # Record the plant's fine samples, then the computer's tick
            t_plant_parts.append(t_append)
            x_plant_parts.append(x_append)
            record_plant_samples(
                u_plant_parts,
                plant_signal_parts,
                plant_eval,
                diagram,
                t_append,
                x_append,
                u_plant,
                u_computer,
                plant_m,
            )
            k_hist[tick_idx] = k
            x_computer_hist[:, tick_idx] = computer.x
            record_tick_signals(
                computer_signal_hist,
                diagram,
                u_computer,
                outs,
                sample_buffers,
                tick_idx,
                plant_port_aliases=plant_port_aliases,
            )

        # The histories stacked; x and u are channels of their own, never extra signals
        t_plant = np.concatenate(t_plant_parts)
        x_plant_hist = np.vstack(x_plant_parts).T
        u_plant_fine = (
            np.column_stack(u_plant_parts) if u_plant_parts else np.zeros((plant_m, 0))
        )
        plant_signals = {}
        for name, parts in plant_signal_parts.items():
            if not parts or name in {"x", "u"}:
                continue
            plant_signals[name] = np.column_stack(parts)

        computer_signals = {
            name: values
            for name, values in computer_signal_hist.items()
            if name not in {"x", "u"}
        }

        # The computer's step rollout and the plant's trajectory
        computer_rollout = StepRollout(
            k=k_hist,
            x=x_computer_hist,
            u=u_traj,
            signals=computer_signals,
        )
        plant_traj = Trajectory(
            t=t_plant,
            x=x_plant_hist,
            u=u_plant_fine,
            signals=plant_signals,
        )

        # The result, kept as the latest run
        result = HybridSimResult(
            computer=computer_rollout,
            plant=plant_traj,
            hybrid=self.hybrid,
        )
        self.last_result = result

        return result


@dataclass(frozen=True)
class HybridSimResult:
    """
    Hybrid rollout with separate computer and plant views.

    Parameters
    ----------
    computer : StepRollout
        Tick-indexed computer diagram state and boundary signals.
    plant : Trajectory
        Continuous-time plant state and boundary channels at ``plant_dt_inner``
        (or ``dt_base`` when inner sub-stepping is unset).
    hybrid : HybridDiagram, optional
        Source diagram for plot labels (optional).
    """

    computer: StepRollout
    plant: Trajectory
    hybrid: HybridDiagram | None = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        require_views(self.computer, self.plant)

    @property
    def n_samples(self) -> int:
        """Number of plant trajectory samples."""
        return self.plant.n_samples

    def plot(
        self,
        *,
        signals: tuple[str, ...] | None = None,
        show: bool = True,
        backend="matplotlib",
        **kwargs,
    ):
        """Plot plant channels (continuous-time view)."""
        return self.plot_view(
            self.plant,
            sys=self.hybrid.plant if self.hybrid is not None else None,
            signals=signals,
            show=show,
            backend=backend,
            abscissa_label=None,
            control_key=self.plant_control_signal_key(),
            **kwargs,
        )

    def plot_computer(
        self,
        *,
        signals: tuple[str, ...] | None = None,
        show: bool = True,
        backend="matplotlib",
        **kwargs,
    ):
        """Plot computer boundary channels on the tick index ``k``.

        Channels keep their computer port names (a ``u_cmd`` command is drawn as
        ``u_cmd``); ``u`` here is the computer's input vector, not the plant command.
        """
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import STEP_ABSCISSA_LABEL

        if signals is None:
            signals = tuple(self.computer.signals.keys())

        return self.plot_view(
            self.computer.as_trajectory(),
            sys=self.hybrid.computer.diagram if self.hybrid is not None else None,
            signals=signals,
            show=show,
            backend=backend,
            abscissa_label=STEP_ABSCISSA_LABEL,
            control_key=None,
            **kwargs,
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def plot_view(
        self,
        traj,
        *,
        sys,
        signals,
        show,
        backend,
        abscissa_label,
        control_key,
        **kwargs,
    ):
        """Plot one view, the plant's or the computer's, on its abscissa."""
        # Lazy import: the graphics load on first use, not with the core
        from minilink.graphical.signals.time_signals import (
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        if sys is None:
            sys = self.hybrid.plant if self.hybrid is not None else self

        # The default channels: the system's own, or the view's x and u; u always
        # drawn when the view has one
        if signals is None:
            if isinstance(sys, System):
                signals = resolve_plot_signals(sys)
            else:
                names = []
                if traj.x.shape[0]:
                    names.append("x")
                if traj.u.shape[0]:
                    names.append("u")
                signals = tuple(names)
            if traj.u.shape[0]:
                signals = tuple(dict.fromkeys((*signals, "u")))

        plot_signals = self.plot_signal_names(signals, control_key=control_key)
        label = abscissa_label or TIME_ABSCISSA_LABEL

        return plot_time_signals(
            sys,
            traj,
            signals=plot_signals,
            abscissa_label=label,
            backend=backend,
            show=show,
            **kwargs,
        )

    @staticmethod
    def plot_signal_names(
        signals: tuple[str, ...],
        *,
        control_key: str | None,
    ) -> tuple[str, ...]:
        """Map boundary control names routed into ``Trajectory.u`` for plotting."""
        if control_key is None:
            return signals

        plot_key = "u"
        if control_key == plot_key:
            return signals

        return tuple(plot_key if name == control_key else name for name in signals)

    def plant_control_signal_key(self) -> str | None:
        """The computer port that commands the plant, ``None`` without a diagram."""
        if self.hybrid is None:
            return None

        for conn in self.hybrid.connections:
            if conn.direction == "computer_to_plant":
                return conn.computer_port

        return None


###############################################################################
# Internal machinery
###############################################################################


def require_hybrid_diagram(hybrid):
    """Refuse anything but a :class:`HybridDiagram`."""
    if not isinstance(hybrid, HybridDiagram):
        raise TypeError(
            f"HybridSimulator requires HybridDiagram, got {type(hybrid).__name__}"
        )


def select_tick_grid(*, t0, tf, dt_base, n_steps):
    """The base period, the tick count (``n_steps`` over ``tf``) and the tick times."""
    t0 = float(t0)
    tf = float(tf)
    if tf <= t0:
        raise ValueError("tf must be greater than t0")
    dt_base = float(dt_base)
    if dt_base <= 0.0:
        raise ValueError("computer.schedule.dt_base must be positive")

    if n_steps is None:
        n_ticks = max(1, int(round((tf - t0) / dt_base)))
    else:
        if int(n_steps) < 1:
            raise ValueError("n_steps must be >= 1")
        derived = max(1, int(round((tf - t0) / dt_base)))
        if int(n_steps) != derived:
            logging.warning(
                "HybridSimulator: n_steps=%s overrides tf-derived tick count %s",
                n_steps,
                derived,
            )
        n_ticks = int(n_steps)

    times = t0 + np.arange(n_ticks) * dt_base

    return dt_base, n_ticks, times


def index_boundary_connections(hybrid: HybridDiagram):
    """The boundary links as port pairs: computer to plant, then plant to computer."""
    c2p = []
    p2c = []
    for conn in hybrid.connections:
        if conn.direction == "computer_to_plant":
            c2p.append((conn.computer_port, conn.plant_port))
        else:
            p2c.append((conn.plant_port, conn.computer_port))

    return tuple(c2p), tuple(p2c)


def sample_plant_outputs(plant_eval, x_plant, u_plant, t, p2c):
    """The plant outputs the computer reads, by computer port, at ``(x, u, t)``."""
    if not p2c:
        return {}

    outs = plant_eval.outputs(x_plant, u_plant, t)

    return {
        computer_port: np.asarray(outs[plant_port], dtype=float).copy()
        for plant_port, computer_port in p2c
    }


def allocate_signal_hist(diagram, p2c, n_ticks):
    """A zero ``(dim, n_ticks)`` history for every computer boundary channel."""
    names = collect_computer_signal_names(diagram, p2c)

    return {name: np.zeros((signal_dim(name, diagram, p2c), n_ticks)) for name in names}


def collect_computer_signal_names(diagram, p2c):
    """The computer's outputs, the ports it samples from the plant, and ``r``."""
    names = set(diagram.outputs)
    for _plant_port, computer_port in p2c:
        names.add(computer_port)
    if "r" in diagram.inputs:
        names.add("r")

    return tuple(sorted(names))


def signal_dim(name, diagram, p2c):
    """The dimension of a boundary channel, read on the computer diagram's ports."""
    if name in diagram.outputs:
        return diagram.outputs[name].dim
    if name in diagram.inputs:
        return diagram.inputs[name].dim
    for plant_port, computer_port in p2c:
        if name == plant_port:
            return diagram.inputs[computer_port].dim
        if name == computer_port:
            return diagram.inputs[computer_port].dim
    raise KeyError(f"Unknown signal {name!r}")


def collect_plant_signal_names(diagram, p2c):
    """The plant's sampled output ports, and ``r`` when the computer has one."""
    names = set()
    if "r" in diagram.inputs:
        names.add("r")
    for plant_port, _computer_port in p2c:
        names.add(plant_port)

    return tuple(sorted(names))


def assemble_computer_u(diagram, forced_u, sample_buffers, p2c):
    """The computer input: the forced vector, the sampled plant outputs on their ports."""
    u = np.asarray(forced_u, dtype=float).reshape(diagram.m).copy()
    for plant_port, computer_port in p2c:
        sl = diagram.get_input_port_slice(computer_port)
        u[sl] = sample_buffers[computer_port]

    return u


def assemble_plant_u(plant, u_nominal, zoh_buffers, c2p):
    """The plant input: the nominal vector, with the held commands on their ports."""
    u = np.asarray(u_nominal, dtype=float).copy()
    for computer_port, plant_port in c2p:
        sl = plant.get_input_port_slice(plant_port)
        u[sl] = zoh_buffers[plant_port]

    return u


def record_plant_samples(
    u_parts,
    signal_parts,
    plant_eval,
    diagram,
    t_seg,
    x_seg,
    u_plant,
    u_computer,
    plant_m,
):
    """Append the held input and the channels of each fine plant sample of one tick."""
    for i in range(x_seg.shape[0]):
        t_i = float(t_seg[i])
        x_i = x_seg[i]
        u_parts.append(np.asarray(u_plant, dtype=float).reshape(plant_m))
        outs_fine = plant_eval.outputs(x_i, u_plant, t_i)
        for name in signal_parts:
            if name == "r" and name in diagram.inputs:
                sl = diagram.get_input_port_slice(name)
                signal_parts[name].append(
                    np.asarray(u_computer[sl], dtype=float).reshape(-1)
                )
            elif name in outs_fine:
                signal_parts[name].append(
                    np.asarray(outs_fine[name], dtype=float).reshape(-1)
                )


def record_tick_signals(
    signal_hist,
    diagram,
    u_computer,
    outs,
    sample_buffers,
    tick_idx,
    *,
    plant_port_aliases,
):
    """Write one tick's boundary channels into their histories."""
    for name in signal_hist:
        if name in outs:
            values = np.asarray(outs[name], dtype=float).reshape(-1)
        elif name in sample_buffers:
            values = np.asarray(sample_buffers[name], dtype=float).reshape(-1)
        elif name in plant_port_aliases:
            computer_port = plant_port_aliases[name]
            values = np.asarray(sample_buffers[computer_port], dtype=float).reshape(-1)
        elif name in diagram.inputs:
            sl = diagram.get_input_port_slice(name)
            values = np.asarray(u_computer[sl], dtype=float).reshape(-1)
        else:
            continue

        signal_hist[name][:, tick_idx] = values


def require_views(computer, plant):
    """Refuse views other than a ``StepRollout`` computer and a ``Trajectory`` plant."""
    if not isinstance(computer, StepRollout):
        raise TypeError("computer must be a StepRollout")
    if not isinstance(plant, Trajectory):
        raise TypeError("plant must be a Trajectory")
