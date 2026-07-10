"""
Hybrid simulation: scheduled computer ticks + ZOH continuous plant integration.

Public API mirrors :class:`~minilink.simulation.simulator.Simulator`:
construct with ``t0`` / ``tf``, then :meth:`solve` or :meth:`solve_forced`.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.core.step_rollout import StepRollout
from minilink.core.trajectory import Trajectory
from minilink.simulation.computer import Computer

# Public API


class HybridSimulator:
    """
    Advance a :class:`HybridDiagram` on the computer base tick grid.

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
    compile_backend : str
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
        compile_backend=BACKEND_NUMPY,
        verbose=False,
    ):
        if not isinstance(hybrid, HybridDiagram):
            raise TypeError(
                f"HybridSimulator requires HybridDiagram, got {type(hybrid).__name__}"
            )

        self.verbose = verbose
        self.hybrid = hybrid
        self.computer: Computer = hybrid.computer
        self.plant = hybrid.plant
        self.plant_dt_inner = plant_dt_inner
        self.t0 = float(t0)
        self.tf = float(tf)

        self.dt_base, self.n_ticks, self.times = _select_tick_grid(
            t0=self.t0,
            tf=self.tf,
            dt_base=self.computer.schedule.dt_base,
            n_steps=n_steps,
        )

        self.computer.compile()
        self.plant_eval = self.plant.compile(backend=compile_backend)

        self.x0_plant = _validate_x0(
            self.plant.x0 if x0_plant is None else x0_plant,
            self.plant.n,
            label="x0_plant",
        )
        self.x0_computer = _validate_x0(
            self.computer.diagram.x0 if x0_computer is None else x0_computer,
            self.computer.diagram.n,
            label="x0_computer",
        )

        self._c2p, self._p2c = _index_boundary_connections(hybrid)
        self.last_result = None

        if self.verbose:
            print(
                f"HybridSimulator: dt_base={self.dt_base}, n_ticks={self.n_ticks}, "
                f"t=[{self.t0}, {self.tf}]"
            )

    def solve(self):
        """Simulate with nominal computer-boundary inputs."""
        u_nominal = np.asarray(
            self.computer.diagram.get_u_from_input_ports(), dtype=float
        )
        if self.computer.diagram.m == 0:
            u_forced = np.zeros((0, self.n_ticks))
        else:
            u_forced = np.repeat(u_nominal.reshape(-1, 1), self.n_ticks, axis=1)
        return self._run(u_forced)

    def solve_forced(self, u, *, input_port_id=None):
        """
        Simulate with prescribed computer-boundary input.

        ``u`` may be a callable ``u(t)``, constant vector, or sampled matrix
        on the tick grid (``m × n_ticks``).
        """
        u_traj = _coerce_forced_input(
            u,
            self.computer.diagram,
            self.n_ticks,
            self.times,
            input_port_id=input_port_id,
        )
        return self._run(u_traj)

    def _run(self, u_traj: np.ndarray) -> HybridSimResult:
        computer = self.computer
        diagram = computer.diagram
        computer.reset(self.x0_computer)

        x_plant = self.x0_plant.copy()
        u_plant_nominal = np.asarray(self.plant.get_u_from_input_ports(), dtype=float)
        zoh_buffers: dict[str, np.ndarray] = {}
        sample_buffers = _sample_plant_outputs(
            self.plant_eval,
            x_plant,
            u_plant_nominal,
            self.t0,
            self._p2c,
        )

        n_computer = diagram.n
        n_ticks = self.n_ticks
        plant_m = self.plant.m

        k_hist = np.zeros(n_ticks, dtype=float)
        x_computer_hist = np.zeros((n_computer, n_ticks))
        computer_signal_hist = _allocate_signal_hist(
            diagram, self._p2c, n_ticks, plant_signals=False
        )
        plant_port_aliases = {
            plant_port: computer_port for plant_port, computer_port in self._p2c
        }

        plant_signal_names = _collect_plant_signal_names(diagram, self._p2c)
        t_plant_parts: list[np.ndarray] = []
        x_plant_parts: list[np.ndarray] = []
        u_plant_parts: list[np.ndarray] = []
        plant_signal_parts = {
            name: [] for name in plant_signal_names
        }

        for tick_idx in range(n_ticks):
            k = computer.k
            t_k = self.t0 + k * self.dt_base
            u_computer = _assemble_computer_u(
                diagram,
                u_traj[:, tick_idx],
                sample_buffers,
                self._p2c,
            )
            outs = computer.tick(u_computer)

            for computer_port, plant_port in self._c2p:
                zoh_buffers[plant_port] = np.asarray(
                    outs[computer_port], dtype=float
                ).copy()

            u_plant = _assemble_plant_u(
                self.plant,
                u_plant_nominal,
                zoh_buffers,
                self._c2p,
            )
            t_seg, x_seg = self.plant_eval.integrate_zoh_rollout(
                x_plant,
                u_plant,
                t_k,
                self.dt_base,
                dt_inner=self.plant_dt_inner,
            )
            x_plant = x_seg[-1]
            sample_buffers = _sample_plant_outputs(
                self.plant_eval,
                x_plant,
                u_plant,
                t_k + self.dt_base,
                self._p2c,
            )

            if tick_idx == 0:
                t_append = t_seg
                x_append = x_seg
            else:
                t_append = t_seg[1:]
                x_append = x_seg[1:]

            t_plant_parts.append(t_append)
            x_plant_parts.append(x_append)
            for i in range(x_append.shape[0]):
                t_i = float(t_append[i])
                x_i = x_append[i]
                u_plant_parts.append(np.asarray(u_plant, dtype=float).reshape(plant_m))
                outs_fine = self.plant_eval.outputs(x_i, u_plant, t_i)
                for name in plant_signal_names:
                    if name == "r" and name in diagram.inputs:
                        sl = diagram.get_input_port_slice(name)
                        plant_signal_parts[name].append(
                            np.asarray(u_computer[sl], dtype=float).reshape(-1)
                        )
                    elif name in outs_fine:
                        plant_signal_parts[name].append(
                            np.asarray(outs_fine[name], dtype=float).reshape(-1)
                        )

            k_hist[tick_idx] = k
            x_computer_hist[:, tick_idx] = computer.x
            _record_tick_signals(
                computer_signal_hist,
                diagram,
                u_computer,
                outs,
                sample_buffers,
                tick_idx,
                plant_port_aliases=plant_port_aliases,
            )

        t_plant = np.concatenate(t_plant_parts)
        x_plant_hist = np.vstack(x_plant_parts).T
        u_plant_fine = np.column_stack(u_plant_parts) if u_plant_parts else np.zeros((plant_m, 0))
        plant_signals = {}
        for name, parts in plant_signal_parts.items():
            if not parts:
                continue
            plant_signals[name] = np.column_stack(parts)

        computer_signals = {
            name: values
            for name, values in computer_signal_hist.items()
            if name not in {"x", "u"}
        }

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
        if not isinstance(self.computer, StepRollout):
            raise TypeError("computer must be a StepRollout")
        if not isinstance(self.plant, Trajectory):
            raise TypeError("plant must be a Trajectory")

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
        return self._plot_view(
            self.plant,
            sys=self.hybrid.plant if self.hybrid is not None else None,
            signals=signals,
            show=show,
            backend=backend,
            abscissa_label=None,
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
        """Plot computer boundary channels on the tick index ``k``."""
        from minilink.graphical.signals.time_signals import STEP_ABSCISSA_LABEL

        if signals is None:
            signals = tuple(self.computer.signals.keys())
        return self._plot_view(
            self.computer.as_trajectory(),
            sys=self.hybrid.computer.diagram if self.hybrid is not None else None,
            signals=signals,
            show=show,
            backend=backend,
            abscissa_label=STEP_ABSCISSA_LABEL,
            **kwargs,
        )

    def _plot_view(
        self,
        traj,
        *,
        sys,
        signals,
        show,
        backend,
        abscissa_label,
        **kwargs,
    ):
        from minilink.graphical.signals.time_signals import (
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
        )

        if signals is None:
            names = list(traj.signals.keys())
            if traj.u.shape[0]:
                names = ["u", *names]
            if traj.x.shape[0]:
                names = ["x", *names]
            signals = tuple(dict.fromkeys(names))

        plot_signals = self._normalize_plot_signal_names(
            signals,
            control_key=self._plant_control_signal_key(),
        )
        label = abscissa_label or TIME_ABSCISSA_LABEL
        if sys is None:
            sys = self.hybrid.plant if self.hybrid is not None else self
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
    def _normalize_plot_signal_names(
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

    def _plant_control_signal_key(self) -> str | None:
        if self.hybrid is None:
            return None
        for conn in self.hybrid.connections:
            if conn.direction == "computer_to_plant":
                return conn.computer_port
        return None


# Internal machinery


def _select_tick_grid(*, t0, tf, dt_base, n_steps):
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


def _validate_x0(x0, n, *, label):
    x0_arr = np.asarray(x0, dtype=float).reshape(-1)
    if x0_arr.shape[0] != n:
        raise ValueError(f"{label} must have shape ({n},)")
    if not np.all(np.isfinite(x0_arr)):
        raise ValueError(f"{label} must contain only finite values")
    return x0_arr


def _index_boundary_connections(hybrid: HybridDiagram):
    c2p = []
    p2c = []
    for conn in hybrid.connections:
        if conn.direction == "computer_to_plant":
            c2p.append((conn.computer_port, conn.plant_port))
        else:
            p2c.append((conn.plant_port, conn.computer_port))
    return tuple(c2p), tuple(p2c)


def _sample_plant_outputs(plant_eval, x_plant, u_plant, t, p2c):
    if not p2c:
        return {}
    outs = plant_eval.outputs(x_plant, u_plant, t)
    return {
        computer_port: np.asarray(outs[plant_port], dtype=float).copy()
        for plant_port, computer_port in p2c
    }


def _assemble_plant_u(plant, u_nominal, zoh_buffers, c2p):
    u = np.asarray(u_nominal, dtype=float).copy()
    for computer_port, plant_port in c2p:
        sl = plant.get_input_port_slice(plant_port)
        u[sl] = zoh_buffers[plant_port]
    return u


def _assemble_computer_u(diagram, forced_u, sample_buffers, p2c):
    u = np.asarray(forced_u, dtype=float).reshape(diagram.m).copy()
    for plant_port, computer_port in p2c:
        sl = diagram.get_input_port_slice(computer_port)
        u[sl] = sample_buffers[computer_port]
    return u


def _collect_computer_signal_names(diagram, p2c):
    names = set(diagram.outputs)
    for _plant_port, computer_port in p2c:
        names.add(computer_port)
    if "r" in diagram.inputs:
        names.add("r")
    return tuple(sorted(names))


def _collect_plant_signal_names(diagram, p2c):
    names = set()
    if "r" in diagram.inputs:
        names.add("r")
    for plant_port, _computer_port in p2c:
        names.add(plant_port)
    return tuple(sorted(names))


def _collect_signal_names(diagram, p2c):
    return _collect_computer_signal_names(diagram, p2c)


def _signal_dim(name, diagram, p2c):
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


def _allocate_signal_hist(diagram, p2c, n_ticks, *, plant_signals=True):
    if plant_signals:
        names = _collect_signal_names(diagram, p2c)
    else:
        names = _collect_computer_signal_names(diagram, p2c)
    return {
        name: np.zeros((_signal_dim(name, diagram, p2c), n_ticks))
        for name in names
    }


def _record_tick_signals(
    signal_hist,
    diagram,
    u_computer,
    outs,
    sample_buffers,
    tick_idx,
    *,
    plant_port_aliases,
):
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


def _coerce_forced_input(u, diagram, n_ticks, times, *, input_port_id=None):
    m = diagram.m
    if input_port_id is None:
        return _coerce_full_forced_input(u, m, n_ticks, times)

    port = diagram.inputs[input_port_id]
    port_slice = diagram.get_input_port_slice(input_port_id)
    u_nominal = diagram.get_u_from_input_ports().reshape(m, 1)
    u_traj = np.repeat(u_nominal, n_ticks, axis=1)
    u_traj[port_slice, :] = _coerce_forced_signal(
        u,
        port.dim,
        n_ticks,
        times,
        label=f"u for input port '{input_port_id}'",
    )
    return u_traj


def _coerce_full_forced_input(u, m, n_ticks, times):
    if m == 0:
        return np.zeros((0, n_ticks))
    return _coerce_forced_signal(u, m, n_ticks, times, label="u")


def _coerce_forced_signal(data, expected_dim, n_ticks, times, *, label):
    if callable(data):
        return _sample_forced_callable(data, expected_dim, times, label=label)

    arr = np.asarray(data, dtype=float)
    expected_shape = (expected_dim, n_ticks)

    if arr.ndim == 0:
        if expected_dim != 1:
            raise ValueError(f"{label} must have shape {expected_shape}")
        return np.full((1, n_ticks), float(arr), dtype=float)

    if arr.ndim == 1:
        if expected_dim == 1 and arr.shape[0] == n_ticks:
            return arr.reshape(1, n_ticks)
        if arr.shape[0] == expected_dim:
            return np.repeat(arr.reshape(expected_dim, 1), n_ticks, axis=1)
        raise ValueError(f"{label} must have shape {expected_shape}")

    if arr.ndim == 2 and arr.shape == expected_shape:
        return arr

    raise ValueError(f"{label} must have shape {expected_shape}")


def _sample_forced_callable(fn, expected_dim, times, *, label):
    samples = np.zeros((expected_dim, times.size), dtype=float)
    for i, ti in enumerate(times):
        value = np.asarray(fn(float(ti)), dtype=float)
        if expected_dim == 1 and value.ndim == 0:
            samples[0, i] = float(value)
            continue
        if value.reshape(-1).shape[0] != expected_dim:
            raise ValueError(f"{label} callable returned wrong dimension at t={ti}")
        samples[:, i] = value.reshape(expected_dim)
    return samples
