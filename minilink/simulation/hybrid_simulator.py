"""
Hybrid simulation: scheduled computer ticks + ZOH continuous plant integration.

Public API mirrors :class:`~minilink.simulation.simulator.Simulator`:
construct with ``t0`` / ``tf``, then :meth:`solve` or :meth:`solve_forced`.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from types import MappingProxyType

import numpy as np

from minilink.core.backends import BACKEND_NUMPY
from minilink.core.hybrid_diagram import HybridDiagram
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
        RK4 sub-step inside each base tick for the plant.
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
        n_plant = self.plant.n
        n_ticks = self.n_ticks

        k_hist = np.zeros(n_ticks, dtype=float)
        t_hist = np.zeros(n_ticks, dtype=float)
        x_computer_hist = np.zeros((n_computer, n_ticks))
        x_plant_hist = np.zeros((n_plant, n_ticks))
        signal_hist = _allocate_signal_hist(diagram, self._p2c, n_ticks)
        plant_port_aliases = {
            plant_port: computer_port for plant_port, computer_port in self._p2c
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
            x_plant = self.plant_eval.integrate_zoh(
                x_plant,
                u_plant,
                t_k,
                self.dt_base,
                dt_inner=self.plant_dt_inner,
            )
            sample_buffers = _sample_plant_outputs(
                self.plant_eval,
                x_plant,
                u_plant,
                t_k + self.dt_base,
                self._p2c,
            )

            k_hist[tick_idx] = k
            t_hist[tick_idx] = t_k
            x_computer_hist[:, tick_idx] = computer.x
            x_plant_hist[:, tick_idx] = x_plant
            _record_tick_signals(
                signal_hist,
                diagram,
                u_computer,
                outs,
                sample_buffers,
                x_plant,
                tick_idx,
                plant_port_aliases=plant_port_aliases,
            )

        result = HybridSimResult(
            k=k_hist,
            t=t_hist,
            x_computer=x_computer_hist,
            x_plant=x_plant_hist,
            signals=signal_hist,
            hybrid=self.hybrid,
        )
        self.last_result = result
        return result


@dataclass(frozen=True)
class HybridSimResult:
    """
    Hybrid rollout sampled once per base tick.

    Parameters
    ----------
    k : np.ndarray
        Integer step index at tick start, shape ``(N,)``.
    t : np.ndarray
        Wall time ``t0 + k * dt_base``, shape ``(N,)``.
    x_computer : np.ndarray
        Computer state after each tick, shape ``(n_computer, N)``.
    x_plant : np.ndarray
        Plant state after each tick, shape ``(n_plant, N)``.
    signals : dict of str to np.ndarray
        Named boundary channels, each shape ``(dim, N)``.
    hybrid : HybridDiagram, optional
        Source diagram for plot labels (optional).
    """

    k: np.ndarray
    t: np.ndarray
    x_computer: np.ndarray
    x_plant: np.ndarray
    signals: dict[str, np.ndarray] = field(default_factory=dict)
    hybrid: HybridDiagram | None = field(default=None, compare=False, repr=False)

    def __post_init__(self) -> None:
        k = np.asarray(self.k, dtype=float).reshape(-1).copy()
        t = np.asarray(self.t, dtype=float).reshape(-1).copy()
        x_computer = np.asarray(self.x_computer, dtype=float).copy()
        x_plant = np.asarray(self.x_plant, dtype=float).copy()

        if k.ndim != 1 or t.ndim != 1:
            raise ValueError("k and t must be 1-D arrays")
        if k.size == 0:
            raise ValueError("HybridSimResult must contain at least one sample")
        if t.size != k.size:
            raise ValueError("t must have the same length as k")
        if x_computer.ndim != 2 or x_computer.shape[1] != k.size:
            raise ValueError("x_computer must have shape (n_computer, N)")
        if x_plant.ndim != 2 or x_plant.shape[1] != k.size:
            raise ValueError("x_plant must have shape (n_plant, N)")

        signals = {}
        for name, values in dict(self.signals).items():
            arr = np.asarray(values, dtype=float).copy()
            if arr.ndim != 2 or arr.shape[1] != k.size:
                raise ValueError(
                    f"Signal {name!r} must have shape (dim, N) where N == k.size"
                )
            signals[name] = arr

        object.__setattr__(self, "k", k)
        object.__setattr__(self, "t", t)
        object.__setattr__(self, "x_computer", x_computer)
        object.__setattr__(self, "x_plant", x_plant)
        object.__setattr__(self, "signals", MappingProxyType(signals))

    @property
    def n_samples(self) -> int:
        return int(self.k.size)

    def as_trajectory_plant(self) -> Trajectory:
        """Plant-side view as :class:`~minilink.core.trajectory.Trajectory`."""
        u = self.signals.get("u_cmd")
        if u is None:
            u = np.zeros((0, self.n_samples))
        return Trajectory(
            t=self.t,
            x=self.x_plant,
            u=u,
            signals={key: val for key, val in self.signals.items() if key != "u_cmd"},
        )

    def plot(
        self,
        *,
        signals: tuple[str, ...] | None = None,
        abscissa: str = "t",
        show: bool = True,
        backend="matplotlib",
        **kwargs,
    ):
        """Plot named channels via :func:`~minilink.graphical.signals.time_signals.plot_time_signals`."""
        from minilink.graphical.signals.time_signals import (
            STEP_ABSCISSA_LABEL,
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
        )

        if signals is None:
            signals = tuple(self.signals.keys())

        traj_signals = dict(self.signals)
        if "x_plant" in signals:
            traj_signals["x_plant"] = self.x_plant
        if "x_computer" in signals:
            traj_signals["x_computer"] = self.x_computer

        u = traj_signals.get("u_cmd", np.zeros((0, self.n_samples)))
        plot_t = self.t if abscissa == "t" else self.k
        traj = Trajectory(t=plot_t, x=self.x_plant, u=u, signals=traj_signals)

        abscissa_label = TIME_ABSCISSA_LABEL if abscissa == "t" else STEP_ABSCISSA_LABEL
        sys = self.hybrid.plant if self.hybrid is not None else self
        return plot_time_signals(
            sys,
            traj,
            signals=signals,
            abscissa_label=abscissa_label,
            backend=backend,
            show=show,
            **kwargs,
        )


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


def _collect_signal_names(diagram, p2c):
    names = set(diagram.outputs)
    for plant_port, computer_port in p2c:
        names.add(computer_port)
        names.add(plant_port)
    if "r" in diagram.inputs:
        names.add("r")
    return tuple(sorted(names))


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


def _allocate_signal_hist(diagram, p2c, n_ticks):
    return {
        name: np.zeros((_signal_dim(name, diagram, p2c), n_ticks))
        for name in _collect_signal_names(diagram, p2c)
    }


def _record_tick_signals(
    signal_hist,
    diagram,
    u_computer,
    outs,
    sample_buffers,
    x_plant,
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
        elif name == "x_plant":
            values = np.asarray(x_plant, dtype=float).reshape(-1)
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
