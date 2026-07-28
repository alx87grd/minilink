"""
Hybrid diagrams: scheduled computer side + continuous plant side.

Boundary ports between sides use ZOH (computer → plant) or sample (plant →
computer). Schedule lives on :class:`~minilink.simulation.computer.Computer`.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal

from minilink.core.diagram import DiagramSystem, StepDiagramSystem

if TYPE_CHECKING:
    from minilink.simulation.computer import Computer

_DIRECTION_ALIASES = {
    "step_to_plant": "computer_to_plant",
    "plant_to_step": "plant_to_computer",
}

BoundaryDirection = Literal["computer_to_plant", "plant_to_computer"]


# Public API


@dataclass(frozen=True)
class BoundaryConnection:
    """One channel across the computer ↔ plant boundary."""

    direction: BoundaryDirection
    computer_port: str
    plant_port: str

    def __post_init__(self) -> None:
        direction = _normalize_direction(self.direction)
        object.__setattr__(self, "direction", direction)


@dataclass
class HybridDiagram:
    """
    Two-side hybrid topology: :class:`Computer` + continuous :class:`DiagramSystem`.

    Parameters
    ----------
    computer : Computer
        Scheduled step runtime (diagram + :class:`~minilink.simulation.computer.StepSchedule`).
    plant : DiagramSystem
        Continuous plant diagram.
    connections : list of BoundaryConnection, optional
        Multi-channel boundary wiring.
    """

    computer: Computer
    plant: DiagramSystem
    connections: list[BoundaryConnection] = field(default_factory=list)

    def __post_init__(self) -> None:
        from minilink.simulation.computer import Computer

        if not isinstance(self.computer, Computer):
            raise TypeError(
                f"HybridDiagram requires Computer, got {type(self.computer).__name__}"
            )
        if not isinstance(self.plant, DiagramSystem):
            raise TypeError(
                f"HybridDiagram requires DiagramSystem plant, "
                f"got {type(self.plant).__name__}"
            )
        self.last_result = None
        self.traj = None

    @property
    def rollout(self):
        """Tick-indexed computer view from :attr:`last_result` (``None`` if unset)."""
        if self.last_result is None:
            return None
        return self.last_result.computer

    @classmethod
    def from_diagrams(
        cls,
        step_diagram: StepDiagramSystem,
        plant: DiagramSystem,
        schedule,
        *,
        connections: list[BoundaryConnection] | None = None,
    ) -> HybridDiagram:
        """Wrap ``step_diagram`` and ``schedule`` in a :class:`Computer`."""
        from minilink.simulation.computer import Computer, StepSchedule

        if not isinstance(schedule, StepSchedule):
            schedule = StepSchedule(dt_base=float(schedule))
        computer = Computer(step_diagram, schedule)
        return cls(computer=computer, plant=plant, connections=list(connections or []))

    def _simulator(
        self,
        *,
        t0=0,
        tf=10,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=False,
    ):
        from minilink.simulation.hybrid_simulator import HybridSimulator

        return HybridSimulator(
            self,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        )

    def connect_boundary(
        self,
        *,
        direction: BoundaryDirection | str,
        computer_port: str,
        plant_port: str,
    ) -> None:
        """Append and validate one boundary channel."""
        direction = _normalize_direction(direction)
        conn = BoundaryConnection(
            direction=direction,
            computer_port=computer_port,
            plant_port=plant_port,
        )
        _validate_boundary_connection(self, conn)
        self.connections.append(conn)

    def compute_trajectory(
        self,
        t0=0,
        tf=10,
        *,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=True,
        show=False,
    ):
        """Simulate with nominal boundary inputs on ``[t0, tf]``.

        Parameters
        ----------
        verbose : bool
            Print hybrid simulator setup (default ``True`` for interactive use).

        Returns
        -------
        HybridSimResult
            Stored on :attr:`last_result`; plant view on :attr:`traj`.
        """
        result = self._simulator(
            t0=t0,
            tf=tf,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        ).solve()
        self._cache_result(result)
        if show:
            result.plot()
        return result

    def compute_forced(
        self,
        u,
        *,
        input_port_id=None,
        t0=0,
        tf=10,
        x0_plant=None,
        x0_computer=None,
        n_steps=None,
        plant_dt_inner=None,
        compile_backend="numpy",
        verbose=True,
        show=False,
    ):
        """Simulate with prescribed computer-boundary input on ``[t0, tf]``.

        Parameters
        ----------
        verbose : bool
            Print hybrid simulator setup (default ``True`` for interactive use).

        Returns
        -------
        HybridSimResult
            Stored on :attr:`last_result`; plant view on :attr:`traj`.
        """
        result = self._simulator(
            t0=t0,
            tf=tf,
            x0_plant=x0_plant,
            x0_computer=x0_computer,
            n_steps=n_steps,
            plant_dt_inner=plant_dt_inner,
            compile_backend=compile_backend,
            verbose=verbose,
        ).solve_forced(u, input_port_id=input_port_id)
        self._cache_result(result)
        if show:
            result.plot()
        return result

    def plot_trajectory(
        self,
        traj=None,
        *,
        signals=None,
        abscissa="t",
        show=True,
        backend="matplotlib",
        **kwargs,
    ):
        """
        Plot plant channels (continuous-time view) from the last rollout or result.

        If ``traj`` is omitted and :attr:`traj` is unset, runs :meth:`compute_trajectory`
        first (same convenience pattern as :class:`~minilink.core.facades.SharedSystemFacades`).
        Pass ``traj=result.computer.as_trajectory()`` and use :meth:`HybridSimResult.plot_computer`
        on a :class:`~minilink.simulation.hybrid_simulator.HybridSimResult` for tick-indexed
        computer internals.
        """
        if traj is None:
            if self.last_result is None:
                self.compute_trajectory(show=False, verbose=False)
            return self.last_result.plot(
                signals=signals,
                show=show,
                backend=backend,
                **kwargs,
            )
        from minilink.simulation.hybrid_simulator import HybridSimResult

        if isinstance(traj, HybridSimResult):
            return traj.plot(
                signals=signals,
                show=show,
                backend=backend,
                **kwargs,
            )
        from minilink.graphical.signals.time_signals import (
            TIME_ABSCISSA_LABEL,
            plot_time_signals,
            resolve_plot_signals,
        )

        if signals is None:
            signals = resolve_plot_signals(self.plant)
            if traj.u.shape[0]:
                signals = tuple(dict.fromkeys((*signals, "u")))
        return plot_time_signals(
            self.plant,
            traj,
            signals=signals,
            abscissa_label=TIME_ABSCISSA_LABEL,
            backend=backend,
            show=show,
            **kwargs,
        )

    def animate(
        self,
        traj=None,
        time_factor_video=1.0,
        is_3d=False,
        html: bool | None = None,
        renderer="matplotlib",
        native: bool = True,
        scene_title: str | None = None,
        show: bool = True,
        save: bool = False,
        file_name: str = "Animation",
        camera=None,
        overlays=None,
    ):
        """
        Animate the continuous plant using the fine :attr:`traj` cache.

        Same kwargs as :meth:`~minilink.core.facades.SharedSystemFacades.animate`
        on a flow diagram. Uses :attr:`plant` geometry; ``traj`` defaults to the
        last plant trajectory from :meth:`compute_trajectory` / :meth:`compute_forced`.
        """
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        if traj is None:
            if self.traj is not None:
                traj = self.traj
            elif self.last_result is not None:
                traj = self.last_result.plant
            else:
                self.compute_trajectory(verbose=False)
                traj = self.traj

        resolved_html = prefers_inline_animation() if html is None else html
        from minilink.core.hybrid_composition import refresh_plant_animation_camera

        refresh_plant_animation_camera(self.plant)
        animator = Animator(self.plant)
        show_plot = show and not resolved_html
        return animator.animate_simulation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=resolved_html,
            show=show_plot,
            save=save,
            file_name=file_name,
            renderer=renderer,
            native=native,
            scene_title=scene_title,
            camera=camera,
            overlays=overlays,
        )

    def _cache_result(self, result) -> None:
        """Store full hybrid result and plant trajectory cache."""
        self.last_result = result
        self.traj = result.plant

    def plot_diagram(
        self,
        filename=None,
        show=True,
        show_inline=None,
        show_pdf=None,
        *,
        abstract_boundary=True,
    ):
        """Render Plant + Computer clusters with boundary ZOH/sample edges.

        Display policy matches :meth:`minilink.core.facades.SharedSystemFacades.plot_diagram`
        (notebook SVG; script Matplotlib window; ``show_pdf`` / ``filename`` /
        ``show=False`` overrides). ``abstract_boundary=True`` (default) omits
        external Inputs/Outputs routing nodes and anchors hybrid edges on the
        wired subsystem ports.
        """
        from minilink.graphical.diagrams.hybrid_dot import plot_hybrid_diagram

        return plot_hybrid_diagram(
            self,
            filename=filename,
            show=show,
            show_inline=show_inline,
            show_pdf=show_pdf,
            abstract_boundary=abstract_boundary,
        )

    def _repr_svg_(self):
        """Notebook inline SVG for the hybrid topology (IPython / Jupyter)."""
        from minilink.graphical.diagrams.hybrid_dot import export_hybrid_graphviz
        from minilink.graphical.diagrams.hybrid_topology import build_hybrid_topology

        graph = export_hybrid_graphviz(build_hybrid_topology(self))
        if graph is None:
            return None
        try:
            return graph._repr_image_svg_xml()
        except Exception:
            return None


# Internal machinery


def _normalize_direction(direction: str) -> BoundaryDirection:
    key = _DIRECTION_ALIASES.get(direction, direction)
    if key not in ("computer_to_plant", "plant_to_computer"):
        raise ValueError(
            f"direction must be 'computer_to_plant' or 'plant_to_computer', "
            f"got {direction!r}"
        )
    return key  # type: ignore[return-value]


def _validate_boundary_connection(
    hybrid: HybridDiagram, conn: BoundaryConnection
) -> None:
    diagram = hybrid.computer.diagram
    plant = hybrid.plant

    if conn.direction == "computer_to_plant":
        if conn.computer_port not in diagram.outputs:
            raise ValueError(
                f"Unknown computer output port {conn.computer_port!r}; "
                f"available: {', '.join(diagram.outputs) or '(none)'}"
            )
        if conn.plant_port not in plant.inputs:
            raise ValueError(
                f"Unknown plant input port {conn.plant_port!r}; "
                f"available: {', '.join(plant.inputs) or '(none)'}"
            )
        c_dim = diagram.outputs[conn.computer_port].dim
        p_dim = plant.inputs[conn.plant_port].dim
    else:
        if conn.computer_port not in diagram.inputs:
            raise ValueError(
                f"Unknown computer input port {conn.computer_port!r}; "
                f"available: {', '.join(diagram.inputs) or '(none)'}"
            )
        if conn.plant_port not in plant.outputs:
            raise ValueError(
                f"Unknown plant output port {conn.plant_port!r}; "
                f"available: {', '.join(plant.outputs) or '(none)'}"
            )
        c_dim = diagram.inputs[conn.computer_port].dim
        p_dim = plant.outputs[conn.plant_port].dim

    if c_dim != p_dim:
        raise ValueError(
            f"Boundary dimension mismatch on {conn.computer_port!r} ↔ "
            f"{conn.plant_port!r}: {c_dim} vs {p_dim}"
        )
