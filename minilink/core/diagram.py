"""
Block diagrams: systems built by connecting other systems.

A :class:`DiagramSystem` is a :class:`~minilink.core.system.DynamicSystem`,
so diagrams nest, simulate, and compile like any other continuous system.
"""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.system import DynamicSystem, System
from minilink.core.trajectory import Trajectory
from minilink.core.wiring import WiredDiagramMixin, validate_diagram_params

__all__ = ["DiagramSystem", "validate_diagram_params"]


class DiagramSystem(WiredDiagramMixin, DynamicSystem):
    """
    A system composed of subsystems connected through their named ports.

    The diagram state stacks the subsystem states in insertion order,

        x = [x_1; x_2; ...; x_k],        dx_i = f_i(x_i, u_i, t; p_i),

    where each local input ``u_i`` is gathered from the output ports connected
    to subsystem ``i``, from the diagram boundary inputs, or from the port's
    constant nominal value when unconnected. Diagram-level params are nested
    by subsystem id: ``{"sys": {...}, "ctl": {...}}``. Shortcut-built
    diagrams default to role ids ``ref``, ``ctl``, and ``sys``.

    Build diagrams with :meth:`add_subsystem` + :meth:`connect` (canonical
    wiring for any topology) or with the shortcuts ``+``, ``>>``, ``@``, and
    :meth:`autowire` from :mod:`minilink.core.composition`.

    This class implements the wiring contract and a simple recursive
    evaluation of ``f`` used as the reference; :meth:`compile` produces the
    fast :class:`~minilink.core.compile.execution_plan.ExecutionPlan`-based
    evaluator used by simulation and optimization.
    """

    def __init__(self):
        # Registry before System.__init__: its params setter runs during shell init.
        self.subsystems = {}
        self.connections = {}
        System.__init__(self, 0)
        self._init_wiring(name="Diagram")

    def f(self, x, u, t=0, params=None):
        """
        Stacked state derivative ``dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...]``.

        This is the interpreted reference implementation; :meth:`compile`
        produces the fast equivalent used by simulation and optimization.
        """
        validate_diagram_params(params, self.subsystems)

        dx_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            if not isinstance(subsystem, DynamicSystem):
                continue
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self._subsystem_params(params, sys_id)

            dx_pieces.append(subsystem.f(local_x, local_u, t, local_params))

        xp = array_module(x, u, *dx_pieces)
        if not dx_pieces:
            return xp.array([])
        return xp.concatenate([xp.asarray(dx).reshape(-1) for dx in dx_pieces])

    def compile(self, backend="numpy", bind_params=False, verbose=False):
        """
        Compile the diagram into a stateless evaluator for fast simulation.

        Runs algebraic-loop detection internally; raises RuntimeError if a
        loop is found.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        bind_params : bool, optional
            If ``True``, subsystem ``params`` are deep-copied into the plan at
            compile time (see :func:`minilink.core.compile.compiler.compile_diagram`).
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        NumpyDiagramEvaluator or JaxDiagramEvaluator
        """
        from minilink.core.compile.compiler import compile_diagram

        return compile_diagram(
            self, backend=backend, bind_params=bind_params, verbose=verbose
        )

    def reconstruct_internal_signals(self, traj: Trajectory) -> Trajectory:
        """
        Reconstruct all subsystem output-port trajectories for this diagram.

        Parameters
        ----------
        traj : Trajectory
            State-input trajectory sampled on a time grid.

        Returns
        -------
        Trajectory
            New trajectory enriched with one sampled signal per subsystem
            output port, keyed as ``"sys_id:port_id"``.
        """
        evaluator = self.compile(backend="numpy")
        internal_signals = {}
        for sys_id, subsystem in self.subsystems.items():
            for port_id, port in subsystem.outputs.items():
                internal_signals[f"{sys_id}:{port_id}"] = np.zeros(
                    (port.dim, traj.n_samples)
                )

        for i, t in enumerate(traj.t):
            step_signals = evaluator.compute_internal_signals_dict(
                traj.x[:, i], traj.u[:, i], t
            )
            for key, value in step_signals.items():
                internal_signals[key][:, i] = value

        return traj.with_signals(internal_signals)


if __name__ == "__main__":
    # Hello world: unity-feedback loop  dx = Kp (r - x)
    from minilink.blocks.basic import Integrator
    from minilink.control.output import ProportionalController

    diagram = DiagramSystem()
    diagram.add_subsystem(ProportionalController(), "ctl")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")

    diagram.plot_diagram()

    evaluator = diagram.compile()
    print(evaluator.f(x=np.array([0.5]), u=np.array([1.0]), t=0.0))
