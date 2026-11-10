"""Block diagrams: systems built by connecting other systems."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.step_system import StepSystem
from minilink.core.system import DynamicSystem, System
from minilink.core.trajectory import Trajectory
from minilink.core.wiring import WiredDiagramMixin, check_diagram_params

__all__ = ["DiagramSystem", "check_diagram_params"]


###############################################################################
# Public API
###############################################################################


class DiagramSystem(WiredDiagramMixin, DynamicSystem):
    """
    A system composed of subsystems connected through their named ports.

    A diagram is a :class:`~minilink.core.system.DynamicSystem`, so diagrams
    nest, simulate, and compile like any other continuous system. The diagram
    state stacks the subsystem states in insertion order,

        x = [x_1; x_2; ...; x_k],        dx_i = f_i(x_i, u_i, t; p_i),

    where each local input ``u_i`` is gathered from the output ports connected
    to subsystem ``i``, from the diagram boundary inputs, or from the port's
    constant nominal value when unconnected. Diagram-level params are nested
    by subsystem id: ``{"plant": {...}, "controller": {...}}``. ``@`` names
    its blocks ``controller`` and ``plant``; ``+`` and ``>>`` keep the blocks'
    own names.

    Build diagrams with :meth:`add_subsystem` + :meth:`connect` (canonical
    wiring for any topology) or with the shortcuts ``+``, ``>>``, ``@``, and
    :meth:`autowire` from :mod:`minilink.core.operators`.

    This class implements the wiring contract and a simple recursive
    evaluation of ``f`` used as the reference; :meth:`compile` produces the
    fast :class:`~minilink.core.compile.plan.ExecutionPlan`-based
    evaluator used by simulation and optimization.
    """

    def __init__(self):
        # The registry first: the params setter reads it while System.__init__ runs
        self.subsystems = {}
        self.connections = {}

        # The System shell alone: a diagram's ports are the ones it exposes, not
        # DynamicSystem's standard u and y
        System.__init__(self, 0)

        # Wiring tables and the stacked-state layout
        self.init_wiring()

    def add_subsystem(self, sys, sys_id):
        """Add a continuous or static subsystem under a unique id.

        A :class:`StepSystem` has no ``f`` to stack, so it is refused here
        rather than left out of the state derivative.
        """
        require_flow_subsystem(sys, sys_id)

        super().add_subsystem(sys, sys_id)

    def f(self, x, u, t=0, params=None):
        """
        Stacked state derivative ``dx = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); ...]``.

        This is the interpreted reference implementation; :meth:`compile`
        produces the fast equivalent used by simulation and optimization.
        """
        check_diagram_params(params, self.subsystems)

        dx_pieces = []
        for sys_id, subsystem in self.subsystems.items():
            # A static block has no f to stack
            if not isinstance(subsystem, DynamicSystem):
                continue

            # x_i, u_i and p_i of block i, gathered through the wiring
            local_x = self.get_local_state(x, sys_id)
            local_u = self.get_local_input(x, u, t, sys_id, params=params)
            local_params = self.subsystem_params(params, sys_id)

            # ẋ_i = f_i(x_i, u_i, t; p_i)
            dx_pieces.append(subsystem.f(local_x, local_u, t, local_params))

        xp = array_module(x, u, *dx_pieces)

        # ẋ = [f_1(x_1, u_1, t); f_2(x_2, u_2, t); …], one block per subsystem,
        # and empty when no block carries an f
        if not dx_pieces:
            dx = xp.array([])
        else:
            dx = xp.concatenate([xp.asarray(dx_i).reshape(-1) for dx_i in dx_pieces])

        return dx

    def compile(self, *, backend="numpy", verbose=False, flatten=False):
        """
        Compile the diagram into a stateless evaluator for fast simulation.

        Runs algebraic-loop detection internally; raises RuntimeError if a
        loop is found.

        Parameters
        ----------
        backend : str
            ``'numpy'`` (default) or ``'jax'``.
        flatten : bool
            If ``True``, inline nested diagram blocks before compiling (default
            ``False``).
        verbose : bool
            If ``True``, print timed compilation steps.

        Returns
        -------
        DiagramEvaluator
        """
        # Lazy import: a cycle, the compiler reads DiagramSystem from this module
        from minilink.core.compile.compiler import compile_diagram

        return compile_diagram(self, backend=backend, verbose=verbose, flatten=flatten)

    def reconstruct_internal_signals(
        self, traj: Trajectory, ports=None, params=None
    ) -> Trajectory:
        """
        Reconstruct subsystem output-port trajectories for this diagram.

        Parameters
        ----------
        traj : Trajectory
            State-input trajectory sampled on a time grid.
        ports : sequence of str, optional
            The signals to compute, ``"sys_id:port_id"``, an output port or a
            block's whole input (``"plant:u"``), a nested block by its path
            (``"plant/arm:u"``): only those and the blocks they read are
            evaluated. ``None``: every output port.
        params : dict, optional
            The tree the run was made with; ``None`` reads the blocks' current
            numbers, which are the run's only when they were not changed since.

        Returns
        -------
        Trajectory
            New trajectory enriched with one sampled signal per subsystem
            output port, keyed as ``"sys_id:port_id"``.
        """
        if ports is not None:
            return traj.with_signals(
                {name: self.port_samples(traj, name, params) for name in ports}
            )

        evaluator = self.compile(backend="numpy")

        # One table per subsystem output port, keyed "sys_id:port_id"
        internal_signals = {}
        for sys_id, subsystem in self.subsystems.items():
            for port_id, port in subsystem.outputs.items():
                internal_signals[f"{sys_id}:{port_id}"] = np.zeros(
                    (port.dim, traj.n_samples)
                )

        # Every port's value at each sample, from the compiled evaluator
        for i, t in enumerate(traj.t):
            step_signals = evaluator.compute_internal_signals_dict(
                traj.x[:, i], traj.u[:, i], t, params
            )
            for key, value in step_signals.items():
                internal_signals[key][:, i] = value

        return traj.with_signals(internal_signals)

    def port_samples(self, traj: Trajectory, name: str, params=None) -> np.ndarray:
        """
        One signal's value at every sample of ``traj``, ``(dim, N)``.

        ``"block:port"`` is an output port and the blocks it reads; ``"block:u"``
        on a block without an output port named ``u`` is its whole input, every
        port stacked; another input port by its name; a nested block by its path
        (``"plant/arm:u"``), one level at a time. ``params`` is the tree the run
        was made with (``None``: the blocks' current numbers).
        """
        path, _, port_id = name.rpartition(":")
        outer, _, rest = path.partition("/")
        if rest:
            # Down one level: the nested diagram's own run, then its signal
            sys_id = self.subsystem_id(outer)
            inner = self.subsystems[sys_id]
            inner_params = None if params is None else params[sys_id]
            return inner.port_samples(
                subsystem_run(self, outer, traj, params),
                f"{rest}:{port_id}",
                inner_params,
            )

        sys_id, port_id = signal_port_of(self, name)
        block = self.subsystems[sys_id]

        # An output port's value; else the block's whole input, or one input port
        if port_id in block.outputs:
            values = [
                self.compute_subsystem_output_port(
                    traj.x[:, i], traj.u[:, i], t, sys_id, port_id, params=params
                )
                for i, t in enumerate(traj.t)
            ]
        else:
            whole = port_id == "u"
            port_slice = slice(None) if whole else block.get_input_port_slice(port_id)
            values = [
                self.get_local_input(
                    traj.x[:, i], traj.u[:, i], t, sys_id, params=params
                )[port_slice]
                for i, t in enumerate(traj.t)
            ]

        return np.stack(
            [np.asarray(v, dtype=float).reshape(-1) for v in values], axis=1
        )


###############################################################################
# Internal machinery
###############################################################################


def require_flow_subsystem(sys, sys_id):
    """Refuse a :class:`StepSystem` in a flow diagram: it has no ``f`` to stack."""
    if isinstance(sys, StepSystem):
        raise TypeError(
            f"{type(sys).__name__} {sys_id!r} is a StepSystem and cannot join "
            "a flow DiagramSystem: a StepSystem belongs in a StepDiagramSystem, "
            "or on a clock in a sampled loop: block % dt @ plant"
        )


def subsystem_run(diagram, path, traj, params=None) -> Trajectory:
    """
    The run of one block of ``diagram`` as the block saw it, reconstructed from the diagram's run.

    ``x`` is the block's own state and ``u`` the input it received from the
    diagram (connected signals, boundary inputs, or nominal values), at each
    sample of ``traj``; the input is recomputed from the stored states on
    ``params``, the tree the run was made with (``None``: the blocks' current
    numbers). ``path`` is a block id, or ``"a/b"`` for a block inside a nested
    diagram, one level at a time.
    """
    sys_id, _, rest = path.partition("/")
    sys_id = diagram.subsystem_id(sys_id)
    subsystem = diagram.subsystems[sys_id]

    # The block's own state, and the input the diagram fed it, at each sample
    x = np.zeros((subsystem.n, traj.n_samples))
    u = np.zeros((subsystem.m, traj.n_samples))
    for k, t in enumerate(traj.t):
        x[:, k] = diagram.get_local_state(traj.x[:, k], sys_id)
        u[:, k] = diagram.get_local_input(
            traj.x[:, k], traj.u[:, k], t, sys_id, params=params
        )
    run = Trajectory(t=traj.t, x=x, u=u)
    inner_params = None if params is None else params[sys_id]

    return run if not rest else subsystem_run(subsystem, rest, run, inner_params)


def subsystem_roles(diagram, recorded=(), offset=0, prefix=""):
    """
    Every block's layout in a diagram's run, by id: its state slice, and its recorded input signal or ``None``.

    A block inside a nested diagram is keyed by its path (``"inner/plant"``),
    its slice offset into the outer state; ``recorded`` names the signals the
    run kept, the block's input among them as ``"plant:u"``.
    """
    roles = {}
    for sys_id, block in diagram.subsystems.items():
        start, end = diagram.state_index[sys_id]
        path = f"{prefix}{sys_id}"
        u_name = f"{path}:u"
        roles[path] = {
            "x": (offset + start, offset + end),
            "u": u_name if u_name in recorded else None,
        }
        if hasattr(block, "state_index"):
            roles.update(subsystem_roles(block, recorded, offset + start, f"{path}/"))

    return roles


if __name__ == "__main__":
    # Hello world: unity-feedback loop  dx = Kp (r - x)
    from minilink.blocks.basic import Integrator
    from minilink.control.siso import P

    diagram = DiagramSystem()
    diagram.add_subsystem(P(ports="reference"), "controller")
    diagram.add_subsystem(Integrator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "controller", "r")
    diagram.connect("plant", "y", "controller", "y")
    diagram.connect("controller", "u", "plant", "u")

    diagram.plot_diagram()

    evaluator = diagram.compile()
    print(evaluator.f(x=np.array([0.5]), u=np.array([1.0]), t=0.0))


def signal_port_of(diagram, name):
    """``(sys_id, port_id)`` of a signal named ``"sys_id:port_id"``, an output or an input port, refused unless the diagram has it."""
    sys_id, _, port_id = name.partition(":")
    subsystem = diagram.subsystems.get(sys_id)
    if subsystem is None or (
        port_id not in subsystem.outputs
        and port_id not in subsystem.inputs
        and port_id != "u"
    ):
        known = [
            f"{block_id}:{port}"
            for block_id, block in diagram.subsystems.items()
            for port in (*block.outputs, *block.inputs)
        ]
        raise KeyError(f"no signal {name!r} in {diagram.name}; its signals are {known}")

    return sys_id, port_id
