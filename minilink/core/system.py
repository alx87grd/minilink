"""
The System contract: the base class every block, plant, and controller extends.

A system is a named set of input/output ports around the two governing
equations

    dx = f(x, u, t; p)        state derivative
    y  = h(x, u, t; p)        output

Signal and port metadata live in :mod:`minilink.core.signals`; user shortcut
methods (``compute_trajectory``, ``plot_*``, ``animate``, ``modal_analysis``, ...)
live on the :class:`~minilink.core.facades.SystemFacades` mixin.
"""

from typing import TYPE_CHECKING

import numpy as np

from minilink.core.facades import SystemFacades
from minilink.core.signals import InputPort, OutputPort, VectorSignal

if TYPE_CHECKING:
    from minilink.core.diagram import DiagramSystem


class System(SystemFacades):
    """
    Base class describing a dynamical input-output system

        dx = f(x, u, t; p)
        y  = h(x, u, t; p)

    A :class:`System` serves several roles at once:

    - **Core dynamical contract**: it defines a functional modeling interface
      through :meth:`f` and :meth:`h`, intended to describe the system as a
      function of ``(x, u, t, params)``.
    - **Structural model description**: it stores dimensions, ports, state
      metadata, labels, units, bounds, and nominal values.
    - **Model defaults and metadata**: it carries default parameters
      (:attr:`params`), default initial condition (:attr:`x0`), and solver
      hints (:attr:`solver_info`).
    - **Visualization contract**: it may describe forward-kinematic geometry
      for rendering and animation (API still under graphical/animation
      review). Camera hints (``camera_*`` attributes) configure the animator
      boundary resolver; pass ``animate(camera=...)`` for custom views.
    - **User shortcut façade**: it exposes convenience methods such as
      :meth:`compile`, :meth:`compute_trajectory`, :meth:`render`,
      :meth:`animate`, and :meth:`game`, defined on the
      :class:`~minilink.core.facades.SystemFacades` mixin so this module
      stays focused on the mathematical, structural, and visualization
      contracts. Camera hints (``camera_target``, ``camera_plot_axes``,
      ``camera_scale``, ``camera_follow_frame``) are resolved by the animator;
      custom views use ``animate(camera=...)``.

    Notes on dynamics and purity
    ----------------------------
    The intended dynamical contract is **functional/stateless in intent**:
    overridden :meth:`f` and :meth:`h` should behave as functions of
    ``(x, u, t, params)`` only. Python cannot enforce purity, so users should
    avoid relying on hidden mutable instance state inside dynamics or output
    functions. The object itself still stores model defaults, metadata, and
    user convenience state.
    """

    #: Opt-in swappable look: a callable ``(plant) -> dict[str, list[prim]]`` or
    #: ``None``. ``get_kinematic_geometry`` delegates to ``skin``; ``None`` means
    #: no skin (empty geometry). Set per instance/class to swap a look without
    #: touching ``f``/``tf`` (``skin`` is to ``get_kinematic_geometry`` as
    #: ``params`` is to ``f``).
    skin = None

    def __init__(self, n=0):
        """
        Initialize the system with ``n`` continuous states.

        Systems start with no input or output ports. Add ports explicitly with
        :meth:`add_input_port` and :meth:`add_output_port`; the input and
        output dimensions :attr:`m` and :attr:`p` are derived from the ports.
        """
        # Structural model description
        self.n = int(n)
        if self.n < 0:
            raise ValueError("System dimension n must be nonnegative")

        # Human-readable identifier
        self.name = "System"

        # Optional diagram subsystem key override (see composition shortcuts).
        self.id = None

        # State metadata
        self.state = VectorSignal("x", dim=n)

        # Port structure
        self.inputs = {}
        self.outputs = {}

        # Model defaults and metadata
        # Default initial condition used by convenience simulation paths.
        self.x0 = np.zeros(self.n)

        # Default model parameter set. Explicit ``params=...`` arguments
        # passed to ``f`` / ``h`` override this default.
        self.params = {}

        # Solver hints used by high-level simulation shortcuts.
        self.solver_info = {
            "continuous_time_equation": True,
            "smallest_time_constant": 0.001,
            "discontinuous_behavior": False,  # Will use a fixed time step
            "discrete_time_period": None,
            "require_building": False,  # If True, build before simulating
        }

        # Runtime convenience cache for the last trajectory produced by
        # ``compute_trajectory``.
        self.traj = None

        # Standard camera hints (resolved by ``Animator`` via ``camera.py``).
        self.camera_target = np.zeros(3, dtype=float)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 10.0
        # Camera hints read by the ``Animator`` camera resolver.
        # ``camera_follow_frame`` is a ``tf`` key to track (or ``None`` for a
        # fixed view); ``camera_priority`` tie-breaks when several hint-carrying
        # drawables exist.
        self.camera_follow_frame = None
        self.camera_priority = 0.0

    # Core Dynamical Contract

    def f(self, x, u, t=0, params=None):
        """
        State derivative ``dx = f(x, u, t; p)``.

        Subclasses should treat this as a function of ``(x, u, t, params)``
        only; the library does not verify purity (see the class docstring).

        Parameters
        ----------
        x : array of shape (n,)
        u : array of shape (m,)
        t : float
        params : dict, optional
            ``None`` means "use ``self.params``".

        Returns
        -------
        dx : array of shape (n,)
        """
        dx = np.zeros(self.n)
        return dx

    def h(self, x, u, t=0, params=None):
        """
        Output ``y = h(x, u, t; p)``.

        Same purity contract as :meth:`f` (convention only; not enforced).

        Parameters
        ----------
        x : array of shape (n,)
        u : array of shape (m,)
        t : float
        params : dict, optional

        Returns
        -------
        y : array of shape (p,)
        """
        y = np.zeros(self.p)
        return y

    def compute_state(self, x, u, t=0, params=None):
        """Output helper returning the state vector directly (``y = x``)."""
        return x

    def refresh(self):
        """
        Recompute derived internals from current parameters/topology.

        Base implementation is a no-op and can be overridden by subclasses.
        """
        return

    # Structural Model API

    @property
    def m(self):
        """Total input dimension: the sum of all input-port dimensions."""
        return sum(port.dim for port in self.inputs.values())

    @property
    def p(self):
        """Primary output dimension: the dimension of the ``"y"`` port, or 0."""
        return self.outputs["y"].dim if "y" in self.outputs else 0

    def add_input_port(
        self,
        id,
        *,
        dim=None,
        nominal_value=None,
        labels=None,
        units=None,
        lower_bound=None,
        upper_bound=None,
    ):
        """
        Add a new input port to the system.

        Parameters
        ----------
        id : str
            The programmatic identifier for the port.
        dim : int, optional
            Dimension of the input port. If omitted, inferred from supplied
            metadata or defaults to 1.
        nominal_value : np.ndarray, optional
            The nominal value of the signal, used as the constant fallback
            when the port is unconnected.
        """
        self.inputs[id] = InputPort(
            id,
            dim=dim,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

    def add_output_port(
        self,
        id,
        *,
        dim=None,
        function=None,
        dependencies=(),
        nominal_value=None,
        labels=None,
        units=None,
        lower_bound=None,
        upper_bound=None,
    ):
        """
        Add a new output port to the system.

        Parameters
        ----------
        id : str
            The programmatic identifier for the port.
        dim : int, optional
            Dimension of the output port. If omitted, inferred from supplied
            metadata or defaults to 1.
        function : callable, optional
            The function ``(x, u, t, params) -> value`` computing the port's
            signal.
        dependencies : sequence of str, or "all", optional
            The input-port ids this output directly feeds through from
            (default: no direct feedthrough).
        """
        self._validate_output_dependencies(id, dependencies)
        self.outputs[id] = OutputPort(
            id,
            dim=dim,
            function=function,
            dependencies=dependencies,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

    def get_u_from_input_ports(self):
        """
        Concatenated default values of all input ports.

        This is the disconnected / nominal fallback used for standalone
        systems and for the evaluator IVP tier.
        """
        u = np.zeros(self.m)
        i = 0
        for port in self.inputs.values():
            # Unconnected input ports contribute their constant default value.
            u[i : i + port.dim] = port.get_default_value()
            i += port.dim
        return u

    def get_port_values_from_u(self, u, *port_ids):
        """
        Extract named port signals from the concatenated input vector ``u``.

        * No port ids: return a dict mapping every input-port id to its value.
        * One port id: return that port's value.
        * Several port ids: return a tuple in the requested order.
        """
        input_signals = {}
        i = 0
        for port_id, port in self.inputs.items():
            input_signals[port_id] = u[i : i + port.dim]
            i += port.dim
        if not port_ids:
            return input_signals
        if len(port_ids) == 1:
            return input_signals[port_ids[0]]
        return tuple(input_signals[port_id] for port_id in port_ids)

    def get_input_port_slice(self, port_id):
        """Return the slice of one input port inside the flat input vector ``u``."""
        i = 0
        for current_port_id, port in self.inputs.items():
            port_slice = slice(i, i + port.dim)
            if current_port_id == port_id:
                return port_slice
            i += port.dim

        raise KeyError(f"Unknown input port '{port_id}'")

    def get_all_input_labels_and_units(self):
        """Flattened component labels and units across all input ports."""
        input_labels = []
        input_units = []

        for port in self.inputs.values():
            input_labels.extend(port.labels)
            input_units.extend(port.units)

        return input_labels, input_units

    def _validate_output_dependencies(self, output_id, dependencies):
        if dependencies == "all":
            return
        if dependencies in ("", None):
            raise ValueError(
                f"dependencies for output port '{output_id}' must be (), "
                "a sequence of input-port ids, or 'all'"
            )
        if isinstance(dependencies, str):
            raise ValueError(
                f"dependencies for output port '{output_id}' must be a sequence "
                "of input-port ids or 'all', not a bare string"
            )
        unknown = [
            port_id for port_id in tuple(dependencies) if port_id not in self.inputs
        ]
        if unknown:
            names = ", ".join(repr(port_id) for port_id in unknown)
            raise ValueError(
                f"Unknown input dependencies for output port '{output_id}': {names}"
            )

    # Visualization / Kinematic Contract (frame-keyed)
    #
    # The three drawable hooks return **frame-keyed dicts**: ``tf`` gives world
    # poses per named frame (forward kinematics), ``get_kinematic_geometry``
    # gives the static skin (cached once), and ``get_dynamic_geometry`` gives
    # per-frame geometry rebuilt each frame (force/torque arrows). The base
    # ``get_kinematic_geometry`` delegates to the opt-in ``skin`` attribute, so a
    # plant gets a look by setting ``skin`` without overriding the method.

    def get_kinematic_geometry(self):
        """Static skin as ``dict[str, list[primitive]]`` (delegates to ``skin``)."""
        return {} if self.skin is None else self.skin(self)

    def tf(self, x, u, t=0, params=None):
        """World transforms as ``dict[str, 4x4]`` (the forward-kinematics hook).

        Return only named articulation frames (``body``, ``link0``, …). Do not
        emit ``"world": I`` — the animator injects an implicit identity root
        for geometry keyed to ``"world"``.
        """
        return {}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """Per-frame geometry as ``dict[str, list[primitive]]`` (rebuilt each frame)."""
        return {}

    # Composition Operators

    def __add__(self, other: object) -> "DiagramSystem":
        """Return a diagram containing ``self`` and ``other`` as subsystems."""
        from minilink.core.composition import add_systems

        return add_systems(self, other)

    def __rshift__(self, other: object) -> "DiagramSystem":
        """Return a diagram with ``self`` connected in series to ``other``."""
        from minilink.core.composition import series

        return series(self, other)

    def __matmul__(self, other: object) -> "DiagramSystem":
        """Return a closed-loop diagram ``self @ other``."""
        from minilink.core.composition import closed_loop

        return closed_loop(self, other)


# Specialized System Types


class StaticSystem(System):
    """
    A block with no internal states (``n = 0``): a pure input-output map

        y = h(u, t; p)

    Static blocks default to no kinematic primitives (empty ``skin``) so diagram
    animation shows only dynamic plants unless a subclass opts in.
    """

    def __init__(self):
        """Initialize with no states and no ports; add ports explicitly."""
        System.__init__(self, 0)
        self.name = "StaticSystem"


class DynamicSystem(System):
    """
    A system with continuous states (``n > 0``)

        dx = f(x, u, t; p)
        y  = h(x, u, t; p)

    The constructor can create the standard ports in one line: an input ``u``,
    a primary output ``y`` wired to :meth:`~System.h`, and optionally an
    auxiliary state output ``x``.
    """

    def __init__(
        self,
        n,
        *,
        input_dim=None,
        output_dim=None,
        expose_state=False,
        y_dependencies=(),
    ):
        """
        Initialize a DynamicSystem.

        Parameters
        ----------
        n : int
            Number of continuous states.
        input_dim : int, optional
            If provided, create a standard input port named ``u``.
        output_dim : int, optional
            If provided, create a standard primary output port named ``y``.
        expose_state : bool, optional
            If True, create an auxiliary state output port named ``x``.
        y_dependencies : tuple or "all", optional
            Direct-feedthrough dependencies for the standard ``y`` output.
        """
        System.__init__(self, n)

        self.name = "DynamicSystem"

        if input_dim is not None:
            self.add_input_port("u", dim=input_dim)
        if output_dim is not None:
            self.add_output_port(
                "y", dim=output_dim, function=self.h, dependencies=y_dependencies
            )
        if expose_state:
            self.add_output_port("x", dim=self.n, function=self.compute_state)


if __name__ == "__main__":
    # Hello world: a double integrator dx = [x[1], u[0]]

    class DoubleIntegrator(DynamicSystem):
        def __init__(self):
            super().__init__(n=2, input_dim=1, output_dim=2)

        def f(self, x, u, t=0, params=None):
            return np.array([x[1], u[0]])

    sys = DoubleIntegrator()
    print(sys.f(x=np.array([0.0, 1.0]), u=np.array([2.0])))
