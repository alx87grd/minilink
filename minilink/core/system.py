"""
System, signals, and ports

This module defines the foundational modeling classes for minilink.
It includes base signal representations (`VectorSignal`, `InputPort`,
`OutputPort`) and the base `System` class from which all static and
dynamic systems inherit.
"""

import numpy as np


def _as_optional_vector(value, *, dim, field_name, signal_id):
    if value is None:
        return None
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (dim,):
        raise ValueError(
            f"{field_name} for signal '{signal_id}' must have length {dim}, "
            f"got {array.shape[0]}"
        )
    return array.copy()


def _as_optional_string_list(value, *, dim, field_name, signal_id):
    if value is None:
        return None
    if isinstance(value, str):
        items = [value]
    else:
        items = list(value)
    if len(items) != dim:
        raise ValueError(
            f"{field_name} for signal '{signal_id}' must have length {dim}, "
            f"got {len(items)}"
        )
    return [str(item) for item in items]


def _length_or_none(value, *, field_name, signal_id):
    if value is None:
        return None
    if isinstance(value, str):
        return 1
    if np.isscalar(value):
        return 1
    try:
        return int(np.asarray(value).reshape(-1).shape[0])
    except Exception as exc:
        raise ValueError(
            f"Could not infer dimension from {field_name} for signal '{signal_id}'"
        ) from exc


def _infer_signal_dim(
    signal_id,
    *,
    dim=None,
    nominal_value=None,
    labels=None,
    units=None,
    lower_bound=None,
    upper_bound=None,
):
    if dim is not None:
        dim = int(dim)
        if dim < 0:
            raise ValueError(f"dim for signal '{signal_id}' must be nonnegative")
        return dim

    candidates = [
        ("nominal_value", nominal_value),
        ("labels", labels),
        ("units", units),
        ("lower_bound", lower_bound),
        ("upper_bound", upper_bound),
    ]
    inferred = []
    for field_name, value in candidates:
        length = _length_or_none(value, field_name=field_name, signal_id=signal_id)
        if length is not None:
            inferred.append((field_name, length))

    if not inferred:
        return 1

    first_name, first_dim = inferred[0]
    for field_name, length in inferred[1:]:
        if length != first_dim:
            raise ValueError(
                f"Conflicting inferred dimensions for signal '{signal_id}': "
                f"{first_name} has length {first_dim}, {field_name} has length {length}"
            )
    return first_dim


class VectorSignal:
    """
    A class representing a generic vector signal.

    Attributes
    ----------
    dim : int
        The dimension of the vector signal.
    id : str
        The programmatic string identifier of the signal (e.g., 'x', 'u').
    labels : list of str
        The display string label for each component of the signal.
    units : list of str
        The physical units for each component of the signal.
    upper_bound : np.ndarray
        The upper numerical bound of each component.
    lower_bound : np.ndarray
        The lower numerical bound of each component.
    nominal_value : np.ndarray
        The default or operating point value of the signal.
    """

    def __init__(
        self,
        id="x",
        *,
        dim=None,
        nominal_value=None,
        labels=None,
        units=None,
        lower_bound=None,
        upper_bound=None,
    ):

        self.id = id
        self.dim = _infer_signal_dim(
            id,
            dim=dim,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )
        self.labels = [f"{id}[{i}]" for i in range(self.dim)]
        self.units = [""] * self.dim
        self.upper_bound = np.inf * np.ones(self.dim)
        self.lower_bound = -np.inf * np.ones(self.dim)
        self.set_nominal_value(nominal_value)
        if labels is not None:
            self.labels = _as_optional_string_list(
                labels, dim=self.dim, field_name="labels", signal_id=id
            )
        if units is not None:
            self.units = _as_optional_string_list(
                units, dim=self.dim, field_name="units", signal_id=id
            )
        lower = _as_optional_vector(
            lower_bound, dim=self.dim, field_name="lower_bound", signal_id=id
        )
        if lower is not None:
            self.lower_bound = lower
        upper = _as_optional_vector(
            upper_bound, dim=self.dim, field_name="upper_bound", signal_id=id
        )
        if upper is not None:
            self.upper_bound = upper

    def set_nominal_value(self, nominal_value=None):
        """
        Set the nominal (default) value of the vector signal.

        Parameters
        ----------
        nominal_value : list or np.ndarray, optional
            The nominal value to set. Must match the dimension `dim`.
            If None, defaults to an array of zeros.
        """
        if nominal_value is not None:
            value = np.asarray(nominal_value, dtype=float).reshape(-1)
            if value.shape != (self.dim,):
                raise ValueError(
                    f"nominal_value must have shape ({self.dim},), got {value.shape}"
                )
            self.nominal_value = value.copy()
        else:
            self.nominal_value = np.zeros(self.dim)

    def __repr__(self):
        """
        Return a string representation of the VectorSignal.

        Returns
        -------
        str
            A string describing the dimension and nominal value.
        """
        return f"VectorSignal: dim={self.dim}, nominal={self.nominal_value}"


class InputPort(VectorSignal):
    """
    A vector signal with a constant default value used when unconnected.
    """

    def get_default_value(self) -> np.ndarray:
        """
        Return the default value of the input port.

        Returns
        -------
        np.ndarray
            The nominal value of the input port.
        """
        return self.nominal_value


class OutputPort(VectorSignal):
    """
    An output signal port containing a callback function to compute its value dynamically.

    Attributes
    ----------
    compute : callable
        The function used to compute the signal value based on the inputs, state, time, and parameters.
        Signature: ``compute(x, u, t, params=None) -> np.ndarray``.
        Purity is not enforced by the library (same contract as :meth:`System.f`).
    dependencies : list of str, tuple, or "all"
        The list of `InputPort` IDs that this output directly depends on for immediate feedthrough.
        - `"all"`: depends on all inputs of the system (safest, but can create false algebraic loops in MIMO systems).
        - `list`/`tuple`: specific input IDs (e.g., `["r", "y"]`).
        - `empty list/tuple`: depends only on the state `x` or internal time, no direct feedthrough from inputs.
    """

    def __init__(
        self,
        id="y",
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

        VectorSignal.__init__(
            self,
            id=id,
            dim=dim,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

        if function is not None:
            self.compute = function
        else:
            self.compute = self.default_function

        self.dependencies = dependencies

    def default_function(self, x=None, u=None, t=0, params=None) -> np.ndarray:
        """
        Default compute function returning the nominal value.

        Parameters
        ----------
        x : np.ndarray, optional
            The state vector of the system.
        u : np.ndarray, optional
            The input vector of the system.
        t : float, optional
            The current time.
        params : dict, optional
            A dictionary of system parameters.

        Returns
        -------
        np.ndarray
            The nominal value of the output port.
        """
        return self.nominal_value


class System:
    """
    Base class describing a dynamical input-output system.

    A :class:`System` serves several roles at once:

    - **Core dynamical contract**: it defines a functional modeling interface
      through :meth:`f` and :meth:`h`, intended to describe the system as a
      function of ``(x, u, t, params)``.
    - **Structural model description**: it stores dimensions, ports, state
      metadata, labels, units, bounds, and nominal values.
    - **Model defaults and metadata**: it carries default parameters
      (:attr:`params`), default initial condition (:attr:`x0`), and solver
      hints (:attr:`solver_info`).
    - **Optional visualization contract**: it may describe forward-kinematic
      geometry for rendering and animation (still MVP / TRL 1). Default
      ``camera_*`` fields configure :meth:`get_camera_transform`; see that method.
    - **User shortcut façade**: it exposes convenience methods such as
      :meth:`compile`, :meth:`compute_trajectory`, :meth:`render`,
      :meth:`animate`, and :meth:`game`.

    Notes on dynamics and purity
    ----------------------------
    The intended dynamical contract is **functional/stateless in intent**:
    overridden :meth:`f` and :meth:`h` should behave as functions of
    ``(x, u, t, params)`` only. Python cannot enforce purity, so users should
    avoid relying on hidden mutable instance state inside dynamics or output
    functions. The object itself still stores model defaults, metadata, and
    user convenience state.
    """

    def __init__(self, n=0):
        """
        Initialize the system with dimensions.

        Parameters
        ----------
        n : int, optional
            Number of continuous states (default is 0).
        Systems start with no input or output ports. Add ports explicitly with
        :meth:`add_input_port` and :meth:`add_output_port`.
        """
        # Structural model description
        self.n = int(n)
        if self.n < 0:
            raise ValueError("System dimension n must be nonnegative")
        self.m = 0
        self.p = 0

        # Human-readable identifier
        self.name = "System"

        # State metadata
        self.state = VectorSignal("x", dim=n)

        # Model defaults and metadata
        # Default initial condition used by convenience simulation paths.
        self.x0 = np.zeros(self.n)

        # Port structure
        self.inputs = {}
        self.outputs = {}

        # Default model parameter set. Explicit ``params=...`` arguments
        # passed to ``f`` / ``h`` override this default.
        self.params = {}

        # Solver hints used by high-level simulation shortcuts.
        self.solver_info = {
            "continuous_time_equation": True,
            "smallest_time_constant": 0.001,
            "discontinuous_behavior": False,  # Will use a fixed time step
            "discrete_time_period": None,
            "require_building": False,  # If True, the system needs to be built before being simulated
        }

        # Runtime convenience cache for the last trajectory produced by
        # ``compute_trajectory``.
        self.traj = None

        # Standard camera for :meth:`get_camera_transform`: ``camera_*`` fields below.
        self.camera_target = np.zeros(3, dtype=float)
        self.camera_plot_axes = (0, 1)
        self.camera_scale = 10.0

    # Core Dynamical Contract

    def refresh(self):
        """
        Recompute derived internals from current parameters/topology.

        Base implementation is a no-op and can be overridden by subclasses.
        """
        return

    def f(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Compute the state derivative `dx/dt`.

        Subclasses should treat this as a function of ``(x, u, t, params)`` only; the
        the library does not verify purity (see :class:`System` class docstring).

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The state derivative vector.
        """
        dx = np.zeros(self.n)
        return dx

    def h(self, x, u, t=0, params=None) -> np.ndarray:
        """
        Compute the output `y`.

        Same purity / side-effect contract as :meth:`f` (convention only; not enforced).

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The output vector.
        """
        y = np.zeros(self.p)
        return y

    def compute_state(self, x, u, t=0, params=None):
        """
        Convenience output helper returning the state vector directly.

        Parameters
        ----------
        x : np.ndarray
            The current state vector.
        u : np.ndarray
            The current input vector.
        t : float, optional
            The current time.
        params : dict, optional
            The system parameters.

        Returns
        -------
        np.ndarray
            The state vector.
        """
        return x

    # Structural Model API

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
            The nominal value of the signal.
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
        self.recompute_input_properties()

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
            The function used to compute the port's output signal.
        dependencies : list of str, tuple, or "all", optional
            The list of input port IDs this output depends on (default is empty).
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
        self.recompute_output_properties()

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

    def recompute_input_properties(self):
        """
        Recalculate the total input dimension `m` based on all input ports.
        """
        self.m = 0
        for _, port in self.inputs.items():
            self.m += port.dim

    def recompute_output_properties(self):
        """
        Recalculate the primary output dimension `p` from the ``y`` output port.
        """
        self.p = self.outputs["y"].dim if "y" in self.outputs else 0

    def get_all_input_labels_and_units(self):
        """
        Retrieve a flattened list of all input labels and units across all ports.

        Returns
        -------
        input_labels : list of str
            Combined list of signal labels.
        input_units : list of str
            Combined list of signal units.
        """
        input_labels = []
        input_units = []

        for _, port in self.inputs.items():
            for i in range(port.dim):
                input_labels.append(port.labels[i])
                input_units.append(port.units[i])

        return input_labels, input_units

    def get_u_from_input_ports(self) -> np.ndarray:
        """
        Get the default value of all input ports.

        This is the disconnected / nominal fallback used for standalone systems
        and for the evaluator IVP tier.

        Returns
        -------
        np.ndarray
            The concatenated default values of all input ports.
        """
        u = np.zeros(self.m)
        i = 0
        for _, port in self.inputs.items():
            # Unconnected input ports contribute their constant default value.
            u[i : i + port.dim] = port.get_default_value()
            i += port.dim
        return u

    def get_port_values_from_u(self, u, *port_ids):
        """
        Get the values of all input ports from a concatenated array.

        Parameters
        ----------
        u : np.ndarray
            The concatenated values of all input ports.

        * If no port ids are passed, return a dictionary mapping every input
          port id to its signal value.
        * If one port id is passed, return that port's signal value.
        * If multiple port ids are passed, return a tuple in the requested order.
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
        """
        Return the slice of one input port inside the flat input vector ``u``.

        Parameters
        ----------
        port_id : str
            Programmatic input-port identifier.

        Returns
        -------
        slice
            Slice selecting that port inside the concatenated input vector.
        """
        i = 0
        for current_port_id, port in self.inputs.items():
            port_slice = slice(i, i + port.dim)
            if current_port_id == port_id:
                return port_slice
            i += port.dim

        raise KeyError(f"Unknown input port '{port_id}'")

    # Visualization / Kinematic Contract

    def get_kinematic_geometry(self):
        """
        Return static graphical primitives for this system.

        This visualization contract is intentionally still provisional.
        By default, the base :class:`System` generates one point per state and
        one point per input.
        """
        from minilink.graphical.animation.primitives import Point

        primitives = []
        for i in range(self.n):
            primitives.append(Point(color="blue", marker="o"))
        for i in range(self.m):
            primitives.append(Point(color="red", marker="x"))
        return primitives

    def get_kinematic_transforms(self, x, u, t):
        """
        Return transforms corresponding 1-to-1 with the static geometry.

        This visualization contract is intentionally still provisional.
        By default, states and inputs are mapped to simple translations.
        """
        from minilink.graphical.animation.primitives import translation_matrix

        transforms = []

        for i in range(self.n):
            transforms.append(translation_matrix(dx=x[i], dy=float(i)))
        for i in range(self.m):
            transforms.append(translation_matrix(dx=u[i], dy=float(-i - 1)))

        return transforms

    def get_dynamic_geometry(self, x, u, t):
        """
        Return frame-specific temporary graphical primitives.

        This visualization contract is intentionally still provisional.
        """
        return []

    def get_camera_transform(self, x, u, t):
        """
        Return the standard 4x4 camera transform for this system.

        The matrix follows :func:`minilink.graphical.animation.primitives.camera_matrix`:
        ``T[:3, 3]`` is the look-at target in world, the columns of ``T[:3, :3]``
        are the world directions of camera-X (plot horizontal), camera-Y
        (plot vertical), and camera-Z (view-out), and ``T[3, 3]`` is the view
        scale (orthographic half-extent / perspective camera distance).

        The default matches ``camera_matrix()`` via ``camera_target``,
        ``camera_plot_axes``, and ``camera_scale`` on ``self``. Edit those
        attributes for a fixed view, or override this method for a time-varying
        camera.

        TODO: User Architectural Review (visualization contract is TRL 1).
        """
        from minilink.graphical.animation.primitives import camera_matrix

        return camera_matrix(
            target=self.camera_target,
            plot_axes=self.camera_plot_axes,
            scale=self.camera_scale,
        )

    # User Shortcut / Facade API

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

    def compile(self, backend="numpy", verbose=False):
        """
        Convenience shortcut to compile the system into a backend evaluator.

        This delegates to :func:`minilink.compile.compile`.
        """
        from minilink.compile.compiler import compile as compile_system

        return compile_system(self, backend=backend, verbose=verbose)

    def compute_trajectory(
        self,
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
        verbose=True,
    ):
        """
        Convenience shortcut to simulate the system and return a trajectory.

        This method is a façade over :class:`minilink.simulation.Simulator`.
        It uses model defaults such as :attr:`x0` and stores the resulting
        trajectory in :attr:`traj` for later convenience.

        Parameters
        ----------
        compile_backend : str
            Passed to :class:`~minilink.simulation.Simulator` (default ``\"numpy\"``).
            Use ``compile_backend=\"auto\"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.

        Returns
        -------
        Trajectory
            The simulated trajectory, also stored in :attr:`traj`.
        """
        from minilink.simulation.simulator import Simulator

        sim = Simulator(
            self,
            x0=x0,
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
            verbose=verbose,
        )
        traj = sim.solve()

        if show:
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def compute_forced(
        self,
        u,
        input_port_id=None,
        t0=0,
        tf=10,
        n_steps=None,
        dt=None,
        solver=None,
        show=False,
        x0=None,
        compile_backend="numpy",
        verbose=True,
    ):
        """
        Convenience shortcut to simulate the system under a prescribed input.

        This method is a façade over :class:`minilink.simulation.Simulator`
        and :meth:`minilink.simulation.Simulator.solve_forced`.

        Parameters
        ----------
        u : np.ndarray or callable
            Forced input description.
            - If ``input_port_id is None``: either a full input trajectory with
              shape ``(m, n_pts)`` or a callable ``u(t)`` returning the full
              input vector.
            - If ``input_port_id`` is provided: either a trajectory for that
              port only with shape ``(port_dim, n_pts)`` or a callable
              returning that port signal. Other ports stay at their default
              values.
        input_port_id : str, optional
            Named input port to force while keeping the others at default
            values.

        Returns
        -------
        Trajectory
            Simulated state-input trajectory.
        """
        from minilink.simulation.simulator import Simulator

        sim = Simulator(
            self,
            x0=x0,
            t0=t0,
            tf=tf,
            n_steps=n_steps,
            dt=dt,
            solver=solver,
            compile_backend=compile_backend,
            verbose=verbose,
        )

        traj = sim.solve_forced(u, input_port_id=input_port_id)

        if show:
            from minilink.graphical.signals import plot_time_signals

            plot_time_signals(self, traj)

        self.traj = traj

        return traj

    def plot_data(
        self,
        traj=None,
        *,
        signals=("x", "u"),
        x_label,
        y_labels=None,
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot signal components against another signal.

        Unlike :meth:`plot_trajectory`, which plots signals against time, this
        plots the components named in ``y_labels`` against the single component
        named ``x_label`` (for example a vehicle's Y position against its X
        position).

        .. note::

            TODO: User Architectural Review. Experimental facade; the API is not
            frozen. Open questions: matplotlib-only (no plotly parity yet),
            label-based selection vs the name-based selection used by
            :meth:`plot_trajectory`, and overlap with :meth:`plot_phase_plane`.

        If the trajectory is not computed yet, it is computed using
        :meth:`compute_trajectory`.

        Parameters
        ----------
        x_label : str
            Label of the signal component to use as the x-axis.
        y_labels : tuple of str, optional
            Labels of the components to plot. Defaults to every gathered
            component other than ``x_label``.
        signals : tuple of str, optional
            Signal names to gather; see
            :func:`minilink.graphical.signals.plot_data_signals`.

        Returns
        -------
        PlotResult
            The plot result from
            :func:`minilink.graphical.signals.plot_data_signals`.
        """
        from minilink.graphical.signals import plot_data_signals

        if traj is None:
            traj = (
                self.traj
                if self.traj is not None
                else self.compute_trajectory(show=False)
            )

        return plot_data_signals(
            self,
            traj,
            signals=signals,
            x_label=x_label,
            y_labels=y_labels,
            backend=backend,
            show=show,
        )

    def plot_trajectory(
        self,
        traj=None,
        *,
        signals=("x", "u"),
        backend="matplotlib",
        show=True,
    ):
        """
        Convenience shortcut to plot sampled time signals.

        If the trajectory is not computed yet, it is computed using :meth:`compute_trajectory`.
        If the trajectory is already computed, it is used directly.
        If the trajectory is provided, it is used directly.

        Parameters
        ----------
        signals : tuple of str, optional
            Signal names to plot; see
            :func:`minilink.graphical.signals.plot_time_signals`.

        Returns
        -------
        PlotResult
            The plot result from
            :func:`minilink.graphical.signals.plot_time_signals`.
        """
        from minilink.graphical.signals import plot_time_signals

        if traj is not None:
            return plot_time_signals(
                self, traj, signals=signals, backend=backend, show=show
            )

        if self.traj is not None:
            return plot_time_signals(
                self, self.traj, signals=signals, backend=backend, show=show
            )

        traj = self.compute_trajectory(show=False)
        return plot_time_signals(
            self,
            traj,
            signals=signals,
            backend=backend,
            show=show,
        )

    def plot_phase_plane(
        self,
        traj=None,
        *,
        x_axis=0,
        y_axis=None,
        backend="matplotlib",
        show=True,
        **kwargs,
    ):
        """
        Convenience shortcut to plot a phase-plane vector field.

        If ``traj`` is provided, or if :attr:`traj` contains a previous
        simulation result, the sampled state path is overlaid on the vector
        field. Otherwise only the vector field is plotted.
        """
        from minilink.graphical.phase_plane import plot_phase_plane

        if traj is None:
            traj = self.traj
        return plot_phase_plane(
            self,
            traj,
            x_axis=x_axis,
            y_axis=y_axis,
            backend=backend,
            show=show,
            **kwargs,
        )

    def get_block_html(self, label="sys1"):
        """
        Convenience shortcut returning an HTML block representation.
        """
        from minilink.graphical.diagrams import get_system_block_html

        return get_system_block_html(self, label)

    def print_html(self):
        """
        Convenience shortcut to display the HTML block representation.
        """
        try:
            import IPython.display as display

            display.display(display.HTML(self.get_block_html()))
        except ImportError:
            print("IPython is not available")
            return

    def get_diagram(self):
        """
        Convenience shortcut returning a renderable diagram representation.
        """
        from minilink.graphical.diagrams import get_diagram

        return get_diagram(self)

    def _repr_svg_(self):
        """
        Convenience notebook representation for the system diagram.
        """
        g = self.get_diagram()
        if g is None:
            return None
        return g._repr_image_svg_xml()

    def plot_diagram(self, filename=None, show_inline=None, show_pdf=None):
        """
        Convenience shortcut to render the system diagram.

        ``show_inline`` and ``show_pdf`` default to ``None`` and auto-resolve
        via :func:`minilink.graphical.common.environment.is_inline_capable`:
        Jupyter / Colab get inline SVG only (no viewer pop-up, no disk write),
        while bare scripts and IPython REPLs get the legacy render-to-temp-file
        + open-in-OS-PDF-viewer behavior. Pass explicit booleans to override;
        pass ``filename`` to force a specific on-disk output.
        """
        from minilink.graphical.diagrams import plot_diagram

        return plot_diagram(
            self,
            show_inline=show_inline,
            show_pdf=show_pdf,
            filename=filename,
        )

    def render(self, x, u, t, is_3d=False, renderer="matplotlib"):
        """
        Convenience shortcut rendering a single frame of the system.
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        return animator.show(x, u, t, is_3d=is_3d, renderer=renderer)

    def animate(
        self,
        traj=None,
        time_factor_video=1.0,
        is_3d=False,
        html: bool | None = None,
        renderer="matplotlib",
        native: bool = True,
    ):
        """
        Convenience shortcut to animate a trajectory of this system.

        ``html=None`` auto-resolves via
        :func:`minilink.graphical.common.environment.prefers_inline_animation`:
        ``True`` in Colab and in local Jupyter with a non-interactive
        matplotlib backend (``inline`` / ``agg``); ``False`` for bare
        script, IPython REPL, and Jupyter with an interactive backend
        (``qt`` / ``widget`` / ``macosx`` / ``tk`` / ``nbagg``).
        ``native=True`` (default) drives each backend's own animation
        engine (matplotlib ``FuncAnimation`` / meshcat ``Animation``).
        Pass ``native=False`` to fall back to the legacy per-frame
        Python-loop playback (useful for debugging or when the native
        path's limitations matter — e.g. meshcat freezes dynamic-geometry
        primitives such as ``TorqueArrow``; see ``DESIGN.md`` §4.7).
        """
        from minilink.graphical.animation import Animator
        from minilink.graphical.common.environment import prefers_inline_animation

        if traj is None:
            if self.traj is not None:
                traj = self.traj
            else:
                traj = self.compute_trajectory()

        resolved_html = prefers_inline_animation() if html is None else html

        animator = Animator(self)
        show_plot = not resolved_html
        ani_obj = animator.animate_simulation(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=resolved_html,
            show=show_plot,
            renderer=renderer,
            native=native,
        )

        # For html output, return the IPython.display.HTML object and let the
        # notebook auto-display it via the standard last-expression rule.
        # Calling display.display() *and* returning the object renders twice.
        return ani_obj

    def game(
        self,
        *,
        dt=1 / 30.0,
        dynamics_substeps=1,
        renderer="pygame",
        is_3d=False,
        x0=None,
        u0=None,
        t0=0.0,
        max_steps=None,
    ):
        """
        Convenience shortcut for the prototype interactive game loop.

        See ``Animator.game`` and ``ROADMAP.md`` §7: integrator + live I/O are planned as
        pluggable backends (today: pygame keyboard + Euler in the animator loop).
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        return animator.game(
            dt=dt,
            dynamics_substeps=dynamics_substeps,
            renderer=renderer,
            is_3d=is_3d,
            x0=self.x0 if x0 is None else x0,
            u0=np.zeros(self.m) if u0 is None else u0,
            t0=t0,
            max_steps=max_steps,
        )


# Specialized System Types


class StaticSystem(System):
    """
    A block-diagram system with no internal continuous states (`n=0`).
    Outputs depend strictly on inputs and time.

    Static blocks do not inherit the base :meth:`~System.get_kinematic_geometry`
    point cloud (one marker per input); they default to no kinematic primitives so
    diagram animation shows only dynamic plants unless a subclass opts in.
    """

    def __init__(self):
        """
        Initialize a StaticSystem.

        Parameters
        ----------
        Static systems start with no ports. Add input/output ports explicitly.
        """
        System.__init__(self, 0)
        self.name = "StaticSystem"

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, x, u, t):
        return []


class DynamicSystem(System):
    """
    A block-diagram system with continuous states (`n > 0`).
    State derivatives (`dx/dt`) are computed from inputs and current state.
    """

    # dx = f(x, u, t)
    # y  = h(x, u, t)

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
    x = VectorSignal("x", dim=2)

    sys = DynamicSystem(2, input_dim=1, output_dim=1, expose_state=True)

    func = StaticSystem()
