"""
System, signals, and ports

This module defines the foundational modeling classes for minilink.
It includes base signal representations (`VectorSignal`, `InputPort`,
`OutputPort`) and the base `System` class from which all static and
dynamic systems inherit.
"""

import numpy as np


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

    def __init__(self, dim=1, id="x", nominal_value=None):

        self.id = id
        self.dim = int(dim)
        if self.dim < 0:
            raise ValueError("dim must be nonnegative")
        self.labels = [f"{id}[{i}]" for i in range(self.dim)]
        self.units = [""] * self.dim
        self.upper_bound = np.inf * np.ones(self.dim)
        self.lower_bound = -np.inf * np.ones(self.dim)
        self.set_nominal_value(nominal_value)

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
                    f"nominal_value must have shape ({self.dim},), "
                    f"got {value.shape}"
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
        - `list`/`tuple`: specific input IDs (e.g., `["ref", "y"]`).
        - `empty list/tuple`: depends only on the state `x` or internal time, no direct feedthrough from inputs.
    """

    def __init__(self, dim=1, id="y", function=None, dependencies="all"):

        VectorSignal.__init__(self, dim=dim, id=id)

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
      geometry for rendering and animation. This API is still MVP / TRL 1.
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

    def __init__(self, n=0, m=0, p=1):
        """
        Initialize the system with dimensions.

        Parameters
        ----------
        n : int, optional
            Number of continuous states (default is 0).
        m : int, optional
            Number of input dimensions (default is 0).
        p : int, optional
            Number of output dimensions (default is 1).
        """
        # Structural model description
        self.n = int(n)
        self.m = int(m)
        self.p = int(p)
        if self.n < 0 or self.m < 0 or self.p < 0:
            raise ValueError("System dimensions n, m, and p must be nonnegative")

        # Human-readable identifier
        self.name = "System"

        # State metadata
        self.state = VectorSignal(n, "x")

        # Model defaults and metadata
        # Default initial condition used by convenience simulation paths.
        self.x0 = np.zeros(self.n)

        # Port structure
        self.inputs = {}
        self.add_input_port(self.m, "u")
        self.outputs = {}
        self.add_output_port(self.p, "y", function=self.h, dependencies="all")

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

    def add_input_port(self, dim, id, nominal_value=None):
        """
        Add a new input port to the system.

        Parameters
        ----------
        dim : int
            The dimension of the input port.
        id : str
            The programmatic identifier for the port.
        nominal_value : np.ndarray, optional
            The nominal value of the signal.
        """
        self.inputs[id] = InputPort(dim, id, nominal_value)
        self.recompute_input_properties()

    def add_output_port(self, dim, id, function=None, dependencies="all"):
        """
        Add a new output port to the system.

        Parameters
        ----------
        dim : int
            The dimension of the output port.
        id : str
            The programmatic identifier for the port.
        function : callable, optional
            The function used to compute the port's output signal.
        dependencies : list of str, tuple, or "all", optional
            The list of input port IDs this output depends on (default is "all").
        """
        self.outputs[id] = OutputPort(dim, id, function, dependencies)

    def recompute_input_properties(self):
        """
        Recalculate the total input dimension `m` based on all input ports.
        """
        self.m = 0
        for key, port in self.inputs.items():
            self.m += port.dim

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

        for key, port in self.inputs.items():
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
        for key, port in self.inputs.items():
            # Unconnected input ports contribute their constant default value.
            u[i : i + port.dim] = port.get_default_value()
            i += port.dim
        return u

    def get_port_values_from_u(self, u):
        """
        Get the values of all input ports from a concatenated array.

        Parameters
        ----------
        u : np.ndarray
            The concatenated values of all input ports.

        Returns
        -------
        dict
            A dictionary mapping port IDs to their respective signal values.
        """
        input_signals = {}
        i = 0
        for port_id, port in self.inputs.items():
            input_signals[port_id] = u[i : i + port.dim]
            i += port.dim
        return input_signals

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

    def u2input_signal(self, u, port_id):
        """
        Extract the signal value for a specific input port from the concatenated input array.

        Parameters
        ----------
        u : np.ndarray
            The concatenated values of all input ports.
        port_id : str
            The identifier of the input port whose signal value is to be extracted.

        Returns
        -------
        np.ndarray
            The signal value corresponding to the specified input port.
        """
        input_signals = self.get_port_values_from_u(u)
        return input_signals[port_id]

    # Visualization / Kinematic Contract

    def get_kinematic_geometry(self):
        """
        Return static graphical primitives for this system.

        This visualization contract is intentionally still provisional.
        By default, the base :class:`System` generates one point per state and
        one point per input.
        """
        from minilink.graphical.primitives import Point

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
        from minilink.graphical.primitives import translation_matrix

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

    # User Shortcut / Facade API

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
        show=True,
        plot="xu",
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
        plot : {'x', 'u', 'xu'}, optional
            Passed to :func:`minilink.graphical.plotting.plot_trajectory` when
            ``show`` is True: states only, inputs only, or both (default).
        compile_backend : str
            Passed to :class:`~minilink.simulation.Simulator` (default ``\"numpy\"``).
            Use ``compile_backend=\"auto\"`` (see :data:`~minilink.simulation.COMPILE_BACKEND_AUTO`)
            to try JAX then fall back to NumPy.

        Returns
        -------
        Trajectory
            The simulated trajectory, also stored in :attr:`traj`.
        """
        from minilink.graphical.plotting import plot_trajectory
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
            plot_trajectory(self, traj, plot=plot)

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
        show=True,
        plot="xu",
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
        plot : {'x', 'u', 'xu'}, optional
            Passed to :func:`minilink.graphical.plotting.plot_trajectory` when
            ``show`` is True.
        compile_backend : str
            Same as :meth:`compute_trajectory`.

        Returns
        -------
        Trajectory
            Simulated state-input trajectory.
        """
        from minilink.graphical.plotting import plot_trajectory
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

        n_pts = sim.n_pts
        times = sim.times

        def _sample_callable(fn, expected_dim):
            samples = np.zeros((expected_dim, n_pts), dtype=float)
            for i, ti in enumerate(times):
                value = np.asarray(fn(float(ti)), dtype=float)
                if expected_dim == 1 and value.ndim == 0:
                    samples[0, i] = float(value)
                else:
                    value = np.asarray(value, dtype=float).reshape(expected_dim)
                    samples[:, i] = value
            return samples

        def _coerce_series(data, expected_dim, label):
            if callable(data):
                return _sample_callable(data, expected_dim)

            arr = np.asarray(data, dtype=float)

            if arr.ndim == 0:
                if expected_dim != 1:
                    raise ValueError(
                        f"{label} must have shape ({expected_dim}, {n_pts})"
                    )
                return np.full((1, n_pts), float(arr), dtype=float)

            if arr.ndim == 1:
                if expected_dim == 1 and arr.shape[0] == n_pts:
                    return arr.reshape(1, n_pts)
                if arr.shape[0] == expected_dim:
                    return np.repeat(arr.reshape(expected_dim, 1), n_pts, axis=1)
                raise ValueError(f"{label} must have shape ({expected_dim}, {n_pts})")

            if arr.ndim == 2 and arr.shape == (expected_dim, n_pts):
                return arr

            raise ValueError(f"{label} must have shape ({expected_dim}, {n_pts})")

        if input_port_id is None:
            u_traj = _coerce_series(u, self.m, "u")
        else:
            port = self.inputs[input_port_id]
            port_slice = self.get_input_port_slice(input_port_id)
            u_traj = np.repeat(
                self.get_u_from_input_ports().reshape(self.m, 1),
                n_pts,
                axis=1,
            )
            u_traj[port_slice, :] = _coerce_series(
                u,
                port.dim,
                f"u for input port '{input_port_id}'",
            )

        traj = sim.solve_forced(u_traj)
        if show:
            plot_trajectory(self, traj, plot=plot)
        self.traj = traj

        return traj

    def plot_trajectory(self, traj=None, *, plot="xu"):
        """
        Convenience shortcut to plot the trajectory.

        If the trajectory is not computed yet, it is computed using :meth:`compute_trajectory`.
        If the trajectory is already computed, it is used directly.
        If the trajectory is provided, it is used directly.

        Parameters
        ----------
        plot : {'x', 'u', 'xu'}, optional
            States only, inputs only, or both (default); see
            :func:`minilink.graphical.plotting.plot_trajectory`.

        Returns
        -------
        Trajectory or tuple
            :class:`~minilink.core.trajectory.Trajectory` if a simulation is run; otherwise
            the ``(fig, ax)`` pair from :func:`minilink.graphical.plotting.plot_trajectory`.
        """
        from minilink.graphical.plotting import plot_trajectory

        if traj is not None:
            return plot_trajectory(self, traj, plot=plot)

        if self.traj is not None:
            return plot_trajectory(self, self.traj, plot=plot)

        return self.compute_trajectory(plot=plot)

    def get_block_html(self, label="sys1"):
        """
        Convenience shortcut returning an HTML block representation.
        """
        from minilink.graphical.graphe import get_system_block_html

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

    def get_graphe(self):
        """
        Convenience shortcut returning the Graphviz graph representation.
        """
        from minilink.graphical.graphe import get_system_graphe

        return get_system_graphe(self)

    def _repr_svg_(self):
        """
        Convenience notebook representation for the system graph.
        """
        g = self.get_graphe()
        if g is None:
            return None
        return g._repr_image_svg_xml()

    def plot_graphe(self, filename=None, show_inline=None, show_pdf=None):
        """
        Convenience shortcut to render the Graphviz system graph.

        ``show_inline`` and ``show_pdf`` default to ``None`` and auto-resolve
        via :func:`minilink.graphical.environment.is_inline_capable`:
        Jupyter / Colab get inline SVG only (no viewer pop-up, no disk write),
        while bare scripts and IPython REPLs get the legacy render-to-temp-file
        + open-in-OS-PDF-viewer behavior. Pass explicit booleans to override;
        pass ``filename`` to force a specific on-disk output.
        """
        from minilink.graphical.graphe import plot_graphviz

        g = self.get_graphe()
        plot_graphviz(g, show_inline=show_inline, show_pdf=show_pdf, filename=filename)

    def render(self, x, u, t, is_3d=False, renderer="matplotlib"):
        """
        Convenience shortcut rendering a single frame of the system.
        """
        from minilink.graphical.animation import Animator

        animator = Animator(self)
        animator.show(x, u, t, is_3d=is_3d, renderer=renderer)

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
        :func:`minilink.graphical.environment.prefers_inline_animation`:
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
        from minilink.graphical.environment import prefers_inline_animation

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

    def __init__(self, m, p):
        """
        Initialize a StaticSystem.

        Parameters
        ----------
        m : int
            Number of input dimensions.
        p : int
            Number of output dimensions.
        """
        System.__init__(self, 0, m, p)
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

    def __init__(self, n, m, p):
        """
        Initialize a DynamicSystem.

        Parameters
        ----------
        n : int
            Number of continuous states.
        m : int
            Number of input dimensions.
        p : int
            Number of output dimensions.
        """
        System.__init__(self, n, m, p)

        self.name = "DynamicSystem"

        # By default, a dynamic system's output 'y' only depends on its state 'x', not 'u'
        self.outputs["y"].dependencies = ()

        # Add a default output port 'x' that outputs the state vector directly, useful for connecting to other blocks in the diagram
        self.add_output_port(self.n, "x", function=self.compute_state)


if __name__ == "__main__":
    x = VectorSignal(2, "x")

    sys = DynamicSystem(2, 1, 1)

    func = StaticSystem(2, 2)
