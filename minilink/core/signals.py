"""Signals and ports: the wires of a block diagram."""

import numpy as np

from minilink.core.sets import BoxSet

###############################################################################
# Public API
###############################################################################


class VectorSignal:
    """
    A vector signal: a dimension plus human metadata.

    A signal is a vector with labels, units, bounds and a nominal value; a port
    (:class:`InputPort`, :class:`OutputPort`) is a signal on a system boundary.

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
        # Dimension: the explicit dim, else the length the metadata implies
        self.id = id
        self.dim = infer_signal_dim(
            id,
            dim=dim,
            nominal_value=nominal_value,
            labels=labels,
            units=units,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

        # Default metadata: indexed labels, no units, no bounds
        self.labels = [f"{id}[{i}]" for i in range(self.dim)]
        self.units = [""] * self.dim
        self.upper_bound = np.inf * np.ones(self.dim)
        self.lower_bound = -np.inf * np.ones(self.dim)

        # Nominal value: the given one, else zeros
        self.set_nominal_value(nominal_value)

        # The user's metadata over the defaults, each field checked against dim
        if labels is not None:
            self.labels = as_optional_string_list(
                labels, dim=self.dim, field_name="labels", signal_id=id
            )
        if units is not None:
            self.units = as_optional_string_list(
                units, dim=self.dim, field_name="units", signal_id=id
            )
        lower = as_optional_vector(
            lower_bound, dim=self.dim, field_name="lower_bound", signal_id=id
        )
        if lower is not None:
            self.lower_bound = lower
        upper = as_optional_vector(
            upper_bound, dim=self.dim, field_name="upper_bound", signal_id=id
        )
        if upper is not None:
            self.upper_bound = upper

    @property
    def box(self) -> BoxSet:
        """The bounds as a set, ``lower_bound <= z <= upper_bound``, derived on each read."""
        return BoxSet(self.lower_bound, self.upper_bound)

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
            require_nominal_shape(value, self.dim)
            self.nominal_value = value.copy()
        else:
            self.nominal_value = np.zeros(self.dim)

    def __repr__(self):
        return f"VectorSignal: dim={self.dim}, nominal={self.nominal_value}"


class InputPort(VectorSignal):
    """
    A signal entering a system.

    The nominal value doubles as the constant fallback used when the port is
    left unconnected in a diagram.
    """

    def get_default_value(self):
        """Return the constant fallback value (the nominal value)."""
        return self.nominal_value


class OutputPort(VectorSignal):
    """
    A signal leaving a system, with the function that computes it.

    Attributes
    ----------
    compute : callable
        The function computing the signal value from the system state, inputs,
        time, and parameters. Signature: ``compute(x, u, t, params=None)``.
        Purity is not enforced by the library (same contract as
        :meth:`minilink.core.system.System.f`).
    dependencies : list of str, tuple, or "all"
        The input-port ids this output directly feeds through from — the
        direct-feedthrough structure used for algebraic-loop detection:

        - ``"all"``: depends on all inputs (safest, but can create false
          algebraic loops in MIMO systems);
        - a sequence of input ids (e.g., ``("r", "y")``): only those inputs;
        - ``()``: no direct feedthrough — the output depends only on the
          state ``x`` and time.
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
        # The signal and its metadata
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

        # Compute function: the given one, else the nominal value
        if function is not None:
            self.compute = function
        else:
            self.compute = self.default_function

        # Direct feedthrough: the inputs the output reads at the same instant
        self.dependencies = dependencies

    def default_function(self, x=None, u=None, t=0, params=None):
        """Default compute function returning the nominal value."""
        return self.nominal_value


###############################################################################
# Internal machinery
###############################################################################


def infer_signal_dim(
    signal_id,
    *,
    dim=None,
    nominal_value=None,
    labels=None,
    units=None,
    lower_bound=None,
    upper_bound=None,
):
    """Determine a signal's dimension from whatever the user provided.

    Resolution order:

    1. An explicit ``dim`` wins (validated nonnegative).
    2. Otherwise, every provided metadata field (nominal value, labels, units,
       bounds) implies a length; they must all agree, and the agreed length is
       the dimension. Disagreement raises with both offending fields named.
    3. If nothing was provided, the signal defaults to dimension 1.

    This is what lets users write ``add_input_port("r", nominal_value=[0, 0])``
    without repeating ``dim=2``.
    """
    # An explicit dim wins
    if dim is not None:
        require_signal_dim(dim, signal_id)
        dim = int(dim)
        return dim

    # The length each provided field implies
    candidates = [
        ("nominal_value", nominal_value),
        ("labels", labels),
        ("units", units),
        ("lower_bound", lower_bound),
        ("upper_bound", upper_bound),
    ]
    inferred = []
    for field_name, value in candidates:
        length = length_or_none(value, field_name=field_name, signal_id=signal_id)
        if length is not None:
            inferred.append((field_name, length))

    # Nothing provided: a scalar signal
    if not inferred:
        return 1

    # The implied lengths agree, or the first disagreement names both fields
    first_name, first_dim = inferred[0]
    for field_name, length in inferred[1:]:
        if length != first_dim:
            raise ValueError(
                f"Conflicting inferred dimensions for signal '{signal_id}': "
                f"{first_name} has length {first_dim}, {field_name} has length {length}"
            )

    return first_dim


def require_signal_dim(dim, signal_id):
    """Refuse a dimension that is a bool, not a whole number, or negative."""
    if isinstance(dim, (bool, np.bool_)) or int(dim) != dim:
        raise ValueError(
            f"dim for signal '{signal_id}' must be a whole number, got {dim!r}"
        )
    if dim < 0:
        raise ValueError(f"dim for signal '{signal_id}' must be nonnegative")


def length_or_none(value, *, field_name, signal_id):
    """Return the vector length implied by one metadata value, or ``None``.

    Used by :func:`infer_signal_dim` to ask each metadata field "what
    dimension do you imply?": ``None`` implies nothing, a string or scalar
    implies 1, and any array-like implies its flattened length.
    """
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


def require_nominal_shape(value, dim):
    """Refuse a nominal value that does not flatten to exactly ``dim`` entries."""
    if value.shape != (dim,):
        raise ValueError(f"nominal_value must have shape ({dim},), got {value.shape}")


def as_optional_string_list(value, *, dim, field_name, signal_id):
    """Convert one metadata field (labels or units) to a list of ``dim`` strings.

    ``None`` passes through (meaning "use the default"). A bare string is
    accepted for 1-D signals; a sequence must have exactly ``dim`` items, so a
    disagreement with the declared dimension fails at construction.
    """
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


def as_optional_vector(value, *, dim, field_name, signal_id):
    """Convert one metadata field (a bound) to a float vector.

    ``None`` passes through (meaning "use the default"). Anything else must
    flatten to exactly ``dim`` entries, otherwise the declared dimension and
    the supplied metadata disagree, and the error names both lengths.
    """
    if value is None:
        return None
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.shape != (dim,):
        raise ValueError(
            f"{field_name} for signal '{signal_id}' must have length {dim}, "
            f"got {array.shape[0]}"
        )
    return array.copy()
