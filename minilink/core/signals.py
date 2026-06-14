"""
Signals and ports: the wires of a block diagram.

A **signal** is a vector of known dimension plus its human metadata: component
labels, physical units, lower/upper bounds, and a nominal (default) value. A
**port** is a signal with a role on a system boundary:

- :class:`InputPort` — a signal entering the system; its nominal value is the
  constant fallback used when the port is left unconnected.
- :class:`OutputPort` — a signal leaving the system; it carries the compute
  function ``(x, u, t, params) -> value`` and the list of input-port ids it
  directly feeds through from (used for algebraic-loop detection).

The module-level helpers implement *dimension inference*: when a port is
declared without an explicit ``dim``, the dimension is deduced from whichever
metadata the user did provide (nominal value, labels, units, or bounds), and
conflicting lengths raise immediately so dimension bugs surface at
construction time, not at simulation time.
"""

import numpy as np


def _as_optional_vector(value, *, dim, field_name, signal_id):
    """Convert one metadata field (bound or nominal value) to a float vector.

    ``None`` passes through (meaning "use the default"). Anything else must
    flatten to exactly ``dim`` entries, otherwise the declared port dimension
    and the supplied metadata disagree and we raise with both lengths named.
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


def _as_optional_string_list(value, *, dim, field_name, signal_id):
    """Convert one metadata field (labels or units) to a list of ``dim`` strings.

    A bare string is accepted for 1-D signals; sequences must match ``dim``
    exactly, for the same fail-at-construction reason as
    :func:`_as_optional_vector`.
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


def _length_or_none(value, *, field_name, signal_id):
    """Return the vector length implied by one metadata value, or ``None``.

    Used by :func:`_infer_signal_dim` to ask each metadata field "what
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
    A vector signal: a dimension plus human metadata.

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

        # Defaults first, then overwrite with any user-provided metadata.
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

    def default_function(self, x=None, u=None, t=0, params=None):
        """Default compute function returning the nominal value."""
        return self.nominal_value
