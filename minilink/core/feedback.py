"""Who is who in a loop: the roles the port names and the reserved subsystem names declare."""

from dataclasses import dataclass

import numpy as np

#: The three subsystem names that say what role a block plays in a diagram. One of
#: each per diagram level; a second one is refused, never suffixed.
RESERVED_IDS = ("plant", "controller", "observer")

#: Marks an unset ``plot_space`` attribute: the ports then decide.
_UNSET = object()


###############################################################################
# Public API
###############################################################################


@dataclass(frozen=True)
class BlockRoles:
    """What a block reads and commands, read from its port names alone.

    A controller is an ordinary :class:`~minilink.core.system.System` (a
    :class:`~minilink.core.system.DynamicSystem` when it has state): its roles
    change nothing in how it computes. ``@``, ``feedback``, the sampled loop,
    ``plot_control_law`` and the evaluators read them; a block with other port
    names is wired by hand.

    Attributes
    ----------
    measurements : tuple of str
        The plant signals the block reads, ``"y"`` and ``"x"`` among its inputs, in
        the order a loop tries them. Empty for an error-driven block or a source.
    reference : str or None
        ``"r"`` when the block has a reference input.
    command : str or None
        ``"u"`` when the block commands one, else ``None``.
    entry : str or None
        The input an ``Error`` block ``e = r - y`` drives when the block reads no
        measurement itself: ``"e"``, or the one input of a plain compensator; a
        disturbance ``d`` or a noise ``n`` input is never the entry.
    stateful : bool
        Whether the block has internal state (``n > 0``).
    """

    measurements: tuple
    reference: str | None
    command: str | None
    entry: str | None
    stateful: bool

    @property
    def measurement(self):
        """The first measurement the block reads, or ``None``."""
        return self.measurements[0] if self.measurements else None

    @property
    def error_driven(self):
        """Whether an ``Error`` block drives the block's entry."""
        return self.entry is not None


def block_roles(block) -> BlockRoles:
    """What ``block`` reads and commands, from its port names (see :class:`BlockRoles`)."""
    inputs = getattr(block, "inputs", {})
    outputs = getattr(block, "outputs", {})

    # The plant signals it reads, its reference and its command
    measurements = tuple(port for port in ("y", "x") if port in inputs)
    reference = "r" if "r" in inputs else None
    command = next((port for port in ("u", "u_ff") if port in outputs), None)

    # The entry an Error block drives: e, else the one free input of a block that
    # reads no measurement
    entry = None
    if "e" in inputs:
        entry = "e"
    elif not measurements:
        free = free_inputs(inputs)
        entry = free[0] if len(free) == 1 else None

    stateful = int(getattr(block, "n", 0)) > 0

    return BlockRoles(measurements, reference, command, entry, stateful)


def command_port(sys) -> str:
    """The input a decision drives on ``sys``: ``u``, else its one input that is not ``d`` or ``n``.

    The design tools (LQR, pole placement, value iteration, trajectory
    optimization, MPC), the cost and the Gym bridge decide this port only; a
    disturbance ``d`` and a measurement noise ``n`` are never decision variables,
    and a cascade's outer loop drives the inner loop's one free input.
    """
    inputs = getattr(sys, "inputs", {})
    if "u" in inputs:
        return "u"

    # Else the one input that is neither a disturbance nor a noise
    free = free_inputs(inputs)
    require_one_free_input(sys, inputs, free)

    return free[0]


def require_command(block) -> str | None:
    """The command output of ``block`` (``u``, else ``u_ff``), or ``None`` to fall back on its default output.

    A block that reads a measurement (``y`` or ``x``) and gives the state ``x``
    back, with no ``u`` and no ``y`` to command through, is an observer: it is
    refused as a controller, with the recipe that makes it one. A compensator
    (no measurement) or a box that commands through an output named ``y`` (a
    gain in series with a saturation) is left to its default output.
    """
    roles = block_roles(block)
    outputs = getattr(block, "outputs", {})

    # An observer reads a measurement and gives the state back, with nothing to
    # command through
    if (
        roles.command is None
        and roles.measurements
        and "x" in outputs
        and "y" not in outputs
    ):
        raise ValueError(
            f"{getattr(block, 'name', block)!r} reads {roles.measurement!r} but gives "
            "no command 'u': an observer, not a controller. Put it in a controller box "
            "with a gain (add_subsystem(observer, 'observer'), add_subsystem(gain, "
            "'gain'), connect its x to the gain and the gain's u back, then "
            "expose('r', 'y', 'u')) and close that box on the plant, or design the "
            "pair with lqg()."
        )

    return roles.command


@dataclass(frozen=True)
class Roles:
    """The plant, the controller and the observer of a diagram, or ``None`` each."""

    plant: object | None
    controller: object | None
    observer: object | None


def roles(system) -> Roles:
    """The plant, the controller and the observer of ``system``, from the reserved names.

    A leaf is its own plant. A diagram answers from its top-level keys ``plant``,
    ``controller`` and ``observer`` (the observer may also sit inside the
    controller); a sampled loop from its ``plant`` and ``computer`` sides. A
    missing name is ``None``: a tool that needs that block raises, and the plots
    and the camera keep their plain defaults.
    """
    # A sampled loop: the plant from its plant side, the laws from its computer
    if hasattr(system, "computer") and hasattr(system, "plant"):
        plant_side = roles(system.plant)
        computer_side = roles(system.computer.diagram)
        return Roles(
            plant_side.plant,
            computer_side.controller,
            computer_side.observer or plant_side.observer,
        )

    # A leaf is its own plant
    subsystems = getattr(system, "subsystems", None)
    if subsystems is None:
        return Roles(system, None, None)

    # A diagram: the blocks under the reserved names, the observer possibly inside
    # the controller
    plant = block_under(subsystems, "plant")
    controller = block_under(subsystems, "controller")
    observer = block_under(subsystems, "observer")
    if observer is None and controller is not None:
        observer = block_under(getattr(controller, "subsystems", {}), "observer")

    return Roles(plant, controller, observer)


def check_reserved_id(sys_id, existing) -> None:
    """Refuse a second block under a reserved name: one of each per diagram level."""
    if sys_id in RESERVED_IDS and sys_id in existing:
        raise ValueError(
            f"the diagram already has a {sys_id!r}: one {sys_id} per diagram level. "
            "Nest the second one in a box, or give it a name of its own."
        )


def plot_space_of(block):
    """The sweep space of ``block``'s law for ``plot_control_law``, or ``None``.

    An explicit ``plot_space`` attribute wins, the one hint the ports cannot give
    (``None`` says the block has no static law to draw, a solver call such as
    MPC); else the ports decide: a block that reads ``x`` sweeps the state, one
    that reads ``y`` and a reference of the same size the error ``e = r - y``
    with ``r`` pinned, one that reads ``y`` otherwise the measurement, and an
    error-driven block its entry port.
    """
    # An explicit plot_space wins, None included
    declared = getattr(block, "plot_space", _UNSET)
    if declared is not _UNSET:
        return declared

    # Else the ports decide: the state, the error e = r − y, or the measurement
    roles = block_roles(block)
    if "x" in roles.measurements:
        return "state"

    if "y" in roles.measurements:
        inputs = block.inputs
        if roles.reference and inputs["r"].dim == inputs["y"].dim:
            return "error"
        return "measurement"

    # An error-driven block sweeps its entry port
    return "measurement" if roles.error_driven else None


class ErrorDriven:
    """Port-layout switch for laws written on the tracking error ``e = r - y``.

    ``ports="error"`` declares one input ``e`` — the compensator form, where
    ``block @ plant`` inserts the Error block and ``block >> plant`` is
    the loop gain. ``ports="reference"`` declares ``r`` and ``y`` — the
    controller form, where ``@`` wires them to the plant. The law reads
    ``e = self.error(u)`` either way, so it is written once.
    """

    PORT_LAYOUTS = ("error", "reference")

    def add_error_ports(self, ports, dim, *, nominal_value=None):
        """Declare the input ports of the chosen layout."""
        require_port_layout(ports, self.PORT_LAYOUTS)

        self.port_layout = ports
        nominal = np.zeros(dim) if nominal_value is None else nominal_value

        # The compensator form reads e alone; the controller form reads r and y
        if ports == "error":
            self.add_input_port("e", dim=dim, nominal_value=nominal)
            self.error_port = "e"
        else:
            self.add_input_port("r", dim=dim, nominal_value=nominal)
            self.add_input_port("y", dim=dim, nominal_value=np.zeros(dim))

    @property
    def error_dependencies(self):
        """Input ports the command feeds through from."""
        return ("e",) if self.port_layout == "error" else ("r", "y")

    def error(self, u):
        """The tracking error from the flat input: ``u`` itself, or ``r - y``."""
        if self.port_layout == "error":
            return u

        p = self.inputs["r"].dim

        r = u[:p]
        y = u[p:]

        e = r - y

        return e


###############################################################################
# Internal machinery
###############################################################################


def free_inputs(inputs):
    """The input ports that are neither a disturbance ``d`` nor a noise ``n``."""
    return [port for port in inputs if port not in ("d", "n")]


def require_one_free_input(sys, inputs, free):
    """Refuse a block with no ``u`` and not exactly one other free input."""
    if len(free) != 1:
        raise ValueError(
            f"{getattr(sys, 'name', sys)!r} needs an input port named 'u' or a single "
            f"input port (besides a disturbance 'd' and a noise 'n'); got {list(inputs)}"
        )


def block_under(subsystems, *names):
    """The block under the first of ``names`` present in ``subsystems``, or ``None``."""
    for name in names:
        if name in subsystems:
            return subsystems[name]
    return None


def require_port_layout(ports, layouts):
    """Refuse a port layout the error-driven law does not know."""
    if ports not in layouts:
        raise ValueError(f"ports must be one of {layouts}, got {ports!r}")
