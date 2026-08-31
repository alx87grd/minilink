"""
Feedback-port declaration: which ports form a block's standard feedback loop.

A controller in minilink is an ordinary :class:`~minilink.core.system.System`:
explicit ports, and a ``ctl`` port compute that *is* the control law. This
module adds the read-only context that tools use to interpret such blocks:

- :data:`PROFILE_PORTS` — the standard port roles implied by a block's
  ``feedback_profile`` string.
- :func:`feedback_ports` — resolve ``(measurement, ref, control)`` roles for
  any block (explicit attrs first, profile registry second).
- :class:`Controller` / :class:`DynamicController` — thin marker/facade base
  classes providing the ``plot_control_law`` shortcut. They add no ports, no
  state, and no behavior.

Nothing here changes how a block computes. Undeclared blocks keep working
everywhere; ``@`` composition falls back to its name/dimension heuristics.
"""

from dataclasses import dataclass

from minilink.core.system import DynamicSystem, System

# Public API


@dataclass(frozen=True)
class FeedbackRoles:
    """Resolved feedback port roles (and plot semantics) for a block.

    Attributes
    ----------
    measurement : str
        Input-port id carrying the feedback measurement (``"y"`` or ``"x"``).
    ref : str or None
        Input-port id carrying the reference, or ``None`` when the block has
        no boundary reference (lookup policies, MPC).
    control : str
        Output-port id carrying the control command.
    plot_space : str or None
        Sweep-space semantics for ``plot_control_law``: ``"error"``,
        ``"error_qdq"``, ``"measurement"``, ``"state"``, ``"absolute_qdq"``,
        or ``None`` when the block has no static law to plot (e.g. MPC).
    """

    measurement: str
    ref: str | None
    control: str
    plot_space: str | None = None


#: ``feedback_profile`` → standard port roles + plot semantics.
PROFILE_PORTS = {
    "error": FeedbackRoles("y", "r", "u", "error"),  # u = c(e), e = r - y
    "output": FeedbackRoles("y", "r", "u", "measurement"),  # general u = c(y, r)
    "state": FeedbackRoles("x", "r", "u", "state"),  # u = c(x), absolute state
    "impedance": FeedbackRoles("y", "r", "u", "error_qdq"),  # y=[pos; rate]
    "modelbased": FeedbackRoles("y", "r", "u", "absolute_qdq"),  # y=[q; dq]
    "task": FeedbackRoles("y", "r", "u", "absolute_qdq"),  # y=[q; dq], FK inside
    "kinematic": FeedbackRoles("y", "r", "u", "measurement"),  # y=q, FK inside
    "siso": FeedbackRoles("y", "r", "u", "error"),  # decoupled PID loops
}

# Sentinel distinguishing "attr not declared" from an explicit ``None``
# (e.g. ``ref_port = None`` on a block with no boundary reference).
_UNSET = object()


def feedback_ports(block):
    """Resolve the feedback port roles declared by ``block``, or ``None``.

    Priority per role: explicit class attr (``measurement_port`` /
    ``ref_port`` / ``control_port`` / ``plot_space``) → the block's
    ``feedback_profile`` row in :data:`PROFILE_PORTS` → ``None``.

    Returns ``None`` unless the resolved measurement and control port names
    actually exist on the block — the declaration never guesses silently;
    callers fall back to heuristics or raise their own precise error.
    """
    profile_roles = PROFILE_PORTS.get(getattr(block, "feedback_profile", None))

    def resolve(attr, default):
        value = getattr(block, attr, _UNSET)
        return default if value is _UNSET else value

    defaults = profile_roles or FeedbackRoles(None, None, None, None)
    measurement = resolve("measurement_port", defaults.measurement)
    ref = resolve("ref_port", defaults.ref)
    control = resolve("control_port", defaults.control)
    plot_space = resolve("plot_space", defaults.plot_space)

    if measurement is None or control is None:
        return None
    inputs = getattr(block, "inputs", {})
    outputs = getattr(block, "outputs", {})
    if measurement not in inputs or control not in outputs:
        return None
    if ref is not None and ref not in inputs:
        ref = None
    return FeedbackRoles(measurement, ref, control, plot_space)


class Controller(System):
    """A :class:`System` wired as a feedback controller (marker + facade).

    Adds no ports, no state, and no behavior — subclasses declare ports and
    the ``ctl`` port compute exactly like any :class:`System`. Subclassing
    (or duck-typing the same attrs on a plain ``System``) gives tools the
    declaration context:

    - ``feedback_profile`` names the law family (see :data:`PROFILE_PORTS`);
    - explicit ``measurement_port`` / ``ref_port`` / ``control_port`` /
      ``plot_space`` attrs override the profile defaults for nonstandard
      blocks.

    The declaration drives ``ctl @ plant`` wiring and the
    :meth:`plot_control_law` teaching shortcut; it never changes how the
    block computes.
    """

    def plot_control_law(self, **kwargs):
        """Plot the textbook slice of this block's control law.

        Zero-argument call sweeps the most relevant coordinates for the
        declared ``feedback_profile`` and pins everything else at nominal.
        See :func:`minilink.graphical.port_map.plot_control_law`.
        """
        from minilink.graphical.port_map import plot_control_law

        return plot_control_law(self, **kwargs)


class DynamicController(Controller, DynamicSystem):
    """Controller with internal state ``x`` (filters, integrators, observers).

    ``plot_control_law`` draws the instantaneous map with the internal state
    pinned at ``x0`` by default; ``x_axis`` / ``y_axis`` index the workspace
    ``z = [measurement-space; x_ctrl]`` for teaching slices (e.g. windup).
    """
