"""A feedback controller block read as its law, the function ``u = π(x, t)`` the step tools call."""

import numpy as np

from minilink.core.roles import subsystem_roles

###############################################################################
# Public API
###############################################################################


def policy_from_block(block):
    """Adapt a feedback controller block into its policy, a ``u = policy(x, t=0.0)`` callable.

    The block is evaluated through its control-port compute at its own
    initial state ``x0``, with ``x`` on the measurement port, every other
    input (a reference) pinned at its port nominal, and the time ``t``.
    """
    roles = subsystem_roles(block)
    require_feedback_roles(block, roles)

    output = block.outputs[roles.command]
    measurement = block.get_input_port_slice(roles.measurement)
    u_base = block.u_nominal.astype(float)
    x_ctrl = np.asarray(block.x0, dtype=float)

    # The command of the block at t, the state on its measurement port
    def policy(x, t=0.0):
        u = u_base.copy()
        u[measurement] = np.asarray(x, dtype=float).reshape(-1)
        command = np.asarray(output.compute(x_ctrl, u, t, None), dtype=float)
        command = command.reshape(-1)

        return command

    return policy


###############################################################################
# Internal machinery
###############################################################################


def require_feedback_roles(block, roles):
    """Refuse a block that does not read the state on ``x`` (or ``y``) and command ``u``."""
    if roles.measurement is None or roles.command is None:
        raise ValueError(
            "policy must be a callable u = policy(x) or a controller block that "
            "reads the state on an input named 'x' (or 'y') and commands 'u'; "
            f"{type(block).__name__} reads {list(block.inputs)} and gives "
            f"{list(block.outputs)}"
        )
