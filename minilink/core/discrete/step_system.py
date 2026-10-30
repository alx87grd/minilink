"""The discrete-time system: ``x_{k+1} = step(x_k, u_k, k; p)`` on an integer index ``k``."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.facades.analysis import StepAnalysis
from minilink.core.facades.plotting import StepPlotting
from minilink.core.facades.simulation import StepSimulation
from minilink.core.system import System, add_standard_ports

###############################################################################
# Public API
###############################################################################


class StepSystem(StepSimulation, StepAnalysis, StepPlotting, System):
    """
    A system with discrete states (``n >= 1``)

        x_{k+1} = step(x, u, k; p)
        y_k     = h(x, u, k; p)

    The constructor can create the standard ports in one line: an input ``u``,
    a primary output ``y`` wired to :meth:`~System.h`, and optionally an
    auxiliary state output ``x``.
    """

    #: The clock's period in seconds; a tick law has none
    sample_period = None

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
        Initialize a StepSystem.

        Parameters
        ----------
        n : int
            Number of discrete states.
        input_dim : int, optional
            If provided, create a standard input port named ``u``.
        output_dim : int, optional
            If provided, create a standard primary output port named ``y``.
        expose_state : bool, optional
            If True, create an auxiliary state output port named ``x``.
        y_dependencies : tuple or "all", optional
            Direct-feedthrough dependencies for the standard ``y`` output.
        """
        require_step_state(n)
        System.__init__(self, n)

        # Ports, each when asked: the input u, the output y_k = h(x, u), the state x
        add_standard_ports(self, input_dim, output_dim, expose_state, y_dependencies)

    def step(self, x, u, k=0, params=None):
        """
        Discrete state update ``x_{k+1} = step(x, u, k; p)``.

        Parameters
        ----------
        x : array of shape (n,)
        u : array of shape (m,)
        k : int
        params : dict, optional
            ``None`` means "use ``self.params``".

        Returns
        -------
        x_next : array of shape (n,)
        """
        xp = array_module(x)

        n = self.n

        # x_{k+1} = x_k, as a fresh copy of the state
        x_next = xp.array(x).reshape(n)

        return x_next

    def h(self, x, u, k=0, params=None):
        """
        Output ``y_k = h(x, u, k; p)``.

        Default: ``y = x`` when ``p == n``; zeros otherwise.

        Parameters
        ----------
        x : array of shape (n,)
        u : array of shape (m,)
        k : int
        params : dict, optional

        Returns
        -------
        y : array of shape (p,)
        """
        p = self.p
        n = self.n

        # y_k = x_k when the output carries the full state (p = n), y_k = 0 otherwise
        if p == n:
            return x

        y = np.zeros(p)

        return y


###############################################################################
# Internal machinery
###############################################################################


def require_step_state(n):
    """Refuse a step system with no state to update."""
    if int(n) < 1:
        raise ValueError("StepSystem requires n >= 1")
