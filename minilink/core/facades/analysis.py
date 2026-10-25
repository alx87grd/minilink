"""Analysis shortcuts: a system's Jacobian, linearization, transfer function and equilibria."""


###############################################################################
# Public API
###############################################################################


class SystemAnalysis:
    """
    Analysis shortcuts every :class:`~minilink.core.system.System` has.
    """

    def jacobian(
        self,
        of,
        wrt,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        method="auto",
        eps=1e-6,
    ):
        """Return ``d(of)/d(wrt)`` at an operating point, as a NumPy array.

        ``plant.jacobian("f", "x")`` is ∂f/∂x at the nominal point;
        ``plant.jacobian("f", "u", x_bar, u_bar)`` at ``(x̄, ū)``. ``of`` is
        ``"f"``, an output port id, or a diagram wire ``"block:port"``; ``wrt``
        is ``"x"``, ``"u"``, an input port id, ``"t"``, ``"params"`` (dict
        result), or a wire. See :func:`minilink.analysis.derivatives.jacobian`.
        """
        # Tools import core (DESIGN §3): the analysis band loads on first use
        from minilink.analysis.derivatives import jacobian

        return jacobian(self, of, wrt, x_bar, u_bar, t, params, method=method, eps=eps)


class DynamicAnalysis:
    """
    Analysis shortcuts for :class:`~minilink.core.system.DynamicSystem`: each forwards to its :mod:`minilink.analysis` function.
    """

    def linearize(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
    ):
        """Linearize about ``(x_bar, u_bar)`` and return an ``LTISystem``.

        ``lin = plant.linearize(x_bar)`` gives ``lin.A()``, ``lin.B()``,
        ``lin.C()``, ``lin.D()``. See :func:`minilink.analysis.linearization.linearize`.
        """
        # A real cycle through minilink.core.system
        from minilink.analysis.linearization import linearize

        return linearize(
            self, x_bar, u_bar, t, params, of=of, wrt=wrt, method=method, eps=eps
        )

    def transfer_function(
        self,
        x_bar=None,
        u_bar=None,
        t=0.0,
        params=None,
        *,
        of=None,
        wrt=None,
        method="auto",
        eps=1e-6,
        minimal=None,
    ):
        """Return one SISO channel of the linearization as a ``TransferFunction``.

        See :func:`minilink.analysis.frequency.transfer_function`.
        """
        # A real cycle through minilink.core.system
        from minilink.analysis.frequency import transfer_function

        return transfer_function(
            self,
            x_bar,
            u_bar,
            t,
            params,
            of=of,
            wrt=wrt,
            method=method,
            eps=eps,
            minimal=minimal,
        )

    def find_equilibrium(
        self, x_guess=None, u_bar=None, t=0.0, params=None, *, tol=1e-9
    ):
        """Return a state near ``x_guess`` where ``f`` vanishes.

        See :func:`minilink.analysis.equilibria.find_equilibrium`.
        """
        # Tools import core (DESIGN §3): the analysis band loads on first use
        from minilink.analysis.equilibria import find_equilibrium

        return find_equilibrium(self, x_guess, u_bar, t, params, tol=tol)


class StepAnalysis:
    """
    Analysis shortcuts for :class:`~minilink.core.discrete.step_system.StepSystem`: the Jacobian of ``step`` at a tick ``k``.
    """

    def jacobian(
        self,
        of,
        wrt,
        x_bar=None,
        u_bar=None,
        k=0,
        params=None,
        *,
        method="auto",
        eps=1e-6,
    ):
        """Return ``d(of)/d(wrt)`` at ``(x_bar, u_bar, k)``; ``of="step"`` is the update map.

        See :func:`minilink.analysis.derivatives.jacobian`.
        """
        # Tools import core (DESIGN §3): the analysis band loads on first use
        from minilink.analysis.derivatives import jacobian

        return jacobian(self, of, wrt, x_bar, u_bar, k, params, method=method, eps=eps)
