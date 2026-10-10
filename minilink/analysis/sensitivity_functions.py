"""Sensitivity functions of a feedback loop, built from its plant, controller and filter."""

from __future__ import annotations

import numpy as np

from minilink.analysis.frequency import transfer_function
from minilink.analysis.linearization import output_selectors


def sensitivity(
    *,
    plant,
    controller,
    filter=None,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method: str = "auto",
    eps: float = 1e-6,
):
    """Return the sensitivity function ``S`` of the loop as a ``TransferFunction``.

    ``S`` maps an output disturbance to ``y`` and the reference ``r`` to the
    error ``e``. The loop is the plant ``H``, the controller ``C`` and the
    optional filter ``F`` on the measurement, each a SISO ``System``
    linearized first.

    Parameters
    ----------
    plant : System
        The plant ``H``, from its first input to its output.
    controller : System
        The controller ``C``, from the error (or the reference) to the command.
    filter : System, optional
        The filter ``F`` on the measurement; ``F = 1`` when omitted.
    x_bar, u_bar, t, params : optional
        The plant's operating point, as in
        :func:`~minilink.analysis.frequency.bode`; the controller and the
        filter are taken at their nominal point.
    method : {"auto", "fd", "jax"}, optional
        Differentiation backend of the linearizations.
    eps : float, optional
        Central-difference step.

    Returns
    -------
    TransferFunction
        ``S(s)``, ready for :func:`~minilink.analysis.frequency.bode`,
        ``plot_bode`` or a simulation.
    """
    H, C, F = loop_pieces(
        plant, controller, filter, x_bar, u_bar, t, params, method, eps
    )

    # L(s) = C(s) H(s) F(s)
    L_num = np.polymul(np.polymul(C.numerator, H.numerator), F.numerator)
    L_den = np.polymul(np.polymul(C.denominator, H.denominator), F.denominator)

    # S = 1 / (1 + L)
    S_num = L_den
    S_den = np.polyadd(L_den, L_num)

    return loop_function(S_num, S_den, "Sensitivity S")


def complementary_sensitivity(
    *,
    plant,
    controller,
    filter=None,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method: str = "auto",
    eps: float = 1e-6,
):
    """Return the complementary sensitivity ``T``, the map from ``r`` to ``y``.

    Same arguments as :func:`sensitivity`; without a filter, ``S + T = 1``.
    """
    H, C, F = loop_pieces(
        plant, controller, filter, x_bar, u_bar, t, params, method, eps
    )

    # L(s) = C(s) H(s) F(s)
    L_num = np.polymul(np.polymul(C.numerator, H.numerator), F.numerator)
    L_den = np.polymul(np.polymul(C.denominator, H.denominator), F.denominator)

    # T = C H / (1 + L)
    T_num = np.polymul(np.polymul(C.numerator, H.numerator), F.denominator)
    T_den = np.polyadd(L_den, L_num)

    return loop_function(T_num, T_den, "Complementary sensitivity T")


def load_sensitivity(
    *,
    plant,
    controller,
    filter=None,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method: str = "auto",
    eps: float = 1e-6,
):
    """Return the load sensitivity ``PS``, the map from a disturbance ``w`` at the plant input to ``y``.

    Same arguments as :func:`sensitivity`.
    """
    H, C, F = loop_pieces(
        plant, controller, filter, x_bar, u_bar, t, params, method, eps
    )

    # L(s) = C(s) H(s) F(s)
    L_num = np.polymul(np.polymul(C.numerator, H.numerator), F.numerator)
    L_den = np.polymul(np.polymul(C.denominator, H.denominator), F.denominator)

    # PS = H / (1 + L)
    PS_num = np.polymul(np.polymul(H.numerator, C.denominator), F.denominator)
    PS_den = np.polyadd(L_den, L_num)

    return loop_function(PS_num, PS_den, "Load sensitivity PS")


def noise_sensitivity(
    *,
    plant,
    controller,
    filter=None,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    method: str = "auto",
    eps: float = 1e-6,
):
    """Return the noise sensitivity ``CS``, the map from measurement noise ``v`` to the command ``u``.

    Same arguments as :func:`sensitivity`. In the loop, ``v`` reaches ``u``
    as ``-CS``: the sign of the negative feedback is left out.
    """
    H, C, F = loop_pieces(
        plant, controller, filter, x_bar, u_bar, t, params, method, eps
    )

    # L(s) = C(s) H(s) F(s)
    L_num = np.polymul(np.polymul(C.numerator, H.numerator), F.numerator)
    L_den = np.polymul(np.polymul(C.denominator, H.denominator), F.denominator)

    # CS = C F / (1 + L)
    CS_num = np.polymul(np.polymul(C.numerator, F.numerator), H.denominator)
    CS_den = np.polyadd(L_den, L_num)

    return loop_function(CS_num, CS_den, "Noise sensitivity CS")


# =============================================================================
# Internal machinery
# =============================================================================


def loop_pieces(plant, controller, filter, x_bar, u_bar, t, params, method, eps):
    """``H``, ``C`` and ``F`` as SISO transfer functions; ``F = 1`` without a filter."""
    from minilink.blocks.transfer_function import TransferFunction

    H = siso_piece(plant, "plant", x_bar, u_bar, t, params, method, eps)
    C = siso_piece(controller, "controller", None, None, 0.0, None, method, eps)
    if filter is None:
        F = TransferFunction([1.0], [1.0], name="F")
    else:
        F = siso_piece(filter, "filter", None, None, 0.0, None, method, eps)
    return H, C, F


def siso_piece(sys, role, x_bar, u_bar, t, params, method, eps):
    """The transfer function of one loop piece; a piece with several channels is refused."""
    outputs = output_selectors(sys, None)
    p = sys.n if outputs is None else sum(sys.outputs[name].dim for name, _ in outputs)
    m = next(iter(sys.inputs.values())).dim if sys.inputs else 0
    if m != 1 or p != 1:
        raise ValueError(
            f"The {role} {sys.name!r} must be SISO, got {m} input(s) and {p} "
            f"output(s). Select one channel first: "
            f"{role}=transfer_function({role}, of=..., wrt=...)."
        )
    return transfer_function(
        sys, x_bar, u_bar, t, params, method=method, eps=eps, minimal=True
    )


def loop_function(numerator, denominator, name):
    """The closed-loop function as a ``TransferFunction`` block."""
    from minilink.blocks.transfer_function import TransferFunction

    return TransferFunction(
        np.atleast_1d(numerator), np.atleast_1d(denominator), name=name
    )
