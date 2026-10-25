"""Modal analysis: the poles and mode shapes of the linearized dynamics, and their animation."""

import numpy as np

from minilink.analysis.derivatives import jacobian, operating_point
from minilink.analysis.linearization import require_own_point
from minilink.core.trajectory import Trajectory

###############################################################################
# Public API
###############################################################################


def modal_analysis(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="auto",
    eps=1e-6,
):
    """
    Linearize about ``(x_bar, u_bar)`` and eigendecompose ``A``.

    Parameters
    ----------
    sys : System
        Plant or other system with ``f`` (and ``h``).
    x_bar : array of shape (n,), optional
        Operating-point state. Defaults to the nominal state.
    u_bar : array of shape (m,), optional
        Operating-point input. Defaults to port nominals.
    t : float, optional
        Time at which the Jacobian is evaluated.
    params : dict, optional
        Parameter set; default the live ``sys.params``.
    method : {"auto", "fd", "jax"}
        Differentiation backend, see :func:`~minilink.analysis.derivatives.jacobian`.
    eps : float, optional
        Central-difference step.

    Returns
    -------
    poles : ndarray of shape (n,)
        Eigenvalues of ``A``.
    modes : ndarray of shape (n, n)
        Eigenvectors (columns are mode shapes in perturbation coordinates).
    """
    require_own_point(sys, x_bar, u_bar)

    # A = ∂f/∂x at (x_bar, u_bar)
    A = jacobian(sys, "f", "x", x_bar, u_bar, t, params, method=method, eps=eps)

    # A V = V Λ: the poles λᵢ and the mode shapes vᵢ
    poles, modes = np.linalg.eig(A)

    return poles, modes


def animate_modal(
    sys,
    x_bar=None,
    u_bar=None,
    t=0.0,
    params=None,
    *,
    mode="all",
    method="auto",
    eps=1e-6,
    amplitude=1.0,
    tf=None,
    n_steps=2001,
    time_factor_video=3.0,
    renderer="matplotlib",
    is_3d=False,
    show=True,
    html=None,
    native=True,
):
    """
    Linearize, excite selected mode(s), and animate them on ``sys``.

    Each mode is animated as an absolute state, the operating point plus the
    mode's motion, so the system's own kinematics draw it. Calls
    :func:`modal_analysis`, then
    :meth:`~minilink.core.facades.animation.SystemAnimation.animate` once per mode.

    Parameters
    ----------
    sys : System
        System used for graphics (usually the nonlinear plant).
    x_bar : array of shape (n,), optional
        Linearization operating point. Defaults to the nominal state.
    u_bar : array of shape (m,), optional
        Operating-point input during the animation.
    mode : int or ``'all'``
        Mode index to animate, or every index ``0 … n-1``.

    Returns
    -------
    poles, modes
        Same as :func:`modal_analysis`.
    """
    # The operating point: the one given, else the nominal
    x_bar, u_bar, params = operating_point(sys, x_bar, u_bar, params)

    # The poles and mode shapes of A = ∂f/∂x at the operating point
    poles, modes = modal_analysis(sys, x_bar, u_bar, t, params, method=method, eps=eps)

    # The modes to animate: the one asked for, or every one
    indices = range(len(poles)) if mode == "all" else [int(mode)]
    for index in indices:
        # Mode i: its pole λᵢ and its shape vᵢ
        pole = poles[index]
        vector = modes[:, index]

        # Time long enough for a few periods, or for the decay
        time = np.linspace(0.0, mode_horizon(pole, tf), n_steps)

        # Δx(t) = amp · Re{ v e^{λ t} }
        delta_x = amplitude * np.real(vector[:, None] * np.exp(pole * time))

        # x(t) = x_bar + Δx(t), under the input held at u_bar
        x = x_bar[:, None] + delta_x
        u = np.tile(u_bar[:, None], (1, n_steps))

        # The mode as an absolute trajectory, drawn by the system's own kinematics
        traj = Trajectory(t=time, x=x, u=u)
        title = f"Mode {index}: {pole.real:.1f}{pole.imag:+.1f}j"
        inline = sys.animate(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=html,
            renderer=renderer,
            native=native,
            scene_title=title,
            show=show,
        )
        display_inline(inline)

    return poles, modes


###############################################################################
# Internal machinery
###############################################################################


def mode_horizon(pole, tf):
    """``tf``, or about two periods of the mode (clipped to 1–30 s), 5 s for a pole near 0."""
    if tf is not None:
        return tf

    # Two periods 2 · 2π/|λ| and a second, within 1–30 s; 5 s when |λ| ≈ 0
    norm = abs(pole)
    horizon = 5.0
    if norm > 0.001:
        horizon = float(np.clip(4.0 * np.pi / norm + 1.0, 1.0, 30.0))

    return horizon


def display_inline(animation):
    """Show the inline animation a notebook renderer returned (``html=True``); nothing otherwise."""
    if animation is None:
        return

    # IPython is optional: it loads only to show a notebook's inline animation
    from IPython.display import display

    display(animation)


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    poles, modes = modal_analysis(Pendulum(), x_bar=[0.0, 0.0])
    for index, pole in enumerate(poles):
        print(index, pole)
