"""Modal analysis: linearize, eig(A), optional mode animation."""

import numpy as np

from minilink.analysis.linearize import linearize_matrices
from minilink.core.trajectory import Trajectory


def modal_analysis(
    sys,
    x_bar,
    u_bar=None,
    *,
    t=0.0,
    params=None,
    method="fd",
    epsilon=1e-6,
):
    """
    Linearize about ``(x_bar, u_bar)`` and eigendecompose ``A``.

    Parameters
    ----------
    sys : System
        Plant or other system with ``f`` (and ``h``).
    x_bar : array of shape (n,)
        Operating-point state.
    u_bar : array of shape (m,), optional
        Operating-point input. Defaults to port nominals.
    method : {"fd", "jax"}
        Linearization method passed to
        :func:`~minilink.analysis.linearize.linearize_matrices`.

    Returns
    -------
    poles : ndarray of shape (n,)
        Eigenvalues of ``A``.
    modes : ndarray of shape (n, n)
        Eigenvectors (columns are mode shapes in perturbation coordinates).
    """
    x_bar = np.asarray(x_bar, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = sys.get_u_from_input_ports()
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)

    A, _, _, _ = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        t=t,
        params=params,
        epsilon=epsilon,
        method=method,
    )
    return np.linalg.eig(A)


def animate_modal(
    plant,
    x_bar,
    mode="all",
    u_bar=None,
    *,
    t=0.0,
    params=None,
    method="fd",
    epsilon=1e-6,
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
    Linearize, excite selected mode(s), and animate on ``plant``.

    Trajectories use absolute states ``x = x_bar + Δx(t)`` so the original
    plant kinematics apply. Calls :func:`modal_analysis`, then
    :meth:`~minilink.core.facades.SystemFacades.animate` once per mode.

    Parameters
    ----------
    plant : System
        System used for graphics (usually the nonlinear plant).
    x_bar : array of shape (n,)
        Linearization operating point.
    mode : int or ``'all'``
        Mode index to animate, or every index ``0 … n-1``.
    u_bar : array of shape (m,), optional
        Operating-point input during the animation.

    Returns
    -------
    poles, modes
        Same as :func:`modal_analysis`.
    """
    x_bar = np.asarray(x_bar, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = plant.get_u_from_input_ports()
    u_bar = np.asarray(u_bar, dtype=float).reshape(-1)

    poles, modes = modal_analysis(
        plant,
        x_bar,
        u_bar,
        t=t,
        params=params,
        method=method,
        epsilon=epsilon,
    )

    indices = range(len(poles)) if mode == "all" else [int(mode)]
    for index in indices:
        pole = poles[index]
        vector = modes[:, index]

        horizon = tf
        if horizon is None:
            norm = abs(pole)
            horizon = (
                float(np.clip(4.0 * np.pi / norm + 1.0, 1.0, 30.0))
                if norm > 0.001
                else 5.0
            )

        time = np.linspace(0.0, horizon, n_steps)
        delta_x = amplitude * np.real(vector[:, None] * np.exp(pole * time))
        traj = Trajectory(
            t=time,
            x=x_bar[:, None] + delta_x,
            u=np.tile(u_bar[:, None], (1, n_steps)),
        )
        title = f"Mode {index}: {pole.real:.1f}{pole.imag:+.1f}j"
        plant.animate(
            traj,
            time_factor_video=time_factor_video,
            is_3d=is_3d,
            html=html,
            renderer=renderer,
            native=native,
            scene_title=title,
            show=show,
        )

    return poles, modes


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    poles, modes = modal_analysis(Pendulum(), x_bar=[0.0, 0.0])
    for index, pole in enumerate(poles):
        print(index, pole)
