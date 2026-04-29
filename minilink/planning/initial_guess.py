"""Initial trajectory guesses for planning problems."""

import numpy as np

from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import PlanningProblem


def boundary_point(set_: object | None) -> np.ndarray | None:
    """Return the point for singleton boundary sets, else ``None``."""
    if isinstance(set_, SingletonSet):
        return set_.point.copy()
    return None


def nominal_input(sys: object, n_samples: int) -> np.ndarray:
    """Return the system nominal input repeated on a time grid."""
    if hasattr(sys, "get_u_from_input_ports"):
        u0 = sys.get_u_from_input_ports()
    else:
        u0 = np.zeros(sys.m)
    return np.repeat(np.asarray(u0, dtype=float).reshape(sys.m, 1), n_samples, axis=1)


def linear_initial_trajectory(
    problem: PlanningProblem,
    t: np.ndarray,
) -> Trajectory:
    """
    Return a linear state interpolation with nominal input.

    This generic guess works for any state-space system. It uses singleton
    boundary sets when present and falls back to ``problem.x_start`` /
    ``problem.x_goal`` shortcuts.
    """
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    x0 = boundary_point(problem.X0)
    if x0 is None:
        x0 = problem.x_start
    xf = boundary_point(problem.Xf)
    if xf is None:
        xf = x0 if problem.x_goal is None else problem.x_goal

    x = np.linspace(x0, xf, t_arr.size).T
    return Trajectory(t=t_arr, x=x, u=nominal_input(problem.sys, t_arr.size))


def mechanical_cubic_initial_trajectory(
    problem: PlanningProblem,
    t: np.ndarray,
) -> Trajectory:
    """
    Return a cubic generalized-coordinate guess for mechanical systems.

    The expected state convention is ``x = [q; dq]``. The guess is a cubic
    Hermite curve matching boundary positions and velocities.
    """
    sys = problem.sys
    if not hasattr(sys, "dof") or int(sys.n) != 2 * int(sys.dof):
        return linear_initial_trajectory(problem, t)

    t_arr = np.asarray(t, dtype=float).reshape(-1)
    if t_arr.size < 2 or t_arr[-1] <= t_arr[0]:
        return linear_initial_trajectory(problem, t_arr)

    x0 = boundary_point(problem.X0)
    if x0 is None:
        x0 = problem.x_start
    xf = boundary_point(problem.Xf)
    if xf is None:
        xf = x0 if problem.x_goal is None else problem.x_goal

    dof = int(sys.dof)
    duration = float(t_arr[-1] - t_arr[0])
    s = (t_arr - t_arr[0]) / duration

    q0 = x0[:dof]
    dq0 = x0[dof:]
    qf = xf[:dof]
    dqf = xf[dof:]

    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    h11 = s**3 - s**2

    dh00 = (6.0 * s**2 - 6.0 * s) / duration
    dh10 = 3.0 * s**2 - 4.0 * s + 1.0
    dh01 = (-6.0 * s**2 + 6.0 * s) / duration
    dh11 = 3.0 * s**2 - 2.0 * s

    q = (
        q0[:, None] * h00[None, :]
        + duration * dq0[:, None] * h10[None, :]
        + qf[:, None] * h01[None, :]
        + duration * dqf[:, None] * h11[None, :]
    )
    dq = (
        q0[:, None] * dh00[None, :]
        + dq0[:, None] * dh10[None, :]
        + qf[:, None] * dh01[None, :]
        + dqf[:, None] * dh11[None, :]
    )

    x = np.vstack((q, dq))
    return Trajectory(t=t_arr, x=x, u=nominal_input(sys, t_arr.size))


def default_initial_trajectory(
    problem: PlanningProblem,
    t: np.ndarray,
) -> Trajectory:
    """
    Return the default planning-domain trajectory guess for ``problem``.

    Mechanical systems get a cubic position/velocity guess; other systems get a
    linear state interpolation.
    """
    return mechanical_cubic_initial_trajectory(problem, t)
