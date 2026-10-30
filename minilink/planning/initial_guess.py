"""Initial guesses: the first trajectory a planning problem's solver starts from."""

import numpy as np

from minilink.core.sets import SingletonSet
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import PlanningProblem

###############################################################################
# Public API
###############################################################################


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

    # Duck-typed mechanical check (a tools package must not import dynamics/;
    # see the DESIGN.md dependency law)
    if not hasattr(sys, "dof") or int(sys.n) != 2 * int(sys.dof):
        return linear_initial_trajectory(problem, t)

    # A time grid without a positive span gets the straight line
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    if t_arr.size < 2 or t_arr[-1] <= t_arr[0]:
        return linear_initial_trajectory(problem, t_arr)

    x0, xf = boundary_states_of(problem)
    dof = int(sys.dof)
    duration = float(t_arr[-1] - t_arr[0])

    # s = (t − t₀) / T, the normalized time on [0, 1]
    s = (t_arr - t_arr[0]) / duration

    # The boundary positions and velocities of x = [q; q̇]
    q0 = x0[:dof]
    dq0 = x0[dof:]
    qf = xf[:dof]
    dqf = xf[dof:]

    # The cubic Hermite basis on [0, 1]
    h00 = 2.0 * s**3 - 3.0 * s**2 + 1.0
    h10 = s**3 - 2.0 * s**2 + s
    h01 = -2.0 * s**3 + 3.0 * s**2
    h11 = s**3 - s**2

    # Its rates in t: d/dt = (1/T) d/ds, whose 1/T cancels the T of the velocity terms
    dh00 = (6.0 * s**2 - 6.0 * s) / duration
    dh10 = 3.0 * s**2 - 4.0 * s + 1.0
    dh01 = (-6.0 * s**2 + 6.0 * s) / duration
    dh11 = 3.0 * s**2 - 2.0 * s

    # q(s) = h₀₀ q₀ + T h₁₀ q̇₀ + h₀₁ q_f + T h₁₁ q̇_f, one column per sample
    q = (
        q0[:, None] * h00[None, :]
        + duration * dq0[:, None] * h10[None, :]
        + qf[:, None] * h01[None, :]
        + duration * dqf[:, None] * h11[None, :]
    )

    # q̇(s) = (h₀₀′ q₀ + T h₁₀′ q̇₀ + h₀₁′ q_f + T h₁₁′ q̇_f) / T
    dq = (
        q0[:, None] * dh00[None, :]
        + dq0[:, None] * dh10[None, :]
        + qf[:, None] * dh01[None, :]
        + dqf[:, None] * dh11[None, :]
    )

    x = np.vstack((q, dq))
    u = nominal_input(sys, t_arr.size)

    return Trajectory(t=t_arr, x=x, u=u)


def linear_initial_trajectory(
    problem: PlanningProblem,
    t: np.ndarray,
) -> Trajectory:
    """
    Return a linear state interpolation with nominal input.

    This generic guess works for any state-space system: from the start to
    :meth:`~minilink.planning.problems.PlanningProblem.goal_point`.
    """
    t_arr = np.asarray(t, dtype=float).reshape(-1)
    x0, xf = boundary_states_of(problem)

    # x(t) on the straight line from x₀ to x_f, one column per sample
    x = np.linspace(x0, xf, t_arr.size).T
    u = nominal_input(problem.sys, t_arr.size)

    return Trajectory(t=t_arr, x=x, u=u)


###############################################################################
# Internal machinery
###############################################################################


def boundary_states_of(problem):
    """
    The start and goal states of ``problem``.

    The start is the point of a singleton ``X0``, else ``x_start``; the goal is
    the problem's :meth:`~minilink.planning.problems.PlanningProblem.goal_point`
    (``Xf``'s point, else the cost's ``xbar``, else the start).
    """
    x0 = boundary_point(problem.X0)
    if x0 is None:
        x0 = problem.x_start

    xf = problem.goal_point()

    return x0, xf


def boundary_point(set_: object | None) -> np.ndarray | None:
    """Return the point for singleton boundary sets, else ``None``."""
    if isinstance(set_, SingletonSet):
        return set_.point.copy()
    return None


def nominal_input(sys, n_samples: int) -> np.ndarray:
    """Return the system nominal input repeated on a time grid."""
    u0 = sys.get_u_from_input_ports()
    u0_column = np.asarray(u0, dtype=float).reshape(sys.m, 1)
    U = np.repeat(u0_column, n_samples, axis=1)

    return U
