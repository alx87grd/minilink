"""Contract of the pure-pursuit tracker, of the test circuit and of the rate limiter."""

import numpy as np
import pytest

from minilink.blocks import RateLimiter
from minilink.control import PurePursuit
from minilink.control.geometric import is_closed
from minilink.planning.spatial.paths import circuit_waypoints

jax = pytest.importorskip("jax", reason="the JAX parity checks need jax")
jnp = jax.numpy

WHEELBASE = 0.34
LOOKAHEAD = 0.6


def state_at(x, y, theta, speed, dim=9):
    """Vehicle state in the minilink layout ``[x, y, theta, vx, ...]``."""
    state = np.zeros(dim)
    state[0], state[1], state[2], state[3] = x, y, theta, speed

    return state


def straight_path(length=10.0, n_points=51):
    """Waypoints along ``y = 0``, from the origin forward."""
    return np.column_stack([np.linspace(0.0, length, n_points), np.zeros(n_points)])


def circle_path(radius, n_points=721):
    """Waypoints on a circle centred on the origin, counter-clockwise, not closed up."""
    angles = np.linspace(0.0, 2.0 * np.pi, n_points, endpoint=False)

    return radius * np.column_stack([np.cos(angles), np.sin(angles)])


# The test circuit


def test_circuit_is_a_closed_loop_with_even_spacing():
    """Waypoints go once around, evenly spaced, with no repeated point."""
    path = circuit_waypoints(length=10.0, width=6.0, radius=1.5, spacing=0.2)

    steps = np.linalg.norm(np.diff(path, axis=0), axis=1)

    assert steps.min() > 0.0
    assert steps.max() == pytest.approx(steps.min(), rel=2e-3)
    assert steps.max() == pytest.approx(0.2, rel=0.05)
    assert np.linalg.norm(path[-1] - path[0]) == pytest.approx(steps[0], rel=2e-3)
    assert path[:, 0].max() == pytest.approx(5.0, abs=0.05)
    assert path[:, 1].max() == pytest.approx(3.0, abs=0.05)


def test_circuit_turns_counter_clockwise():
    """The signed area of the loop is positive."""
    path = circuit_waypoints()

    shifted = np.roll(path, -1, axis=0)
    area = 0.5 * np.sum(path[:, 0] * shifted[:, 1] - shifted[:, 0] * path[:, 1])

    assert area > 0.0


def test_a_loop_is_read_as_closed_and_a_line_as_open():
    """The closing gap tells the two apart, and the tracker picks it up by itself."""
    assert is_closed(circuit_waypoints()) is True
    assert is_closed(straight_path()) is False
    assert PurePursuit(circuit_waypoints()).params["closed"] is True
    assert PurePursuit(straight_path()).params["closed"] is False
    assert PurePursuit(straight_path(), closed=True).params["closed"] is True


def test_a_path_needs_at_least_two_points_in_the_plane():
    """A malformed path is refused at construction, not at the first solver step."""
    with pytest.raises(ValueError):
        PurePursuit(np.zeros((1, 2)))
    with pytest.raises(ValueError):
        PurePursuit(np.zeros((5, 3)))


# The steering law


def test_straight_path_ahead_needs_no_steering():
    """A car already on a straight line, pointing along it, holds the wheel still."""
    tracker = PurePursuit(straight_path(), wheelbase=WHEELBASE, lookahead=LOOKAHEAD)

    delta = tracker.ctl(np.zeros(0), state_at(1.0, 0.0, 0.0, 3.0))

    assert delta[0] == pytest.approx(0.0, abs=1e-9)


def test_steering_follows_the_side_the_path_is_on():
    """A path to the left steers left, a path to the right steers right."""
    tracker = PurePursuit(straight_path(), wheelbase=WHEELBASE, lookahead=LOOKAHEAD)

    from_the_right = tracker.ctl(np.zeros(0), state_at(1.0, -0.3, 0.0, 3.0))
    from_the_left = tracker.ctl(np.zeros(0), state_at(1.0, 0.3, 0.0, 3.0))

    assert from_the_right[0] > 0.0
    assert from_the_left[0] == pytest.approx(-from_the_right[0], rel=1e-6)


def test_steering_matches_the_pursuit_arc():
    """``delta = arctan(L * 2 y_t / d^2)`` for the target the tracker picked."""
    straight = straight_path(n_points=101)
    tracker = PurePursuit(
        straight, wheelbase=WHEELBASE, lookahead=1.0, lookahead_gain=0.0
    )
    offset = 0.25

    delta = tracker.ctl(np.zeros(0), state_at(0.0, offset, 0.0, 2.0))

    # the chosen target is the first sample at least 1 m away, on the line y = 0
    reach = np.hypot(straight[:, 0], straight[:, 1] - offset)
    target = straight[np.argmax(reach >= 1.0)]
    x_t, y_t = target[0], target[1] - offset
    assert delta[0] == pytest.approx(
        np.arctan(WHEELBASE * 2.0 * y_t / (x_t**2 + y_t**2)), rel=1e-9
    )


@pytest.mark.parametrize("radius", [1.0, 2.5, 8.0])
@pytest.mark.parametrize("lookahead", [0.05, 0.6, 2.0])
def test_a_car_on_a_circle_holds_the_kinematic_steer_angle(radius, lookahead):
    """On a circle of radius R the law gives ``arctan(L / R)`` — at any lookahead.

    The chord from the car to a target on the same circle subtends the tangent by
    ``eta`` with ``d = 2 R sin(eta)``, so the pursuit curvature ``2 sin(eta) / d`` is
    ``1 / R`` however far ahead the target sits. Shrinking the lookahead is therefore
    not what makes the answer right — it only makes the target a nearer point of the
    same circle.
    """
    path = circle_path(radius)
    tracker = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=lookahead, lookahead_gain=0.0
    )
    psi = 0.7  # anywhere on the circle; tangent heading for a counter-clockwise lap
    state = state_at(radius * np.cos(psi), radius * np.sin(psi), psi + 0.5 * np.pi, 3.0)

    delta = tracker.ctl(np.zeros(0), state)

    assert delta[0] == pytest.approx(np.arctan(WHEELBASE / radius), abs=1e-9)


def test_lookahead_grows_with_speed():
    """Faster means aiming further ahead, so the same offset asks for less steering."""
    tracker = PurePursuit(
        straight_path(20.0, 201),
        wheelbase=WHEELBASE,
        lookahead=0.5,
        lookahead_gain=0.5,
    )

    slow = tracker.ctl(np.zeros(0), state_at(0.0, 0.4, 0.0, 1.0))
    fast = tracker.ctl(np.zeros(0), state_at(0.0, 0.4, 0.0, 8.0))

    assert abs(fast[0]) < abs(slow[0])


def test_steering_is_clipped_to_the_end_stops():
    """A target abeam of the car still asks only for what the servo can give."""
    tracker = PurePursuit(
        straight_path(),
        wheelbase=WHEELBASE,
        lookahead=0.3,
        lookahead_gain=0.0,
        delta_max=0.52,
    )

    delta = tracker.ctl(np.zeros(0), state_at(0.0, 1.0, 0.0, 2.0))

    assert delta[0] == pytest.approx(-0.52)


def test_the_geometry_is_written_at_the_rear_axle():
    """``rear_offset`` moves the pursuit point back, which asks for more steering.

    The same pose with the geometry written at the centre of gravity aims from a point
    ``rear_offset`` further up the road, so it sees a smaller lateral error.
    """
    straight = straight_path()
    at_the_cg = PurePursuit(
        straight, wheelbase=WHEELBASE, lookahead=1.0, rear_offset=0.0
    )
    at_the_axle = PurePursuit(
        straight, wheelbase=WHEELBASE, lookahead=1.0, rear_offset=0.17
    )
    state = state_at(1.0, 0.3, 0.0, 3.0)

    assert abs(at_the_axle.ctl(np.zeros(0), state)[0]) > abs(
        at_the_cg.ctl(np.zeros(0), state)[0]
    )
    # a state that already locates the rear axle is the zero-offset case
    assert at_the_axle.ctl(np.zeros(0), state_at(1.17, 0.3, 0.0, 3.0))[
        0
    ] == pytest.approx(at_the_cg.ctl(np.zeros(0), state)[0], rel=1e-12)


# The seam of a closed path


def test_a_whole_lap_of_a_circle_holds_the_same_steer_angle():
    """Round a closed circle the law gives ``arctan(L / R)`` at every waypoint.

    The array ends in the middle of the lap, so the last fifth of the poses can only
    be served by a target at the start of it. Searching by array index instead of
    along the path leaves those poses with nothing ahead, and the fallback — the last
    waypoint, the one under the car — divides by a zero-length arc. Here they all
    give the one right answer, so the search crossed the seam.
    """
    radius, n_points = 2.5, 361
    path = circle_path(radius, n_points)
    tracker = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=LOOKAHEAD, lookahead_gain=0.0
    )
    psi = np.linspace(0.0, 2.0 * np.pi, n_points, endpoint=False)

    deltas = np.array(
        [
            tracker.ctl(
                np.zeros(0),
                state_at(point[0], point[1], angle + 0.5 * np.pi, 3.0),
            )[0]
            for point, angle in zip(path, psi)
        ]
    )

    assert np.all(np.isfinite(deltas))
    assert deltas == pytest.approx(np.arctan(WHEELBASE / radius), abs=1e-9)


def test_a_lap_of_the_circuit_has_no_jump_in_it():
    """Driven along its own waypoints, the circuit asks for one smooth corner each.

    The corner is taken at less than the ``arctan(L / R)`` a steady turn would need:
    a lookahead of a quarter of the corner radius cuts the corner, which is the
    known price of the method, not a defect of the search.
    """
    radius = 2.5
    path = circuit_waypoints(length=14.0, width=9.0, radius=radius)
    tracker = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=LOOKAHEAD, lookahead_gain=0.0
    )

    heading = np.arctan2(*(np.roll(path, -1, axis=0) - path)[:, ::-1].T)
    deltas = np.array(
        [
            tracker.ctl(np.zeros(0), state_at(point[0], point[1], angle, 3.0))[0]
            for point, angle in zip(path, heading)
        ]
    )

    assert np.all(np.isfinite(deltas))
    assert (
        0.5 * np.arctan(WHEELBASE / radius)
        < deltas.max()
        < np.arctan(WHEELBASE / radius)
    )
    assert deltas.min() > -0.01  # four left-hand corners, no right-hand flick
    # and the lap has no jump in it, seam included
    assert np.abs(np.diff(deltas, append=deltas[0])).max() < 0.05


def test_the_target_crosses_the_seam_instead_of_stopping_at_it():
    """At the last waypoint the tracker aims at the start of the array."""
    path = circuit_waypoints(length=14.0, width=9.0, radius=2.5)
    tracker = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=LOOKAHEAD, lookahead_gain=0.0
    )
    heading = np.arctan2(path[0, 1] - path[-1, 1], path[0, 0] - path[-1, 0])
    state = state_at(path[-1, 0], path[-1, 1], heading, 3.0)

    delta = tracker.ctl(np.zeros(0), state)

    # the arc the tracker drew reaches a point of the path a lookahead away, ahead
    curvature = np.tan(delta[0]) / WHEELBASE
    reach = np.linalg.norm(path - path[-1], axis=1)
    wrapped = reach[: len(path) // 2]
    assert np.isfinite(delta[0])
    assert abs(curvature) < 1.0 / LOOKAHEAD
    assert wrapped.max() > LOOKAHEAD  # a target does exist across the seam


def test_an_open_path_stops_at_its_end_instead_of_wrapping():
    """Past the end of a line the car keeps aiming at the last point, not the first."""
    straight = straight_path()
    tracker = PurePursuit(
        straight, wheelbase=WHEELBASE, lookahead=1.0, lookahead_gain=0.0
    )

    # abeam the end of the path, heading along it: the last point is behind and to the
    # left, the first point is far behind — a wrap would swing the wheel hard over
    delta = tracker.ctl(np.zeros(0), state_at(12.0, 0.0, 0.0, 3.0))

    assert delta[0] == pytest.approx(0.0, abs=1e-9)


# Backends


@pytest.mark.parametrize("where", ["corner", "seam", "off the path"])
def test_the_law_traces_under_jax(where):
    """The tracker runs inside a compiled loop, with the same number out."""
    path = circuit_waypoints(length=8.0, width=6.0, radius=1.5)
    tracker = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=LOOKAHEAD, rear_offset=0.17
    )
    poses = {
        "corner": state_at(path[20, 0], path[20, 1], 1.2, 3.0),
        "seam": state_at(path[-1, 0], path[-1, 1], 1.5708, 3.0),
        "off the path": state_at(path[0, 0], path[0, 1] + 0.2, 0.3, 3.0),
    }

    on_numpy = tracker.ctl(np.zeros(0), poses[where])
    on_jax = np.asarray(jax.jit(tracker.ctl)(jnp.zeros(0), jnp.asarray(poses[where])))

    assert np.isfinite(on_numpy[0])
    assert on_jax == pytest.approx(on_numpy, rel=1e-6)


# The rate limiter


def test_a_large_step_leaves_at_the_rate_limit():
    """Far from its target the block ramps at ``rate_max``, to within a percent."""
    block = RateLimiter(rate_max=2.0, tau=0.05)

    dx = block.f(np.zeros(1), np.array([10.0]))

    assert dx[0] == pytest.approx(2.0, rel=1e-2)


def test_a_small_step_is_a_first_order_lag():
    """Inside the knee the block is ``dx = (u - x) / tau``, the ordinary lag."""
    block = RateLimiter(rate_max=2.0, tau=0.05)
    step = 1e-3  # much smaller than the knee tau * rate_max = 0.1

    dx = block.f(np.zeros(1), np.array([step]))

    assert dx[0] == pytest.approx(step / 0.05, rel=1e-4)


def test_the_output_never_passes_an_end_stop():
    """A command beyond the stop is clipped, and the state is pushed back inside."""
    block = RateLimiter(rate_max=2.0, tau=0.05, lower=-1.0, upper=1.0)

    assert block.f(np.array([1.0]), np.array([5.0]))[0] == pytest.approx(0.0)
    assert block.f(np.array([-1.0]), np.array([-5.0]))[0] == pytest.approx(0.0)
    assert block.f(np.array([1.5]), np.array([5.0]))[0] < 0.0
    assert block.f(np.array([-1.5]), np.array([-5.0]))[0] > 0.0


def test_the_ramp_takes_the_time_the_rate_asks_for():
    """A step of 10 units at 2 units/s is followed in 5 s, not sooner."""
    block = RateLimiter(rate_max=2.0, tau=0.05)

    traj = block.compute_forced(np.array([10.0]), tf=6.0, n_steps=601, verbose=False)

    reached = traj.t[np.argmax(traj.x[0] > 9.9)]
    assert traj.x[0].max() == pytest.approx(10.0, abs=1e-2)
    assert reached == pytest.approx(5.0, abs=0.2)
    assert np.diff(traj.x[0]).max() / np.diff(traj.t).max() <= 2.0 + 1e-3


def test_channels_run_independently():
    """Each channel carries its own state and sees its own command."""
    block = RateLimiter(rate_max=1.0, tau=0.05, dim=3, x0=[0.0, 1.0, -1.0])

    dx = block.f(block.x0, np.array([5.0, 1.0, 5.0]))

    assert block.x0 == pytest.approx([0.0, 1.0, -1.0])
    assert dx[0] == pytest.approx(1.0, rel=1e-2)
    assert dx[1] == pytest.approx(0.0, abs=1e-12)
    assert dx[2] > 0.0


def test_an_impossible_rate_limiter_is_refused():
    """A zero rate or a negative time constant has no dynamics to write."""
    with pytest.raises(ValueError):
        RateLimiter(rate_max=0.0)
    with pytest.raises(ValueError):
        RateLimiter(tau=-0.1)
    with pytest.raises(ValueError):
        RateLimiter(lower=1.0, upper=-1.0)


def test_the_rate_limiter_traces_under_jax():
    """Same equation on both backends, inside a compiled step."""
    block = RateLimiter(rate_max=2.0, tau=0.05, lower=-1.0, upper=1.0)
    x, u = np.array([0.3]), np.array([4.0])

    on_numpy = block.f(x, u)
    on_jax = np.asarray(jax.jit(block.f)(jnp.asarray(x), jnp.asarray(u)))

    assert on_jax == pytest.approx(on_numpy, rel=1e-6)
