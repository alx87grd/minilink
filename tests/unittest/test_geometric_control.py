"""Contract of the geometric path-tracking laws, of the paths they read, and of the heading loop."""

import numpy as np
import pytest

from minilink.control import HeadingController, LineOfSight, PurePursuit, Stanley
from minilink.core.geometry import circuit_waypoints, from_waypoints, is_closed

WHEELBASE = 0.34
LOOKAHEAD = 0.6


def state_at(x, y, theta, speed, dim=9):
    """Vehicle state in the minilink layout ``[x, y, theta, vx, ...]``."""
    state = np.zeros(dim)
    state[0], state[1], state[2], state[3] = x, y, theta, speed

    return state


def straight_path(length=10.0, n_points=51):
    """An open path along ``y = 0``, from the origin forward."""
    waypoints = np.column_stack(
        [np.linspace(0.0, length, n_points), np.zeros(n_points)]
    )

    return from_waypoints(waypoints)


def circle_path(radius, n_points=3601):
    """A closed path on a circle centred on the origin, counter-clockwise."""
    angles = np.linspace(0.0, 2.0 * np.pi, n_points, endpoint=False)
    waypoints = radius * np.column_stack([np.cos(angles), np.sin(angles)])

    return from_waypoints(waypoints, closed=True)


def circuit_path(**dimensions):
    """The closed test circuit as a path."""
    return from_waypoints(circuit_waypoints(**dimensions), closed=True)


# The path


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
    """The closing gap tells the two apart; a path is open unless told it loops."""
    line = np.column_stack([np.linspace(0.0, 10.0, 51), np.zeros(51)])

    assert is_closed(circuit_waypoints()) is True
    assert is_closed(line) is False
    assert from_waypoints(circuit_waypoints()).closed is False
    assert from_waypoints(circuit_waypoints(), closed=True).closed is True


def test_a_path_needs_at_least_two_points():
    """A malformed path is refused at construction, not at the first solver step."""
    with pytest.raises(ValueError):
        from_waypoints(np.zeros((1, 2)))


def test_a_closed_path_wraps_round_its_closing_segment():
    """A loop's length includes the closing gap, and its arc length wraps there."""
    waypoints = circuit_waypoints(length=12.0, width=8.0, radius=2.0)
    loop = from_waypoints(waypoints, closed=True)
    line = from_waypoints(waypoints)
    gap = np.linalg.norm(waypoints[0] - waypoints[-1])
    L = loop.total_length

    # the gap between the last waypoint and the first belongs to the loop only
    middle = 0.5 * (waypoints[0] + waypoints[-1])
    s, c = loop.project(middle)

    assert L == pytest.approx(line.total_length + gap, rel=1e-12)
    assert loop.sample(L + 0.3) == pytest.approx(loop.sample(0.3), abs=1e-12)
    assert loop.sample(-0.3) == pytest.approx(loop.sample(L - 0.3), abs=1e-12)
    assert c == pytest.approx(middle, abs=1e-12)
    assert s == pytest.approx(L - 0.5 * gap, rel=1e-12)


def test_an_open_path_holds_its_ends():
    """Past either end of an open path, the point is that end."""
    line = straight_path()

    assert line.sample(-1.0) == pytest.approx([0.0, 0.0])
    assert line.sample(15.0) == pytest.approx([10.0, 0.0])


# Pure pursuit


def test_pursuit_on_a_straight_path_ahead_needs_no_steering():
    """A car already on a straight line, pointing along it, holds the wheel still."""
    law = PurePursuit(straight_path(), wheelbase=WHEELBASE, lookahead=LOOKAHEAD)

    delta = law.ctl(np.zeros(0), state_at(1.0, 0.0, 0.0, 3.0))

    assert delta[0] == pytest.approx(0.0, abs=1e-12)


def test_pursuit_steers_toward_the_side_the_path_is_on():
    """A path to the left steers left, a path to the right steers right, alike."""
    law = PurePursuit(straight_path(), wheelbase=WHEELBASE, lookahead=LOOKAHEAD)

    from_the_right = law.ctl(np.zeros(0), state_at(1.0, -0.3, 0.0, 3.0))
    from_the_left = law.ctl(np.zeros(0), state_at(1.0, 0.3, 0.0, 3.0))

    assert from_the_right[0] > 0.0
    assert from_the_left[0] == pytest.approx(-from_the_right[0], rel=1e-12)


def test_pursuit_aims_one_lookahead_along_the_path():
    """``δ = arctan(L 2 y_t / d²)`` for the point one lookahead past the projection."""
    law = PurePursuit(
        straight_path(), wheelbase=WHEELBASE, lookahead=1.0, lookahead_gain=0.0
    )
    offset = 0.25

    delta = law.ctl(np.zeros(0), state_at(2.0, offset, 0.0, 2.0))

    # the projection is (2, 0), so the target is (3, 0): one metre down the line
    x_t, y_t = 1.0, -offset
    assert delta[0] == pytest.approx(
        np.arctan(WHEELBASE * 2.0 * y_t / (x_t**2 + y_t**2)), rel=1e-12
    )


@pytest.mark.parametrize("radius", [1.0, 2.5, 8.0])
@pytest.mark.parametrize("lookahead", [0.05, 0.6, 2.0])
def test_pursuit_on_a_circle_holds_the_kinematic_steer_angle(radius, lookahead):
    """On a circle of radius R the law gives ``arctan(L / R)``, at any lookahead.

    The chord from the car to a target on the same circle subtends the tangent by
    ``eta`` with ``d = 2 R sin(eta)``, so the pursuit curvature ``2 sin(eta) / d`` is
    ``1 / R`` however far ahead the target sits. The tolerance is the polygon's: the
    target lies on a chord a hair inside the circle, which tells most over a lookahead
    of a few chords.
    """
    law = PurePursuit(
        circle_path(radius),
        wheelbase=WHEELBASE,
        lookahead=lookahead,
        lookahead_gain=0.0,
    )
    psi = 0.7  # anywhere on the circle; tangent heading for a counter-clockwise lap
    state = state_at(radius * np.cos(psi), radius * np.sin(psi), psi + 0.5 * np.pi, 3.0)

    delta = law.ctl(np.zeros(0), state)

    assert delta[0] == pytest.approx(np.arctan(WHEELBASE / radius), abs=1e-3)


def test_pursuit_lookahead_grows_with_speed():
    """Faster means aiming further ahead, so the same offset asks for less steering."""
    law = PurePursuit(
        straight_path(20.0, 201), wheelbase=WHEELBASE, lookahead=0.5, lookahead_gain=0.5
    )

    slow = law.ctl(np.zeros(0), state_at(0.0, 0.4, 0.0, 1.0))
    fast = law.ctl(np.zeros(0), state_at(0.0, 0.4, 0.0, 8.0))

    assert abs(fast[0]) < abs(slow[0])


def test_pursuit_is_clipped_to_the_end_stops():
    """A target abeam of the car still asks only for what the servo can give."""
    law = PurePursuit(
        straight_path(),
        wheelbase=WHEELBASE,
        lookahead=0.3,
        lookahead_gain=0.0,
        delta_max=0.52,
    )

    delta = law.ctl(np.zeros(0), state_at(1.0, 1.0, 0.0, 2.0))

    assert delta[0] == pytest.approx(-0.52)


def test_pursuit_is_written_at_the_rear_axle():
    """``rear_offset`` moves the pursuit point back along the heading.

    A state that already locates the rear axle is the zero-offset case.
    """
    path = straight_path()
    at_the_cg = PurePursuit(path, wheelbase=WHEELBASE, lookahead=1.0, rear_offset=0.0)
    at_the_axle = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=1.0, rear_offset=0.17
    )

    behind = at_the_axle.ctl(np.zeros(0), state_at(1.17, 0.3, 0.0, 3.0))
    there = at_the_cg.ctl(np.zeros(0), state_at(1.0, 0.3, 0.0, 3.0))

    assert behind[0] == pytest.approx(there[0], rel=1e-12)


def test_pursuit_holds_one_steer_angle_round_a_whole_circle():
    """Round a closed circle the law gives ``arctan(L / R)`` everywhere, seam included."""
    radius = 2.5
    path = circle_path(radius, n_points=721)
    law = PurePursuit(
        path, wheelbase=WHEELBASE, lookahead=LOOKAHEAD, lookahead_gain=0.0
    )
    psi = np.linspace(0.0, 2.0 * np.pi, 360, endpoint=False)

    deltas = np.array(
        [
            law.ctl(
                np.zeros(0),
                state_at(radius * np.cos(a), radius * np.sin(a), a + 0.5 * np.pi, 3.0),
            )[0]
            for a in psi
        ]
    )

    assert deltas == pytest.approx(np.arctan(WHEELBASE / radius), abs=1e-4)


def test_pursuit_laps_the_circuit_without_a_jump():
    """Driven along its own waypoints, the circuit asks for one smooth corner each.

    The corner is taken at less than the ``arctan(L / R)`` a steady turn would need:
    the lookahead cuts the corner, the known price of the method.
    """
    radius = 2.5
    waypoints = circuit_waypoints(length=14.0, width=9.0, radius=radius)
    law = PurePursuit(
        from_waypoints(waypoints, closed=True),
        wheelbase=WHEELBASE,
        lookahead=LOOKAHEAD,
        lookahead_gain=0.0,
    )

    heading = np.arctan2(*(np.roll(waypoints, -1, axis=0) - waypoints)[:, ::-1].T)
    deltas = np.array(
        [
            law.ctl(np.zeros(0), state_at(point[0], point[1], angle, 3.0))[0]
            for point, angle in zip(waypoints, heading)
        ]
    )

    assert (
        0.5 * np.arctan(WHEELBASE / radius)
        < deltas.max()
        < np.arctan(WHEELBASE / radius)
    )
    assert deltas.min() > -0.01  # four left-hand corners, no right-hand flick
    assert np.abs(np.diff(deltas, append=deltas[0])).max() < 0.05


def test_pursuit_aims_at_the_end_of_an_open_path_near_it():
    """Within one lookahead of the end, the target is the end point, never the start."""
    law = PurePursuit(
        straight_path(), wheelbase=WHEELBASE, lookahead=1.0, lookahead_gain=0.0
    )

    delta = law.ctl(np.zeros(0), state_at(9.5, 0.1, 0.0, 3.0))

    # the end point (10, 0) is half a metre ahead and 0.1 m to the right
    x_t, y_t = 0.5, -0.1
    assert delta[0] == pytest.approx(
        np.arctan(WHEELBASE * 2.0 * y_t / (x_t**2 + y_t**2)), rel=1e-12
    )


def test_pursuit_target_is_the_point_the_steer_aims_at():
    """``target`` is one lookahead past the rear axle's projection, and the steer's arc."""
    law = PurePursuit(
        straight_path(),
        wheelbase=WHEELBASE,
        lookahead=1.0,
        lookahead_gain=0.5,
        rear_offset=0.17,
    )
    state = state_at(2.17, 0.25, 0.0, 2.0)

    target = law.target(np.zeros(0), state)
    delta = law.ctl(np.zeros(0), state)

    # the rear axle (2, 0.25) projects to (2, 0), and L_d = 1 + 0.5 * 2 = 2 m further on
    x_t, y_t = 2.0, -0.25
    assert target == pytest.approx([4.0, 0.0], abs=1e-12)
    assert delta[0] == pytest.approx(
        np.arctan(WHEELBASE * 2.0 * y_t / (x_t**2 + y_t**2)), rel=1e-12
    )


# Stanley


def test_stanley_on_a_straight_path_ahead_needs_no_steering():
    """On the line and along it: no heading error, no cross-track error, no steer."""
    law = Stanley(straight_path())

    delta = law.ctl(np.zeros(0), state_at(1.0, 0.0, 0.0, 3.0))

    assert delta[0] == pytest.approx(0.0, abs=1e-12)


def test_stanley_on_the_path_steers_the_heading_error():
    """With the front axle on the path, the steer is the heading error alone."""
    law = Stanley(straight_path())

    delta = law.ctl(np.zeros(0), state_at(1.0, 0.0, -0.2, 3.0))

    assert delta[0] == pytest.approx(0.2, rel=1e-12)


def test_stanley_corrects_the_cross_track_error():
    """Parallel to the path, ``δ = arctan(−k e / (k_s + |vx|))``, toward the path."""
    law = Stanley(straight_path(), gain=2.0, softening=0.5)
    e, vx = 0.3, 3.0

    delta = law.ctl(np.zeros(0), state_at(1.0, e, 0.0, vx))

    assert delta[0] == pytest.approx(np.arctan(-2.0 * e / (0.5 + vx)), rel=1e-12)
    assert delta[0] < 0.0


def test_stanley_is_softened_at_rest_and_clipped():
    """At rest the correction is finite; far off the path it stops at the end stops."""
    law = Stanley(straight_path(), gain=2.0, softening=0.5, delta_max=0.52)

    at_rest = law.ctl(np.zeros(0), state_at(1.0, 0.05, 0.0, 0.0))
    far_off = law.ctl(np.zeros(0), state_at(1.0, 2.0, 0.0, 3.0))

    assert at_rest[0] == pytest.approx(np.arctan(-2.0 * 0.05 / 0.5), rel=1e-12)
    assert far_off[0] == pytest.approx(-0.52)


def test_stanley_is_written_at_the_front_axle():
    """``front_offset`` moves the projected point forward along the heading."""
    path = straight_path()
    at_the_cg = Stanley(path, front_offset=0.17)
    at_the_axle = Stanley(path, front_offset=0.0)

    ahead = at_the_cg.ctl(np.zeros(0), state_at(1.0, 0.0, 0.3, 3.0))
    there = at_the_axle.ctl(
        np.zeros(0), state_at(1.0 + 0.17 * np.cos(0.3), 0.17 * np.sin(0.3), 0.3, 3.0)
    )

    assert ahead[0] == pytest.approx(there[0], rel=1e-12)


# Line of sight


def test_line_of_sight_on_the_path_is_the_path_heading():
    """With no cross-track error the course to steer is the path's own heading."""
    law = LineOfSight(circle_path(2.0), lookahead=1.0)
    psi = 0.7

    reference = law.ctl(
        np.zeros(0), state_at(2.0 * np.cos(psi), 2.0 * np.sin(psi), 0.0, 3.0)
    )

    assert reference[0] == pytest.approx(psi + 0.5 * np.pi, abs=1e-3)


def test_line_of_sight_turns_toward_the_path_by_the_angle_of_sight():
    """Off the path, ``ψ_ref = ψ_p − arctan(e / Δ)``, whatever the car's own heading."""
    law = LineOfSight(straight_path(), lookahead=1.5)
    e = 0.4

    pointing_along = law.ctl(np.zeros(0), state_at(2.0, e, 0.0, 3.0))
    pointing_away = law.ctl(np.zeros(0), state_at(2.0, e, 1.0, 3.0))

    assert pointing_along[0] == pytest.approx(-np.arctan(e / 1.5), rel=1e-12)
    assert pointing_away[0] == pytest.approx(pointing_along[0], rel=1e-12)


def test_line_of_sight_reads_its_control_point():
    """``control_point`` moves the projected point forward along the heading."""
    path = straight_path()
    ahead = LineOfSight(path, lookahead=1.0, control_point=0.5)
    there = LineOfSight(path, lookahead=1.0, control_point=0.0)

    from_ahead = ahead.ctl(np.zeros(0), state_at(1.0, 0.0, 0.4, 3.0))
    from_there = there.ctl(
        np.zeros(0), state_at(1.0 + 0.5 * np.cos(0.4), 0.5 * np.sin(0.4), 0.4, 3.0)
    )

    assert from_ahead[0] == pytest.approx(from_there[0], rel=1e-12)


def test_line_of_sight_is_wrapped_to_a_half_turn():
    """Going round a loop, the reference stays in (−π, π]."""
    law = LineOfSight(circle_path(2.0), lookahead=1.0)
    psi = np.linspace(0.0, 2.0 * np.pi, 73)

    references = np.array(
        [
            law.ctl(
                np.zeros(0),
                state_at(2.1 * np.cos(a), 2.1 * np.sin(a), a + 0.5 * np.pi, 3.0),
            )[0]
            for a in psi
        ]
    )

    assert np.all(references > -np.pi)
    assert np.all(references <= np.pi)


# The heading loop


def test_heading_controller_holds_the_wheel_on_the_reference():
    """Heading on the reference: no steer."""
    law = HeadingController()

    delta = law.ctl(np.zeros(0), np.r_[0.3, state_at(0.0, 0.0, 0.3, 3.0)])

    assert delta[0] == pytest.approx(0.0, abs=1e-12)


def test_heading_controller_inverts_the_kinematic_bicycle():
    """Off the end stops, ``vx tan(δ) / L = k_ψ e_ψ`` with the softened speed."""
    law = HeadingController(gain=2.0, wheelbase=WHEELBASE, softening=0.5)
    e_psi, vx = 0.1, 3.0

    delta = law.ctl(np.zeros(0), np.r_[e_psi, state_at(0.0, 0.0, 0.0, vx)])

    assert (vx + 0.5) * np.tan(delta[0]) / WHEELBASE == pytest.approx(
        2.0 * e_psi, rel=1e-12
    )


def test_heading_controller_turns_the_short_way_round():
    """A reference a full turn away less 0.1 rad steers as 0.1 rad the other way."""
    law = HeadingController()

    wrapped = law.ctl(np.zeros(0), np.r_[2.0 * np.pi - 0.1, state_at(0, 0, 0.0, 3.0)])
    short = law.ctl(np.zeros(0), np.r_[-0.1, state_at(0, 0, 0.0, 3.0)])

    assert wrapped[0] == pytest.approx(short[0], rel=1e-9)
    assert wrapped[0] < 0.0


def test_heading_controller_is_finite_at_rest_and_clipped():
    """At rest the steer is finite; a large error stops at the end stops."""
    law = HeadingController(delta_max=0.52)

    at_rest = law.ctl(np.zeros(0), np.r_[0.05, state_at(0, 0, 0.0, 0.0)])
    large = law.ctl(np.zeros(0), np.r_[1.5, state_at(0, 0, 0.0, 0.5)])

    assert np.isfinite(at_rest[0])
    assert large[0] == pytest.approx(0.52)


# Refusals


@pytest.mark.parametrize(
    ("build", "name"),
    [
        (lambda path: PurePursuit(path, wheelbase=0.0), "wheelbase"),
        (lambda path: PurePursuit(path, lookahead=0.0), "lookahead"),
        (lambda path: Stanley(path, gain=-1.0), "gain"),
        (lambda path: Stanley(path, softening=0.0), "softening"),
        (lambda path: LineOfSight(path, lookahead=0.0), "lookahead"),
        (lambda path: LineOfSight(path, lookahead=-1.0), "lookahead"),
        (lambda path: HeadingController(gain=0.0), "gain"),
        (lambda path: HeadingController(wheelbase=0.0), "wheelbase"),
        (lambda path: HeadingController(softening=-0.5), "softening"),
    ],
)
def test_the_laws_refuse_numbers_they_divide_by_or_read_the_sign_of(build, name):
    """A zero or negative length, speed or gain is refused at construction, by name."""
    with pytest.raises(ValueError, match=name):
        build(straight_path())


# The drawing


@pytest.mark.parametrize("rear_offset", [0.0, 0.17])
def test_the_pursuit_arc_runs_from_the_rear_axle_to_the_target(rear_offset):
    """Off the end stops, the drawn arc starts at the rear axle and ends on the target."""
    path = circle_path(3.0)
    law = PurePursuit(path, wheelbase=WHEELBASE, rear_offset=rear_offset)
    state = state_at(3.0 + 0.2, 0.0, 0.5 * np.pi + 0.1, 2.0)

    arc, aim = law.get_dynamic_geometry(np.zeros(0), state)["world"]

    rear_axle = state[0:2] - rear_offset * np.array(
        [np.cos(state[2]), np.sin(state[2])]
    )
    s, _ = path.project(rear_axle)
    target = path.sample(s + LOOKAHEAD + 0.25 * 2.0)
    assert abs(law.ctl(np.zeros(0), state)[0]) < law.params["delta_max"]
    assert arc.pts[0, :2] == pytest.approx(rear_axle, abs=1e-12)
    assert arc.pts[-1, :2] == pytest.approx(target, abs=1e-9)
    assert aim.pt[:2] == pytest.approx(target)


def test_the_stanley_segment_ends_on_the_projection():
    """The drawn cross-track segment joins the front axle to its projection."""
    law = Stanley(straight_path(), front_offset=0.17)

    segment, foot = law.get_dynamic_geometry(np.zeros(0), state_at(2.0, 0.3, 0.0, 3.0))[
        "world"
    ]

    assert segment.pts[0, :2] == pytest.approx([2.17, 0.3])
    assert segment.pts[1, :2] == pytest.approx([2.17, 0.0])
    assert foot.pt[:2] == pytest.approx([2.17, 0.0])


def test_the_line_of_sight_ends_down_the_path():
    """The drawn line of sight runs from the control point to Δ down the tangent."""
    law = LineOfSight(straight_path(), lookahead=1.5)

    sight, point = law.get_dynamic_geometry(np.zeros(0), state_at(2.0, 0.4, 0.0, 3.0))[
        "world"
    ]

    assert sight.pts[0, :2] == pytest.approx([2.0, 0.4])
    assert sight.pts[1, :2] == pytest.approx([3.5, 0.0])
    assert point.pt[:2] == pytest.approx([3.5, 0.0])


# Backends


LAWS = {
    "pursuit": lambda path: PurePursuit(path, wheelbase=WHEELBASE, rear_offset=0.17),
    "stanley": lambda path: Stanley(path, front_offset=0.17),
    "line of sight": lambda path: LineOfSight(path, lookahead=1.0),
}


@pytest.mark.jax
@pytest.mark.parametrize("law", sorted(LAWS))
@pytest.mark.parametrize("where", ["corner", "seam", "off the path"])
def test_the_laws_trace_under_jax(law, where):
    """Each law runs inside a compiled loop with the same number out, the path's
    numbers traced too, and a finite gradient with respect to the waypoints."""
    jax = pytest.importorskip("jax")
    jnp = jax.numpy
    waypoints = circuit_waypoints(length=8.0, width=6.0, radius=1.5)
    block = LAWS[law](from_waypoints(waypoints, closed=True))
    poses = {
        "corner": state_at(waypoints[20, 0], waypoints[20, 1], 1.2, 3.0),
        "seam": state_at(waypoints[-1, 0], waypoints[-1, 1], 1.5708, 3.0),
        "off the path": state_at(waypoints[0, 0], waypoints[0, 1] + 0.2, 0.3, 3.0),
    }
    y = poses[where]

    def steer(y, params):
        return block.ctl(jnp.zeros(0), y, params=params)[0]

    on_numpy = block.ctl(np.zeros(0), y)
    on_jax = jax.jit(steer)(jnp.asarray(y), block.params)
    gradient = jax.grad(steer, argnums=1)(jnp.asarray(y), block.params)

    assert np.isfinite(on_numpy[0])
    assert float(on_jax) == pytest.approx(float(on_numpy[0]), rel=1e-6, abs=1e-6)
    assert np.all(np.isfinite(gradient["path"]["waypoints"]))


@pytest.mark.jax
def test_the_heading_controller_traces_under_jax():
    """The heading loop runs inside a compiled loop with the same number out."""
    jax = pytest.importorskip("jax")
    jnp = jax.numpy
    law = HeadingController()
    u = np.r_[2.5, state_at(0.0, 0.0, -2.8, 2.0)]

    on_numpy = law.ctl(np.zeros(0), u)
    on_jax = np.asarray(jax.jit(law.ctl)(jnp.zeros(0), jnp.asarray(u)))

    assert on_jax == pytest.approx(on_numpy, rel=1e-6)
