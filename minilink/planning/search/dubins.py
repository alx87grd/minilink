"""Dubins paths: the shortest path of a forward-only car with a minimum turning radius."""

import math

# The geometry is scalar, so it runs on ``math`` rather than NumPy: it is the
# nearest-neighbour metric of a steering RRT, evaluated against every node.
_2PI = 2.0 * math.pi
_SIGN = {"L": 1.0, "S": 0.0, "R": -1.0}


###############################################################################
# Public API
###############################################################################


def shortest_path(q0, q1, radius):
    """
    Return ``(word, segments, length)`` for the shortest of the six Dubins words.

    The words are LSL, RSR, LSR, RSL, RLR and LRL, between two planar poses
    ``(x, y, theta)``. ``segments`` lists ``(curvature_sign, arc_length)``, the
    sign in ``{+1, 0, -1}`` (left, straight, right); ``length`` is the total arc
    length. ``None`` if degenerate. Used by
    :class:`~minilink.planning.search.steering.DubinsSteering`.
    """
    # The normalised frame: the start-to-goal line as the x axis, a unit radius
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    d = math.hypot(dx, dy) / radius
    theta = mod2pi(math.atan2(dy, dx))
    a = mod2pi(q0[2] - theta)
    b = mod2pi(q1[2] - theta)

    # The shortest of the feasible words
    best_word = None
    best_segs = None
    best_len = math.inf
    for word, segs in word_segments(a, b, d).items():
        length = sum(segs)
        if length < best_len:
            best_word = word
            best_segs = segs
            best_len = length
    if best_word is None:
        return None

    # Back to the true radius: one (curvature sign, arc length) per letter
    segments = [
        (_SIGN[letter], radius * seg) for letter, seg in zip(best_word, best_segs)
    ]
    length = radius * best_len

    return best_word, segments, length


###############################################################################
# Internal machinery
###############################################################################


def word_segments(a, b, d):
    """Segment triples ``(t, p, q)``, normalised, of each feasible word."""
    sa = math.sin(a)
    sb = math.sin(b)
    ca = math.cos(a)
    cb = math.cos(b)
    c_ab = math.cos(a - b)
    out = {}

    # LSL: left, straight, left, when p² ≥ 0
    psq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if psq >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (mod2pi(-a + tmp), math.sqrt(psq), mod2pi(b - tmp))

    # RSR: right, straight, right, when p² ≥ 0
    psq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if psq >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (mod2pi(a - tmp), math.sqrt(psq), mod2pi(-b + tmp))

    # LSR: left, straight, right, when p² ≥ 0
    psq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (mod2pi(-a + tmp), p, mod2pi(-mod2pi(b) + tmp))

    # RSL: right, straight, left, when p² ≥ 0
    psq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (mod2pi(a - tmp), p, mod2pi(b - tmp))

    # RLR: right, left, right, when |cos p| ≤ 1 for the middle arc p
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = mod2pi(_2PI - math.acos(tmp))
        t = mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        out["RLR"] = (t, p, mod2pi(a - b - t + p))

    # LRL: left, right, left, when |cos p| ≤ 1 for the middle arc p
    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = mod2pi(_2PI - math.acos(tmp))
        t = mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out["LRL"] = (t, p, mod2pi(mod2pi(b) - a - t + p))

    return out


def mod2pi(angle):
    """``angle`` wrapped to ``[0, 2π)``."""
    return angle % _2PI
