"""
Shortest Dubins paths for a forward-only car with a minimum turning radius.

`shortest_path` returns the shortest of the six Dubins words (LSL, RSR, LSR, RSL,
RLR, LRL) between two planar poses as a list of ``(curvature_sign, arc_length)``
segments (``+1`` left, ``0`` straight, ``-1`` right). Used by
:class:`~minilink.planning.search.steering.DubinsSteering`.

The geometry is scalar, so it runs on the standard-library ``math`` module
rather than numpy — this is the nearest-neighbour metric of a steering RRT and
is evaluated against every node, so the per-call cost matters.
"""

import math

_2PI = 2.0 * math.pi
_SIGN = {"L": 1.0, "S": 0.0, "R": -1.0}


def _mod2pi(angle):
    return angle % _2PI


def _words(a, b, d):
    """Segment triples ``(t, p, q)`` (normalised) for each feasible word."""
    sa, sb, ca, cb = math.sin(a), math.sin(b), math.cos(a), math.cos(b)
    c_ab = math.cos(a - b)
    out = {}

    psq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if psq >= 0:
        tmp = math.atan2(cb - ca, d + sa - sb)
        out["LSL"] = (_mod2pi(-a + tmp), math.sqrt(psq), _mod2pi(b - tmp))

    psq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if psq >= 0:
        tmp = math.atan2(ca - cb, d - sa + sb)
        out["RSR"] = (_mod2pi(a - tmp), math.sqrt(psq), _mod2pi(-b + tmp))

    psq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out["LSR"] = (_mod2pi(-a + tmp), p, _mod2pi(-_mod2pi(b) + tmp))

    psq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if psq >= 0:
        p = math.sqrt(psq)
        tmp = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out["RSL"] = (_mod2pi(a - tmp), p, _mod2pi(b - tmp))

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(_2PI - math.acos(tmp))
        t = _mod2pi(a - math.atan2(ca - cb, d - sa + sb) + p / 2.0)
        out["RLR"] = (t, p, _mod2pi(a - b - t + p))

    tmp = (6.0 - d * d + 2 * c_ab + 2 * d * (-sa + sb)) / 8.0
    if abs(tmp) <= 1.0:
        p = _mod2pi(_2PI - math.acos(tmp))
        t = _mod2pi(-a - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
        out["LRL"] = (t, p, _mod2pi(_mod2pi(b) - a - t + p))

    return out


def shortest_path(q0, q1, radius):
    """
    Return ``(word, segments, length)`` for the shortest Dubins path.

    ``segments`` is a list of ``(curvature_sign, arc_length)`` with sign in
    ``{+1, 0, -1}``; ``length`` is the total arc length. ``None`` if degenerate.
    """
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / radius
    theta = _mod2pi(math.atan2(dy, dx))
    a = _mod2pi(q0[2] - theta)
    b = _mod2pi(q1[2] - theta)

    best_word, best_segs, best_len = None, None, math.inf
    for word, segs in _words(a, b, d).items():
        length = sum(segs)
        if length < best_len:
            best_word, best_segs, best_len = word, segs, length
    if best_word is None:
        return None

    segments = [
        (_SIGN[letter], radius * seg) for letter, seg in zip(best_word, best_segs)
    ]
    return best_word, segments, radius * best_len
