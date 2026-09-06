"""Inertia dyadic helpers for symbolic multibody models."""

import sympy as sp
from sympy.physics.mechanics import inertia as _make_inertia


def inertia_tensor(frame, Ixx=0, Iyy=0, Izz=0, Ixy=0, Iyz=0, Izx=0):
    """Create an inertia dyadic expressed in the given reference frame."""
    return _make_inertia(frame, Ixx, Iyy, Izz, Ixy, Iyz, Izx)


def inertia_rod(mass, length, frame, axis="z"):
    """Uniform thin rod about its center, rotation perpendicular to axis."""
    I_perp = mass * length**2 / 12
    if axis == "x":
        return inertia_tensor(frame, 0, I_perp, I_perp)
    if axis == "y":
        return inertia_tensor(frame, I_perp, 0, I_perp)
    return inertia_tensor(frame, I_perp, I_perp, 0)


def inertia_cylinder(mass, radius, length, frame, axis="z"):
    """Solid cylinder about its center of mass."""
    I_axial = mass * radius**2 / 2
    I_perp = mass * (3 * radius**2 + length**2) / 12
    if axis == "x":
        return inertia_tensor(frame, I_axial, I_perp, I_perp)
    if axis == "y":
        return inertia_tensor(frame, I_perp, I_axial, I_perp)
    return inertia_tensor(frame, I_perp, I_perp, I_axial)


def inertia_sphere(mass, radius, frame):
    """Solid sphere about its center."""
    inertia_value = sp.Rational(2, 5) * mass * radius**2
    return inertia_tensor(frame, inertia_value, inertia_value, inertia_value)


def inertia_box(mass, lx, ly, lz, frame):
    """Solid rectangular box about its center."""
    Ixx = mass * (ly**2 + lz**2) / 12
    Iyy = mass * (lx**2 + lz**2) / 12
    Izz = mass * (lx**2 + ly**2) / 12
    return inertia_tensor(frame, Ixx, Iyy, Izz)


def smart_simplify(expr):
    """Trigonometric simplification tuned for dynamics expressions."""
    if isinstance(expr, sp.Matrix):
        return expr.applyfunc(lambda e: sp.trigsimp(sp.simplify(e)))
    return sp.trigsimp(sp.simplify(expr))
