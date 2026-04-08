"""
Lambdify a symbolic MechanicalSystem into a NumPy minilink MechanicalSystem.
"""

from __future__ import annotations

import copy

import numpy as np
import sympy as sp

from minilink.graphical.primitives import CustomLine
from minilink.mechanics.mechanical import MechanicalSystem as NumericMechanicalSystem


def _pose2d(px: float, py: float, theta: float, scale_x: float = 1.0) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    t = np.eye(4, dtype=float)
    t[0, 0], t[0, 1] = scale_x * c, -scale_x * s
    t[1, 0], t[1, 1] = scale_x * s, scale_x * c
    t[0, 3], t[1, 3] = px, py
    return t


def _lam(expr, args, subs):
    e = expr.subs(subs) if subs else expr
    return sp.lambdify(args, e, modules="numpy")


def create_minilink_system(sym_sys, parameters=None):
    """
    Build a NumPy :class:`~minilink.mechanics.mechanical.MechanicalSystem` with
    lambdified ``H``, ``C``, ``g``, ``d``, ``B`` and optional chain kinematics
    for :meth:`get_kinematic_geometry` / :meth:`get_kinematic_transforms`.
    """
    subs = list((parameters or {}).items())

    t = sp.Symbol("t")
    q = sym_sys.coordinates
    dq_syms = [qi.diff(t) for qi in q]
    q_args = list(q)
    qdq_args = list(q) + list(dq_syms)

    def _mk(expr, args):
        return _lam(expr, args, subs)

    dof = sym_sys.dof
    m_act = sym_sys.m

    H_func = _mk(sym_sys.H, q_args)
    C_func = _mk(sym_sys.C, qdq_args)
    g_func = _mk(sym_sys.g, q_args)
    d_func = _mk(sym_sys.d, qdq_args) if sym_sys.d is not None else None
    B_func = _mk(sym_sys.B, q_args) if sym_sys.B is not None else None
    B_rows = sym_sys.B.rows if sym_sys.B is not None else dof
    B_cols = sym_sys.B.cols if sym_sys.B is not None else dof

    chain_fk_funcs = None
    n_seg = 0
    if sym_sys.chain_fk is not None:
        chain_fk_funcs = [_mk(fk_pt, q_args) for fk_pt in sym_sys.chain_fk]
        n_seg = max(0, len(chain_fk_funcs) - 1)

    def _H(qv):
        return np.asarray(H_func(*qv), dtype=float).reshape(dof, dof)

    def _C(qv, dq_):
        return np.asarray(C_func(*qv, *dq_), dtype=float).reshape(dof, dof)

    def _g(qv):
        return np.asarray(g_func(*qv), dtype=float).reshape(-1)

    def _d(qv, dq_):
        if d_func is None:
            return np.zeros(dof)
        return np.asarray(d_func(*qv, *dq_), dtype=float).reshape(-1)

    def _B(qv):
        if B_func is None:
            return np.eye(dof)
        return np.asarray(B_func(*qv), dtype=float).reshape(B_rows, B_cols)

    def _chain_pts(qv):
        pts = np.zeros((len(chain_fk_funcs), 3))
        for i, f in enumerate(chain_fk_funcs):
            v = np.asarray(f(*qv), dtype=float).reshape(-1)
            pts[i, : min(3, v.size)] = v[:3]
        return pts

    class _Generated(NumericMechanicalSystem):
        def __init__(self):
            super().__init__(dof=dof, actuators=m_act)
            self.name = f"minilink:{sym_sys.name}"
            self._chain_fk_funcs = chain_fk_funcs
            self._n_seg = n_seg

        def H(self, qv):
            return _H(qv)

        def C(self, qv, dq_):
            return _C(qv, dq_)

        def g(self, qv):
            return _g(qv)

        def d(self, qv, dq_):
            return _d(qv, dq_)

        def B(self, qv):
            return _B(qv)

        def get_kinematic_geometry(self):
            if self._chain_fk_funcs is None or self._n_seg == 0:
                return super().get_kinematic_geometry()
            return [
                CustomLine(
                    np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                    color="blue",
                    linewidth=2,
                )
                for _ in range(self._n_seg)
            ]

        def get_kinematic_transforms(self, x, u, t):
            if self._chain_fk_funcs is None or self._n_seg == 0:
                return super().get_kinematic_transforms(x, u, t)
            qv = x[: self.dof]
            pts = _chain_pts(qv)
            transforms = []
            for i in range(self._n_seg):
                p0, p1 = pts[i], pts[i + 1]
                d = p1 - p0
                L = float(np.hypot(d[0], d[1]))
                th = float(np.arctan2(d[1], d[0]))
                if L < 1e-12:
                    transforms.append(np.eye(4, dtype=float))
                else:
                    transforms.append(_pose2d(float(p0[0]), float(p0[1]), th, L))
            return transforms

    return _Generated()


def to_minilink(sym_sys, parameters=None):
    """Alias for :func:`create_minilink_system`."""
    return create_minilink_system(sym_sys, parameters)
