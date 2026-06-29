"""
Lambdify a symbolic MechanicalSystem into a numeric plant.

* ``backend="numpy"`` → :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem`
* ``backend="jax"`` → :class:`~minilink.dynamics.abstraction.mechanical.JaxMechanicalSystem`

Chain kinematics for drawing are always lambdified with NumPy for Matplotlib.
"""

from __future__ import annotations

import numpy as np
import sympy as sp

from minilink.graphical.animation.primitives import CustomLine


def _pose2d(px: float, py: float, theta: float, scale_x: float = 1.0) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    t = np.eye(4, dtype=float)
    t[0, 0], t[0, 1] = scale_x * c, -scale_x * s
    t[1, 0], t[1, 1] = scale_x * s, scale_x * c
    t[0, 3], t[1, 3] = px, py
    return t


def _lambdify_expr(expr, args, subs, backend: str):
    if isinstance(expr, (list, tuple)):
        e = [x.subs(subs) if subs else x for x in expr]
    else:
        e = expr.subs(subs) if subs else expr
    if backend == "jax":
        import jax.numpy as jnp

        return sp.lambdify(args, e, modules=[jnp])
    return sp.lambdify(args, e, modules="numpy")


def _lambdify_matrix_to_flat_func(matrix_expr, args, subs, backend: str):
    """
    Lambdify a SymPy Matrix by flattening to a list of scalar entries.

    Avoids lambdify emitting ``ImmutableDenseMatrix(...)`` in generated code
    (NameError at runtime). Returns ``(callable, nrows, ncols)``.
    """
    m = matrix_expr.subs(subs) if subs else matrix_expr
    M = sp.Matrix(m)
    rows, cols = M.rows, M.cols
    flat = []
    for i in range(rows):
        for j in range(cols):
            flat.append(M[i, j])
    f = _lambdify_expr(flat, args, None, backend)

    def eval_matrix(*call_args):
        out = f(*call_args)
        if backend == "jax":
            import jax.numpy as jnp

            flat_arr = jnp.asarray(out, dtype=jnp.float64).reshape(-1)
        else:
            flat_arr = np.asarray(out, dtype=float).reshape(-1)
        return flat_arr, rows, cols

    return eval_matrix, rows, cols


def create_minilink_system(sym_sys, parameters=None, *, backend: str = "numpy"):
    """
    Build a numeric mechanical plant with lambdified ``H``, ``C``, ``g``, ``d``, ``B``
    and optional chain kinematics.

    Parameters
    ----------
    sym_sys
        Symbolic system from ``minilink.symbolic.mechanics``.
    parameters : dict, optional
        ``{symbol: value}`` substitution before lambdify.
    backend : {"numpy", "jax"}
        ``"numpy"`` → :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem``.
        ``"jax"`` → :class:`~minilink.dynamics.abstraction.mechanical.JaxMechanicalSystem``
        (requires ``jax`` / ``jaxlib``). Chain FK for :meth:`get_kinematic_transforms`
        is always NumPy-based.
    """
    if backend not in ("numpy", "jax"):
        raise ValueError("backend must be 'numpy' or 'jax'")
    if backend == "jax":
        try:
            import jax.numpy as jnp  # noqa: F401
        except ImportError as e:
            raise ImportError(
                "backend='jax' requires JAX. Install with: pip install jax jaxlib"
            ) from e
        from minilink.dynamics.abstraction.mechanical import (
            JaxMechanicalSystem as NumericMechanicalSystem,
        )
    else:
        from minilink.dynamics.abstraction.mechanical import (
            MechanicalSystem as NumericMechanicalSystem,
        )

    subs = list((parameters or {}).items())

    t = sp.Symbol("t")
    q = sym_sys.coordinates
    dq_syms = [qi.diff(t) for qi in q]
    q_args = list(q)
    qdq_args = list(q) + list(dq_syms)

    dof = sym_sys.dof
    m_act = sym_sys.m

    H_eval, _, _ = _lambdify_matrix_to_flat_func(sym_sys.H, q_args, subs, backend)
    C_eval, _, _ = _lambdify_matrix_to_flat_func(sym_sys.C, qdq_args, subs, backend)
    g_eval, _, _ = _lambdify_matrix_to_flat_func(sym_sys.g, q_args, subs, backend)
    if sym_sys.d is not None:
        d_eval, _, _ = _lambdify_matrix_to_flat_func(sym_sys.d, qdq_args, subs, backend)
    else:
        d_eval = None
    if sym_sys.B is not None:
        B_eval, _, _ = _lambdify_matrix_to_flat_func(sym_sys.B, q_args, subs, backend)
    else:
        B_eval = None

    chain_fk_funcs = None
    n_seg = 0
    if sym_sys.chain_fk is not None:
        chain_fk_funcs = []
        for fk_pt in sym_sys.chain_fk:
            m = fk_pt.subs(subs) if subs else fk_pt
            M = sp.Matrix(m)
            flat = []
            for i in range(M.rows):
                for j in range(M.cols):
                    flat.append(M[i, j])
            chain_fk_funcs.append(_lambdify_expr(flat, q_args, None, "numpy"))
        n_seg = max(0, len(chain_fk_funcs) - 1)

    if backend == "jax":
        import jax.numpy as jnp

        xp = jnp
    else:
        xp = np

    def _from_flat(flat_arr, rows, cols):
        a = xp.asarray(flat_arr, dtype=float).reshape(rows, cols)
        return a

    def _H(qv):
        flat_arr, rows, cols = H_eval(*qv)
        return _from_flat(flat_arr, rows, cols)

    def _C(qv, dq_):
        flat_arr, rows, cols = C_eval(*qv, *dq_)
        return _from_flat(flat_arr, rows, cols)

    def _g(qv):
        flat_arr, rows, cols = g_eval(*qv)
        return xp.reshape(_from_flat(flat_arr, rows, cols), (-1,))

    def _d(qv, dq_):
        if d_eval is None:
            return xp.zeros(dof)
        flat_arr, rows, cols = d_eval(*qv, *dq_)
        return xp.reshape(_from_flat(flat_arr, rows, cols), (-1,))

    def _B(qv):
        if B_eval is None:
            return xp.eye(dof)
        flat_arr, rows, cols = B_eval(*qv)
        return _from_flat(flat_arr, rows, cols)

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

        def H(self, qv, params=None):
            return _H(qv)

        def C(self, qv, dq_, params=None):
            return _C(qv, dq_)

        def g(self, qv, params=None):
            return _g(qv)

        def d(self, qv, dq_, u=None, t=0.0, params=None):
            return _d(qv, dq_)

        def B(self, qv, params=None):
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
            qv = np.asarray(x[: self.dof], dtype=float)
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

        # === v2 frame-keyed visualization contract =======================
        #
        # Each chain segment is a unit ``CustomLine`` posed by its per-frame
        # stretch transform (``_pose2d`` with ``scale_x = L``) — pixel-identical
        # to the legacy path, with the stretch carried in the frame.

        def get_kinematic_geometry_v2(self):
            if self._chain_fk_funcs is None or self._n_seg == 0:
                return super().get_kinematic_geometry_v2()
            return {
                f"seg{i}": [
                    CustomLine(
                        np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]),
                        color="blue",
                        linewidth=2,
                    )
                ]
                for i in range(self._n_seg)
            }

        def tf_v2(self, x, u, t=0, params=None):
            if self._chain_fk_funcs is None or self._n_seg == 0:
                return super().tf_v2(x, u, t)
            qv = np.asarray(x[: self.dof], dtype=float)
            pts = _chain_pts(qv)
            frames = {"world": np.eye(4, dtype=float)}
            for i in range(self._n_seg):
                p0, p1 = pts[i], pts[i + 1]
                d = p1 - p0
                L = float(np.hypot(d[0], d[1]))
                th = float(np.arctan2(d[1], d[0]))
                frames[f"seg{i}"] = (
                    np.eye(4, dtype=float)
                    if L < 1e-12
                    else _pose2d(float(p0[0]), float(p0[1]), th, L)
                )
            return frames

    return _Generated()


def to_minilink(sym_sys, parameters=None, *, backend: str = "numpy"):
    """Alias for :func:`create_minilink_system`."""
    return create_minilink_system(sym_sys, parameters, backend=backend)
