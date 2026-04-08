"""
Symbolic mechanical system in manipulator form (SymPy).

    H(q) q̈ + C(q, q̇) q̇ + d(q, q̇) + g(q) = B(q) u

This class lives in ``minilink.mechanics.symbolic`` only. For a numeric block, call
:meth:`to_minilink` (NumPy :class:`~minilink.mechanics.mechanical.MechanicalSystem`
or, with ``backend="jax"``, :class:`~minilink.mechanics.mechanical.JaxMechanicalSystem`).
"""

import sympy as sp


class MechanicalSystem:
    """
    Symbolic EoM holder produced by :meth:`MechanicalModel.derive`.

    Attributes
    ----------
    H, C, g, d, B : Matrix
        Manipulator equation matrices (SymPy).
    fk, J, chain_fk : optional
        Forward kinematics and Jacobian (SymPy), when defined on the model.
    """

    def __init__(self, mechanical_model):
        self.name = mechanical_model.name
        self.coordinates = mechanical_model.q
        self.parameters = mechanical_model._params
        self.dof = mechanical_model.dof
        self._mechanical_model = mechanical_model

        self.H = None
        self.C = None
        self.g = None
        self.d = None
        self.B = None

        self.fk = None
        self.J = None
        self.chain_fk = None

        self.kinetic_energy = None
        self.potential_energy = None

        self.method = None

        self._f = None
        self._h = None
        self._input_symbols = None

    @property
    def m(self):
        """Number of actuator inputs."""
        return self.B.cols if self.B is not None else self.dof

    @property
    def e(self):
        """Dimension of end-effector position vector."""
        return self.fk.shape[0] if self.fk is not None else 0

    @property
    def T(self):
        return self.kinetic_energy

    @property
    def V(self):
        return self.potential_energy

    @property
    def u_sym(self):
        if self._input_symbols is None:
            self._input_symbols = sp.Matrix(sp.symbols(f"u0:{self.m}"))
        return self._input_symbols

    @property
    def f(self):
        """Symbolic dx/dt = f(x, u); state x = [q; dq]."""
        if self._f is None:
            self._compute_state_space()
        return self._f

    @property
    def h(self):
        """Symbolic output y = x."""
        if self._h is None:
            self._compute_state_space()
        return self._h

    def _compute_state_space(self):
        t = sp.Symbol("t")
        q = self.coordinates
        dq = [qi.diff(t) for qi in q]
        dq_vec = sp.Matrix(dq)
        u_vec = self.u_sym

        rhs = self.B * u_vec - self.C * dq_vec - self.d - self.g
        ddq = self.H.LUsolve(rhs)

        self._f = dq_vec.col_join(ddq)
        self._h = sp.Matrix(list(q) + list(dq))

    def to_minilink(self, parameters=None, *, backend: str = "numpy"):
        """Export to a numeric mechanical plant (see :func:`.export.create_minilink_system`).

        Parameters
        ----------
        parameters : dict, optional
            Symbol substitutions for numeric values.
        backend : {"numpy", "jax"}
            ``"numpy"`` → :class:`~minilink.mechanics.mechanical.MechanicalSystem``.
            ``"jax"`` → :class:`~minilink.mechanics.mechanical.JaxMechanicalSystem``.
        """
        from .export import create_minilink_system

        return create_minilink_system(self, parameters, backend=backend)

    def __repr__(self):
        h = self.H.shape if self.H is not None else None
        c = self.C.shape if self.C is not None else None
        gsh = self.g.shape if self.g is not None else None
        return (
            f"MechanicalSystem('{self.name}', dof={self.dof}, "
            f"method='{self.method}')\n"
            f"  H: {h}  C: {c}  g: {gsh}  m: {self.m}"
        )
