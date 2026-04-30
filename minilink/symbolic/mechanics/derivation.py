"""Lagrangian and Kane derivation: MechanicalModel → symbolic MechanicalSystem."""

import sympy as sp
from sympy.physics.mechanics import KanesMethod

from .symbolic_system import MechanicalSystem


def _extract_H(T, coordinates, simplify=True):
    """H_ij = d2T / (dq_dot_i dq_dot_j)."""
    t = sp.Symbol("t")
    qdots = [qi.diff(t) for qi in coordinates]
    dof = len(coordinates)
    H = sp.zeros(dof)
    for i in range(dof):
        for j in range(i, dof):
            hij = sp.diff(sp.diff(T, qdots[i]), qdots[j])
            if simplify:
                hij = sp.trigsimp(hij)
            H[i, j] = hij
            H[j, i] = hij
    return H


def _extract_C(H, coordinates, simplify=True):
    """Coriolis matrix via Christoffel symbols of H(q)."""
    t = sp.Symbol("t")
    qdots = [qi.diff(t) for qi in coordinates]
    dof = len(coordinates)
    C = sp.zeros(dof)
    for i in range(dof):
        for j in range(dof):
            cij = sp.S(0)
            for k in range(dof):
                cijk = sp.Rational(1, 2) * (
                    sp.diff(H[i, j], coordinates[k])
                    + sp.diff(H[i, k], coordinates[j])
                    - sp.diff(H[j, k], coordinates[i])
                )
                cij += cijk * qdots[k]
            if simplify:
                cij = sp.trigsimp(cij)
            C[i, j] = cij
    return C


def _extract_g(V, coordinates, simplify=True):
    """g_i = dV / dq_i."""
    dof = len(coordinates)
    g = sp.zeros(dof, 1)
    for i in range(dof):
        gi = sp.diff(V, coordinates[i])
        if simplify:
            gi = sp.trigsimp(gi)
        g[i] = gi
    return g


def _attach_fk(result, mechanical_model, simplify=True):
    if mechanical_model._effector_point is not None:
        try:
            fk_full = mechanical_model.forward_kinematics(simplify=simplify)
            q = mechanical_model._coordinates
            active = [
                i
                for i in range(fk_full.rows)
                if fk_full[i] != 0 or any(fk_full[i].diff(qi) != 0 for qi in q)
            ]
            fk = sp.Matrix([fk_full[r] for r in active]) if active else fk_full
            result.fk = fk
            jac = fk.jacobian(q)
            if simplify:
                jac = jac.applyfunc(sp.trigsimp)
            result.J = jac
        except Exception:
            pass

    chain = mechanical_model._compute_chain_fk(simplify=simplify)
    if chain is not None:
        result.chain_fk = chain


def derive_lagrange(mechanical_model, simplify=True):
    """Energy-based Lagrangian derivation."""
    result = MechanicalSystem(mechanical_model)
    result.method = "lagrange"

    T = mechanical_model._compute_kinetic_energy()
    V = mechanical_model._compute_potential_energy()
    result.kinetic_energy = T
    result.potential_energy = V

    q = mechanical_model.q

    result.H = _extract_H(T, q, simplify=simplify)
    result.C = _extract_C(result.H, q, simplify=simplify)
    result.g = _extract_g(V, q, simplify=simplify)
    result.d = mechanical_model._compute_dissipative_forces(simplify=simplify)
    result.B = (
        mechanical_model._B_matrix
        if mechanical_model._B_matrix is not None
        else sp.eye(mechanical_model.dof)
    )

    _attach_fk(result, mechanical_model, simplify=simplify)
    return result


def derive_kane(mechanical_model, simplify=True):
    """Kane's method; H/C/g still from energies for manipulator form."""
    result = MechanicalSystem(mechanical_model)
    result.method = "kane"

    forces, torques = mechanical_model._collect_loads()
    loads = forces + torques

    kane = KanesMethod(
        mechanical_model.N,
        q_ind=mechanical_model.q,
        u_ind=mechanical_model.u,
        kd_eqs=mechanical_model._kdes,
    )
    kane.kanes_equations(mechanical_model._bodies, loads)

    T = mechanical_model._compute_kinetic_energy()
    V = mechanical_model._compute_potential_energy()
    result.kinetic_energy = T
    result.potential_energy = V

    q = mechanical_model.q

    result.H = _extract_H(T, q, simplify=simplify)
    result.C = _extract_C(result.H, q, simplify=simplify)
    result.g = _extract_g(V, q, simplify=simplify)
    result.d = mechanical_model._compute_dissipative_forces(simplify=simplify)
    result.B = (
        mechanical_model._B_matrix
        if mechanical_model._B_matrix is not None
        else sp.eye(mechanical_model.dof)
    )

    _attach_fk(result, mechanical_model, simplify=simplify)
    return result
