"""
Quadruple planar pendulum — symbolic derivation (Lagrange / Kane) and minilink export.

Mirrors the pymotion quadruple_pendulum example: DH chain, derive EoM, export to a
NumPy :class:`~minilink.mechanics.mechanical.MechanicalSystem`, simulate with
:meth:`~minilink.core.framework.System.compute_trajectory`.

Symbolic derivation of 4×4 H, C, g can take tens of seconds on first run.
"""

from __future__ import annotations

import numpy as np

from minilink.mechanics.symbolic import MechanicalModel

N_LINKS = 4


def build_model():
    m = MechanicalModel("QuadruplePendulum")

    lengths, com_offsets, masses, inertias = [], [], [], []
    for i in range(1, N_LINKS + 1):
        li, lci = m.parameters(f"l{i} lc{i}")
        mi = m.parameters(f"m{i}")
        Ii = m.parameters(f"I{i}")
        lengths.append(li)
        com_offsets.append(lci)
        masses.append(mi)
        inertias.append(Ii)

    g_sym = m.parameters("g")

    coord_names = " ".join(f"q{i}" for i in range(1, N_LINKS + 1))
    coords = m.coordinates(coord_names)

    dh_table = [
        {"theta": coords[i], "d": 0, "a": lengths[i], "alpha": 0}
        for i in range(N_LINKS)
    ]
    link_properties = [
        {
            "mass": masses[i],
            "inertia": {"Izz": inertias[i]},
            "com_offset": com_offsets[i],
        }
        for i in range(N_LINKS)
    ]

    m.add_dh_chain(dh_table, link_properties)
    m.add_gravity(-g_sym * m.N.y)
    return m, lengths, com_offsets, masses, inertias, g_sym


m, lengths, com_offsets, masses, inertias, g_sym = build_model()


sym_sys = m.derive(method="lagrange", simplify=True)

params = {}
for i in range(N_LINKS):
    params[lengths[i]] = 1.5
    params[com_offsets[i]] = 1.5
    params[masses[i]] = 10.0
    params[inertias[i]] = 0.1
params[g_sym] = 9.81


sys = sym_sys.to_minilink(parameters=params)


sys.x0 = np.zeros(2 * N_LINKS)
sys.x0[0] = np.pi / 3
sys.x0[1] = -np.pi / 6
sys.x0[2] = np.pi / 4
sys.x0[3] = -np.pi / 8

sys.compute_trajectory(tf=10.0)

sys.animate()
