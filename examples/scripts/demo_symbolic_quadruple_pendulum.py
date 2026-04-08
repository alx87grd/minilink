"""
Quadruple planar pendulum — symbolic derivation (Lagrange / Kane) and minilink export.

Mirrors the pymotion quadruple_pendulum example: DH chain, derive EoM, export to a
NumPy :class:`~minilink.mechanics.mechanical.MechanicalSystem`, simulate with
:meth:`~minilink.core.framework.System.compute_trajectory`.

Symbolic derivation of 4×4 H, C, g can take tens of seconds on first run.
"""

from __future__ import annotations

import time

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

###########################################################
# Symbolic sys class mirror
# #########################################################

print("\n--- [1/3] EoM derivation (symbolic) ---")
t0 = time.perf_counter()
sym_sys = m.derive(method="lagrange", simplify=True)
t_derive = time.perf_counter() - t0
print(
    f"  elapsed: {t_derive:.3f} s\n"
    f"  state:   name={sym_sys.name!r} dof={sym_sys.dof} method={sym_sys.method!r} "
    f"H{sym_sys.H.shape} C{sym_sys.C.shape} g{sym_sys.g.shape} "
    f"d{sym_sys.d.shape} B{sym_sys.B.shape}"
)

params = {}
for i in range(N_LINKS):
    params[lengths[i]] = 1.5
    params[com_offsets[i]] = 1.5
    params[masses[i]] = 10.0
    params[inertias[i]] = 0.1
params[g_sym] = 9.81

print("Symbolic system:")
print("H:", sym_sys.H)
print("C:", sym_sys.C)
print("g:", sym_sys.g)
print("d:", sym_sys.d)
print("B:", sym_sys.B)


###########################################################
# Basic minilink ecosystem class
# #########################################################

print(f"\n--- [2/3] Export to numeric ) ---")
t0 = time.perf_counter()

sys = sym_sys.to_minilink(parameters=params, backend="jax")

t_export = time.perf_counter() - t0
print(f"  elapsed: {t_export:.3f} s\n")


# sys.compute_trajectory(tf=10.0)

# sys.animate()


from minilink.blocks.sources import Step
from minilink.core.diagram import DiagramSystem

# Source input
step = Step(
    initial_value=np.array([0.0, 0.0, 0.0, 0.0]),
    final_value=np.array([1000.0, 0.0, 0.0, 0.0]),
    step_time=3.0,
)

diagram = DiagramSystem()
diagram.add_subsystem(step, "step")
diagram.add_subsystem(sys, "plant")
diagram.connect("step", "y", "plant", "u")
diagram.compute_trajectory(tf=20.0)
diagram.plot_graphe()
diagram.animate()


print(f"\n--- [3/3] Compile diagram ---")
t0 = time.perf_counter()

evaluator = diagram.compile(backend="jax")

t_compile = time.perf_counter() - t0
print(f"  elapsed: {t_compile:.3f} s\n")


x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
u = np.array([0.0, 0.0, 0.0, 0.0])

# print("diagram.f(x,u,0):", diagram.f(x, u, 0))
# print("evaluator.f(x,u,0):", evaluator.f(x, u, 0))

n_iter = 2000
t0 = time.perf_counter()
for _ in range(n_iter):
    diagram.f(x, u, 0)
t_diagram = time.perf_counter() - t0

t0 = time.perf_counter()
for _ in range(n_iter):
    evaluator.f(x, u, 0)
t_eval = time.perf_counter() - t0

print(
    f"\ndiagram.f {1e6 * t_diagram / n_iter:.2f} µs/call, "
    f"\nevaluator.f {1e6 * t_eval / n_iter:.2f} µs/call "
)
