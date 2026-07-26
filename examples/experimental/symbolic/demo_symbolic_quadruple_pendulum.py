"""
Quadruple planar pendulum — symbolic derivation (Lagrange / Kane) and minilink export.

Mirrors the pymotion quadruple_pendulum example: DH chain, derive EoM, export to a
NumPy :class:`~minilink.dynamics.abstraction.mechanical.MechanicalSystem`, simulate with
:meth:`~minilink.core.system.System.compute_trajectory`.

Symbolic derivation of 4×4 H, C, g can take tens of seconds on first run.
"""

from minilink.symbolic.mechanics.model import MechanicalModel

# Build the symbolic model
m = MechanicalModel("QuadruplePendulum")
N_LINKS = 4

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
    {"theta": coords[i], "d": 0, "a": lengths[i], "alpha": 0} for i in range(N_LINKS)
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

# Derive the symbolic equations of motion
print("Deriving the symbolic equations of motion...")

sym_sys = m.derive(method="lagrange", simplify=True)


# Basic minilink ecosystem class
# #########################################################

params = {}
for i in range(N_LINKS):
    params[lengths[i]] = 1.5
    params[com_offsets[i]] = 1.5
    params[masses[i]] = 10.0
    params[inertias[i]] = 0.1
params[g_sym] = 9.81

print("\nSymbolic mass matrix:")
print("H[0,0]:", sym_sys.H[0, 0])
print("H[0,1]:", sym_sys.H[0, 1])
print("H[1,0]:", sym_sys.H[1, 0])
print("H[1,1]:", sym_sys.H[1, 1])

print("\nSymbolic Coriolis matrix:")
print("C[0,0]:", sym_sys.C[0, 0])
print("C[0,1]:", sym_sys.C[0, 1])
print("C[1,0]:", sym_sys.C[1, 0])
print("C[1,1]:", sym_sys.C[1, 1])

print("\nSymbolic gravitational vector:")
print("g[0]:", sym_sys.g[0])
print("g[1]:", sym_sys.g[1])

print("\nSymbolic actuator matrix:")
print("B[0,0]:", sym_sys.B[0, 0])
print("B[0,1]:", sym_sys.B[0, 1])
print("B[1,0]:", sym_sys.B[1, 0])
print("B[1,1]:", sym_sys.B[1, 1])

print("Converting the symbolic equations of motion to a minilink ecosystem...")
sys = sym_sys.to_minilink(parameters=params, backend="jax")


sys.compute_trajectory(tf=10.0)

sys.animate()
