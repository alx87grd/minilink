"""Build a Lagrange-derived UR5 plant from :mod:`minilink.symbolic.mechanics`.

First call to :func:`build_symbolic_ur5` takes on the order of minutes (6×6
derivation + lambdify). Subsequent calls return a cached numeric plant.
Requires ``pip install minilink[symbolic]`` (SymPy).
"""

from __future__ import annotations

import time

import numpy as np

from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

_PLANTS: dict[str, object] = {}


def build_symbolic_ur5(*, backend: str = "numpy", verbose: bool = False):
    """
    Derive UR5 equations of motion symbolically and export a numeric plant.

    Parameters
    ----------
    backend : {"numpy", "jax"}
        Passed to :meth:`~minilink.symbolic.mechanics.symbolic_system.MechanicalSystem.to_minilink`.
    verbose : bool
        Print build timings when ``True``.

    Returns
    -------
    MechanicalSystem
        Numeric plant with ``H`` / ``C`` / ``g`` from Lagrange derivation.
    """
    global _PLANTS
    if backend in _PLANTS:
        return _PLANTS[backend]

    from minilink.symbolic.mechanics.model import MechanicalModel

    catalog = UR5Manipulator()
    params = catalog.params

    model = MechanicalModel("UR5Symbolic")
    coords = model.coordinates("q1 q2 q3 q4 q5 q6")
    g_sym = model.parameters("g")

    dh_table = []
    link_properties = []
    for i in range(catalog.dof):
        dh_table.append(
            {
                "theta": coords[i],
                "d": float(params["d"][i]),
                "a": float(params["a"][i]),
                "alpha": float(params["alpha"][i]),
            }
        )
        inertia = params["inertia"][i]
        link_properties.append(
            {
                "mass": float(params["mass"][i]),
                "inertia": {
                    "Ixx": float(inertia[0, 0]),
                    "Iyy": float(inertia[1, 1]),
                    "Izz": float(inertia[2, 2]),
                },
                "com_offset": {
                    "x": float(params["com"][i, 0]),
                    "y": float(params["com"][i, 1]),
                    "z": float(params["com"][i, 2]),
                },
            }
        )

    model.add_dh_chain(dh_table, link_properties)
    model.add_gravity(-g_sym * model.N.z)

    if verbose:
        print("Deriving symbolic UR5 EoM (Lagrange, simplify=False)...")
    t0 = time.perf_counter()
    sym_sys = model.derive(method="lagrange", simplify=False)
    if verbose:
        print(f"  derive: {time.perf_counter() - t0:.1f} s")

    param_map = {g_sym: float(params["gravity"])}
    if verbose:
        print("Exporting to minilink plant...")
    t0 = time.perf_counter()
    plant = sym_sys.to_minilink(parameters=param_map, backend=backend)
    if verbose:
        print(f"  export: {time.perf_counter() - t0:.1f} s")

    _PLANTS[backend] = plant
    return plant


def catalog_params_no_damping():
    """Catalog UR5 parameters with viscous damping disabled for EoM parity."""
    params = dict(UR5Manipulator().params)
    params["damping"] = np.zeros(6)
    return params
