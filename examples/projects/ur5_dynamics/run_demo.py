"""Compare UR5 equation-of-motion techniques: RNEA–H, ABA, symbolic Lagrange.

Run from repo root::

    PYTHONPATH=. python examples/projects/ur5_dynamics/run_demo.py

Optional flags (environment):

* ``UR5_SKIP_SYMBOLIC=1`` — skip the slow SymPy derive/export step.
* ``UR5_SYMBOLIC_VERBOSE=1`` — print symbolic build timings.

The catalog spatial RNEA path and ABA always run. Symbolic comparison requires
``pip install minilink[symbolic]`` and adds several minutes on the first run.
"""

import os

import numpy as np

from examples.projects.ur5_dynamics.compare_eom import (
    run_parity_table,
    sample_configurations,
)
from examples.projects.ur5_dynamics.symbolic_ur5 import (
    build_symbolic_ur5,
    catalog_params_no_damping,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

N_SAMPLES = 12
SEED = 0

arm = UR5Manipulator()
params = catalog_params_no_damping()
rng = np.random.default_rng(SEED)
configs = sample_configurations(rng, N_SAMPLES)

print("UR5 forward dynamics — EoM technique comparison")
print("Reference: RNEA bias + explicit H solve (catalog forward_dynamics)")
print("Damping disabled so symbolic Lagrange plant matches spatial parameters.")

symbolic_plant = None
if os.environ.get("UR5_SKIP_SYMBOLIC", "").strip().lower() in {"1", "true", "yes"}:
    print("\nSkipping symbolic path (UR5_SKIP_SYMBOLIC set).")
else:
    try:
        verbose = os.environ.get("UR5_SYMBOLIC_VERBOSE", "").strip().lower() in {
            "1",
            "true",
            "yes",
        }
        symbolic_plant = build_symbolic_ur5(verbose=verbose)
    except ImportError:
        print(
            "\nSymPy not installed — skipping symbolic Lagrange path."
            " Install with: pip install minilink[symbolic]"
        )

run_parity_table(arm, symbolic_plant, configs, params=params)

q, v, u = configs[0]
qdd_rnea = arm.forward_dynamics(q, v, u, params=params)
qdd_aba = arm.forward_dynamics_aba(q, v, u, params=params)
print("\nSample qdd (first random config):")
print("  RNEA-H:", np.array2string(qdd_rnea, precision=4, suppress_small=True))
print("  ABA:   ", np.array2string(qdd_aba, precision=4, suppress_small=True))
if symbolic_plant is not None:
    qdd_sym = symbolic_plant.forward_dynamics(q, v, u)
    print("  Symbolic:", np.array2string(qdd_sym, precision=4, suppress_small=True))
