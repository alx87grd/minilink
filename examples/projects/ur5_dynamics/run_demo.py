"""UR5 EoM comparison — quick text smoke (teaching notebook is the main entry).

Full textbook walkthrough with math, plots, and tables:

    examples/projects/ur5_dynamics/eom_comparison.ipynb

Run this script for a fast reproducible summary without opening the notebook::

    PYTHONPATH=. python examples/projects/ur5_dynamics/run_demo.py

Set ``UR5_SKIP_SYMBOLIC=1`` to skip the slow SymPy build.
"""

import os

from examples.projects.ur5_dynamics.compare_eom import (
    DEFAULT_N_SAMPLES,
    DEFAULT_N_TIMING,
    DEFAULT_SEED,
    build_evaluation_batch,
    comprehensive_comparison,
    print_comprehensive_report,
)
from examples.projects.ur5_dynamics.symbolic_ur5 import (
    build_symbolic_ur5,
    catalog_params_no_damping,
)
from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator

seed = int(os.environ.get("UR5_SEED", DEFAULT_SEED))
n_samples = int(os.environ.get("UR5_N_SAMPLES", DEFAULT_N_SAMPLES))
n_timing = int(os.environ.get("UR5_N_TIMING", DEFAULT_N_TIMING))

arm = UR5Manipulator()
params = catalog_params_no_damping()
configs, labels = build_evaluation_batch(seed=seed, n_random=n_samples)

print("Teaching notebook: examples/projects/ur5_dynamics/eom_comparison.ipynb")
print("Quick smoke below (same batch as the notebook, seed=%d)\n" % seed)

symbolic_plant = None
if os.environ.get("UR5_SKIP_SYMBOLIC", "").strip().lower() not in {"1", "true", "yes"}:
    try:
        symbolic_plant = build_symbolic_ur5(
            verbose=os.environ.get("UR5_SYMBOLIC_VERBOSE", "").strip().lower()
            in {"1", "true", "yes"}
        )
    except ImportError:
        print("SymPy not installed — ABA-only comparison.\n")

result = comprehensive_comparison(
    arm,
    symbolic_plant,
    configs,
    params=params,
    seed=seed,
    n_timing=n_timing,
    case_labels=labels,
)
print_comprehensive_report(result)
