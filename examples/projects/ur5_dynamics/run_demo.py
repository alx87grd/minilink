"""Compare UR5 equation-of-motion techniques: RNEA–H, ABA, symbolic Lagrange.

Run from repo root::

    PYTHONPATH=. python examples/projects/ur5_dynamics/run_demo.py

Optional environment variables:

* ``UR5_SKIP_SYMBOLIC=1`` — skip the slow SymPy derive/export step.
* ``UR5_SYMBOLIC_VERBOSE=1`` — print symbolic build timings.
* ``UR5_N_SAMPLES`` — random samples after named case studies (default 64).
* ``UR5_SEED`` — RNG seed (default 0).
* ``UR5_N_TIMING`` — timing repeats (default 200).

Textbook walkthrough with plots: ``eom_comparison.ipynb`` in this folder.
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

print("UR5 forward dynamics — EoM technique comparison")
print("Reference: RNEA bias + explicit H solve (catalog forward_dynamics)")
print("Damping disabled so symbolic Lagrange plant matches spatial parameters.")
print(f"Evaluation batch: {len(labels) - n_samples} named cases + {n_samples} random (seed={seed})")

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
