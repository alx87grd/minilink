"""C export teaching demo: export a proportional controller only.

Pipeline
--------
1. Build ``ProportionalController`` (static: ``u = K (r - y)``).
2. Export its output map to a standalone C function (not a closed-loop plant).
3. Compile with ``cc``, call via ctypes, and compare to Python (JAX).

The generated C is short: read ``r`` and ``y`` from the input vector, subtract,
multiply by ``K``.

Run from repo root::

    PYTHONPATH=. python examples/scripts/interfaces/c_export_proportional.py
"""

import numpy as np

from minilink.control.output import ProportionalController
from minilink.interfaces.c_export import export_system_to_c, load_exported_c

# Static controller: n=0, stacked input u_in = [r, y], output port "u".
controller = ProportionalController(K=8.7)
print(f"n={controller.n}, m={controller.m}  (expect n=0, m=2 for [r, y])")

# Trace outputs → jaxpr → C (float32). No plant, no closed loop.
func_name = "evaluate_controller"
c_code = export_system_to_c(controller, func_name)
print("\n--- generated C ---")
print(c_code)
print("--- end C ---\n")

# Python oracle vs compiled C.
evaluator = controller.compile(backend="jax")
c_eval = load_exported_c(c_code, func_name, n_out=1)

# r=0, y=1 → u = 4 (0 - 1) = -4
x = np.zeros(0, dtype=np.float32)
u_in = np.array([0.0, 1.0], dtype=np.float32)
u_py = np.asarray(evaluator.outputs(x, u_in, 0.0)["u"], dtype=np.float32).ravel()
u_c = c_eval(x, u_in)
err = float(np.max(np.abs(u_py - u_c)))
print(f"[r, y]={u_in}")
print(f"u_py={u_py}  u_c={u_c}  abs_err={err:.3e}")
if err > 1e-5:
    raise SystemExit("Python and C diverge beyond float32 tolerance.")
print("OK — C matches Python within float32 tolerance.")
