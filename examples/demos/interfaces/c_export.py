"""C export: FilteredController alone (anti-windup / richer C).

Exports the controller leaf only — its continuous ``f`` (integrator + filter
state), not a plant closed loop. Saturation logic becomes comparisons and
``select_n`` / ``fmaxf`` / ``fminf`` in the C.

For a minimal static export plus ctypes round-trip, see
``c_export_proportional.py``.

Run from repo root::

    PYTHONPATH=. python examples/demos/interfaces/c_export.py
"""

from minilink.control.siso import FilteredController
from minilink.interfaces.c_export import export_system_to_c

# Dynamic SISO PID with filtered derivative and anti-windup (n=2, m=2 for [r, y]).
controller = FilteredController(
    dof=1,
    Kp=10.0,
    Ki=1.0,
    Kd=2.0,
    tau=0.05,
    u_min=-5.0,
    u_max=5.0,
    e_int_min=-1.0,
    e_int_max=1.0,
)
print(f"n={controller.n}, m={controller.m}")

c_code = export_system_to_c(controller, "evaluate_controller")
print("\n--- generated C ---")
print(c_code)
print("--- end C ---")
