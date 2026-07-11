import numpy as np

from minilink.control.siso import FilteredController
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.interfaces.c_export import export_system_to_c


def main():
    print(
        "Building Diagram: DoubleIntegrator + FilteredController (Anti-windup PID)..."
    )
    plant = DoubleIntegrator()
    # FilteredController has anti-windup logic (u_min, u_max, e_int_min, e_int_max)
    # which will generate JAX logic primitives (gt, lt, select_n)
    controller = FilteredController(
        dof=1,
        kp=10.0,
        ki=1.0,
        kd=2.0,
        tau=0.05,
        u_min=-5.0,
        u_max=5.0,
        e_int_min=-1.0,
        e_int_max=1.0,
    )

    diagram = DiagramSystem()
    diagram.add_subsystem(plant, "plant")
    diagram.add_subsystem(controller, "ctl")

    # Connect plant output 'y' directly to controller measurement 'y'
    diagram.connect("plant", "y", "ctl", "y")
    # Connect controller output 'u' to plant input 'u'
    diagram.connect("ctl", "u", "plant", "u")

    print(f"Diagram state size: {diagram.n}")
    print(f"Diagram input size (reference r): {diagram.m}")

    print("\nExporting execution plan to C...")
    # Provide dummy arrays for shape tracing
    x_sample = np.zeros(diagram.n)
    u_sample = np.zeros(diagram.m)

    c_code = export_system_to_c(diagram, "evaluate_closed_loop", x_sample, u_sample)

    print("\nGenerated C Code:")
    print("-" * 60)
    print(c_code)
    print("-" * 60)

    # Save to a file for testing
    with open("pendulum_controller.c", "w") as f:
        f.write(c_code)
    print("\nSaved output to pendulum_controller.c")


if __name__ == "__main__":
    main()
