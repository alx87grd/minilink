import numpy as np
from minilink.core.diagram import DiagramSystem
from minilink.core.analysis import Simulator, compute_internal_signals
from minilink.graphical.plotting import plot_signals

from minilink.blocks.basic import Pendulum, PendulumPDController
from minilink.blocks.sources import Step, WhiteNoise

if __name__ == "__main__":
    # --- 1. Set up the System Components ---
    # Plant system
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 5.0
    sys.x0[0] = 2.0

    # Source reference input
    step = Step()
    step.params["initial_value"] = np.array([0.0])
    step.params["final_value"] = np.array([1.0])
    step.params["step_time"] = 5.0

    # Sensor noise
    noise2 = WhiteNoise(1)
    noise2.params["var"] = 0.05
    noise2.params["mean"] = 0.0
    noise2.params["seed"] = 2

    # Closed loop controller
    ctl = PendulumPDController()
    ctl.params["Kp"] = 1000.0
    ctl.params["Kd"] = 100.0

    # --- 2. Construct Diagram System ---
    diagram = DiagramSystem()
    diagram.name = "Closed Loop Noisy Pendulum"

    diagram.add_subsystem(step, "step")
    diagram.add_subsystem(ctl, "controller")
    diagram.add_subsystem(sys, "plant")
    diagram.add_subsystem(noise2, "noise2")

    diagram.connect("step", "y", "controller", "ref")
    diagram.connect("controller", "u", "plant", "u")
    diagram.connect("plant", "y", "controller", "y")
    diagram.connect("noise2", "y", "plant", "v")

    # External input
    diagram.add_input_port(1, "w", nominal_value=np.array([0.0]))
    diagram.connect("input", "w", "plant", "w")

    diagram.compile()
    diagram.plot_graphe()

    # --- 3. Simulate System ---
    print("Running simulation...")
    sim = Simulator(diagram, t0=0, tf=15, dt=0.01, solver="euler", verbose=False)
    traj = sim.solve(show=False)

    # --- 4. Advanced Plotting ---
    print("Reconstructing internal signals...")
    # Scrape all diagram data to view internal outputs
    traj_plus = compute_internal_signals(diagram, traj)

    print(f"Diagram State Labels: {diagram.state.labels}")
    print("Generating advanced selective plots...")
    # Pick completely custom layouts for your multi-subplot
    plot_signals(
        diagram,
        traj_plus,
        signals=[
            {
                "sys": "plant",
                "state": ["theta", "theta_dot"],
            },
            {"sys": "plant", "output": "y"},
            {"sys": "controller", "output": "u"},
            {"sys": "step", "output": "y"},
        ],
    )
    print("Demo complete!")
