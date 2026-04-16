import numpy as np

from minilink.core.framework import DynamicSystem
from minilink.graphical.animation import Animator


class DemoSystem(DynamicSystem):
    """
    A simple custom system to demonstrate the baseline graphical engine.
    State 0 integrates the input.
    State 1 moves at a constant velocity.
    """

    def __init__(self):
        super().__init__(n=2, m=1, p=1)
        self.name = "Demo Points System"

        # Override the input port's signal so it is not just constant 0
        self.inputs["u"].get_signal = lambda t: np.array([np.sin(t) * 3.0])

    def f(self, x, u, t=0, params=None) -> np.ndarray:
        dx = np.zeros(self.n)
        dx[0] = u[0]  # x0 velocity is the input function
        dx[1] = 0.5  # x1 velocity is constant 0.5
        return dx


if __name__ == "__main__":
    # 1. Instantiate the system and set initial conditions
    sys = DemoSystem()
    sys.x0 = np.array([-3.0, -1.0])

    # 2. Simulate the system to get a trajectory
    print("Computing simulation trajectory...")
    traj = sys.compute_trajectory(t0=0, tf=10, n_steps=200, show=False)

    # 3. Create the Animator and adjust the zoom bounding box domain
    animator = Animator(sys)
    animator.domain = [[-5, 5], [-5, 5], [-5, 5]]

    # 4. Demonstrate a single frame using show()
    # print("Showing a single static frame initialization...")
    # initial_u = sys.get_u_from_input_ports()
    # animator.show(x=sys.x0, u=initial_u, t=0, is_3d=False)

    # 5. Demonstrate the full playback video
    print("Playing back the full animation...")
    animator.animate_simulation(traj, time_factor_video=2.0, is_3d=False)
