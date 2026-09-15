# Tutorial Notebooks

Step-by-step walkthroughs of the **minilink** framework, numbered sequentially by package:

## Flagship Showcases
- [**`showcase_minilink.ipynb`**](showcase_minilink.ipynb): The full tool ladder on real plants (modeling, simulation, control, trajectory optimization).
- [**`showcase_jax.ipynb`**](showcase_jax.ipynb): Write `f` once, evaluate every gradient and Jacobian with native JAX autodiff.
- [**`showcase_from_rl_to_bode.ipynb`**](showcase_from_rl_to_bode.ipynb): Six-axis UR5 robot, impedance control, learned neural policy, frequency response, and Lyapunov stability certificates.

## Numbered Package Walkthrough
- [**`00_core.ipynb`**](00_core.ipynb): Systems, ports, signals, parameters, state vectors.
- [**`01_blocks.ipynb`**](01_blocks.ipynb): Combinators, gains, sums, saturations, transfer functions.
- [**`02_dynamics.ipynb`**](02_dynamics.ipynb): Physics equations, nonlinear dynamics, standard catalog models.
- [**`03_control.ipynb`**](03_control.ipynb): Feedback loops, PID, pole placement, state feedback.
- [**`04_analysis.ipynb`**](04_analysis.ipynb): Linearization, Jacobians, modal analysis, Bode and Nyquist plots.
- [**`05_simulation.ipynb`**](05_simulation.ipynb): Time-domain simulation, integrators (RK4, Euler, adaptive), trajectories.
- [**`06_hybrid.ipynb`**](06_hybrid.ipynb): Hybrid systems, discrete steps, MPC simulation orchestration.
- [**`07_compile.ipynb`**](07_compile.ipynb): JIT compilation, XLA backend, vectorized simulation.
- [**`08_optimization.ipynb`**](08_optimization.ipynb): Mathematical programming, IPOPT and SciPy interfaces.
- [**`09_planning.ipynb`**](09_planning.ipynb): Trajectory optimization, direct collocation, shooting methods.
- [**`10_graphical.ipynb`**](10_graphical.ipynb): Visualizations, Meshcat 3D animations, interactive diagrams.
- [**`11_reinforcement_learning.ipynb`**](11_reinforcement_learning.ipynb): Reinforcement learning as a planner: stochastic problems, the rollout environment, the discount, PPO and SAC, and the learned law as a controller block.
