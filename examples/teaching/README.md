# Teaching Notebooks

Universal, course-agnostic lessons organized by domain. Each notebook is a self-contained, open-and-run study of a specific control or dynamics topic using **minilink**.

## Topic Categories

### 1. [`classical_control/`](classical_control/)
LTI system analysis and classical frequency-domain design:
- [**`frequency_domain_tools.ipynb`**](classical_control/frequency_domain_tools.ipynb): Frequency response, Bode diagrams, Nyquist analysis, gain and phase margins on underdamped and resonant physical systems.

### 2. [`optimal_control/`](optimal_control/)
Dynamic programming, trajectory optimization, and direct methods:
- [**`grid_world_exact_dp.ipynb`**](optimal_control/grid_world_exact_dp.ipynb): Exact tabular dynamic programming, Bellman value iteration, policy convergence.
- [**`pendulum_swing_up_cost_function_vi.ipynb`**](optimal_control/pendulum_swing_up_cost_function_vi.ipynb): Value iteration on continuous state spaces and cost function shaping.
- [**`pendulum_swing_up_vi_vs_lqr.ipynb`**](optimal_control/pendulum_swing_up_vi_vs_lqr.ipynb): Global dynamic programming vs. local LQR basin of attraction.
- [**`cartpole_rollout_gradients.ipynb`**](optimal_control/cartpole_rollout_gradients.ipynb): Differentiating through simulation rollouts using JAX autodiff for trajectory optimization and co-design.

### 3. [`reinforcement_learning/`](reinforcement_learning/)
Policy gradients, deep RL on dynamical systems, and method comparisons:
- [**`policy_gradient_to_ppo.ipynb`**](reinforcement_learning/policy_gradient_to_ppo.ipynb): The mathematics from the policy gradient theorem to PPO — REINFORCE, baseline and advantage, actor-critic, GAE, the clipped objective — each checked against the planner's own functions.
- [**`gymnasium_interface.ipynb`**](reinforcement_learning/gymnasium_interface.ipynb): The Gymnasium environment contract read in optimal-control terms, written by hand, obtained from a planning problem, and trained through.
- [**`pendulum_swing_up_vi_vs_lqr_vs_rl.ipynb`**](reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_rl.ipynb): Dynamic programming vs. LQR vs. PPO on one plant and one cost, scored on one Monte Carlo yardstick.
- [**`drone_learn_to_fly.ipynb`**](reinforcement_learning/drone_learn_to_fly.ipynb): PPO policy optimization for 2D quadrotor hover and stabilization.
- [**`drone_ppo_learn_to_fly.ipynb`**](reinforcement_learning/drone_ppo_learn_to_fly.ipynb) and [**`pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb`**](reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb): the earlier versions of the last two, trained through Gymnasium with Stable-Baselines3.

### 4. [`robotics/`](robotics/)
Multibody kinematics, dynamics, and control of articulated robots:
- [**`articulated_robot_eom.ipynb`**](robotics/articulated_robot_eom.ipynb): UR5 6-DoF manipulator forward/inverse dynamics: Articulated-Body Algorithm (ABA), Recursive Newton-Euler (RNEA), and symbolic Euler-Lagrange.

---

## Pedagogical Design Policy

- **Course-Agnostic:** Notebooks are titled and written around the engineering topic, without university-specific course codes or semester labels.
- **Standalone:** Every notebook runs independently in Google Colab or locally in the `minilink` conda environment.
