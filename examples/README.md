# Examples

Human-facing runnable learning and experiments. Automated contracts and
runners live under [`tests/`](../tests/) and [`benchmarks/`](../benchmarks/).

Start in Colab:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb)
(Colab opens **notebook files** only — use the per-notebook badges below. Folder browsing is on GitHub.)

## Layout

```
examples/
  # --- Teaching Lane (Pedagogical contract: curated, canonical, CI-checked) ---
  tutorial/                    # basics/ (students), core/ (the foundation), advanced/ (tools) + showcases
  teaching/
    topics/<domain>/           # Textbook pages — refactorable (classical_control, optimal_control, …)
    courses/<id>/              # Live course pins (udes_gro860, udes_gro501, later udes_gmc714) — frozen paths
  demos/<chapter>/             # Canonical single-file textbook scripts, by topic (the chapters' areas)
  demos/udes_racecar/          # Plant folder: MPC and path tracking on the 1/10 car

  # --- Research Lane (Repo-only, unconstrained, exploratory) ---
  projects/<name>/             # Multi-file applications (pathtracking, car_trajopt, mpc)
```

Single-file prototypes, engine checks and scratch live outside `examples/`, in the top-level
[`experimental/`](../experimental/) folder: the experimental tier, never cited by a course,
untested by default (ROADMAP §2).

| Folder | Lane | When to use |
| --- | --- | --- |
| [`tutorial/`](tutorial/) | teaching, smoked | Learn **minilink** — numbered API notebooks + showcases |
| [`teaching/topics/`](teaching/topics/) | teaching, smoked unless long | Learn a **subject** — course-agnostic textbook page |
| [`teaching/courses/<id>/`](teaching/courses/) | teaching, smoked unless long | Homework cited by a live course — do not rename without the notes |
| [`demos/<chapter>/`](demos/) | teaching, nightly sweep | Canonical single-file demo (incl. pedagogical compares) |
| [`demos/udes_racecar/`](demos/udes_racecar/) | teaching, nightly | 1/10 racecar plant folder (MPC on the kinematic and dynamic models; pure pursuit, Stanley, line of sight and their comparison on the dynamic model) |
| [`projects/<name>/`](projects/) | research | Multi-file experiment — outside release contract, repo-only |
| [`../experimental/<topic>/`](../experimental/) | experimental tier | Non-core single files, scenario sprawl, WIP; `scratch/` for personal checks; outside `examples/` |

**Notebook names**

- Tutorial: `NN_<topic>.ipynb` and `showcase_<topic>.ipynb`.
- Topics and courses: `<plant>_<method>[_vs_<method>…][_sb3].ipynb` (snake_case).
  Plant first when there is one (`pendulum_`, `cartpole_`, `drone_`).
  Name the **method** (`value_iteration`, `lqr`, `ppo`). Add the **task**
  (`minimum_time`, `cost_to_go`) only when the same plant+method would collide.
  Default missions stay out of the filename (pendulum → swing-up, drone → fly).
  Compares use `_vs_` in lesson order. `_sb3` marks a Stable-Baselines3 twin.
  No `tuto_`, `template_`, `demo_`, `learn_to_`, or redundant `swingup`.
- The folder carries the audience (`topics/` vs `courses/`) and the domain; do not repeat them in the filename.

**Notebook links** (one map, this file): a tutorial chapter carries at most one same-folder
**Next** link to its numbered successor; the showcases are self-contained; a teaching
notebook links no other notebook, demo or library source (its Colab badge, the `git clone`
cell and external library docs stay); demos and projects link no notebook. `teaching/topics/`
is the textbook; `teaching/courses/<id>/` is a live homework pin.

**Promotion:** `experimental/scratch/` → `experimental/<topic>/` → `demos/<chapter>/`
(single-file) **or** `projects/<name>/` (multi-file) → optional `teaching/`
twin → README / `tutorial` only if core-tool canonical.

**Compare rule:** method/API side-by-sides that *are* the lesson stay in
`demos/` (e.g. RRT vs RRT*, VI vs LQR). Mission ladders and `*_v2*` live under
`experimental/<topic>/` or `projects/`.

**Demo naming:** no `demo_` prefix. Keep the topic in the stem when needed —
e.g. `mpc/mpc_car_minimal.py`. **Demo style:** RULES 6.1, 6.10, 6.11, 6.13, 6.16 and 6.17 — the
shortest sequence of library verbs, native object `print` and plots (not diagnostic f-strings), flat, side work in its own cell, at most three knobs in capitals on top and the other numbers named where the story needs them, each system named by the dynamics it is (`plant`, `closed_loop`). Do not mix
`.ipynb` and `.py` in the same leaf folder (except inside one named project).

## Open in Colab

Colab requires a `/blob/…/*.ipynb` path (not a GitHub `/tree/` folder). Browse
folders on GitHub, then open a notebook badge.

| Folder (GitHub) |
| --- |
| [`tutorial/`](https://github.com/alx87grd/minilink/tree/main/examples/tutorial) |
| [`teaching/`](https://github.com/alx87grd/minilink/tree/main/examples/teaching) |

### Tutorial — learn minilink

| Notebook | Colab |
| --- | --- |
| [showcase_minilink](tutorial/showcase_minilink.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_minilink.ipynb) |
| [showcase_jax](tutorial/showcase_jax.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_jax.ipynb) |
| [showcase_from_rl_to_bode](tutorial/showcase_from_rl_to_bode.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/showcase_from_rl_to_bode.ipynb) |

#### Basics — students: model, connect, simulate first; each notebook one step deeper

| Notebook | Colab |
| --- | --- |
| [01_minilink](tutorial/basics/01_minilink.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/basics/01_minilink.ipynb) |
| [02_analyze_and_control](tutorial/basics/02_analyze_and_control.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/basics/02_analyze_and_control.ipynb) |
| [03_plan_and_sample](tutorial/basics/03_plan_and_sample.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/basics/03_plan_and_sample.ipynb) |
#### Core — the foundation and the pipeline: systems, parameters, sets, fields, distributions, costs, the problem, compile and evaluate, programs, planners

| Notebook | Colab |
| --- | --- |
| [01_systems](tutorial/core/01_systems.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/01_systems.ipynb) |
| [02_parameters](tutorial/core/02_parameters.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/02_parameters.ipynb) |
| [03_sets_fields_distributions](tutorial/core/03_sets_fields_distributions.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/03_sets_fields_distributions.ipynb) |
| [04_costs](tutorial/core/04_costs.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/04_costs.ipynb) |
| [05_problem](tutorial/core/05_problem.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/05_problem.ipynb) |
| [06_compile_and_evaluate](tutorial/core/06_compile_and_evaluate.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/06_compile_and_evaluate.ipynb) |
| [07_program](tutorial/core/07_program.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/07_program.ipynb) |
| [08_planners](tutorial/core/08_planners.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/core/08_planners.ipynb) |
#### Advanced — tools: discrete and hybrid loops, optimal control, MPC and MPPI, reinforcement learning, differentiable loops

| Notebook | Colab |
| --- | --- |
| [01_discrete_and_hybrid](tutorial/advanced/01_discrete_and_hybrid.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/advanced/01_discrete_and_hybrid.ipynb) |
| [02_optimal_control](tutorial/advanced/02_optimal_control.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/advanced/02_optimal_control.ipynb) |
| [03_mpc_and_mppi](tutorial/advanced/03_mpc_and_mppi.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/advanced/03_mpc_and_mppi.ipynb) |
| [04_reinforcement_learning](tutorial/advanced/04_reinforcement_learning.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/advanced/04_reinforcement_learning.ipynb) |
| [05_differentiable](tutorial/advanced/05_differentiable.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/advanced/05_differentiable.ipynb) |

### Teaching — topics

Course-agnostic textbook pages. Long notebooks are skipped in fast CI smoke checks (`"smoke": false` in `tests/demo_checks/notebook_overrides.json`).

#### Classical control
| Notebook | Colab |
| --- | --- |
| [frequency_response](teaching/topics/classical_control/frequency_response.ipynb) (Bode, Nyquist, margins) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/classical_control/frequency_response.ipynb) |
| [sensitivity_functions](teaching/topics/classical_control/sensitivity_functions.ipynb) (S, T, PS, CS; disturbance and noise in simulation) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/classical_control/sensitivity_functions.ipynb) |
| [state_space_design](teaching/topics/classical_control/state_space_design.ipynb) (pole placement, LQR, Luenberger and Kalman observers, LQG, the reference gain N) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/classical_control/state_space_design.ipynb) |

#### Optimal control
| Notebook | Colab |
| --- | --- |
| [cartpole_rollout_gradients](teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) |

#### Machine learning
| Notebook | Colab |
| --- | --- |
| [least_squares_sgd](teaching/topics/machine_learning/least_squares_sgd.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/machine_learning/least_squares_sgd.ipynb) |

### Teaching — UdeS GRO860

Pinned by the course notes. Rename or move only with the notes and webpage in the same change.

| Notebook | Colab |
| --- | --- |
| [sgd_line](teaching/courses/udes_gro860/sgd_line.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/sgd_line.ipynb) |
| [pendulum_cost_to_go_approximation](teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb) |
| [pendulum_value_iteration](teaching/courses/udes_gro860/pendulum_value_iteration.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration.ipynb) |
| [pendulum_value_iteration_vs_lqr](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr.ipynb) |
| [cartpole_lqr](teaching/courses/udes_gro860/cartpole_lqr.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb) |
| [double_integrator_minimum_time](teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb) |
| [double_integrator_policy_evaluation](teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb) |
| [grid_world_dynamic_programming](teaching/courses/udes_gro860/grid_world_dynamic_programming.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/grid_world_dynamic_programming.ipynb) |
| [particle_gymnasium_sb3](teaching/courses/udes_gro860/particle_gymnasium_sb3.ipynb) (lab, French, Stable-Baselines3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/particle_gymnasium_sb3.ipynb) |
| [gymnasium_interface](teaching/courses/udes_gro860/gymnasium_interface.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/gymnasium_interface.ipynb) |
| [drone_ppo](teaching/courses/udes_gro860/drone_ppo.ipynb) (native JAX) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/drone_ppo.ipynb) |
| [pendulum_value_iteration_vs_lqr_vs_rl](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) (native JAX) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) |
| [drone_ppo_sb3](teaching/courses/udes_gro860/drone_ppo_sb3.ipynb) (Stable-Baselines3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/drone_ppo_sb3.ipynb) |
| [pendulum_value_iteration_vs_lqr_vs_ppo_sb3](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb) (Stable-Baselines3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb) |

### Teaching — UdeS GRO501

Pinned by the course notes (C5.1, C5.3, C5.4, C5.5). For now the only external pointer is the Notes-Commande `\colab{}` (no GRO501 course webpage). Rename or move only with that box in the same change.

| Notebook | Colab |
| --- | --- |
| [numpy_state_space](teaching/courses/udes_gro501/numpy_state_space.ipynb) (C5.1: NumPy / SciPy, python-control, minilink) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/numpy_state_space.ipynb) |
| [siso_transfer_function_analysis](teaching/courses/udes_gro501/siso_transfer_function_analysis.ipynb) (SISO analysis template: NumPy / SciPy, python-control, minilink) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/siso_transfer_function_analysis.ipynb) |
| [ode_simulation](teaching/courses/udes_gro501/ode_simulation.ipynb) (nonlinear simulation template: SciPy, minilink) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/ode_simulation.ipynb) |
| [cartpole_static_controller](teaching/courses/udes_gro501/cartpole_static_controller.ipynb) (C5.3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/cartpole_static_controller.ipynb) |
| [cartpole_dynamic_controller](teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb) (C5.4) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb) |
| [racecar_toward_mpc](teaching/courses/udes_gro501/racecar_toward_mpc.ipynb) (C5.5, optional: from LQR to MPC, then the same MPC on the dynamic 1/10 car) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro501/racecar_toward_mpc.ipynb) |

**Colab tip:** open a badge → **File → Save a copy in Drive** → run from the top.
The first code cell clones the repo and installs `meshcat` when needed. Locally,
use the `minilink` conda env (the [README's Install](../README.md#install)).

## By chapter

The demo folders grouped by tutorial chapter (a chapter may span several folders, and the
simulation chapter's demos sit in `demos/graphical/`; the tutorial notebooks are the Colab table
above); the research lane sits apart.

| Chapter | Demos | Teaching notebooks / projects |
| --- | --- | --- |
| Core / diagrams | [`demos/core/`](demos/core/) | |
| Blocks | [`demos/blocks/`](demos/blocks/) | |
| Dynamics catalog | [`demos/dynamics/`](demos/dynamics/) | `from minilink import Pendulum, CartPole, …` |
| Control | [`demos/control/`](demos/control/), [`demos/robotic/`](demos/robotic/) | [state_space_design](teaching/topics/classical_control/state_space_design.ipynb) (place, LQR, observers, LQG) |
| Analysis | [`demos/analysis/`](demos/analysis/) | [frequency_response](teaching/topics/classical_control/frequency_response.ipynb) (Bode, margins, Nyquist, root locus, step response) |
| Simulation | [`demos/graphical/`](demos/graphical/) | |
| Hybrid / step | [`demos/hybrid/`](demos/hybrid/), [`demos/mpc/`](demos/mpc/) | [`projects/mpc/`](projects/mpc/) (spatial MPC stack, dual-rate `mpc_dual_rate.py`), [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Compile / autodiff | [`demos/compile/`](demos/compile/) (`pid_autotuning_jax.py`: a PID tuned by a hand gradient loop through a JAX rollout, drawn live by `pe.live_plot`; `pid_autotuning_native.py`: the same PID tuned by the pipeline, problem → `problem.compile` → `pe.program` → `Optimizer` (native gradient descent, then L-BFGS), with the live plot; `cartpole_rollout_gradients.py`, `neural_controller_jax.py`, `params_gradient.py`: a loss through a rollout, a batch of closed loops or the equation error, wrapped in a `MathematicalProgram` and solved by `Optimizer`; `digital_pid_tuning.py`: the same pipeline on a digital PID, `problem.compile(pid % dt)`; `multi_rate_computer.py`: a 100 Hz filter and a 10 Hz gain on a continuous plant, one sampled loop; `differentiable_double_loop.py`: the core in one script, a neural inner loop on the pendulum and an MPC around it solved by five Adam steps a tick, the whole double loop one pure JAX function: one gradient over a plant parameter, a network weight and an MPC weight, their co-design by `Optimizer`, and the Bode plot of the closed loop in `z`, network and solver included; `native_mpc_tuning.py`: an MPC on a native fixed-iteration planner, its closed loop compiled once and its plan's weights tuned by L-BFGS through it; `terminal_field_mpc.py`: a three-period MPC on the inverted pendulum ended by the LQR value, `QuadraticField(x̄, S).as_terminal_cost()`, against the same MPC without it; `grid_world_step_problem.py`: a grid world as a `StepPlanningProblem` (a native step system, a cost per tick) solved by value iteration on its transition; `lqg_monte_carlo.py`: an LQG loop with a white noise on the torque and a datasheet sensor, its Monte Carlo mean against the covariance prediction) | [cartpole_rollout_gradients](teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) |
| Optimization | [`demos/optimization/`](demos/optimization/) | |
| Evaluation | [`demos/evaluation/`](demos/evaluation/) (`evaluate_controller.py`: a controller scored on its problem, one run, Monte Carlo over the problem's draws, two controllers compared on the same draws, the compiled evaluator) | |
| Planning | [`demos/trajopt/`](demos/trajopt/), [`demos/rrt/`](demos/rrt/), [`demos/value_iteration/`](demos/value_iteration/) | [cost+VI](teaching/courses/udes_gro860/pendulum_value_iteration.ipynb), [VI vs LQR](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr.ipynb), [grid world DP](teaching/courses/udes_gro860/grid_world_dynamic_programming.ipynb), [minimum-time VI](teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb), [policy evaluation](teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb), [cart-pole LQR](teaching/courses/udes_gro860/cartpole_lqr.ipynb); [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Reinforcement learning | [`demos/rl/`](demos/rl/) (pendulum swing-up, cart-pole, rocket landing, learn to fly) | [Gymnasium interface](teaching/courses/udes_gro860/gymnasium_interface.ipynb), [VI vs LQR vs RL](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb), [learn to fly](teaching/courses/udes_gro860/drone_ppo.ipynb), [function approximation by SGD](teaching/topics/machine_learning/least_squares_sgd.ipynb), [cost-to-go approximation](teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb); Lyapunov check in [`experimental/rl/`](../experimental/rl/) |
| Graphical | [`demos/graphical/`](demos/graphical/), [`demos/realtime/`](demos/realtime/) (keyboard/gamepad game mode: `game_cartpole.py`, `game_car.py`, `game_ur5_setpoint.py`) | |
| UdeS racecar | [`demos/udes_racecar/`](demos/udes_racecar/) (MPC on the kinematic model, MPC on the dynamic model; pure pursuit, Stanley, line of sight and their comparison on the dynamic model) | [racecar_toward_mpc](teaching/courses/udes_gro501/racecar_toward_mpc.ipynb) (GRO501 C5.5: kinematic MPC, then the same MPC on the dynamic car) |
| Experimental tier | [`../experimental/`](../experimental/) (`c_export/`, `compile/` diagram flattening, `engine/`, `symbolic/`, `robotic/` UR5, `boat/` parking in a current: cascade vs MPC vs PPO, `mppi/` pendulum and racecar MPPI, `rl/` Lyapunov check, `random/` noise and seeds) | not nightly-swept; C export demos: `experimental/c_export/c_export_proportional.py`, `experimental/c_export/c_export.py` |

Run demos from the repo root, e.g.
`PYTHONPATH=. python examples/demos/core/readme_examples.py`.

## CI smoke

Which notebooks and demos CI runs, and how: [tests/README.md](../tests/README.md#entry-points).
