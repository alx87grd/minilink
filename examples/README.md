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
  tutorial/                    # Numbered minilink API walk: 00_core … 11_reinforcement_learning + showcases
  teaching/
    topics/<domain>/           # Textbook pages — refactorable (classical_control, optimal_control, …)
    courses/<id>/              # Live course pins (udes_gro860, later udes_gmc714) — frozen paths
  demos/<chapter>/             # Canonical single-file textbook scripts (1:1 with tutorial chapters)

  # --- Research Lane (Repo-only, unconstrained, exploratory) ---
  projects/<name>/             # Multi-file applications (pathtracking, car_trajopt, mpc)
  experimental/<topic>/        # Bleeding-edge single-file prototypes, engine checks, scratch
```

| Folder | Lane | When to use |
| --- | --- | --- |
| [`tutorial/`](tutorial/) | teaching, smoked | Learn **minilink** — numbered API notebooks + showcases |
| [`teaching/topics/`](teaching/topics/) | teaching, smoked unless long | Learn a **subject** — course-agnostic textbook page |
| [`teaching/courses/<id>/`](teaching/courses/) | teaching, smoked unless long | Homework cited by a live course — do not rename without the notes |
| [`demos/<chapter>/`](demos/) | teaching, nightly sweep | Canonical single-file demo (incl. pedagogical compares) |
| [`projects/<name>/`](projects/) | research | Multi-file experiment — outside release contract, repo-only |
| [`experimental/<topic>/`](experimental/) | research | Non-core single files, scenario sprawl, WIP; `scratch/` for personal checks |

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

**Promotion:** `experimental/scratch/` → `experimental/<topic>/` → `demos/<chapter>/`
(single-file) **or** `projects/<name>/` (multi-file) → optional `teaching/`
twin → README / `tutorial` only if core-tool canonical.

**Compare rule:** method/API side-by-sides that *are* the lesson stay in
`demos/` (e.g. RRT vs RRT*, VI vs LQR). Mission ladders and `*_v2*` live under
`experimental/<topic>/` or `projects/`.

**Demo naming:** no `demo_` prefix. Keep the topic in the stem when needed —
e.g. `mpc/mpc_car_minimal.py`. **Demo style:** RULES 6.1, 6.10, 6.11 and 6.13 — the
shortest sequence of library verbs, native object `print` and plots (not diagnostic f-strings), flat, side work in its own cell. Do not mix
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
| [00_core](tutorial/00_core.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/00_core.ipynb) |
| [01_blocks](tutorial/01_blocks.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/01_blocks.ipynb) |
| [02_dynamics](tutorial/02_dynamics.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/02_dynamics.ipynb) |
| [03_control](tutorial/03_control.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/03_control.ipynb) |
| [04_analysis](tutorial/04_analysis.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/04_analysis.ipynb) |
| [05_simulation](tutorial/05_simulation.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/05_simulation.ipynb) |
| [06_hybrid](tutorial/06_hybrid.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/06_hybrid.ipynb) |
| [07_compile](tutorial/07_compile.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/07_compile.ipynb) |
| [08_optimization](tutorial/08_optimization.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/08_optimization.ipynb) |
| [09_planning](tutorial/09_planning.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/09_planning.ipynb) |
| [10_graphical](tutorial/10_graphical.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/10_graphical.ipynb) |
| [11_reinforcement_learning](tutorial/11_reinforcement_learning.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tutorial/11_reinforcement_learning.ipynb) |

### Teaching — topics

Course-agnostic textbook pages. Long notebooks (UR5 EoM, DP grids, PPO training) are skipped in fast CI smoke checks.

#### Classical control
| Notebook | Colab |
| --- | --- |
| [frequency_response](teaching/topics/classical_control/frequency_response.ipynb) (Bode, Nyquist, margins) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/classical_control/frequency_response.ipynb) |

#### Optimal control
| Notebook | Colab |
| --- | --- |
| [cartpole_rollout_gradients](teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) |

#### Reinforcement learning
| Notebook | Colab |
| --- | --- |
| [pendulum_value_iteration_vs_lqr_vs_rl](teaching/topics/reinforcement_learning/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/reinforcement_learning/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) |
| [drone_ppo](teaching/topics/reinforcement_learning/drone_ppo.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/reinforcement_learning/drone_ppo.ipynb) |

#### Robotics
| Notebook | Colab |
| --- | --- |
| [manipulator_eom](teaching/topics/robotics/manipulator_eom.ipynb) (UR5 ABA, RNEA, Euler-Lagrange) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/topics/robotics/manipulator_eom.ipynb) |

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
| [gymnasium_interface](teaching/courses/udes_gro860/gymnasium_interface.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/gymnasium_interface.ipynb) |
| [drone_ppo](teaching/courses/udes_gro860/drone_ppo.ipynb) (native JAX) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/drone_ppo.ipynb) |
| [pendulum_value_iteration_vs_lqr_vs_rl](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) (native JAX) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb) |
| [drone_ppo_sb3](teaching/courses/udes_gro860/drone_ppo_sb3.ipynb) (Stable-Baselines3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/drone_ppo_sb3.ipynb) |
| [pendulum_value_iteration_vs_lqr_vs_ppo_sb3](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb) (Stable-Baselines3) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb) |

**Colab tip:** open a badge → **File → Save a copy in Drive** → run from the top.
The first code cell clones the repo and installs `meshcat` when needed. Locally,
use the `minilink` conda env (see root [README](../README.md#install)).

## By chapter

One demo folder per tutorial chapter; the research lane sits apart.

| Chapter | Tutorial notebook | Demos | Teaching notebooks / projects |
| --- | --- | --- | --- |
| Core / diagrams | [00_core](tutorial/00_core.ipynb) | [`demos/core/`](demos/core/) | |
| Blocks | [01_blocks](tutorial/01_blocks.ipynb) | [`demos/blocks/`](demos/blocks/) | |
| Dynamics catalog | [02_dynamics](tutorial/02_dynamics.ipynb) | [`demos/dynamics/`](demos/dynamics/) | `from minilink import Pendulum, CartPole, …` |
| Control | [03_control](tutorial/03_control.ipynb) | [`demos/control/`](demos/control/), [`demos/robotic/`](demos/robotic/) | [manipulator_eom](teaching/topics/robotics/manipulator_eom.ipynb) |
| Analysis | [04_analysis](tutorial/04_analysis.ipynb) | [`demos/analysis/`](demos/analysis/) | [frequency_response](teaching/topics/classical_control/frequency_response.ipynb) (Bode, margins, Nyquist, root locus, step response) |
| Simulation | [05_simulation](tutorial/05_simulation.ipynb) | [`demos/graphical/`](demos/graphical/) | |
| Hybrid / step | [06_hybrid](tutorial/06_hybrid.ipynb) | [`demos/hybrid/`](demos/hybrid/), [`demos/mpc/`](demos/mpc/) | [`projects/mpc/`](projects/mpc/) (spatial MPC stack, dual-rate `mpc_dual_rate.py`), [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Compile / autodiff | [07_compile](tutorial/07_compile.ipynb), [showcase_jax](tutorial/showcase_jax.ipynb) | [`demos/compile/`](demos/compile/) | [cartpole_rollout_gradients](teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb) |
| Optimization | [08_optimization](tutorial/08_optimization.ipynb) | [`demos/optimization/`](demos/optimization/) | |
| Planning | [09_planning](tutorial/09_planning.ipynb) | [`demos/trajopt/`](demos/trajopt/), [`demos/rrt/`](demos/rrt/), [`demos/value_iteration/`](demos/value_iteration/) | [cost+VI](teaching/courses/udes_gro860/pendulum_value_iteration.ipynb), [VI vs LQR](teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr.ipynb), [grid world DP](teaching/courses/udes_gro860/grid_world_dynamic_programming.ipynb), [minimum-time VI](teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb), [policy evaluation](teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb), [cart-pole LQR](teaching/courses/udes_gro860/cartpole_lqr.ipynb); [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Reinforcement learning | [11_reinforcement_learning](tutorial/11_reinforcement_learning.ipynb), [showcase_from_rl_to_bode](tutorial/showcase_from_rl_to_bode.ipynb) | [`demos/rl/`](demos/rl/) (pendulum swing-up, cart-pole, rocket landing, learn to fly) | [Gymnasium interface](teaching/courses/udes_gro860/gymnasium_interface.ipynb), [VI vs LQR vs RL](teaching/topics/reinforcement_learning/pendulum_value_iteration_vs_lqr_vs_rl.ipynb), [learn to fly](teaching/topics/reinforcement_learning/drone_ppo.ipynb), [function approximation by SGD](teaching/topics/machine_learning/least_squares_sgd.ipynb), [cost-to-go approximation](teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb); Lyapunov check in [`experimental/rl/`](experimental/rl/) |
| Graphical | [10_graphical](tutorial/10_graphical.ipynb) | [`demos/graphical/`](demos/graphical/), [`demos/realtime/`](demos/realtime/) (keyboard game mode: `game_cartpole.py`) | |
| Research lane | — | [`experimental/`](experimental/) (`c_export/`, `engine/`, `symbolic/`, `robotic/` UR5) | not nightly-swept; `c_export` is TRL 1 (flagship smoke in the JAX regression job); C export demos: `experimental/c_export/c_export_proportional.py`, `c_export.py` |
| Solver benchmarks | — | [`benchmarks/`](../benchmarks/) | Performance tracking and backend sweeps live in repo-root [`benchmarks/`](../benchmarks/) (run locally or in Colab via [`benchmarks/ode_solver_benchmark.ipynb`](../benchmarks/ode_solver_benchmark.ipynb)) |

Run demos from the repo root, e.g.
`PYTHONPATH=. python examples/demos/core/readme_examples.py`.

## CI smoke

Notebook smoke covers `tutorial/` and `teaching/` (except long
notebooks with `"smoke": false` — UR5 EoM, DP grids, PPO;
not `projects/` or `experimental/`). Locally:

```bash
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
```

The nightly sweep runs every script under `demos/`; the JAX regression job runs
the flagship manifest (`tests/demo_checks/flagship_manifest.json`).
