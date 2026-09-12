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
  teaching/                    # Universal, course-agnostic domain lessons
    classical_control/         # Frequency domain, Bode/Nyquist, PID, state observers
    optimal_control/           # DP, value iteration, cartpole rollouts, trajopt
    reinforcement_learning/    # PPO, policy gradients, RL swing-up compares
    robotics/                  # Manipulator kinematics, Euler-Lagrange EoM, impedance
  demos/<chapter>/             # Canonical single-file textbook scripts (1:1 with tutorial chapters)

  # --- Research Lane (Repo-only, unconstrained, exploratory) ---
  projects/<name>/             # Multi-file applications (pathtracking, car_trajopt, mpc)
  experimental/<topic>/        # Bleeding-edge single-file prototypes, engine checks, scratch
```

| Folder | Lane | When to use |
| --- | --- | --- |
| [`tutorial/`](tutorial/) | teaching, smoked | Learn **minilink** — numbered API notebooks + showcases |
| [`teaching/`](teaching/) | teaching, smoked unless long | Learn a **subject** — universal, course-agnostic domain lessons |
| [`demos/<chapter>/`](demos/) | teaching, nightly sweep | Canonical single-file demo (incl. pedagogical compares) |
| [`projects/<name>/`](projects/) | research | Multi-file experiment — outside release contract, repo-only |
| [`experimental/<topic>/`](experimental/) | research | Non-core single files, scenario sprawl, WIP; `scratch/` for personal checks |

**Promotion:** `experimental/scratch/` → `experimental/<topic>/` → `demos/<chapter>/`
(single-file) **or** `projects/<name>/` (multi-file) → optional `teaching/`
twin → README / `tutorial` only if core-tool canonical.

**Compare rule:** method/API side-by-sides that *are* the lesson stay in
`demos/` (e.g. RRT vs RRT*, VI vs LQR). Mission ladders and `*_v2*` live under
`experimental/<topic>/` or `projects/`.

**Demo naming:** no `demo_` prefix. Keep the topic in the stem when needed —
e.g. `mpc/mpc_car_minimal.py`. **Demo style:** one-line docstring, top-level
constants + sequential calls (open and run); no `def main()`. Do not mix
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

### Teaching — learn a subject

Universal, course-agnostic lessons organized by domain. Long notebooks (UR5 EoM, DP grids, PPO training) are skipped in fast CI smoke checks.

#### Classical Control
| Notebook | Colab |
| --- | --- |
| [frequency_domain_tools](teaching/classical_control/frequency_domain_tools.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/classical_control/frequency_domain_tools.ipynb) |

#### Optimal Control
| Notebook | Colab |
| --- | --- |
| [grid_world_exact_dp](teaching/optimal_control/grid_world_exact_dp.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/optimal_control/grid_world_exact_dp.ipynb) |
| [pendulum_swing_up_cost_function_vi](teaching/optimal_control/pendulum_swing_up_cost_function_vi.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/optimal_control/pendulum_swing_up_cost_function_vi.ipynb) |
| [pendulum_swing_up_vi_vs_lqr](teaching/optimal_control/pendulum_swing_up_vi_vs_lqr.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/optimal_control/pendulum_swing_up_vi_vs_lqr.ipynb) |
| [cartpole_rollout_gradients](teaching/optimal_control/cartpole_rollout_gradients.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/optimal_control/cartpole_rollout_gradients.ipynb) |

#### Reinforcement Learning
| Notebook | Colab |
| --- | --- |
| [policy_gradient_to_ppo](teaching/reinforcement_learning/policy_gradient_to_ppo.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/policy_gradient_to_ppo.ipynb) |
| [gymnasium_interface](teaching/reinforcement_learning/gymnasium_interface.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/gymnasium_interface.ipynb) |
| [pendulum_swing_up_vi_vs_lqr_vs_rl](teaching/reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_rl.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_rl.ipynb) |
| [drone_learn_to_fly](teaching/reinforcement_learning/drone_learn_to_fly.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/drone_learn_to_fly.ipynb) |
| [drone_ppo_learn_to_fly, Stable-Baselines3 version](teaching/reinforcement_learning/drone_ppo_learn_to_fly.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/drone_ppo_learn_to_fly.ipynb) |
| [pendulum_swing_up_vi_vs_lqr_vs_ppo, Stable-Baselines3 version](teaching/reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb) |

#### Robotics
| Notebook | Colab |
| --- | --- |
| [articulated_robot_eom](teaching/robotics/articulated_robot_eom.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/teaching/robotics/articulated_robot_eom.ipynb) |

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
| Control | [03_control](tutorial/03_control.ipynb) | [`demos/control/`](demos/control/), [`demos/robotic/`](demos/robotic/) | [articulated_robot_eom](teaching/robotics/articulated_robot_eom.ipynb) |
| Analysis | [04_analysis](tutorial/04_analysis.ipynb) | [`demos/analysis/`](demos/analysis/) | [frequency_domain_tools](teaching/classical_control/frequency_domain_tools.ipynb) (Bode, margins, Nyquist, root locus, step response) |
| Simulation | [05_simulation](tutorial/05_simulation.ipynb) | [`demos/graphical/`](demos/graphical/) | |
| Hybrid / step | [06_hybrid](tutorial/06_hybrid.ipynb) | [`demos/hybrid/`](demos/hybrid/), [`demos/mpc/`](demos/mpc/) | [`projects/mpc/`](projects/mpc/) (spatial MPC stack, dual-rate `mpc_dual_rate.py`), [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Compile / autodiff | [07_compile](tutorial/07_compile.ipynb), [showcase_jax](tutorial/showcase_jax.ipynb) | [`demos/compile/`](demos/compile/) | [cartpole_rollout_gradients](teaching/optimal_control/cartpole_rollout_gradients.ipynb) |
| Optimization | [08_optimization](tutorial/08_optimization.ipynb) | [`demos/optimization/`](demos/optimization/) | |
| Planning | [09_planning](tutorial/09_planning.ipynb) | [`demos/planning/`](demos/planning/) (`trajopt/`, `rrt/`, `value_iteration/`) | [cost+VI](teaching/optimal_control/pendulum_swing_up_cost_function_vi.ipynb), [VI vs LQR](teaching/optimal_control/pendulum_swing_up_vi_vs_lqr.ipynb), [grid world DP](teaching/optimal_control/grid_world_exact_dp.ipynb); [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Reinforcement learning | [11_reinforcement_learning](tutorial/11_reinforcement_learning.ipynb), [showcase_from_rl_to_bode](tutorial/showcase_from_rl_to_bode.ipynb) | [`demos/rl/`](demos/rl/) | [policy gradient to PPO](teaching/reinforcement_learning/policy_gradient_to_ppo.ipynb), [Gymnasium interface](teaching/reinforcement_learning/gymnasium_interface.ipynb), [VI vs LQR vs RL](teaching/reinforcement_learning/pendulum_swing_up_vi_vs_lqr_vs_rl.ipynb), [learn to fly](teaching/reinforcement_learning/drone_learn_to_fly.ipynb); research prototypes in [`experimental/rl/`](experimental/rl/) |
| Graphical | [10_graphical](tutorial/10_graphical.ipynb) | [`demos/graphical/`](demos/graphical/), [`demos/realtime/`](demos/realtime/) (keyboard game mode: `game_cartpole.py`) | |
| Research lane | — | [`experimental/`](experimental/) (`c_export/`, `engine/`, `symbolic/`, `robotic/` UR5) | not swept; `c_export` runs in the JAX regression job; C export demos: `experimental/c_export/c_export_proportional.py`, `c_export.py` |
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
