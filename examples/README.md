# Examples

Human-facing runnable learning and experiments. Automated contracts and
runners live under [`tests/`](../tests/) and [`benchmarks/`](../benchmarks/).

Start in Colab:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)
(Colab opens **notebook files** only — use the per-notebook badges below. Folder browsing is on GitHub.)

## Layout

```
examples/
  learn/
    intro/         # tutorials of the library: 00_core … 10_graphical + two showcases
    teaching/      # subject lessons (swing-up, DP, PPO, robot EoM, autodiff rollouts)
  demos/<chapter>/ # canonical single-file scripts, one folder per intro chapter
  projects/<name>/ # multi-file experiments (run_demo.py + helpers ± notebook)
  experimental/    # research-lane single files (engines, symbolic, C export, UR5)
  tooling/         # benches (notebooks)
```

| Folder | Lane | When to use |
| --- | --- | --- |
| [`learn/intro/`](learn/intro/) | teaching, smoked | Learn **minilink** — numbered API notebooks + showcase |
| [`learn/teaching/`](learn/teaching/) | teaching, smoked unless long | Learn a **subject** (reusable domain notebooks) |
| [`demos/<chapter>/`](demos/) | teaching, nightly sweep | Canonical single-file demo (incl. pedagogical compares) |
| [`projects/<name>/`](projects/) | research | Multi-file experiment — outside the release contract, not CI-checked |
| [`experimental/<topic>/`](experimental/) | research | Non-core single files, scenario sprawl, WIP; `scratch/` for personal checks (not README/CI) |
| [`tooling/`](tooling/) | dev | Dev matrices / benches |

**Promotion:** `experimental/scratch/` → `experimental/<topic>/` → `demos/<chapter>/`
(single-file) **or** `projects/<name>/` (multi-file) → optional `learn/teaching/`
twin → README / `intro` only if core-tool canonical.

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
| [`learn/`](https://github.com/alx87grd/minilink/tree/main/examples/learn) |
| [`learn/intro/`](https://github.com/alx87grd/minilink/tree/main/examples/learn/intro) |
| [`learn/teaching/`](https://github.com/alx87grd/minilink/tree/main/examples/learn/teaching) |
| [`tooling/notebooks/`](https://github.com/alx87grd/minilink/tree/main/examples/tooling/notebooks) |

### Intro — learn minilink

| Notebook | Colab |
| --- | --- |
| [showcase_minilink](learn/intro/showcase_minilink.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb) |
| [showcase_jax](learn/intro/showcase_jax.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_jax.ipynb) |
| [00_core](learn/intro/00_core.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/00_core.ipynb) |
| [01_blocks](learn/intro/01_blocks.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/01_blocks.ipynb) |
| [02_dynamics](learn/intro/02_dynamics.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/02_dynamics.ipynb) |
| [03_control](learn/intro/03_control.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/03_control.ipynb) |
| [04_analysis](learn/intro/04_analysis.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/04_analysis.ipynb) |
| [05_simulation](learn/intro/05_simulation.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/05_simulation.ipynb) |
| [06_hybrid](learn/intro/06_hybrid.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/06_hybrid.ipynb) |
| [07_compile](learn/intro/07_compile.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/07_compile.ipynb) |
| [08_optimization](learn/intro/08_optimization.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/08_optimization.ipynb) |
| [09_planning](learn/intro/09_planning.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/09_planning.ipynb) |
| [10_graphical](learn/intro/10_graphical.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/10_graphical.ipynb) |

### Teaching

Long notebooks (UR5 EoM, DP grids, PPO training) are not smoked by CI.

| Notebook | Colab |
| --- | --- |
| [frequency_domain_tools](learn/teaching/frequency_domain_tools.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/frequency_domain_tools.ipynb) |
| [pendulum_swing_up_cost_function_vi](learn/teaching/pendulum_swing_up_cost_function_vi.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/pendulum_swing_up_cost_function_vi.ipynb) |
| [pendulum_swing_up_vi_vs_lqr](learn/teaching/pendulum_swing_up_vi_vs_lqr.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/pendulum_swing_up_vi_vs_lqr.ipynb) |
| [pendulum_swing_up_vi_vs_lqr_vs_ppo](learn/teaching/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb) |
| [grid_world_exact_dp](learn/teaching/grid_world_exact_dp.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/grid_world_exact_dp.ipynb) |
| [drone_ppo_learn_to_fly](learn/teaching/drone_ppo_learn_to_fly.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/drone_ppo_learn_to_fly.ipynb) |
| [cartpole_rollout_gradients](learn/teaching/cartpole_rollout_gradients.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/cartpole_rollout_gradients.ipynb) |
| [articulated_robot_eom](learn/teaching/articulated_robot_eom.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/articulated_robot_eom.ipynb) |

### Tooling

| Notebook | Colab |
| --- | --- |
| [benchmark](tooling/notebooks/benchmark.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tooling/notebooks/benchmark.ipynb) |

**Colab tip:** open a badge → **File → Save a copy in Drive** → run from the top.
The first code cell clones the repo and installs `meshcat` when needed. Locally,
use the `minilink` conda env (see root [README](../README.md#install)).

## By chapter

One demo folder per intro chapter; the research lane sits apart.

| Chapter | Intro notebook | Demos | Teaching notebooks / projects |
| --- | --- | --- | --- |
| Core / diagrams | [00_core](learn/intro/00_core.ipynb) | [`demos/core/`](demos/core/) | |
| Blocks | [01_blocks](learn/intro/01_blocks.ipynb) | [`demos/blocks/`](demos/blocks/) | |
| Dynamics catalog | [02_dynamics](learn/intro/02_dynamics.ipynb) | [`demos/dynamics/`](demos/dynamics/) | `from minilink import Pendulum, CartPole, …` |
| Control | [03_control](learn/intro/03_control.ipynb) | [`demos/control/`](demos/control/), [`demos/robotic/`](demos/robotic/) | [articulated_robot_eom](learn/teaching/articulated_robot_eom.ipynb) |
| Analysis | [04_analysis](learn/intro/04_analysis.ipynb) | [`demos/analysis/`](demos/analysis/) | [frequency_domain_tools](learn/teaching/frequency_domain_tools.ipynb) (Bode, margins, Nyquist, root locus, step response) |
| Simulation | [05_simulation](learn/intro/05_simulation.ipynb) | [`demos/graphical/`](demos/graphical/) | |
| Hybrid / step | [06_hybrid](learn/intro/06_hybrid.ipynb) | [`demos/hybrid/`](demos/hybrid/), [`demos/mpc/`](demos/mpc/) | [`projects/mpc/`](projects/mpc/) (spatial MPC stack, dual-rate `mpc_dual_rate.py`), [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Compile / autodiff | [07_compile](learn/intro/07_compile.ipynb), [showcase_jax](learn/intro/showcase_jax.ipynb) | [`demos/compile/`](demos/compile/) | [cartpole_rollout_gradients](learn/teaching/cartpole_rollout_gradients.ipynb) |
| Optimization | [08_optimization](learn/intro/08_optimization.ipynb) | [`demos/optimization/`](demos/optimization/) | |
| Planning | [09_planning](learn/intro/09_planning.ipynb) | [`demos/planning/`](demos/planning/) (`trajopt/`, `rrt/`, `value_iteration/`) | [cost+VI](learn/teaching/pendulum_swing_up_cost_function_vi.ipynb), [VI vs LQR](learn/teaching/pendulum_swing_up_vi_vs_lqr.ipynb), [grid world DP](learn/teaching/grid_world_exact_dp.ipynb); [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Graphical | [10_graphical](learn/intro/10_graphical.ipynb) | [`demos/graphical/`](demos/graphical/), [`demos/realtime/`](demos/realtime/) (keyboard game mode: `game_cartpole.py`) | |
| RL | — | — | [VI vs LQR vs PPO](learn/teaching/pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb), [drone_ppo_learn_to_fly](learn/teaching/drone_ppo_learn_to_fly.ipynb) |
| Research lane | — | [`experimental/`](experimental/) (`c_export/`, `engine/`, `symbolic/`, `robotic/` UR5) | not swept; `c_export` runs in the JAX regression job; C export demos: `experimental/c_export/c_export_proportional.py`, `c_export.py` |
| Solver benchmarks | — | [benchmark](tooling/notebooks/benchmark.ipynb) | Uses repo-root `benchmarks/` |

Run demos from the repo root, e.g.
`PYTHONPATH=. python examples/demos/core/readme_examples.py`.

## CI smoke

Notebook smoke covers `learn/intro/`, `learn/teaching/` (except long
notebooks with `"smoke": false` — UR5 EoM, DP grids, PPO), and
`tooling/notebooks/` (not `projects/` or `experimental/`). Locally:

```bash
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
```

The nightly sweep runs every script under `demos/`; the JAX regression job runs
the flagship manifest (`tests/demo_checks/flagship_manifest.json`).
