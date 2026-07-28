# Examples

Human-facing runnable learning and experiments. Automated contracts and
runners live under [`tests/`](../tests/) and [`benchmarks/`](../benchmarks/).

Start in Colab:
[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/intro/showcase_minilink.ipynb)
(Colab opens **notebook files** only — use the per-notebook badges below. Folder browsing is on GitHub.)

## Layout

```
examples/
  learn/           # curriculum notebooks
    intro/         # learn minilink (library + showcase)
    teaching/
      topics/      # generic domain lessons
      courses/     # class / workshop packs
  demos/           # canonical single-file scripts by topic
  projects/        # multi-file named experiments
  tooling/         # developer exploration (notebooks/ and scripts/)
  sandbox/         # non-core / WIP / personal (+ scratch/)
```

| Folder | When to use |
| --- | --- |
| [`learn/intro/`](learn/intro/) | Learn **minilink** — numbered API notebooks + showcase |
| [`learn/teaching/topics/`](learn/teaching/topics/) | Learn a **subject** (reusable domain notebooks) |
| [`learn/teaching/courses/`](learn/teaching/courses/) | Syllabus-tied labs / problems (e.g. UdeS) |
| [`demos/<topic>/`](demos/) | Canonical single-file feature demo (incl. pedagogical compares) |
| [`projects/<name>/`](projects/) | Multi-file experiment (`run_demo.py` + helpers ± notebook) |
| [`tooling/`](tooling/) | Dev matrices / benches — `notebooks/` and/or `scripts/` |
| [`sandbox/<topic>/`](sandbox/) | Non-core, scenario sprawl, versioned WIP |
| [`sandbox/scratch/`](sandbox/scratch/) | Personal quick checks; not README/CI |

**Promotion:** `sandbox/scratch/` → `sandbox/<topic>/` → `demos/<topic>/` (if
single-file) **or** `projects/<name>/` (if multi-file) → optional
`learn/teaching/topics/` twin → README / `intro` only if core-tool canonical.

**Compare rule:** method/API side-by-sides that *are* the lesson stay in
`demos/` (e.g. continuous vs hybrid SMC, RRT vs RRT*). Mission ladders and
`*_v2*` live under `sandbox/<topic>/` or `projects/`.

**Demo naming:** no `demo_` prefix. Keep the topic in the stem when needed —
e.g. `mpc/mpc_car_minimal.py`. **Demo style:** top-level constants + sequential
calls (open and run); no `def main()`. Do not mix `.ipynb` and `.py` in the same
leaf folder (except inside one named project). Tooling may use both
`tooling/notebooks/` and `tooling/scripts/`.

## Open in Colab

Colab requires a `/blob/…/*.ipynb` path (not a GitHub `/tree/` folder). Browse
folders on GitHub, then open a notebook badge.

| Folder (GitHub) |
| --- |
| [`learn/`](https://github.com/alx87grd/minilink/tree/main/examples/learn) |
| [`learn/intro/`](https://github.com/alx87grd/minilink/tree/main/examples/learn/intro) |
| [`learn/teaching/topics/`](https://github.com/alx87grd/minilink/tree/main/examples/learn/teaching/topics) |
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

### Teaching topics

| Notebook | Colab |
| --- | --- |
| [mpc](learn/teaching/topics/mpc.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/topics/mpc.ipynb) |
| [articulated_robot_eom](learn/teaching/topics/articulated_robot_eom.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/learn/teaching/topics/articulated_robot_eom.ipynb) |

### Tooling

| Notebook | Colab |
| --- | --- |
| [benchmark](tooling/notebooks/benchmark.ipynb) | [![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/alx87grd/minilink/blob/main/examples/tooling/notebooks/benchmark.ipynb) |

**Colab tip:** open a badge → **File → Save a copy in Drive** → run from the top.
The first code cell clones the repo and installs `meshcat` when needed. Locally,
use the `minilink` conda env (see root [README](../README.md#install)).

## By topic

| Topic | Intro notebook | Canonical demos | Notes |
| --- | --- | --- | --- |
| Core / diagrams | [00_core](learn/intro/00_core.ipynb) | [`demos/diagrams/`](demos/diagrams/) | |
| Blocks | [01_blocks](learn/intro/01_blocks.ipynb) | [`demos/blocks/`](demos/blocks/) | |
| Dynamics catalog | [02_dynamics](learn/intro/02_dynamics.ipynb) | — | `from minilink.catalog import …` |
| Control | [03_control](learn/intro/03_control.ipynb) | [`demos/control/`](demos/control/) | |
| Analysis | [04_analysis](learn/intro/04_analysis.ipynb) | [`demos/analysis/`](demos/analysis/) | |
| Simulation | [05_simulation](learn/intro/05_simulation.ipynb) | [`demos/plots/`](demos/plots/), [`demos/animation/`](demos/animation/) | |
| Hybrid / step | [06_hybrid](learn/intro/06_hybrid.ipynb) | [`demos/hybrid/`](demos/hybrid/), [`demos/step/`](demos/step/), [`demos/mpc/`](demos/mpc/) | Topic: [mpc](learn/teaching/topics/mpc.ipynb); scenarios: [`projects/mpc/`](projects/mpc/) |
| Compile | [07_compile](learn/intro/07_compile.ipynb) | [`demos/diagrams/`](demos/diagrams/) | |
| Optimization | [08_optimization](learn/intro/08_optimization.ipynb) | [`demos/optimization/`](demos/optimization/) | |
| Planning / trajopt | [09_planning](learn/intro/09_planning.ipynb) | [`demos/planning/`](demos/planning/), [`demos/trajopt/`](demos/trajopt/) | [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Graphical | [10_graphical](learn/intro/10_graphical.ipynb) | [`demos/plots/`](demos/plots/), [`demos/animation/`](demos/animation/), [`demos/realtime/`](demos/realtime/) | |
| Robotic | [articulated_robot_eom](learn/teaching/topics/articulated_robot_eom.ipynb) | [`demos/robotic/`](demos/robotic/) | UR5 variants under `sandbox/robotic/` |
| State-space / LQR | — | [`demos/statespace/`](demos/statespace/) | |
| Identification | — | [`demos/identification/`](demos/identification/) | |
| C export | — | [`demos/interfaces/`](demos/interfaces/) | |
| Physics engine | — | [`sandbox/engine/`](sandbox/engine/) | Non-core |
| Symbolic | — | [`sandbox/symbolic/`](sandbox/symbolic/) | Non-core |
| Solver benchmarks | — | [benchmark](tooling/notebooks/benchmark.ipynb) | Uses repo-root `benchmarks/` |

Run demos from the repo root, e.g.
`PYTHONPATH=. python examples/demos/plots/plot_readme.py`.

## CI smoke

Notebook smoke covers `learn/intro/`, `learn/teaching/topics/`, and
`tooling/notebooks/` (not `courses/`, `projects/`, or `sandbox/`). Locally:

```bash
MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py
```
