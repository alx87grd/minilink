# Examples

Human-facing runnable learning and experiments. Automated contracts and
runners live under [`tests/`](../tests/) and [`benchmarks/`](../benchmarks/).

## Layout

```
examples/
  notebooks/       # showcase · intro · applications · tooling
  scripts/         # canonical feature demos by topic
  experimental/    # non-core, scenario sprawl, versioned WIP (+ scratch/)
  projects/        # multi-file named experiments
```

| Folder | When to use |
| --- | --- |
| [`notebooks/intro/`](notebooks/intro/) / [`showcase/`](notebooks/showcase/) | Core API teaching / marketing tour |
| [`scripts/<topic>/`](scripts/) | Canonical single-file feature demo (incl. pedagogical compares) |
| [`notebooks/applications/`](notebooks/applications/) | Longer application narrative notebook |
| [`experimental/<topic>/`](experimental/) | Shared non-core features, scenario matrices, versioned WIP |
| [`experimental/scratch/`](experimental/scratch/) | Personal quick checks; not README/CI |
| [`projects/<name>/`](projects/) | Multi-file experiment (`run_demo.py` + helpers) |

**Promotion:** `experimental/scratch/` → `experimental/<topic>/` (if worth sharing) →
`scripts/<topic>/` (if canonical) → `projects/` (if multi-file) → optional
applications notebook.

**Compare rule:** method/API side-by-sides that *are* the lesson stay in
`scripts/` (e.g. continuous vs hybrid SMC, RRT vs RRT*). Mission ladders and
`*_v2*` live under `experimental/<topic>/` or `projects/`.

**Script naming:** no `demo_` prefix. Keep the topic in the stem when the
name would otherwise be too generic — e.g. `mpc/mpc_minimal.py`,
`diagrams/diagram_shortcuts.py` (not bare `minimal.py` / `shortcuts.py`).
Plant or scenario words stay when they discriminate (`sliding_mode_pendulum.py`).

**Script style:** top-level constants + sequential calls (open and run). No
`def main()` / `if __name__` guard; thin helpers are fine. Do **not** add a
scripts file whose only job is “run this catalog plant” — use the plant
module’s ≤10-line `__main__` instead (AGENTS rule 12).

Do **not** mix `.ipynb` and `.py` in the same per-module folder — keep format
trees separate and cross-link below. There is no `scripts/tooling/`; use
[`notebooks/tooling/`](notebooks/tooling/), [`tests/demo_checks/`](../tests/demo_checks/),
and [`benchmarks/`](../benchmarks/).

See also [notebooks/README.md](notebooks/README.md).

## By topic

| Topic | Intro notebook | Canonical scripts | Notes |
| --- | --- | --- | --- |
| Core / diagrams | [intro/00_core](notebooks/intro/00_core.ipynb) | [`scripts/diagrams/`](scripts/diagrams/) | |
| Blocks | [intro/01_blocks](notebooks/intro/01_blocks.ipynb) | [`scripts/blocks/`](scripts/blocks/) | |
| Dynamics catalog | [intro/02_dynamics](notebooks/intro/02_dynamics.ipynb) | — | `from minilink.catalog import …` |
| Control | [intro/03_control](notebooks/intro/03_control.ipynb) | [`scripts/control/`](scripts/control/) | |
| Analysis | [intro/04_analysis](notebooks/intro/04_analysis.ipynb) | [`scripts/analysis/`](scripts/analysis/) | |
| Simulation | [intro/05_simulation](notebooks/intro/05_simulation.ipynb) | [`scripts/plots/`](scripts/plots/), [`scripts/animation/`](scripts/animation/) | |
| Hybrid / step | [intro/06_hybrid](notebooks/intro/06_hybrid.ipynb) | [`scripts/hybrid/`](scripts/hybrid/), [`scripts/step/`](scripts/step/), [`scripts/mpc/`](scripts/mpc/) | MPC minimal + dual-rate; scenarios: [`projects/mpc/`](projects/mpc/) |
| Compile | [intro/07_compile](notebooks/intro/07_compile.ipynb) | [`scripts/diagrams/`](scripts/diagrams/) | |
| Optimization | [intro/08_optimization](notebooks/intro/08_optimization.ipynb) | [`scripts/optimization/`](scripts/optimization/) | |
| Planning / trajopt | [intro/09_planning](notebooks/intro/09_planning.ipynb) | [`scripts/planning/`](scripts/planning/), [`scripts/trajopt/`](scripts/trajopt/) | Car / LOS: [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Graphical | [intro/10_graphical](notebooks/intro/10_graphical.ipynb) | [`scripts/plots/`](scripts/plots/), [`scripts/animation/`](scripts/animation/), [`scripts/realtime/`](scripts/realtime/) | |
| Robotic | — | [`scripts/robotic/`](scripts/robotic/) | UR5 variants under `experimental/robotic/` |
| State-space / LQR | — | [`scripts/statespace/`](scripts/statespace/) | |
| Identification | — | [`scripts/identification/`](scripts/identification/) | |
| C export | — | [`scripts/interfaces/`](scripts/interfaces/) | |
| Physics engine | — | [`experimental/engine/`](experimental/engine/) | Non-core |
| Symbolic | — | [`experimental/symbolic/`](experimental/symbolic/) | Non-core |
| Solver benchmarks | — | [notebooks/tooling/benchmark.ipynb](notebooks/tooling/benchmark.ipynb) | Uses repo-root `benchmarks/` |

Run scripts from the repo root, e.g.
`PYTHONPATH=. python examples/scripts/plots/plot_readme.py`.
