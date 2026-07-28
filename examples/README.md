# Examples

Human-facing runnable learning and experiments. Automated contracts and
runners live under [`tests/`](../tests/) and [`benchmarks/`](../benchmarks/).

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

**Demo naming:** no `demo_` prefix. Keep the topic in the stem when the name
would otherwise be too generic — e.g. `mpc/mpc_car_minimal.py`,
`diagrams/diagram_shortcuts.py`. Plant or scenario words stay when they
discriminate (`sliding_mode_pendulum.py`).

**Demo style:** top-level constants + sequential calls (open and run). No
`def main()` / `if __name__` guard; thin helpers are fine. Do **not** add a
demos file whose only job is “run this catalog plant” — use the plant
module’s ≤10-line `__main__` instead (AGENTS rule 12).

Do **not** mix `.ipynb` and `.py` in the same per-module leaf folder — keep
format trees separate (`learn/` vs `demos/`, or `tooling/notebooks` vs
`tooling/scripts`). Projects may ship both inside one named bundle.

See also [learn/README.md](learn/README.md).

## By topic

| Topic | Intro notebook | Canonical demos | Notes |
| --- | --- | --- | --- |
| Core / diagrams | [intro/00_core](learn/intro/00_core.ipynb) | [`demos/diagrams/`](demos/diagrams/) | |
| Blocks | [intro/01_blocks](learn/intro/01_blocks.ipynb) | [`demos/blocks/`](demos/blocks/) | |
| Dynamics catalog | [intro/02_dynamics](learn/intro/02_dynamics.ipynb) | — | `from minilink.catalog import …` |
| Control | [intro/03_control](learn/intro/03_control.ipynb) | [`demos/control/`](demos/control/) | |
| Analysis | [intro/04_analysis](learn/intro/04_analysis.ipynb) | [`demos/analysis/`](demos/analysis/) | |
| Simulation | [intro/05_simulation](learn/intro/05_simulation.ipynb) | [`demos/plots/`](demos/plots/), [`demos/animation/`](demos/animation/) | |
| Hybrid / step | [intro/06_hybrid](learn/intro/06_hybrid.ipynb) | [`demos/hybrid/`](demos/hybrid/), [`demos/step/`](demos/step/), [`demos/mpc/`](demos/mpc/) | Topic notebook: [`teaching/topics/mpc`](learn/teaching/topics/mpc.ipynb); scenarios: [`projects/mpc/`](projects/mpc/) |
| Compile | [intro/07_compile](learn/intro/07_compile.ipynb) | [`demos/diagrams/`](demos/diagrams/) | |
| Optimization | [intro/08_optimization](learn/intro/08_optimization.ipynb) | [`demos/optimization/`](demos/optimization/) | |
| Planning / trajopt | [intro/09_planning](learn/intro/09_planning.ipynb) | [`demos/planning/`](demos/planning/), [`demos/trajopt/`](demos/trajopt/) | [`projects/car_trajopt/`](projects/car_trajopt/), [`projects/pathtracking/`](projects/pathtracking/) |
| Graphical | [intro/10_graphical](learn/intro/10_graphical.ipynb) | [`demos/plots/`](demos/plots/), [`demos/animation/`](demos/animation/), [`demos/realtime/`](demos/realtime/) | |
| Robotic | — | [`demos/robotic/`](demos/robotic/) | UR5 variants under `sandbox/robotic/` |
| State-space / LQR | — | [`demos/statespace/`](demos/statespace/) | |
| Identification | — | [`demos/identification/`](demos/identification/) | |
| C export | — | [`demos/interfaces/`](demos/interfaces/) | |
| Physics engine | — | [`sandbox/engine/`](sandbox/engine/) | Non-core |
| Symbolic | — | [`sandbox/symbolic/`](sandbox/symbolic/) | Non-core |
| Solver benchmarks | — | [tooling/notebooks/benchmark.ipynb](tooling/notebooks/benchmark.ipynb) | Uses repo-root `benchmarks/` |

Run demos from the repo root, e.g.
`PYTHONPATH=. python examples/demos/plots/plot_readme.py`.
