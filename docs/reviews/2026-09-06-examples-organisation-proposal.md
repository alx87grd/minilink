# Examples and lane organisation — proposal (2026-09-06)

Survey of `examples/` (63 scripts, 21 notebooks, 19 demo folders) and the
research-lane placement in `minilink/`, with one recommended layout. Nothing
here is applied; each numbered item is a ruling for the maintainer.

## 1. Stable vs experimental in the package

Today the research lane is a *list of paths*: `symbolic/`, `dynamics/engines/`,
`interfaces/c_export.py` are quarantined by DESIGN and excluded from the wheel
one by one in `pyproject.toml`; the provisional bands (`control/mpc`,
`simulation/realtime`, `planning/spatial`, hybrid) ship and are labelled only in
the TRL table.

**Recommended:** one `minilink/experimental/` package for the quarantine tier,
the way JAX marks maturity in the import path.

```
minilink/experimental/
  symbolic/      ← minilink/symbolic/
  engines/       ← minilink/dynamics/engines/
  c_export.py    ← minilink/interfaces/c_export.py
```

- The import line tells the student what they are holding
  (`from minilink.experimental.symbolic ...`).
- The wheel rule becomes one glob (`experimental/**`); `docs/api/experimental.rst`
  already carries that name.
- Provisional bands stay where they are: they ship, teaching notebooks use them,
  and their status belongs in the TRL table, not in a path.
- Cost: three deep-import paths change. The quarantine rule already says nothing
  imports them, so no shims — the seven research-lane demos and two tests that
  do are moved in the same commit.

Alternative: keep the paths and only label — no churn, but "what is stable" stays
a table to look up rather than a path to read.

## 2. Examples tree

Five kinds of material already exist; the folders mostly match them. The changes
below make the mapping one-to-one and remove the one-file folders.

```
examples/
  learn/intro/      tutorials of the library, 00 … 10 + two showcases   (stable, smoked)
  learn/teaching/   subject lessons                                       (stable, smoked or marked long)
  demos/<chapter>/  canonical single-file scripts, one folder per tutorial chapter  (stable, swept nightly)
  projects/<name>/  multi-file research experiments, each with its own README     (research lane, not CI)
  experimental/     single-file research scripts + scratch/  (rename of sandbox/)  (research lane)
  tooling/          benches (notebooks only — drop the empty scripts/)
```

Keeping `learn/intro` and `learn/teaching` is deliberate for this term: the
Colab badges on `main` point at those paths and students have them open. A
rename to `tutorials/` + `teaching/` (one level flatter) is a term-boundary
change.

### Demo folders: 19 → 13, keyed to the tutorial chapters

| Folder (new) | Chapter | Contents |
| --- | --- | --- |
| `core/` | 00 | `diagrams/` minus `diagram_compiling` |
| `blocks/` | 01 | unchanged |
| `dynamics/` | 02 | `astro/three_body`, `plots/lorenz_attractor` — fills the empty "Dynamics catalog" row |
| `control/` | 03 | `control/` (3 classical) + `statespace/cartpole_lqr` |
| `analysis/` | 04 | unchanged (7) |
| `hybrid/` | 06 | `hybrid/` + `step/` (rename `step/diagram_*` → `step_*`) |
| `compile/` | 07 | the JAX-claim demos: `diagram_compiling`, `identification/params_gradient`, `control/pid_autotuning_jax`, `control/neural_controller_jax`, `trajopt/trajopt_cartpole_rollout_gradients` |
| `optimization/` | 08 | unchanged (2) |
| `planning/` | 09 | `rrt/`, `value_iteration/`, **`trajopt/`** (the two remaining trajopt scripts) |
| `graphical/` | 05 + 10 | `plots/` (4) + one merged animation demo |
| `realtime/` | 10 | the three keyboard games incl. `animation/game_bicycle` (removes the special case in `run_all_demos.py`) |
| `robotic/` | teaching | unchanged (5), plus the two UR5 impedance scripts from `sandbox/robotic/` if they run headless |
| `mpc/` | teaching | unchanged (3) |

`interfaces/c_export*.py` → `experimental/c_export/` (research lane; nightly + JAX
regression job keep running them from the manifest).

### Wrongly categorised or duplicated (each a ruling)

1. `demos/animation/animation_backends.py` + `animation_native_comparison.py` — same pendulum, both compare renderers → one `graphical/animation_renderers.py`.
2. `demos/diagrams/diagram_noise_ports.py` and `demos/plots/plot_internal_signals.py` share the same five-block noise diagram; the second only shows the `signals=` API → let it use `ImpedanceController @ Pendulum` and keep the noise wiring in one place.
3. `demos/plots/plot_readme.py` is the README figure generator, not a lesson → `tooling/` or `core/readme_examples.py`.
4. `demos/hybrid/smc_pendulum_rate.py` says "continuous vs hybrid" but the continuous path is commented out → rename `sampled_smc_pendulum.py` and drop the dead block, or restore the compare.
5. `learn/teaching/articulated_robot_eom.ipynb` and `projects/ur5_dynamics/eom_comparison.ipynb` carry the same title; the project copy is the older deep-import version → delete the project copy.
6. `sandbox/scratch/cartpole_rollout_gradients.ipynb` is the declared "teaching twin" of the rollout-gradients demo → promote to `learn/teaching/` (and smoke it) or delete.
7. `learn/teaching/mpc.ipynb` ("Spatial MPC full stack") imports 17 research-lane modules and MPC is not in GRO860 → `projects/mpc/mpc_spatial_stack.ipynb`, or keep as the GMC714-style capstone.
8. `learn/intro/06_hybrid.ipynb` has two code cells and the only one is an MPC loop on `BicycleDynRate` → a real chapter (StepSystem, `%`, `Computer @ plant` with a P controller), MPC left to `teaching/mpc`.
9. `pendulum_swing_up_vi_vs_lqr.ipynb` ⊂ `..._vs_lqr_vs_ppo.ipynb` (13 vs 19 cells) → keep both as the lecture progression, or one notebook with the PPO section optional.
10. `projects/pathtracking/`: `vehicle.py` (25 KB) ×3 identical, `path_generator.py` ×3, `allocation.py` ×2, `servos.py` ×2 diverged → `projects/pathtracking/common/` shared by the three runs, or archive `bicycle_los` v1.
11. `examples/tooling/scripts/` is empty; `sandbox/scratch/` holds one notebook.
12. `vi_quadratic.py` / `vi_minimum_time.py` (same plant, cost differs) — two short files read better than one with a switch; keep.

### Mechanics

`git mv` + one path-rewrite script over README / AGENTS / DESIGN / examples
README / `flagship_manifest.json` / `notebook_overrides.json` /
`teaching_import_allowlist.txt` / notebook markdown, then sweep + smoke. About
one agent-hour for the demo regroup; one more for §1.
