# `dev-fable` → `main` pre-PR review — 2026-09-09

Review of `dev-fable` (`1b94bb1`, 80 commits since the merge base
`c7cd13e`, 318 files) before the pull request on `main`. The 2026-09-06 punch
list ([2026-09-06-dev-fable-pre-merge.md](2026-09-06-dev-fable-pre-merge.md))
is fully landed; this file is the new action list.

## 1. What was verified (this session, conda env `minilink`, Apple M4 Max)

| Check | Result |
| --- | --- |
| `ruff check .` / `ruff format --check .` | clean (435 files) |
| `pytest tests/unittest` | 1063 passed, 2 skipped (twice, last on `1b94bb1` minus the final `camera_fit` rename, whose 97 graphics tests were rerun green) |
| Notebook smoke, default suite (`run_notebook_checks.py`) | 16 passed, 0 failed, 0 skipped: `00`–`10` intro, both showcases, `cartpole_rollout_gradients`, `frequency_domain_tools`, `grid_world_exact_dp` |
| DP notebooks CI skips (`smoke: false`), run explicitly | `pendulum_swing_up_cost_function_vi` pass, `pendulum_swing_up_vi_vs_lqr` pass |
| Demo sweep (`run_all_demos.py --timeout 120 --continue-on-error`) | 59 passed, 0 failed, 3 skipped of 62 (skips: the realtime game demos, which need a window) |
| Regression gates (`run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes solve_s,nlp_s,speedup`) | pass; every speed metric within 0.8–1.1× of the macOS reference baseline |
| PPO notebooks (`pendulum_swing_up_vi_vs_lqr_vs_ppo`, 200k–1M steps; `drone_ppo_learn_to_fly`), `articulated_robot_eom` | NOT RUN (long; see §4) |
| README examples (`examples/demos/core/readme_examples.py`) | runs (19 s) |
| Punch list 2026-09-06, "fix before merge" 1–4 | all in the code: `success=feasible` (`planner.py:287`), `Transcription` docstring first, nightly installs `ipopt`, `compile_step_diagram` validates shapes |

GitHub Actions status was not read (`gh` is not authenticated on this machine).

## 2. Merge mechanics

- **One conflict is certain.** `origin/main` has `c7e96c5` ("add grid-world
  exact DP teaching notebook", 21 cells); `dev-fable` has the same notebook
  with 12 more cells (the minilink DP section, `1c18f2a`). `git merge-tree`
  reports an add/add conflict on `examples/learn/teaching/grid_world_exact_dp.ipynb`.
  All 21 `main` cells are verbatim in the `dev-fable` version, so the
  resolution is "take `dev-fable`". Nothing else conflicts; `dev-alex` has no
  commits that `dev-fable` lacks.
- Merging with a merge commit keeps the 47 archive tags and the review
  trail meaningful; a squash would lose the per-step history the docs cite.

## 3. Standing work found on the branch

**Stale text (fix before PR, no code):**

1. ROADMAP §4.1 gate 2 says the default grid is 1 001 points; the code's
   `DEFAULT_N_STEPS` is 10 001 (ruled fine, 2026-09-07). Fix the number.
2. ROADMAP §4.1 table cites `demos/statespace/` and `demos/trajopt/`; those
   folders are now `demos/control/` and `demos/planning/trajopt/`.
3. Three notebooks link to the old `showcase/` folder: `00_core` (3 links),
   `07_compile` (8), `cartpole_rollout_gradients` (1). The targets are
   `showcase_minilink.ipynb` / `showcase_jax.ipynb` in `learn/intro/`.
4. `docs/plans/control-plots.md` and `docs/plans/derivatives-facade.md` are
   self-marked "implemented". AGENTS.md says finished plan docs are deleted;
   their decision records already live in DESIGN. Delete both and drop their
   rows from `docs/plans/README.md`. `gro501-classical-control.md` (wave 1
   partly landed) and `planning-pipeline-architecture.md` (partial) stay.

**Half-built, honestly labelled (keep, no action):**

- `minilink/estimation/` and `minilink/identification/` are docstring-only
  placeholder packages (ROADMAP TRL 1–2). They ship in the wheel as empty
  homes; harmless, but a `pip install` user sees two packages with nothing in
  them. Either accept (they document the plan) or exclude them from the wheel
  until content lands.
- Five `TODO` markers in the library: three `User Architectural Review`
  (`experimental/engines/ancf_tire_jax.py`, `simulation/realtime/io.py`,
  `simulation/realtime/simulator.py`), one plotting note in `blocks/sources.py`,
  one performance note in `trajectory_optimization/parametric_evaluator.py`.
  All are in provisional or research-lane modules and match their ROADMAP TRL.
- `KinodynamicExtender` still ignores `problem.params.system` (TODO §5
  hardening row); the new default extender inherits that.

**Research lane is already separated:** `minilink/experimental/` (c_export,
engines, symbolic) is excluded from the wheel; `examples/experimental/`
(c_export, engine, robotic UR5, symbolic) and `examples/projects/` are outside
the release contract and not CI-checked. Nothing in the teaching tree needs to
move. The two `c_export` demos run in CI through the flagship manifest.

**Deleted or moved on the branch (all accounted for):** the 21 deleted paths
are renames into the chapter-keyed demo folders (`analysis_bode` →
`analysis_frequency`, `animation_backends` → `graphical/animation_renderers`,
`plot_readme` → `core/readme_examples`, `smc_pendulum_rate` →
`hybrid/sampled_smc_pendulum`, `trajopt_cartpole_collocation_jax` →
`planning/trajopt/`, `ur5_dynamics/eom_comparison.ipynb` →
`teaching/articulated_robot_eom.ipynb`, …), the two finished plan docs, the
`jax_vehicles.py` twins retired by the `xp` sweep (ruled 2026-09-06), and the
pathtracking `mpc_v1` / `bicycle_los_v2` helper duplicates folded into
`pathtracking/common/`.

## 4. Not verified, and why

- The two PPO notebooks train for 200 000 and 1 000 000 timesteps and the
  drone one similar; each is tens of minutes on CPU. They are `smoke: false`
  by design. Run them once locally (or on the nightly) before the term's RL
  week; not a merge blocker.
- `articulated_robot_eom` (SymPy derivation) is `smoke: false` for length;
  same treatment.
- `examples/projects/` scripts are research lane and were not run.

## 5. Cleanup plan before the PR

Ordered; the first block is docs-only and takes minutes.

1. **Docs fixes** (agent, no code): ROADMAP gate number and demo paths;
   the twelve stale `showcase/` links in the three notebooks; delete the two
   implemented plan docs and their index rows; add this file's verdict line
   to ROADMAP §5 status.
2. ~~Run what is pending~~ — demo sweep and regression gates done (§1).
3. **Maintainer rulings**: keep or wheel-exclude the two placeholder packages;
   merge commit vs squash; who runs the PPO notebooks.
4. **PR**: open `dev-fable` → `main`, resolve the one notebook conflict by
   taking `dev-fable`, let CI (`test` + `regression` + docs) run once on the
   PR, merge with a merge commit.
