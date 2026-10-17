# Docs consistency review: the maintainer's rulings (2026-10-12)

**Date:** 2026-10-12
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** A consistency pass over the governance and planning docs (CONSTITUTION, RULES,
AGENTS, CLAUDE, DESIGN, README, install, ROADMAP, the workboard and the plan docs) against
the code. It was first done on a stale local `dev` (branch `dev-local`, commits `edfa3326`
to `4329b253`, based on `03f6e79f`) while GitHub's `dev` had already slimmed ROADMAP and
turned TODO into the workboard (2026-10-11). The content was then re-applied onto `dev` in
its new structure; `dev-local`'s restructure of ROADMAP and TODO was not.
**Applied:** the same day. This file is the decision record: one entry per ruling, with the
question that raised it and where the ruling now lives.

---

## Install and packaging

- **Conda is the default install; pip is the Basic alternative.** The docs disagreed on
  which install leads. Conda from `environment.yml` stays the default supported install and
  README leads with it; `pip install minilink` is the Basic alternative; `pip install
  "minilink[full]"` installs the libraries only (notebooks run on the conda env). Lives in:
  `install.md` (the one owner of install), README, ROADMAP §5.4 V6, TODO V6 (what remains
  is the pinned Colab cell and the `showcase_jax` C-export question).
- **The requirements files are deleted.** Four `requirements*.txt` files restated
  `environment.yml` and `pyproject.toml` and had drifted. Lives in: `install.md`,
  `tests/README.md`.
- **The lanes match the code.** ROADMAP §2 said the provisional bands stay out of the wheel;
  the wheel ships them. The provisional bands (hybrid, MPC, realtime) ship in the wheel with
  no stability promise; only `minilink/experimental/` and the research examples stay out; the
  spatial names the planning facade exports (`Scene`, `bind`, `ReferenceTrack`,
  `car_outline`, `plot_track`, …) are provisional teaching names until A3 moves them to
  `core/geometry/`. Lives in: ROADMAP §2 and §3, the wheel comment in `pyproject.toml`.

## Scope and rungs

- **Course notebooks are standalone teaching designs, never roadmap items.** The roadmap
  creates features; the maintainer designs the course material on top of them. Lives in:
  AGENTS.md (student-facing material is maintainer-owned).
- **V1, the differentiable closed-loop cost, moves to v0.3.** It sat after v1.0 although the
  prototype already ran (a PID gain's gradient through `PID() @ Pendulum()` matched finite
  differences to 8 digits). params → closed loop → RK4 run → `J` → the mean over stochastic
  rollouts, fully differentiable, one standard entry point; API and architecture not
  decided. Lives in: ROADMAP §5.3, TODO §4 (V1), [cost-params.md](../plans/cost-params.md).
- **A behavior-cloning pipeline, after v1.0.** A game mode on `RealtimeSimulator` records
  human demonstrations as trajectory datasets, a planner fits a policy to them, and the
  cloned controller is compared with the demonstrator on the plant. Lives in: ROADMAP §3
  (realtime row) and §5.5, TODO §5 (after v1.0).
- **The DC motor leaves the GRO501 modelling gate.** It is not on the critical path; the
  longitudinal-vehicle plant (`LongitudinalFrontWheelDriveCarWithTorqueInput`) meets the
  gate. Lives in: ROADMAP §4.2; TODO S71 keeps the DC motor with no conditional.
- **The realtime architectural review is a v1.0 step**, not a wave-D hardening row. Lives
  in: ROADMAP §3 and §5.5, TODO §5 (v1.0).
- **The estimation follow-ups are v0.9 syllabus holes.** EKF, time-varying and discrete
  Kalman, online parameter estimators. Lives in: ROADMAP §3 and §5.4, TODO §5.
- **P6's row sits in v0.9.** The z tier stays held for GRO501; its row moves to the v0.9
  syllabus holes. Lives in: TODO §5, `gro501-classical-control.md` (plan doc retired 2026-10-17),
  [pyro-port-remaining.md](../plans/pyro-port-remaining.md) (the discrete-time row is no
  longer a drop).
- **OPW-1–OPW-5 get their own D3 row.** They change behaviour, and T5 is byte-identical.
  Lives in: TODO §3 (D3), [optimizer-parametric-wiring.md](../plans/optimizer-parametric-wiring.md),
  the plans index.
- **A3 absorbs S57.** One id for the workspace-geometry step. Lives in: TODO A3, ROADMAP
  §5.3.
- **S49 names its four notebooks.** `drone_ppo_sb3` and
  `pendulum_value_iteration_vs_lqr_vs_ppo_sb3` retire at v1.0; `particle_gymnasium_sb3` and
  `gymnasium_interface` are decided then. Lives in: TODO S49, ROADMAP §1 and §5.5.
- **The frozen names wait for V4.** `QuadraticCost.from_system(xbar=, ubar=)`, `cost.xbar`,
  `TimeCost`'s `xbar`, `plot_cost2go` and `JaxMechanicalSystem` keep their names to v1.0;
  the renames (`x_bar`, `u_bar`, `plot_cost_to_go`, the twin retired into the `xp` base)
  are on V4's list, applied with shims under the deprecation policy, with no alias before.
  This closes the ROADMAP §6 `JaxMechanicalSystem` entry. Lives in: TODO V4, A4 and D2,
  ROADMAP §5.4.

## Architecture

- **CONSTITUTION stays the target design.** `Field` stays a core noun; the parametric
  `MathematicalProgram` is the same NLP shape; the hybrid seam (`Computer`, `HybridDiagram`
  not `System`s) is stated as an open ROADMAP §6 decision, not a licence. Lives in:
  CONSTITUTION §2, ROADMAP §6 (S31).
- **The dependency-law exceptions are declared.** The imports that cross the band law today
  are listed once. Lives in: DESIGN §3.
- **The wiring conveniences beside the frozen grammar** (`closed_loop`, `closed_loop_qdq`,
  `autowire`) are reviewed with the composition work. That work is now the
  automation-by-convention plan, which removes `closed_loop_qdq` and keeps `closed_loop` and
  `autowire` on standard names. Lives in:
  [automation-by-convention.md](../plans/automation-by-convention.md).
- **The run-state caches are decided with S31.** RULES 2.4 allows only `self.traj`, while
  `StepSystem.rollout` and `HybridDiagram.last_result` cache a run too. Lives in: TODO S31,
  ROADMAP §6 (open until S31).

## Rules and docs ownership

- **RULES 5.22: every math line an agent edits follows the textbook rule**; a whole function
  the edit does not touch waits for a picked step. Lives in: RULES 5.22.
- **RULES §5 owns the textbook rule**, with a new 5.24 for timeless docstrings and comments.
  Lives in: RULES §5; AGENTS points to it.
- **One owner per rule.** RULES keeps a review question and a pointer for the laws
  CONSTITUTION and DESIGN own; `tests/README.md` owns the commands and the CI jobs;
  `install.md` owns install; DESIGN keeps two "Not adopted" notes and states the
  minimal-realization default of `pzmap`, `root_locus` and `transfer_function`. Lives in:
  RULES, `tests/README.md`, `install.md`, DESIGN.
- **CLAUDE.md: read CONSTITUTION before any interaction**, as AGENTS already said. Lives in:
  CLAUDE.md.
- **Plan-doc local ids carry a prefix.** Bare local ids collided with workboard ids (the
  GRO501 audit's G1 against the GMC714 audit G1, randomness.md's A5 against workboard A5).
  `GRO-` marks the GRO501 audit's findings, `RD-` randomness.md's rules, rulings and
  amendments, `FD-` the foundations review's findings and comments; workboard ids stay bare.
  Lives in: the plans index, gro501-classical-control.md, randomness.md, fields.md and every
  citation in ROADMAP, TODO and the plans.

## CI and housekeeping

- **The nightly sweep runs everything.** Every notebook with `--all` (the `smoke: false`
  ones included; the two `nightly: false` ones, `manipulator_eom` and
  `particle_gymnasium_sb3`, only by name), Stable-Baselines3 and torch, symbolic, Ipopt and
  a `pytest` step; the `regression` job installs symbolic. Lives in:
  `.github/workflows/nightly.yml` and `test.yml`, `tests/README.md`; the S68 clause
  "every teaching notebook executed nightly" is done.
- **The 12 benchmark shims are deleted**; `run_study.py` presets cover them. Lives in:
  `benchmarks/README.md`; the TODO D2 row is gone.
- **README examples resynced.** `examples/demos/core/readme_examples.py` runs README's
  snippets exactly as written. Lives in: README, that demo.
- **Quick fixes:** the stale CI branch filters removed; `furo`, `build` and `twine` in
  `environment.yml`; README's links absolute for the PyPI page; the PyPI summary is the
  tagline; the heading line count; timeless docstrings (RULES 5.24). Lives in: the
  workflows, `environment.yml`, README, `pyproject.toml`, RULES.

## Process

- **Fetch and check the remote branch before any work.** This pass was lost once to a stale
  local branch. Lives in: AGENTS.md ("Before you start"), CLAUDE.md, the SessionStart hook in
  `.claude/settings.json`.
