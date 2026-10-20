# Core consolidation: what leaves the core

**Status:** analysis and recommendations, 2026-10-20 — ideas to review, nothing decided.  
**Lane:** maintainer-owned (every item removes or moves a user-importable name, so each is
an ask under AGENTS.md; the inventory and the tiers are the agent's analysis).  
**Rung:** the convergence pass the roadmap does not yet name; it fits between the GMC714
audit (step G1 of [TODO.md](TODO.md)) and the freeze candidate (ROADMAP §5.4), because the
v0.9 freeze should cover a smaller surface than today's.  
**Companion:** [repository-structure.md](repository-structure.md) decides *where* things
live; this document decides *what* stays in the library at all.

## 1. Why: divergence, then convergence

The last two releases were a divergence phase on purpose: build every tool of the toolbox,
from LQR to PPO through trajectory optimization, to see the whole picture. It worked, and it
left a library of **70.7k lines**, a root prelude of **162 names**, **50 catalog plants**
on that prelude, **76 demo scripts**, **42 notebooks** and **28k lines of tests**. The v0.9
freeze (ROADMAP §5.4) promises every teaching-surface name stability for three years. The
smaller that surface is when the freeze lands, the cheaper the promise.

This pass asks one question of every module and public name: **does anything on the
teaching path need it?** What nothing needs moves out of the core, to the experimental
tier or to the example that motivated it, and nothing is deleted from history. The point is
not the line count. It is that every name left on the surface has a lesson behind it, and
that the maintainer can hold the whole core in his head.

## 2. The bar: what "core" means

A module or name is core when it meets one of these, in this order:

1. **A live course uses it.** It appears in a GRO860 or GRO501 notebook, or the GMC714
   checklist (ROADMAP §4.3) names it.
2. **The tutorial path uses it.** It appears in one of the twelve tutorial chapters or the
   showcases (RULES 3.7: the canonical intro surfaces).
3. **It is a textbook object or the one verb of a band.** The constitution's nouns
   (`System`, `Trajectory`, `Set`, `Distribution`, `CostFunction`, `PlanningProblem`,
   `PlanningSolution`, `MathematicalProgram`) and one verb per tool band on any `System`.
4. **It is the substrate of a core tool** (the compile backends, the evaluator, the
   Riccati solver, the signal wiring), even when no notebook names it.

Everything else is provisional by the two-lane contract (ROADMAP §2) and is a candidate
here. The lane contract already has an entry rule ("nothing enters without a demo or
notebook, a both-backends test, a docstring"); it has no exit rule. §6 proposes one.

## 3. The evidence

The inventory counts, for every module and every facade name, the files that use it in
seven places: the course notebooks (crs), the topic notebooks (top), the tutorial (tut), the
demos (dem), the research examples (res: `projects/` and `experimental/`), the tests (tst),
and the library itself outside the defining module (lib). A name with zero in the first
three columns has no lesson behind it. The two scripts and their full tables are in the
session's scratchpad; the numbers quoted below come from them.

### 3.1 By package

| Package | Lines | Teaching use (course · tutorial) | TRL (ROADMAP §3) | Verdict |
| --- | --- | --- | --- | --- |
| `core/` + `core/compile/` | 8.8k + 4.1k | everything | 7 / 4 | core; the hot spots split under V1, nothing moves |
| `dynamics/` | 9.1k (abstraction 1.1k, catalog 8.0k) | 14 of 50 plants in a notebook; 10 more in a demo only; 2 in a research script only; **24 in nothing** | 7 | the catalog is the largest trim: §4.1 |
| `planning/` | 15.6k | problems, DP, trajopt, RL planner, comparison, evaluation in courses | 5–6 | `search/` (2.5k) has no course; `tabular.py` (0.6k) and `approximation.py` (0.2k) have no user: §4 |
| `graphical/` | 12.9k | diagrams, trajectories, animation, phase plane, control-law plots in courses | 5 | the racecar skin + mesh readers (2.5k, 580 KB of assets), two exporters, cameras and the pygame renderer have no user: §4 |
| `analysis/` | 4.3k | all frequency, structural, time and discretization tools in GRO501 | 6 | `lyapunov.py` (0.9k) is provisional research with one demo: §4.1 |
| `control/` + `control/mpc/` | 2.8k + 1.7k | design verbs, PID family, impedance, model-based, neural, MPC in courses | 6 / 4 | `geometric.py` (PurePursuit) has no user; GMC714 names it: §4.2 |
| `simulation/` + `realtime/` | 2.9k + 1.4k | `Simulator`, the hybrid loop | 7 / 2 | realtime is TRL 2 with three demos: §4.2 |
| `blocks/` | 1.5k | routing, sources, TF, filters, nonlinear in notebooks | 5 | keep whole: the block palette is the textbook's table and the whole package is small |
| `optimization/` | 1.4k | via trajopt; V1 builds on it | 5 | keep |
| `estimation/` | 0.3k | GRO501 | 6 | keep |
| `interfaces/` | 0.5k | the two SB3 twin notebooks (retire at v1.0, S49) | 4 | `ProblemEnv` / `to_gymnasium` have no user: §4.1; the rest follows S49 |
| `catalog/` alias | 0.1k | the short import | — | keep until the frozen-surface decision (V3) |
| `identification/` | 5 lines | nothing | 2 | remove until the fitting verb lands (C4) |
| `experimental/` | 2.5k | the C-export showcase | 1 | already the research tier; the destination of this pass |

### 3.2 The root prelude

Of the 162 names on `from minilink import …`, **39 have no use in any notebook, demo or
research example** (only tests), and **about 65 appear in no notebook at all**. By category:
50 plants, 23 control names, 23 blocks, 22 planning names, 19 analysis verbs, 16 core
objects, 5 estimation, 2 optimization, 2 simulation. A course notebook imports about 60
distinct names in total; the tutorial about 90. The prelude is twice the surface the
lessons use.

### 3.3 The hybrid sibling

The sampled loop is one seam the constitution leaves open (invariant 1) and the roadmap
freezes until the hybrid-as-a-System decision (S31, v0.9): `StepSystem`,
`StepDiagramSystem`, `Computer`, `HybridDiagram`, `HybridSimulator`, `ZOHHold`, the step
compiler and rollouts, the hybrid diagram exporters and MPC on top. Together about
**5.0k lines**. It stays: the tutorial has a chapter on it, MPC is its exemplar (RULES 3.7),
and the GRO501 racecar notebook closes a loop with it. But it is the largest provisional
block in the wheel, and §4.3 names what inside it has no user.

## 4. The recommendations, in tiers

Tier 1 is safe today. Tier 2 waits for the robotics-course audit this term (G1), which may
claim items. Tier 3 stays and shrinks. Tier 4 is the core that remains. "Move" means `git
mv` into `minilink/experimental/` (or into the example folder that motivated the code), off
the prelude and the band facade, off the docs site, with its tests following (the
experimental tier is tested today: `test_symbolic.py`, `test_engine_jax.py`). "Delete"
means git history keeps it.

### 4.1 Tier 1 — move now: nothing teaching-facing depends on it

| What | Lines | Evidence | Where to |
| --- | --- | --- | --- |
| `analysis/lyapunov.py`: `region_of_attraction`, `LyapunovCertificate`, `VerificationReport`, `plot_region_of_attraction` | 931 | provisional research (TRL 4); one demo, the showcase's §11, one experimental RL script; no course | `experimental/lyapunov.py`; returns as a `Field` verb when fields land (A1) and G1 claims Lyapunov for GMC714. The showcase section is student-facing: ask |
| `planning/reinforcement_learning/tabular.py`: `TabularLearningPlanner`, `QLearning`, `SARSA`, `MonteCarloControl`, `EpsilonGreedy`, `UCB` | 565 | no demo, no notebook (GRO860's grid world is dynamic programming); fails the lane's own entry rule | `experimental/tabular_learning.py` until a lesson needs Q-learning |
| `planning/policy_synthesis/approximation.py`: `LinearApproximator`, `QuadraticFeatures`, `RadialBasisFeatures` | 199 | no user anywhere; the GRO860 cost-to-go lesson defines its own approximators in a cell | `experimental/`; adopting it in the lesson is the maintainer's pedagogical call, not a gap |
| Catalog plants with no notebook or demo (25 of 50; the boat has one experimental script): `Plane2D`, `Plane3D` (`aerial/plane.py`, with `plane_skin_3d` and `plane_airframe_3d`), the two `LongitudinalFrontWheelDriveCar…` (`vehicles/propulsion.py`), `QuarterCarOnRoughTerrain` (`vehicles/suspension.py`), `MountainCar`, `Boat2D` / `Boat2DWithCurrent` (`marine/boat.py`, one experimental script), `Acrobot`, `TwoIndependentPendulums`, `RotatingCartPole`, `UnderactuatedRotatingCartPole`, `ConstantSpeedHelicopterTunnel`, `Drone2DWithSideThruster`, `SpeedControlledDrone2D`, `OneLinkManipulator`, `ThreeLinkManipulator3D`, `FloatingSingleMass`, `FloatingTwoMass`, `FloatingThreeMass`, `TwoMass`, `ThreeMass`, `SimpleIntegrator`, `TripleIntegrator`, `VanderPol` | ≈ 2.8k (1.8k in whole files, 1.0k as classes inside kept files) | each costs a prelude name, a both-backends contract test and a docs page, for no lesson | `experimental/catalog/<domain>/`; the boat and its skin (`graphical/catalog/boat_skin.py`, 240 lines) to `examples/experimental/boat/`. The vehicle ladder's four rungs, `tires.py`, the pendulum family, `Drone2D`, `Rocket`, the arms a demo uses, `Lorenz`, `ThreeBodyProblem` and `DoubleIntegrator` stay |
| `graphical/diagrams/mermaid.py`, `hybrid_mermaid.py`, `export.py` (`export_diagram_topology`, `export_hybrid_topology`) | 256 | no example, three tests; Graphviz is the one diagram path the notebooks use | delete |
| `graphical/animation/camera.py` (`fixed_camera`, `follow_frame_camera`) | 157 | no user in the library, the examples or the tests | delete |
| `graphical/animation/renderers/pygame_renderer.py` (`PygameRenderer`) | 335 | reachable only through `make_renderer`; no example, no test; the realtime loop does not use it | delete, or `experimental/` with the realtime line if Tier 2 keeps that |
| `interfaces/`: `ProblemEnv`, `to_gymnasium` | part of `gymnasium.py` (452) | no user; `Sys2Gym` and `SB3Controller` are what the SB3 twin notebooks import | trim now; the rest leaves with the twins at v1.0 (S49) |
| `identification/` | 5 | a stub | delete until C4 lands `fitting.py` |
| Prelude trim: every name on `from minilink import …` that no notebook uses leaves the root prelude and stays on its band facade (`minilink.catalog`, `.control`, `.planning`, …) | ≈ 60 names | §3.2; the lane contract says the prelude is "the names students meet" | the root prelude is the teaching set; the bands are the full index. A deprecation note per name is not needed before v0.9 (ROADMAP §2 "Breaking changes so far" is the ledger) |

Tier 1 removes about **5.5k lines** from the wheel and about **60 names** from the prelude,
and touches no course notebook.

### 4.2 Tier 2 — move unless the GMC714 audit claims it (decide at G1, this term)

| What | Lines | Evidence | Recommendation |
| --- | --- | --- | --- |
| `planning/search/` (RRT, RRT*, the extenders, Dubins steering, the live plot, 463 lines of plotting) | 2.5k | provisional (TRL 5); tutorial chapter 9 and three demos; no course and not on the GMC714 table; the only planner with no lesson | move to `experimental/search/` with its demos under `examples/experimental/rrt/`; the tutorial section is student-facing: ask. Keep if G1 adds sampling-based planning to the robotics course |
| `control/geometric.py` (`PurePursuit`) + `test_geometric_control.py` | 186 + 400 | no notebook, demo or project uses it (the path-tracking projects carry their own laws); GMC714 lists `control.geometric` under nonlinear control | keep only if G1 writes the lesson; else move to `examples/projects/pathtracking/` where its users are |
| `simulation/realtime/` (`RealtimeSimulator`, the I/O contract, `PygameInput`, `LiveSignalPlotOutput`) and the `game()` facade | 1.4k | TRL 2, provisional; three demos, one showcase cell; no course; the ledger's own next step is "architectural review (v1.0)" | keep in the wheel as the one interactive path, off the docs site and the prelude (it already is); the first thing to cut at v0.9 if no course has used it by then. Moving it to `experimental/` would make a `System` facade import the experimental tier, which the dependency law forbids: then drop `game()` too |
| `graphical/catalog/racecar_skin.py` + `graphical/meshes.py` (STL / COLLADA readers) + `graphical/assets/` | 1.3k + 0.6k + 580 KB | one plant's 3-D skin, used by one GRO501 notebook (deep import), two demos and one project; the mesh readers serve only it and the boat | move to the racecar plant folder (`examples/demos/udes_racecar/`, or the chapter folder B2 of the structure plan gives it) **between terms**, since the GRO501 notebook imports it by path; the generic `skins.py` / `shapes.py` stay |
| `control/robotic.py`: `ModelJointImpedance` (no user), the five-law family | 546 | GMC714 names `control.robotic` and `control.impedance`; `TaskKinematic` / `TaskKinematicNullspace` have one demo each | G1 decides which laws the arm lesson teaches; the rest moves with the audit's ruling |

### 4.3 Tier 3 — keep, but freeze and shrink

- **The hybrid sibling** (§3.3, 5.0k lines): keep; no new features before the
  hybrid-as-a-System decision (S31, already the rule). Inside it, the mermaid exporters go
  with Tier 1, and `control/mpc/utilities.py` and `viz.py` (489 lines) are internal
  machinery that should lose their band names (`mpc_warm_start_guess`,
  `mpc_default_computer_x0`, `mpc_plans_from_rollout`: one demo or none).
- **Spatial geometry** (`planning/spatial/`, 2.2k): keep and move to `core/geometry/` as
  planned (A3), cutting `spatial/plotting.py` (800 lines) to what the demos call
  (`plot_track`: one demo) and `workspace_fields.py` (one demo) with it.
- **The RL planner** (2.4k): PPO is the course algorithm; REINFORCE and the actor-critic
  are the textbook ladder to it (244 lines together) and SAC (200) has an open step (S50).
  Keep all four; keep the algorithms as options on one planner, never four planners.
- **The `System` facade** (`core/facades.py`, 1.2k lines, 32 verbs): the courses call
  fourteen of them. `get_diagram` (no user; `_repr_svg_` does the job), `plot_input_output_map`
  (one demo) and `render` (one course cell) are the candidates; `compute_cost` and
  `plot_cost` stay (four and five course notebooks).
- **The control laws** no notebook uses but a design verb returns
  (`TimeVaryingStateFeedbackController`, `TrajectoryFeedbackController`,
  `OutputReferenceController`): keep as the outputs of `trajectory_lqr` and friends, off the
  prelude.
- **The `core/` hot spots** (`composition.py` 1.3k with 60 functions, `facades.py` 1.2k,
  `wiring.py` 0.9k): no move; the split rides the differentiable-problem work (V1) that
  rewrites their neighbours.

### 4.4 Tier 4 — the core that remains

What the three courses and the tutorial actually stand on, by band:

| Band | The core after the trim |
| --- | --- |
| framework | `System`, `DynamicSystem`, `StepSystem`, `DiagramSystem`, `StepDiagramSystem`, the operators and `closed_loop` / `feedback`, `compile`, `Trajectory`, `BoxSet` / `BallSet`, `Gaussian` / `Uniform`, `QuadraticCost` / `TimeCost` / `CostFunction`, `Sphere`, the geometry package (A3) |
| system libraries | the block palette as is; **25 plants**: the pendulums (3), the cart-poles (2), `DoublePendulum`, `DoubleIntegrator`, `Lorenz`, `SingleMass`, `Drone2D`, `Rocket`, `ThreeBodyProblem`, the three planar arms and the UR5, the vehicle ladder (4) with tires, the three racecars, `HolonomicMobileRobot`, `KinematicCar`; the control laws of the four families (PID, state feedback, impedance, model-based) and the neural policy; the observer and its designs |
| tools | `Simulator` and the hybrid loop; the analysis verbs as they are, minus Lyapunov; `PlanningProblem` and its stochastic sibling, DP, trajectory optimization, the RL planner, `compare`, `MonteCarloEvaluator`; `MathematicalProgram` / `Optimizer`; MPC as the hybrid exemplar |
| presentation | diagrams (Graphviz), time signals, phase plane, control-law maps, animation with three renderers, the generic skins; the gymnasium bridge until v1.0 |

Roughly **65k lines** after Tier 1 and **59k** if Tier 2 moves too, instead of 70.7k, and
a prelude of about **100 names** instead of 162, every one with a notebook behind it.

## 5. What the trim buys

- **The freeze covers less.** v0.9 pins the export list (V4); each name it pins is a
  three-year promise. Sixty fewer names on the prelude is sixty fewer promises.
- **The gates run less.** Every catalog plant runs the both-backends contract test; every
  module is a docs page the Docs workflow builds; `test_geometric_control.py` alone is 400
  lines for a law nothing calls.
- **The wheel is the library.** 580 KB of racecar meshes and a one-plant skin leave the
  published package; the research tier stays repo-only as the lane contract says.
- **The tree says what matters.** A reader of `minilink/analysis/` sees the GRO501 toolbox,
  not a research certificate beside it; `planning/` is the three planner families with a
  lesson each.

## 6. The exit rule (proposed amendment to the lane contract)

ROADMAP §2's teaching-surface rule says what it takes to enter. Add the symmetric rule:

> A teaching-surface or provisional name that no notebook, course checklist or tutorial
> chapter uses for two consecutive releases moves to `experimental/` (or to the example
> that motivated it) at the next convergence pass; the pass is a step of every minor
> release's close-out.

This makes convergence a habit rather than an event: divergence between releases, one
trim at each close-out, and the surface the freeze pins is the surface the lessons use.

## 7. Mechanics of one move (so each item is a small, reviewable commit)

1. `git mv minilink/<band>/<module>.py minilink/experimental/<module>.py` (history kept).
2. Drop the names from the root prelude and the band facade; `test_teaching_surface.py`
   and `test_public_imports.py` pin the new list.
3. Drop the `docs/api/*.rst` entry; the research tier is not on the API site.
4. Move the demo to `examples/experimental/<topic>/`; move or trim the tests, keeping them
   running (the experimental tier is tested, not gated on both backends).
5. The TRL ledger row (ROADMAP §3) moves to "research lane"; one dated record under
   `docs/reviews/` lists the pass's rulings; the "Breaking changes so far" ledger in
   ROADMAP §2 gets one line per release.
6. Before any move that touches a course notebook's import (the racecar skin): between
   terms only, with the citing documents updated in the same change.

## 8. The asks

1. **Tier 1 as a block**, or item by item. The two student-facing pieces inside it are the
   showcase's Lyapunov section and the tutorial's RRT section (Tier 2): they move with the
   code or are rewritten on the kept tools.
2. **The plant rule** behind the 23-plant list: a plant stays when a notebook or a course
   checklist uses it, or a demo is its lesson; otherwise it moves. Any plant the maintainer
   wants kept on sentiment or on a planned lesson stays, named here.
3. **Tier 2's date**: the GMC714 audit (G1) rules on RRT, PurePursuit, the robotic laws and
   Lyapunov; the realtime line is decided at v0.9.
4. **Delete or move** for the dead code (the mermaid exporters, the cameras, the pygame
   renderer, the stub): deletion is the recommendation, git is the archive.
5. **The exit rule** of §6 into ROADMAP §2 (the lane contract is the maintainer's).
6. **The prelude trim's form**: silent before v0.9 (the ledger line) or a one-release
   `DeprecationWarning` shim per name, which the deprecation policy (V4) only requires from
   v1.0.

When the maintainer rules, the rulings go to a dated record under
[../reviews/](../reviews/), the moves to [TODO.md](TODO.md) as steps, and this document is
deleted once they land (one fact, one home: [README.md](README.md)).
