# The convergence inventory: public names, plants and demos (2026-10-23)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** The first step of the November clean-up (wave K of [TODO.md](../plans/TODO.md) §4,
step K1): the one table the library is pruned from. It updates the usage inventory of
[core-consolidation.md](../plans/core-consolidation.md) §3 (same lens, recounted) and carries
that document's §4–§6 verdicts into a proposal per row. It also drafts the two rename tables
the maintainer asked for before the GMC714 term: the short catalog names (A6, RULES 2.6) and
the GMC714 names frozen by January (G3, R49–R50). Every proposal here is a proposal: nothing
leaves or is renamed without a ruling (RULES 7.1; the asks are K2, K3, K4, K8 and A6/G3).
**Snapshot:** commit `121dfe00` on `dev` (2026-10-23), before the experimental-tier commits
that landed the same afternoon (the prototype scripts' move, K9-1 `5cb67611`, and the tier's
tests leaving the merge gate, K9-2/K9-3 `eeb00b9a`). The experimental demos are named by
their path at that commit, `examples/experimental/…`; K9-1 moves them to a top-level
`experimental/…`, and K9-2 moves `test_symbolic.py` and `test_engine_jax.py` to
`tests/experimental/` (they are still counted under "tst" here).

## Summary

- **259 public names** on 13 facades: the root prelude (`minilink`, 162 names, of which 50
  plants), `catalog` (50), `core` (20), `blocks` (24), `control` (29, one a subpackage),
  `control.mpc` (6), `analysis` (34), `estimation` (5), `simulation` (7), `optimization` (3),
  `planning` (46), `interfaces` (4), `graphical.catalog` (34). Plus 11 deep-import names the
  G3 table needs (`Manipulator`, `MechanicalSystem`, the MPC classes, …): **270 rows**.
- **Maturity of the 209 facade names that are not plants:** 125 final, 55 prototype, 29 dead
  (no caller outside their module and the tests). Of the 162 root names, **73 appear in no
  notebook** and **34 are used by tests only** (core-consolidation §3 counted 39; this count
  also credits the nine facade verbs called as methods, `sys.linearize()` and kin).
- **Plants:** all 50 pass the both-backends contract test (67 passed, run on the snapshot).
  14 are imported by a course notebook or the tutorial, 10 by a demo only, **24 by nothing but
  tests** (the same split as core-consolidation §3). Proposal: **14 on the root** (the course
  set, one slot left under R2's 15), **13 more on `minilink.catalog`**, **18 to
  `experimental/catalog/`**, **5 merged or removed** (three parameter-only twins, a second
  `Integrator`, a second holonomic robot).
- **Demos:** 74 in `examples/demos/`, 17 in `examples/experimental/`. **6 strict twin
  pairs** (same feature, same plant) in the first, 5 in the second, and one script that
  repeats a topic notebook. Proposal for `examples/demos/`: 59 keep, 6 merge, 3 remove, 2
  follow their plant to `experimental/`, 4 asks.
- **Top candidates for removal** (no caller outside the tests): `ProblemParameters` (R41
  retired it), `ProblemEnv` and `to_gymnasium`, the camera helpers `fixed_camera` and
  `follow_frame_camera`, the skin helpers `merge_skins`, `debug_state_skin` and `Replay`,
  `Sampler`; `NeuralNetwork` merged into `MLP`; `Lag` and `RateLimiter` (pinned on the teaching
  surface, so an ask); the plants `FloatingSingleMass`, `FloatingTwoMass`, `FloatingThreeMass`,
  `SimpleIntegrator`, `SpeedControlledDrone2D`; the demos `step_fibonacci`, `rrt_star_live`,
  `vi_pendulum_swingup`.
- **Root prelude after the proposals:** 162 − 36 plants − 20 names (§1, "off the root") + 4
  G3 names = **110** (core-consolidation §6.4 estimated about 115).
- **Open questions** (§5): the 15th root plant; the skin twins and noise-port twins on the
  root; A6's option A (drop display names, rename six long classes) or B (also drop the
  `Controller` suffix, which G3's names carry); **the diagram-id rule turns `Drone2D` into
  `drone2_d` once display names drop**; MPC's one name stays a factory or becomes a class, and
  which loop syntax is frozen (`mpc @ plant` in every demo, `mpc % dt @ plant` in the G3 row).

## How the counts were made

Three scripts, run on a `git archive` of the snapshot with every optional dependency installed
(NumPy, SciPy, Matplotlib, JAX, Plotly, MeshCat, pygame, SymPy, gymnasium); two runs give
byte-identical JSON. They live in the session scratchpad
(`/tmp/claude-0/-home-user-minilink/c2480aba-0f3a-5640-bb6c-80a34f9a4b05/scratchpad/k1/`:
`count_names.py` for the names, `plants.py` and `plants_table.py` for §2, `demos.py` for §3,
`display_names.py` and `a6_table.py` for §4.1, `deep_imports.py` for the deep-import check,
`make_tables.py` for §1), not in the repo. What they count:

1. **The registry.** Each facade is imported and its `__all__` walked; an exported object is
   keyed by identity, so a name re-exported by several facades (`minilink.Pendulum` is
   `minilink.catalog.Pendulum`) is one row; its home is `obj.__module__`.
2. **Callers, one per file.** Every `.py` file and every notebook's code cells (`%` and `!`
   lines dropped) is parsed with `ast`. A file uses a name when it imports it
   (`from minilink.x import Name`, relative imports in the library resolved, the object found
   by import and `getattr`, so deep and facade imports count alike) or reaches it by attribute
   from a module alias (`import minilink as ml; ml.control.PID`). A System facade verb that
   shares a tool's name (nine: `linearize`, `jacobian`, `find_equilibrium`,
   `transfer_function`, `plot_bode`, `plot_pzmap`, `plot_root_locus`, `plot_step_response`,
   `animate_modal`) counts when called as a method. The library column skips the defining
   module and the facades' `__init__.py`.
3. **Docs.** `README.md`, `examples/README.md`, `docs/dev/DESIGN.md`, `docs/index.rst` and
   `docs/api/*.rst`: a file mentions a name when the word appears in a code span or a code
   block. "D" marks a DESIGN mention.
4. **Not counted:** a name reached through `getattr` on a string (the teaching-surface test,
   the catalog contract test, which is reported in §2 instead), a class built by a factory
   (`JointImpedance(plant)` returns a `ModelJointImpedance`) or returned by a tool (`Command`,
   `StructuralResult`): those read "dead" and their proposal says why.

**Columns.** lib = `minilink/` outside `experimental/`; tst = `tests/`; demo =
`examples/demos/`; tut = `examples/tutorial/`; top = the topic notebooks
(`examples/teaching/topics/`); crs = the course notebooks (`examples/teaching/courses/`); proj =
`examples/projects/`; exp = `minilink/experimental/` and `examples/experimental/`; bch =
`benchmarks/`; docs = files mentioning it. R = on the root prelude; S = pinned by
`tests/unittest/test_teaching_surface.py`.

**Maturity**, by K1's rule applied literally: **dead** — no caller in the library, the examples
or the benchmarks; **prototype** — `t` no test file uses it, `c` no course notebook and no
DESIGN mention, `m` its module carries `TODO: User Architectural Review` (two modules:
`simulation/realtime/simulator.py` and `io.py`); **final** otherwise. The `c` test is strict
(the tutorial does not count), so many sound teaching names read prototype; the counts beside
it say how much.

**Proposals** follow core-consolidation §4–§6 (spine, pillar, content, side) and the rulings
R1–R5 (no exit rule: the seeds stay without a clock). "Off the root" is the prelude trim
(R5, silent): the name stays on its band facade.

## 1. Every public name, by band (the plants are in §2)

A name two bands export appears once, under the first: `Gaussian`, `Uniform`, `Particles`
and `Sampler` are on both `minilink.core` and `minilink.planning` and sit under `core`.

### `minilink.core` (20)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `BallSet` | RS | `core.sets` | 3 | 4 | 2 | 1 | · | · | · | · | 1 | 1 | prototype (c) | keep |
| `BoxInputSet` | RS | `core.sets` | 6 | 2 | · | · | · | · | · | · | · | · | prototype (c) | keep on the root: a foundation object |
| `BoxSet` | RS | `core.sets` | 7 | 4 | 3 | · | · | · | 1 | · | 1 | 1D | final | keep on the root: a foundation object |
| `CostFunction` | RS | `core.costs` | 4 | 3 | 5 | 2 | 1 | 4 | · | 3 | · | 1D | final | keep |
| `DiagramSystem` | RS | `core.diagram` | 12 | 14 | 10 | 3 | 2 | 3 | 4 | 2 | 3 | 1D | final | keep |
| `Distribution` |  | `core.distributions` | 2 | · | · | · | · | · | · | · | · | 1D | prototype (t) | keep: the base class |
| `DynamicSystem` | RS | `core.system` | 28 | 19 | 4 | 2 | · | 3 | 7 | 2 | 2 | 2D | final | keep |
| `Gaussian` | RS | `core.distributions` | 1 | 4 | 2 | 1 | 1 | 2 | · | 1 | · | 1D | final | keep |
| `InputSet` |  | `core.sets` | 2 | 1 | · | · | · | · | · | · | · | · | prototype (c) | keep: the base class |
| `Particles` |  | `core.distributions` | 2 | 3 | · | · | · | · | · | · | · | 1D | final | keep |
| `QuadraticCost` | RS | `core.costs` | 1 | 16 | 13 | 3 | 1 | 8 | 12 | 2 | 7 | 2D | final | keep |
| `Sampler` |  | `core.distributions` | · | 1 | · | · | · | · | · | · | · | 1D | dead | remove — no caller; the draws work (v0.4) can bring it back |
| `Set` |  | `core.sets` | 6 | 2 | · | · | · | · | · | · | · | 1D | final | keep: the base class |
| `SingletonSet` |  | `core.sets` | 7 | 4 | 1 | · | · | · | · | · | · | · | prototype (c) | keep |
| `StepDiagramSystem` | RS | `core.diagram` | 6 | 5 | 3 | 1 | · | · | 1 | · | 1 | 1D | final | keep |
| `StepSystem` | RS | `core.system` | 10 | 10 | 4 | 1 | · | · | · | · | 1 | 1D | final | keep |
| `System` | RS | `core.system` | 28 | 9 | 3 | 1 | · | 3 | 8 | 2 | 1 | 2D | final | keep |
| `TimeCost` | RS | `core.costs` | · | 3 | 1 | · | · | 2 | · | · | · | · | final | keep |
| `Trajectory` | RS | `core.trajectory` | 24 | 13 | 3 | · | 1 | · | 2 | 3 | 2 | 1D | final | keep |
| `Uniform` | RS | `core.distributions` | 1 | 7 | 2 | 3 | 1 | 3 | · | 3 | · | 1D | final | keep |

### `minilink.blocks` (24)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `DeadZone` | RS | `blocks.nonlinear` | · | 1 | 1 | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `Demux` | RS | `blocks.routing` | 1 | 3 | · | · | · | 1 | · | · | · | 1D | final | keep |
| `Error` | RS | `blocks.routing` | 1 | 2 | · | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `Gain` | RS | `blocks.routing` | 1 | 8 | 2 | 2 | 1 | · | · | · | 1 | 1D | final | keep |
| `Integrator` | RS | `blocks.basic` | 1 | 12 | 4 | 1 | · | · | · | · | · | 1D | final | keep |
| `Lag` | RS | `blocks.transfer_function` | · | 1 | · | · | · | · | · | · | · | 1D | dead | ask: remove, or a demo with `Lead` — pinned on the teaching surface |
| `Lead` | RS | `blocks.transfer_function` | · | 2 | 1 | 1 | 1 | · | · | · | · | 1D | final | keep |
| `LowPassFilter` | RS | `blocks.filters` | · | 4 | 1 | · | · | · | · | 1 | · | · | prototype (c) | keep; off the root (no notebook imports it) |
| `MLP` | RS | `blocks.neural` | 3 | · | · | · | · | · | · | · | · | 1D | prototype (t) | keep; off the root (no notebook imports it) |
| `Mux` | RS | `blocks.routing` | · | 2 | 2 | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `NeuralNetwork` |  | `blocks.neural` | · | 1 | 1 | · | · | · | · | · | · | 1D | final | merge into `MLP` — one hidden layer is `MLP` with one width; one demo |
| `NotchFilter` | RS | `blocks.filters` | · | 1 | 1 | · | · | · | · | · | · | · | prototype (c) | keep; off the root (no notebook imports it) |
| `RateLimiter` | RS | `blocks.nonlinear` | · | 2 | · | · | · | · | · | · | · | · | dead | ask: remove, or a demo — pinned on the teaching surface |
| `Relay` | RS | `blocks.nonlinear` | · | 1 | 1 | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `Saturation` | RS | `blocks.nonlinear` | · | 6 | 3 | 1 | · | · | · | · | · | 1D | final | keep |
| `Sine` | R | `blocks.sources` | · | 1 | 1 | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `Source` | RS | `blocks.sources` | · | 2 | 3 | · | · | · | 1 | · | 1 | · | prototype (c) | keep; off the root (no notebook imports it) |
| `Step` | RS | `blocks.sources` | · | 10 | 16 | 5 | 2 | · | · | 4 | 1 | 1D | final | keep |
| `Sum` | RS | `blocks.routing` | 2 | 3 | 1 | 1 | · | · | · | · | · | 1D | final | keep |
| `TrajectorySource` | RS | `blocks.sources` | 1 | 3 | 1 | · | · | 1 | · | · | · | 1D | final | keep |
| `TransferFunction` | RS | `blocks.transfer_function` | 3 | 7 | · | 1 | · | 1 | · | · | · | 1D | final | keep |
| `Washout` | RS | `blocks.filters` | · | 1 | 1 | · | · | · | · | · | · | · | prototype (c) | keep; off the root (no notebook imports it) |
| `WhiteNoise` | RS | `blocks.sources` | · | 5 | 4 | 2 | 1 | 1 | · | 1 | · | 1D | final | keep |
| `ZOHHold` | RS | `blocks.step` | · | 1 | 1 | · | · | · | · | · | · | · | prototype (c) | keep; off the root (no notebook imports it) |

### `minilink.control` (28)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `angle_features` |  | `control.neural` | · | · | · | 1 | · | · | · | 1 | · | 1D | prototype (t) | keep |
| `ComputedTorqueController` | RS | `control.modelbased` | · | 3 | 2 | 1 | · | · | · | · | · | 2D | final | keep |
| `ImpedanceController` | RS | `control.impedance` | 1 | 8 | 5 | 4 | · | · | · | 1 | · | 2D | final | keep |
| `ImpedanceIntegralController` | RS | `control.impedance` | · | 2 | 1 | 1 | · | · | · | · | · | 1D | final | keep |
| `JointImpedance` | RS | `control.robotic` | · | 1 | 1 | 2 | · | · | · | 1 | · | · | prototype (c) | keep |
| `lqg` | RS | `control.design` | · | 2 | · | 1 | 1 | · | · | · | · | 2D | final | keep |
| `lqr` | RS | `control.design` | 1 | 9 | 3 | 2 | 1 | 2 | · | · | · | 3D | final | keep |
| `lqr_finite_horizon` | RS | `control.design` | · | 1 | · | · | · | 1 | · | · | · | 1D | final | keep |
| `lqr_gain` | RS | `control.design` | · | 3 | · | 1 | · | 1 | · | 1 | · | 1D | final | keep |
| `lqr_gain_schedule` |  | `control.design` | 1 | 2 | · | · | · | 1 | · | · | · | 1D | final | keep |
| `ModelJointImpedance` |  | `control.robotic` | · | 1 | · | · | · | · | · | · | · | 1D | dead | keep; to the root (G3) — reached through `JointImpedance(plant)`; owes a demo |
| `NeuralPolicyController` | RS | `control.neural` | 1 | 4 | · | 1 | · | · | · | 1 | · | 2D | final | keep |
| `OutputReferenceController` | R | `control.state` | · | 2 | · | · | 1 | · | · | · | · | 1D | final | keep |
| `P` | RS | `control.siso` | 1 | 10 | 4 | 1 | · | · | · | 1 | · | 1D | final | keep |
| `PD` | RS | `control.siso` | · | 2 | 1 | · | · | · | · | · | · | 1D | final | keep on the root — with `P`, `PI`, `PID` |
| `PI` | RS | `control.siso` | · | 3 | 1 | · | · | · | · | · | · | 1D | final | keep on the root — with `P`, `PD`, `PID` (the classical family) |
| `PID` | RS | `control.siso` | · | 7 | 4 | 1 | 2 | 2 | · | 1 | · | 2D | final | keep |
| `place` | RS | `control.design` | · | 1 | 1 | · | 1 | 1 | · | · | · | 2D | final | keep |
| `place_gain` | RS | `control.design` | 1 | 2 | · | · | · | 1 | · | · | · | 1D | final | keep |
| `PurePursuit` | RS | `control.geometric` | · | 1 | · | · | · | · | · | · | · | · | dead | keep: a pillar's seed (R4, core-consolidation §6.3), off the root — the vehicle baseline; no lesson yet |
| `SlidingModeController` | RS | `control.modelbased` | · | 4 | 2 | 1 | · | · | · | · | · | 2D | final | keep |
| `StateFeedbackController` | RS | `control.state` | 2 | 12 | · | 1 | · | · | · | · | · | 2D | final | keep |
| `TaskImpedance` | RS | `control.robotic` | · | 1 | 2 | · | · | · | · | 1 | · | 1D | final | keep on the root (G3) — a GMC714 lab law |
| `TaskKinematic` |  | `control.robotic` | · | 1 | 1 | · | · | · | · | · | · | · | prototype (c) | keep; to the root (G3) |
| `TaskKinematicNullspace` |  | `control.robotic` | · | 1 | 1 | · | · | · | · | · | · | · | prototype (c) | keep; to the root (G3) |
| `TimeVaryingStateFeedbackController` | RS | `control.state` | 2 | 3 | · | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `trajectory_lqr` | RS | `control.design` | · | 1 | 1 | · | · | 1 | · | · | · | 1D | final | keep |
| `TrajectoryFeedbackController` | RS | `control.state` | 1 | 1 | · | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |

### `minilink.control.mpc` (6)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `Command` |  | `control.mpc.controller` | · | 1 | · | · | · | · | · | · | · | 1D | dead | keep: a return type |
| `ModelPredictiveController` |  | `control.mpc.controller` | · | 3 | 5 | 1 | · | 1 | 5 | 3 | 1 | 2D | final | keep; to `minilink.control` and the root (G3) — already the factory |
| `mpc_animation_overlays` |  | `control.mpc.viz` | · | 1 | 4 | 1 | · | 1 | 4 | 1 | · | · | final | keep — a GRO501 notebook imports it |
| `mpc_default_computer_x0` |  | `control.mpc.utilities` | 1 | 2 | · | · | · | · | 1 | · | 1 | · | prototype (c) | internal: off the band — core-consolidation §6.2 |
| `mpc_plans_from_rollout` |  | `control.mpc.viz` | · | 1 | · | · | · | · | · | · | · | · | dead | internal: off the band — tests only |
| `mpc_warm_start_guess` |  | `control.mpc.utilities` | 1 | 1 | · | · | · | · | · | · | · | · | prototype (c) | internal: off the band — core-consolidation §6.2 |

### `minilink.analysis` (34)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `animate_modal` |  | `analysis.modal` | 1 | 1 | 1 | 1 | · | · | · | · | · | 1D | final | keep |
| `bode` | RS | `analysis.frequency` | · | 4 | 1 | 1 | 2 | · | · | · | · | 2D | final | keep |
| `complementary_sensitivity` |  | `analysis.sensitivity_functions` | · | 1 | · | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `controllability` | RS | `analysis.structural` | · | 2 | 1 | 1 | 1 | 1 | · | · | · | · | final | keep |
| `controllability_matrix` |  | `analysis.structural` | 2 | 2 | · | · | · | · | · | · | · | · | prototype (c) | keep |
| `discretize` | RS | `analysis.discretization` | · | 3 | 1 | 1 | · | · | · | · | · | · | prototype (c) | keep |
| `find_equilibrium` | RS | `analysis.equilibria` | 2 | 3 | 1 | 2 | · | · | · | 2 | · | 1D | final | keep |
| `frequency_response` |  | `analysis.frequency` | · | 3 | · | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `jacobian` | RS | `analysis.derivatives` | 6 | 7 | 6 | 5 | 1 | · | · | 4 | · | 2D | final | keep |
| `linearize` | RS | `analysis.linearization` | 1 | 6 | 3 | 3 | 2 | 2 | · | 2 | · | 2D | final | keep |
| `linearize_matrices` |  | `analysis.linearization` | · | 2 | · | · | · | · | · | 1 | · | 1D | final | keep |
| `load_sensitivity` |  | `analysis.sensitivity_functions` | · | 2 | · | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `LyapunovCertificate` |  | `analysis.lyapunov` | · | 1 | · | · | · | · | · | · | · | · | dead | keep: a pillar's seed (R4, core-consolidation §6.3) — rebuilt as a Field verb (A1) |
| `margins` | RS | `analysis.frequency` | · | 2 | 1 | 1 | 1 | · | · | · | · | 2D | final | keep |
| `modal_analysis` | RS | `analysis.modal` | · | 3 | · | · | · | · | · | · | · | · | dead | keep, off the root — `animate_modal` (its demo) calls it in-module |
| `noise_sensitivity` |  | `analysis.sensitivity_functions` | · | 2 | · | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `nyquist` | RS | `analysis.frequency` | · | 1 | · | · | · | · | · | · | · | 1D | dead | keep — `plot_nyquist` calls it in-module |
| `observability` | RS | `analysis.structural` | · | 2 | 1 | 1 | 1 | 1 | · | · | · | · | final | keep |
| `observability_matrix` |  | `analysis.structural` | 2 | 1 | · | · | · | · | · | · | · | · | prototype (c) | keep |
| `plot_bode` | RS | `analysis.frequency` | 1 | 2 | 1 | 4 | 2 | 1 | · | · | · | 1D | final | keep |
| `plot_nyquist` | RS | `analysis.frequency` | · | 1 | 1 | · | 1 | · | · | · | · | · | prototype (c) | keep |
| `plot_pzmap` | RS | `analysis.frequency` | 1 | 2 | 3 | 2 | 2 | 2 | · | · | · | 1D | final | keep |
| `plot_region_of_attraction` |  | `analysis.lyapunov` | · | 1 | · | · | · | · | · | · | · | · | dead | keep: a pillar's seed (R4, core-consolidation §6.3) |
| `plot_root_locus` | RS | `analysis.frequency` | 1 | 1 | 1 | 2 | 1 | 1 | · | · | · | 1D | final | keep |
| `plot_step_response` | RS | `analysis.time_response` | 1 | 1 | 1 | 1 | 1 | 1 | · | · | · | 1D | final | keep |
| `pzmap` | RS | `analysis.frequency` | · | 3 | 1 | · | 1 | 1 | · | · | · | 2D | final | keep |
| `region_of_attraction` |  | `analysis.lyapunov` | · | 1 | 1 | 1 | · | · | · | · | · | · | prototype (c) | keep: a pillar's seed (R4, core-consolidation §6.3) |
| `root_locus` | RS | `analysis.frequency` | · | 1 | 1 | · | 1 | · | · | · | · | 2D | final | keep |
| `sensitivity` |  | `analysis.sensitivity_functions` | · | 2 | · | · | 1 | · | · | · | · | 1 | prototype (c) | keep |
| `step_info` |  | `analysis.time_response` | · | 2 | 1 | · | 1 | · | · | · | · | 1D | final | keep |
| `step_response` | RS | `analysis.time_response` | · | 2 | · | · | 1 | · | · | · | · | 2D | final | keep |
| `StructuralResult` |  | `analysis.structural` | · | · | · | · | · | · | · | · | · | · | dead | keep: a return type — of `controllability` / `observability` |
| `transfer_function` | RS | `analysis.frequency` | 2 | 4 | 1 | 1 | 1 | 1 | · | · | · | 1D | final | keep |
| `VerificationReport` |  | `analysis.lyapunov` | · | · | · | · | · | · | · | · | · | · | dead | keep: a pillar's seed (R4, core-consolidation §6.3) — return type of the certificate |

### `minilink.estimation` (5)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `kalman` | RS | `estimation.design` | 1 | 1 | 2 | 1 | 1 | 1 | · | · | · | 2D | final | keep |
| `kalman_gain` | RS | `estimation.design` | · | 1 | · | · | · | 1 | · | · | · | · | final | keep |
| `luenberger` | RS | `estimation.design` | · | 1 | · | · | 1 | · | · | · | · | 1D | final | keep |
| `luenberger_gain` | RS | `estimation.design` | · | 1 | · | · | · | · | · | · | · | · | dead | keep — the gain form of `luenberger`, beside `kalman_gain` |
| `StateObserver` | RS | `estimation.observer` | 1 | 1 | · | · | · | · | · | · | · | 1D | final | keep on the root — the estimation pillar's block; pinned on the root |

### `minilink.simulation` (7)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `Computer` | S | `simulation.computer` | 4 | 3 | · | · | · | · | 1 | · | · | 1D | final | keep |
| `HybridSimResult` | S | `simulation.hybrid_simulator` | 2 | · | · | · | · | · | · | · | · | 1D | prototype (t) | keep: a return type |
| `HybridSimulator` | S | `simulation.hybrid_simulator` | 1 | 2 | · | · | · | · | · | · | · | 1D | final | keep |
| `RealtimeSimulator` | S | `simulation.realtime.simulator` | 2 | 2 | 2 | · | · | · | · | · | · | 1D | prototype (m) | keep: a pillar's seed (R4, core-consolidation §6.3) — the imitation pillar's recorder |
| `Simulator` | RS | `simulation.simulator` | 2 | 7 | · | 1 | 1 | · | · | 1 | 2 | 1D | final | keep |
| `StaticSimulator` | RS | `simulation.static_simulator` | 1 | 3 | · | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `StepSchedule` | S | `simulation.computer` | 3 | 3 | 1 | · | · | · | 1 | · | · | 1D | final | keep |

### `minilink.optimization` (3)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `MathematicalProgram` | RS | `optimization.mathematical_program` | 11 | 4 | 2 | 1 | · | · | · | · | 2 | 1D | final | keep |
| `OptimizationResult` |  | `optimization.mathematical_program` | 8 | · | · | · | · | · | · | · | 1 | · | prototype (tc) | keep: a return type |
| `Optimizer` | RS | `optimization.optimizer` | 1 | 2 | 2 | 1 | · | · | 1 | · | 1 | 1D | final | keep |

### `minilink.planning` (42)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `as_stochastic` |  | `planning.problems` | 3 | 1 | · | 1 | · | · | · | · | · | · | prototype (c) | keep |
| `bind` |  | `planning.spatial.collision` | · | 2 | 6 | · | · | 1 | 8 | 1 | 1 | 1D | final | keep |
| `car_outline` |  | `planning.spatial.collision` | · | 1 | 2 | · | · | · | 5 | · | · | 1D | final | keep |
| `circuit_waypoints` |  | `planning.spatial.paths` | · | 2 | 2 | · | · | 1 | · | 1 | · | · | final | keep |
| `compare` | RS | `planning.comparison` | · | 1 | 2 | · | 1 | 3 | · | · | · | 1D | final | keep |
| `Comparison` |  | `planning.comparison` | · | · | · | · | · | · | · | · | · | 1D | dead | keep: a return type — of `compare` |
| `DynamicProgrammingOptions` |  | `planning.policy_synthesis.dp` | 2 | 1 | · | · | · | · | · | · | 2 | · | prototype (c) | keep |
| `DynamicProgrammingPlanner` | RS | `planning.policy_synthesis.dp` | · | 5 | 7 | 1 | 1 | 8 | · | · | 2 | 2D | final | keep |
| `EpsilonGreedy` |  | `planning.reinforcement_learning.tabular` | · | 1 | · | · | · | · | · | · | · | 1D | dead | keep: a pillar's seed (R4, core-consolidation §6.3) — with `TabularLearningPlanner` |
| `Evaluation` | RS | `planning.evaluation` | 4 | 1 | · | · | · | · | · | · | · | 1D | final | keep on the root: a foundation object |
| `from_waypoints` |  | `planning.spatial.paths` | · | 1 | 4 | · | · | 1 | 5 | 1 | · | 1D | final | keep |
| `inverse_barrier` |  | `planning.spatial.shaping` | · | 1 | 1 | · | · | · | 6 | · | · | 1D | final | keep |
| `KinodynamicExtender` | RS | `planning.search.extenders` | 2 | 2 | 2 | · | · | · | · | · | 1 | 1D | final | keep (G3, with `RRTPlanner`) |
| `LinearApproximator` | RS | `planning.policy_synthesis.approximation` | · | 1 | · | · | · | · | · | · | · | 1D | dead | off the root; fold into fields (A1) — the course lesson carries its own |
| `LookupTableController` | RS | `planning.policy_synthesis.lookup_policy` | 2 | 1 | · | · | · | · | · | · | · | 1D | final | keep; off the root (no notebook imports it) |
| `LQRPlanner` | RS | `planning.policy_synthesis.lqr` | · | 2 | 1 | · | 1 | 4 | · | · | · | 1D | final | keep |
| `MonteCarloEvaluator` | RS | `planning.evaluation` | 2 | 3 | 4 | 2 | 2 | 2 | · | 1 | · | · | final | keep |
| `PlanningProblem` | RS | `planning.problems` | 15 | 15 | 18 | 4 | 1 | 10 | 12 | 4 | 8 | 2D | final | keep |
| `PlanningSolution` | RS | `planning.results` | 11 | 4 | · | · | · | · | · | · | · | 2D | final | keep on the root: a foundation object |
| `plot_track` |  | `planning.spatial.plotting` | 1 | · | · | · | · | · | · | · | · | 1D | prototype (t) | keep |
| `point_probe` |  | `planning.spatial.collision` | · | 1 | 4 | · | · | 1 | 7 | 1 | · | 1D | final | keep |
| `PolicyEvaluator` | RS | `planning.policy_synthesis.policy_eval` | · | 2 | 1 | · | · | 1 | · | · | · | 1D | final | keep |
| `ProblemParameters` |  | `planning.problems` | · | 3 | · | · | · | · | · | · | · | 1D | dead | remove — retired by R41; tests only |
| `quadratic_excess` |  | `planning.spatial.shaping` | · | 1 | 3 | · | · | · | 5 | · | · | 1D | final | keep |
| `quadratic_hinge` |  | `planning.spatial.shaping` | · | 1 | 3 | · | · | 1 | 5 | 1 | · | 1D | final | keep |
| `QuadraticFeatures` | RS | `planning.policy_synthesis.approximation` | · | 1 | · | · | · | · | · | · | · | 1D | dead | off the root; fold into fields (A1) |
| `RadialBasisFeatures` | RS | `planning.policy_synthesis.approximation` | · | 1 | · | · | · | · | · | · | · | 1D | dead | off the root; fold into fields (A1) |
| `ReferenceTrack` |  | `planning.spatial.track` | 1 | 1 | 4 | · | · | 1 | 5 | 1 | · | 1D | final | keep |
| `ReinforcementLearningPlanner` | RS | `planning.reinforcement_learning.planner` | · | 2 | 4 | 3 | 2 | 4 | · | 2 | · | 2D | final | keep |
| `RRTOptions` | RS | `planning.search.rrt` | 1 | 2 | 1 | · | · | · | · | · | 2 | 1D | final | keep (G3, with `RRTPlanner`) |
| `RRTPlanner` | RS | `planning.search.rrt` | 1 | 3 | 3 | 1 | · | · | · | · | 3 | 2D | final | keep; teaching lane (G3) |
| `RRTStarPlanner` | RS | `planning.search.rrt_star` | · | 1 | 2 | · | · | · | · | · | 2 | 1D | final | keep, off the root — side pillar (R1) |
| `Scene` |  | `planning.spatial.scene` | 2 | 3 | 6 | · | · | 1 | 6 | 1 | 1 | 1D | final | keep |
| `Sphere` |  | `core.geometry` | 3 | 3 | 6 | · | · | 1 | 6 | 1 | 1 | 1D | final | keep |
| `StateSpaceGrid` | RS | `planning.policy_synthesis.discretizer` | 1 | 2 | 2 | · | · | 4 | · | · | 2 | 1D | final | keep |
| `SteeringExtender` | RS | `planning.search.extenders` | 1 | 1 | 2 | · | · | · | · | · | 1 | 1D | final | keep (G3, with `RRTPlanner`) |
| `StochasticPlanningProblem` | RS | `planning.problems` | · | 6 | 4 | 3 | 2 | 4 | · | 3 | · | 1D | final | keep |
| `TabularLearningPlanner` | RS | `planning.reinforcement_learning.tabular` | · | 2 | · | · | · | · | · | · | · | 1D | dead | keep: a pillar's seed (R4, core-consolidation §6.3) — owes a demo |
| `TrackCorridorOverlay` |  | `planning.spatial.overlays` | 1 | · | · | · | · | · | 2 | 1 | · | 1D | prototype (t) | keep |
| `TrajectoryOptimizationOptions` |  | `planning.trajectory_optimization.planner` | · | 5 | · | · | · | · | · | · | 5 | 1D | final | keep |
| `TrajectoryOptimizationPlanner` | RS | `planning.trajectory_optimization.planner` | 2 | 7 | 9 | 3 | · | 2 | 12 | 3 | 5 | 2D | final | keep |
| `UCB` |  | `planning.reinforcement_learning.tabular` | · | 1 | · | · | · | · | · | · | · | 1D | dead | keep: a pillar's seed (R4, core-consolidation §6.3) — with `TabularLearningPlanner` |

### `minilink.interfaces` (4)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `ProblemEnv` |  | `interfaces.gymnasium` | · | · | · | · | · | · | · | · | · | 1D | dead | remove — K8-4 names it |
| `SB3Controller` |  | `interfaces.gymnasium` | · | 1 | · | · | · | 3 | · | · | · | 1D | final | keep to S49 (v1.0), then `experimental/` — three GRO860 notebooks |
| `Sys2Gym` |  | `interfaces.gymnasium` | · | 2 | · | · | · | 4 | · | · | · | 1D | final | keep to S49 (v1.0), then `experimental/` — four GRO860 notebooks |
| `to_gymnasium` |  | `interfaces.gymnasium` | · | · | · | · | · | · | · | · | · | 1D | dead | remove — K8-4 names it |

### `minilink.graphical.catalog` (34)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `Arrow` |  | `graphical.animation.primitives` | 21 | 3 | · | · | · | 1 | 4 | · | · | 1D | final | keep |
| `Box` |  | `graphical.animation.primitives` | 14 | 2 | · | 1 | · | 1 | · | · | · | 1D | final | keep |
| `camera_matrix` |  | `graphical.animation.primitives` | 1 | 1 | · | · | · | · | · | · | · | 1D | final | keep |
| `car_skin_2d` |  | `graphical.catalog.skins` | 3 | · | · | · | · | · | 2 | · | · | · | prototype (tc) | keep |
| `car_skin_3d` |  | `graphical.catalog.skins` | 1 | 1 | · | · | · | · | · | · | · | · | prototype (c) | keep |
| `Circle` |  | `graphical.animation.primitives` | 15 | 2 | · | · | · | 1 | 2 | · | · | · | final | keep |
| `debug_state_skin` |  | `graphical.catalog.skins` | · | 1 | · | · | · | · | · | · | · | · | dead | remove — tests only |
| `ExtrudedPolygon` |  | `graphical.animation.primitives` | 8 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `fixed_camera` |  | `graphical.animation.camera` | · | · | · | · | · | · | · | · | · | · | dead | remove — `camera.py`, K8-4 |
| `follow_frame_camera` |  | `graphical.animation.camera` | · | · | · | · | · | · | · | · | · | · | dead | remove — `camera.py`, K8-4 |
| `ground_line` |  | `graphical.animation.primitives` | 8 | · | · | 1 | · | 1 | · | · | · | 1D | prototype (t) | keep |
| `HorizonPolyline` |  | `graphical.animation.primitives` | 3 | 1 | · | · | · | · | 1 | · | · | · | prototype (c) | keep |
| `Line` |  | `graphical.animation.primitives` | 17 | 1 | 1 | · | · | 1 | 4 | 2 | · | · | final | keep |
| `line_segment` |  | `graphical.catalog.shapes` | · | · | · | · | · | · | · | 1 | · | · | prototype (tc) | to `experimental/` — the contact engine is its one user |
| `link_pose_3d` |  | `graphical.catalog.shapes` | 5 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `merge_skins` |  | `graphical.catalog.skins` | · | · | · | · | · | · | · | · | · | · | dead | remove — no caller |
| `Plane` |  | `graphical.animation.primitives` | 7 | 1 | · | · | · | · | · | 2 | · | 1D | final | keep |
| `plane_airframe_3d` |  | `graphical.catalog.shapes` | 1 | · | · | · | · | · | · | · | · | · | prototype (tc) | to `experimental/catalog/` — follows `Plane3D` |
| `plane_skin_3d` |  | `graphical.catalog.skins` | 1 | · | · | · | · | · | · | · | · | · | prototype (tc) | to `experimental/catalog/` — follows `Plane3D` |
| `Point` |  | `graphical.animation.primitives` | 11 | 1 | · | · | · | 1 | 2 | · | · | · | final | keep |
| `point_pose` |  | `graphical.catalog.shapes` | 5 | · | · | · | · | 1 | · | · | · | · | prototype (t) | keep |
| `Replay` |  | `graphical.animation.drawables` | · | 1 | · | · | · | · | · | · | · | · | dead | remove — tests only |
| `Rod` |  | `graphical.animation.primitives` | 11 | 2 | · | · | · | · | · | · | · | · | prototype (c) | keep |
| `SceneHistory` |  | `graphical.animation.drawables` | 1 | 1 | 1 | · | · | · | 1 | 1 | · | · | prototype (c) | keep |
| `segment_pose_2d` |  | `graphical.catalog.shapes` | 1 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `Sphere` |  | `graphical.animation.primitives` | 11 | 1 | · | · | · | · | 1 | 2 | · | 1D | final | keep |
| `spring_between` |  | `graphical.catalog.shapes` | 2 | · | · | · | · | 1 | · | · | · | · | prototype (t) | keep |
| `spring_line` |  | `graphical.animation.primitives` | 1 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `TorqueArrow` |  | `graphical.animation.primitives` | 9 | 1 | · | · | · | · | 1 | · | · | 1D | final | keep |
| `TrajectoryPolyline` |  | `graphical.animation.primitives` | 3 | 1 | 1 | · | · | · | 1 | 1 | · | · | prototype (c) | keep |
| `ur5_skin` |  | `graphical.catalog.skins` | 1 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `vehicle_body` |  | `graphical.animation.primitives` | 2 | 1 | · | · | · | · | · | · | · | · | prototype (c) | keep |
| `wheel_box` |  | `graphical.animation.primitives` | 3 | · | · | · | · | · | · | · | · | · | prototype (tc) | keep |
| `world_to_camera` |  | `graphical.animation.primitives` | 4 | 1 | · | · | · | · | · | · | · | · | prototype (c) | keep |

### root prelude only (2)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `closed_loop` | R | `core.composition` | 1 | 5 | 1 | · | 2 | · | · | · | · | 1D | final | keep |
| `feedback` | RS | `core.composition` | · | 3 | 1 | · | · | · | · | · | · | 1D | final | keep on the root: a foundation object — the named form of the loop operator |

### deep imports the G3 table needs (no facade today) (11)

| Name | R | Module | lib | tst | demo | tut | top | crs | proj | exp | bch | docs | Maturity | Proposal |
| --- | --- | --- | --: | --: | --: | --: | --: | --: | --: | --: | --: | --: | --- | --- |
| `export_mpc_dual_rate_computer` |  | `control.mpc.controller` | · | · | · | · | · | · | · | · | · | · | dead | internal — the mixin's method calls it |
| `export_mpc_to_computer` |  | `control.mpc.controller` | · | · | · | · | · | · | · | · | · | · | dead | internal — the mixin's method calls it |
| `GeneralizedMechanicalSystem` |  | `dynamics.abstraction.generalized_mechanical` | 2 | 1 | · | · | · | · | · | · | · | · | prototype (c) | on `minilink.dynamics` (proposed) — the boat's and the 3-D plane's base |
| `JaxMechanicalSystem` |  | `dynamics.abstraction.mechanical` | · | 2 | · | · | · | · | · | 1 | · | · | prototype (c) | to `experimental/` with symbolic (ask) — its one non-test user |
| `LTISystem` |  | `dynamics.abstraction.state_space` | 4 | 5 | · | · | · | · | · | 1 | · | 1D | final | on `minilink.dynamics` (proposed) — DESIGN names it |
| `Manipulator` |  | `dynamics.abstraction.manipulator` | 3 | 2 | · | · | · | · | · | · | · | 1D | final | on a new `minilink.dynamics` (G3) |
| `MechanicalSystem` |  | `dynamics.abstraction.mechanical` | 10 | 3 | · | · | · | · | · | 1 | · | 1D | final | on a new `minilink.dynamics` (G3) |
| `MPCBroadcastController` |  | `control.mpc.controller` | · | 1 | · | · | · | · | 1 | · | · | · | prototype (c) | internal (research lane) — one project script |
| `MPCStatefulController` |  | `control.mpc.controller` | · | · | · | · | · | · | · | · | · | · | dead | internal, behind `ModelPredictiveController` |
| `MPCStatelessController` |  | `control.mpc.controller` | · | 1 | · | · | · | · | · | · | · | · | dead | internal, behind `ModelPredictiveController` |
| `StateSpaceSystem` |  | `dynamics.abstraction.state_space` | 1 | 1 | · | · | · | · | · | · | · | 1D | final | on `minilink.dynamics` (proposed) |

## 2. The catalog plants (R2: two import layers, support by test)

All 50 pass `test_plant_compiles_on_both_backends_and_agrees` (NumPy and JAX agree on random
points; 67 passed in `tests/unittest/test_catalog_backends.py`), so support by test does not
separate them yet; use does. The course set is what a course notebook or the tutorial imports;
the topic notebooks add none. "Twin" names the plant it differs from and what differs. Only
the three floating mass chains differ by parameters alone; next come `UdeSRacecar`
(parameters and skin), `KinematicCar` and `DynamicBicycleCar3D` (the skin alone) and
`SpeedControlledDrone2D` (the same `f` as `HolonomicMobileRobot`, other labels). Proposal column: root (course set) /
`minilink.catalog` (supported, tested, documented) / `experimental/catalog/` (K8-3) / merge.

| Domain | Plant | n, m | Both backends | Course notebooks | tut | demos | Twin of / differs by | Proposal |
| --- | --- | --- | --- | --- | --: | --: | --- | --- |
| vehicles | `BicycleDynRate` | 8, 2 | pass | — | 1 | 2 | `DynamicBicycle` with rate inputs (own `f`) | root (course set) |
| vehicles | `DynamicBicycle` | 6, 2 | pass | — | · | 1 | — | `minilink.catalog` (GMC714 bicycle lab) |
| vehicles | `DynamicBicycleCar3D` | 6, 2 | pass | — | 1 | · | skin twin of `DynamicBicycle` (same `f`) | root (course set); or merge into `DynamicBicycle` (ask: tutorial) |
| vehicles | `HolonomicMobileRobot` | 2, 2 | pass | — | · | 3 | — | `minilink.catalog` (RRT, trajopt demos) |
| vehicles | `KinematicBicycle` | 3, 2 | pass | — | · | · | — | `minilink.catalog` (GMC714 bicycle lab) |
| vehicles | `KinematicCar` | 3, 2 | pass | — | 1 | · | skin twin of `KinematicBicycle` (same `f`) | root (course set); or merge into `KinematicBicycle` (ask: tutorial) |
| vehicles | `LongitudinalFrontWheelDriveCarWithTorqueInput` | 4, 1 | pass | — | · | · | slip model + wheel dynamics (the ladder, not a twin) | `minilink.catalog`; rename (A6) |
| vehicles | `LongitudinalFrontWheelDriveCarWithWheelSlipInput` | 2, 1 | pass | — | · | · | — | `minilink.catalog` (GMC714 longitudinal lab); rename (A6) |
| vehicles | `MountainCar` | 2, 1 | pass | — | · | · | — | `experimental/catalog/` |
| vehicles | `QuarterCarOnRoughTerrain` | 3, 1 | pass | — | · | · | — | `minilink.catalog` (vision: vehicle content) or `experimental/catalog/` (no user) — ask |
| vehicles | `UdeSRacecar` | 3, 2 | pass | racecar_toward_mpc | · | 1 | `KinematicBicycle` with the racecar's parameters and skin | root (course set) |
| vehicles | `UdeSRacecarDyn` | 9, 2 | pass | — | · | 1 | — | `minilink.catalog` |
| vehicles | `UdeSRacecarDyn3D` | 11, 2 | pass | racecar_toward_mpc | · | 1 | `UdeSRacecarDyn` + 3-D skin and two states | root (course set) |
| manipulators | `FiveLinkPlanarManipulator` | 10, 5 | pass | — | · | 1 | — | `minilink.catalog` (nullspace demo) |
| manipulators | `OneLinkManipulator` | 2, 1 | pass | — | · | · | `Pendulum` as a `Manipulator` (+ task outputs) | `experimental/catalog/` (or GMC714 keeps it — ask) |
| manipulators | `SpeedControlledManipulator` | 2, 2 | pass | — | · | 2 | — | `minilink.catalog` (kinematic demos) |
| manipulators | `ThreeLinkManipulator3D` | 6, 3 | pass | — | · | · | — | `minilink.catalog` (GMC714 3-DoF impedance lab) |
| manipulators | `TwoLinkManipulator` | 4, 2 | pass | — | · | 4 | — | `minilink.catalog` (GMC714 arm labs) |
| manipulators | `UR5Manipulator` | 12, 6 | pass | — | 2 | 1 | — | root (course set) |
| pendulums and cart-poles | `Acrobot` | 4, 1 | pass | — | · | · | actuation twin of `DoublePendulum` (`B` only) | `experimental/catalog/` |
| pendulums and cart-poles | `CartPole` | 4, 1 | pass | cartpole_dynamic_controller, cartpole_static_controller, cartpole_lqr | 4 | 8 | — | root (course set) |
| pendulums and cart-poles | `CartPoleWithNoisePort` | 4, 6 | pass | cartpole_dynamic_controller | · | 1 | port twin of `CartPole` (+ noise inputs) | root (course set); merge as a `CartPole` option later (ask) |
| pendulums and cart-poles | `DoublePendulum` | 4, 2 | pass | — | · | 3 | — | `minilink.catalog` |
| pendulums and cart-poles | `InvertedPendulum` | 2, 1 | pass | pendulum_cost_to_go_approximation, pendulum_value_iteration_vs_lqr_vs_ppo_sb3, pendulum_value_iteration_vs_lqr_vs_rl | 2 | 4 | convention twin of `Pendulum` (angle from upright: `g` sign) | root (course set) |
| pendulums and cart-poles | `Pendulum` | 2, 1 | pass | grid_world_dynamic_programming, gymnasium_interface, pendulum_value_iteration, pendulum_value_iteration_vs_lqr | 8 | 22 | — | root (course set) |
| pendulums and cart-poles | `PendulumWithNoisePort` | 2, 3 | pass | — | 2 | 2 | port twin of `Pendulum` (+ noise inputs) | root (course set); merge as a `Pendulum` option later (ask) |
| pendulums and cart-poles | `RotatingCartPole` | 4, 2 | pass | — | · | · | — | `experimental/catalog/` |
| pendulums and cart-poles | `TwoIndependentPendulums` | 4, 2 | pass | — | · | · | two `Pendulum`s side by side (a diagram) | `experimental/catalog/` or remove |
| pendulums and cart-poles | `UnderactuatedRotatingCartPole` | 4, 1 | pass | — | · | · | actuation twin of `RotatingCartPole` (`B` only) | `experimental/catalog/` |
| aerial | `ConstantSpeedHelicopterTunnel` | 3, 1 | pass | — | · | · | — | `experimental/catalog/` |
| aerial | `Drone2D` | 6, 2 | pass | drone_ppo, drone_ppo_sb3, gymnasium_interface | 1 | 1 | — | root (course set) |
| aerial | `Drone2DWithSideThruster` | 6, 3 | pass | — | · | · | actuation twin of `Drone2D` (+ one input) | `experimental/catalog/` |
| aerial | `Plane2D` | 6, 2 | pass | — | · | · | — | `experimental/catalog/` (+ its skin) |
| aerial | `Plane3D` | 12, 4 | pass | — | · | · | — | `experimental/catalog/` (+ `plane_skin_3d`, `plane_airframe_3d`) |
| aerial | `Rocket` | 6, 2 | pass | — | · | 1 | — | `minilink.catalog` (RL demo) |
| aerial | `SpeedControlledDrone2D` | 2, 2 | pass | — | · | · | same `f` as `HolonomicMobileRobot` (`ẋ = u`, n = 2) | merge into `HolonomicMobileRobot` (remove) |
| marine | `Boat2D` | 6, 2 | pass | — | · | · | — | `experimental/catalog/` (its demos are already experimental) |
| marine | `Boat2DWithCurrent` | 6, 2 | pass | — | · | · | disturbance twin of `Boat2D` (`d` only) | `experimental/catalog/` |
| astro | `ThreeBodyProblem` | 18, 0 | pass | — | · | 1 | — | `experimental/catalog/` (its demo follows) |
| equations | `DoubleIntegrator` | 2, 1 | pass | double_integrator_minimum_time, double_integrator_policy_evaluation | · | 3 | — | root (course set) |
| equations | `Lorenz` | 3, 0 | pass | — | · | 1 | — | `experimental/catalog/` (its demo follows) |
| equations | `SimpleIntegrator` | 1, 1 | pass | — | · | · | the block `Integrator` with labels | remove (use `Integrator`) |
| equations | `TripleIntegrator` | 3, 1 | pass | — | · | · | chain family with `DoubleIntegrator` | `experimental/catalog/` |
| equations | `VanderPol` | 2, 1 | pass | — | · | · | — | `experimental/catalog/` |
| mass-spring-damper | `FloatingSingleMass` | 2, 1 | pass | — | · | · | parameters only: `SingleMass` with `k1 = 0` | merge into `SingleMass` (remove) |
| mass-spring-damper | `FloatingThreeMass` | 6, 1 | pass | — | · | · | parameters only: `ThreeMass` with `k1 = 0` | merge into `ThreeMass` (remove) |
| mass-spring-damper | `FloatingTwoMass` | 4, 1 | pass | — | · | · | parameters only: `TwoMass` with `k1 = 0` | merge into `TwoMass` (remove) |
| mass-spring-damper | `SingleMass` | 2, 1 | pass | numpy_state_space | · | · | — | root (course set) |
| mass-spring-damper | `ThreeMass` | 6, 1 | pass | — | · | · | — | `experimental/catalog/` |
| mass-spring-damper | `TwoMass` | 4, 1 | pass | — | · | · | — | `experimental/catalog/` |

**Tally:** root 14 (`Pendulum`, `InvertedPendulum`, `PendulumWithNoisePort`, `CartPole`,
`CartPoleWithNoisePort`, `DoubleIntegrator`, `SingleMass`, `Drone2D`, `UdeSRacecar`,
`UdeSRacecarDyn3D`, `BicycleDynRate`, `DynamicBicycleCar3D`, `KinematicCar`,
`UR5Manipulator`); `minilink.catalog` 13 more; `experimental/catalog/` 18; merged or removed 5.
The floating chains merge into their parents (`TwoMass` and `ThreeMass` go to the experimental
tier, so the merge happens there). The vehicle and arm plants proposed for `minilink.catalog`
are the ones the GMC714 labs are likely to load (R48's lab list: the bicycle models, the
longitudinal car, the arms); the labs are not in the repo, so that column is an assumption to
confirm.

## 3. Every demo

"Twin" below means what K4 removes: two demos showing the same feature on the same plant.
"Near" marks the same feature on another plant, or a demo that contains another; those are
rulings, not defects. A course or tutorial notebook that shows the same thing is named, never
as a change to the notebook (AGENTS: notebooks are teaching designs). L = lines.

### `examples/demos/` (74)

| Demo | L | Feature it shows | Plant | Twins | Proposal |
| --- | --: | --- | --- | --- | --- |
| `analysis/analysis_discretize.py` | 16 | `discretize` (Euler, RK4) to a `StepSystem` | `Pendulum` | — | keep |
| `analysis/analysis_equilibrium.py` | 13 | `find_equilibrium` (trim point) | `Pendulum` | — | keep |
| `analysis/analysis_frequency.py` | 36 | Bode with margins, Nyquist, step, on the plant and a PID loop | `Pendulum` | — | keep |
| `analysis/analysis_linearize.py` | 34 | `linearize` at an operating point | `InvertedPendulum` | near: `analysis_linearize_fd_vs_jax` | keep |
| `analysis/analysis_linearize_fd_vs_jax.py` | 14 | finite-difference vs JAX Jacobians | `CartPole`, `DoublePendulum` | near: `analysis_linearize` | merge into `analysis_linearize` |
| `analysis/analysis_modal.py` | 8 | modal analysis and `animate_modal` | `DoublePendulum` | — | keep |
| `analysis/analysis_region_of_attraction.py` | 38 | Lyapunov region of attraction of a saturated LQR loop | `Pendulum` | near: `experimental/rl/pendulum_rl_lyapunov_certificate` | keep (the seed's one demo) |
| `analysis/analysis_root_locus.py` | 37 | root locus, angle feedback vs `Lead` | `InvertedPendulum` | — | keep |
| `analysis/analysis_structural.py` | 15 | controllability and observability | `InvertedPendulum` | — | keep |
| `blocks/blocks_sources.py` | 23 | `Step` and `WhiteNoise` sources | — | — | keep |
| `blocks/signal_blocks.py` | 63 | the nonlinear and filter blocks in parallel | — | — | keep |
| `compile/cartpole_rollout_gradients.py` | 157 | gradients through a rollout, inputs vs inputs + parameters | `CartPole` | twin: the topic notebook `optimal_control/cartpole_rollout_gradients.ipynb` (same name, plant, feature) | ask: keep one form |
| `compile/diagram_compiling.py` | 150 | compiling a custom-block diagram, the compile report | custom | — | keep |
| `compile/neural_controller_jax.py` | 158 | a neural controller trained over batched closed-loop rollouts | custom (integrator) | — | keep (moves to `MLP` if `NeuralNetwork` merges) |
| `compile/params_gradient.py` | 137 | gradients with respect to parameters: sensitivity, identification | custom | near: `rollout_param_family` | keep (the identification seed's demo) |
| `compile/pid_autotuning_jax.py` | 134 | PID gains tuned by differentiating a rollout | `Pendulum` | — | keep |
| `compile/rollout_param_family.py` | 33 | one rollout over a family of parameters (`vmap`) | `Pendulum` | — | keep |
| `control/cartpole_lqr.py` | 22 | LQR balance | `CartPole` | near: `lqg_cartpole` (contains it); the GRO860 notebook `cartpole_lqr` | keep |
| `control/classical_family.py` | 40 | P, PI, PD, PID and impedance side by side | `Pendulum` | — | keep |
| `control/computed_torque_pendulum.py` | 24 | computed torque | `Pendulum` | near: `robotic/computed_torque_two_link` | merge into `computed_torque_two_link` (the GMC714 demo) |
| `control/kalman_observer_pendulum.py` | 26 | a Kalman observer from the angle alone | `Pendulum` | — | keep |
| `control/lqg_cartpole.py` | 38 | LQG under noise | `CartPoleWithNoisePort` | — | keep |
| `control/pid_anti_windup.py` | 23 | PID anti-windup, the loop closed with `@` | `DoubleIntegrator` | — | keep |
| `control/pole_placement_pendulum.py` | 32 | pole placement | `InvertedPendulum` | — | keep |
| `control/sliding_mode_pendulum.py` | 27 | sliding-mode control | `Pendulum` | **twin**: `hybrid/sampled_smc_pendulum` runs the same continuous law first | ask: keep as G3's sliding-mode demo, or merge into `sampled_smc_pendulum` |
| `control/trajectory_lqr_cartpole.py` | 60 | a planned swing-up, then LQR along it | `CartPole` | near: `trajopt/trajopt_cartpole_collocation_jax` (the same plan) | keep |
| `core/diagram_advanced_autowire.py` | 127 | autowire on a multi-loop cascade | `DynamicBicycle` | — | keep |
| `core/diagram_closed_loop.py` | 45 | wiring a loop by hand | `Pendulum` | near: `diagram_shortcuts` (the same loop by operators) | keep |
| `core/diagram_nested_loop.py` | 36 | a cascade wired by hand | custom | — | keep |
| `core/diagram_noise_ports.py` | 67 | process and measurement noise through plant ports, by hand | `PendulumWithNoisePort` | **twin**: the noise section of `diagram_shortcuts` | merge into `diagram_shortcuts` |
| `core/diagram_shortcuts.py` | 105 | the composition operators | `Pendulum`, `PendulumWithNoisePort` | — | keep |
| `core/readme_examples.py` | 109 | the README's snippets, runnable | `Pendulum` | repeats others by design | keep (the README contract) |
| `dynamics/lorenz_attractor.py` | 25 | open-loop chaotic simulation | `Lorenz` | — | to `experimental/` with its plant |
| `dynamics/three_body.py` | 15 | open-loop gravitational simulation | `ThreeBodyProblem` | — | to `experimental/` with its plant |
| `graphical/animation_renderers.py` | 18 | one trajectory on each renderer | `Pendulum` | — | keep |
| `graphical/plot_internal_signals.py` | 17 | choosing a diagram plot's internal signals | `Pendulum` | — | keep |
| `graphical/plot_phase_plane.py` | 16 | the phase-plane plot | `Pendulum` | — | keep |
| `graphical/plot_trajectory_options.py` | 21 | trajectory and plotting options | `Pendulum` | — | keep |
| `hybrid/hybrid_multi_rate.py` | 75 | a multi-rate `Computer` on a continuous plant | custom (integrator) | — | keep |
| `hybrid/sampled_smc_pendulum.py` | 38 | continuous vs sampled sliding mode | `Pendulum` | twin: `control/sliding_mode_pendulum` | keep |
| `hybrid/step_accumulator.py` | 29 | a `StepSystem` leaf | custom | **twin**: `step_fibonacci`; `step_unity_feedback` contains it | merge into `step_unity_feedback` |
| `hybrid/step_cascade_zoh.py` | 27 | `ZOHHold` blocks in series | — | — | keep |
| `hybrid/step_fibonacci.py` | 24 | a `StepSystem` leaf | custom | **twin**: `step_accumulator` | remove |
| `hybrid/step_unity_feedback.py` | 38 | a `StepDiagramSystem` feedback loop | custom | — | keep |
| `mpc/mpc_car_circuit.py` | 210 | MPC on a circuit with corridor and obstacles | `BicycleDynRate` | near: `udes_racecar/mpc_racecar_dynamic` (circuit, obstacles, dynamic bicycle) | merge into `mpc_racecar_dynamic` |
| `mpc/mpc_car_minimal.py` | 66 | the minimal MPC loop, `mpc @ plant` | `BicycleDynRate` | — | keep (G3's MPC demo) |
| `mpc/mpc_integrator_numpy.py` | 71 | MPC without JAX (rebuild each tick) | custom (integrator) | — | keep |
| `optimization/optim_basic.py` | 15 | `MathematicalProgram` and `Optimizer` | — | near: `optim_plot` (adds the iterate trace) | keep |
| `optimization/optim_plot.py` | 68 | the optimizer's iterates on a non-convex cost | — | near: `optim_basic` | ask: keep if the trace is a feature, else remove |
| `realtime/game_car.py` | 13 | gamepad driving with MeshCat and Plotly | `UdeSRacecarDyn3D` | — | keep (the recorder seed) |
| `realtime/game_cartpole.py` | 28 | keyboard game mode | `CartPole` | — | keep |
| `realtime/game_ur5_setpoint.py` | 55 | a keyboard-steered task-impedance setpoint in 3-D | `UR5Manipulator` | near: `experimental/robotic/task_impedance_ur5` | keep |
| `rl/cartpole_swing_up_rl.py` | 83 | the RL planner on a swing-up | `CartPole` | near: `pendulum_swing_up_rl` | ask: remove (the pendulum one is the course's) |
| `rl/drone_learn_to_fly_rl.py` | 86 | PPO hover | `Drone2D` | the GRO860 notebook `drone_ppo` and the topic notebook `drone_ppo` | keep (the script form) |
| `rl/pendulum_swing_up_rl.py` | 83 | the RL planner on a swing-up | `Pendulum` | — | keep |
| `rl/rocket_landing_rl.py` | 92 | termination with a crash cost | `Rocket` | — | keep |
| `robotic/computed_torque_two_link.py` | 31 | computed torque on an arm | `TwoLinkManipulator` | — | keep (G3) |
| `robotic/joint_impedance_two_link.py` | 29 | joint impedance (`JointImpedance(arm)` builds a `ModelJointImpedance`) | `TwoLinkManipulator` | near: `experimental/robotic/joint_impedance_ur5` | keep (G3) |
| `robotic/kinematic_nullspace_five_link.py` | 43 | kinematic control with a nullspace posture | `FiveLinkPlanarManipulator`, `SpeedControlledManipulator` | — | keep (G3) |
| `robotic/kinematic_two_link.py` | 24 | task-space kinematic control | `SpeedControlledManipulator`, `TwoLinkManipulator` | — | keep (G3) |
| `robotic/task_impedance_two_link.py` | 32 | end-effector impedance | `TwoLinkManipulator` | near: `realtime/game_ur5_setpoint` | keep (G3) |
| `rrt/rrt_holonomic_obstacles.py` | 244 | RRT vs RRT*, the extenders, a scene | `HolonomicMobileRobot` | — | keep (G3) |
| `rrt/rrt_pendulum_swingup.py` | 37 | kinodynamic RRT | `Pendulum` | — | keep (G3) |
| `rrt/rrt_star_live.py` | 156 | RRT* converging, drawn live | `HolonomicMobileRobot` | **twin**: `rrt_holonomic_obstacles` (RRT*, same plant, a scene) | remove, with `live_plot.py` (core-consolidation §6.2) |
| `trajopt/trajopt_cartpole_collocation_jax.py` | 57 | JAX direct collocation | `CartPole` | near: `control/trajectory_lqr_cartpole` | keep |
| `trajopt/trajopt_holonomic_corridor.py` | 217 | trajectory optimization in a corridor with obstacles | `HolonomicMobileRobot` | — | keep |
| `udes_racecar/mpc_racecar_dynamic.py` | 165 | MPC round a circuit past two cones | `UdeSRacecarDyn` | near: `mpc/mpc_car_circuit` | keep |
| `udes_racecar/mpc_racecar_kinematic.py` | 89 | MPC on a small circuit, one cone | `UdeSRacecar` | the GRO501 notebook `racecar_toward_mpc` | keep |
| `value_iteration/vi_cartpole_swingup.py` | 86 | value iteration on a coarse 4-D grid | `CartPole` | — | keep |
| `value_iteration/vi_double_pendulum_jax.py` | 118 | value iteration in 4-D on JAX | `DoublePendulum` | — | keep |
| `value_iteration/vi_minimum_time.py` | 50 | minimum time (`TimeCost`) | `DoubleIntegrator` | near: `vi_quadratic`; the GRO860 notebook `double_integrator_minimum_time` | keep |
| `value_iteration/vi_pendulum_lqr.py` | 69 | value iteration vs LQR (`compare`) | `Pendulum` | contains `vi_pendulum_swingup`; the GRO860 notebook `pendulum_value_iteration_vs_lqr` | keep |
| `value_iteration/vi_pendulum_swingup.py` | 51 | value-iteration swing-up | `Pendulum` | **twin**: `vi_pendulum_lqr` contains it | remove |
| `value_iteration/vi_quadratic.py` | 48 | value iteration with a quadratic cost | `DoubleIntegrator` | **twin**: `vi_minimum_time` (value iteration on the same plant; another cost) | merge into `vi_minimum_time` (two costs) |

### `examples/experimental/` (17; moving to a top-level `experimental/` with K9-1)

These sit in the experimental tier (R3): untested, out of the student tree, free to go. The
proposals only name twins.

| Demo | L | Feature it shows | Plant | Twins | Proposal |
| --- | --: | --- | --- | --- | --- |
| `boat/parking.py` | 293 | the shared plant, dock, cost and cascade law (a module the others import) | `Boat2DWithCurrent` | — | keep |
| `boat/parking_cascade.py` | 21 | the cascade law, with and without current | `Boat2DWithCurrent` | **twin**: `parking_compare` runs it | merge into `parking_compare` |
| `boat/parking_compare.py` | 120 | cascade, MPC and the learned law from one start | `Boat2DWithCurrent` | — | keep |
| `boat/parking_mpc.py` | 54 | MPC on the parking cost | `Boat2DWithCurrent` | **twin**: `parking_compare` runs it | merge into `parking_compare` |
| `boat/parking_ppo.py` | 80 | PPO training of the parking policy | `Boat2DWithCurrent` | — | keep (it trains `parking_ppo_policy.npz`) |
| `c_export/c_export.py` | 24 | C export of a PID block | — | **twin**: `c_export_proportional` | keep (the flagship smoke) |
| `c_export/c_export_proportional.py` | 33 | C export of a P block | — | **twin**: `c_export` | merge into `c_export` (both are in the flagship manifest) |
| `engine/engine_ancf_tire_fall.py` | 41 | a JAX ANCF tire ring falling on a plane | — | — | ask: R3 keeps the engines; keep one demo or two |
| `engine/engine_physics_in_diagram.py` | 57 | a contact world as a leaf inside a diagram | — | — | keep (R3's contact-leaf demo) |
| `engine/engine_physics_many_spheres.py` | 112 | the same world, many spheres | — | **twin**: `engine_physics_in_diagram` | remove |
| `mppi/pendulum_mppi.py` | 142 | MPPI swing-up | `Pendulum` | — | keep (MPPI's seed, S72) |
| `mppi/racecar_mppi.py` | 273 | MPPI vs collocation MPC on the cone circuit | `UdeSRacecar` | — | keep |
| `random/noise_and_seeds.ipynb` | — | the draws: noise sources, seeds, stochastic problems | `PendulumWithNoisePort` | — | keep |
| `rl/pendulum_rl_lyapunov_certificate.py` | 253 | certifying a learned policy | `Pendulum` | near: `analysis/analysis_region_of_attraction` | keep (the learned-policy certificate seed) |
| `robotic/joint_impedance_ur5.py` | 27 | joint impedance with gravity compensation | `UR5Manipulator` | near: `robotic/joint_impedance_two_link` | remove |
| `robotic/task_impedance_ur5.py` | 34 | end-effector impedance with gravity compensation | `UR5Manipulator` | **twin**: `realtime/game_ur5_setpoint` (task impedance on the UR5) | remove |
| `symbolic/symbolic_quadruple_pendulum.py` | 84 | symbolic equations of motion and export | custom | — | keep |

**Tally:** `examples/demos/`: 59 keep, 6 merge, 3 remove, 2 to `experimental/`, 4 asks.
`examples/experimental/`: 10 keep, 3 merge, 3 remove, 1 ask. The demos G3 needs (one per GMC714
name under `examples/demos/`) are all kept; `ModelJointImpedance`, `Manipulator` and
`MechanicalSystem` have none yet (§4).

## 4. Two draft rename tables (proposals for the maintainer, not decided)

### 4.1 Short catalog, control and block names (A6, RULES 2.6)

Every class whose display `name` differs from its class name: 64 of the 83 literal
`self.name = "…"` strings in `minilink/` (19 already equal the class name, e.g. `Integrator`,
`Acrobot`), plus four plants that build theirs at run time (`SingleMass`, `TwoMass`,
`ThreeMass`, `SpeedControlledManipulator`): 68 rows. The rule of the draft: **every explicit display name is dropped**, so the class
name shows (that alone meets A6's gate, "no display name longer than its class name");
**option A** also renames the ten classes whose name is long or misleading; **option B** also
drops the `Controller` suffix from the control classes (RULES 2.6 asks for short names; the
constitution's §4.5 names by diagram role, and `Controller` is the role, hence an option).

The diagram id is the name in snake case (`core/composition.py`, `normalize_identifier`), so
dropping a display name that only spaces out the class name changes nothing ("same"); 31 ids
change. Searched across `examples/`, `tests/`, `benchmarks/` and `minilink/`, no course notebook
quotes a display name or a changing id as a string; one test does (`"Cart Pole with noise ports"`
in `test_dynamics_catalog.py`), and `catalog_check_registry.py` mentions
`five_link_planar_manipulator`.

**One defect the drop exposes:** the id rule splits only before an upper-case letter that
follows a lower-case letter or a digit, so `Drone2D` becomes `drone2_d`, `Boat2DWithCurrent`
`boat2_dwith_current`, `UdeSRacecarDyn` `ude_sracecar_dyn` and `MPCStatefulController`
`mpcstateful_controller`. Fixing the rule for digits and acronyms belongs in the same step (it
changes ids, a public behaviour: an ask), or those classes keep a short explicit name.

| Band | Class (chars) | Display name today | id today → after | Option A class | Option B | Course notebooks importing it | Flags |
| --- | --- | --- | --- | --- | --- | --- | --- |
| catalog | `LongitudinalFrontWheelDriveCarWithWheelSlipInput` (48) | Front Wheel Drive Car With Slip Input (37) | `front_wheel_drive_car_with_slip_input` → `longitudinal_slip_car` | `LongitudinalSlipCar` | — | — | GMC714 lab plant? |
| catalog | `LongitudinalFrontWheelDriveCarWithTorqueInput` (45) | Front Wheel Drive Car With Torque Input (39) | `front_wheel_drive_car_with_torque_input` → `longitudinal_torque_car` | `LongitudinalTorqueCar` | — | — | GMC714 lab plant? |
| catalog | `ConstantSpeedHelicopterTunnel` (29) | Constant Speed Helicopter Tunnel (32) | `constant_speed_helicopter_tunnel` → same | — | — | — | moot if K8-3 moves it |
| catalog | `UnderactuatedRotatingCartPole` (29) | Underactuated Rotating Cart Pole (32) | `underactuated_rotating_cart_pole` → same | — | — | — | moot if K8-3 moves it |
| catalog | `SpeedControlledManipulator` (26) | 2 Joint Speed Controlled Manipulator (36) | `sys_2_joint_speed_controlled_manipulator` → `kinematic_manipulator` | `KinematicManipulator` | — | — | GMC714 lab plant? |
| catalog | `FiveLinkPlanarManipulator` (25) | Five Link Planar Manipulator (28) | `five_link_planar_manipulator` → `five_link_manipulator` | `FiveLinkManipulator` | — | — | GMC714 lab plant? |
| catalog | `QuarterCarOnRoughTerrain` (24) | Quarter Car On Rough Terrain (28) | `quarter_car_on_rough_terrain` → `quarter_car` | `QuarterCar` | — | — |  |
| catalog | `Drone2DWithSideThruster` (23) | Planar Drone With Side Thruster (31) | `planar_drone_with_side_thruster` → `drone2_dwith_side_thruster` | — | — | — | moot if K8-3 moves it |
| catalog | `TwoIndependentPendulums` (23) | Two Independent Pendulums (25) | `two_independent_pendulums` → same | — | — | — | moot if K8-3 moves it |
| catalog | `SpeedControlledDrone2D` (22) | Speed Controlled Planar Drone (29) | `speed_controlled_planar_drone` → `speed_controlled_drone2_d` | — | — | — | moot if K8-3 moves it |
| catalog | `ThreeLinkManipulator3D` (22) | Three Link Manipulator 3D (25) | `three_link_manipulator_3_d` → `three_link_manipulator` | `ThreeLinkManipulator` | — | — | GMC714 lab plant? |
| catalog | `CartPoleWithNoisePort` (21) | Cart Pole with noise ports (26) | `cart_pole_with_noise_ports` → `noisy_cart_pole` | `NoisyCartPole` | — | cartpole_dynamic_controller |  |
| catalog | `PendulumWithNoisePort` (21) | Pendulum (8) | `pendulum` → `noisy_pendulum` | `NoisyPendulum` | — | — | tutorial ×2 |
| catalog | `HolonomicMobileRobot` (20) | Holonomic Mobile Robot (22) | `holonomic_mobile_robot` → same | — | — | — |  |
| catalog | `DynamicBicycleCar3D` (19) | Dynamic Bicycle (3D car) (24) | `dynamic_bicycle_3_d_car` → `dynamic_bicycle_car3_d` | — | — | — | tutorial ×1 |
| catalog | `OneLinkManipulator` (18) | One Link Manipulator (20) | `one_link_manipulator` → same | — | — | — | GMC714 lab plant? |
| catalog | `TwoLinkManipulator` (18) | Two Link Manipulator (20) | `two_link_manipulator` → same | — | — | — | GMC714 lab plant? |
| catalog | `FloatingSingleMass` (18) | Floating Single Mass (20) | `floating_single_mass` → same | — | — | — | moot if K8-3 moves it |
| catalog | `Boat2DWithCurrent` (17) | Planar Boat With Current (24) | `planar_boat_with_current` → `boat2_dwith_current` | — | — | — | moot if K8-3 moves it |
| catalog | `FloatingThreeMass` (17) | Floating Three Mass (19) | `floating_three_mass` → same | — | — | — | moot if K8-3 moves it |
| catalog | `ThreeBodyProblem` (16) | Three-Body Problem (18) | `three_body_problem` → same | — | — | — | moot if K8-3 moves it |
| catalog | `SimpleIntegrator` (16) | Simple Integrator (17) | `simple_integrator` → same | — | — | — | moot if K8-3 moves it |
| catalog | `DoubleIntegrator` (16) | Double Integrator (17) | `double_integrator` → same | — | — | double_integrator_minimum_time, double_integrator_policy_evaluation |  |
| catalog | `TripleIntegrator` (16) | Triple Integrator (17) | `triple_integrator` → same | — | — | — | moot if K8-3 moves it |
| catalog | `RotatingCartPole` (16) | Rotating Cart Pole (18) | `rotating_cart_pole` → same | — | — | — | moot if K8-3 moves it |
| catalog | `InvertedPendulum` (16) | Inverted Pendulum (17) | `inverted_pendulum` → same | — | — | pendulum_cost_to_go_approximation, pendulum_value_iteration_vs_lqr_vs_ppo_sb3, pendulum_value_iteration_vs_lqr_vs_rl | tutorial ×2 |
| catalog | `UdeSRacecarDyn3D` (16) | UdeS Racecar (3D) (17) | `ude_s_racecar_3_d` → `ude_sracecar_dyn3_d` | — | — | racecar_toward_mpc |  |
| catalog | `KinematicBicycle` (16) | Kinematic Bicycle (17) | `kinematic_bicycle` → same | — | — | — | GMC714 lab plant? |
| catalog | `FloatingTwoMass` (15) | Floating Two Mass (17) | `floating_two_mass` → same | — | — | — | moot if K8-3 moves it |
| catalog | `UR5Manipulator` (14) | UR5 Manipulator (15) | `ur5_manipulator` → same | — | — | — | tutorial ×2 |
| catalog | `DoublePendulum` (14) | Double Pendulum (15) | `double_pendulum` → same | — | — | — |  |
| catalog | `DynamicBicycle` (14) | Dynamic Bicycle (15) | `dynamic_bicycle` → same | — | — | — | GMC714 lab plant? |
| catalog | `UdeSRacecarDyn` (14) | UdeS Racecar (12) | `ude_s_racecar` → `ude_sracecar_dyn` | — | — | — |  |
| catalog | `KinematicCar` (12) | Kinematic Car (13) | `kinematic_car` → same | — | — | — | tutorial ×1 |
| catalog | `MountainCar` (11) | Mountain Car (12) | `mountain_car` → same | — | — | — | moot if K8-3 moves it |
| catalog | `UdeSRacecar` (11) | UdeS Racecar (12) | `ude_s_racecar` → `ude_sracecar` | — | — | racecar_toward_mpc |  |
| catalog | `SingleMass` (10) | Single Mass Spring Damper (25) | `single_mass_spring_damper` → `single_mass` | — | — | numpy_state_space |  |
| catalog | `VanderPol` (9) | Van der Pol Oscillator (22) | `van_der_pol_oscillator` → `vander_pol` | — | — | — | moot if K8-3 moves it |
| catalog | `ThreeMass` (9) | Three Mass Spring Damper (24) | `three_mass_spring_damper` → `three_mass` | — | — | — | moot if K8-3 moves it |
| catalog | `CartPole` (8) | Cart Pole (9) | `cart_pole` → same | — | — | cartpole_dynamic_controller, cartpole_lqr, cartpole_static_controller | tutorial ×4 |
| catalog | `Drone2D` (7) | Planar Drone (12) | `planar_drone` → `drone2_d` | — | — | drone_ppo, drone_ppo_sb3, gymnasium_interface | tutorial ×1 |
| catalog | `Plane2D` (7) | Planar Aircraft (15) | `planar_aircraft` → `plane2_d` | — | — | — | moot if K8-3 moves it |
| catalog | `Plane3D` (7) | Aircraft 3D (11) | `aircraft_3_d` → `plane3_d` | — | — | — | moot if K8-3 moves it |
| catalog | `TwoMass` (7) | Two Mass Spring Damper (22) | `two_mass_spring_damper` → `two_mass` | — | — | — | moot if K8-3 moves it |
| catalog | `Rocket` (6) | Planar Rocket (13) | `planar_rocket` → `rocket` | — | — | — |  |
| catalog | `Lorenz` (6) | Lorenz Attractor (16) | `lorenz_attractor` → `lorenz` | — | — | — | moot if K8-3 moves it |
| catalog | `Boat2D` (6) | Planar Boat (11) | `planar_boat` → `boat2_d` | — | — | — | moot if K8-3 moves it |
| control | `TimeVaryingStateFeedbackController` (34) | Time-Varying State Feedback Controller (38) | `time_varying_state_feedback_controller` → `time_varying_feedback` | `TimeVaryingFeedback` | — | — |  |
| control | `TrajectoryFeedbackController` (28) | Trajectory Feedback Controller (30) | `trajectory_feedback_controller` → same | — | `TrajectoryFeedback` | — |  |
| control | `ImpedanceIntegralController` (27) | Impedance Integral Controller (29) | `impedance_integral_controller` → `integral_impedance` | `IntegralImpedance` | — | — | tutorial ×1 |
| control | `OutputReferenceController` (25) | Output Reference Controller (27) | `output_reference_controller` → same | — | `OutputReference` | — |  |
| control | `ComputedTorqueController` (24) | Computed Torque Controller (26) | `computed_torque_controller` → same | — | `ComputedTorque` | — | G3 freezes; tutorial ×1 |
| control | `StateFeedbackController` (23) | State Feedback Controller (25) | `state_feedback_controller` → same | — | `StateFeedback` | — | tutorial ×1 |
| control | `TaskKinematicNullspace` (22) | Task Kinematic Nullspace (24) | `task_kinematic_nullspace` → same | — | — | — | G3 freezes |
| control | `SlidingModeController` (21) | Sliding Mode Controller (23) | `sliding_mode_controller` → same | — | `SlidingMode` | — | G3 freezes; tutorial ×1 |
| control | `ImpedanceController` (19) | Impedance Controller (20) | `impedance_controller` → same | — | `Impedance` | — | G3 freezes; tutorial ×4 |
| control | `ModelJointImpedance` (19) | Joint Impedance (15) | `joint_impedance` → `model_joint_impedance` | — | — | — | G3 freezes |
| control | `TaskImpedance` (13) | Task Impedance (14) | `task_impedance` → same | — | — | — | G3 freezes |
| control | `TaskKinematic` (13) | Task Kinematic (14) | `task_kinematic` → same | — | — | — | G3 freezes |
| control | `PurePursuit` (11) | Pure Pursuit (12) | `pure_pursuit` → same | — | — | — |  |
| blocks | `TrajectorySource` (16) | Trajectory Source (17) | `trajectory_source` → same | — | — | cartpole_lqr |  |
| blocks | `NeuralNetwork` (13) | Neural Network (14) | `neural_network` → same | — | — | — |  |
| blocks | `RateLimiter` (11) | Rate Limiter (12) | `rate_limiter` → same | — | — | — |  |
| planning | `LookupTableController` (21) | Lookup Table Controller (23) | `lookup_table_controller` → same | — | `LookupPolicy` | — |  |
| estimation | `StateObserver` (13) | State Observer (14) | `state_observer` → same | — | — | — |  |
| (deep) | `MPCStatelessController` (22) | MPC Stateless Controller (24) | `mpc_stateless_controller` → `mpcstateless_controller` | — | — | — |  |
| (deep) | `MPCBroadcastController` (22) | MPC Broadcast (13) | `mpc_broadcast` → `mpcbroadcast_controller` | — | — | — |  |
| (deep) | `MPCStatefulController` (21) | MPC Stateful Controller (23) | `mpc_stateful_controller` → `mpcstateful_controller` | — | — | — |  |

### 4.2 The names GMC714's labs need stable by January (G3, R49–R50)

| Name | Today | Callers (lib, tst, demo, tut, crs) | Proposed move | With A6 | Gap before January 9 |
| --- | --- | --- | --- | --- | --- |
| `RRTPlanner` | root and `minilink.planning`; module `planning.search.rrt` (already a teaching-lane module); ROADMAP §3 ledger: provisional | 1, 3, 3, 1, 0 | the ledger row to teaching; no code move | — | the RRT demos still deep-import `StraightLineSteering`, `RRTStarOptions` and the collision helper `disc`: put on `minilink.planning` the ones the lab uses |
| `RRTOptions`, `KinodynamicExtender`, `SteeringExtender` | root and `minilink.planning` | 1/2/1 lib, 2/2/1 tst, 1/2/2 demo | frozen with `RRTPlanner` | — | — |
| `ModelJointImpedance` | `minilink.control` only | 0, 1, 0, 0, 0 (built by `JointImpedance(plant)`, which `joint_impedance_two_link` calls) | to the root | display "Joint Impedance" dropped: id `joint_impedance` → `model_joint_impedance` | a demo that names it, or the ruling that `JointImpedance(plant)` is the teaching call and the class only its type |
| `TaskKinematic` | `minilink.control` only | 0, 1, 1, 0, 0 | to the root | display dropped, id same | — |
| `TaskKinematicNullspace` | `minilink.control` only | 0, 1, 1, 0, 0 | to the root | display dropped, id same | — |
| `Manipulator` | deep: `dynamics.abstraction.manipulator` | 3, 2, 0, 0, 0 | on `minilink.dynamics` | — | no demo writes an arm from it; one short demo (a custom two-link arm) |
| `MechanicalSystem` | deep: `dynamics.abstraction.mechanical` | 10, 3, 0, 0, 0 | on `minilink.dynamics` | — | no demo; the same demo, or a one-mass `MechanicalSystem` |
| `ModelPredictiveController` | `minilink.control.mpc` (research lane); already the factory | 0, 3, 5, 1, 1 (+5 projects, 3 experimental, 1 benchmark) | on `minilink.control` and the root | — | the teaching-surface test refuses a `minilink.control.mpc` home: the factory moves to a teaching-lane module (e.g. `control/predictive.py`) or the test names it as the one exception |
| `ComputedTorqueController`, `SlidingModeController`, `ImpedanceController`, `TaskImpedance`, `JointImpedance` | root and `minilink.control` | demos 2, 2, 5, 2, 1 | frozen as they are | option B renames the first three: G3 would pin `ComputedTorque`, `SlidingMode`, `Impedance` | — |

**The proposed `minilink.dynamics` facade** (R50: the modelling abstractions, beside
`minilink.catalog` for the plants), from `minilink/dynamics/abstraction/`:
`MechanicalSystem`, `Manipulator`, `GeneralizedMechanicalSystem` (the base the boat and the
3-D plane use: `M(q) a + C v + d + g = B u`, `q̇ = N(q) v`), `StateSpaceSystem` and `LTISystem`
(the linear abstractions; DESIGN names `LTISystem`). Left deep: `JaxMechanicalSystem` (its one
non-test user is the symbolic experimental tier; proposed to follow it) and the module helpers
(`require_positive_task_dim`, `initial_guess_of`, `identity_on`, `matrices_text`, …). The
teaching-surface test already lists `minilink.dynamics.abstraction.` as a teaching-lane prefix;
the facade joins `TEACHING_SURFACE` and `BAND_FACADES`.

**What `ModelPredictiveController` wraps**, read from `minilink/control/mpc/controller.py`:
it is already a function, `ModelPredictiveController(planner, *, dt_mpc, warm_start=True,
verbose=False, t0=0.0, debug=False)`, returning `MPCStatefulController` (a `StepSystem` whose
state is the last solve's packed `z`, the warm start) when `warm_start` is true, else
`MPCStatelessController` (an algebraic `System`); both share `ModelPredictiveControllerMixin`
(planner, replan clock `dt_mpc`, epoch `t0`, the per-tick solve latch, deploy and debug
methods). All 17 files outside the library and the tests that call it pass `warm_start=True`; the
stateless block is reached only from `test_mpc.py`. **Proposal:** freeze today's signature
as the teaching constructor, with `planner` and `dt_mpc` the promised arguments and
`warm_start`, `t0`, `verbose`, `debug` documented keywords; the two classes and
`MPCBroadcastController` stay internal (off every facade); `Command` stays the return type of
`compute_command`. A factory means `isinstance(mpc, ModelPredictiveController)` is not
available; a class would have to pick one base (`System` or `StepSystem`), which is the step
model's and S76's business in v0.4, so the factory is the smaller freeze.

**A6 names G3 also freezes:** under option A, none of G3's laws change class name (only
`ModelJointImpedance`, `TaskImpedance`, `TaskKinematic`, `TaskKinematicNullspace` lose their
display names); four plants the labs may load are renamed (`LongitudinalSlipCar`,
`LongitudinalTorqueCar`, `KinematicManipulator`, `ThreeLinkManipulator`, plus
`FiveLinkManipulator` for the nullspace demo), so G3's list is pinned after them (R50). Under
option B, `ComputedTorqueController`, `SlidingModeController` and `ImpedanceController` (R49
names all three) are renamed too.

## 5. Open questions for the ruling

1. **The 15th root plant.** The course set is 14; R2 allows 15. A GMC714 arm
   (`TwoLinkManipulator`) is the natural candidate, or leave the slot empty (the labs can
   import from `minilink.catalog`).
2. **Twins on the root.** `KinematicCar` and `DynamicBicycleCar3D` differ from their bicycles
   by the skin alone; `PendulumWithNoisePort` and `CartPoleWithNoisePort` by their noise inputs.
   The tutorial (and one GRO501 notebook for the noisy cart-pole) imports them, so merging them
   into a constructor option changes student-facing material: keep for now, or merge between
   terms?
3. **A6, option A or B**, and the ten class renames of option A (the proposed short names are drafts).
4. **The id rule** (§4.1): fix `normalize_identifier` for digits and acronyms with A6, or keep
   a short explicit display name on those classes.
5. **MPC's one name** (§4.2): the factory frozen as is; its teaching-lane home; and which loop
   syntax the term freezes: `mpc @ plant` (every demo, the GRO501 racecar notebook and the
   tutorial's showcase today; the block carries its own `dt_mpc`) or `mpc % dt @ plant` (as the
   G3 row writes it).
6. **`ModelJointImpedance` on the root** when `JointImpedance(plant)` is the call that builds
   it: both names, or the factory only?
7. **Pinned names with no caller:** `Lag`, `RateLimiter` (blocks), `TabularLearningPlanner`
   (a seed that owes a demo), the three value-function approximators (fold into fields, A1):
   each leaves the teaching-surface test's list only by a ruling.
8. **`QuarterCarOnRoughTerrain` and `OneLinkManipulator`:** the vision keeps the first as
   vehicle content and G1 left the second to the arms lesson; neither has a user.
9. **Demo asks:** the rollout-gradients script vs its topic notebook; `sliding_mode_pendulum`
   as G3's sliding-mode demo or merged into `sampled_smc_pendulum`; `optim_plot`;
   `cartpole_swing_up_rl`; the second engine demo.
