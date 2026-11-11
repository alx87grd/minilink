# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

A ruling the maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan
doc it shapes) in the same change; the commit message carries its why. It then leaves this
file. The core's architecture is of record: the v0.3 rulings and the v0.4 rulings of
2026-11-04 to 2026-11-11 live in DESIGN and RULES. Open here: four small names the API
settlement (PR #139) left, three questions on the car ladder, and one question on the vision.

## The API

### Four small names the API settlement left
- **Context:** the API settlement before the demo rebuild (PR #139, merged 2026-11-11) logged
  four names it did not touch, because no ruling listed them; each is a public name, so each
  waits for you. They are small and independent; the rebuilt demos would meet the
  last two.
  1. `SampledDiagramEvaluator.block_input(path, x, u, k)`, a public method of the compiled
     evaluator (DESIGN §4, one test, the MPC drawing helper): it reads a subsystem's input,
     where ruling 1 made "subsystem" the word for a system placed in a diagram.
  2. `TrajectoryOptimizationPlanner.transcription_options()`, the verbose print's line: it
     names the option records the settlement deleted (the transcription now takes
     `n_steps=` itself) and returns `{"n_steps": N}`; its only caller is the print.
  3. `animate_control_law(axis=0, ...)` (value iteration's planner and
     `planning/dynamic_programming/plotting.py`): the input channel drawn, the idea
     `plot_control_law` now names `u_axis=`.
  4. The planners' `compile(backend=None)` (trajectory optimization, MPPI): `backend` stays
     positional (the MPC calls `planner.compile(backend)`), where `System.compile(*, backend=)`
     is keyword-only.
- **Options**, per name:
  1. `block_input` → `subsystem_input` (one library caller, one test), or kept as is.
  2. `transcription_options()` → `transcription_numbers()`, folded into the print (no method
     left), or kept as is.
  3. `animate_control_law(axis=)` → `u_axis=`, as `plot_control_law`; or kept as is.
  4. `compile(*, backend=None)` keyword-only, as the system's (two callers in the MPC), or
     kept positional.
- **Recommendation:** rename 1 and 3 and make 4 keyword-only (one word, one shape per idea,
  RULES 2.7 and DESIGN §4); fold 2 into the print, since nothing else reads it.
- **Blocks:** nothing; each is one commit with its callers, no alias.
- **Answer:**

### The 3-D car's rear axle: locked, or an open differential?
- **Context:** the 3-D car (`SuspensionCar3D`, the last rung of the car ladder, in review on
  `vehicle-models-phase1`) drives both rear wheels at one rate `w_rear`, a locked axle (ruled
  2026-11-11). Measured on the built car against the drivetrain bicycle with the same numbers,
  steady yaw rate at a steer of 0.01 rad, 5 and 15 m/s: 65 % of the bicycle's on the default
  car, 79 % on the off-road preset, 88 % on the formula preset. The two rear tires slip against
  each other and resist the yaw, like about 1 m more wheelbase on a 2 m car. The reduction test
  passes because it narrows the track to 0.3 m. So "the two rungs are the same car" and "a
  planar MPC drives the 3-D car" do not hold at the default track.
- **Code, today and under option 1:**
  ```python
  # Today: both rear wheels at the axle's rate
  w_wheel = vx_w[i] / r_wheel if front else w_rear
  # Open: each rear wheel at its own rate, w_rear their mean (side = +1 left, -1 right)
  w_wheel = vx_w[i] / r_wheel if front else w_rear + side * 0.5 * dw_rear
  ```
- **Options:**
  1. *An open differential by default.* Each rear wheel turns at its own rate, the drive torque
     split evenly; `w_rear` stays the axle's mean rate, so the first nine states keep their
     meaning (plan §4 already designs for it). The 3-D car then reduces to the bicycle at any
     track. Costs: the two rates are either two more states, or solved from the even torque
     split inside `f`.
  2. *Keep the locked axle,* and say in the class docstring and the tests that the car steers
     less than the bicycle, with the numbers above. Costs nothing; the model-mismatch study
     starts with a mismatch that is not the suspension's.
  3. *A `locked_axle=` option,* open by default. Both behaviours, one more structural switch.
- **Recommendation:** option 1: it is what makes the 3-D car the bicycle's limit, which the
  ladder promises.
- **Blocks:** the planar-controller-drives-the-3-D-car demo, and the presets' final numbers.
- **Answer:**

### Which point does the kinematic bicycle's `(x, y)` follow?
- **Context:** `KinematicBicycle.f` writes `ẋ = v cos ψ`, `ψ̇ = v tan δ / L`, the equations of
  the rear axle's centre. Its frames draw `(x, y)` as the centre of gravity, `b` ahead of the
  rear axle, as on the dynamic rungs, where `(x, y)` is the centre of gravity. The ladder's
  rule is that a state keeps its meaning from rung to rung. This predates the vehicle branch.
- **Options:**
  1. *The centre of gravity,* with the sideslip `β = atan(b tan δ / L)`:
     `ẋ = v cos(ψ + β)`, `ψ̇ = v cos β tan δ / L`. The same point on every rung; one more line
     in the first model a student reads, and the numbers of every kinematic demo move.
  2. *The rear axle, drawn there:* `f` unchanged, the body frame's drawing moved back by `b`.
     The simplest textbook model stays; `(x, y)` changes meaning between rungs 2 and 3, said
     in both docstrings.
  3. *Leave it,* and state the approximation in the docstring.
- **Recommendation:** option 2: rung 2 is the no-slip lesson and its page should stay three
  lines; the drawing and the docstring then tell the truth.
- **Blocks:** nothing.
- **Answer:**

### The public face of the car's parts
- **Context:** the tires, the springs and the terrains are parts a car is built from. Three
  small inconsistencies, each a public name or call:
  1. tires and springs take their numbers at construction (`BrushTire(mu=1.6)`); terrains do
     not (`Bumps()`, then `car.params["terrain"]["amplitude"] = 0.8`);
  2. the parts are imported from their modules
     (`from minilink.dynamics.catalog.vehicles.terrain import Bumps`), which the demos'
     import check allowlists; the cars come from `minilink`;
  3. `TerrainBlock` (ruled: "also exported as a static block") has no car port to wire to:
     no car outputs its contact positions or takes ground heights.
- **Options,** per item:
  1. `Bumps(amplitude=0.8, wavelength=30.0)`, as the tires; or kept.
  2. the parts on `minilink.catalog` beside the cars; or kept in their modules.
  3. a `contacts` output on the 3-D car for the block; or the block waits for a diagram that
     needs it; or kept as it is.
- **Recommendation:** constructor numbers on the terrains and the parts on `minilink.catalog`
  (one way to build a car, one import line); the block waits.
- **Blocks:** nothing; each is one commit.
- **Answer:**

## The vision

### Where is the fair comparison stated?
- **Context:** the maintainer's direction of 2026-11-09 (ROADMAP §5): one problem handed to
  every planner and one judge for every answer is the toolbox's distinctive contribution, to be
  promoted in the vision. Today CONSTITUTION §1.1 names the three foundations and calls every
  approach "a tool on that path", without saying the approaches are compared on it; the README
  shows it in §7 of 9, after the model, the wiring, the graphics, JAX and the optimizer.
- **Proposed wording, CONSTITUTION §1** (a new item after "One toolbox, one model"):
  "**One problem, one judge.** Every approach takes the same `PlanningProblem` and returns a
  `PlanningSolution`; every answer is scored by the same evaluation, on the same model, cost,
  sets, start law and disturbances. A method is never scored by a judge of its own."
- **Options:**
  1. *CONSTITUTION §1 and the README's lead.* The item above, and the README's first example
     the comparison table. Costs: the README leads with the advanced tool, before a student's
     first simulation.
  2. *CONSTITUTION §1 only, the README's order kept.* The item above; README §1's backbone
     paragraph gains one sentence and §7 is fixed. The teaching order stays.
  3. *A governing principle (CONSTITUTION §4) instead of the vision.* "One judge for every
     planner" beside "One set of tools for every System": a rule for reviews, not an identity.
- **Recommendation:** option 2 with the principle of option 3 as well: the identity in §1, the
  review rule in §4, and the README's teaching order kept until its example wins.
- **Blocks:** nothing in the code; the README's wording waits on it.
- **Answer:**

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open. No alias, one sweep, holds meanwhile.
- **Your answer (2026-11-03):** the pending renames land before the freeze, one sweep per
  group, under the vocabulary of RULES 2.7; so the freeze itself needs no shim.
- **Blocks:** the trust contract (V4), for the names changed after v1.0.
- **Today:** no alias or shim ever. The frozen names are drawn at v0.9 from RULES 2.7.
- **Options still open (after v1.0 only):** a shim of some length; a major version per rename;
  a snapshot test of the export list either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
