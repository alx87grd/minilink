# Consolidation recap: what stays, what moves, what goes

One table per area of the repository, for the maintainer's choice of a consolidation path
(v0.4, the clean surface). Nothing here is decided; every row is a recommendation.
The size orientations are in ROADMAP §5.4, the demo and tutorial drafts in
[minimal-surface.md](minimal-surface.md) §4.

## 1. How each row was judged

Usage by the current demos is one input among nine, and never the deciding one.

| # | Criterion | What it asks |
| --- | --- | --- |
| 1 | **Course material** | Does a chapter, a lab or a homework of GRO860, GRO501 or GMC714 need it? If yes it stays, used by a demo today or not. |
| 2 | **Direction** | Is it a building block of where the library goes (one problem for every planner, the evaluation pipeline, differentiable everything, certificates, the vehicle ladder)? |
| 3 | **Coupling** | A leaf nobody imports costs almost nothing to keep. A gear many files import costs on every refactor. |
| 4 | **Noise for a reader** | Does it sit in the path of someone learning the library (the top-level names, a package's listing), or out of the way in a catalog? |
| 5 | **Textbook value** | Is it a readable reference implementation of a chapter of the notes (the way `dp.py` is)? |
| 6 | **Trust** | Is it exercised by anything, or is it code that claims to work and never runs? |
| 7 | **Dependency weight** | Does it pull an optional install (SB3, torch, MeshCat, pygame, Plotly, Ipopt) or CI time? |
| 8 | **Duplication** | Is it a second way to do what another part already does? |
| 9 | **Reversibility and timing** | A move after a snapshot tag is reversible; a rename during a term breaks course links. |

**Recommendations:** keep · clean (keep, tidy the code) · minimize the surface (keep the
feature, fewer public names and options) · reduce to core (keep one small feature, drop the rest) ·
move to the teaching repo (public, pinned to a release) · move to the research repo (private) ·
move to the scratchpad (personal, private) · delete.

**Priority:** 1 = do first (safe, large gain in clarity) · 2 = do with the API review ·
3 = when the file is next touched, or leave.
**Confidence:** H = I would do it without asking · M = sound but your call matters ·
L = I miss information; the row says what.

**Measured columns.** Lines: `wc -l`. Use: share of statements run by the kept surface (the
showcase, the tutorials, the nine proposed demos, the 25 course notebooks; minimal-surface.md
§4.8), with its blind spots (MeshCat, pygame, real time, the SB3 notebooks did not run).
Imported by: library files that import the module.

## 2. What the courses need

| Course | Topics (the site, the notes' table of contents, ROADMAP §4) | What it pins in the library |
| --- | --- | --- |
| **GRO860**, reinforcement learning and optimal control (fall) | cost and policy; dynamic programming on a grid; stochastic and minimax control; partial observation and estimation; Bellman equations, value iteration, policy evaluation; LQR and LQG; planning algorithms, receding horizon; Q-learning, SARSA, exploration; function approximation; policy gradient, actor-critic, PPO; Gymnasium and Stable-Baselines3 lab | `PlanningProblem`, the grid and `DynamicProgrammingPlanner`, `LookupPolicy`, the cost-to-go and control-law plots, `lqr`, `kalman`, `lqg`, the tabular learners and exploration rules, the linear approximators and features, REINFORCE, actor-critic, PPO, `NeuralPolicy`, `evaluate` and `compare`, MPC and MPPI, the Gymnasium interface, pendulum, cart-pole, double integrator, drone, quarter-car suspension, the grid world |
| **GRO501**, classical control (the racecar APP) | modelling and block diagrams, linearization, transfer functions; poles, root locus, Bode, margins, step specifications; PID, lead; sensitivity functions; digital implementation; state space, controllability, LQR, pole placement, nested loops, observer and Kalman filter, reference scaling | the whole of `analysis/` except the region of attraction, `control/siso.py`, `state.py`, `design.py`, `estimation/`, `blocks/`, `core/discrete/`, the diagram operators, `KinematicBicycle`, `LongitudinalTorqueCar` |
| **GMC714**, robotics: modelling and control (winter, from January 9) | phase plane; arm kinematics and dynamics; impedance control; computed torque, sliding mode, adaptive control; vehicle modelling with tire slip, the dynamic bicycle; trajectory optimization and RRT | the phase plane, the manipulators and `UR5Manipulator`, `Impedance`, `JointImpedance`, `TaskImpedance`, the task-space kinematic laws, `ComputedTorque`, `SlidingMode`, the vehicle ladder and tires, `TrajectoryOptimizationPlanner`, `RRTPlanner` |

Research on the site that points the direction: low-adhesion and off-road vehicle control,
dimensionless policies, closed-loop stability of learned policies, estimation for inspection
drones, impedance and force control of arms, sampling-based planning.

Three corrections this makes to the usage-only reading of minimal-surface.md §4.8: the
computed-torque and sliding-mode laws (0 % run) are GMC714 week 5; the tabular learners (27 %)
and the linear approximators (0 %) are the centre of GRO860; the region of attraction (0 %)
is the direction of the stability research.

## 3. The library (`minilink/`, 82 100 lines)

### 3.1 Core and compile

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| The `System` family, diagrams, wiring, params, roles, signals, kinematics (`core/system.py`, `diagram.py`, `wiring.py`, `assembly.py`, `closed_loop.py`, `operators.py`, `params.py`, …) | 5 900 | 70–90 % | keep, clean | 2 | H | The gear everything turns on (`system.py` imported by 75 files). Only your API review should move it; the 19 inconsistencies logged in QUESTIONS are its input. |
| The core math objects (`trajectory.py`, `sets.py`, `costs.py`, `distributions.py`) | 2 400 | 68–86 % | keep, clean to textbook | 2 | H | The objects a student reads. The place where "reads like a textbook" matters most. |
| Fields (`core/fields.py`: `Field`, `GridField`, `MarginField`, `QuadraticField`, `EllipsoidSet`, field costs) | 950 | 50 % | minimize the surface | 2 | M | A building block of the direction (cost-to-go as a terminal cost, margins, safety), imported by 14 files. Too many classes for what is used today; decide in the API review which ones are the object and which are helpers. |
| Shaping (`core/shaping.py`: hinge, excess, barrier, occupancy) | 175 | 56 % | keep | 3 | H | Small, and the lane costs of the racecar demos and both research projects use it. |
| Inspection text (`core/inspect.py`, `reporting.py`) | 320 | 27 % | minimize the surface | 3 | M | One formatter per object type, imported by 14 files, mostly not run. Worth one pass: keep `repr` and the notebook display, drop what no object calls. |
| Compile (`core/compile/`: plan, compiler, evaluators) | 3 900 | 57–93 % | keep, clean | 2 | H | A gear: every simulation, planner and gradient goes through it. `evaluators/discrete.py` (846 lines) and `jacobian.py` (57 %) are the two to read first; the three defects in KNOWN_ISSUES live here. |
| Sampled and step systems (`core/discrete/`) | 1 400 | 57–79 % | keep | 3 | H | GRO501's digital implementation and the `%` operator. |
| Facades (`core/facades/`: the methods `sys.plot…`, `sys.animate`, `sys.compute_trajectory`) | 1 700 | 50–63 % | minimize the surface | 2 | M | The first thing a student types. Half of each file is options nobody passes; the API review decides which keywords stay. |
| Paths and shapes (`core/geometry/`) | 600 | 62–80 % | keep | 3 | M | Tracks and obstacles need it; its plan doc (geometry-module.md) is open. |

### 3.2 Analysis, control, estimation, blocks

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Linear analysis (`linearization`, `linear`, `frequency`, `time_response`, `sensitivity_functions`, `structural`, `riccati`, `derivatives`, `equilibria`, `discretization`) | 3 300 | 75–100 % with the course notebooks | keep | 3 | H | All of GRO501. Leaves, well tested, low upkeep. |
| Region of attraction (`analysis/lyapunov.py`) | 910 | 0 % | **reduce to core** | 1 | H | Your call already: keep `region_of_attraction`, the certificate and its plot; drop the sampling variants, the reports and the windowing helpers that surround them. A target near 250 lines. A leaf (nothing imports it), so the cut is safe. |
| Modal analysis (`analysis/modal.py`: `poles`, `modal_analysis`, `animate_modal`) | 220 | 0 % | keep | 3 | M | A small leaf, a classic of vibrations teaching, and `poles` is used by GRO501. No course of the three animates modes today; if none will, it goes to the scratchpad. |
| Classical and state-space laws (`control/siso.py`, `state.py`, `design.py`) | 1 200 | 86–96 % | keep | 3 | H | GRO501. |
| Arm laws (`control/impedance.py`, `robotic.py`) | 770 | 63 % | keep | 3 | H | GMC714 weeks 4–5 and the UR5 demo. |
| Computed torque and sliding mode (`control/modelbased.py`) | 270 | 0 % | keep | 3 | H | GMC714 week 5. Unused by the kept demos, needed by the course. |
| Path-tracking laws (`control/geometric.py`: pure pursuit, Stanley, line of sight, heading) | 620 | 0 % | move to the research repo | 1 | M | You called path tracking research. Four public names at the top level for a topic no course teaches. They travel with the path-tracking project that compares them. If one stays as a teaching example, keep pure pursuit alone. |
| Neural policy (`control/neural.py`) | 340 | 70 % | keep | 3 | H | What every learner returns. |
| Gaussian policies (`control/stochastic.py`) | 260 | 0 % | minimize the surface | 2 | L | Imported by two library files, run by nothing. I have not traced whether the learners still need it or `NeuralPolicy` replaced it; if replaced, delete. |
| Observers (`estimation/`) | 375 | 87–92 % | keep | 3 | H | GRO501, the LQG chapter of GRO860, the estimation research. |
| Signal blocks (`blocks/`) | 1 230 | 48–100 % | keep | 3 | H | Small leaves, the vocabulary of diagrams. |

### 3.3 Dynamics catalog

A catalog plant is a leaf: nothing depends on it, and it is out of a reader's path. The
question for each is trust and size, not use.

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Abstractions (`state_space`, `mechanical`, `manipulator`) | 1 070 | 67–72 % | keep | 3 | H | The base classes of the catalog. |
| `GeneralizedMechanicalSystem` | 230 | 17 % | keep, check | 3 | L | Three library files use it; I have not checked which plants need it over `MechanicalSystem`. If one or none, fold it. |
| Pendulum, cart-pole, double pendulum | 1 250 | 16–62 % | keep | 3 | H | The teaching plants of all three courses. A third pendulum file is a leaf. |
| Mass-spring-damper, integrators, oscillators | 720 | 23–48 % | keep | 3 | H | The README's example, the double integrator of GRO860, Van der Pol and Lorenz for the phase plane. |
| Manipulators (`arms.py`: one to five links, kinematic; `ur5.py`) | 1 650 | 13 % / 82 % | keep, clean `arms.py` | 2 | H | GMC714 weeks 2–5. `arms.py` is 1 040 lines and the file a robotics student opens to see `M`, `C`, `g`: the best candidate in the catalog for a textbook pass. |
| The vehicle ladder (kinematic bicycle, dynamic bicycle, drivetrain, racecar, tires, terrain) | 2 400 | 70–92 % | keep | 2 | H | GRO501, GMC714 week 6, the vehicle research. Its open design points are in QUESTIONS. |
| The 3-D car (`suspension_car.py`, presets, 3-D skins) | 1 750 | 22 % | keep, marked as a prototype | 3 | M | The direction of the off-road research and the game demo. Valid on mostly flat ground only (stated in its docstring). If it stops growing, it belongs in the research repo. |
| Longitudinal cars (`propulsion.py`), quarter car (`suspension.py`), mountain car | 630 | 17–23 % | keep | 3 | H | GRO501 and GMC714 week 6; the active-suspension exercises of GRO860; a classic RL problem. Small leaves. |
| Drone, rocket | 550 | 62 % / 20 % | keep | 3 | H | The RL demos and the project page. The rocket-landing training does not learn here: a known issue, not a reason to drop the plant. |
| Planes (`aerial/plane.py`: 2-D and 3-D) | 840 | 10 % | reduce to core | 2 | L | The largest unused plant. I do not know whether "learning to fly" in GRO860 uses the plane or the drone. If the plane: keep the 2-D one, move the 3-D one to the scratchpad. If the drone: move both. |
| Boat and its skin, three-body | 840 | 0 % | move to the scratchpad | 1 | M | No course, no research project on the site, no demo in the kept set. Cheap leaves, so low gain; the reason to move them is trust (never run) more than size. |

### 3.4 Planning, learning, optimization

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| The planning framework (`problems.py`, `evaluator.py`, `evaluation.py`, `comparison.py`, `results.py`, `planner.py`, `environment.py`, `step_model.py`, `step_problems.py`, `policies.py`) | 6 100 | 52–86 % | keep, clean; own subpackage | 2 | H | The direction of the library and just built; normal that few demos use `evaluate` and `compare` yet. `evaluator.py` (1 409 lines) and `problems.py` (1 248) are the two largest files of the library and the first to split. The subpackage question is in QUESTIONS. |
| Value iteration and LQR, the reference (`dp.py`, `discretizer.py`, `lookup_policy.py`, `lqr.py`, `progress.py`) | 2 600 | 54–80 % | keep | 3 | H | GRO860's centre and the textbook reference of the code base. |
| Value iteration on JAX (`dp_jax.py`) | 450 | 0 % | keep, and put it under a test and a demo | 1 | M | The fast path of the same algorithm, run by nothing in the kept surface. Keeping untested speed code beside the reference is the risk; one demo (the cart-pole swing-up table) and the benchmark settle it. |
| Linear approximators and features (`approximation.py`) | 200 | 0 % | keep | 3 | M | GRO860 week 8. The course notebook builds its own by hand today; the library version is the tidy one a later term can use. |
| Planner plots (`dynamic_programming/plotting.py`, `planning/plotting.py`, `spatial/plotting.py`, `search/plotting.py`, `mpc/viz.py`) | 2 700 | 35–51 % without the courses | minimize the surface | 2 | M | Five plotting files, a third of the planning code by volume, each with options the courses do not pass. One pass per file with the notebooks open. |
| RRT and RRT* (`planning/search/`) | 1 900 without plots | 81–96 % | keep | 3 | H | GMC714 week 7. Dubins steering (115 lines, 9 %) is a leaf worth keeping for cars. |
| Scenes, tracks, collision (`planning/spatial/`) | 1 000 without plots | 58–73 % | keep | 3 | H | Every obstacle and racecar demo, both research projects. |
| Trajectory optimization and MPC (`planner.py`, `direct_collocation.py`, `shooting.py`, `transcription.py`, `compiled.py`, `mpc/predictive.py`, `path_integral.py`) | 3 800 | 66–95 % | keep | 3 | H | GMC714 week 7, GRO860 chapters 12–13 and 21, every racecar demo. |
| Multiple shooting | 160 | 0 % | keep | 3 | M | A leaf the roadmap names for GRO860; one line in a tutorial would run it. |
| Policy-gradient learners (`planner.py`, `ppo.py`, `reinforce.py`, `actor_critic.py`, `policy.py`, `critics.py`, `collect.py`) | 2 100 | 23–98 % | keep | 3 | H | REINFORCE and actor-critic are chapters 18 and 19 of the notes: textbook value beyond their use. |
| SAC (`sac.py`) | 200 | 14 % | keep | 3 | M | The advanced chapter and the project page name it. A leaf; drop it only if it fails where PPO works. |
| Tabular learners (`tabular.py`: Q-learning, SARSA, Monte Carlo, ε-greedy, UCB) | 660 | 27 % | keep | 3 | H | GRO860 chapters 14 to 16. |
| Gymnasium and SB3 interface (`interfaces/gymnasium.py`) | 420 | 38 % | keep | 3 | H | GRO860 week 5 is a lab on exactly this. |
| Optimization boundary (`mathematical_program.py`, `optimizer.py`, evaluators, first-order, SciPy) | 2 550 | 60–100 % | keep | 2 | H | The second pillar of the showcase. `optimizer.py` (754 lines) is the one to clean. |
| Ipopt backend | 110 | 0 % | keep | 3 | M | An optional leaf; the conda stack installs it. Untested on this machine. |

### 3.5 Simulation and graphics

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| Simulator and solvers | 1 900 | 42–86 % | keep | 3 | H | A gear, stable. |
| Static simulator, clock runner | 270 | 0 % | delete or fold | 2 | L | Nothing in the kept surface runs them; tests do. I have not checked which user call reaches them. |
| Real time and games (`simulation/realtime/`, the pygame renderer) | 1 700 | not measured | keep, and fix the two-window bug first | 2 | M | Games are the best first contact for a student. The Matplotlib-plus-pygame bug you named decides whether it deserves its place. |
| Animation core (animator, camera, primitives, drawables, the Matplotlib renderer) | 2 600 | 24–97 % | keep | 3 | H | Every `animate()`. |
| MeshCat renderer | 1 110 | not measured | keep | 3 | M | The 3-D view of the UR5 and the cars. The largest renderer; hangs without a browser (known issue). |
| Plotly: the animation renderer (739 lines) and the two plot backends (570 lines, 0 %) | 1 300 | 54 % / 0 % | reduce to core | 1 | M | A second way to draw what Matplotlib draws. Keep the renderer if Colab animations rely on it; delete the two plot backends nothing selects. |
| Skins (`racecar_skin.py` 1 328 lines, `car_skins_3d.py`, `skins.py`, `shapes.py`) | 2 700 | 11–83 % | clean | 3 | M | Drawing code, leaves, but the racecar skin alone is larger than the vehicle ladder's dynamics. Worth shrinking when touched, not before. |
| Mesh tools (`graphical/meshes.py`: STL and Collada loaders, weld, simplify, refine) | 610 | 45 % | minimize the surface | 3 | M | Needed for the UR5; the simplify and refine tools look like one-time build tools. |
| Diagram pictures, phase plane, control-law maps, time signals, control figures | 4 700 | 45–97 % | keep | 3 | H | Used by all three courses. `port_map.py` (740 lines) is the one to read with the notebooks open. |

### 3.6 Experimental

| Chunk | Lines | Use | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- | --- |
| C export, the contact and tire engines, the world (`experimental/c_export.py`, `engines/`) | 1 400 | 0 % | move to the research repo | 1 | H | Research prototypes with their own tests; nothing on the kept surface touches them. One course notebook mentions `experimental`: check it before the move. |
| Symbolic mechanics (`experimental/symbolic/`) | 1 100 | 0 % | move to the research repo | 1 | M | Derives `M`, `C`, `g` symbolically, which is GMC714 week 3 done by hand today. If you want it in a lab one day, it returns as a finished feature. |

## 4. Outside the library

| Chunk | Size | Recommendation | Pri. | Conf. | Why |
| --- | --- | --- | --- | --- | --- |
| Research projects (`examples/projects/`: car trajectory optimization, MPC, path tracking) | 31 files, 9 100 lines | move to the research repo | 1 | H | More code than all demos together, read-only for agents, and it subclasses catalog classes, so it blocks renames. Pinned to a release in its own repository it stops constraining the library. |
| The old demo folders (`examples/demos/`, 94 scripts) | 5 800 lines | delete after a snapshot tag; the keepers are rewritten | 1 | H | Replaced by the curated set. The good ideas are listed in minimal-surface.md §4.4 and §4.7; your own favourites go to the scratchpad first. |
| The proposed demos (`examples/demos_proposed_minimal/`, 9 scripts) | 820 lines | keep; becomes `examples/demos/` once you choose | 1 | M | Waits for your selection, and for the choice between the two racecar cascade demos. |
| Course notebooks (`examples/teaching/courses/`, 20) | 3 200 code lines | move to the teaching repo, between terms | 2 | H | They pin a release there, and the library is free to rename. GRO860 runs now and GMC714 starts January 9: move in the December gap, with the links in the course PDFs in the same change. |
| Topic notebooks (`examples/teaching/topics/`, 5) | 455 code lines | move to the teaching repo | 2 | M | Same audience as the course notebooks. |
| Showcase, basics, advanced tutorials | 9 notebooks | keep | 3 | H | Just brought up to date; one per audience. |
| Core notebooks (`examples/tutorial/core/`, 8) | 870 code lines | keep until the API review, then turn into the developer docs or delete | 2 | H | Your working documents for that review. |
| Unit tests (`tests/unittest/`, 77 files) | 40 300 lines | reduce, by rewriting around contracts | 2 | M | Half the size of the library. Six files over 1 500 lines. Not a deletion: each file is rewritten when its module passes the API review, keeping the contract tests and the textbook checks. |
| Vocabulary test (`test_vocabulary.py`) | 1 055 lines | minimize | 2 | M | Guards the public names; shrinks with the surface. |
| Tests of experimental code (`tests/experimental/`) | 430 lines | move with the code | 1 | H | |
| Demo and notebook checks (`tests/demo_checks/`, `tests/run/`) | 1 600 lines | keep | 3 | H | The gate that kept the tutorials honest in this run. |
| Benchmarks (`benchmarks/`, 27 files) | 6 000 lines | reduce to core | 2 | M | Keep the regression gate CI runs and the speed claims the README makes (four suites). The scenarios and host profiles beyond that are the scratchpad's. |
| Developer docs (`docs/dev/`) | 4 400 lines | reduce | 2 | M | DESIGN and RULES are law; ROADMAP and QUESTIONS shrink as v0.4 closes. |
| Plan docs (`docs/plans/`, 13) | 4 200 lines | delete the landed ones, keep the open ones | 1 | H | The vehicle run log and the pyro port map describe finished work. TODO stays. |
| API docs (`docs/api/`, Sphinx) | 520 lines | keep | 3 | H | Generated from docstrings. |

## 5. Read by destination

| Destination | What goes there | Lines leaving the library |
| --- | --- | --- |
| **Research repo** (private) | the three projects; the experimental package and its tests; the path-tracking laws | about 3 100, plus 9 100 of projects |
| **Teaching repo** (public, pinned) | the 20 course notebooks and the 5 topic notebooks | none (examples only) |
| **Scratchpad** (personal) | the old demos worth a second look; boat, three-body, the 3-D plane; the extra benchmark scenarios | about 1 400 |
| **Deleted** | the remaining old demos; the two Plotly plot backends; the landed plan docs; the static simulator if unused | about 850 |
| **Reduced in place** | the region of attraction; the plotting files; fields, facades, inspection; the skins | about 3 000 to 4 000, an estimate, not a measure |
| **Kept as is** | everything a course names, the planning framework, compile, the catalog's teaching plants | |

The library would land near 73 000 lines from these rows alone. The 55 000 to 60 000
orientation of ROADMAP §5.4 is reached only by the cleaning pass of the large files
(`evaluator.py`, `problems.py`, `dp.py`, `arms.py`, `racecar_skin.py`, the plotting files), which
is the API review's work, not a move.

## 6. A suggested order

1. **Tag a snapshot**, so every later move is reversible.
2. **Moves that touch no kept code** (priority 1, high confidence): the projects, the
   experimental package, the old demo folders, the landed plan docs.
3. **The small reductions** (priority 1): the region of attraction to its core, the Plotly
   plot backends, the path-tracking laws, value iteration on JAX under a test.
4. **The course notebooks**, in the December gap between terms.
5. **The API review**, module by module, with the test file of each module rewritten in the
   same step (priority 2 rows).
6. **Priority 3 rows**: only when a file is opened for another reason.

## 7. What I need from you

- **The planes:** does "learning to fly" in GRO860 use `Plane2D` or the drone?
- **Modal analysis:** will a course animate vibration modes?
- **Path-tracking laws:** all to the research repo, or pure pursuit stays as a teaching law?
- **Plotly:** do the Colab animations of the courses rely on the Plotly renderer?
- **Symbolic mechanics:** research repo, or a future GMC714 lab feature?
- **The 3-D car:** stays in the catalog as a marked prototype, or moves with the research?
