# Core consolidation: what is core, what is a side feature

**Status:** analysis and recommendations, 2026-10-20 (second pass, by the vision). Asks 1–3,
5 and 6 of §10 ruled 2026-10-22 (R1–R5 of
[the rulings](../reviews/2026-10-22-rulings-session-1.md); the shape amended: three
foundations, model, problem and program, with six tooling pillars); ask 4 open. The tiers'
item-by-item ruling (K7) is still to come from K1's table.  
**Lane:** maintainer-owned (every item removes or moves a user-importable name, so each is
an ask under AGENTS.md; the inventory and the tiers are the agent's analysis).  
**Rung:** v0.3, wave K: the convergence pass (step K7 of [TODO.md](TODO.md) with K1–K4, an
open decision in ROADMAP §6), which the roadmap calls a standing principle, not a rung (ROADMAP §5.6,
"Simplify and consolidate"); it fits between the GMC714 audit (step G1) and the freeze
candidate (ROADMAP §5.5), because the v0.9 freeze should cover a smaller surface than
today's.  
**Companion:** [repository-structure.md](repository-structure.md) decides *where* things
live; this document decides *what* stays in the library at all.

## 1. Why: divergence, then convergence

The last two releases were a divergence phase on purpose: build every tool of the toolbox,
from LQR to PPO through trajectory optimization, to see the whole picture. It worked, and it
left a library of **70.7k lines**, a root prelude of **162 names**, **50 catalog plants**
on that prelude, **76 demo scripts**, **42 notebooks** and **28k lines of tests**. The v0.9
freeze (ROADMAP §5.5) promises every teaching-surface name stability for three years. The
smaller that surface is when the freeze lands, the cheaper the promise.

Two lenses sort the library. The first pass (§3) asked **who uses it today**; that lens is
biased toward the course taught this fall, so it is evidence, not the verdict. The second
pass (§4) asks **which pillar of the long-term framework stands on it**. The verdict (§5,
§6) comes from the second lens; the first says how cheap a move is.

## 2. The vision, as the bar

One framework for dynamics and control, continuous time at the center, on JAX, covering
every approach that controls a vehicle or a robot today: classical control, modern
(state-space) control, nonlinear control, trajectory optimization and MPC, reinforcement
learning, and later imitation (behavior cloning). The constitution says the same in its
first line ("one toolbox, one model") and bounds it (§1.4: not a block-diagram editor, not
a multibody engine, not a modeling language, not a set library).

Read as an architecture, that vision has one spine and six pillars on it:

**The spine** — one differentiable path every approach is a special case of:

```
System  →  compile  →  rollout  →  Trajectory  →  cost / margin  →  MathematicalProgram  →  Optimizer
   p (plant, controller, cost, x0, draws)                                       ∂J/∂p
```

A pillar is a *problem family* on that spine, named by which parameters it optimizes and
against what:

| Pillar | What is optimized | Against | Lives today in |
| --- | --- | --- | --- |
| classical and modern control | nothing by search: closed-form designs (`lqr`, `place`, `kalman`, `lqg`, PID tuning rules) on the linearization | specifications (poles, margins, bandwidth) | `analysis/`, `control/design.py`, `control/siso.py`, `estimation/` |
| nonlinear control | hand-designed laws (computed torque, sliding mode, impedance, geometric), certified by a Lyapunov function | the model's structure | `control/modelbased.py`, `impedance.py`, `robotic.py`, `geometric.py`, `analysis/lyapunov.py` |
| trajectory optimization and MPC | the open-loop plan (knots) | the problem's cost and sets; MPC re-solves it online in a sampled loop | `planning/trajectory_optimization/`, `control/mpc/`, the hybrid sibling |
| reinforcement learning | a stochastic policy's weights | sampled returns of the stochastic problem | `planning/reinforcement_learning/`, `planning/dynamic_programming/` (the DP reference), `control/neural.py`, `blocks/neural.py` |
| identification and imitation (planned) | plant parameters, or a policy's weights | logged trajectories: measurements, or human demonstrations | `identification/` (stub), `simulation/realtime/` (the recorder), the RL planner's rollout and data path, `NeuralPolicyController` |
| estimation | nothing by search: observer designs; later the EKF | the measurement model | `estimation/` |

Around the spine, two kinds of content the vision names: **the plants it is about**
(vehicles and robots, plus the textbook benchmarks every course uses: pendulums,
cart-poles, integrators, a mass-spring) and **the presentation** (diagrams, signals,
animation, phase plane, the control-law map).

That gives the test of this pass. A module is **core** when it is on the spine, is one
pillar's home, or is the named content; it stays even if no course uses it yet. A module is
a **side feature** when it is none of those: another domain's plant, an interop bridge, an
exporter, a second way to do what the spine already does. It leaves the core even if a demo
uses it, or stays small and off the surface.

## 3. The evidence: who uses what today

The inventory counts, for every module and every facade name, the files that use it in
seven places: the course notebooks, the topic notebooks, the tutorial, the demos, the
research examples (`projects/`, `experimental/`), the tests, and the library outside the
defining module. The two scripts and their full tables are in the session's scratchpad.
What it says:

- Of the 162 prelude names, **39 are used only by tests** and **about 65 appear in no
  notebook**. A course notebook imports about 60 distinct names; the tutorial about 90.
- Of the 50 catalog plants on the prelude, 14 are in a notebook, 10 in a demo only, 2 in a
  research script only, **24 in nothing**.
- The courses stand on: the core objects, `Simulator`, the hybrid loop once, the analysis
  verbs (all of GRO501's), the design verbs, the PID family, DP, trajectory optimization,
  the RL planner with PPO and SAC, `compare`, `MonteCarloEvaluator`, MPC once, diagrams,
  signal plots, animation, phase plane, the control-law map, the gymnasium bridge (two SB3
  twin notebooks that retire at v1.0, S49).
- Used by nothing: the Lyapunov certificate (one demo), tabular RL, the value-function
  approximators (the course lesson defines its own in a cell), PurePursuit, the Mermaid
  exporters, the camera helpers, the pygame renderer, `ProblemEnv` / `to_gymnasium`, the
  identification stub, and the 24 plants.

This fall's course is over-represented: the usage lens would keep the SB3 bridge and the
racecar 3-D skin (a GRO501 notebook imports each) and drop PurePursuit, the Lyapunov
certificate and the realtime loop. The vision lens says the opposite on all five. Hence §4.

## 4. The second pass: every module by its role in the vision

Verdicts: **spine** (invest; the framework is this), **pillar** (the home of one approach;
keep and finish, even unused today), **content** (named plants and presentation; keep
small), **side** (leave the core: experimental tier, the example that motivated it, or git
history). The last column says where the usage lens disagreed.

### 4.1 Framework

| Module | Role in the vision | Verdict | Usage lens |
| --- | --- | --- | --- |
| `core/system.py`, `diagram.py`, `wiring.py`, `composition.py`, `feedback.py`, `signals.py`, `backends.py` | the model and its algebra | spine | agrees |
| `core/compile/` (the evaluators, the step compiler) | the rollout; JAX is the accelerator | spine | agrees |
| `core/trajectory.py`, `sets.py`, `costs.py`, `distributions.py`, `geometry.py` (+ `planning/spatial/` → `core/geometry/`, A3) | the textbook objects every pillar speaks | spine | agrees |
| `core/hybrid_composition.py`, `hybrid_diagram.py`, `step_rollout.py`; `simulation/computer.py`, `hybrid_simulator.py`; `blocks/step.py` | the sampled loop: every real controller, learned or optimized, runs at a sample rate on a continuous plant. MPC and a deployed policy need it; so does a recorded demonstration | spine (provisional in shape; S31's recommended option: the loop as a tool, one sampled kernel in two faces) | agrees |
| `core/facades.py` (32 verbs on `System`) | the student's one-line entry to every pillar | spine; trim the verbs no pillar needs (`get_diagram`, `plot_input_output_map`, `render`) | agrees |
| `core/kinematics.py` (SE2 / SE3) | pose algebra for robots and vehicles | spine | agrees |
| `core/inspect.py`, `options.py` | `print` on every noun; option checks | substrate | agrees |

### 4.2 Simulation

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `simulation/simulator.py`, `solvers/`, `time_grid.py`, forcing | integrating time | spine | agrees |
| `simulation/static_simulator.py` | a memoryless leaf on a grid | substrate, off the prelude | agrees |
| `simulation/realtime/` (`RealtimeSimulator`, the I/O contract, `PygameInput`, the live plot) and `game()` | **the human in the loop**: the recorder of the imitation pillar (ROADMAP §5.6 names it: "a game mode on `RealtimeSimulator` that records human demonstrations as trajectory datasets") and the only path that shows a controller *feels* right | **pillar seed**: keep in the core, rebuild under the architectural review the ledger already schedules; its next feature is the dataset it records, not more input devices | **disagrees** (TRL 2, three demos: the usage lens moved it) |

### 4.3 Analysis and estimation

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `linearization`, `derivatives`, `structural`, `equilibria`, `frequency`, `time_response`, `sensitivity_functions`, `discretization`, `linear.py`, `riccati.py` | the classical and modern pillar's toolbox; `discretize` is also the digital-implementation step every pillar's controller takes | pillar | agrees |
| `analysis/modal.py` | structural vibration | side-ish; 182 lines, keep off the prelude | agrees |
| `analysis/lyapunov.py` | **the certificate of the nonlinear pillar** and, later, of a learned policy (a region of attraction of a neural controller is the question the RL pillar cannot answer by itself); `V(x)` is the first `Field` the constitution lists | **pillar**: keep the concept, rebuild it as a `Field` verb when fields land (A1); until then it is a 931-line provisional implementation with one demo | **disagrees** (the usage lens moved it) |
| `estimation/` (`StateObserver`, `luenberger`, `kalman`, `lqg`) | the estimation pillar; a real vehicle or robot never has its state | pillar; the EKF and the trajectory observer are its next rows | agrees |

### 4.4 Control laws

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `control/design.py`, `siso.py`, `state.py` | the classical and modern pillar's laws | pillar | agrees |
| `control/modelbased.py`, `impedance.py`, `robotic.py` | the nonlinear pillar's laws for arms (computed torque, sliding mode, joint and task impedance, kinematic and nullspace laws) | pillar; `ModelJointImpedance` and the law count are G1's call | agrees |
| `control/geometric.py` (`PurePursuit`) | **the vehicle pillar's baseline path tracker**: every vehicle stack compares its MPC or learned policy against pure pursuit (Stanley is the other); the geometry module (A3) is built so this law can take a `Track` | **pillar**: keep; it needs the vehicle lesson (GMC714) that makes it visible, and it is the first consumer of `core/geometry/` | **disagrees** (no user today: the usage lens moved it) |
| `control/neural.py` (`NeuralPolicyController`), `blocks/neural.py` (`MLP`, `NeuralNetwork`) | the output of the RL pillar and of the imitation pillar: the one learned law, deployed in the same diagram as any other | spine (shared by two pillars) | agrees |
| `control/mpc/` | the trajectory pillar online; the state of the art for vehicles and arms; the hybrid sibling's exemplar | pillar; `utilities.py` and `viz.py` are internal (lose their band names) | agrees (G1 decides teaching surface or research lane; the vision says it is a pillar either way) |

### 4.5 Planning

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `planning/problems.py`, `results.py`, `evaluation.py`, `comparison.py`, `planner.py`, `initial_guess.py` | the problem, the solution, the evidence, and the side-by-side that *is* the unification's demonstration (VI vs LQR vs PPO on one problem) | spine | agrees |
| `planning/trajectory_optimization/` | the trajectory pillar | pillar | agrees |
| `planning/dynamic_programming/dp.py`, `dp_jax.py`, `discretizer.py`, `lookup_policy.py`, `policy_eval.py`, `lqr.py` | dynamic programming: the reference every other pillar is checked against; `dp.py` is the textbook of the code style | pillar | agrees |
| `planning/dynamic_programming/approximation.py` | value-function approximation: the rung between the lookup table and the neural critic, the cost-to-go as a `Field` | pillar concept, orphaned implementation: fold into the fields plan (A1) rather than move; the course lesson that teaches it carries its own code today, and whether it adopts the library's is the maintainer's pedagogical call | **disagrees in direction** (the usage lens moved it out; the vision folds it in) |
| `planning/reinforcement_learning/` (`planner.py`, PPO, SAC, REINFORCE, actor-critic, `policy.py`, `critics.py`, `collect.py`, `environment.py`, `optim.py`) | the RL pillar; PPO and SAC are the two modern algorithms, REINFORCE and the actor-critic the textbook ladder; `collect.py` and `environment.py` are also the data path imitation will reuse | pillar | agrees |
| `planning/reinforcement_learning/tabular.py` (Q-learning, SARSA, Monte Carlo control) | the rung between DP on a grid and deep RL: the textbook's own order | pillar rung without its lesson: keep, and it owes a demo before v0.9 (the lane's entry rule) or it moves then | **disagrees** (no user: the usage lens moved it) |
| `planning/search/` (RRT, RRT*, extenders, Dubins, live plot, plotting) | sampling-based motion planning: a robotics planner, not a control approach; the vision's sentence does not name it; the roadmap owns it only as pyro parity | **side pillar**: keep the two planners as proof the `PlanningProblem` → `PlanningSolution` contract spans the family, stop investing; the extras (`dubins.py`, `steering.py`, `live_plot.py`, 463 lines of plotting) trim to what the two demos need | agrees in effect (the usage lens moved all of it; the vision keeps the planners) |
| MPPI (planned, S72) | sampling-based MPC: the state of the art for the racecar | pillar (trajectory, online) | — |

### 4.6 Optimization and identification

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `optimization/` (`MathematicalProgram`, `Optimizer`, the evaluators, SciPy and Ipopt backends) | the solver boundary of the spine; after V1 it is the trainer of every pillar (trajopt, controller tuning, identification, imitation) and gains the first-order backend | spine | agrees |
| `identification/` (5-line stub) | **the identification pillar**: fitting a plant's parameters to logged runs is the same verb as fitting a policy to demonstrations, on the same spine (DESIGN §3 says so: "physical params and NN weights are the same verb") | pillar; the stub is honest about the plan but says nothing: replace it with the verb when V1 lands (C4), or remove it until then | agrees on the stub, not on the pillar |

### 4.7 Content: the plants

The vision is about vehicles and robots; the courses add the textbook benchmarks. A plant
is content, not a feature: cheap to keep (a leaf imports nothing) but each one costs a
prelude name, a both-backends contract test and a docs page.

| Plants | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| the vehicle ladder (`KinematicBicycle`, `DynamicBicycle`, `BicycleDynRate`, `DynamicBicycleCar3D`, `tires.py`), the three racecars, `KinematicCar`, `HolonomicMobileRobot` | the vehicle pillar's content | content: keep | agrees |
| `vehicles/propulsion.py` (longitudinal cars with torque or slip input), `vehicles/suspension.py` (quarter car), `MountainCar` | vehicle dynamics beyond steering: the course's "kinematic to dynamic with tires" ladder reaches them | content: keep `propulsion.py` and `suspension.py` with the ladder; `MountainCar` is an RL benchmark nothing uses: side | **disagrees** on propulsion and suspension (no user today) |
| the manipulators (`arms.py`, `ur5.py`) | the robot pillar's content | content: keep the arms a lesson uses; `OneLinkManipulator` and `ThreeLinkManipulator3D` are G1's call | agrees |
| pendulums, cart-poles, `DoublePendulum`, `DoubleIntegrator`, `SingleMass` | the textbook benchmarks every course and every pillar's demo uses | content: keep | agrees |
| `Drone2D`, `Rocket` | the RL pillar's two benchmarks (learn to fly, land) | content: keep | agrees |
| `aerial/plane.py` (`Plane2D`, `Plane3D`, 827 lines, with their 3-D skin), `marine/boat.py`, `astro/three_body.py`, `oscillators.py` (`Lorenz`, `VanderPol`), the floating and multi-mass springs, the drone variants, the rotating cart-poles, `Acrobot`, `TwoIndependentPendulums`, `SimpleIntegrator`, `TripleIntegrator` | other domains and variants; none is a vehicle, a robot or a benchmark a course needs | **side**: `experimental/catalog/<domain>/`, or the example that motivated them (the boat to `experimental/boat/`); `Lorenz` and `ThreeBodyProblem` keep their one-file demos under the research examples | agrees, except the usage lens kept `Lorenz` and `ThreeBodyProblem` for their demos |

### 4.8 Presentation and interfaces

| Module | Role | Verdict | Usage lens |
| --- | --- | --- | --- |
| `graphical/diagrams/dot.py`, `topology.py`; `signals/`; `phase_plane/`; `port_map.py` (the control-law map); `animation/` with the Matplotlib, Plotly and MeshCat renderers; `common/`, `control/` (the figure spec) | how every pillar shows its result | content: keep | agrees |
| `graphical/diagrams/mermaid.py`, `hybrid_mermaid.py`, `export.py`; `animation/camera.py`; `renderers/pygame_renderer.py` | a second exporter, dead helpers, a fourth renderer nothing calls | side: delete | agrees |
| `graphical/catalog/racecar_skin.py` (1.3k), `boat_skin.py`, `meshes.py` (STL / COLLADA readers), `assets/` (580 KB) | one plant's 3-D look and the readers that serve only it | side: to the racecar plant folder, between terms (a GRO501 notebook imports it by path); the generic `skins.py` / `shapes.py` stay | **disagrees** (a course notebook imports it) |
| `interfaces/gymnasium.py` (`Sys2Gym`, `SB3Controller`, `ProblemEnv`, `to_gymnasium`) | interop with another ecosystem; the native RL planner is the pillar, the bridge is a courtesy | side: keep `Sys2Gym` / `SB3Controller` thin until the SB3 twin notebooks retire (S49, v1.0), then experimental; `ProblemEnv` / `to_gymnasium` trim now | **disagrees** (two course notebooks import it) |
| `experimental/symbolic/` (SymPy equations of motion) | a way to write `f` for an arm; the vision's modelling path is `MechanicalSystem` / `Manipulator` with ABA / RNEA | side, already in the experimental tier; stays | agrees |
| `experimental/c_export.py` (JAX → C) | **deployment**: a controller designed here running on the real racecar's firmware is the last step of "control a vehicle"; the showcase already demonstrates it | side today, strategic later: stays in the experimental tier until a real vehicle runs it | agrees |
| `experimental/engines/` (contact worlds, ANCF tire, 853 lines) | a multibody contact engine: the constitution's §1.4 says this is not the product | side, beyond the vision: delete, or keep one external-engine leaf as the template the MJX spike will follow | agrees |

## 5. Where the two lenses disagree, and what that teaches

| Item | Usage says | Vision says | Why the vision wins |
| --- | --- | --- | --- |
| realtime loop | move (TRL 2, three demos) | keep: the imitation pillar's recorder | the pillar is on the roadmap; the module is its seed, not a toy |
| Lyapunov certificate | move (one demo) | keep the concept as a `Field` verb | nonlinear control has no other certificate; a learned policy needs one too |
| `PurePursuit` | move (no user) | keep: the vehicle baseline | the first consumer of the geometry module; the lesson is missing, not the need |
| tabular RL | move (no user) | keep, with a demo owed | the textbook's rung between DP and PPO |
| value-function approximators | move (no user) | fold into fields (A1) | the concept is a pillar's; only the implementation is orphaned |
| propulsion, suspension plants | move (no user) | keep with the vehicle ladder | vehicle dynamics is named content |
| racecar skin + mesh readers | keep (a course imports it) | move to the plant folder | one plant's look is content for that plant's folder, not a library feature |
| gymnasium / SB3 bridge | keep (two courses import it) | thin now, experimental at v1.0 | interop is a courtesy; the native planner is the pillar, and the twins retire |
| `Lorenz`, `ThreeBodyProblem` | keep (a demo each) | side | other domains; the demos follow them |

The pattern: usage under-counts **seeds** (a pillar's first module, written before its
lesson) and over-counts **content** (a notebook imports a skin or a bridge because it was
there). The exit rule in §8 is written to tell the two apart.

## 6. The recommendations, in tiers (reconciled)

"Move" means `git mv` into `minilink/experimental/` or into the example folder that
motivated the code, off the prelude and the band facade, off the docs site, with its tests
following (the experimental tier is tested today: `test_symbolic.py`, `test_engine_jax.py`).
"Delete" means git history keeps it.

### 6.1 Tier 1 — side features: move or delete now

| What | Lines | Where to |
| --- | --- | --- |
| 24 plants of other domains and variants (§4.7, last row, plus `MountainCar`): `Plane2D`, `Plane3D` (+ `plane_skin_3d`, `plane_airframe_3d`), `Boat2D`, `Boat2DWithCurrent` (+ `boat_skin.py`), `ThreeBodyProblem`, `Lorenz`, `VanderPol`, `Acrobot`, `TwoIndependentPendulums`, `RotatingCartPole`, `UnderactuatedRotatingCartPole`, `ConstantSpeedHelicopterTunnel`, `Drone2DWithSideThruster`, `SpeedControlledDrone2D`, `FloatingSingleMass`, `FloatingTwoMass`, `FloatingThreeMass`, `TwoMass`, `ThreeMass`, `SimpleIntegrator`, `TripleIntegrator`, `MountainCar`, `OneLinkManipulator` (G1 may keep the arms) | ≈ 3.0k | **the candidate list** for `experimental/catalog/` (unsupported: no test, no docs, out of the wheel); the maintainer rules item by item in K1 (ruled 2026-10-22, [end-state-api.md](end-state-api.md) D2, TODO K8). Twins (`SimpleIntegrator`, `TwoIndependentPendulums`, parameter-only variants) merge instead |
| the Mermaid exporters, `camera.py`, `PygameRenderer`, `ProblemEnv` / `to_gymnasium`, the `identification/` stub (until C4) | ≈ 0.8k | delete |
| `experimental/engines/` (contact worlds, ANCF tire) | 853 | delete, or keep `world.py` as the external-leaf template (ask) |
| the prelude trim: every name no notebook uses **and no pillar claims** leaves the root prelude and stays on its band facade | ≈ 45 names | the root prelude is the teaching set; the bands are the full index |

About **4.6k lines** out of the wheel and about 45 names off the prelude; no course
notebook touched; no pillar touched.

### 6.2 Tier 2 — content that moves between terms, or at a decision already scheduled

| What | Lines | When |
| --- | --- | --- |
| `racecar_skin.py` + `meshes.py` + `assets/` → the racecar plant folder | 1.9k + 580 KB | between terms (the GRO501 racecar notebook imports it by path) |
| `Sys2Gym` / `SB3Controller` → experimental | 0.3k | when the SB3 twin notebooks retire (S49, v1.0) |
| `planning/search/` extras: `dubins.py`, `steering.py`, `live_plot.py`, `plotting.py` cut to the two demos; the two planners stay as a side pillar | ≈ 0.9k of 2.5k | any time; the planners' tutorial section is student-facing (ask) |
| `spatial/plotting.py` (800) cut to `plot_track`; `workspace_fields.py` folded into the fields plan | ≈ 0.6k | with the geometry move (A3) |
| `control/mpc/utilities.py`, `viz.py` lose their band names (internal) | 0 | any time |
| the facade verbs no pillar needs (`get_diagram`, `plot_input_output_map`, `render`) | ≈ 0.1k | before the freeze (V3) |

### 6.3 Tier 3 — pillar seeds: keep, and finish (unused today is not the point)

- **The realtime loop**: the imitation pillar's recorder. Its next step is a trajectory
  dataset out of `game()` (the behavior-cloning row, ROADMAP §5.6), under the architectural
  review already scheduled; nothing moves.
- **The Lyapunov certificate**: rebuilt as a `Field` verb when fields land (A1); the
  current module is the prototype, kept provisional until then.
- **`PurePursuit`**: the vehicle baseline; takes a `Track` once geometry moves (A3); its
  lesson is the GMC714 vehicle row (C2).
- **Tabular RL**: owes its demo (the rung between the grid-world DP lesson and PPO) before
  v0.9, or moves then.
- **The value-function approximators**: folded into the fields plan (A1) as the cost-to-go
  approximators; the module itself retires into that work.
- **The identification verb**: the stub says nothing; the verb arrives with the
  differentiable problem (V1) and fitting (C4), as the same program export (`pe.program(wrt)`
solved by `Optimizer`, R23) with a data cost.
- **The hybrid sibling** (5.0k lines): spine, provisional in shape only; no new features
  before S31.
- **The RL planner**: four algorithms as options on one planner; `collect.py` and
  `environment.py` are also the data path imitation reuses, so they are spine, not RL-only.

### 6.4 Tier 4 — the core that remains

| | The core |
| --- | --- |
| spine | `System` and its algebra, compile and the rollout, the textbook objects (with geometry), the hybrid loop, the facade, `Simulator`, `PlanningProblem` / `PlanningSolution` / `Evaluation` / `compare`, `MathematicalProgram` / `Optimizer`, the neural policy and its MLP |
| pillars | classical and modern (analysis, design verbs, PID family, observers); nonlinear (model-based, impedance, robotic, geometric, the Lyapunov field); trajectory (trajopt, MPC, MPPI to come); RL (DP reference, the planner, tabular rung); identification and imitation (the realtime recorder, the fitting verb to come); estimation (observers, the EKF to come) |
| content | the vehicle ladder with propulsion, suspension and tires; the three racecars; the arms and the UR5; the benchmarks (pendulums, cart-poles, double pendulum, double integrator, single mass, drone, rocket); the three renderers, diagrams, signals, phase plane, the control-law map, the generic skins |
| side, kept small | RRT and RRT* as the proof of the planning contract; the gymnasium bridge until v1.0; C export and symbolic mechanics in the experimental tier |

About **65k lines** after Tiers 1 and 2 instead of 70.7k, and a prelude of about **115
names** instead of 162, each one on the spine, in a pillar, or the content the vision
names. Less than the usage pass cut, on purpose: the vision keeps five modules usage would
have moved.

## 7. The seeds the vision still needs a home for

Not new scope: the roadmap already lists each. This pass only notes which existing module
is the seed, so the convergence does not cut the branch the next pillar grows on.

| Planned (ROADMAP) | Seed in the library today | What it reuses |
| --- | --- | --- |
| behavior cloning (after v1.0) | `simulation/realtime/` (the recorder), `planning/reinforcement_learning/collect.py` and `environment.py` (the data path), `NeuralPolicyController` (the output), the first-order optimizer backend (V1) | the spine end to end: demonstrations are a `Trajectory` dataset, imitation is the program export with a data cost, or a `fit` verb beside it (future-policies.md) |
| identification (C4) | the `rollout_batch` evaluator, the plant's own `params` (R41; `ProblemParameters` retired), the same program export (`pe.program(wrt=["plant…"])`) | the spine: plant params instead of policy params |
| MPPI (S72) | `planning/trajectory_optimization/`'s problem and transcription, `realize` (the draws, RN-5) | the trajectory pillar, sampled |
| the EKF and the trajectory observer (v0.9) | `estimation/observer.py`, `analysis/linearization` | the estimation pillar |
| the Lyapunov field (A1) | `analysis/lyapunov.py` | fields |
| a learned-policy certificate (research) | the Lyapunov field on a compiled closed loop | nonlinear + RL pillars |
| deployment on the real racecar | `experimental/c_export.py` | the experimental tier, until a vehicle runs it |

The long-term shape this implies, consistent with [repository-structure.md](repository-structure.md)
A2: once imitation and identification exist, RL, imitation and identification are one
"learning from data" family on the spine and may share a package. Not before v1.0, and not
a reason to move RL now.

## 8. The exit rule (proposed amendment to the lane contract)

**Not adopted (R4, 2026-10-22):** no exit rule; the maintainer prunes by hand at each release
from a K1-style table, and the seeds have no clock. The proposal stays below as the record.

ROADMAP §2's teaching-surface rule says what it takes to enter. Add the symmetric rule,
with the two lenses built in:

> At each minor release's close-out, a teaching-surface or provisional name that no
> notebook, course checklist or tutorial chapter uses for two consecutive releases moves to
> `experimental/` or to the example that motivated it, **unless a roadmap row names it as a
> pillar's seed**; a seed that has not grown (no step taken on it) for two releases moves
> too.

This makes convergence a habit rather than an event, protects the seeds, and puts a clock
on them.

## 9. Mechanics of one move (each item a small, reviewable commit)

1. `git mv minilink/<band>/<module>.py minilink/experimental/<module>.py` (history kept).
2. Drop the names from the root prelude and the band facade; `test_teaching_surface.py`
   and `test_public_imports.py` pin the new list.
3. Drop the `docs/api/*.rst` entry; the research tier is not on the API site.
4. Move the demo to `experimental/<topic>/`; move or trim the tests, keeping them
   running.
5. The TRL ledger row (ROADMAP §3) moves to "research lane"; one dated record under
   `docs/reviews/` lists the pass's rulings; the "Breaking changes so far" ledger in
   ROADMAP §2 gets one line per release.
6. A move that touches a course notebook's import (the racecar skin): between terms only,
   with the citing documents updated in the same change.

## 10. The asks

Ruled 2026-10-22 ([the record](../reviews/2026-10-22-rulings-session-1.md)): ask 1 (R1,
amended: model, problem and program as the three foundations, the six pillars on them; RRT a
side pillar, deployment experimental), ask 2 (R2: the catalog clean-up by two import layers and
support by test, TODO K8; the tiers' items still per K1's table), ask 3 (R3: the engines stay
as the contact-leaf demo, the tier untested, TODO K9), ask 5 (R4: no exit rule), ask 6 (R5: the
trim silent, one ledger line per release). Open: ask 4.

1. **The six pillars of §2** as the stated shape of the framework. If the maintainer's list
   differs (RRT as a seventh pillar, imitation folded into RL, deployment as a pillar), the
   verdicts in §4 change with it; that list is the real decision here.
2. **Tier 1 as a block**, or item by item; the plant rule behind it: a plant stays when it
   is a vehicle, a robot, or a benchmark a course uses.
3. **`experimental/engines/`**: delete, or keep one file as the external-leaf template.
4. **The racecar skin** to the plant folder between terms, and **the SB3 bridge** to the
   experimental tier at S49.
5. **The exit rule** of §8 into ROADMAP §2, and the three seeds with a clock on them
   (tabular RL's demo, the Lyapunov field, PurePursuit's lesson).
6. **The prelude trim's form**: silent before v0.9 (the ledger line) or a one-release
   `DeprecationWarning` shim per name, which the deprecation policy (V4) only requires from
   v1.0.

When the maintainer rules, the rulings go to a dated record under
[../reviews/](../reviews/), the moves to [TODO.md](TODO.md) as steps, and this document is
deleted once they land (one fact, one home: [README.md](README.md)).
