# Automation by convention: the decision record (2026-10-11)

This is the full record of how the automation-by-convention plan was decided, kept word for
word as it stood on 2026-10-11, when all fifteen decisions were ruled and the operator
review closed. It holds:
- the requirements;
- the analysis of the code as it was;
- the evidence: surveys, probes, replays and "today" tables;
- the options considered and rejected;
- each decision with its ruling and date.

It is history, read-only. The live plan, rewritten by topic with no decision numbers, is
[docs/plans/automation-by-convention.md](../plans/automation-by-convention.md).

Rulings made after the text below was frozen (2026-10-11):
- Decisions 7, 9 and 12, marked "agreed in principle" below, are final.
- The composition steps are built in the order AC-0, AC-2, AC-OP, AC-3, then AC-4 to AC-8.
- The camera copy is removed in AC-6 and the shape-based ids in AC-2b, not in AC-3.
- Decision 12 (role boxes) amends decision 8 for `@`: `@` nests a box under its role name;
  `+` and `>>` stay flat.
- Where the text below says a ruling "is recorded in ROADMAP §6", that was the practice of
  the time: rulings now live in this record and in the plan, and leave ROADMAP §6.

Rulings of 2026-10-13, from the second opinion
([2026-10-13-automation-by-convention-second-opinion.md](2026-10-13-automation-by-convention-second-opinion.md)):
- `rename_ports` is simultaneous: chains work, only a collision with an untouched port is
  refused (R1).
- `@` reads a diagram plant's state for an `x`-reading controller; no `expose("x")` (R2).
- One guard in `>>`: a source into a controller's `y` is refused (R3).
- The loop keywords are words, `reference=`, `disturbance=`, `noise=`; the ports stay
  `r`, `d`, `n`. Where the text below says `closed_loop(d=, n=)` or `r=`, read the words
  (R4).
- The `@` dispatch is one rule, "same-name `y` and `x`, else an Error on the entry" (R5).
- `loop >> loop` and `loop + loop` refuse (R6).
- P4 starts now, in parallel, hand-wired until AC-4 (R7).
- The build order is AC-0, AC-2a, AC-OP, AC-3, then AC-2b and AC-2c as one sweep (R8).

---

**Status:** v2, analysis and recommendation (2026-10-10). Every decision of §5 (1 to 15) is
ruled as of 2026-10-11, and the operator review is closed (§3.10).
- v1 (commit 4b654d8) read the roles from the wiring.
- v2 reads them from three reserved subsystem ids. It follows the maintainer's direction
  of the same day (§1, requirements 5–8), an adversarial review, and a check against every
  planned feature (§3.7).

**Rung:** v0.2 wave B4 (P10, widened) for AC-0 to AC-3; AC-4 to AC-8 placed in §4.

**Evidence:** eleven read-only surveys.
- Seven on `dev` at `d1b4bae`:
  - loop dispatch internals;
  - a census of every shortcut and hand-wired loop;
  - controller declarations;
  - plant disturbance and noise ports;
  - plotting and animation automation;
  - tools that take or return a plant and a controller;
  - every other core heuristic.
- Four at `4b654d8`:
  - what the cost sees on a closed loop;
  - the planned features that put more than a plant and a controller in a loop;
  - a census of port names and override attributes;
  - an adversarial review of the v2 design.

The defects marked *reproduced* were re-run by hand. The loop-building defects D1–D8 come from the 2026-10-10 feedback-loop review (in git history).

The question this plan answers is: *what is the minimum the library must know about a plant
and a controller to build a loop in one line, pick the signals worth plotting, frame the
animation, and score a policy?*

The answer it argues for: **standard names, read at the moment of use, and nothing else.**
- Port names say what a block reads and commands, and drive the automatic wiring.
- Three reserved subsystem ids (`plant`, `controller`, `estimator`) say what role a block
  plays, and every other tool reads them.
- A handful of explicit hints remain for the things names cannot say.

## 1. Requirements (maintainer, 2026-10-10)

1. **Manual first.** A controller is a `System`, a plant is a `System`, and a loop is
   `add_subsystem` plus `connect`. That path stays clean and simple, and the automatic
   machinery never gets in its way.
2. **The minimum information.** For a one-line `ctl @ plant`, find what must be known about the
   plant and about the controller. In particular, find out whether a profile, a class
   hierarchy or a stored hint is needed, or whether the ports alone are enough.
3. **State.** Find out whether the tools need to know that a controller has internal state,
   and whether `Controller` / `DynamicController` earn their place.
4. **Every automatic use case.** Cover all of them, not only composition: plot signal
   selection, the animation camera, Monte Carlo evaluation and every tool that takes or
   returns a plant with a controller, plus any other place that guesses.
5. **Roles.** A diagram assigns roles automatically: `plant` and `controller`, later
   `estimator`. The tools use them:
   - plots get an auto mode, the signals of the plant and/or the controller;
   - the animation camera comes from the plant;
   - evaluation scores the cost on the plant, never on a controller's internal states.
6. **Standard names only.** Port names are standardized. The automatic mode works only with
   standard names; a block with custom names is wired by hand. There are no override
   attributes.
7. **Future-proof.** Project every coming feature onto the proposal and check that it holds:
   randomness, RL, MPC, estimation and the rest (§3.7).
8. **No freeze.** We are between terms, so the teaching design is unfrozen: no decision
   rests on the name freeze (ROADMAP §4.1 gate 7, paused on 2026-10-10). A rename migrates
   every course notebook in the same commit, with no alias.
9. **Operators stay simple** (2026-10-10). `>>` is a plain manual serial connection, not an
   auto-wiring operator; the automation lives in `@`. No operator modifies its operands
   (§3.10).

## 2. Analysis: what the library automates today

### 2.1 Diagram composition

`ctl @ plant` calls `closed_loop` (`core/composition.py`), which takes one of three paths:

- **(a)** a numeric return gain;
- **(b)** an Error junction for an error-driven block;
- **(c)** two-port wiring for a block that reads `y` or `x`.

The path is chosen through four declaration layers:

- the `feedback_profile` string, mapped to roles by `PROFILE_PORTS` (`core/feedback.py`): eight rows, five distinct;
- attribute overrides (`measurement_port`, `ref_port`, `control_port`, `plot_space`);
- the `ErrorDriven` port-layout switch (`ports="error"` or `"reference"`);
- a single-input heuristic (`error_input`).

The hybrid `% dt @` has its own resolver, and `>>`, `+` and `autowire` have their own
fallback chains.

- **The ports already carry the decision.** A rule that reads only port names reproduces
  today's choice for **19 of 19** library controllers:
  - input `e` → Error junction;
  - input `x` → state feedback;
  - input `y` → output feedback;
  - one other free input → textbook loop gain.

  The controllers checked: PID, PI, PD, Lead, TransferFunction (every layout), P (both
  layouts), StateFeedback (plain, `N=`, time-varying), Impedance (plain, integral),
  TaskImpedance, ComputedTorque, SlidingMode, NeuralPolicy.

  On `(lqr >> Saturation())`, whose only boundary input is `r`, the same rule *refuses*
  instead of building D1's silently wrong loop.
- **Shortcut-only state lives on every diagram.** `DiagramSystem` carries
  `_composition_entry` and `_composition_output` (`core/wiring.py:248`). They are set by the
  shortcuts and are `None` on a hand-wired diagram.
- **Plant disturbance and noise ports are ignored.** `PendulumWithNoisePort` and
  `CartPoleWithNoisePort` declare their own `w` and `v`. `closed_loop(w=, v=)` adds Sum
  blocks at `u` and on the measurement instead, so about eleven sites wire noise by hand.
  The randomness plan (A9, D24) says the loop inputs *are* those plant ports; the two answers
  were never reconciled.
- **Defects** (in addition to D1–D8):
  - **A** (*reproduced*): `>>` on a hand-wired diagram attaches to the last-added block, not
    to the declared boundary output.
  - **D** (*reproduced*): `(Step + P + Saturation + Integrator).autowire()` wires both the
    plant and the saturation from the controller, bypassing the saturation, and leaves the
    measurement unconnected: an open loop with no error raised.

### 2.2 Plot signal selection

`plot_trajectory()` picks signals in `graphical/signals/time_signals.py`.

- **What the defaults show.** The defaults are the state `x` plus every subsystem output
  *literally named* `u`, plus the outputs of input-less sources. Hidden by default:
  - the plant's `y`;
  - the boundary inputs `r`, `w` and `v`;
  - the error;
  - a command whose port is not named `u` (`Gain @ p`, `p @ 1`, MPC's `u_ff`).
- **Boundary names cannot be plotted.** `y`, `r`, `w` and `v` raise "Unknown signal".
  `u` means the stacked *boundary inputs*: on a closed loop that is the reference, not the
  command.
- **Two defaults.** `compute_trajectory(show=True)` uses a different default
  (`("x", "u")`) from `plot_trajectory()`, so the same loop plots the reference in one
  place and the command in the other.
- **Colours follow literal names.** On a closed loop the reference (`u`) is drawn red and
  the real command (`ctl:u`) green.
- **Labels and units are lost.** Shortcut boundary ports (`y`, `w`, `v`, and `r` on paths
  (a) and (b)) and `connect_new_output_port` drop the source's labels and units, so a
  pendulum loop shows `y[0]` instead of `theta`.
- **Flow and hybrid name the same signals differently.** The command is `u[0]` in hybrid
  and `ctl:u` in flow; the plant output is `plant:y` vs `sys:y`.
- **Other rules.** Every source gets the id `ref` (a `WhiteNoise` becomes `ref2`). Planning
  and comparison plots hard-code `("x", "u")`. The phase plane's default axes are
  controller states in `PID @ plant`, because the controller is inserted first.

### 2.3 Animation camera

- **When and where the camera is copied.** A diagram's camera is the plant's `camera_*`
  fields, copied once, *at composition time*, by `_propagate_animation_camera`
  (`core/composition.py`). Only `@` (the plant side), the right operand of `>>`, inlined
  diagram operands and hybrid loops copy it.
  - Hand-wired diagrams get nothing.
  - `plant @ 1`, `plant + source` and the left operand of `>>` get nothing.
- **How the source is picked.** The source block is "the first block with a static skin,
  else the first leaf". So:
  - `Lead() @ plant` and `(C >> G) @ 1` give the camera to the *compensator*, because
    `TransferFunction` draws a skin;
  - `plant >> Gain` gives it to the `Gain`.
- **Unused or uncopied fields.**
  - `camera_priority` exists but is never read.
  - `scene_grid` is never copied to a diagram.
  - Follow frames in nested diagrams are never namespaced.
- **What gets drawn.** Every subsystem with geometry draws, by presence alone, so a
  compensator draws a ground line and a ball in every loop.

### 2.4 Monte Carlo and the other plant-with-controller tools

These tools are `MonteCarloEvaluator` and its three backends, `RolloutEnvironment`, the RL
and tabular planners, `PolicyEvaluator`, `PlanningProblem` / `StochasticPlanningProblem`,
`LQRPlanner`, `lqr_at_operating_point`, `place_at_operating_point`, `trajectory_lqr`,
trajectory optimization, value iteration, MPC, `Sys2Gym`, `discretize`, the sensitivity
functions and the analysis channel defaults. Each answers the same few questions its own way:

| Question | Distinct rules today | Example of disagreement |
|---|---|---|
| Which plant input is the command? | **7** | `PendulumWithNoisePort` (`u`, `w`, `v`): `@` and the evaluators drive `u`; LQR, place, value iteration, trajopt, MPC, Gym and `discretize` treat all three ports as actuators |
| Which inputs are disturbances? | **4** | `problem.disturbances` keys; "all but the action, at nominal"; "none, every port is a decision variable"; the loop's own Sums |
| What goes back to the controller? | **6** | `@` wires `plant.y` or `plant.x`; the evaluators write the internal state into a port that may be named `y`; Gym calls `sys.h` (zeros or `x` on a diagram); MPC reads `y` as the state |
| What does the controller block read? | **5** | `feedback_ports`, `error_input`, the plot fallback, the computer boundary, the first declared port |
| What is the controller's command? | **6** | `u` → `u_ff` → single output; `u_nom` → `u` → `u_ff`; "an output named `u`" for plots; RL calls a method |
| Does it have state? | 1 test (`n`), 3 behaviours | Monte Carlo refuses; `PolicyEvaluator` pins `x0` silently; the simulator integrates |
| Is it time-varying? | **0 rules**, 2 behaviours | Monte Carlo freezes the law at t = 0; the simulator honours t |
| What `u` does the cost see? | **2** | The action port only, or every port stacked; they disagree even inside `RolloutEnvironment` |

They all agree only on a single-input plant whose input is named `u` and whose `y` equals its state.

**Defects:**

- **Time-varying laws scored at t = 0** (`planning/evaluation.py:409, 467, 476`): finite-horizon
  `LQRPlanner(evaluate=True)` and `trajectory_lqr` controllers are scored with `K(0)`.
- `disturbances={"u": …}` overwrites the policy's command (`environment.py:172-174`).
- **The three Monte Carlo backends score different loops.** jax and numpy use the state
  held per `dt`; the simulator uses the continuous `plant.y` and ignores the draws.
- **Disturbance ports become decision variables.** In planners, LQR, value iteration and
  Gym's action space, `w` and `v` are treated as free inputs, unbounded when the port has
  no bounds.
- **`wrt="u"` means every input stacked**, even when a port is named `u`. `wrt="x"`
  raises on the same ambiguity.

**What the cost sees.**
- The evaluators already score `problem.sys` in its own coordinates; only the simulator
  backend builds a diagram.
- The primitive for scoring a block inside a loop exists:
  - `DiagramSystem.trajectory_of(block)` (`core/diagram.py:150`) returns the block's own
    `x` and the `u` it received;
  - its docstring shows `cost.total_cost(loop.trajectory_of(plant))`.
- Six disagreements remain:
  1. **Which `u`.**
     - The action port: Monte Carlo, the rollout environment, RL.
     - Every input stacked: DP, LQR, tabular, Gym, trajopt and MPC, `compute_cost`.

     A loop built with `w=True` therefore charges the disturbance as control effort in
     `compute_cost(of=plant)`.
  2. **What the controller measures.** The plant state `x` on the numpy and jax backends
     (`planning/evaluation.py:466, 475`); the plant `y` on the simulator backend.
  3. **Controller state.**
     - Refused by numpy and jax (`evaluation.py:439-443`).
     - Integrated from `controller.x0` by the simulator backend.
     - Pinned at `x0` by `PolicyEvaluator` (`policy_eval.py:165`).
  4. **Time.** Laws run at `t = 0` everywhere except on the simulator backend.
  5. **Cost params.** Honoured by DP and `PolicyEvaluator`; ignored by Monte Carlo and the
     rollout environment.
  6. **The price of leaving `X`.** `+inf` on the simulator backend, a derived bound on
     numpy and jax.
- Also broken:
  - The simulator backend fails on MPC: `Computer @ plant` is a `HybridDiagram`, which has
    no `trajectory_of`.
  - Gym's observation is `sys.h`, but its observation space is the state box
    (`interfaces/gymnasium.py:124, 205, 245`).
  - The rollout environment's step reward and its price bound call `g` with different
    `u` (`environment.py:132` against `:303-309`).
  - MPC's internal cost is undiscounted.

### 2.5 Other places that guess

- **Analysis channel defaults** (`analysis/linearization.py`). The output is `y`, then `u`,
  then … . The input is the *first-declared* port, so a hand-wired loop that declares `w`
  first changes `bode(loop)`.
- **Ids.** Four schemes: `ctl`/`sys` (`@`), `ctl`/`plant` (hybrid), `replan`/`broadcast`
  (dual-rate MPC), and guessed from shape (`+`, `>>`). Ids also name the random streams
  (`realize` derives a child seed per subsystem id) and the params keys, so the shortcut
  that built a loop decides its realizations.
- **Diagram names.** `PID() @ Pendulum()` is named "Closed loop of Diagram".
- **Simulation.**
  - **B** (*reproduced*): a diagram's `x0` set by the user is silently replaced, because
    `Simulator` calls `refresh()`.
  - **C** (*reproduced*): `ProportionalController().compute_trajectory()` raises because a
    static block's output named `u` collides with the reserved signal `u`.
  - The automatic solver choice silently changes the input-hold model (linear vs
    zero-order hold).
  - The backend fallback is written three times, with different triggers.
- **`discretize` and Gym.** `discretize` stacks every input into one `u` and drops `q`,
  `dq`. Gym's action space includes the disturbance ports.
- **`plot_control_law`.** It reads roles through the same declaration stack; `plot_space`
  (error, absolute, error and rate) is the one plot hint that ports cannot express.

### 2.6 What the hints are today

| Hint | Used for | Needed? |
|---|---|---|
| Port names (`u`, `r`, `y`, `x`, `e`, `w`, `v`, `q`, `dq`, `u_ff`) | almost every decision | **yes: the convention** |
| Port dimensions | matching, Mux, Demux | yes |
| `n` (static vs stateful) | `%`, `static_law`, plot pinning, source detection | yes, and already the only test |
| Port `dependencies` | algebraic loops, feedthrough | yes |
| Nominal values and labels | defaults, unconnected inputs, plot labels | yes |
| `feedback_profile`, `PROFILE_PORTS` | wiring, plot sweep, naming | **no**: the ports decide the wiring; the sweep reads the ports, plus `plot_space` where it differs |
| Role overrides (`measurement_port`, `control_port`, `ref_port`) | nonstandard names (only MPC's `u_ff` needs one) | **no**: standard names, or wire by hand |
| `Controller` / `DynamicController` | `.plot_control_law()` only; never `isinstance`-checked | **no**: deleted (§3.6) |
| Subsystem ids (`ctl`, `sys`, `plant`, `ref`, minted from shape) | params keys, random streams, plot names, the hybrid plant | **yes: the roles**, as three reserved ids written by the shortcuts (§3.2) |
| `ErrorDriven` layouts | which ports a classical law exposes | yes, as a port-layout choice; not as a tool hint |
| `_composition_entry` / `_composition_output` | where the next `>>` attaches | **no**: shortcut state on every diagram |
| `camera_*`, `skin`, `scene_grid`, `camera_priority` | animation | yes, read at animate time; `camera_priority` is unused and goes |
| `solver_info` (time constant, discontinuity, sample period) | solver and `dt` | yes: physical, not structural |
| `is_random`, `params["seed"]`, ids | realizations | yes, and ids must be stable |
| Insertion order | tie-breaks (`>>` fallback, analysis `wrt`, hybrid ports) | **no**: the weakest hint and the cause of defect A |

### 2.7 Port names in use

- **Plants.** Almost every plant follows the convention through its base class:
  - `DynamicSystem`, `StateSpaceSystem`, `LTISystem`: `u` → `y`, with `x` when
    `expose_state=True` (`core/system.py:467`);
  - `MechanicalSystem` adds `q` and `dq`.
- **The exceptions among plants:**
  - `PendulumWithNoisePort` and `CartPoleWithNoisePort` add `w` and `v`.
  - `DynamicBicycle` (`w_rear`, `delta`) and `UdeSRacecarDyn` (`P_cmd`, `delta_cmd`) have
    no `u` unless built with `named_ports=False`.
  - Manipulators add the outputs `p` and `pdot`; the racecar adds `speed`, `slip`,
    `grip`, `imu` and `power`.
  - Only about 16 of 50 catalog constructors expose `x`; the racecar does not.
- **Controllers.** Every library controller reads `e`, `y` or `x` (plus `r`) and commands
  `u`, except two:
  - MPC reads the full state on a port named `y` and commands `u_ff`, with `x_ff` and
    `z` beside it (`control/mpc/controller.py:426-444`); its dual-rate broadcast block
    outputs `u_nom` and `x_nom`.
  - `TaskKinematicNullspace` adds `r_null`.
- **Blocks.**
  - Generic blocks use `u` → `y`: `Gain`, `Saturation`, `Integrator`, `ZOHHold`,
    `TransferFunction` and the filters.
  - Error uses `+`, `-` → `e`; Sum and Mux use `in0..`; sources output `y`.
- **Override attributes.**
  - MPC (`control_port="u_ff"`) and `TransferFunction(ports="reference")` set them.
  - So do four controllers whose ports are already standard, for nothing:
    `NeuralPolicyController`, `SB3Controller`, `LookupTableController` and `PurePursuit`.
- **Port keywords.**
  - `closed_loop`'s five port keywords have no caller outside the tests.
  - `hybrid_closed_loop`'s four have 22 uses in 6 files, including the course notebook
    `racecar_toward_mpc`.
- **Estimators.** None exists yet; P4 plans `x_hat`.

### 2.8 Why roles cannot be read from the wiring

v1 found the plant from the graph: "the block whose `u` is driven by another block's
command". The planned features break that rule:

| Case | What the graph shows | Where |
|---|---|---|
| An observer | inputs `u` and `y`, driven by the controller: a second plant | P4 |
| An actuator | `Saturation` or `ZOHHold`, `u` → `y`, between the controller and the plant: the actuator looks like the plant | AC-3 |
| `closed_loop(w=True)` | a Sum between the command and the plant: no plant is driven by a command | today |
| An open-loop policy | a source drives the plant's `u`: no controller | MPPI, S72 |
| A cascade | the outer controller's `u` drives the inner loop's `r` | nested loops |
| A cost block or a CBF filter | reads `x` and `u`: a plant candidate | V1, research |

Shape-based naming already fails silently:
- `_default_subsystem_id` (`core/composition.py:1374`) mints `ref`, `ctl` and `sys` from a
  block's ports, after `System.id`;
- `_unique_id` (`:1417`) turns a clash into `plant2` without a word.

The ids, by contrast, are already half a role system:
- hybrid loops use `ctl` / `plant`;
- `HybridDiagram.plant` and `.computer` are explicit fields;
- `closed_loop` knows the plant at build time, then drops it.

## 3. Main recommendation: the minimal system

### 3.1 Six principles

1. **Manual first.** A controller is a `System`, a plant is a `System`, and a loop is
   `add_subsystem` plus `connect`.
   - This needs no base class, no declaration and no hint.
   - Every shortcut builds exactly the diagram a student could write by hand (the same
     blocks, the same ids), and stores nothing else on `System` or `DiagramSystem`.
2. **Two kinds of standard names are the declaration.**
   - Port names say what a block reads and commands; they drive the automatic wiring.
   - Three subsystem ids say what role a block plays in a diagram; every other tool reads
     them.
   - The two tables of §3.2 extend RULES 4.9.
3. **Roles are read at use time, by one resolver, from the ids.**
   - They are never inferred from the graph, and never stored beyond the ids.
   - They are never guessed: a diagram with no `plant` id has no plant.
4. **State is `n`.** No class carries it, and there are no controller classes (§3.6).
5. **Standard names only.**
   - A block with custom names is wired by hand. It can still use the role ids, which give
     it the camera, the plot defaults and the cost.
   - There are no override attributes.
   - The few hints that remain explicit say what names cannot:
     - `plot_space`, the control-law sweep;
     - `solver_info`, which is physical;
     - `is_random` and seeds;
     - camera, skin and `scene_grid`, which are visual.
6. **Evaluation by identity.** A tool that receives a plant and a policy builds the loop
   itself and scores the plant it was given.

### 3.2 The two convention tables

**Ports: what a block reads and commands.**

**The letters (decision 13).** The plan writes the classical loop letters from here on:
`d` is the disturbance and `n` the measurement noise. §2 keeps `w` and `v`, because it
describes today's code. `v` is free for generalized velocity, as the plant equations already
write it (`C(q, v)`, `forward_dynamics(q, v, u)`), and `w` for angular speed and frequency.

| Block | Reads | Gives |
|---|---|---|
| Plant | `u` (the command), `d` (disturbance), `n` (measurement noise), other names held at nominal | `y` (measured output; `[q; dq]` on a mechanical plant), `x` (state) |
| Controller | `e` (the loop inserts `Error`, `e = r − y`), or `y`, or `x`; plus `r` | `u` |
| Estimator | `u` and `y` | `x`, the estimate (labelled x̂) |

- **Each row is a box (decision 12).** The row lists the ports on the edge of the role. The
  inside is one block or many; `@` and every tool read only the edge. On a box's edge, `x` is
  the box's own state, the stack of its blocks' states.
- **`x` only where needed (decision 1, as amended on 2026-10-11).**
  - Plants expose `x`: every catalog plant already does, and so do `StateSpace` and
    `TransferFunction` in the plant layout (`state_space.py:80`).
  - A user plant opts in with `expose_state=True`, and a box with `expose("x")`.
  - Controllers, compensators, filters, sources and routing blocks do not expose `x`.
    `LowPassFilter` and `NotchFilter` (plant layout today) stop exposing it; check their
    users first.
  - `@` with a controller that reads `x`, on a plant without `x`, is refused: "plant has
    no `x` output; build it with `expose_state=True`".
- A mechanical plant gives `y` and `x` only (decision 11). Its `q` and `dq` ports go, and
  with them the Mux. A positions-only sensor is a custom port or a `Demux`.
- Dimensions decide the Demux (component 0), for a controller that reads a position and
  computes its own derivative.
- Other names are allowed on any block, and the automation leaves them alone:
  - the controller extras `x_ff` and `z`;
  - the manipulator outputs `p` and `pdot`;
  - the input `r_null`.

**Ids: what role a block plays.**

The rule is: **blocks get words, signals get symbols.**
- Ports are mathematical variables (`u`, `r`, `d`, `n`, `y`, `x`, `e`).
- Subsystem ids are things in the diagram, so they are nouns.

A role is a reserved key of `diagram.subsystems`: the `sys_id` given to `add_subsystem`.
The same key names everything else about that block:
- its state slice (`state_index`);
- its params (`params["controller"]["Kp"]`, `"plant.mass"` in a distribution);
- its signals (`"plant:y"`);
- its random stream;
- its box in `plot_diagram` (`Pendulum::plant`).

| Id | Role | Written by |
|---|---|---|
| `plant` | the system under control | `@`, `closed_loop`, `% dt @` (as `HybridDiagram.plant`), the evaluators, a hand-wired diagram |
| `controller` | the law that commands the plant | the same |
| `estimator` | the state estimate the controller reads (`u`, `y` → `x`); a filter or a sensor is not an estimator | `closed_loop(estimator=)`, or inside a `controller` composite |

- The three role ids are reserved.
  - There is one of each per diagram level, and a clash raises; it is never suffixed.
  - A cascade nests (`plant/plant`).
  - A second physical plant gets a name of its own and is handled by hand.
- Variable names stay free: `ctl @ plant` gives the keys `controller` and `plant`.
- Every other id is a name, not a role, but it still matters, because ids are the params
  keys and the random-stream names:
  - sources take the word for the input they drive: `reference`, `disturbance`, `noise`,
    or the port's own name for a custom port (`wind`);
  - helpers are `error` and `filter`; a filter on the measurement is a helper, not the
    estimator role.

**What sits on the way back (ruling 2026-10-11).** The `estimator` role is a state estimate
only. A filter and a sensor are not estimators.

| On the way back | What it is | Where it belongs |
|---|---|---|
| a sensor's lag, gain or quantization | physics | inside the plant box: the plant is what you measure through |
| a low-pass or notch filter on `y` | an algorithm that gives a cleaned `y` | the helper `filter`: `feedback(C >> G, F)`, `closed_loop(filter=)`, or inside a controller box |
| a Luenberger observer, a Kalman filter | an algorithm that gives the state estimate x̂ | the role `estimator`: reads `u` and `y`, gives `x` (P4) |

Why they stay apart:
- **Physics and algorithms are scored differently.** An estimator's state is integrated,
  never scored; a physical sensor filed as an estimator would leave the plant.
- **Port names would wire a filter wrong, silently.**
  - Library filters (`LowPassFilter`, `NotchFilter`, a `TransferFunction`) read an input
    named `u` and give `y` plus their own state `x`.
  - Under the estimator convention, `u` is the command and `x` the estimate of the plant's
    state.
  - A state controller would read the filter's internal state as the plant's, unnoticed
    when the sizes match.
  - Only a special rule ("one input means a filter") could fix that.
- **The textbook keeps them apart:** `H` or `F` on the measurement in classical control,
  and an observer or a Kalman filter in state space.

**Names considered and rejected:**
- `sys`, `system`: they mean any `System` (`problem.sys`, every `sys` argument).
- `ctl`: an abbreviation, and already the course notebooks' law method `def ctl(self, x, u, t)`.
- `policy`, `agent`: RL nouns; a policy plays the controller role, and one word across the
  three courses wins.
- `process`, `model`, `env`:
  - `process` is a process-control dialect;
  - `model` clashes with MPC's internal model;
  - `env` is RL vocabulary, kept for the Gym bridge.
- `observer`: Luenberger only; a Kalman filter is an estimator.
- `G`, `C`, `F`, the textbook letters: `C` is already the output matrix (RULES 5.4), and a
  nonlinear plant is not a G(s).

### 3.3 One resolver, three questions

- **`roles(diagram)`: who is who.** A role is a reserved key of `diagram.subsystems`, so
  `roles()` is three lookups.
  - It returns the top-level `plant`, `controller` and `estimator`, read from the ids. The
    estimator may also sit inside the controller (`controller/estimator`).
  - A leaf is its own plant.
  - A `HybridDiagram` answers from its `.plant` and `.computer` fields, which is the shape
    S31 keeps. The error and the filter of a sampled loop sit inside the computer
    (decision 15).
  - There is no "deepest plant" rule. In a cascade the outer loop's plant is the inner
    loop (compositional closure), and `plant/plant` reaches the motor.
  - With no `plant` id the plant is `None`: a tool that needs it raises, and plots and the
    camera keep today's defaults.
- **`block_roles(block)`: what a block reads and commands.** It reads the port names only,
  and serves composition:
  - the measurement (`e`, `y` or `x`);
  - the reference (`r` or none);
  - the command (`u`);
  - whether the block is error-driven: it reads no `y` and no `x`, so the error drives its
    entry (`e`, or a plain compensator's one input; decision 14);
  - whether it is stateful (from `n`).

  It replaces `feedback_ports`, `error_input`, the `PROFILE_PORTS` lookups and the five
  fallback chains.
- **`command_port(sys)`: which input a decision drives.**
  - It is `u`, else the single input that is not `d` or `n`, else it refuses.
  - It replaces `action_port_of` (`control/neural.py:12`).
  - Gym, the cost, the design tools and a cascade's outer `@` all use it.

### 3.4 Composition writes the ids

- **`@`:**
  - `ctl @ plant` gives `controller` and `plant`.
  - `@` needs a plant on the right (decision 14). `feedback(L)` and `feedback(L, K)` close
    an open loop on itself: a leaf `L` is `plant`, a diagram `L` stays flat with its own
    ids, and the return block is `filter`.
- **Role boxes (decision 12): a plant, a controller or an estimator is one block or many.**
  - A role is a box with the ports of §3.2 on its edge. `@` and every tool read only the
    edge, so what is inside is free.
  - `@` nests a box under its role key, one level deep. A leaf stays a leaf: no wrapper and
    no extra level of names.
  - A box without the standard edge is refused, with a message that points to `expose` or
    hand wiring.
  - `>>` and `+` stay flat (decision 8). Only `@` nests, because only `@` assigns roles.
  - Ids stay unique at each level, and the params keys and random streams follow the
    nesting.
- **Building a box, simplest first:**
  1. By hand, which works today: `DiagramSystem`, `add_subsystem`, `connect`,
     `add_input_port`, `connect_new_output_port`. A hand-built nested loop already
     simulates.
  2. A serial chain with `>>` already has a plant's edge (`u` in, `y` out):
     `ctl @ (Saturation(-1, 1) >> arm)`.
  3. A new small helper, `diagram.expose(*names)`, about 25 lines over the existing API:
     - a name left open on inside inputs becomes one edge input that feeds all of them;
     - the name of one inside output becomes an edge output;
     - `"x"` gives the box's own state;
     - an unknown or ambiguous name is refused.

     ```python
     controller = (observer + gain).autowire().expose("r", "y", "u")
     loop = controller @ plant
     ```

     The order of `expose` and `autowire`, and how the open-input warning treats edge
     inputs, are settled with the AC-OP code.
- **Typical boxes:**
  - a closed loop used as a plant is `plant`, holding its own `controller` and `plant`;
  - a cascade is `outer @ ((inner @ motor) >> load)`: the inner loop's command input is `r`,
    by the `command_port` rule;
  - P4's `compensator(observer, K)` is `controller`, holding `estimator` and `gain`;
  - actuator limits belong to the plant box (`Saturation >> arm`), not to the controller;
  - the hybrid loop's `computer` and `plant` fields are boxes already (AC-8a).
- **Facts on `dev` @ 178f290.**
  - `@` refuses a diagram operand (`_add_system_to_diagram`, `composition.py:806`).
  - From the top of a nested loop, only the box's edge can be plotted (`plant:y`); a block
    inside cannot, neither by object nor as `plant:motor:y` (AC-5).
  - `P >> Sat` has no controller edge: its measurement stays open inside, and its output is
    `y`. A multi-block controller is therefore built by hand or with `expose`.
  - A `DiagramSystem` has no `x` output today.
- **Where the filter goes (ruling 2026-10-11).** In `closed_loop(C, G, filter=F)` and in
  `feedback(C >> G, F)`, the filter is a plain helper block at the loop's top level, beside
  `error`. It is in neither the plant nor the controller.

  ```
  r ──► error(+) ──► controller ──► plant ──┬──► y   (true output, the loop's output)
          ▲(−)                              │
          └─────────── filter ◄─────────────┘   (plant y + n when noise is on)
  ```

  - **Wiring by keyword, with no name guessed:**
    - plant `y` goes into F's one input;
    - F's `y` (else its only output) goes where the controller's measurement goes: the
      Error's minus input, or the controller's `y`;
    - a transfer-function filter's input named `u` does not matter.
  - **A controller that reads `x` with `filter=` is refused:** "a filter on the state is an
    estimator; use `estimator=`".
  - **Why the top level:**
    - only the plant's state and command are scored;
    - `plant:y` stays the true output, and the filtered signal is `filter:y`;
    - the sensitivity functions take `plant=`, `controller=` and `filter=` as three blocks;
    - the noise `n` enters before the filter, so the filter filters it.
  - **Elsewhere, by building a box (decision 12):**
    - a physical sensor goes in the plant box (`G >> SensorLag(...)`), and `plant:y` is then
      the measured value;
    - a filter that belongs to a reusable or sampled controller goes in a controller box
      (edge `r`, `y` → `u`). This is the form the `%` review will use, since the computer
      runs the filter.
- **Loop inputs (decision 3).** A plant that declares `d` / `n` owns them and is used as
  is: `d` is never charged as control effort, and `n` enters what the controller reads.
  `r=`, `d=`, `n=` accept a source block.
  - **The wrapper is deferred (ruling 2026-10-11).** For a plant without those ports,
    `closed_loop(d=, n=)` keeps today's summing junctions in the loop. The automatic
    wrapper (a plant box with edge `u`, `d`, `n`) waits until someone needs it.
  - **Known limit:** for such a loop, the cost sees `u + d` as the plant's command.
    Revisit with AC-7 if it matters.
- **`>>` and `+`** stop minting role ids from block shape. They name blocks plainly, and a
  hand-written role id is kept.
- In a shortcut, a role id wins over `System.id`.
- The composition-time state goes: `_composition_entry`, `_composition_output` and the
  camera copy.

### 3.5 What each use case becomes

| Use case | After |
|---|---|
| Composition | One port-based dispatch for `controller @ plant`: reads `y`, reads `x`, else an error junction on its entry, or a refusal that points to hand wiring; `feedback(L, K)` closes an open loop on itself (decision 14). `closed_loop(controller, plant, *, r, d, n, filter, estimator)`: each loop input is `False`, `True` (a boundary port) or a source block. `filter=` puts the helper `filter` at the loop's top level, on what the controller reads. No port keywords. |
| Plot signals | `signals="plant"`, `"controller"` or `"estimator"` selects every port of that block, as `"plant:y"` selects one. The default *auto* mode shows every port of the plant and the controller, each wire once; the reference, the error, `d` and `n` come in through their inputs. Signal names take id paths (`plant/plant:y`), reconstructed recursively, and a block inside a box is found by object too. Labels and units are copied onto exposed ports, and colours follow roles. With no roles, plots keep today's default. A leaf keeps `("x", "u")`. `compute_trajectory(show=True)` and the planning and comparison plots use the same default. |
| Animation | Resolved at animate time. The camera comes from the one drawable block anywhere in the tree, else the drawable block on the `plant` path, else an auto-fit view. A compensator draws no skin inside a loop. `camera_priority` is removed, and `scene_grid` and the follow frame come from the same block. |
| Cost and evaluation | The cost is relative to the plant. The tools build `policy @ problem.sys` (compiled once for all three Monte Carlo backends) and score `trajectory_of(problem.sys)` by identity: `x` is the plant's state, and `u` is its `command_port`. Controller and estimator states are integrated, never scored. A dynamic controller is integrated, not refused; `t` is honoured; the cost's params are passed; there is one price for leaving `X`. The `x0` draw is on the plant only, and the controller and the estimator start at their own `x0`. |
| Design and planning tools | DP, LQR, `place`, trajopt, MPC and value iteration decide `command_port` only. `d` and `n` are never decision variables; they are drawn or held at nominal. `discretize` keeps the plant's ports. |
| RL and Gym | The observation is what the controller reads (`x` today). The action is `command_port`, and the reward is `−g` on the plant. A policy with state is integrated by the loop. Gym's observation space matches its observation. |
| Analysis | The default input is `r`, else the command port; the default output is `y`. `wrt="u"` means the port `u` when one exists. |
| Ids | Renamed once, in flow and hybrid, before RN-4 names the random streams by id path: `ctl` → `controller`, `sys` → `plant`, `ref` → `reference`; sources named `disturbance` and `noise`. |

### 3.6 No controller classes

- **What the classes carry.** `Controller(System)` (`core/feedback.py:93`) adds no ports,
  state or behaviour; its one method, `plot_control_law(**kw)`, forwards to
  `graphical/port_map.plot_control_law`. `DynamicController` (`:123`) adds nothing.
- **Who checks them.** No `isinstance` or `issubclass` on either class exists in the
  library or the tests. Wiring, plotting and evaluation read attributes and `n`, never
  the class.
- **Who uses them.**
  - 16 library classes;
  - the root and `minilink.core` exports;
  - two test subclasses;
  - four course notebooks.

  Each course controller already uses the conventional ports, so its `feedback_profile`
  line is redundant:

  | Notebook | Class | Ports | Profile line |
  |---|---|---|---|
  | GRO860 `double_integrator_policy_evaluation` | `PositioningPolicy(Controller)` | `x` → `u` | `"state"`, plus a markdown sentence explaining it |
  | GRO501 `cartpole_static_controller` | `CustomController(Controller)` | `y`, `r` → `u` | `"output"` |
  | GRO501 `cartpole_dynamic_controller` | `LQG_Controller(DynamicController)` | `y`, `r` → `u`, `z`; `n = 4` | `"output"` |
  | GRO501 `ode_simulation` | `MyCustomController(Controller)` | `y`, `r` → `u` | none |

**Recommendation:**
1. **A controller is a `System`**, or a `DynamicSystem` when `n > 0`.
   - The 16 library classes take those bases.
   - `Controller` and `DynamicController` are deleted, with no aliases.
   - `ErrorDriven` stays: it is a port layout, not a role marker.
2. **`plot_control_law()` moves onto `System`** (`core/facades.py`), beside
   `plot_input_output_map()`.
   - The free function in `graphical/port_map.py` stays the engine.
   - It reads the measurement and the reference from `block_roles`; the sweep space comes
     from the ports, or from `plot_space` where it differs.
   - A block with no measurement input refuses and names `plot_input_output_map()`.
   - On a diagram it plots the law of `roles(diagram).controller`.
   - Every existing call keeps working, and a hand-written plain-`System` controller gets
     the method too.
3. **The course notebooks are migrated, not polished.**
   - Only the lines a removed name touches change:
     - the import line;
     - the base class (AC-0);
     - the profile line, with the GRO860 sentence reworded to "the input port is named
       `x`, so the tools know the block reads the full state" (AC-3).
   - The `PlanningSolution`, `Planner` and `Comparison` methods of the same name keep
     delegating to their policy.

### 3.7 Future-proof check

Each planned feature, projected onto §3.2 to §3.5:

| Feature (where) | In the loop | How it fits | Rename or gap |
|---|---|---|---|
| Luenberger and Kalman estimators (P4, v0.2 B3) | estimator `u`, `y` → `x` | id `estimator`, either top-level or inside the `controller` composite. `closed_loop(estimator=)` wires `y`, `u` and `x`. Plots compare `estimator:x` with `plant:x`. x̂0 starts at the estimator's own `x0` | `x_hat` → `x`; P4's composition ruling stays open |
| MPC (today; T6, MP-4) | `x` (+ `r`) → `u` | the internal cost is the same `CostFunction` on the model's `x` and `u`; `n` reaches the `x` it reads | `y` → `x`, `u_ff` → `u`; the racecar exposes `x`; the undiscounted internal cost is a defect to fix |
| Dual-rate MPC (S31) | replan and broadcast | roles from the hybrid fields; the composite exposes `x` → `u` | the broadcast's `u_nom` → `u`; replan's `u` not exposed |
| MPPI (S72, v0.3) | an open-loop source on the plant's `u` | the evaluator wires `controller` and `plant` itself | none |
| CBF safety filter (research lane) | `CBFSafetyFilter(controller, …)`, `x` → `u` | a filtered controller is a controller; hand-wired, `x` fans out by hand; the cost sees the plant's command | `u_nom` and `u_safe` stay internal |
| Actuators, zero-order hold | inside the `plant` box (decision 12) | part of the plant (the textbook H); to score the saturated command, put the saturation on the controller side | none |
| Loop noise and randomness (AC-4; RN-1 to RN-5) | `d` and `n`, owned by the plant that declares them (the wrapper is deferred) | the source ids `disturbance` and `noise` name the random streams; coloured noise (`WhiteNoise >> LowPassFilter`) is the `d` source, not a role; Kalman design reads the plant's `d` and `n` | ids renamed before RN-4 |
| RL, Gym, policies with state | `x` → `u` | the observation is what the controller reads; the action is `command_port`; the reward is `−g` on the plant; the loop integrates a stateful policy | Gym's observation space |
| Hybrid as a System (S31, v0.9) | `[plant; computer]` | the roles are its fields; the ids are renamed for params keys and streams | `hybrid_closed_loop` keywords settled there |
| A cost block, the differentiable cost V1 | reads `plant:x` and the command | id `cost`, not a role | none |
| Discrete time (P6, v0.9) | `discretize(plant)` | keeps the plant's ports (follow-up) | today it stacks the inputs and drops `q`, `dq`, which decision 11 removes anyway |
| Cascades, LQI, tracking, gain schedules | the inner loop as `plant` | closure, as in §3.3; LQI is `r`, `y` → `u` with state; tracking is `x` → `u`, time-varying | none |
| Named-port plants (racecar, bicycle) | custom names | wired by hand; the `plant` id still gives the camera, plots and cost | none |

None of these needs a new role, a class or an override. Two need a rename (MPC and the
estimator's `x`), and the rest need the defect fixes listed in §2.

### 3.8 Why this is the minimal system

- **Nothing new is declared.** Every decision in §2 reads a port name, a dimension, `n`, a
  `dependencies` entry or a subsystem id, and all of these already exist.
- **The roles cost three reserved words.** They are the ids the shortcuts and the hybrid
  loop already half use. They are written by hand as easily as by a shortcut, and they
  are visible in `plot_diagram`, in the params keys and in the random streams.
- **Things get deleted:**
  - the profiles, the overrides and the port keywords;
  - the controller classes;
  - the composition-time state and the shape-based ids.
- **What is left is small:** one resolver, two tables (RULES 4.9 extended), and evaluation
  by identity.

### 3.9 The compromise: automatic for standard diagrams, a few manual steps otherwise

**Automatic.** Standard diagrams get everything from one line: the wiring, the plot
defaults, the camera, the cost on the plant, the params paths and the random streams. This
covers:
- every library controller (the port rule reproduces all 19 wirings checked);
- every course-notebook loop;
- almost every catalog plant.

This is an estimate of most cases, not a measured share.

**Manual steps.**

| Situation | Manual step |
|---|---|
| Plant inputs with custom names (racecar `P_cmd`, bicycle `w_rear`, `delta`) | wire with `connect`; the key `plant` brings back the plots, camera and cost |
| Several controllers on one plant (the path-tracking cascade) | wired by hand; at most one block holds the `controller` key |
| Two physical plants in one diagram | the second gets its own key; tools are pointed at it explicitly |
| A hand-wired safety filter, extra inputs such as `r_null` | wired by hand; the rest still works |

**Never affected.** The manual path does not depend on roles:
- simulation, `linearize`, `bode`, `plot_diagram` and `trajectory_of` work on any
  `System`, whatever its keys and port names;
- without role keys, plots and the camera keep today's defaults instead of guessing;
- a tool that needs a plant (the cost) asks for it rather than picking a block.

**Details still open inside the steps.** They do not change the model:
- AC-5: the deduplication rule of the plot auto mode, and whether a plant with many outputs
  (the racecar) needs a name filter;
- AC-4, when the deferred wrapper is built: the keys inside the `d` / `n` plant wrapper, and
  how its boundary re-exposes the leaf's `x`.

### 3.10 The operators

The composition operators were reviewed one at a time, and the review closed on 2026-10-11
with decision 15. This section records what was decided; code follows in AC-OP, and in
AC-8a / AC-8b for `%`.

**Decision 8 (agreed 2026-10-10): no operator modifies its operands.**
- Today `+` and `>>` extend a diagram left operand in place and return it. After
  `L = C >> G; L2 = L >> H`, `L` itself contains `H`, and a start state set on `L` is
  reset to zero. `@` already returns a new diagram.
- After the change, `L >> H` and `L + H` return a new diagram, and `L` keeps its blocks,
  wiring, ports, name, `x0` and camera settings.
- **The result stays flat:**
  - operands' blocks are laid out side by side, with their ids kept;
  - the left diagram's `x0`, camera settings and name are carried into it;
  - blocks are shared, not copied, so long chains stay cheap.
- `connect()` and `autowire()` are methods and still modify the diagram they are called on.
- **Evidence:**
  - no code reuses a left diagram after `>>` or `+`;
  - no test asserts the in-place identity;
  - `closed_loop` loses the copy it makes only to protect the user's controller.
- This reverses the v0.1 "operator mutation semantics" record of DESIGN §4. DESIGN is
  rewritten when the change lands.

**Decision 9 (agreed in principle 2026-10-10): `>>` is a plain serial connection.** `A >> B`
wires one output of A to one input of B. The first case that applies wins:
1. A has one output and B has one input: connect them, whatever their names.
2. Exactly one output of A and one input of B share a name (`y`→`y`, `x`→`x`, `u`→`u`):
   connect those.
3. A has an output `y` and B has an input `u`: connect `y` → `u`, the textbook output and
   input.
4. Otherwise refuse: "`>>` connects one output to one input; wire this one with connect".

Details of the rule:
- **Sizes are a check, never a choice.** A mismatch is refused, as today.
- **A diagram operand counts by its boundary ports**: its boundary outputs and its free
  boundary inputs.
- **The result** keeps A's free inputs as its inputs, and B's output (`y`, else its only
  one) as its output. So `C >> G` still exposes `e`.
- **Dropped from `>>`:** the preference for `r` and the hidden entry and output memory.
- **Kept until their own steps:** the composition-time camera copy (AC-6) and the guessed
  ids (AC-2b).

**Evidence for decision 9.** Every `>>` call that runs was replayed: the tests, 16 demo
scripts and 10 notebooks, 104 calls in all.

| Outcome | Calls |
|---|---|
| One-to-one (rule 1), connected exactly as today; includes all 31 `Step() >> ctl @ plant`, since every `@` loop has the single input `r` | 100 |
| Series of two transfer functions (`G1 >> G2`, `G >> Integrator()`): a transfer-function plant has outputs `y` and `x`, so rule 3 is needed | 2 |
| Changes: `Step() >> ProportionalController()` goes to the measurement `y` by name, not `r` (a test) | 1 |
| Refused today and still refused: white noise into a loop with no free input | 1 |

Notes on the evidence:
- **The rule fixes a bug of today's `>>`.** `plant >> P` fed the controller's reference
  `r` and left its measurement at zero; rule 2 gives `y` → `y`, and
  `Pendulum >> StateFeedback` gives `x` → `x`.
- **Choosing by size was considered and rejected.** On the two transfer-function calls,
  `y` and `x` both have size 1, so size is just as ambiguous. It would also make the wiring
  depend on model order.
- **`Step() >> closed_loop(..., d=True)` becomes a refusal.** The loop's inputs are `r`
  and `d`. No code does this, and the message points to `closed_loop(r=Step(), d=...)`
  (AC-4).

**Decision 10 (agreed 2026-10-11): `+` is plain, `autowire` matches names only, and a
port has one name.** Custom diagrams are built from blocks whose port names already say
how they connect; `autowire` then joins equal names with no guessing.

1. **Ports outside, positions inside.** A port name is the block's external interface:
   wiring, `autowire`, plot signals, diagram drawings, error messages. The equations see one
   stacked `u = [port_1; port_2; ...]` in port order, the textbook `f(x, u)`, and read slices
   by position. Port order is fixed at construction, and renaming changes only the outside.
2. **One name per port, no aliases.** A second, external name would appear everywhere a port
   is named and double every concept. The only reason for aliases is to protect equations
   that read by name, and point 3 handles that.
3. **Reading by position is the default; reading by name stays supported.**
   - Library code, docs and examples read `u` by position. Only 8 library lines read their
     own inputs by name today, all in plants: the noisy pendulum and cart-pole, the two
     bicycles, the racecar, and the arm's tool force. They become positional
     (`tau, w = u[0], u[1]`) with identical results, and RULES gets one line.
   - Students and user blocks keep `get_port_values_from_u(u, "y")`, the readable helper.
4. **`rename_ports(**names)` on the base `System`.**
   - About 15 lines, written once. It moves each port to its new name, keeping the order,
     and updates the output dependency lists.
   - It works for every library block, catalog plant and user block.
   - It is chained at creation: `ProportionalController(2.0).rename_ports(u="u_cmd")`.
   - Leaf blocks only; a diagram names its boundary ports when it exposes them.
   - A renamed block opts out of `@` automation by design.
   - A keyword on every constructor was rejected: dozens of classes to edit, for uneven
     coverage.
5. **Sources name their signal at creation**, the most common case:
   - `Step(final_value=1.0, port="r")`, `Sine(..., port="n")`, `WhiteNoise(port="d")`,
     `TrajectorySource(..., port="u")`; the default stays `y`.
   - The four sources build their one output in the shared `Source.__init__`, so it is one
     keyword there, passed through by four constructors.
6. **No silent mistake.** An in-memory probe found two cases:
   - **A block whose equation asks for `"y"` after `y` was renamed** fails loudly but with
     a bare `KeyError: 'y'`, and `compile()` does not catch it. The name lookup will raise a
     clear message naming the block, its current inputs, and the fix.
   - **Swapping two names in one call** is silently wrong: the equation reads the other
     port, and in the probe the controller's sign flipped. `rename_ports` refuses any new
     name that is already a port of the block, so swaps and chains are impossible.

   A stale name can then only fail loudly, never resolve to another port. Renaming a block
   already placed in a diagram also fails loudly, and the docs say to rename at creation.
7. **`autowire()` connects by name only.**
   - Each unconnected input takes the one output with the same name and size.
   - An output may feed several inputs.
   - No match, or several, leaves the input open and listed in a warning; `strict=True`
     raises.
   - The three guessing rules go (source → `r`, plant `y` → measurement, the `r_` prefix).
     About 160 lines become about 30.
8. **`+` stays plain:** blocks side by side in a new flat diagram, with no wiring.

```python
ref = Step(final_value=1.0, port="r")
ctl = ProportionalController(2.0).rename_ports(u="u_cmd")
sat = Saturation(-1, 1).rename_ports(u="u_cmd", y="u")
plant = Integrator()
loop = (ref + ctl + sat + plant).autowire()   # every connection read from the names
```

**Evidence for decision 10.** A read-only replay of today's four `autowire` uses with the
name-only rule:

| Use | Name-only result |
|---|---|
| The bicycle cascade demo (planner, tracking, heading, yaw-rate and speed loops, vehicle; custom names) | the same wiring on all 10 inputs: it replaces ten `connect` lines |
| `Step() + ImpedanceController() + Pendulum()` (`00_core`, a demo) | the reference stays open: the step's output is named `y`. It becomes `Step() >> ctl @ plant`, or `Step(port="r")` |
| `Step() + ProportionalController() + Integrator()` (tests) | the reference open, the measurement ambiguous; rewritten to the new rule |
| The arm's second reference | stays open; one `connect` line, or `port="r_null"` on its source |

**Decision 14 (chosen 2026-10-11): `@` always means controller at plant; `feedback()`
closes an open loop on itself.**

**When `@` builds what, today** (`composition.py:398-413`; `@` always calls `closed_loop`):

| You write | Right side | The left block reads | `@` builds | Blocks |
|---|---|---|---|---|
| `PID(ports="reference") @ plant` | a System | `r` and `y` | plant `y` → controller `y`; the controller computes `r − y` itself | controller, plant |
| `StateFeedback(K) @ plant` | a System | `x` | plant `x` → controller `x` | controller, plant |
| `PID() @ plant`, `Lead() @ plant`, `TransferFunction(...) @ plant` | a System | only its error (`e`, or one plain input) | an `Error` block `e = r − y` in front of the controller | controller, plant, error |
| `L @ 1`, `L @ 0.5` | a number | n/a: `L` is the whole forward path | `L` closed on itself, `y` back through 1 or K | L's blocks, error, (gain) |
| `feedback(L, H, of=, sign=)` | (function) | n/a | the same as the row above, plus what `@` cannot say: a System on the way back, one component, positive sign | same |

- **The first three rows are all `controller @ plant`.** It is one loop, and the roles are
  assigned. The Error block appears only when the controller does not read the measurement
  itself.
- **The last two rows are block-diagram algebra.** `L` already holds controller and plant
  together; you only close it. `feedback()` is not another kind of loop: it is the code
  that adds the Error block, and the third row uses it too.
- **Same math, different form.** `C @ G` equals `(C >> G) @ 1`, which equals
  `feedback(C >> G)`, for a controller that reads only its error.
- **The confusion it removes:** `@` meant "controller at plant" with a System on the right,
  but "close on yourself" with a number on the right.

**Where `feedback()` guesses today** (probe on `dev` @ ce1094d):

| Call | Today |
|---|---|
| `H @ 1`, `Pendulum() @ 1` | closes on `u`; `y` returns (a Demux takes component 0 for the pendulum) |
| `(PID() >> H) @ 1` | closes on the entry `>>` remembered (`e`) |
| `(P(2) >> H) @ 1` | refused by a special check (AC-1), because the remembered entry was `r` |
| `TransferFunction(...) @ H` (default ports `u → y`) | treated as a compensator; ids `sys`, `sys2` |
| `feedback(L, through=TF)` / `L @ 0.5` | return block id `sensor` / `gain` |
| no `y` output on `L` | takes the *first* output |

The entry comes from three mechanisms:
- the `_composition_entry` memory (`_get_available_diagram_entry`);
- `error_input` (`feedback.py:132`);
- `refuse_two_port_entry`.

**Decision 14, item by item:**

0. **`@` needs a plant on the right.** The `L @ number` form is removed (a feature removal,
   decided by the maintainer). `L @ 1` is refused with: "`@` needs a plant on the right; to
   close an open loop on itself use `feedback(L)` or `feedback(L, K)`". This is the MATLAB
   and python-control notation.
   - `(C >> G) @ 1` becomes `feedback(C >> G)`.
   - `L @ 0.5` becomes `feedback(L, 0.5)`.
1. **Kept as is:**
   - the textbook loop `T = L / (1 + L H)` with a visible `Error` block;
   - the return path: `1`, a gain or a System;
   - `of=(port, i)`, the component-0 Demux for a scalar entry on a vector output, and
     `sign=+1`;
   - the function on the teaching surface.

   The loop's only boundary input stays `r`. A plant's `d` / `n` stay at nominal unless
   `closed_loop(d=, n=)` asks for them (AC-4).
2. **The entry is plain, with no memory.** `L` closes on its command input, by the
   `command_port` rule: `u`, else its one free input that is not `d` or `n`, else refuse.
   - A diagram counts its free boundary inputs, as for `>>`.
   - `feedback(PID >> H)` closes on `e`, the one free input.
   - `feedback(P >> H)` has two free inputs (`r`, `y`) and is refused. The message points to
     `P @ H`, as today, without the special check.
   - `error_input`, `refuse_two_port_entry` and the entry memory go.
3. **What returns is plain:** `of=`, else `y`, else `L`'s only output, else refuse. The
   "first output" guess goes.
4. **One name for the return block: `filter`.**
   - It is the block's id, whether a gain or a System (today `gain` or `sensor`).
   - The keyword `through=` becomes `filter=`, the word `closed_loop(filter=)` and
     `sensitivity(..., filter=)` already use. Positional calls (`feedback(C >> H, F)`) are
     unchanged.
   - The junction stays `error` (`sum` for positive feedback).
   - The `filter` is a plain helper, never the `estimator` role (ruling 2026-10-11, §3.2).
5. **Roles: only `controller @ plant` assigns them.**
   - In `feedback(L)`, a leaf `L` is `plant`, the one system in the loop.
   - A diagram `L` is laid out flat with its own ids (decision 8), so `feedback(C >> G)` is
     plain topology.
   - The role-keeping form of the same loop is `C @ G`, or `pid @ (sat >> plant)` with a
     plant box (decision 12).
6. **`C @ G` dispatch, in three cases:**
   - if C reads `y`, it is the measurement loop;
   - if C reads `x`, it is the state loop;
   - otherwise the error drives C's entry, by the rule of item 2.

   This keeps a plain `TransferFunction` compensator working (tested today) without
   requiring a port `e`. It refines `block_roles` (§3.3).
7. **One junction helper**, shared by `feedback` and `C @ G`. It adds the Error, the
   optional `filter` and the Demux. `C @ G` is then the same loop as `feedback(C >> G)`, with
   the role ids `controller` and `plant`, where the plant may be a box.

**Migration when the code lands (AC-OP):**
- `L @ number` becomes `feedback(L, ...)`:
  - 5 test lines;
  - the anti-windup demo's second loop, as `feedback(pid2 >> sat >> sys)`, or
    `pid2 @ (sat >> sys)` to keep the roles;
  - the `L @ 1` mention in the frequency-response notebook table and the `@` docstrings.
- The tests' `through=` becomes `filter=`.
- The asserts on ids `gain` / `sensor` become `filter`.
- The asserts on `sys` / `sys2` ids follow AC-2b.

**Decision 15 (chosen 2026-10-11): `%` runs something with no continuous state on a clock;
`computer @ plant` follows the same rules as `@`.**

`block % dt` runs a block on a computer clock (a `Computer`). `computer @ plant` closes a
sampled loop: the computer reads the plant through a sampler and drives it through a
zero-order hold.

**Scope (maintainer, 2026-10-11).** The hybrid loop is not a mainstream core component
yet: today it is mostly a side helper for MPC simulation. What exists should still behave
like the core, so that integrating it later is easy. Decision 15 therefore splits in two:
- **Keep in step (AC-8a), now.** As each core rule lands, the hybrid helpers follow it:
  - the same ids;
  - the same dispatch as `@`;
  - boxes on both sides;
  - no port guessing;
  - no port keywords;
  - clear refusals.

  No new hybrid feature is built for this.
- **Mainstream integration (AC-8b), later, with S31 (v0.9):**
  - `source >> sampled loop`;
  - `(P >> Sat) % dt` (a flow diagram with no continuous state converted to a step
    diagram);
  - the hybrid loop as a `System` for plots, cost and Monte Carlo;
  - sampled dynamic controllers, which are future work.

**What `%` does today** (probe on `dev` @ 2bf1c2d):

| You write | Today |
|---|---|
| `P(0.5) % 0.01 @ Integrator()` | works: computer ids `ctl`, plant ids `plant` |
| `StateFeedback(K) % 0.01 @ Pendulum()`, `SMC % 0.05 @ Pendulum()` | works |
| `PID() % 0.01 @ plant`, `Lead() % 0.01 @ plant` | refused, with a wrong message: "continuous **plant** 'PID' belongs in Simulator" |
| `discretize(PID(), 0.01) % 0.01 @ plant` | refused: `discretize` returns a block with one stacked `u` and no output port (the P6 follow-up) |
| `P(ports="error") % 0.01 @ plant` | refused: no Error block in sampled loops |
| `P % 0.01 @ (Sat >> Integrator())` | refused: "a single-subsystem plant diagram or leaf plant" |
| `(P >> Sat) % 0.01 @ plant` | refused: `>>` builds a flow diagram and `%` wants a step diagram, although nothing in it has a continuous state |
| `Step() >> (P % 0.01 @ plant)` | refused: a `HybridDiagram` is not a `System` (S31, v0.9); the reference goes through `compute_forced(r, input_port_id="r")` |

- **The port resolver guesses** (`resolve_hybrid_feedback_ports`,
  `hybrid_composition.py:148`):
  - for a multi-block computer, the output is `u_nom`, else `u`, else `u_ff`;
  - the plant's input and output fall back to the *first* port.
- **Users.**
  - `%`: 3 tests, the MPC tests (`mpc % dt @ plant`), and the showcase notebook.
  - `hybrid_closed_loop` with port keywords: 2 demos (`sampled_smc_pendulum`,
    `hybrid_multi_rate`) and the GRO501 `racecar_toward_mpc` notebook.
  - That notebook hand-builds a plant box `inner` (edge `u` → `y`: speed loop, car, pose
    Demux). It passes `computer_out="u_ff"` because `@` refuses multi-block plants and MPC
    gives `u_ff`.

**Decision 15, item by item:**

1. **What `%` accepts:** anything with no continuous state.
   - Accepted: a static block (P, state feedback, sliding mode), a step block, a step
     diagram, or a flow diagram whose blocks have no continuous state. `(P >> Sat) % dt`
     works: it is converted to a step diagram.
   - A block with a continuous state (PID, Lead, a transfer function) raises
     `NotImplementedError`: "sampling a controller with continuous states (PID) is future
     work; use a static or discrete controller". This is the maintainer's choice.
     - It replaces today's wrong "continuous plant" message.
     - It does not depend on the `discretize` port fix.
     - Sampled dynamic controllers are a future-work item in TODO.
2. **`computer @ plant` dispatches like `@` (decision 14):**
   - the computer reads `y`: the sampler reads the plant's `y`;
   - it reads `x`: the sampler reads the plant's `x`;
   - otherwise an `Error` block `e = r − y` sits **inside the computer**, on the sampled
     measurement. This is the digital error.
3. **Boxes (decision 12) on both sides:**
   - the plant may be a box with the edge `u` → `y`. The GRO501 racecar becomes
     `mpc % dt @ inner`, once MPC gives `u` (decision 2);
   - the computer may be a box (filter + controller, multi-rate) with the edge `r`, `y` (or
     `x`) → `u`;
   - the guessing chain (`u_nom` / `u` / `u_ff`, the first port) goes, and the dual-rate MPC
     composite exposes `u`.
4. **Where things sit:**
   - the filter and the error run on the computer, inside the computer box (the filter
     ruling, §3.4);
   - the sampler and the zero-order hold are the boundary between the two boxes, not blocks;
   - the roles are the hybrid fields: `.plant` is the plant box, and `.computer` holds the
     controller;
   - inside the computer the ids are `controller`, `error`, `filter` (AC-2b), and the
     multi-rate schedule's keys use those ids.
5. **The reference.** `Step() >> ctl % dt @ plant` reads like the continuous loop.
   - The source sits on the continuous side, and the computer samples it at its own rate:
     the reference is a world signal the computer reads.
   - It lands with AC-8b (mainstream integration). Until then,
     `compute_forced(r, input_port_id="r")` as today.
6. **Manual path and keywords.** Custom names use `HybridDiagram` + `connect_boundary`.
   - The port keywords of `hybrid_closed_loop` go, like `closed_loop`'s (decision 2).
   - Migration: the 2 demos use standard names or the manual path, and the GRO501
     notebook becomes `mpc % dt @ inner`.
   - MPC keeps its own `%` (its `dt` check).

**The port keywords that go** (decisions 2 and 15; facts on `dev` @ 09ee075). These exist
only to tell a loop builder about non-standard port names. Standard names plus hand wiring
(`connect`, `connect_boundary`), `rename_ports` and `expose` cover the same cases.

*1. Keywords of `closed_loop` / `@`, removed in AC-3.*

| Keyword | What it does | Callers outside the library |
|---|---|---|
| `ref_port="r"` | names the controller's reference input | none |
| `measurement_port="y"` | names the controller's measurement input | none |
| `control_port="u"` | names the controller's command output | none |
| `plant_input_port="u"` | names the plant's command input | none |
| `plant_output_port="y"` | names the plant's measured output | 1 test, which checks a refusal |
| `output_port="y"` | names the loop's boundary output | 1 test, which checks a refusal |
| `feedback="auto"` / `"y"` / `"qdq"` | forces how the measurement is wired | `"qdq"` goes with decision 11; `"y"` only in tests |

The 2 refusal tests go.

*2. Override attributes on block classes, removed in AC-3.* `measurement_port`, `ref_port`
and `control_port` are class attributes on 6 library classes, and `feedback_profile`
appears in 13 places.

| Class | Overrides today | After |
|---|---|---|
| MPC controller | `y` → `u_ff` | ports `x` → `u` (the MPC rename of decision 2) |
| `LookupPolicy`, the Gym policy, the neural controller | `x` → `u`, no reference | nothing to declare: the ports already say `x` → `u` |
| the geometric controller | `y` → `u`, no reference | nothing to declare |
| `TransferFunction` (`ports=` layout) | sets the three names from the layout | nothing: the layout's port names are read directly |
| impedance, model-based and task controllers | `feedback_profile` | nothing: their `r`, `y`, `u` ports say it |

*3. Keywords of `hybrid_closed_loop`, removed in AC-8a:* `computer_out`, `computer_in`,
`plant_in`, `plant_out`, `ref_port`, `output_port`, `extra_boundaries`.

| Callers | Today | After |
|---|---|---|
| `sampled_smc_pendulum` demo, 4 tests | pass the defaults (`computer_in="y"`, `plant_out="y"`) | drop the keywords |
| GRO501 `racecar_toward_mpc`, 4 MPC tests | `computer_out="u_ff"` | `mpc % dt @ inner`, once MPC gives `u` |
| `hybrid_multi_rate` demo, 1 test | `computer_out="u_cmd"`, `computer_in="y_meas"` | name the step diagram's ports `u` and `y`, or wire by hand |
| `projects/pathtracking/mpc_v1/run_demo.py` | `x_nom`, `y_mpc` (custom names) | the manual path: `HybridDiagram(...)` + `connect_boundary(...)` |
| `extra_boundaries` | no caller | `connect_boundary`, which exists |

*What stays, because these are not port keywords:*
- `closed_loop`'s loop inputs: `r=`, `d=`, `n=`, `filter=`, `estimator=`, `validate=`;
- `hybrid_closed_loop`'s `schedule=` and `computer=`;
- `feedback(L, of=(port, i))`, an explicit choice of what returns (decision 14);
- naming at creation: `Demux(port=)` and the sources' `port=` (decision 10);
- analysis keywords such as `bode(of=, wrt=)`.

**Settled since:**
- what `@` builds and names: it nests a box under its role key and keeps a leaf a leaf,
  while `>>` and `+` stay flat (decision 12, §3.4);
- `feedback()` and its preference for `r` (decision 14);
- `%` for sampled loops (decision 15).

**The operator review is closed (2026-10-11).** Every operator is planned: `+`,
`autowire`, `>>`, `@`, `feedback` and `%`. Code can start with AC-OP.

## 4. Steps

Each step lands on its own commit with the checks of §6.

The census baseline is built in AC-1 and reused. It covers about 40 loops across every
path, plus the course controllers, and records for each:
- ids, connections, and boundary ports with labels and units;
- entry and output, names and params keys;
- `x0` and a fixed-step history;
- roles and control-law sweeps.

It is captured twice and `cmp`'d.

| Step | Scope | Lane, rung |
|---|---|---|
| **AC-0** | No controller classes; `plot_control_law` on `System`; the four course notebooks migrated (import line and base class) | agent, after decision 7; v0.2 B4 |
| **AC-1** | **Landed 2026-10-10.** Safe fixes and pinning tests, one commit each with its own test: D1 and D6 refused (an Error junction on a block that reads its own measurement); D2 (sampled state feedback runs); D4 (unknown or unused `closed_loop` keywords refused); A (`>>` reads a hand-wired diagram's declared output); B (a diagram `x0` set by the user is kept, until S29); C (a static block whose output is `u` simulates; the command's signal name waits for AC-5); D (`autowire` never routes one command to two inputs, and warns when it leaves inputs open); the `t = 0` scoring; `disturbances={"u"}` refused. `TestDispatchTable` pins the loop each library controller gets. The census is a scratchpad script (AGENTS recipe), captured again before each later step | agent (bug fixes) |
| **AC-OP** | The operators of §3.10, one commit each, with the census and the replay of every `>>` and `autowire` call: no operator modifies its operands (decision 8); the plain `>>` rule (decision 9); then decision 10 (the 8 positional plant lines, `rename_ports` with its two safety rules, `port=` on sources, name-only `autowire`, and the migration of `00_core`, two demos and their tests); then `expose` for role boxes (decision 12); then decision 14 (`@` needs a plant, `L @ number` refused and migrated to `feedback`, the plain entry and output rules, `filter=` for `through=`, one junction helper) | core, after the operator review closes; v0.2 B4 |
| **AC-2** | `roles()`, `block_roles()` and `command_port()` in `core/feedback.py`, with every reader switched; reserved ids raise on a clash. (a) Byte-identical census, with today's ids mapped. (b) The id rename in flow and hybrid: `ctl` → `controller`, `sys` → `plant`, `ref` → `reference`; sources `disturbance` and `noise`; helpers `error` and `filter`; the expected diff is ids, params keys, signal names and stream names. (c) The letters of decision 13 in the same pass, no alias: the two noisy plants' ports (`v_noise` becomes `n`), `closed_loop(d=, n=)`, `add_loop_inputs`, the sensitivity module docstrings and the P5 notebook, `diagram_noise_ports`, the tests and RULES 4.9 | v0.2 B4, before RN-4 |
| **AC-3** | Composition on standard names. Removed: profiles, overrides, port keywords, `error_input`, `_composition_*`, shape ids, the camera copy. Changed: role boxes nested by `@` with the edge check (decision 12); `error_input` and `refuse_two_port_entry` removed (decision 14); plants expose `x`, filters stop (check users first), and an `x` controller on a plant without `x` is refused (decision 1); MPC renamed; `filter=`; refusals point to hand wiring. The course notebooks drop `feedback_profile`. Decision 11: `MechanicalSystem.h` returns `[q; dq]`; removed are the `q` / `dq` ports, `StateSliceOutputPort`, `closed_loop_qdq`, `feedback="qdq"` and the Mux branch (about 140 lines); the three demos, the showcase UR5 cell and the tests become `ctl @ plant`; the IK guess reads the state; DESIGN lines 205 and 485-490 | v0.2 B4, gates P4 |
| **AC-4** | Loop inputs: a plant that declares `d` and `n` owns them; sources as values, named `reference`, `disturbance`, `noise`; `estimator=` (with P4); randomness A9 / D24 reconciled. The automatic wrapper is deferred (decision 3) | beside P4, before P11 |
| **AC-5** | Plot auto mode, role selectors, id paths, labels, units and colours by role, one default; signals inside a role box found by object and by path (`plant:motor:y`) | plotting lane, v0.2 D |
| **AC-6** | The camera rule at animate time, looking inside the `plant` box; no compensator skin inside a loop | plotting lane |
| **AC-7** | Cost relative to the plant across every evaluator and design tool (§3.5), on the `plant` box's state and `u`: one compiled loop for the three backends, `command_port`, stateful and time-varying laws, cost params, one price; Gym and the rollout environment aligned | v0.3, with RN-4 / RN-5 |
| **AC-8a** | The hybrid helpers keep in step with the core (decision 15; the hybrid loop is a side helper for MPC simulation, not mainstream yet). Fields as roles; the same ids (AC-2b); a continuous controller in `%` raises `NotImplementedError`; `computer @ plant` dispatches like `@`, with the error inside the computer; boxes on both sides, and the `u_nom` / `u_ff` / first-port guesses go; the dual-rate MPC composite exposes `u`; the `hybrid_closed_loop` port keywords go (2 demos and the GRO501 racecar notebook migrated). No new hybrid feature | each item with the core step it mirrors (AC-2b, AC-3, the MPC rename) |
| **AC-8b** | Mainstream integration: `source >> sampled loop` with the source on the continuous side; `(P >> Sat) % dt`; the hybrid loop as a `System` for plots, cost and Monte Carlo; sampled dynamic controllers (future work, TODO beside P6) | with S31, v0.9 |

**Done when:**

- `ctl @ plant`, the same loop hand-wired with the role ids, and `closed_loop` with sources
  give the same roles, plots, camera and Monte Carlo score;
- the census is byte-identical for every case that worked before, apart from AC-2b's
  expected id diff;
- every defect of §2 has a test.

**Absorbed or reshaped workboard steps:**

- P10 is this plan.
- T6's composition half is AC-3 plus the textbook pass after it.
- TB-b's loop part is AC-3.
- S65's automatic-dt and hold-model rows are noted under AC-1 (C) and left to S65.
- S66 lands after AC-4.
- A4's "one closed-loop name" is AC-3; its default ids are AC-2b.
- P4 builds its estimator to the table of §3.2 (`u`, `y` → `x`, id `estimator`); the
  observer + state feedback composite is a `controller` holding `estimator` and `gain`.
- P11's `Controller(feedback=…)` is unnecessary (§5 decision 6).
- RN-4 and RN-5 consume AC-2b, AC-4 and AC-7.
- S62 renames the LQR module with no alias, since the freeze is paused.
- S70 turns the refusals into `WiringError`.
- S31 takes the hybrid vocabulary.

## 5. Decisions for the maintainer

Each is recorded in ROADMAP §6.

1. **The principles of §3.1 and the two tables of §3.2.** This includes:
   - ports for wiring, and three reserved role ids;
   - `x` only where needed: plants expose it, other blocks do not (§3.2);
   - RULES 4.9 extended with `e`, `x` and the ids, its `w` / `v` becoming `d` / `n`
     (decision 13);
   - a DESIGN §4 "Conventions" section.

   *Approved on 2026-10-11, amended:* `x` only where needed, not on every block.
2. **Standard names only.** Profiles, overrides, port keywords and shape ids are removed;
   MPC is renamed to `x` → `u`; custom names are wired by hand. The exact list is in
   §3.10, *The port keywords that go*. *Approved on 2026-10-11.*
3. **Loop inputs.** The plant that declares `d` and `n` owns them;
   `r=`, `d=`, `n=` accept a source block; `estimator=`. *Approved on 2026-10-11, the
   automatic wrapper deferred* (§3.4).
4. **Cost relative to the plant.** Scored by identity on `trajectory_of(problem.sys)`, with
   `command_port` the only decided input for every tool. This changes plants with several
   inputs (`*WithNoisePort`) in planners, LQR, `place`, value iteration and Gym.
   *Approved on 2026-10-11, in v0.3.*
5. **Role ids** `plant`, `controller` and `estimator`, as reserved subsystem keys. Blocks
   get words and signals get symbols.
   - Renamed once, in flow and hybrid, before RN-4:
     - `ctl` → `controller` (about 216 string uses in tests, examples, the library and one
       course notebook);
     - `sys` → `plant`;
     - `ref` → `reference` (about 31).
   - Sources are named `disturbance` and `noise`.
   - A role id wins over `System.id`.
   - `estimator` is a state estimate only; filters and sensors are not estimators. A filter
     is the helper `filter` at the loop's top level (ruling 2026-10-11, §3.2 and §3.4).

   *Approved on 2026-10-11, in v0.2.*
6. **P11's `Controller(feedback=…)`.** A student writes a `System` with the ports of §3.2
   and gets everything. *Approved on 2026-10-11: the request is dropped.*
7. **No controller classes, no aliases; `plot_control_law` on `System`.** *Agreed in
   principle on 2026-10-10.*
8. **No operator modifies its operands.** `+`, `>>` and `@` return a new flat diagram that
   shares the operands' blocks; this reverses the v0.1 DESIGN record. *Agreed on
   2026-10-10.*
9. **`>>` is a plain serial connection** by the four rules of §3.10: one-to-one, same name,
   `y` → `u`, else refuse. *Agreed in principle on 2026-10-10.*
10. **`+` is plain, `autowire` matches names only, one name per port.** Ports outside,
    positions inside: library equations read `u` by position, and the name helper stays for
    students. `rename_ports` at creation, `port=` on sources, and no silent mistake.
    *Agreed on 2026-10-11.*
11. **A mechanical plant gives `y` and `x`.** `y` is `[q; dq]` by default and `x` the state.
    The `q` / `dq` ports, `StateSliceOutputPort`, `closed_loop_qdq` (a user-importable name,
    removed by this decision), `feedback="qdq"` and the Mux branch go.
    - Impedance controllers are unaffected, whether they measure the velocity or compute it.
      `ImpedanceController`, its integral form, `JointImpedance`, `TaskImpedance`, computed
      torque and sliding mode read `y` straight; PID reads the position through a Demux of
      `y`. None of them reads the `q` or `dq` port.
    - `closed_loop_qdq` built the same loop plus a Mux on the three demos' pairs, with
      identical `f`.
    - A positions-only sensor is a custom port when someone needs one.

    *Agreed on 2026-10-11.*
12. **Role boxes.** A plant, a controller or an estimator is one block or many, with the
    standard ports on its edge (§3.2). `@` nests a box under its role key and keeps a leaf a
    leaf; `>>` and `+` stay flat. A box is built by hand, with `>>` for a serial plant, or
    with the new `expose(*names)`. The tools look inside boxes (AC-5 to AC-7). *Agreed in
    principle on 2026-10-11.*
13. **Disturbance `d`, measurement noise `n`; `v` is velocity.** These are the classical
    loop letters (Åström & Murray, Doyle–Francis–Tannenbaum, Skogestad), the loop the P5
    sensitivity notebook teaches.
    - `v` stays free for generalized velocity, as in the plant equations, Drake and
      Pinocchio; `w` stays free for angular speed and frequency.
    - `n` is also the number of states and `d(q, v)` the damping method. Those are an
      attribute and a method, not ports, so they cause no wiring mistake.
    - The source ids stay `disturbance` and `noise`.
    - Renamed in AC-2c, no alias.

    *Chosen on 2026-10-11.*
14. **`@` always means controller at plant; `feedback()` closes an open loop on itself.**
    - `L @ number` is removed: `feedback(L)` and `feedback(L, K)`, the MATLAB and
      python-control notation.
    - `feedback` closes on the command input and returns `y`, with no memory and no guess.
    - The return block is `filter` (id and keyword).
    - Only `controller @ plant` assigns roles.

    *Chosen on 2026-10-11.*
15. **`%` runs something with no continuous state on a clock; `computer @ plant` follows
    the rules of `@`.**
    - A continuous controller (PID, Lead) raises `NotImplementedError`: sampled dynamic
      controllers are future work.
    - The error and the filter sit inside the computer.
    - Boxes are allowed on both sides.
    - The reference is sampled from the continuous side.
    - The port keywords of `hybrid_closed_loop` go.
    - Scope: the hybrid loop is a side helper for MPC simulation today. AC-8a keeps it in
      step with each core rule as that rule lands; AC-8b integrates it into the mainstream
      later, with S31.

    *Chosen on 2026-10-11.* With it, the operator review is closed.

**Ruling, not a decision.** The name freeze (ROADMAP §4.1 gate 7) is paused between terms
(maintainer, 2026-10-10). A rename migrates every course notebook in the same commit, with
no alias.

## 6. Checks for every step

- `ruff check .` and `ruff format --check .`.
- The targeted tests: `test_feedback_composition`, `test_core`, `test_hybrid`,
  `test_simulation`, `test_mpc`, `test_graphics`, `test_mechanical_robotics`, and the
  planning and evaluation tests touched.
- A census `cmp` after each sub-step: byte-identical for AC-2a and the refactor parts of
  AC-3. Elsewhere only these may change:
  - AC-2b's ids;
  - cases that failed before;
  - new cases.
- At the end of each step:
  - the full `pytest` suite, the regression gates and the flagship demos;
  - a notebook smoke run of `06_hybrid`, `sensitivity_functions`, `frequency_response`,
    `double_integrator_policy_evaluation`, `cartpole_static_controller`,
    `cartpole_dynamic_controller`, `ode_simulation` and `racecar_toward_mpc`;
  - `sphinx-build -W` for docstrings.

## 7. Not in scope

- No new operator, no options on `@`, no `Loop` block.
- No auto-wiring in `>>`: it connects one output to one input by the fixed rule of §3.10.
- No new base classes, traits, intent declarations or override attributes.
- No inference of roles from the graph, and no "deepest plant" rule.
- No tuple operands.
- No hybrid features before S31.
- No flip of `ProportionalController`'s default layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
- No renaming of named-port plants: they are wired by hand.
