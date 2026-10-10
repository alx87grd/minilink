# Automation by convention: plants, controllers, loops, plots and tools

**Status:** v2, analysis and recommendation (2026-10-10), for the maintainer's ruling on §5.
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

The defects marked *reproduced* were re-run by hand. The loop-building defects D1–D8 come from [2026-10-10-feedback-loop-review.md](../reviews/2026-10-10-feedback-loop-review.md).

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

| Block | Reads | Gives |
|---|---|---|
| Plant | `u` (the command), `w` (disturbance), `v` (measurement noise), other names held at nominal | `y` (measured output), `x` (state), `q` / `dq` (positions and velocities) |
| Controller | `e` (the loop inserts `Error`, `e = r − y`), or `y`, or `x`; plus `r` | `u` |
| Estimator | `u` and `y` | `x`, the estimate (labelled x̂) |

- Every leaf plant exposes `x`: `expose_state` becomes the default.
- Dimensions decide the Mux (`[q; dq]`) and the Demux (component 0).
- Other names are allowed on any block, and the automation leaves them alone:
  - the controller extras `x_ff` and `z`;
  - the manipulator outputs `p` and `pdot`;
  - the input `r_null`.

**Ids: what role a block plays.**

The rule is: **blocks get words, signals get symbols.**
- Ports are mathematical variables (`u`, `r`, `w`, `v`, `y`, `x`, `e`, `q`, `dq`).
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
| `estimator` | the state estimate the controller reads | `closed_loop(estimator=)`, or inside a `controller` composite |

- The three role ids are reserved.
  - There is one of each per diagram level, and a clash raises; it is never suffixed.
  - A cascade nests (`plant/plant`).
  - A second physical plant gets a name of its own and is handled by hand.
- Variable names stay free: `ctl @ plant` gives the keys `controller` and `plant`.
- Every other id is a name, not a role, but it still matters, because ids are the params
  keys and the random-stream names:
  - sources take the word for the input they drive: `reference`, `disturbance`, `noise`,
    or the port's own name for a custom port (`wind`);
  - helpers are `error` and `filter`.

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
    S31 keeps.
  - There is no "deepest plant" rule. In a cascade the outer loop's plant is the inner
    loop (compositional closure), and `plant/plant` reaches the motor.
  - With no `plant` id the plant is `None`: a tool that needs it raises, and plots and the
    camera keep today's defaults.
- **`block_roles(block)`: what a block reads and commands.** It reads the port names only,
  and serves composition:
  - the measurement (`e`, `y` or `x`);
  - the reference (`r` or none);
  - the command (`u`);
  - whether the block is error-driven (it has the port `e`);
  - whether it is stateful (from `n`).

  It replaces `feedback_ports`, `error_input`, the `PROFILE_PORTS` lookups and the five
  fallback chains.
- **`command_port(sys)`: which input a decision drives.**
  - It is `u`, else the single input that is not `w` or `v`, else it refuses.
  - It replaces `action_port_of` (`control/neural.py:12`).
  - Gym, the cost, the design tools and a cascade's outer `@` all use it.

### 3.4 Composition writes the ids

- **`@`:**
  - `ctl @ plant` gives `controller` and `plant`.
  - `G @ 1` and `G @ K` give the left operand `plant` and the return block `filter`.
- **Diagram operands are nested under their role, not inlined.**
  - A closed loop used as a plant is `plant`, holding its own `controller` and `plant`.
  - P4's `compensator(observer, K)` is `controller`, holding `estimator` and `gain`.
  - Ids stay unique at each level, and the params keys follow the nesting.
- **Loop inputs.** `closed_loop(w=, v=)` on a plant without those ports nests a plant
  wrapper whose boundary is `u`, `w`, `v`, so the plant always owns its disturbance and
  noise.
  - `w` is never charged as control effort.
  - `v` enters what the controller reads, `y` or `x`.

  A plant that declares `w` / `v` itself is used as is.
- **`>>` and `+`** stop minting role ids from block shape. They name blocks plainly, and a
  hand-written role id is kept.
- In a shortcut, a role id wins over `System.id`.
- The composition-time state goes: `_composition_entry`, `_composition_output` and the
  camera copy.

### 3.5 What each use case becomes

| Use case | After |
|---|---|
| Composition | One port-based dispatch: an error junction, reads `y`, reads `x`, a loop gain, or a refusal that points to hand wiring. `closed_loop(controller, plant, *, r, w, v, filter, estimator)`: each loop input is `False`, `True` (a boundary port) or a source block. No port keywords. |
| Plot signals | `signals="plant"`, `"controller"` or `"estimator"` selects every port of that block, as `"plant:y"` selects one. The default *auto* mode shows every port of the plant and the controller, each wire once; the reference, the error, `w` and `v` come in through their inputs. Signal names take id paths (`plant/plant:y`), reconstructed recursively. Labels and units are copied onto exposed ports, and colours follow roles. With no roles, plots keep today's default. A leaf keeps `("x", "u")`. `compute_trajectory(show=True)` and the planning and comparison plots use the same default. |
| Animation | Resolved at animate time. The camera comes from the one drawable block anywhere in the tree, else the drawable block on the `plant` path, else an auto-fit view. A compensator draws no skin inside a loop. `camera_priority` is removed, and `scene_grid` and the follow frame come from the same block. |
| Cost and evaluation | The cost is relative to the plant. The tools build `policy @ problem.sys` (compiled once for all three Monte Carlo backends) and score `trajectory_of(problem.sys)` by identity: `x` is the plant's state, and `u` is its `command_port`. Controller and estimator states are integrated, never scored. A dynamic controller is integrated, not refused; `t` is honoured; the cost's params are passed; there is one price for leaving `X`. The `x0` draw is on the plant only, and the controller and the estimator start at their own `x0`. |
| Design and planning tools | DP, LQR, `place`, trajopt, MPC and value iteration decide `command_port` only. `w` and `v` are never decision variables; they are drawn or held at nominal. `discretize` keeps the plant's ports. |
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
| MPC (today; T6, MP-4) | `x` (+ `r`) → `u` | the internal cost is the same `CostFunction` on the model's `x` and `u`; `v` reaches the `x` it reads | `y` → `x`, `u_ff` → `u`; the racecar exposes `x`; the undiscounted internal cost is a defect to fix |
| Dual-rate MPC (S31) | replan and broadcast | roles from the hybrid fields; the composite exposes `x` → `u` | the broadcast's `u_nom` → `u`; replan's `u` not exposed |
| MPPI (S72, v0.3) | an open-loop source on the plant's `u` | the evaluator wires `controller` and `plant` itself | none |
| CBF safety filter (research lane) | `CBFSafetyFilter(controller, …)`, `x` → `u` | a filtered controller is a controller; hand-wired, `x` fans out by hand; the cost sees the plant's command | `u_nom` and `u_safe` stay internal |
| Actuators, zero-order hold | inside the nested `plant` | part of the plant (the textbook H); to score the saturated command, put the saturation on the controller side | none |
| Loop noise and randomness (AC-4; RN-1 to RN-5) | `w` and `v`, owned by the plant or its wrapper | the source ids `disturbance` and `noise` name the random streams; coloured noise (`WhiteNoise >> LowPassFilter`) is the `w` source, not a role; Kalman design reads the plant's `w` and `v` | ids renamed before RN-4 |
| RL, Gym, policies with state | `x` → `u` | the observation is what the controller reads; the action is `command_port`; the reward is `−g` on the plant; the loop integrates a stateful policy | Gym's observation space |
| Hybrid as a System (S31, v0.9) | `[plant; computer]` | the roles are its fields; the ids are renamed for params keys and streams | `hybrid_closed_loop` keywords settled there |
| A cost block, the differentiable cost V1 | reads `plant:x` and the command | id `cost`, not a role | none |
| Discrete time (P6, v0.9) | `discretize(plant)` | keeps the plant's ports (follow-up) | today it stacks the inputs and drops `q`, `dq` |
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
- AC-4: the keys inside the `w` / `v` plant wrapper, and how its boundary re-exposes the
  leaf's `x`, `q` and `dq`.

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
| **AC-1** | Safe fixes and pinning tests: D1 (refuse), D2, D4, D6, A, B, C, D, the `t = 0` scoring, `disturbances={"u"}` refused; a `TestDispatchTable`; the census baseline | agent (bug fixes), now |
| **AC-2** | `roles()`, `block_roles()` and `command_port()` in `core/feedback.py`, with every reader switched; reserved ids raise on a clash. (a) Byte-identical census, with today's ids mapped. (b) The id rename in flow and hybrid: `ctl` → `controller`, `sys` → `plant`, `ref` → `reference`; sources `disturbance` and `noise`; helpers `error` and `filter`; the expected diff is ids, params keys, signal names and stream names | after decisions 1 and 5; v0.2 B4, before RN-4 |
| **AC-3** | Composition on standard names. Removed: profiles, overrides, port keywords, `error_input`, `_composition_*`, shape ids, the camera copy. Changed: diagram operands nested; `G @ K` naming; leaves expose `x`; MPC renamed; `filter=`; refusals point to hand wiring. The course notebooks drop `feedback_profile` | after decision 2; v0.2 B4, gates P4 |
| **AC-4** | Loop inputs: the plant wrapper owns `w` and `v`; sources as values, named `reference`, `disturbance`, `noise`; `estimator=` (with P4); randomness A9 / D24 reconciled | after decision 3; beside P4, before P11 |
| **AC-5** | Plot auto mode, role selectors, id paths, labels, units and colours by role, one default | plotting lane, v0.2 D |
| **AC-6** | The camera rule at animate time; no compensator skin inside a loop | plotting lane |
| **AC-7** | Cost relative to the plant across every evaluator and design tool (§3.5): one compiled loop for the three backends, `command_port`, stateful and time-varying laws, cost params, one price; Gym and the rollout environment aligned | after decision 4; v0.3, with RN-4 / RN-5 |
| **AC-8** | Hybrid on the same rules: fields as roles, the `hybrid_closed_loop` keywords, the dual-rate MPC composite | with S31, v0.9 |

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
   - every leaf exposing `x`;
   - RULES 4.9 extended with `e`, `x`, `q`, `dq` and the ids;
   - a DESIGN §4 "Conventions" section.

   *Recommended: yes.*
2. **Standard names only.** Profiles, overrides, port keywords and shape ids are removed;
   MPC is renamed to `x` → `u`; custom names are wired by hand. *Recommended: yes.*
3. **Loop inputs.** The plant, or the wrapper the loop builds around it, owns `w` and `v`;
   `r=`, `w=`, `v=` accept a source block; `estimator=`. *Recommended: yes.*
4. **Cost relative to the plant.** Scored by identity on `trajectory_of(problem.sys)`, with
   `command_port` the only decided input for every tool. This changes plants with several
   inputs (`*WithNoisePort`) in planners, LQR, `place`, value iteration and Gym.
   *Recommended: yes, in v0.3.*
5. **Role ids** `plant`, `controller` and `estimator`, as reserved subsystem keys. Blocks
   get words and signals get symbols.
   - Renamed once, in flow and hybrid, before RN-4:
     - `ctl` → `controller` (about 216 string uses in tests, examples, the library and one
       course notebook);
     - `sys` → `plant`;
     - `ref` → `reference` (about 31).
   - Sources are named `disturbance` and `noise`.
   - A role id wins over `System.id`.

   *Recommended: yes, in v0.2.*
6. **P11's `Controller(feedback=…)`.** A student writes a `System` with the ports of §3.2
   and gets everything. *Recommended: drop the ask.*
7. **No controller classes, no aliases; `plot_control_law` on `System`.** *Agreed in
   principle on 2026-10-10.*

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
- No new base classes, traits, intent declarations or override attributes.
- No inference of roles from the graph, and no "deepest plant" rule.
- No tuple operands.
- No hybrid features before S31.
- No flip of `ProportionalController`'s default layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
- No renaming of named-port plants: they are wired by hand.
