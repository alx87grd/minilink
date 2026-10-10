# Automation by convention: plants, controllers, loops, plots and tools

**Status:** analysis and recommendation (2026-10-10), for the maintainer's ruling on §5.
**Rung:** v0.2 wave B4 (P10, widened) for AC-1 to AC-3; AC-4 to AC-8 placed in §4.
**Evidence:** seven read-only surveys on `dev` at `d1b4bae`:
- loop dispatch internals;
- a census of every shortcut and hand-wired loop;
- controller declarations;
- plant disturbance and noise ports;
- plotting and animation automation;
- tools that take or return a plant and a controller;
- every other core heuristic.

The defects marked *reproduced* were re-run by hand. The loop-building defects D1–D8 come from [2026-10-10-feedback-loop-review.md](../reviews/2026-10-10-feedback-loop-review.md).

The question this plan answers is: *what is the minimum the library must know about a plant
and a controller to build a loop in one line, pick the signals worth plotting, frame the
animation, and score a policy?* The answer it argues for is: **the port names, read at the moment of use, and nothing else.** A handful of explicit hints remain for the things port names cannot say.

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
| `feedback_profile`, `PROFILE_PORTS`, role overrides | wiring, plot sweep, naming | wiring: **no** (ports suffice); plot sweep: yes, as `plot_space` |
| `Controller` / `DynamicController` | `.plot_control_law()` only; never `isinstance`-checked | authoring convenience only |
| `ErrorDriven` layouts | which ports a classical law exposes | yes, as a port-layout choice; not as a tool hint |
| `_composition_entry` / `_composition_output` | where the next `>>` attaches | **no**: shortcut state on every diagram |
| `camera_*`, `skin`, `scene_grid`, `camera_priority` | animation | yes, but read at animate time; `camera_priority` is unused |
| `solver_info` (time constant, discontinuity, sample period) | solver and `dt` | yes: physical, not structural |
| `is_random`, `params["seed"]`, ids | realizations | yes, and ids must be stable |
| Insertion order | tie-breaks (`>>` fallback, analysis `wrt`, hybrid ports) | **no**: the weakest hint and the cause of defect A |

## 3. Main recommendation: the minimal system

### 3.1 Five principles

1. **Manual first.** A controller is a `System`, a plant is a `System`, a loop is
   `add_subsystem` plus `connect`. This needs no base class, no declaration and no hint.
   Every shortcut builds exactly the diagram a student could write by hand (the same
   visible blocks), and stores nothing on `System` or `DiagramSystem`.
2. **Port names are the declaration.** Wiring, plot defaults, the camera owner, the
   evaluators and the design tools read one convention table (§3.2), which extends
   RULES 4.9. Dimensions decide the Mux (`[q; dq]`) and the Demux (component 0).
3. **Roles are resolved at use time, from ports and wiring, by one resolver** (§3.3). Every
   tool calls it, so a hand-wired diagram and a shortcut-built one behave the same by
   construction.
4. **State is `n`.** No class carries it. `Controller` and `DynamicController` stay as
   optional conveniences for authoring and for `.plot_control_law()`. Their names are frozen
   by the course notebooks, and no tool checks them. A plain `System` / `DynamicSystem`
   with the right ports is wired, scored and plotted identically.
5. **The few true exceptions stay explicit hints.**
   - `plot_space` (the control-law sweep);
   - `solver_info` (physical);
   - `is_random` and seeds;
   - camera, skin and `scene_grid` (visual);
   - one override attribute per role for nonstandard port names (`measurement_port`,
     `control_port`, `ref_port`; today only MPC's `u_ff` needs one).

### 3.2 The convention table

| Side | Port | Meaning |
|---|---|---|
| Plant input | `u` | the command: the one port a controller drives and a planner decides |
| Plant input | `w` | disturbance (exogenous, drawn or held at nominal; never decided) |
| Plant input | `v` | measurement noise (enters the measured output) |
| Plant input | other names | exogenous inputs, held at nominal unless wired or drawn |
| Plant output | `y` | measured output |
| Plant output | `x` | state |
| Plant output | `q`, `dq` | positions and velocities (the `[q; dq]` Mux) |
| Controller input | `e` | error: the loop inserts `Error`, `e = r − y` |
| Controller input | `y` | reads the plant's measured output |
| Controller input | `x` | reads the plant's state |
| Controller input | `r` | reference |
| Controller output | `u` | command |

A block whose ports do not follow the table either sets the one override attribute for the
role it renames, or is wired by hand.

### 3.3 One resolver, two questions

- **`block_roles(block)`: what a block reads and commands.** It answers from the port
  names and the three overrides:
  - the measurement (`e`, `y` or `x`);
  - the reference (`r` or none);
  - the command (`u`);
  - whether it is error-driven, from the port `e`;
  - whether it is stateful, from `n`.

  It replaces `feedback_ports`, `error_input`, `PROFILE_PORTS` lookups, the five fallback
  chains and `action_port_of`.
- **`loop_roles(diagram)`: who is who in any diagram, from the wiring.**
  - The plant is the block whose `u` is driven by another block's command.
  - The controller is that driver.
  - Each source is classified by what it drives: the reference, a disturbance or noise.
  - It works on hand-wired diagrams and on shortcuts alike, and is computed when a tool
    asks, never stored.

### 3.4 What each use case becomes

| Use case | After |
|---|---|
| Composition | One port-based dispatch: error junction, reads `y`, reads `x`, loop gain, or refuse with a message. `closed_loop(controller, plant, *, r, w, v, filter)`: each of `r`, `w`, `v` is `False`, `True` (a boundary port) or a source block. A plant's own `w` / `v` port wins over a loop Sum; `v` only when the measured output depends on it. Plant-side diagrams are inlined, so cascades and actuator chains build in one line. `_composition_entry` / `_composition_output` leave `DiagramSystem` if derivable. |
| Plot signals | Defaults from `loop_roles`: the reference, the command, the plant output, and `w` / `v` when present. Boundary names are plottable. Labels and units are copied onto every exposed port. Colours follow roles: command red, state blue. `compute_trajectory(show=True)` uses the same default. |
| Animation | The camera, follow frame and `scene_grid` come from the plant role, resolved at animate time, for any diagram, nested ones included. A compensator does not draw a skin inside a loop. The composition-time copy goes. `camera_priority` is read or removed. |
| Monte Carlo and plant + controller tools | The command is port `u` (else the single input) everywhere: `problem.U`, LQR, place, trajopt, value iteration, Gym's action space, `discretize`, and the `u` a cost sees. The disturbances are `w`, `v` and `problem.disturbances`, never decision variables. The measurement is what the controller's ports say. A time-varying law is honoured. The three Monte Carlo backends score the same loop. |
| Analysis | The default channel follows the same table: from `u` to `y`. `wrt="u"` means the port `u` when one exists. |
| Ids | One scheme across flow and hybrid: `ctl` and `plant` (or the block's role), `ref`, `w`, `v`. They are documented as the params keys and the random-stream names they are. |

### 3.5 Why this is the minimal system

- Every decision in §2 reads, or could read, a port name, a dimension, `n` or a
  `dependencies` entry. All of these already exist on every `System`.
- Nothing new is declared. Profiles shrink to a plot hint, and the composition-time state
  disappears.
- The `isinstance` checks on controller classes were never there.
- The cost is a single resolver, plus a convention table that the course material already
  follows (RULES 4.9).

## 4. Steps

Each step lands on its own commit with the checks of §6. The census baseline is built in
AC-1 and reused: about 40 loops across every path, plus the course controllers, recording
ids, connections, boundary ports with labels and units, entry and output, names, params
keys, `x0`, a fixed-step history, roles and control-law sweeps. It is captured twice and
`cmp`'d.

| Step | Scope | Lane | Rung |
|---|---|---|---|
| **AC-1** | Safe fixes and pinning tests: D1 (refuse), D2, D4, D6, A, B, C, D, the t = 0 scoring, `disturbances={"u"}` refused; a `TestDispatchTable`; the census baseline | agent (bug fixes) | v0.2 B4, now |
| **AC-2** | `block_roles` / `loop_roles` in `core/feedback.py`; every reader switched to them; byte-identical census | agent, after §5 decision 1 | v0.2 B4 |
| **AC-3** | Composition on ports: profiles become plot hints (course strings accepted, an unknown string raises); one vocabulary (`*_port`); `filter=` on `feedback` and `closed_loop`; plant-side diagram operands inlined; `_composition_*` off `DiagramSystem` if derivable | core: §5 decisions 1–2 | v0.2 B4, gates P4 |
| **AC-4** | Loop inputs: sources as values of `r` / `w` / `v`; plant-owned `w` / `v` win; the Sum built before wiring (ahead of S66); randomness A9 / D24 reconciled | §5 decision 3 | beside P4, before P11 |
| **AC-5** | Plot defaults, labels, units and colours from `loop_roles`; boundary names plottable; one default for `show=True` | agent (plotting lane) | v0.2 D |
| **AC-6** | Camera, follow frame and `scene_grid` from the plant role at animate time; compensators draw no skin in a loop | agent (plotting lane) | v0.2 D |
| **AC-7** | Tools on one command / disturbance / measurement convention: evaluation (one loop for the three backends), planning (`U` over `u`), design (LQR, place, trajopt, value iteration), Gym, `discretize`, analysis defaults, the cost's `u` | §5 decision 4 (changes multi-port plants) | v0.3, with RN-4 / RN-5 |
| **AC-8** | One id scheme across flow and hybrid; ids documented as stream names and params keys | §5 decision 5 (keys change); hybrid part with S31 | v0.9 |

**Done when:**

- `ctl @ plant`, a hand-wired loop of the same blocks, and `closed_loop` with sources give
  the same roles, plots, camera and Monte Carlo score;
- the census is byte-identical for every case that worked before;
- every defect of §2 has a test.

**Absorbed or reshaped workboard steps:**

- P10 is this plan.
- T6's composition half is AC-3 plus the textbook pass after it.
- TB-b's loop part is AC-3.
- S65's automatic-dt and hold-model rows are noted under AC-1 (C) and left to S65.
- S66 lands after AC-4.
- A4's "one closed-loop name" is AC-3; its default names are AC-8.
- P4's observer + state feedback becomes a plain `DynamicSystem` with ports `r`, `y` → `u`,
  wired by the port rule.
- P11's `Controller(feedback=…)` is likely unnecessary (§5 decision 6).
- RN-4 and RN-5 consume AC-4 and AC-7.
- S70 turns the refusals into `WiringError`.
- S31 takes the hybrid vocabulary, ids, `w` / `v` and `filter`.

## 5. Decisions for the maintainer

Each is recorded as open in ROADMAP §6.

1. **The five principles of §3.1 and the convention table of §3.2.** This includes
   extending RULES 4.9 with `e`, `x`, `q` and `dq`, and a DESIGN §4 "Conventions" section.
   *Recommended: yes.*
2. **`feedback_profile` leaves wiring and becomes a plot hint.** Every library controller
   wires from its ports. The course strings `state` and `output` stay accepted, and an
   unknown string raises. *Recommended: yes.*
3. **Loop inputs.** A plant's own `w` / `v` port wins over a loop Sum, with `v` taken only
   when the measured output depends on it. `r=`, `w=` and `v=` accept a source block, with
   fixed ids because they name random streams. *Recommended: yes.*
4. **The command is the port `u` for every tool.** This changes plants with several inputs
   (`*WithNoisePort`) in planners, LQR, place, value iteration and Gym: `w` and `v` stop
   being actuators. *Recommended: yes.*
5. **One id scheme across flow and hybrid.** This changes params keys and stream names
   once; the hybrid part waits for S31. *Recommended: yes, at v0.9.*
6. **P11's `Controller(feedback=…)`.** With ports as the declaration, a student writes a
   `System` with the ports of §3.2 and gets everything. *Recommended: drop the ask unless
   the P11 notebooks show a gap.*

## 6. Checks for every step

- `ruff check .` and `ruff format --check .`.
- The targeted tests: `test_feedback_composition`, `test_core`, `test_hybrid`,
  `test_simulation`, `test_mpc`, `test_graphics`, `test_mechanical_robotics` and the
  planning and evaluation tests touched.
- A census `cmp` after each sub-step: byte-identical for AC-2 and the refactor parts of
  AC-3; only cases that failed before, plus new cases, may change elsewhere.
- At the end of each step:
  - the full `pytest` suite, the regression gates and the flagship demos;
  - a notebook smoke run of `06_hybrid`, `sensitivity_functions`, `frequency_response`,
    `double_integrator_policy_evaluation`, `cartpole_static_controller` and
    `cartpole_dynamic_controller`;
  - `sphinx-build -W` for docstrings.

## 7. Not in scope

- No new operator, no options on `@`, no `Loop` block.
- No new base classes, traits or intent declarations.
- No tuple operands.
- No hybrid features before S31.
- No flip of `ProportionalController`'s default layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
