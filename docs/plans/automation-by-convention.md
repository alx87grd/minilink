# Automation by convention: plants, controllers, loops, plots and tools

**Status:** decided on 2026-10-11 and ready to build; steps in §7. Workboard row P10 (v0.2
wave B4) for the core steps; the later steps are placed in §7. How each rule was reached
(the analysis of the old code, the evidence, the options rejected, the fifteen rulings) is in
the decision record,
[2026-10-11-automation-by-convention.md](../reviews/2026-10-11-automation-by-convention.md).

## 1. Goal

One line, `controller @ plant`, should build a loop, and every tool should know what to plot,
where to point the camera and what to score. The library learns all of this from **standard
names, read at the moment of use, and nothing else**:
- **port names** say what a block reads and commands, and drive the automatic wiring;
- **three reserved subsystem names**, `plant`, `controller` and `estimator`, say what role a
  block plays, and every tool reads them.

**Manual first.** A controller is a `System`, a plant is a `System`, and a loop is
`add_subsystem` plus `connect`. That path needs no base class, no declaration and no hint.
Every shortcut builds exactly the diagram a student could write by hand, and stores nothing
else. A block with custom port names is wired by hand; if it uses the role names, it still
gets the plots, the camera and the cost.

Roles are never inferred from the wiring and never guessed: a diagram with no `plant` has no
plant, and a tool that needs one says so. Simulation, linearization, `bode`, `plot_diagram`
and `trajectory_of` work on any `System`, whatever its names.

A few explicit hints stay, for what names cannot say:
- `plot_space`, the control-law sweep;
- `solver_info`, which is physical;
- `is_random` and the seeds;
- the camera, skin and `scene_grid`, which are visual.

## 2. Conventions

### 2.1 Ports: what a block reads and gives

| Block | Reads | Gives |
|---|---|---|
| Plant | `u` (the command), `d` (disturbance), `n` (measurement noise); other inputs are held at nominal | `y` (the measured output), `x` (the state) |
| Controller | `e` (an Error block computes `e = r − y`), or `y`, or `x`; plus `r` | `u` |
| Estimator | `u` and `y` | `x`, the estimate (labelled x̂) |

- **The letters.** `d` and `n` are the classical loop's letters, those of the sensitivity
  functions. `v` stays free for velocity, as the plant equations write it (`C(q, v)`), and
  `w` for angular speed and frequency.
- **A mechanical plant** gives `y = [q; dq]` and `x`. It has no separate `q` and `dq`
  ports; a positions-only sensor is a custom port or a `Demux`.
- **`x` only where needed.**
  - Plants expose `x`: the catalog, and `StateSpace` and `TransferFunction` in the plant
    layout.
  - A user plant opts in with `expose_state=True`, and a box with `expose("x")` (§2.4).
  - Controllers, compensators, filters, sources and routing blocks do not expose `x`.
- **Other names are allowed** on any block, and the automation leaves them alone: the
  controller extras `x_ff` and `z`, the manipulator outputs `p` and `pdot`, the input
  `r_null`.
- **Ports outside, positions inside.**
  - A port name is the block's interface: wiring, `autowire`, plot signals, drawings,
    error messages.
  - The equations see one stacked `u = [port_1; port_2; ...]` in port order, as in the
    textbook `f(x, u)`, and library code reads it by position.
  - Students and user blocks may read by name with `get_port_values_from_u(u, "y")`.
  - A port has one name; there are no aliases.

### 2.2 Role names: what role a block plays

**Blocks get words, signals get symbols.** Ports are mathematical variables (`u`, `r`, `d`,
`n`, `y`, `x`, `e`); subsystem names are things in the diagram, so they are nouns.

| Name | Role | Written by |
|---|---|---|
| `plant` | the system under control | `@`, `closed_loop`, `%`, the evaluators, or by hand |
| `controller` | the law that commands the plant | the same |
| `estimator` | the state estimate the controller reads (`u`, `y` → `x`) | `closed_loop(estimator=)`, or inside a controller box |

- A role is a reserved key of `diagram.subsystems`, the name given to `add_subsystem`. The
  same key names everything else about the block:
  - its state slice;
  - its params (`params["controller"]["Kp"]`);
  - its signals (`"plant:y"`);
  - its random stream;
  - its box in `plot_diagram`.
- There is one of each per diagram level, and a clash raises; a name is never suffixed. A
  cascade nests (`plant/plant`). A second physical plant gets a name of its own and is
  handled by hand. Several controllers on one plant (a path-tracking cascade) are wired by
  hand, and at most one of them holds the name `controller`.
- Variable names stay free: `ctl @ pendulum` gives the keys `controller` and `plant`.
- In a shortcut, a role name wins over `System.id`.
- **Other names** are names, not roles, but they matter, because they key the params and
  the random streams:
  - a source takes the word for the input it drives: `reference`, `disturbance`, `noise`,
    or the port's own name for a custom port (`wind`);
  - the helpers the loop adds are `error` and `filter`.

### 2.3 What sits on the way back

| On the way back | What it is | Where it goes |
|---|---|---|
| a sensor's lag, gain or quantization | physics | inside the plant box: the plant is what you measure through |
| a low-pass or notch filter on `y` | an algorithm that gives a cleaned `y` | the helper `filter` (§4.3, §4.4), or inside a controller box |
| a Luenberger observer, a Kalman filter | an algorithm that gives the state estimate x̂ | the role `estimator` |

A filter is not an estimator. A physical sensor filed as an estimator would leave the plant
and its scoring. Also, library filters read `u` and give their own state as `x`, so the port
names would wire them wrongly.

### 2.4 Role boxes: one block or many

A plant, a controller or an estimator is a **box** with the ports of §2.1 on its edge. `@`
and every tool read only the edge; the inside is one block or many.
- On a box's edge, `x` is the box's own state: the stack of its blocks' states.
- Names stay unique at each level, and the params keys and random streams follow the
  nesting.
- **Building a box, simplest first:**
  1. by hand: `DiagramSystem`, `add_subsystem`, `connect`, `add_input_port`,
     `connect_new_output_port`;
  2. a serial chain with `>>` already has a plant's edge (`u` in, `y` out):
     `ctl @ (Saturation(-1, 1) >> arm)`;
  3. `diagram.expose(*names)`, a small new helper:
     - a name left open on inside inputs becomes one edge input that feeds all of them;
     - the name of one inside output becomes an edge output;
     - `"x"` gives the box's own state;
     - an unknown or ambiguous name is refused.

     ```python
     controller = (observer + gain).autowire().expose("r", "y", "u")
     loop = controller @ plant
     ```
- **Typical boxes:**
  - a closed loop used as a plant: it holds its own `controller` and `plant`;
  - a cascade, `outer @ ((inner @ motor) >> load)`: the inner loop's command input is `r`
    (§3);
  - an observer with a state-feedback gain: a controller box holding `estimator` and
    `gain`;
  - actuator limits, which belong in the plant box (`Saturation >> arm`);
  - a sampled loop's computer and plant (§4.5).

## 3. The resolver

Three small functions answer every question the tools ask.

- **`roles(diagram)`: who is who.**
  - Three lookups of the reserved keys at the top level. The estimator may also sit inside
    the controller (`controller/estimator`).
  - A leaf is its own plant.
  - A sampled loop answers from its `.plant` and `.computer` fields.
  - There is no "deepest plant" rule: in a cascade the outer loop's plant is the inner loop.
  - With no `plant` key the plant is `None`. A tool that needs it raises; plots and the
    camera keep their plain defaults.
- **`block_roles(block)`: what a block reads and commands**, from its port names only:
  - the measurement (`e`, `y` or `x`), the reference (`r` or none), the command (`u`);
  - whether it is error-driven: it reads no `y` and no `x`, so the error drives its entry
    (`e`, or a plain compensator's one input);
  - whether it has state (from `n`).
- **`command_port(sys)`: which input a decision drives.** `u`, else the single input that is
  not `d` or `n`, else refuse. Gym, the cost, the design tools and a cascade's outer `@` all
  use it.

## 4. The operators

**No operator modifies its operands.** `+`, `>>` and `@` return a new diagram that shares
the operands' blocks, so long chains stay cheap, and the operands keep their blocks, wiring,
ports, name, `x0` and camera settings.
- `+` and `>>` give a **flat** diagram: the operands' blocks side by side, with their names.
  The left operand's `x0`, camera settings and name are carried into the result.
- Only `@` nests, because only `@` assigns roles (§2.4).
- `connect()`, `autowire()` and `expose()` are methods and change the diagram they are
  called on.

### 4.1 `+`, `autowire` and naming ports

- **`+`** puts blocks side by side in a new diagram, with no wiring.
- **`autowire()` connects by name only.**
  - Each unconnected input takes the one output with the same name and size.
  - An output may feed several inputs.
  - No match, or several, leaves the input open and listed in a warning; `strict=True`
    raises.
- **`rename_ports(**names)`** on every `System`, chained at creation:
  `ProportionalController(2.0).rename_ports(u="u_cmd")`.
  - It keeps the port order, so the equations are unchanged, and it updates the output
    dependency lists.
  - It is for leaf blocks; a diagram names its edge with `expose`.
  - A renamed block opts out of the `@` automation by design.
- **Sources name their signal at creation:** `Step(final_value=1.0, port="r")`,
  `Sine(..., port="n")`, `WhiteNoise(port="d")`, `TrajectorySource(..., port="u")`. The
  default stays `y`.
- **No silent mistake.**
  - A stale name raises a clear message that names the block, its inputs and the fix.
  - `rename_ports` refuses a new name that is already a port of the block, so a swap
    cannot flip a sign silently.
  - Renaming a block already placed in a diagram fails loudly; rename at creation.

```python
ref = Step(final_value=1.0, port="r")
ctl = ProportionalController(2.0).rename_ports(u="u_cmd")
sat = Saturation(-1, 1).rename_ports(u="u_cmd", y="u")
plant = Integrator()
loop = (ref + ctl + sat + plant).autowire()   # every connection read from the names
```

### 4.2 `>>`: a plain serial connection

`A >> B` wires one output of A to one input of B. The first case that applies wins:
1. A has one output and B has one input: connect them, whatever their names.
2. Exactly one output of A and one input of B share a name (`y`→`y`, `x`→`x`, `u`→`u`):
   connect those.
3. A has an output `y` and B has an input `u`: connect `y` → `u`.
4. Otherwise refuse: "`>>` connects one output to one input; wire this one with connect".

- Sizes are a check, never a choice; a mismatch is refused.
- A diagram operand counts its edge: its outputs and its free inputs.
- The result keeps A's free inputs as its inputs, and B's output (`y`, else its only one)
  as its output. So `C >> G` still exposes `e`.
- `>>` has no preference for `r` and no hidden memory of an entry or an output.
- A loop built with a `d` input has two free inputs, so `Step() >> closed_loop(..., d=True)`
  is refused; the message points to `closed_loop(..., r=Step(), d=True)`.

### 4.3 `@`: controller at plant

`@` always means controller at plant: a System, or a box, on each side.

- **What it builds**, from what the controller reads:

  | The controller reads | `@` wires |
  |---|---|
  | `y` (and `r`) | plant `y` → controller `y`; the controller computes `r − y` itself |
  | `x` (and `r`) | plant `x` → controller `x` |
  | only its error (`e`, or a plain compensator's one input) | an `Error` block `e = r − y` in front of the controller |

  `PID() @ plant`, `Lead() @ plant` and a plain `TransferFunction` compensator take the
  third row.
- **Names.** The left side is `controller`, the right side `plant`. A box is nested under
  its role name, one level deep; a single block stays a single block.
- **Refusals**, each with a message that names the fix:
  - a number on the right: "`@` needs a plant on the right; to close an open loop on
    itself use `feedback(L)` or `feedback(L, K)`";
  - a box without the standard edge: build it by hand or with `expose`;
  - a controller that reads `x` on a plant without `x`: build the plant with
    `expose_state=True`.
- **`closed_loop(controller, plant, *, r, d, n, filter, estimator)`** is `@` with options.
  Each loop input `r`, `d`, `n` is `False`, `True` (an edge port) or a source block. It has
  no port-name keywords.
- **The filter** (`filter=F`) is a helper block at the loop's top level, beside `error`; it
  is in neither the plant nor the controller.

  ```
  r ──► error(+) ──► controller ──► plant ──┬──► y   (the true output, the loop's output)
          ▲(−)                              │
          └─────────── filter ◄─────────────┘   (plant y + n when noise is on)
  ```

  - Plant `y` goes into the filter's one input. The filter's `y` goes where the
    controller's measurement goes: the Error's minus input, or the controller's `y`.
  - The keyword says what the block is, so a filter whose input is named `u` is wired
    right.
  - A controller that reads `x` with `filter=` is refused: "a filter on the state is an
    estimator; use `estimator=`".
  - `plant:y` stays the true output; the filtered signal is `filter:y`. The noise `n`
    enters before the filter.
- **The estimator** (`estimator=E`): plant `y` and the controller's `u` go into `E`, and
  `E`'s `x` goes to the controller's `x`.
- **Disturbance and noise.**
  - A plant that declares `d` and `n` owns them: `d` is never charged as control effort,
    and `n` enters what the controller reads.
  - For a plant without those ports, `d=` and `n=` add summing junctions in the loop, as
    today. The cost then sees `u + d` as the plant's command, a known limit.
  - An automatic wrapper that would give such a plant its own `d` and `n` is deferred
    until someone needs it.

### 4.4 `feedback`: close an open loop on itself

`feedback(L, filter=1, *, of=None, sign=-1)` closes an open loop `L` that already holds the
whole forward path: `r → Error → L → y`, with `y` coming back through `filter`. This is the
MATLAB and python-control notation, and the textbook loop `T = L / (1 + L H)`.

- **The way back:** `1` (a wire), a number or matrix (a gain), or a System. The block's
  name is `filter`; positional calls (`feedback(C >> H, F)`) read the same.
- **The entry** is `L`'s command input, by the `command_port` rule (§3). A diagram counts
  its free inputs.
  - `feedback(PID() >> H)` closes on `e`.
  - `feedback(P >> H)` has two free inputs, `r` and `y`, and is refused; the message
    points to `P @ H`.
- **What comes back:** `of=` if given, else `y`, else `L`'s only output, else refuse.
  `of=(port, i)` picks one component, and a scalar entry on a vector output takes
  component 0 through a visible `Demux`.
- **`sign=+1`** gives positive feedback with a `Sum`.
- **Names:** a single-block `L` is `plant`. A diagram `L` stays flat with its own names,
  so `feedback(C >> G)` is plain topology. The form that names the roles is `C @ G`.
- The loop's only input is `r`.
- One junction helper builds the Error, the filter and the Demux for both `feedback` and
  `@`, so `C @ G` is `feedback(C >> G)` with role names.

### 4.5 `%`: sampled loops

`block % dt` runs a block on a computer clock (a `Computer`). `computer @ plant` closes a
sampled loop: the computer reads the plant through a sampler and drives it through a
zero-order hold.

- **Scope.** The sampled loop is a side helper for MPC simulation today, not a mainstream
  core component. What exists keeps in step with the core rules (step AC-8a), so that
  making it mainstream later (AC-8b) is easy. No new sampled-loop feature is built before
  then.
- **What `%` accepts:** anything with no continuous state, such as a static law (P, state
  feedback, sliding mode), a discrete block, or a discrete diagram.
  - A controller with continuous states (PID, Lead, a transfer function) raises
    `NotImplementedError`: "sampling a controller with continuous states is future work;
    use a static or discrete controller".
  - MPC keeps its own `%`, which checks `dt`.
- **`computer @ plant` follows §4.3:**
  - the computer reads `y` or `x`, and the sampler reads that port of the plant;
  - otherwise an `Error` block sits **inside the computer**, on the sampled measurement.
- **Boxes on both sides.** The plant may be a box with edge `u` → `y`; the computer may be
  a box (filter and controller, at several rates) with edge `r`, `y` (or `x`) → `u`.
- **Where things sit.**
  - The error and the filter run on the computer.
  - The sampler and the hold are the boundary between the two boxes, not blocks.
  - The roles are the fields `.plant` and `.computer`.
  - Inside the computer the names are `controller`, `error` and `filter`, and the
    multi-rate schedule uses them.
- **Custom names** use the manual path, `HybridDiagram(...)` with `connect_boundary(...)`.
- **Later, when sampled loops become mainstream (AC-8b):**
  - `Step() >> ctl % dt @ plant`, with the source on the continuous side and sampled by
    the computer;
  - `(P >> Sat) % dt`, a flow diagram with no continuous state;
  - the sampled loop as a `System` for plots, cost and Monte Carlo.

  Until then, the reference is passed with `compute_forced(r, input_port_id="r")`.
  Sampled controllers with continuous states are future work (TODO, beside P6).

## 5. What each tool does

| Tool | Rule |
|---|---|
| Plots | `signals="plant"`, `"controller"` or `"estimator"` selects every port of that block, as `"plant:y"` selects one. The default *auto* mode shows every port of the plant and the controller, each wire once; the reference, the error, `d` and `n` come in through their inputs. Signal names take paths (`plant/plant:y`), and a block inside a box is found by object too. Labels and units are copied onto edge ports, and colours follow roles. With no roles, plots keep their plain default; a leaf keeps `("x", "u")`. `compute_trajectory(show=True)` and the planning and comparison plots use the same default. |
| Animation | Resolved when animating. The camera comes from the one drawable block anywhere in the tree, else the drawable block inside `plant`, else an auto-fit view. A compensator draws no skin inside a loop. `scene_grid` and the follow frame come from the same block. |
| Cost and evaluation | The cost is scored on the plant. A tool given a plant and a policy builds `policy @ plant` itself, compiled once for the three Monte Carlo backends, and scores `trajectory_of(plant)`: the plant's state and its `command_port`. Controller and estimator states are integrated, never scored. A controller with state is integrated, not refused; time is honoured; the cost's params are passed; leaving `X` has one price. The `x0` draw is on the plant only. |
| Design and planning tools | DP, LQR, `place`, trajectory optimization, MPC and value iteration decide `command_port` only. `d` and `n` are never decision variables: they are drawn or held at nominal. `discretize` keeps the plant's ports. |
| RL and Gym | The observation is what the controller reads (`x` today), and the observation space matches it. The action is `command_port`, and the reward is `−g` on the plant. A policy with state is integrated by the loop. |
| Analysis | The default input is `r`, else the command port; the default output is `y`. `wrt="u"` means the port `u` when one exists. |
| `plot_control_law()` | A method of every `System`, beside `plot_input_output_map()`. It reads the measurement and the reference from `block_roles`; the sweep comes from the ports, or from `plot_space` where it differs. A block with no measurement input refuses and names `plot_input_output_map()`. On a diagram it plots the law of its `controller`. Every existing call keeps working, and the `PlanningSolution`, `Planner` and `Comparison` methods of the same name keep delegating to their policy. |

**There are no controller classes.** A controller is a `System`, or a `DynamicSystem` when it
has state. `Controller` and `DynamicController` are deleted, with no aliases. `ErrorDriven`
stays: it is a port layout, not a role marker.

## 6. What goes, and the migrations

**Removed:**

| What | Replaced by |
|---|---|
| `feedback_profile` and `PROFILE_PORTS` (13 places) | the port names |
| The override attributes `measurement_port`, `ref_port`, `control_port` (6 classes) | the port names; MPC is renamed to read `x` and give `u` (today `y` → `u_ff`) |
| `closed_loop`'s port keywords `ref_port`, `measurement_port`, `control_port`, `plant_input_port`, `plant_output_port`, `output_port`, and `feedback=` | standard names, or wiring by hand |
| `hybrid_closed_loop`'s `computer_out`, `computer_in`, `plant_in`, `plant_out`, `ref_port`, `output_port`, `extra_boundaries` | standard names, or `connect_boundary` |
| The sampled loop's port guesses (`u_nom`, then `u`, then `u_ff`; the first port) | the standard edge; the dual-rate MPC composite exposes `u` |
| Names minted from a block's shape, and their silent suffixes (`plant2`) | the role names, and plain names elsewhere |
| The composition-time memory (`_composition_entry`, `_composition_output`) and the camera copy | the plain `>>` rule, and the camera chosen when animating |
| `error_input`, `refuse_two_port_entry` | `block_roles` and the `feedback` entry rule |
| `camera_priority` (never read) | nothing |
| A mechanical plant's `q` and `dq` ports, `StateSliceOutputPort`, `closed_loop_qdq`, the `qdq` Mux branch (about 140 lines) | `y = [q; dq]`, read straight |
| `Controller`, `DynamicController` | `System`, `DynamicSystem` |
| `L @ number` | `feedback(L)`, `feedback(L, K)` |
| `feedback(..., through=)` | `feedback(..., filter=)` |
| The `x` port of `LowPassFilter` and `NotchFilter` (check its users first) | nothing: filters are not plants |

**Kept:** `closed_loop`'s `r=`, `d=`, `n=`, `filter=`, `estimator=` and `validate=`; the
sampled loop's `schedule=` and `computer=`; `feedback(L, of=)`; `port=` on sources and
`Demux`; analysis keywords such as `bode(of=, wrt=)`.

**Renamed once, with no aliases** (the name freeze is paused between terms, and each rename
migrates every course notebook in the same commit):
- the subsystem names `ctl` → `controller`, `sys` → `plant`, `ref` → `reference`; sources
  `disturbance` and `noise`;
- the ports `w` → `d` and `v` → `n` on the two noisy plants, and
  `closed_loop(w=, v=)` → `closed_loop(d=, n=)`;
- the estimator's output `x_hat` → `x` (P4 builds it so).

**Migrations:**

| Where | Change |
|---|---|
| The four course notebooks with controllers | the import line, the base class, and the `feedback_profile` line; the GRO860 sentence becomes "the input port is named `x`, so the tools know the block reads the full state". Only the lines a removed name touches change. |
| 8 library lines that read their own inputs by name (the noisy pendulum and cart-pole, the two bicycles, the racecar, the arm's tool force) | read `u` by position, with identical results |
| `00_core`, two demos and their tests (`(Step() + ctl + plant).autowire()`) | `Step() >> ctl @ plant`, or `Step(port="r")` |
| The three `closed_loop_qdq` demos, the showcase UR5 cell, the tests | `ctl @ plant` (the same loop, verified) |
| The anti-windup demo's second loop, 5 tests, the frequency-response notebook table | `L @ number` → `feedback(L, ...)`, or `pid @ (sat >> plant)` to keep the roles |
| The sampled-loop demos, the GRO501 `racecar_toward_mpc` notebook, the MPC project | drop the port keywords: `mpc % dt @ inner` once MPC gives `u`; custom names by hand |
| The manipulator's inverse-kinematics guess | reads the state instead of the `q` port |
| DESIGN (the feedback profiles, the three `@` paths, operator mutation, the ids, the controller classes, `w`/`v`) and RULES 4.9 | rewritten as one DESIGN "Conventions" section when each step lands |

## 7. Steps, in build order

Each step lands in its own commits, with the checks of §8. A census of about 40 loops (ids,
connections, edge ports with labels and units, params keys, `x0`, a fixed-step history,
roles and control-law sweeps) is captured before and after each step and compared.

| Step | What | When |
|---|---|---|
| AC-1 | Safe bug fixes and pinning tests. **Landed 2026-10-10.** | — |
| AC-0 | No controller classes; `plot_control_law` on `System`; the four course notebooks migrated (the import line and the base class; their profile line goes in AC-3) | v0.2 B4 |
| AC-2 | The resolver (§3), with every reader switched; reserved names raise on a clash. (a) Byte-identical census. (b) The one-time rename of the subsystem names, in flow and sampled loops; shortcuts stop naming blocks from their shape. (c) The ports `d` and `n` in the same pass. | v0.2 B4, before the randomness steps that name streams (RN-4) |
| AC-OP | The operators (§4.1–§4.4), one commit each: no operator modifies its operands; the plain `>>`; `+`, `autowire`, `rename_ports`, `port=` and the 8 positional lines; `expose`; `@` needs a plant, `feedback`'s entry, return and `filter=`, one junction helper | v0.2 B4 |
| AC-3 | Composition on standard names: the removals of §6; boxes nested by `@`, with the edge check; `x` only where needed and its refusal; the mechanical plant's `y` and `x`; MPC renamed; `filter=` and `estimator=` | v0.2 B4; unblocks P4 |
| AC-4 | Loop inputs: plants that declare `d` and `n` own them; sources as values, named `reference`, `disturbance`, `noise`; `estimator=` with P4; the randomness plan's loop-input rulings reconciled | beside P4, before P11 |
| AC-5 | Plots (§5): the auto mode, role selectors, paths into boxes, labels, units and colours by role, one default | plotting lane, v0.2 wave D |
| AC-6 | Animation (§5): the camera chosen when animating, looking inside `plant`; no compensator skin in a loop; the camera copy goes | plotting lane |
| AC-7 | Cost and tools (§5): one compiled loop for the three Monte Carlo backends, `command_port`, controllers with state, time, cost params, one price; Gym and the rollout environment aligned | v0.3, with RN-4 and RN-5 |
| AC-8a | The sampled loop keeps in step (§4.5): each item with the core step it mirrors (names with AC-2, the dispatch and boxes with AC-OP and AC-3, the MPC rename with AC-3) | alongside |
| AC-8b | The sampled loop goes mainstream (§4.5, "later") | v0.9, with S31 |

AC-2 comes before AC-OP because `feedback`'s entry rule uses `command_port`.

**Done when:**
- `controller @ plant`, the same loop wired by hand with the role names, and `closed_loop`
  with sources give the same roles, plots, camera and Monte Carlo score;
- the census is byte-identical for every case that worked before, apart from the renames;
- every defect listed in the decision record has a test.

## 8. Checks for every step

- `ruff check .` and `ruff format --check .`.
- The targeted tests: `test_feedback_composition`, `test_core`, `test_hybrid`,
  `test_simulation`, `test_mpc`, `test_graphics`, `test_mechanical_robotics`, and the
  planning and evaluation tests touched.
- A census comparison after each sub-step. It is byte-identical for AC-2a and the refactor
  parts of AC-3. Elsewhere only the renames, cases that failed before, and new cases may
  change.
- At the end of each step:
  - the full `pytest` suite, the regression gates and the flagship demos;
  - a smoke run of the notebooks `06_hybrid`, `sensitivity_functions`,
    `frequency_response`, `double_integrator_policy_evaluation`,
    `cartpole_static_controller`, `cartpole_dynamic_controller`, `ode_simulation` and
    `racecar_toward_mpc`;
  - `sphinx-build -W` for the docstrings.

## 9. Not in scope

- No new operator, no options on `@`, no `Loop` block, no tuple operands.
- No automatic wiring in `>>`.
- No new base classes, traits, intent declarations or override attributes.
- No inference of roles from the wiring, and no "deepest plant" rule.
- No new sampled-loop features before S31.
- No change to `ProportionalController`'s default port layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
- No renaming of the named-port plants (racecar, bicycle): they are wired by hand.

## 10. Details settled while building

These do not change the rules:
- **AC-OP:** whether `expose` runs before or after `autowire`, and how the open-input
  warning treats edge inputs.
- **AC-3:** who uses the filters' `x` port, before it goes.
- **AC-5:** how the auto mode removes duplicate wires, and whether a plant with many outputs
  (the racecar) needs a name filter.
- **The deferred wrapper**, if it is ever built: the names inside it, and how its edge shows
  the plant's `x`. Until then, the cost limit of §4.3 stands.
