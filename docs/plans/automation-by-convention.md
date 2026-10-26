# Automation by convention: plants, controllers, loops, plots and tools

**Status:** decided on 2026-10-11, amended on 2026-10-13. The conventions, the resolver and the
operators landed between 2026-10-10 and 2026-10-14 (steps AC-0 to AC-4): the standard port
names, the three reserved role names `plant`, `controller` and `observer`, `roles`,
`block_roles` and `command_port`, and `+`, `autowire`, `>>`, `@`, `closed_loop` and
`feedback`. Their contract is DESIGN §4, "Control laws: the port names are the declaration",
and the `DiagramSystem` section beside it; the cost scored on the plant (the deterministic
half of AC-7) landed with the problem evaluator in v0.3 (DESIGN §6). This doc holds what is
still open: the plots (AC-5) and the animation (AC-6), in the plotting lane; the draws of
AC-7, with V1's step 7 and the randomness steps RN-4 and RN-5 (v0.4); the sampled loop kept
in step with the core rules (AC-8a), alongside; and the sampled loop made mainstream (AC-8b),
with S31 in v0.4. How each rule was reached (the analysis of the old code, the evidence, the
options rejected, the fifteen rulings) is in the decision record,
[2026-10-11-automation-by-convention.md](../reviews/2026-10-11-automation-by-convention.md);
the second opinion and its eight rulings (R1–R8) are in
[2026-10-13-automation-by-convention-second-opinion.md](../reviews/2026-10-13-automation-by-convention-second-opinion.md).
**Rung, 2026-10-26:** AC-7's draws and AC-8a / AC-8b are v0.3's core (the maintainer's direction:
[core-architecture.md](core-architecture.md) §5 and §6; [TODO.md](TODO.md) §1.4, §1.5); AC-5 and AC-6 stay in the
plotting lane (TODO §3). The rungs written below are the ones before that direction.
Sections 1 to 4.4 described the landed part and were cut (the history is in git); the
section numbers of what is left are kept, so the citations to them hold.

## The conventions, in short

Everything is a `System` with named ports, a diagram is `System`s wired by name, and three
reserved subsystem names tell every tool which block is the plant, the controller and the
estimator. A plant reads `u`, `d`, `n` and gives `y`, `x`; a controller reads `e`, `y` or
`x`, plus `r`, and gives `u`; an estimator reads `u`, `y` and gives `x`. Roles are never
inferred from the wiring: a diagram with no `plant` has no plant, and a tool that needs one
says so. Every shortcut (`+` with `autowire`, `>>`, `@`, `closed_loop`, `feedback`) builds
exactly the diagram a student could write with `add_subsystem` and `connect`. DESIGN §4 states
the rules; the open steps below extend them to the plots, the camera, the draws and the
sampled loop.

## 4. The operators

`+`, `autowire`, `>>`, `@`, `closed_loop` and `feedback` (§4.1 to §4.4 of the plan as
decided) landed; DESIGN §4 states them. The sampled loop is what is left.

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
- **`computer @ plant` follows the `@` rule** (DESIGN §4, "Control laws"):
  - the computer reads `y` or `x`, and the sampler reads that port of the plant;
  - otherwise an `Error` block sits **inside the computer**, on the sampled measurement.
- **Boxes on both sides.** The plant may be a box with edge `u` → `y`, and its state is
  read for an `x`-reading computer (MPC), as `@` does; the computer may be a box (filter and
  controller, at several rates) with edge `r`, `y` (or `x`) → `u`.
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
  Sampled controllers with continuous states are future work, with no row yet (P6, the
  discretization step that would have held them, was dropped on 2026-10-17).

## 5. What each tool does

The rows for the design and planning tools, RL and Gym, analysis and `plot_control_law()`
landed with AC-4 and AC-7's deterministic half (DESIGN §4 and §6); the three rows below are
the open ones. Of the cost row, the plant's run scored, controllers with state, time, the
cost's params and one price landed in v0.3 with the problem evaluator; what is left is the
draws (the start drawn on the plant only, and every Monte Carlo trial scored on the same
compiled loop).

| Tool | Rule |
|---|---|
| Plots | `signals="plant"`, `"controller"` or `"observer"` selects every port of that block, as `"plant:y"` selects one. The default *auto* mode shows every port of the plant and the controller, each wire once; the reference, the error, `d` and `n` come in through their inputs. Signal names take paths (`plant/plant:y`), and a block inside a box is found by object too. Labels and units are copied onto edge ports, and colours follow roles. With no roles, plots keep their plain default; a leaf keeps `("x", "u")`. `compute_trajectory(show=True)` and the planning and comparison plots use the same default. |
| Animation | Resolved when animating. The camera comes from the one drawable block anywhere in the tree, else the drawable block inside `plant`, else an auto-fit view. A compensator draws no skin inside a loop. `scene_grid` and the follow frame come from the same block. |
| Cost and evaluation | The cost is scored on the plant. A tool given a plant and a policy builds the loop itself (`closed_loop_of(problem, policy)`, compiled once by `problem.compile(policy, dt)`, the one engine of `evaluate` and `compare` since the rulings of 2026-10-22/23, R15 and R22), and scores the plant's run `traj.of("plant")` (today's `trajectory_of(plant)`): the plant's state and its `command_port`. The problem never fills the loop's reference (R38): every free input, `r` included, stays at its port's nominal value; the setpoint is the controller side's (`controller.inputs["r"].set_nominal_value(...)`, or `closed_loop(..., reference=Step(...))`), and the free inputs stay leaves of the compiled problem's `params`. Controller and estimator states are integrated, never scored. A controller with state is integrated, not refused; time is honoured; the cost's params are passed; leaving `X` has one price. The `x0` draw is on the plant only. |

## 6. What goes, and the migrations

The removals tagged AC-0, AC-2, AC-OP and AC-3, the renames and their migrations landed
(2026-10-13); the decision record lists them. What is left:

**Removed:**

| What | Replaced by | Step |
|---|---|---|
| MPC's output `u_ff` (the override attributes went with AC-3; MPC keeps `y`, the state of its model) | `u` (§10) | AC-8a |
| `hybrid_closed_loop`'s `computer_out`, `computer_in`, `plant_in`, `plant_out`, `ref_port`, `output_port`, `extra_boundaries` | standard names, or `connect_boundary` | AC-8a |
| The sampled loop's port guesses (`u_nom`, then `u`, then `u_ff`; the first port) | the standard edge; the dual-rate MPC composite exposes `u` | AC-8a |
| The camera copied at composition time, and `camera_priority` (never read) | the camera chosen when animating | AC-6 |

**Kept:** the sampled loop's `schedule=` and `computer=`.

**Migrations:**

| Where | Change |
|---|---|
| The sampled-loop demos, the GRO501 `racecar_toward_mpc` notebook, the MPC project | drop the port keywords: `mpc % dt @ inner` once MPC gives `u`; custom names by hand |

## 7. Steps, in build order

Each step lands in its own commits, with the checks of §8. A census of about 40 loops (ids,
connections, edge ports with labels and units, params keys, `x0`, a fixed-step history,
roles and control-law sweeps) is captured before and after each step and compared.

| Step | What | When |
|---|---|---|
| AC-1, AC-0, AC-2, AC-OP, AC-3, AC-4 | Landed 2026-10-10 to 2026-10-14. | — |
| AC-5 | Plots (§5): the auto mode, role selectors, paths into boxes, labels, units and colours by role, one default | plotting lane |
| AC-6 | Animation (§5): the camera chosen when animating, looking inside `plant`; no compensator skin in a loop; the camera copy goes | plotting lane |
| AC-7 | Cost and tools (§5): one compiled loop (the compiled problem) for every evaluation, `command_port`, controllers with state, time, cost params, one price; Gym and the rollout environment aligned | the deterministic part landed with V1's steps 2–4 (v0.3: the plant scored, controllers with state, the price switch of R32); the draws with V1's step 7, RN-4's per-block noise seeds and the rest of RN-5, v0.4 |
| AC-8a | The sampled loop keeps in step (§4.5): `hybrid_closed_loop`'s keywords and the port guesses go, MPC gives `u` | alongside |
| AC-8b | The sampled loop goes mainstream (§4.5, "later") | v0.4, with S31 (R86) |

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
- A census comparison after each sub-step. Only the renames, cases that failed before, and
  new cases may change.
- At the end of each step:
  - the full `pytest` suite, the regression gates and the flagship demos;
  - a smoke run of the notebooks `06_hybrid`, `sensitivity_functions`,
    `frequency_response`, `double_integrator_policy_evaluation`,
    `cartpole_static_controller`, `cartpole_dynamic_controller`, `ode_simulation` and
    `racecar_toward_mpc`;
  - `sphinx-build -W` for the docstrings.

## 9. Not in scope

- No new operator, no options on `@`, no `Loop` block, no tuple operands.
- No automatic wiring in `>>`, and no second guard in it: the source-into-`y` refusal
  (DESIGN §4) is the only special case.
- No new base classes, traits, intent declarations or override attributes.
- No inference of roles from the wiring, and no "deepest plant" rule.
- No new sampled-loop features before S31.
- No change to `ProportionalController`'s default port layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
- No renaming of the named-port plants (racecar, bicycle): they are wired by hand.

## 10. Details settled while building

These do not change the rules:
- **The MPC rename (AC-8a), found while building AC-3:** the MPC reads the state of *its
  model*, which for a higher-fidelity plant box is the box's `y`, not its state (the
  racecar's `inner` box gives the pose as `y`). So the MPC keeps `y`, and only `u_ff` → `u`
  is a rename; it lands with AC-8a, which drops the sampled loop's port keywords.
- **AC-5:** how the auto mode removes duplicate wires, and whether a plant with many outputs
  (the racecar) needs a name filter.
- **The deferred wrapper.** For a plant without `d` and `n` ports, `closed_loop`'s
  `disturbance=` and `noise=` add summing junctions in the loop, named `disturbance_sum` and
  `noise_sum`, and the cost then sees `u + d` as the plant's command: a known limit. An
  automatic wrapper that would give such a plant its own `d` and `n` is deferred until
  someone needs it. If it is ever built, it settles the names inside it and how its edge
  shows the plant's `x`. Until then, the cost limit stands.

**To revisit when the estimation features land** (P4, then the EKF and the time-varying and
discrete Kalman filters). The `observer` role and the standard names around it were set
before any estimator existed, so they get a review then, against the real blocks:
- the estimator's ports (`u`, `y` → `x`, labelled x̂), and whether the controller should read
  the estimate under its own name rather than `x` (**settled 2026-10-14, R19:** it stays
  `x`; the role says whose);
- how `closed_loop(..., observer=)` wires them, and where the estimator sits in a
  controller box (**settled with AC-4, R18:** by role at the loop's top level, or in the
  `lqg` box; both give one loop);
- what else a filter gives: a covariance, an innovation;
- the noise inputs `d` and `n` that a Kalman design reads, and their link to the noise
  blocks' `psd` (**settled 2026-10-14, R20:** the design takes `Q_n`, `R_n`; no link);
- how plots compare the estimate with the plant's state, and how Monte Carlo starts x̂0.
