# Automation by convention: plants, controllers, loops, plots and tools

**Status:** decided on 2026-10-11, amended on 2026-10-13. The conventions, the resolver and the
operators landed between 2026-10-10 and 2026-10-14 (steps AC-0 to AC-4): the standard port
names, the three reserved role names `plant`, `controller` and `observer`, `roles`,
`block_roles` and `command_port`, and `+`, `autowire`, `>>`, `@`, `closed_loop` and
`feedback`. Their contract is DESIGN §4, "Control laws: the port names are the declaration",
and the `DiagramSystem` section beside it; the cost scored on the plant (the deterministic
half of AC-7) landed with the problem evaluator in v0.3 (DESIGN §6). This doc holds what is
left:
- the role-aware plots (AC-5) and the animation (AC-6), in the plotting lane;
- the cost's draws (the rest of AC-7), now on the 2026-10-27 choices R143 and R144: a problem
  stochastic by its parts, one realized tree;
- the sampled loop (AC-8a, AC-8b), now on R145–R147 (the loop a `SampledDiagramSystem`, periods
  on blocks, two run verbs) and R148 (the MPC, and the new standard port `t`); the record is
  [the core-vision decisions](../reviews/2026-10-27-core-vision-decisions.md), the design home
  [sampled-path.md](sampled-path.md) §8.

How each rule was reached (the analysis of the old code, the evidence, the options rejected, the
fifteen rulings) is in the decision record,
[2026-10-11-automation-by-convention.md](../reviews/2026-10-11-automation-by-convention.md);
the second opinion and its eight rulings (R1–R8) are in
[2026-10-13-automation-by-convention-second-opinion.md](../reviews/2026-10-13-automation-by-convention-second-opinion.md).

**Where it sits:**
- the cost's draws are the core's draws ([core-architecture.md](core-architecture.md) §3,
  [TODO.md](TODO.md) §1.3);
- the sampled loop is the core's hybrid loop (core-architecture §5, TODO §1.5); keeping today's
  loop in step (AC-8a) runs alongside (TODO §3);
- the plots and the animation stay in the plotting lane (TODO §3).

Sections 1 to 4.4 described the landed part and were cut (the history is in git); the
section numbers of what is left are kept, so the citations to them hold.

## The conventions, in short

Everything is a `System` with named ports, a diagram is `System`s wired by name, and three
reserved subsystem names tell every tool which block is the plant, the controller and the
estimator. A plant reads `u`, `d`, `n` and gives `y`, `x`; a controller reads `e`, `y` or
`x`, plus `r`, and gives `u`; an estimator reads `u`, `y` and gives `x`. A block that reads the
time has an input `t`, wired by convention to the loop's clock (R148, §4.6). Roles are never
inferred from the wiring: a diagram with no `plant` has no plant, and a tool that needs one
says so. Every shortcut (`+` with `autowire`, `>>`, `@`, `closed_loop`, `feedback`) builds
exactly the diagram a student could write with `add_subsystem` and `connect`. DESIGN §4 states
the rules; the open steps below extend them to the plots, the camera, the draws, the sampled
loop and the clock port.

## 4. The operators

`+`, `autowire`, `>>`, `@`, `closed_loop` and `feedback` (§4.1 to §4.4 of the plan as
decided) landed; DESIGN §4 states them. The sampled loop is what is left.

### 4.5 `%` and `computer @ plant`: the sampled loop

The design is [sampled-path.md](sampled-path.md) §8; what follows is what it means for the
conventions.

- **`%` is `discretize`, Euler by default, for any system** (R145): `ctl % dt` returns a
  `SampledSystem`, "this block on this clock" (R146). A law with continuous states (a PID, a
  lead-lag, a transfer function) becomes its difference equation; a static law and a step
  block are tagged with their period. Today `%` builds a `Computer` and refuses a law with
  continuous states (`require_step_subsystem`); the MPC's own `%` (with `dt_mpc`) goes with
  R148, and `mpc % dt` is the generic one.
- **`computer @ plant` follows the `@` rule** (DESIGN §4, "Control laws"), on the step algebra
  (R126):
  - the computer reads `y` or `x`, and the plant's sampled port gives it; otherwise an `Error`
    block sits inside the computer, on the sampled measurement;
  - the continuous plant becomes `discretize(plant, Δ, substeps=n)` under the role name `plant`,
    and the result is a `SampledDiagramSystem` (R146): no loop class, no `.plant` and
    `.computer` fields of a separate object;
  - boxes on both sides, as `@` allows: the plant a box with edge `u` → `y` (its state read for
    an `x`-reading computer such as the MPC), the computer a box (filter and controller, at
    several rates) with edge `r`, `y` (or `x`) → `u`.
- **Several rates are composition** (R146): periods sit on blocks (`(filt % 0.01) >> (ctl %
  0.1)`), the base period is their greatest common divisor, and held outputs are state; no
  schedule names the blocks.
- **The sampler and the hold are not blocks:** the plant's `SampledSystem` samples at the ticks,
  and a block's held outputs are state slots of the diagram.
- **Custom names** are wired by hand like any diagram (`add_subsystem`, `connect`), not through
  `HybridDiagram(...)` and `connect_boundary(...)`, which go.
- **What the loop gives the tools** (AC-8b): it is a `System` of the step algebra, so
  `Step() >> ctl % dt @ plant` (the source on the continuous side, sampled by the computer),
  `(P >> Sat) % dt`, the plots, the cost and Monte Carlo read it like any loop;
  `compute_rollout` gives the ticks and `compute_trajectory` the dense plant (R147). Until
  then, the reference is passed with `compute_forced(r, input_port_id="r")`.
- **Open:** the role name of the computer side inside the loop (`controller`, as a continuous
  `@` gives, or a box holding `controller`, `error` and `filter`), settled with the landing
  change.

### 4.6 The clock port `t` (R148)

- `t` joins the standard port names: an input that reads the loop's clock, wired by convention
  when the block is composed, as `y` and `u` are. It holds seconds in a sampled or continuous
  loop and the tick in a pure step loop, so a block that reads time never knows the sample
  period.
- Its first reader is the MPC (`inputs={"x_start": "y", "t_start": "t"}`,
  [planner-and-mpc.md](planner-and-mpc.md) §6.1). Delay compensation rewires it (`t + Δ`) by
  hand.
- On the MPC, an unwired `t` leaves the bound leaf (`t_start`) at its value in the tree, so the
  tree's shape never depends on the wiring.
- The core must provide it (planner-and-mpc.md §10, point 4); where the clock comes from in a
  continuous diagram (the simulator's time, a clock source block) is the landing change's
  detail.

## 5. What each tool does

The rows for the design and planning tools, RL and Gym, analysis and `plot_control_law()`
landed with AC-4 and AC-7's deterministic half (DESIGN §4 and §6); the three rows below are
the open ones. Of the cost row, the plant's run scored, controllers with state, time, the
cost's params and one price landed in v0.3 with the problem evaluator; what is left is the
draws, on R143 and R144:
- the problem is stochastic by its parts: `x_start=` a point, a `Set` or a `Distribution`,
  `params=` laws in the tree's own shape; `StochasticPlanningProblem`, `x0_distribution=` and
  `params_distribution=` go;
- `problem.realize(key, n_trials) -> params` is one tree: the start is the `x_start` leaf, the
  plant's start only, with a trial axis; a law in a leaf is drawn to a value from a sub-key by
  its path, and a source block (a noise block, the seeded policy) gets an integer seed leaf;
- every Monte Carlo trial is scored on the same compiled loop, the realized tree passed as its
  `params`.

The design home of the draws is [randomness.md](randomness.md).

| Tool | Rule |
|---|---|
| Plots | `signals="plant"`, `"controller"` or `"observer"` selects every port of that block, as `"plant:y"` selects one. The default *auto* mode shows every port of the plant and the controller, each wire once; the reference, the error, `d` and `n` come in through their inputs. Signal names take paths (`plant/plant:y`), and a block inside a box is found by object too. Labels and units are copied onto edge ports, and colours follow roles. With no roles, plots keep their plain default; a leaf keeps `("x", "u")`. `compute_trajectory(show=True)` and the planning and comparison plots use the same default. |
| Animation | Resolved when animating. The camera comes from the one drawable block anywhere in the tree, else the drawable block inside `plant`, else an auto-fit view. A compensator draws no skin inside a loop. `scene_grid` and the follow frame come from the same block. |
| Cost and evaluation | The cost is scored on the plant. A tool given a plant and a policy builds the loop itself (`closed_loop_of(problem, policy)`, compiled once by `problem.compile(policy, dt)`, the one engine of `evaluate` and `compare` since the rulings of 2026-10-22/23, R15 and R22), and scores the plant's run `traj.of("plant")` (today's `trajectory_of(plant)`): the plant's state and its `command_port`. The problem never fills the loop's reference (R38): every free input, `r` included, stays at its port's nominal value; the setpoint is the controller side's (`controller.inputs["r"].set_nominal_value(...)`, or `closed_loop(..., reference=Step(...))`), and the free inputs stay leaves of the compiled problem's `params`. Controller and estimator states are integrated, never scored. A controller with state is integrated, not refused; time is honoured; the cost's params are passed; leaving `X` has one price. The start drawn is the plant's, the `x_start` leaf of the realized tree (R144). |

## 6. What goes, and the migrations

The removals tagged AC-0, AC-2, AC-OP and AC-3, the renames and their migrations landed
(2026-10-13); the decision record lists them. What is left:

**Removed:**

| What | Replaced by | Step |
|---|---|---|
| MPC's output `u_ff` (the override attributes went with AC-3; MPC keeps `y`, the state of its model) | `u` (§10); the MPC's ports out are `u`, `z`, `x_ref`, `u_ref` (R148) | AC-8a |
| `hybrid_closed_loop`'s `computer_out`, `computer_in`, `plant_in`, `plant_out`, `ref_port`, `output_port`, `extra_boundaries` | standard names; custom names wired by hand | AC-8a |
| The sampled loop's port guesses (`u_nom`, then `u`, then `u_ff`; the first port) | the standard edge; dual rate is `(mpc % 0.05) >> (PlanFollower(planner) % 0.01)`, which exposes `u` (R148) | AC-8a |
| `hybrid_closed_loop`'s `schedule=` and `computer=`, with `Computer`, `StepSchedule` and `HybridDiagram` | periods on blocks; `computer @ plant` a `SampledDiagramSystem` (R146) | AC-8b |
| The camera copied at composition time, and `camera_priority` (never read) | the camera chosen when animating | AC-6 |

AC-8a's three rows edit code that R146 and R148 remove in AC-8b; whether AC-8a still lands
first is the workboard's call ([TODO.md](TODO.md) §1.5).

**Migrations:**

| Where | Change |
|---|---|
| The sampled-loop demos, the GRO501 `racecar_toward_mpc` notebook, the MPC project | drop the port keywords and the schedule: `mpc % dt @ inner` once MPC gives `u`; custom names by hand |
| The callers of `StochasticPlanningProblem(..., x0_distribution=...)` (the GRO860 RL notebooks, tutorial 11, the showcases) | swept once with R143 **[student-facing, given]** |

## 7. Steps, in build order

Each step lands in its own commits, with the checks of §8. A census of about 40 loops (ids,
connections, edge ports with labels and units, params keys, `x0`, a fixed-step history,
roles and control-law sweeps) is captured before and after each step and compared.

| Step | What | When |
|---|---|---|
| AC-1, AC-0, AC-2, AC-OP, AC-3, AC-4 | Landed 2026-10-10 to 2026-10-14. | — |
| AC-5 | Plots (§5): the auto mode, role selectors, paths into boxes, labels, units and colours by role, one default | plotting lane |
| AC-6 | Animation (§5): the camera chosen when animating, looking inside `plant`; no compensator skin in a loop; the camera copy goes | plotting lane |
| AC-7 | Cost and tools (§5): one compiled loop (the compiled problem) for every evaluation, `command_port`, controllers with state, time, cost params, one price; Gym and the rollout environment aligned | the deterministic part landed with V1's steps 2–4 (the plant scored, controllers with state, the price switch of R32); the draws (R143, R144) with V1's step 7, RN-4's per-block noise seeds and the rest of RN-5 (TODO §1.3) |
| AC-8a | The sampled loop keeps in step (§4.5, §6): `hybrid_closed_loop`'s keywords and the port guesses go, MPC gives `u` | alongside (TODO §3) |
| AC-8b | The sampled loop goes mainstream (§4.5, §4.6): `%` as `discretize`, `computer @ plant` a `SampledDiagramSystem`, the two run verbs, the port `t` (R145–R148) | with S31 (TODO §1.5) |

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
- No sampled-loop feature outside S31's design ([sampled-path.md](sampled-path.md) §8).
- No change to `ProportionalController`'s default port layout.
- No sources on arbitrary ports such as a manipulator's tool force `f`.
- No renaming of the named-port plants (racecar, bicycle): they are wired by hand.

## 10. Details settled while building

These do not change the rules:
- **The MPC rename (AC-8a), found while building AC-3:** the MPC reads the state of *its
  model*, which for a higher-fidelity plant box is the box's `y`, not its state (the
  racecar's `inner` box gives the pose as `y`). So the MPC keeps `y` (R148's default binding
  `x_start` ← `y`), and only `u_ff` → `u` is a rename; it lands with AC-8a, which drops the
  sampled loop's port keywords.
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
