# Minilink Code & Review Rules

The universal standards for writing, reviewing, and refactoring code in Minilink.
Followed identically by human maintainers, contributors, and AI agents.

Governed by [CONSTITUTION.md](CONSTITUTION.md). Agent-specific workflow instructions
(such as interaction behavior, git gates, and planning procedures) live in [AGENTS.md](../../AGENTS.md).

---

## The Review Ladder

Reviews and code modifications evaluate code from high-level architectural purpose down to
concrete formatting, ordered by the cost of being wrong:

1. [Should it exist in Minilink? (Scope & Identity)](#1-should-it-exist-in-minilink)
2. [What is it in domain terms? (System Modeling)](#2-what-is-it-in-domain-terms)
3. [Where does it live? (Placement & Dependencies)](#3-where-does-it-live)
4. [What should its API look like? (Interface & Facades)](#4-what-should-its-api-look-like)
5. [What should its code look like? (Textbook Engineering Style)](#5-what-should-its-code-look-like-textbook-style)
6. [What evidence does it owe? (Demos, Verification & Tests)](#6-what-evidence-does-it-owe)
7. [Is it worth keeping? (Lifecycle & Consolidation)](#7-is-it-worth-keeping-consolidation)

---

## 1. Should It Exist in Minilink?

- **1.1 Stay inside the product scope.** Would it make Minilink one of the things it is not?
  Law: CONSTITUTION §1.4, 'Product Boundaries'.
- **1.2 Continuous-time core is primary.** Does it complicate `DynamicSystem`,
  `DiagramSystem`, flow `compile()` or `Simulator` for a discrete convenience? Law:
  CONSTITUTION §2, invariant 4.
- **1.3 Expose structure, never black-box dynamics.** Models make equations, state variables, and
  parameters transparent and inspectable.
- **1.4 One class for NumPy/JAX.** Would a `Jax<Plant>` twin buy readability that one `xp`
  body (5.2) cannot? Law: CONSTITUTION §6, 'Backend-native math'.
- **1.5 Be hard where the identity is.** Guard compile-versus-reference parity, JAX twin
  agreement, and discontinuous closed-loop solvers. Soften elsewhere.

---

## 2. What Is It in Domain Terms?

- **2.1 Name by diagram role, never by implementation technology.** Class and variable names reflect
  their functional role in the block diagram (e.g., `Plant`, `Impedance`, `ErrorBlock`),
  not their software mechanism (e.g., avoid names like `JittedVectorizedEvaluatorLeaf`).
- **2.2 Reserve "Leaf" strictly for diagram roles.** Standalone systems or wrapper classes must use
  descriptive domain names (e.g., `DiscretizedDynamicSystem`), reserving the term `Leaf` exclusively
  for nodes inside a diagram structure.
- **2.3 Dual parameter pattern with student tolerance:** All internal library models follow the dual
  parameter pattern: `self.params` provides the default dictionary for high-level teaching, while
  equations accept an explicit argument (`f(x, u, t, params=None)`) for computing parameter gradients
  ($\partial f / \partial p$) and system identification. `params` holds numeric values only
  (floats, numeric arrays, and integer PRNG seeds; never `bool` or `str` flags). Structural
  choices that change which equation or branch runs (`gravity_comp`, `tracking_ref`, `hold`,
  `interpolation`) are fixed at `__init__` (for example by binding a port's compute function
  to `ctl` vs. `ctl_with_gravity`), never stored in `params`.
  A `params` argument, on an equation or on a tool's verb, is `None` or the complete tree
  (every key, every subsystem); nothing merges, and a one-off tree is a copy of the tool's own,
  edited. Live mutation of one number stays `sys.params["m"] = 4`.
  The same holds for every core math object: a cost, a set, a distribution, a field
  hold a `params` dict (`{}` when they have none), read when the call passes `None`, their
  named attributes views of it; a composed object's `params` holds its parts' own dicts by
  key, so a number has one home; every function is deterministic given its parameters, the
  seed among them. The seven rules: DESIGN §4, 'Parameters'. The written exceptions are DESIGN
  §4's list, and `tests/unittest/test_contract.py` walks every other public core object.
  However, the core framework remains forgiving:
  models where a student hard-codes constants directly into equations must simulate without friction.
  Costs, sets, fields and shapes owe the same signature. Override rule and the remaining
  gap: DESIGN §4, 'Parameters'.
- **2.4 One last answer, read only by the shortcuts.** A run returns its result, and the result
  carries its whole account: the answer with its timings, its history and the solver's raw
  output (`Trajectory`, `PlanningSolution` and its `record`, `OptimizationResult`,
  `Evaluation`). An object stores no other run state, with one exception: an object whose
  convenience verbs need a default (plot, animate, `controller`) keeps the last result it
  returned, once, as the same object, never a copy. That is `sys.traj` on a `System`,
  `last_solution` on a `Planner` (the grid planners' `tables` read through it); a new one is
  named `last_<what the run returns>`. Only those verbs read
  it, as a default; library code never does. A tool takes the result it needs, or uses the one
  its run returned (`tests/unittest/test_last_answer.py` lists every reader). An engine that
  tools call (`Optimizer`, `ProblemEvaluator`, `Simulator`) keeps none. Two other kinds of
  state are not run state: what a method learned and reuses on its next run (the RL weights,
  the tabular `Q`, the RRT tree), and diagnostics of the machinery that no tool reads
  (`last_program` and `last_optimizer` on trajectory optimization, `Simulator.last_debug`). The
  Simulator's `last_traj` and `last_solve_time_s` predate the rule (TODO, D1.4). A new
  exception needs a reason of `sys.traj`'s weight.
- **2.5 Prefer arrays and existing core objects.** New APIs take and return arrays or types
  students already know (`System`, `Trajectory`, `PlanningProblem`, sets, costs). Do not add
  request/response dataclasses, option bags, or adapter layers that only wrap those. A new
  named record is justified when it is a domain noun (a trajectory, a certificate, a plan) —
  not when it is a programming convenience. Before adding a record, name the object it already
  is: a `(lower, upper)` pair is a `BoxSet`, a reset rule is a `Distribution`, a sublevel set
  is a `Set`, a scalar function of the state is a `Field` (`core/fields.py`), a sampled run
  is a `Trajectory`.
- **2.6 Names are short, and the class name is the name.** A block's display `name` is its
  class name unless the class sets its own, and a composite is named as the expression that
  built it (`PID @ Pendulum`, `Step >> Integrator`, `A + B`, `feedback(G)`). Those names
  become diagram labels and plot titles, so a class name is short and compact (`CartPole`,
  `KinematicBicycle`, `PID`), and a class sets an explicit `name` only when the class name
  would mislead, two or three words at most. A long name is a name to shorten, never one to
  restate at length in `name`. Renaming a public class or its display name changes the
  diagram key derived from it, so it waits for a picked step and migrates every course
  notebook in the same commit.
- **2.7 One keyword per kind of choice.** `method=` is the verb's own numerical method (the
  derivative, the optimizer's algorithm, the interpolation, the certificate family); `solver=`
  is the time integration on every tool that integrates (`Simulator`, `compute_trajectory`,
  `evaluate`, `discretize`, the RL planners); `backend=` is the array module, on every tool and facade;
  `renderer=` is the drawing library, on every drawing verb (`plot_renderer=` on `game` and the
  realtime simulator).
  `integrator=` is not used: it names the 1/s block. The flag names of 4.7 follow the same rule.
  Four words, one meaning each (the maintainer's ruling 17b): `cost` is the cost object
  (`problem.cost`, `cost=`, the parameter tree's `"cost"` key); `J` is a scalar, one run's cost
  or a program's at `z` (`J*` at the optimum: a record's `.J`, a solve's `status["J"]`, the
  running signal `"J"` and its rate `"dJ"`); `objective` is the criterion minimized over draws;
  `cost_to_go` is the function `J(x, t)` (a `Field`; `J*` or `J_π` by its source, not its
  name; `ValueTables.J` is the DP table). A quantity that is neither keeps a name of its own
  (`raw_cost`, `path_cost`, `cost_to_come`, `mean_episode_cost`). Recorded exception:
  `Manipulator.J(q)` is the Jacobian, the robotics textbook's letter.
  The whole vocabulary, one word per idea and one idea per word
  (`tests/unittest/test_vocabulary.py` refuses the retired words on the public surface):

  | Idea | The word | Not |
  | --- | --- | --- |
  | Final time, absolute (a run's end, `Trajectory.tf`) | `tf` | `tf` as a length |
  | Start time (a run's, a problem's) | `t0` | `t_start` |
  | A problem's length, from `t0` (`[t0, t0 + horizon]`) | `horizon` | `tf` on a problem |
  | A system's dimensions, in its constructor (words), and as its attributes (textbook symbols) | `state_dim`, `input_dim`, `output_dim`; `n`, `m`, `p` | `n=` as the constructor's keyword |
  | A tool's step or period | `dt` | |
  | A clocked block's own period | `sample_period` | |
  | Number of time steps, the transitions (a transcription's intervals: `N + 1` knots) | `n_steps` | `N` as a count (the equations keep `N`; `N` the reference gain stays) |
  | The RL planner's control periods of each rollout per collection, the data of one update | `n_steps_per_update` | `n_steps` (a horizon's transitions) |
  | Number of time samples on a horizon, endpoints included (`Simulator`, `compute_trajectory`, the frequency and step responses, the LQR gain schedule) | `n_samples` | `n_steps`, `n`, `n_pts` |
  | Number of sampled rollouts (MPPI, a certificate's `verify`, the RL planner's parallel rollouts) | `n_rollouts` | `n_samples` (time samples only), `n_envs` |
  | Iterations, an exact count (a fixed scan); every count is `n_<thing>` | `n_iterations` | `iterations` as the keyword |
  | Iterations, a cap | `max_iterations` | |
  | A training budget, episodes / plant steps (the RL planners' `solve`, their records) | `n_episodes` / `n_timesteps` | `episodes`, `timesteps` |
  | Operating point, target | `x_bar`, `u_bar`, `y_bar` (keyword, `params` key, attribute) | `xbar`, `ubar`, `ybar` |
  | Initial state, `x(t0) = x0 ∈ X0`: a system's, a run's, a problem's (a point or a law) | `x0` | `x_start` |
  | Discount rate ρ (1/s) | `discount_rate` | |
  | Discount factor γ per tick | `gamma` | `alpha` (the CVaR level `alpha` stays) |
  | A per-call draw | `key` | |
  | A seed stored at construction (`0` by default where a run draws; `None` is a random block's mean, never fresh entropy) | `seed` | `rng` (internal only) |
  | The verb's algorithm (value iteration's `method="loop"`, RRT's `nearest_method`) | `method` | `algorithm`, `backend="loop"`, `nearest_backend` |
  | A bound on an object (a port, the state, a set, `Saturation`, the program; `add_input_port(lower=, upper=)`, `port.lower`, `state.upper`); a limit a law applies to a named signal carries that signal's name (`u_min`, `u_max`, `delta_max`, `e_int_min`) | `lower`, `upper` | `lower_bound`, `upper_bound` |
  | The optimizer's algorithm, and the implementation it runs (one table keyed `(method, backend)`, `backend=` the array module) | `method="slsqp"`, `"trust-constr"` (SciPy's spellings), `"l-bfgs"`, `"gd"`, `"adam"`, `"ipopt"` | `"scipy_slsqp"`, `"scipy_trust_constr"`, a library in the method's name |
  | The drawing library of a plot or an animation | `renderer` (`plot_renderer` on `game`) | `backend`, `plot_backend` |
  | The method's numbers, as keywords (`Optimizer`, trajectory optimization, the RL planner) | `**method_numbers` | `**algorithm_kwargs`, `**method_kwargs` |
  | A planner's record | `solution.record` | `solution.solver` |
  | The block the RL planner trains, given and read back (the actor of actor-critic; the handed-out copy is `solution.controller`) | `actor=`, `planner.actor` | `policy=` |
  | Train, and train more: every planner's one verb (the RL planner's `solve(n_timesteps=)` and the tabular learner's `solve(n_episodes=)` continue their training); the learned law's action | `solve`; `solution.controller`, `planner.actor.action(x)` | `learn`, `predict` (Stable-Baselines3's words, on the bridge's `SB3Controller` only) |
  | A grid planner's tables, `J` and `pi` by node: the latest solution's; a tabular learner's table as it stands | `ValueTables`, `solution.record.tables`, `planner.tables`; `planner.greedy_tables` | `DynamicProgrammingResult`, `planner.result` |
  | A solution's planning family (`"value iteration"`, `"riccati"`, ...) | `solution.family` | `solution.method` |
  | A record's counts: tree nodes, iterations | `n_nodes`, `n_iterations` | `nodes`, `tree_nodes`, `iterations` |
  | A solve's wall time, on every planner's record (`None` where the solve has no wall clock) | `solve_time_s` | `train_time_s` for a solve's time (the neural learner's `train_time_s` counts every training of the planner) |
  | A record's cost, named by its quantity: the objective at the solution (the `Optimizer`, trajectory optimization and an iterate, MPPI, a compiled solve's `status`); a search's path cost-to-come, a tree node's, RRT*'s best; a learner's mean episode cost at the end of training (each `history` entry's own cost is `"J"`); a scored run's cost before the failure charge, beside its score `J` (`Evaluation`) | `J`; `path_cost`, `node.cost_to_come`, `best_path_cost`; `mean_episode_cost`; `raw_cost` | `cost` for any scalar (the cost is the object), `episode_cost`, `best_goal_cost` |
  | A run's running cost signals (`traj.with_cost`, `plot_cost`, a step problem's `rollout`): the discounted rate, its integral with the terminal cost at the last sample | `"dJ"`, `"J"` | `"cost_rate"`, `"cost"` |
  | A trajectory-optimization iterate's constraint residuals (as on its record) | `max_equality_violation`, `min_inequality_margin` | `max_eq`, `min_ineq` |
  | A block's channel count (sources, white noise, routing, saturation, delay, the PID family, the impedance laws) | `dim` | `p`, `dof` (`dof` stays a manipulator's joints) |
  | The fixed-step time integration of `solver=` (the simulator, as `discretize` and the evaluators) | `"rk4"`, `"euler"` | `"rk4_fixedsteps"`, `"euler_fixedsteps"` |
  | An author's reads: the input ports' values in `u`, by name; their nominal values, stacked; a recorded signal of a run | `sys.ports(u, "r", "y")`; `sys.u_nominal`; `traj.signal(name)` | `get_port_values_from_u`, `get_u_from_input_ports`, `get_signal` |
  | A system placed in a diagram (`add_subsystem`, `diagram.subsystems`, the printout's `subsystems:`, `get_subsystem_input_port`); "block" is kept for the standard components of `minilink.blocks` | `subsystem` | `block` for a diagram's member, `subsys` |
  | A run's trajectories kept by `evaluate` | `trajectories=True` | `record=True` (a record is a planner's account) |
  | A port a keyword names (a source's output, a run's prescribed input); the run a plot draws | `port=`; `traj=` | `input_port_id=`; `rollout=` |
  | How samples are read between their times (the step model's rollout, the noise sources, `TrajectorySource`) | `hold="zoh"`, `hold="linear"` | `interpolation=`, `"previous"`, `"foh"` |
  | A drawing in 3-D; a file to write; a plot's two state axes; a plot's box | `is_3d`; `file_name`; `x_axis`, `y_axis`; `bounds` | `show_3d`; `filename`, `save_path`, a path in `save=`; `axes=(i, j)`, `axis_x`, `axis_y`; `limits` |
  | Playback rate of an animation (simulated seconds per wall second) | `speed` | `time_factor_video` |
  | An episode's length (`Sys2Gym`) | `episode_length` | `tf` as a length |
  | An external solver's own knobs (SciPy, Ipopt) | `options=` (a dict) | `optimizer_options`, an options dataclass, a native method's numbers (`max_iterations=`, `tol=`, `learning_rate=` are keywords) |
  | A planner's numbers (varied per solve, traced) | `planner.params` (`params["method"]`), `solve(params=)` | `planner.settings` |
  | Print text / draw a figure (and where: `plot_diagram(show="inline" \| "window" \| "pdf")`) | `verbose` (default `False`) / `show` | `show_inline`, `show_pdf` |
  | Finite-difference step / convergence tolerance / a radius | `eps` / `tol` / `radius` | `eps` as a radius |
  | Cost-to-go and control-law plots, and their sweeps animated | `plot_cost_to_go`, `plot_control_law`; `animate_cost_to_go`, `animate_control_law` | `plot_cost2go`, `plot_policy`, `animate_policy` |
| A plant's signals: the state's order (a style guideline, not a rule enforced on a user's plant), a rate's label, a unit, a default label | positions, then their rates (`[q; dq]`, all the positions first on a plant of several bodies); `d<position>` (`dtheta`, `dy`) unless the textbook names the rate (`omega`, `yaw_rate`, `vx`); a bare unit (`"Nm"`, `"rad/s"`: the plot adds the brackets); counted from 1 (`q1…`, `dq1…`, `u1…`, `p1…`) | rates first; `"[Nm]"`, `"rad/sec"`; `"Torque 0"`, `"Angle 0"` |

  **A naming pattern, not a hard rule: the controller is the block,
  the policy the function.** Use it where it fits:
  - a **policy** (or **control law**) is a pure function, `u = π(x, t)`: arrays in, arrays out,
    no state (the compiled face's `pv.command(z, x, t)`);
  - a **controller** is the block, or the diagram of blocks, a loop closes on: it may carry a
    state and several functions (a PID's integrator, an observer, the MPC box), wired with
    `controller @ plant` (`solution.controller`, `planner.controller`).

  A controller built on a static policy (a state feedback, a lookup table, a network) is almost
  the same thing as its policy; the words still keep the two roles apart: the block for the
  wiring, the policy or control law for the function. Class names may name the law a block
  implements (`LookupPolicy`, `NeuralPolicy`, `StateFeedback`).

---

## 3. Where Does It Live?

*Maintain strict modularity and clear dependency boundaries.*

- **3.1 Placement algorithm.** Is a `System` shelved by its diagram role and a verb by its
  tool band, with plant math under `dynamics/` and never under the `minilink.catalog` alias?
  Algorithm: DESIGN §3, 'Placement algorithm'. Runnable examples, prototypes included,
  follow examples/README.md; developer benches live under `benchmarks/`.
- **3.2 The dependency law.** Does a library import a peer library or catalog content outside
  the exceptions DESIGN names? Law: DESIGN §3, 'Dependency law'.
- **3.3 Two lanes.** Which lane does it join, and does it meet that lane's rules? Contract:
  [ROADMAP.md §2](ROADMAP.md#2-two-lanes); philosophy: CONSTITUTION §3.
- **3.4 Clean renaming:** When renaming a component or module, update all call sites across the
  entire repository in the same change. Do not leave deprecated aliases behind.
- **3.5 Decoupled rendering & headless dynamics:** Classes in `minilink.dynamics` must never import
  graphical rendering libraries (Matplotlib, Pygame, Meshcat). Physical geometries are declared using
  lightweight declarative primitives (`Circle`, `Rod`), and rendering engines are loaded lazily by
  user scripts or simulator wrappers, ensuring dynamics remains 100% headless.
- **3.6 A folder README must earn its keep.** Add one only when it states a policy, how to
  run, or a bucket contract that a directory listing cannot. Do not add a README that
  merely lists the files in that folder. The existing maps (root `README.md`,
  `examples/README.md`, `tests/README.md`) stay the maps; leaf demo folders do not get
  a second copy. Do not add new markdown guides unless the maintainer asks.
- **3.7 Intro surfaces stay canonical.** README.md, the showcase notebook, and
  `examples/tutorial/` present the main core tools: `System`, diagrams, simulate,
  compile, analysis, planning trajopt, the sampled step path (`StepSystem`,
  `StepDiagramSystem`, `SampledSystem`, `SampledDiagramSystem`), and MPC as its exemplar.
  Do not update them to track every new demo, compare script, or research-lane
  experiment. New demos land under `examples/`, whose map is `examples/README.md`. Add a
  row to README's verb table only when a core tool gains its canonical call.
- **3.8 Notebooks are not a second map.** The catalog is `examples/README.md`, whose Layout
  section holds the link policy per notebook kind; a notebook does not duplicate it with See
  also / Where next hubs or peer links.

---

## 4. What Should Its API Look Like?

*Provide clean, zero-boilerplate access for students and power for researchers.*

- **4.1 Three import layers.** Does the import use the shortest layer that stays clear: root
  prelude, band facade, defining module? Layers and the factory-module exception: DESIGN §2,
  'Public imports'.
- **4.2 Student code imports through the teaching surface.** README, tutorials, teaching
  examples, and demos import strictly via the root prelude or band facades, never through
  internal paths.
  Reader-facing imports stay light in demos; internal packages may import richly when that is
  clearer. Where a band facade does not exist yet, use the shortest import that works and
  leave the facade to a planned step — do not invent a name.
- **4.3 Preconditions met by named adapters.** Never ask the user to write a separate model for
  different tools. A tool requiring a linear model accepts any `System` via `linearize()`; a tool
  requiring discrete stepping accepts a continuous system via a discretization adapter; a tool
  requiring a deterministic problem accepts a stochastic one via `problem.nominal()`. Convert once
  at the boundary, then use the richer object directly: never probe it with `hasattr` or
  `getattr` defaults inside the math. A tool that needs a box asks the set
  (`X.bounding_box()`); it does not `isinstance` the set. A planner or policy builder that
  needs state or action bounds reads `problem.X.bounding_box()` and `problem.U.bounding_box()`,
  never `sys.state.box` or an input port's `box` behind the problem's back. The one `isinstance` on a set is the
  lowering at the solver boundary: a singleton to an equality residual, a box to decision bounds.
- **4.4 Parameter override rule.** Does `params is None` resolve to the defaults and any
  dictionary replace them whole, never `params or self.params` and never a merge? Does a tool
  check a tree it takes against its own (`checked`)? Contract: DESIGN §4, 'Parameters'.
- **4.5 Unconnected input ports are silent.** Does an unwired input read its nominal value
  without a warning or an error? Contract: DESIGN §5.
- **4.6 Libraries are silent.** Library functions and classes must never `print()`, except when
  an explicit `verbose=True` argument is passed. Delete debug scaffolding before handoff.
- **4.7 Unified flag names.** Standardize parameter names across tools (e.g., use `verbose`,
  `solver`, `backend`). Unify the *name*, not the format: `verbose=True` keeps the
  framed simulation panel.
- **4.8 Diagram connection operators and feedback topologies.** Does it add a way to wire a
  diagram beside `>>`, `+`, `@`, `DiagramSystem.connect()` and `feedback()`, or a hybrid
  operator beside `%`? Law: CONSTITUTION §6, 'Composition grammar is frozen early'; operator
  semantics: DESIGN §4, `DiagramSystem`.
- **4.9 Port names are the wiring contract.** `u`, `r`, `d`, `n` in, `y` and `x` out: the
  names drive the operators. Contract: DESIGN §4, 'Control laws'.
- **4.10 Early structural validation:** Diagram connection utilities (`@`, `>>`, `DiagramSystem.connect`)
  validate port existence and dimensional compatibility at wiring time, failing with explicit
  error messages naming the mismatched ports and subsystems; a connected input is rewired only
  through `disconnect`, and an output seen at the compile probe to move with an undeclared
  input is refused. This rule
  covers wiring, not `f`/`h` return-shape checks at ODE time.
- **4.11 Build vs run.** Does runtime stepping mutate what wiring and `compile()` froze? Law:
  CONSTITUTION §6, 'Build vs run'.
- **4.12 Defaults that change the mathematics are announced.** When a tool fills in a value
  that changes what is optimized or reported (a discount, a horizon, a tolerance), it warns
  once with `warnings.warn`, stating the consequence in domain units (an effective horizon in
  seconds, not a factor), and stays silent when the caller sets the value.

---

## 5. What Should Its Code Look Like? (Textbook Style)

*Reading Minilink source code should feel like reading an engineering textbook.*

- **5.1 Bare signatures in equation paths:** An equation path is a method CONSTITUTION §2
  names ('One signature') or one of DESIGN §4's additions ('Native-array equation rule').
  Do not put type hints inside one. Document expected array shapes and physical units clearly
  in the NumPy-style docstring. Type hints belong on class constructors, public methods, and
  tool orchestrators. Under `from __future__ import annotations`, forward references are
  unquoted.
- **5.2 The `xp` idiom for hybrid NumPy/JAX:** On the line right after the `params` binding of
  any equation path (5.1), bind the array module (block 1 of rule 5.3):
  ```python
  params = self.params if params is None else params
  xp = array_module(x)
  ```
  Write the subsequent mathematical algebra using `xp` so the exact same equation path executes
  on both NumPy and JAX arrays without branching.
- **5.3 Three beats in an equation path; no `self.` in math lines:** Every equation path
  (5.1), the model hooks it calls (`forward_dynamics`, …), and every algorithm body
  (analysis tools, design functions, planners, solvers, learning updates) is three beats:
  **unpack**, **core math**, **output machinery**. They are laid out as the same six blocks,
  always in this order, with a blank line between blocks so the core equation is the thing
  the eye lands on. A block with nothing to say is left out.

  1. **Bindings** (unpack). `params = self.params if params is None else params`, then on
     the next line the backend: `xp = array_module(...)` (5.2), or
     `jnp = require_jax_numpy()` in a JAX-only algorithm.
  2. **Parameters** (unpack). The parameters and the object's fields, **one per line**, in
     short textbook names (`m = params["m"]`). No `self.` remains in the algebra. An input
     check (`require_*`) comes right after this block.
  3. **Pre-computation** (unpack). The state and input unpacked (`theta = q[0]`),
     trigonometric shorthands, intermediate quantities.
  4. **Core math.** Each equation a **named assignment**, with a short textbook comment
     above it only when that comment is not a copy of the code (rule 5.22). A check on a
     computed quantity sits right after the line computing it (`ctrb = …`, then
     `require_controllable(ctrb, n)`). A simple array (a vector, or a matrix with short
     entries) is built here, on its own equation line, the last of the block.
  5. **Assembly** (output). Only for a complex array: its entries are named lines in
     block 4 (`J11`, `J12`, …), and the matrix is assembled here, row by row (5.5).
  6. **Return** (output). `return` of a name. Never an equation or an array on the
     `return` line; a result object built from named results may be
     (`return ValueTables(grid=grid, J=J, …)`).

  A delegation, a constant, a dict literal or a frame composition stays the one line it is:
  the blocks are for equations (`tf` returns its frames as one dict literal). A numeric
  constant stays inline where the textbook writes it, with its meaning in the equation
  comment (`+ 1e-8`, `10⁻⁸`); it becomes a named module constant only when two places share
  it or it is a tuning knob (`RANK_TOL`). Spacing inside a line is `ruff format`'s
  (`m * l**2`); the blank lines between the blocks are this rule's.

  **The math is the story.** The body shows every step of the algorithm in the order the
  textbook derives it, with the textbook's symbols (`A, B, C, D`, `𝒞`, `𝒪`, `K`, `P`, `J`,
  `π`, `n, m, p`). The linear algebra is part of it: a Kalman matrix, an SVD and its rank, a
  basis and a projection, a Riccati solve, a Bellman backup are each a named line in the
  body, never hidden in a helper. **The plumbing is not the story.** Array coercion, shape
  normalization, default filling, validation, backend dispatch, caching and result
  wrapping go to helpers under `# Internal machinery`, or to the unpack and output beats.
  If a helper contains an equation from the textbook page, it is in the wrong place.

  ```python
  # Good — the six blocks; the comment is the implicit form, the code the solved
  # assignment, and the simple array is the last line of the equations:
  def f(self, x, u, t=0.0, params=None):
      params = self.params if params is None else params
      xp = array_module(x)

      m = params["m"]
      l = params["l"]
      g = params["g"]
      d = params["d"]

      theta = x[0]
      omega = x[1]
      tau = u[0]

      # (m l²) ω̇ = τ − m g l sin(θ) − d ω
      dtheta = omega
      domega = (tau - m * g * l * xp.sin(theta) - d * omega) / (m * l**2)
      dx = xp.array([dtheta, domega])

      return dx

  # Good — a complex matrix: named entries in the core math, then their assembly:
  # fmt: off
  C = xp.array([
      [C11, C12],
      [C21, C22],
  ])
  # fmt: on

  # Bad — the parameters packed on one line, the array built on the return line:
  m, l, g, d = params["m"], params["l"], params["g"], params["d"]
  return xp.array([omega, (tau - m * g * l * xp.sin(theta) - d * omega) / (m * l**2)])

  # Bad — a textbook step hidden in a helper; the SVD, rank and basis are the lesson:
  T_c = orthonormal_range(ctrb, tol)

  # Good — each step a named line, the tolerance included:
  # 𝒞 = U Σ Vᵀ, keep the r directions with σᵢ > tol σ₁
  U, sigma, _ = np.linalg.svd(ctrb)
  r = int(np.sum(sigma > tol * sigma.max(initial=0.0)))
  T_c = U[:, :r]
  ```
- **5.4 Mathematical naming conventions:**
  - Matrices: uppercase (`A`, `B`, `C`, `D`, `H`, `M`, `K`).
  - Vectors: lowercase (`x`, `u`, `y`, `q`, `v`, `dq`, `dx`).
  - Dimensions: lowercase integers (`n`, `m`, `p`).
  - A symbol keeps one meaning within an object, and one meaning across the codebase wherever
    the literature allows: `A` is the state matrix, `rho` the discount rate, `z` the policy
    features. Name a local that would collide by its role (`advantage`, `ratio`, `eps`)
    rather than reusing the symbol. The textbook's own overloads across objects are
    inherited and never added to: the closed list is DESIGN §4, 'Notation collisions'.
- **5.5 Format 2D array literals row by row:** Align matrices visually using `# fmt: off` and
  `# fmt: on` to ensure equations remain readable at a glance:
  ```python
  # fmt: off
  A = xp.array([
      [0.0, 1.0],
      [-k / m, -b / m],
  ])
  # fmt: on
  ```
- **5.6 Derived, not cached:** Computable quantities that depend on state or parameters must be
  read-only properties (`@property`), never cached mutable attributes that can become stale.
  One owner per quantity: a value two components need (a discount, a time step) lives on one
  object, and the other reads it through a property instead of keeping a copy that can
  silently disagree.
- **5.7 No shadow state:** Initialize all object attributes explicitly in `__init__`. Never dynamically
  attach attributes at call sites or rely on `hasattr`-and-create patterns. A constructor that
  grows reads as named steps: helpers below return values and `__init__` assigns them, so every
  attribute is still visibly set there. Attributes that only one branch fills start as `None`
  before the branch.
- **5.8 No leading-underscore pseudo-privacy:** Do not mark a method or a function internal with
  a leading underscore (`_`): not on the `System`, facade and simulator classes, not on
  planners and tools, not on module-level helpers (`check_poles`, never `_check_poles`).
  Separate public interfaces from internal machinery using standard file section comments
  (`# Public API` and `# Internal machinery`). These two titles stand out: each sits between
  two full lines of `#`, with two blank lines around the banner; inside a class the banner is
  indented with the class body, after the public methods. Smaller sub-sections are plain
  sentence-case comments (`# Frequency grid`), never `# ====` or `# ----` rules; display hooks
  (`__str__`, `_repr_pretty_`) sit under `# Display`.
  ```python
  ###############################################################################
  # Internal machinery
  ###############################################################################
  ```
  Reserve leading underscores for module-level
  private constants and purely local closures. Existing underscore helpers migrate under
  the repo-wide ratchet `tests/unittest/textbook_style.py`, whose allowlist only shrinks.
- **5.9 The first-screen rule:** The module docstring and primary class definition must appear within
  the first screen (~50 lines). Place complex validation and helper machinery below.
- **5.10 Standard module section order:**
  1. Module docstring & imports
  2. Primary class / public contract
  3. Subclasses & specialized variants
  4. Public functions & convenience facades
  5. `# Internal machinery` (helpers, evaluators, validation), the helpers in the order the
     body calls them

  The module constants the primary code reads sit next to their owner's section. A class
  reads the same way: its docstring, `__init__`, the properties, the public methods (the
  primary verb first), then its indented `Internal machinery` banner and its helpers, in
  the order the public methods call them.
- **5.11 Selector-orchestrator split:** Public math tools follow a clear pattern: inspect inputs
  $\to$ select algorithm backend $\to$ extract $f/h$ callables $\to$ execute core math in place.
  Heavy ceremony and validation belong in helper functions below.
- **5.12 Backend imports come from `minilink.core.backends`:** Never import from `minilink.core.compile`
  inside the system libraries (`blocks/`, `dynamics/`, `control/`, `estimation/`). Use
  `require_jax_numpy()` and `array_module()` from `core.backends`. Tools that
  compile a system for a living (`simulation/`, `analysis/`) import the compiler directly. A
  JAX-only function opens with one binding line, `jax, jnp = require_jax(), require_jax_numpy()`,
  never a bare `import jax`; the lazy-import rule (5.27) keeps that line inside the function.
- **5.13 Familiar patterns first:** Do not introduce programming concepts or advanced Python
  styles absent from the repo and the maintainer's prior choices (e.g. `typing.Protocol`,
  metaclasses) unless there is a strong runtime or maintainability reason. Static-typing-only
  wins are not enough on their own — prefer patterns already in use (mixins, composition,
  unions, duck typing). If the tradeoff is unclear, ask before landing the pattern.
- **5.14 Standard SI units and radian convention:** All angles are in radians; all angular velocities
  in rad/s. State spaces keep unbounded continuous angle representations by default unless explicit
  manifold wrapping is declared. Units are strictly standard SI (kg, m, s, N, N·m).
- **5.15 JAX-compiled branchless algebra vs. student clarity:** Core catalog plants, trajectory
  optimization objectives, and controllers intended for JAX compilation or autodiff must avoid scalar
  Python conditionals (`if x > 0:`) in $f$ and $h$, using vectorizable operations (`xp.where`,
  `xp.clip`, `xp.maximum`). However, outside of JAX-compiled paths—such as in student models or
  non-performance analytical paths—standard Python `if` branching is fully supported and welcomed
  if it enhances pedagogical clarity.
- **5.16 Two-audience files:** Write each file for its primary reader. Student-facing modules
  (`core/system.py`, `blocks/`, `dynamics/`, `control/`) and every algorithm (analysis tools,
  controller and observer design, planners, solvers, learning updates) read like a textbook;
  the reference is `planning/dynamic_programming/dp.py`. Library-developer modules
  (`core/compile/`, evaluators) may carry compiler machinery.
- **5.17 Native-array equation paths:** Does an equation path (5.1) convert its arrays inside,
  or stop tracing in an argument, `params` included? Parameter leaves are strictly numeric
  (floats, numeric arrays, and integer PRNG seeds; never `bool` or `str` mode switches, 2.3).
  Contract: DESIGN §4, 'Native-array
  equation rule'.
- **5.18 `__main__` hello-worlds:** Core modules may ship a `__main__` smoke that constructs the
  class and runs it once. Keep it short enough to read at a glance, and stop where the teaching
  starts: a smoke that grows plots, sweeps, or commentary has become a demo and belongs under
  `examples/`. Do not add plant-only demo scripts whose only job is to smoke a catalog class.
- **5.19 JAX float64 by default:** Does a tool require the caller to call `configure_jax`
  first? Policy: DESIGN §1, 'Precision policy'.
- **5.20 Match the neighborhood:** Change only what the task requires. Public APIs use type hints
  and NumPy docstrings except in equation paths (rule 5.1). Lazy imports follow 5.27. Prefer a low
  helper count in math tools; inline single-use helpers.
- **5.21 Validation in proportion:** Validate at boundaries, not in every helper. Frozen dataclasses
  for records — a `Trajectory`, a certificate, a solver record, a solution; plain classes with
  `params` for every parametric object — systems, costs, sets, fields, laws, shapes. Never dataclasses as
  input/output wrappers around arrays (rule 2.5). Use `ABC` only when enforcement helps.
- **5.22 Comment only when it is not a copy of the code.** Beat 2 of rule 5.3 is
  the highlighted equation: a blank line, then a named assignment. Add a short
  textbook comment on its own line only when that comment is *not* a copy of the
  assignment — when the code cannot write the textbook form (`# (m l²) ω̇ = τ − …`
  above a solved `domega = (tau - …) / (m * l**2)`; `# H v̇ = τ − C v − g − d`
  above `solve`; `# dx = [v; v̇]` above `q2x`; `# g = 0 on the target, 1 elsewhere`
  above `xp.where`). Skip the comment when the assignment is already ~90% the
  textbook (`dx = A @ x + B @ u`, `g = dx.T @ Q @ dx + du.T @ R @ du`,
  `H = xp.array([[m * l**2 + I]])`) — the name and the blank lines still highlight
  it. Never put the equation on `return`; never put it in a module or method
  docstring. Apply this to new math and to every math line you edit, without being
  asked; restyling a whole function or module you did not otherwise touch waits for a
  picked step, and then uses the three beats of rule 5.3.
  Equation comments use Unicode math in the textbook's symbols (`ẋ`, `x_{k+1}`, `Aᵀ`, `R⁻¹`,
  `σᵢ`, `≤`, `√`, `𝒞`); identifiers stay ASCII (`x_next`, `theta`, `sigma`). A math
  comment starts with the equation itself (`# L_clip = −E[…]`); a prose comment starts with
  a capital letter, inline and dict group comments included (`# Coupled inertia of the two
  planar links`). No comment restates the line below it.
- **5.23 No preamble walls.** A module or demo opens with a one-line title docstring. Do not
  add a long introduction, section map, run recipe, or flag explanation at the top — the
  code plus inline comments must tell the story. Notebooks are course material: do not
  trim their markdown unless asked.
- **5.24 Docstrings, comments and messages are timeless.** They state the math and the
  contract as a textbook would. They never name the design process (step ids, audits, dates,
  "decided", "landed", "for now", "currently", "until pipeline B", a ROADMAP or TODO
  section), the course or its exercises, or what another library refuses; that history
  lives in the git history. The same holds for test
  docstrings and comments, and for refusal and warning messages: a refusal says what is
  supported and what to pass instead.
- **5.25 One verb family per role:** A helper's name says its role.
  - `require_*` raises when a precondition fails; its docstring opens with "Refuse".
  - `check_*` validates and returns nothing.
  - `build_*` constructs a runtime object (an evaluator, a simulator, a computer).
  - `*_of` resolves an object's options or attributes (`method_params_of`).
  - `plot_*` draws.

  A check that returns a value is a `require_*` when it refuses, or an `*_of` when it
  resolves. Twin helpers in sibling modules share one name; before naming a new
  `require_*` or `check_*`, look for an existing one doing the same check.
- **5.26 The bodies that are not equations:** They keep the air of rule 5.3: named blocks, a
  blank line between them.
  - **A constructor** reads as named blocks, each opened by a short capitalized comment:
    `super().__init__` and the name, then `# Parameters`, `# Ports`, `# Signal labels and
    units`, `# Graphics`. A value a helper works out is returned by that helper and assigned
    in `__init__` (5.7).
  - **A public verb** (`linearize`, `bode`, `lqr`, `simulate`) reads, in order: its input
    checks; its setup (selectors, the point, the options); its math blocks, each with its
    equation comment; its result. The option plumbing goes to `*_of` and `require_*`
    helpers (5.25).
  - **An iterative algorithm** (an RRT search, a DP sweep, a training loop) shows its steps
    as commented blocks inside the loop: sample, extend, add, test the goal. Logging,
    recording, callbacks and live plots are one call per iteration to a helper, never
    inline in the loop body.
- **5.27 Lazy imports say why:** A minilink module is imported at the top of the file. A lazy
  import of one is allowed to break a real cycle, for an optional dependency, or where
  hoisting it would change what a lighter module loads; it carries a comment naming the
  reason, one comment for a block of consecutive lazy imports:
  - `# A real cycle: <who imports whom>`
  - `# Lazy import: the graphics load on first use, not with the core`
  - `# Tools import core (DESIGN §3): the <band> loads on first use`
  - `# Lazy import: the compiler loads on first use, not with the analysis verbs`

---

## 6. What Evidence Does It Owe?

*Trust in the toolbox is built on verifiable evidence, clean demos, and robust tests.*

- **6.1 A demo is the API, and nothing else.** *Simplicity is the ultimate sophistication*
  holds for a demo or a teaching notebook as it does for the core: the shortest sequence of
  library verbs that tells the story — build, compose, solve, evaluate, plot, animate — run
  from the top level, its numbers named as 6.16 says, a one-line title docstring and the story in
  short inline comments (5.22–5.23). Less is better. Reporting goes through the objects
  themselves: their `print` (`__str__`) and their native plots (`plot_trajectory`,
  `plot_control_law`, `plot_cost_to_go`, `plot_learning_curve`, `plot_tree`,
  `scene.plot`, ...). That is `print(solution)` or `planner.plot_learning_curve()`, not an
  f-string of fields (6.13). A report the library cannot give is first a library gap — a
  missing `__str__` or `plot_*`, in the agent's plotting lane — and only then a demo cell.
- **6.2 No test harness code in demos:** Never add test environment branches (`if CI: ...`),
  smoke env vars (`MINILINK_NOTEBOOK_SMOKE`), mock flags, or headless switches inside
  `examples/demos/`, `examples/tutorial/`, or `examples/teaching/`. Test runners adapt
  from the outside (`MPLBACKEND=Agg`, timeouts, optional-dep skips). Falling back when
  an optional package is missing (Ipopt → SciPy) is user UX, not a test hook.
- **6.3 Tests only when justified.** Write tests for stable public APIs, TRL milestones,
  mathematical contracts, boundary conditions, cross-backend parity (NumPy vs. JAX), or an
  explicit maintainer request. JAX twin plants owe a nominal case and a nontrivial
  parameter case. Do not write brittle tests that assert internal helper details.
  Benchmarks only when a performance claim needs a gate.
- **6.4 One parameterized test over many duplicates:** Consolidate related test configurations into
  parameterized fixtures rather than proliferating near-duplicate test files.
- **6.5 Guard optional dependencies:** Third-party dependencies (JAX, Pygame, Meshcat) must be
  lazily checked and skipped gracefully in the test suite when not installed.
- **6.6 Code formatting and linting:** All code must pass `ruff check .` and `ruff format --check .`
  repo-wide without errors.
- **6.7 Zero-friction execution:** Every teaching notebook and canonical demo must run on Google
  Colab (CPU runtime) and on a standard student laptop without external C++ build chains,
  proprietary licenses, or specialized GPU hardware. Keep the main teaching path short; a hard
  wall-clock cap is not the rule (trajopt and RL cells may take longer).
- **6.8 Demo-gate maturity.** Does it meet the teaching surface's entry rule before it enters?
  Contract: ROADMAP §2.
- **6.9 Public-facing prose is foundational, not a bake-off.** README, the showcase,
  and the docs landing page stay positive about minilink and never name
  other tools. Framing: minilink bridges capabilities that usually live in separate tools.
  No superlatives; speed claims follow DESIGN §5 (batches, never a per-call ratio). The main
  line is readable by an undergraduate; expert depth sits in short "under the hood"
  asides. GIF assets stay under 1 MB
  and use catalog plant framing. The no-other-tool rule
  is enforced by `tests/unittest/test_repo_contract.py`.
- **6.10 Flat demos.** No functions or classes in a demo or a teaching notebook except the
  model or cost it is about (its `f`, `h`, `g`) and a step the notebook's text teaches by hand
  (a hand-written RK4, a policy-gradient update). No wiring, tuning or plotting helpers, no
  configuration dictionaries feeding them, no `main()`, no `try / except ImportError` around
  plots. A loop over variants is fine; a factory is not.
- **6.11 Side work is quarantined.** A custom plot or a side analysis that earns its place is
  its own cell in a notebook, or its own block at the end of a script under one comment line.
  It never interleaves with the API lines, and the demo still reads with that cell removed.
- **6.12 Cheap rules are tests.** When a rule can be checked by an AST walk or a grep, the
  check lands with the rule (`test_teaching_imports.py`, `test_repo_contract.py`, the
  set-probe check).
- **6.13 `print` is rare; a teaching cell stays readable.** A demo *script* is the API
  sequence, then a native plot or `animate`. A notebook splits that (6.14). `print` is
  not how the student is shown what happened. Use it only when the number is the lesson
  *and* no `__str__`, `plot_*`, or `animate` already shows it. Then print the object
  (`print(solution)`, `print(report)`), or in a notebook leave the value as the last
  expression (`np.linalg.eigvals(lin.A())`, `problem.sample_x0(0, n=3)`). Markdown
  already explains; do not restate it in an f-string, and do not add a cell whose only
  job is a diagnostic dump. Never bury arithmetic, formatting, or a second construction
  inside `print` — that line is what the student reads, and it must stay a library verb.
  ```python
  # Bad — the cell is a log, not the API:
  print(f"planner.gamma = {planner.gamma}: a horizon of {DT / -np.log(planner.gamma):.1f} s")

  # Good — the native object, or the last expression:
  print(solution)
  np.linalg.eigvals(lin.A())
  ```
- **6.14 One figure per notebook cell.** In a teaching or tutorial notebook, a call that
  draws (`plot_*`, `animate`, `plot_diagram`, `plot_tree`, `scene.plot`, ...) is the only
  statement in that cell. Setup, `solve`, and `compute_trajectory` stay in the cell
  above; the figure cell is one library verb. Two slices
  (`plot_control_law(..., u_axis=0)` then `u_axis=1`) are two cells. Jupyter displays
  one figure per cell: a later `plot_*` in the same cell hides, races, or drops an
  overlay (`show=False` then `plt.show()`, stacked `animate`). A script may still plot
  after the API in one file. Apply this when writing or editing a notebook; do not
  restyle an existing dense notebook unless asked.
- **6.15 Show the mechanism once, then the tool.** Where a library verb hides a mechanism a
  student should understand (a gradient-descent loop behind `Optimizer`, a Riccati iteration behind
  `lqr`, an integration loop behind `compute_trajectory`), one teaching place writes the mechanism
  out in a few lines, then shows the one-line tool doing the same and agreeing with it; every other
  demo and notebook uses the tool. One place per mechanism, named in the doc that owns the tool
  (the closing PID example,
  `examples/demos/compile/pid_autotuning_jax.py`, for gradient descent).
- **6.16 A demo's numbers: a few knobs on top, the rest where the story needs them.** A demo
  reads like a worked example: sections in the order a textbook builds it (the world, the
  plant, its start, the law, the loop, the run, the look), each a title comment (a sentence
  naming the object, its equation when it has one), then the section's named numbers, then
  the code that builds the object, a blank line between the three. Its numbers are of two
  kinds, and both can sit in one script:
  - **Knobs**, the two or three numbers a reader is invited to change and rerun (the final
    time, the floor's grip, a cruise speed): in capitals at the top, under the imports, each
    with its unit. At most three; a demo that wants more is a research script.
  - **The narrative's numbers**, everything else: lowercase in the textbook's symbols (`mu`,
    `v_0`, the law's own `L_0`, `k_v`), each with its unit, named just before the object it
    sets. A number used once stays an inline keyword with its meaning in a comment
    (`compute_trajectory(tf=12.0, dt=0.002)`). The plant's numbers are read once, by symbol,
    under the plant; geometry and state go through the objects' methods (`path.project`,
    `path.tangent`) rather than array searches.

  ```python
  # Good — the circuit, its number and its code, apart:
  # The circuit: a rounded rectangle, and a lane 0.6 m either side of its line

  width = 8.0  # [m]

  path = from_waypoints(circuit_waypoints(length=12.0, width=width, radius=2.0), closed=True)
  track = ReferenceTrack(path, half_width=0.6)

  # Bad — a wall of capitals the reader scrolls back to:
  LENGTH, WIDTH, RADIUS = 12.0, 8.0, 2.0
  CORRIDOR_HALF_WIDTH = 0.6
  ```

  Research scripts (`examples/projects/`, benchmarks) keep whatever settings block they
  need. A demo over three knobs is listed in `tests/unittest/demo_knob_allowlist.txt`, which
  only shrinks (6.12).
- **6.17 A system is named by the dynamics it is.** A variable holding a system, in a demo,
  a notebook or the library, names what it is in the block diagram: `sys`, `plant`,
  `controller`, `observer`, `inner_loop`, `closed_loop`, `diagram`. Never the scenario it is
  run on (`lap`, `episode`, `sim`, `run`): the scenario is the run, and the run is the
  trajectory it returns (`result`, `traj`).

  ```python
  closed_loop = (mpc % dt) @ car  # Good
  lap = (mpc % dt) @ car  # Bad: a lap is what the closed loop drives, not what it is
  ```

---

## 7. Is It Worth Keeping? (Consolidation)

*Code health is maintained by reducing cognitive and maintenance burden, not by stripping features.*

- **7.1 Consolidate, never strip.** Simplification targets *maintenance cost* — text edited
  twice when code changes, dead API, boilerplate a flag would replace, NumPy/JAX twins one
  `xp` body covers, drift between twin classes. Clean, well-placed code is not a liability.
  Never remove an existing feature or public symbol without an explicit maintainer decision.
- **7.2 An intentional implementation ladder is not duplication.** Providing a clear pedagogical ladder
  (e.g., a simple loop implementation for students, a vectorized NumPy implementation for speed,
  and a JIT-compiled JAX implementation for research) is deliberate architectural design, not code duplication.
- **7.3 Strive for simplicity.** If a component has become over-engineered with helper layers and
  indirections, refactor to make it readable like a textbook chapter. Simplicity is the ultimate sophistication.
- **7.4 Prototype honestly.** Unvalidated architecture gets `TODO: User Architectural Review`.
- **7.5 Incremental refactoring.** No broad restructures unless the maintainer asks. Match the
  neighborhood; change only what the task requires.
- **7.6 Cross-link sparingly.** Every link is a maintenance edge: renames and section
  moves break them (`tests/unittest/test_repo_contract.py` checks the links of the
  governance documents, `README.md`, the `examples/`, `tests/` and
  `benchmarks/` READMEs, and `docs/plans/`). Link on first mention
  only, and only when the reader must open the target to act. Otherwise name the document in
  plain text. Documents meant to be read top to bottom (principle lists, the
  constitution) aim for zero links. A table of "related documents" inside a doc
  usually means the content is in the wrong file. Do not weave "see also" tables,
  comment pointers, or section anchors that go stale on rename.
- **7.7 Refactors that keep the numbers.** Renaming locals, naming temporaries, moving pure
  statements, and extracting helpers leave the traced computation unchanged, so results stay
  bit-identical. Reordering arithmetic (`a * b * c` into `a * (b * c)`), changing a pytree's
  structure or leaf order (a sum over leaves), changing PRNG key splits, or reordering seeded
  construction all change floating-point results even when the mathematics is equal. A
  behavior-preserving refactor proves itself with a seeded baseline (AGENTS.md).
- **7.8 One owner, but a textbook law is not a helper.** Logic written twice (identical once
  named alike) gets one owner: a shared module helper, or a base class when one class truly
  is the other. A textbook law is the exception: it stays in the body of every method that
  uses it, even when two methods share it (the PID's `f` and `ctl`), and plant equations
  stay on each plant's own page. A helper holds math only when it is a generic building
  block reused across the family or the library (`filtered_rate`, `held_integral`,
  `clipped`). No class-attribute aliases (`Cl = Plane2D.Cl`). Copies that differ in behavior
  are not merged by a refactor; name the difference and ask.
