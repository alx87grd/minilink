# Textbook pass, second run (2026-10-16)

The second run of the textbook pass ("code reads like a textbook"), from `dev` at 8b4c4d5, on
branch `claude/textbook-pass-dev` (PR #125). The approved plan (`docs/plans/textbook-pass.md`)
and its worked examples (`docs/plans/textbook-pass-examples.md`) are deleted with this record,
as the plan asked. Both are in the branch's history at 9779c93, and the style sheet they
settled is the appendix below. This record is history, not a backlog: the open items it
lists live on in the PR and in `docs/plans/TODO.md`.

## What ran

| Wave | Scope | Modules | Rewritten | Already at the bar | Other |
| --- | --- | ---: | ---: | ---: | --- |
| 0 | the ratchet, the docs hook, the probes, the baseline, the reference pages `dp.py` and `control/design.py` | 2 | 2 | 0 | |
| 1 | `core/` (T1), the state-feedback lane (TB-b) | 23 | 23 | 0 | three modules re-committed after a tool fix |
| 2 | `blocks/`, `control/`, `estimation/` (T2); `dynamics/` (T3); `analysis/`, `simulation/` (T4) | 84 | 73 | 11 | |
| 3 | `optimization/`, `planning/` (T5) | 74 | 66 | 8 | |
| 4 | `core/composition.py` and the MPC controller (T6); `graphical/` (T7) | 41 | 40 | 1 | the MPC controller reverted at review, then passed on a retry (below) |
| 5 | the library-like modules of `examples/projects/`, the package `__init__` modules | 18 | 17 | 0 | `_version.py` is generated at build time and untracked |

The ratchet's allowlist (`tests/unittest/textbook_style_allowlist.txt`) shrank from 927 rows
on `dev` to 24 deliberate rows. Eight are the cases the sheet keeps. The other sixteen are the MPC controller's documented extension hooks, which keep their underscore.

## How it was checked

- **One gate per commit** (`tools/commit.sh` on the probe branch), in this order:
  - ruff;
  - the tests that import the module;
  - seeded probes, byte-identical to a baseline captured twice at the pinned base (7,253 probes in 20 bands);
  - an audit of literals, raises, attribute strings, signatures and `__all__`, each difference explained in the commit message;
  - the ratchet.
- **Each wave's end:**
  - the lanes were cherry-picked onto the run branch;
  - the ripple aliases were resolved in one commit;
  - the allowlist was regenerated;
  - a read-only reviewer read the merged diff against the sheet, and its style findings landed in one review commit.
- **Verification on the run branch:**
  - the full probe set, identical after wave 1;
  - the full suite: 1,546 passed. The one failure was a flagship demo timing out under load; it takes 59 s alone on both the branch and `dev`;
  - GitHub CI green on every pushed wave head.
- **The probes and tools** live on the unmerged branch `claude/textbook-pass-probes`.

## Decisions taken during the night

These were taken under the plan's latitude. Each can be reverted or revisited.

- **Process.** Writers took batches of up to six modules, not one module each. Each wave's lanes pulled from one shared queue, so the lanes stayed balanced. A wave's reviewer ran alongside the next wave, and its fixes landed at the next merge.
- **Tool fixes, found by the writers:**
  - When no test imports a module, the gate used to run the whole suite.
  - Probe messages carried lane paths and line numbers; they are now scrubbed.
  - Ruff saw non-Python files.
  - The ratchet now lets one comment give its reason to a block of consecutive lazy imports. A writer asked for this in wave 1, and the wave-2 review merged repeated comments.
- **The probes call the dry run's final names.** `renames.tsv` maps each one to the name the code has, so a writer's rename keeps the probes running.
- **`control/mpc/controller.py` was reverted at the wave-4 review.** The pass had given the public `ModelPredictiveControllerMixin` an `__init__` and renamed its documented subclass hooks (`_z_warm_for_command`, `_replan_k`, `_compute_*`). A subclass written to `dev`'s docstring would have broken; the example `mpc_dual_rate.py` did, and the probes caught it. The module was reverted, then passed again with the contract frozen: the mixin has no `__init__`, and the extension surface (the hooks and the attributes a subclass provides) keeps its names. Whether that surface should move to plain names is left to you.
- **One wording per kind of lazy import, and one docstring form for refusals:**
  - `# A real cycle: …`
  - `# Lazy import: the graphics load on first use, not with the core`
  - `# Tools import core (DESIGN §3): …`
  - `require_*` docstrings open with "Refuse"; a `check_*` returns nothing.
  - Display hooks sit under `# Display`.

## Removals (plan §1)

- `PID.command`, `ImpedanceIntegralController.command`, `SlidingModeController.sliding_torque`, and the project servo's `Servo.compute_cmd`: each law now lives in its `f`/`ctl` bodies (the approved PID example). None had a caller outside its module.
- `analysis.lyapunov.lyapunov_matrix` and `control.robotic.impedance_joint_torque`: plain-named helpers under `dev`'s Internal machinery, each with one caller, inlined as textbook laws.
- Private code:
  - `_configure_cartpole`;
  - `_collect_signal_names`;
  - the dead `plant_signals` branch;
  - `JaxMathematicalProgramEvaluator._compile_vector_derivative`, plus two unreachable branches of the JAX evaluator;
  - `StateSpaceGrid._validity_masks`;
  - the project servos' `_unpack`;
  - three identical overrides in the project vehicle.

## Open questions and recorded bugs

They are listed in full in the PR description (#125), by wave. The code stays as on `dev` for each one. The main families:

- **Public names outside the verb families.** The `validate_*` methods, `check_algebraic_loops`, `observer_of`, `reject_unknown_online_params` and `validate_overlay` all keep their names, because they are public.
- **One owner across modules.**
  - Planning: `maybe_show`, `check_derivative_shapes`, the parametric evaluator as a `MathematicalProgramEvaluator`, the schedule coercion, `require_one_of`, `format_axis_label`.
  - The project helpers copied between the path-tracking projects.
  - Each stays duplicated, because merging it would change behaviour or what a module loads.
- **Shadow state left for an API decision.** `wiring.init_wiring`, `ErrorDriven.add_error_ports`, and the MPC exports' writes to the block.
- **The recurring bug family.** Plants' `tf` and `get_dynamic_geometry`, and the arms' `J`, read `self.params` and ignore their `params` argument, which breaks `tf(x, u, t; p)` of CONSTITUTION §2.
- **Bugs recorded per module, never fixed at night.** For example:
  - RK4 on a non-uniform grid;
  - `ReinforcementLearningPlanner.predict` ignoring `deterministic`;
  - `PolylinePath.sample` on a vector of arc lengths;
  - `draw_arrows` raising for 3–6 samples;
  - the MPC tick latch memoizing on `(k, y)` only;
  - the Pacejka combined-slip term in the project vehicle.

## Appendix: the style sheet as applied

### 3.1 Module shape
- Line 1 is a one-line title naming the domain, not the process:
  `"""Discretization: a continuous-time system as a discrete-time step model."""`.
  A longer note moves to the class or function whose contract it describes (5.23).
- The order: imports; the module constants the primary code reads, next to their owner's
  section; `# Public API`; the primary class or function first, within the first 60 lines
  (5.9); then `# Internal machinery`, with the helpers in the order the body calls them
  (5.10). A class with helpers takes its own `# Internal machinery` title after its public
  methods.
- **The two section titles stand out.** `Public API` and `Internal machinery` sit between
  two full lines of `#`, with two blank lines around them. Inside a class, the banner is
  indented with the class body. Smaller sub-sections are plain sentence-case comments
  (`# Frequency grid`).

  ```python
  _2PI = 2.0 * math.pi


  ###############################################################################
  # Public API
  ###############################################################################


  def shortest_path(q0, q1, radius):
      ...


  ###############################################################################
  # Internal machinery
  ###############################################################################


  def word_segments(a, b, d):
      ...
  ```
- **Air:** blocks are separated by blank lines, never packed together.
- A constant has one owner; the other modules import it. There is no lazy import of a
  minilink module, except to break a real cycle or for an optional dependency, with a
  comment saying which.

### 3.2 Equation paths and algorithms (5.3)
- **One pattern for every equation path and algorithm body.** The blocks below appear in
  this order, each separated by a blank line, and any block with nothing to say is left out:
  1. `params = self.params if params is None else params`, then on the next line the
     backend: `xp = array_module(...)` (or `jnp = require_jax_numpy()` in a JAX-only
     algorithm).
  2. The parameters and the object's fields, **one per line**.
  3. The pre-computation: the state and input unpacked (`theta = q[0]`), trigonometric
     shorthands, intermediate quantities.
  4. The core equation(s), each a named line, with its equation comment above when the
     code cannot write the textbook form (5.22).
  5. The matrix assembly, when the result is a complex array (below).
  6. `return` of a name.
- **Assembly.**
  - A simple array (a vector or a matrix with short entries) is built on its own equation
    line: `g = xp.array([m * gravity * l * xp.sin(theta)])`.
  - A complex array gets one named line per entry (`J11`, `J12`, …) in block 4, then its own
    assembly block, a matrix row by row (5.5).
  - The array is never built on the `return` line.
- No `self.` in a math line. Linear algebra is part of the story and stays in the body.
  Plumbing goes to helpers: coercion, shapes, defaults, validation, dispatch, caching,
  wrapping.
- **Checks.** An input check (`require_*`) comes right after the parameter block. A check on
  a computed quantity sits right after the line computing it (`ctrb = …` then
  `require_controllable(ctrb, n)`). An inline `raise` in a body becomes a `require_*`
  helper, with the same exception type and message.
- **Numbers.** A numeric constant stays inline where the textbook writes it, with its meaning
  in the equation comment (`+ 1e-8`, `10⁻⁸`). It becomes a named module constant only when
  two places share it or it is a tuning knob (`RANK_TOL`).
- **Return.** Never an equation or an array on the `return` line. A result object built from
  named results may be (`return DynamicProgrammingResult(grid=..., J=J, ...)`).

  ```python
  def g(self, q, params=None):
      params = self.params if params is None else params
      xp = array_module(q)

      m = params["m"]
      l = params["l"]
      gravity = params["gravity"]

      theta = q[0]

      g = xp.array([m * gravity * l * xp.sin(theta)])

      return g


  def C(self, q, dq, params=None):
      params = self.params if params is None else params
      xp = array_module(q, dq)

      m2 = params["m2"]
      l1 = params["l1"]
      lc2 = params["lc2"]

      _, _, _, s2, _, _ = self.trig(q)
      h = m2 * l1 * lc2 * s2

      # Coriolis and centrifugal coupling, driven by the second joint's rate
      C11 = -h * dq[1]
      C12 = -h * (dq[0] + dq[1])
      C21 = h * dq[0]
      C22 = 0.0

      # fmt: off
      C = xp.array([
          [C11, C12],
          [C21, C22],
      ])
      # fmt: on

      return C
  ```

  The constructor keeps the array module it has on `dev` (`np` or `xp`): no backend change
  (§1). On `dev` the arms' `J` reads `self.params` instead of its argument; that is a
  recorded bug, so the pass keeps the `self.params` reads exactly as they are.
- A delegation, a constant, a dict literal or a frame composition stays the one line it
  is; the beats are for equations. `tf` returns its frames as one dict literal.
- Equation comments use Unicode math in the textbook's symbols: `ẋ`, `x_{k+1}`, `Aᵀ`, `R⁻¹`,
  `σᵢ`, `≤`, `√`, `∏`, `𝒞`, `𝒪`. Identifiers stay ASCII (`x_next`, `theta`, `sigma`). A
  comment states the equation; it never copies the code line.

### 3.3 Constructors, verbs, loops and classes
- **A constructor** reads as named blocks, each opened by a short capitalized comment:
  `super().__init__` and the name, then `# Parameters`, `# Ports`, `# Signal labels and
  units`, `# Graphics`. A value worked out by a helper is returned by that helper and
  assigned in `__init__`; no helper sets attributes on the object (5.7). Every attribute
  starts in `__init__`, `None` when only one branch fills it.
- **A public verb** (`linearize`, `bode`, `lqr`, `simulate`) reads, in order:
  1. its input checks;
  2. its setup (selectors, the point, the options);
  3. its math blocks, each with its equation comment;
  4. its result.

  The option plumbing goes to `*_of` and `require_*` helpers. A public verb keeps its NumPy
  docstring sections.
- **An iterative algorithm** (an RRT search, a DP sweep, a training loop) shows its steps
  as commented blocks inside the loop: sample, extend, add, test the goal. Logging,
  recording, callbacks and live plots are one call per iteration to a helper (`log.sweep(…)`,
  `self.on_search_step(…)`), never inline in the loop body.
- **A class** reads: the docstring, `__init__`, the properties, the public methods (the
  primary verb first), then the indented `Internal machinery` banner and the helpers, in the
  order the public methods call them.

### 3.4 Names
- Plain names everywhere (5.8, module functions included). Underscores only for private
  module constants and local closures.
- One verb family per role:
  - `require_*` raises when a precondition fails;
  - `check_*` validates a shape and returns nothing;
  - `build_*` constructs a runtime object (evaluator, simulator, computer);
  - `*_of` resolves an object's options or attributes (`trajopt_options_of`);
  - `plot_*` draws.

  Twin helpers in sibling modules share one name.
- Textbook symbols, one meaning each:
  - `rho` is the discount rate; a probability ratio is `ratio`.
  - `tau` is a torque in mechanics and a time constant in filters, never both in one module.
  - `eta` is a step size, `eta_k` its value per update.
  - Losses are written `L_pi`, `L_V`.

### 3.5 Comments and docstrings
- Timeless (5.24): no "now", "was", dates or step ids in the code.
- A prose comment starts with a capital letter, as in `dp.py` (`# Coupled inertia of the two
  planar links`). A math comment starts with the equation itself (`# L_clip = −E[…]`).
- No comment restates the line below it. A comment says why, or states the equation.
- A public function's docstring opens with one summary line. NumPy sections appear only
  where the parameters need them. A note on a contract lives on the object that owns it.

### 3.6 Types and imports
- Under `from __future__ import annotations`, forward references are unquoted.
- No type hints in equation paths (5.1). Hints go on constructors, public methods and
  orchestrators.
- `require_jax()` / `require_jax_numpy()`, never a bare `import jax` (5.12).

### 3.7 Backends
- `xp = array_module(...)` is the second line (§3.2), and every array operation after it
  uses `xp` (5.2), **unless the change would alter a result's backend type**. No
  backend changes (§1): an `np.` call under `xp` that decides the returned type stays,
  recorded and allowlisted.

### 3.8 One owner
- Logic written twice (identical once named alike) gets one owner. The owner is either a
  shared module helper, or a base class when one class truly is the other (the parametric
  evaluator is a `MathematicalProgramEvaluator`).
- **A textbook law is not a helper.** The law stays in the body of every method that uses
  it, even when two methods share it (the PID's `f` and `ctl`). A helper holds math only
  when it is a generic building block reused across the family or the library (the
  classical family's `filtered_rate`, `held_integral`, `clipped`).
- No class-attribute aliases (`Cl = Plane2D.Cl`).
- Plant equations stay on each plant's own page, even when two plants share them.
- Copies that differ in behaviour stay as they are, and the record names the difference.

### 3.9 What does not change
- Results, the backend type of every result, refusals (exception type and message),
  warnings, the order of side effects, defaults, public names and signatures. Removals are
  allowed only under §1, each one listed.
- The maintainer's manual edits in `examples/`, notebooks and scratch files. A research-lane
  script takes renames, markers and titles only.

### 3.10 The reference pages
- Algorithm: `planning/policy_synthesis/dp.py`. Design verb: `control/design.py`. Both are
  brought to this sheet first (wave 0), so that no writer reads a reference that contradicts
  it.
- The worked before/after examples: `docs/plans/textbook-pass-examples.md` (at 9779c93),
  given to every writer with this section.
- From the dry run, one example per kind (paths on `claude/vigilant-hopper-poekls`):
  - a plant: `dynamics/catalog/pendulum/pendulum.py`;
  - an evaluator: `planning/policy_synthesis/policy_eval.py`;
  - an orchestrator: `simulation/simulator.py`;
  - a plot module: `graphical/port_map.py`.

### 3.11 Clarifications settled during the run
- A lazy minilink import names its reason with one of:
  - `# A real cycle: <who imports whom>`;
  - `# Lazy import: the graphics load on first use, not with the core`;
  - `# Tools import core (DESIGN §3): the <band> loads on first use`;
  - `# Lazy import: the compiler loads on first use, not with the analysis verbs`.

  One comment covers a block of consecutive lazy imports. An import whose hoisting would
  change what a module loads stays lazy.
- A `require_*` docstring opens with "Refuse". A `check_*` returns nothing; a check that returns
  a value is a `require_*` when it refuses, or an `*_of` when it resolves.
- `__str__` and `_repr_pretty_` sit under a `# Display` sub-section.
- Every prose comment starts with a capital, including inline and dict group comments.
- Twin helpers share one name. Before naming a new `require_`/`check_` helper, look for an
  existing one doing the same check.
- A documented extension surface (the hooks a subclass overrides, such as the MPC mixin's
  `_z_warm_for_command`) keeps its names; the plain-name rule does not apply to it.
