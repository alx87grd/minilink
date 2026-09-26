# Minilink Code & Review Rules

The universal standards for writing, reviewing, and refactoring code in Minilink.
Followed identically by human maintainers, contributors, and AI agents.

Governed by [CONSTITUTION.md](CONSTITUTION.md). Agent-specific workflow instructions
(such as interaction behavior, git gates, and planning procedures) live in [AGENTS.md](AGENTS.md).

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

*An early "no" prevents expensive architectural mistakes. Identity and
boundaries: CONSTITUTION.md §1.*

- **1.1 Stay inside the product scope.** Reject work that would make Minilink an interactive GUI,
  an acausal DAE solver, a heavy multibody contact engine, or a mathematical programming language.
- **1.2 Continuous-time core is primary.** Never complicate continuous core classes
  (`DynamicSystem`, `DiagramSystem`, flow `compile()`, `Simulator`) to accommodate discrete
  conveniences. `StepSystem`, `Computer`, and hybrid diagrams are sibling utilities for
  sampled control in the loop.
- **1.3 Expose structure, never black-box dynamics.** Models make equations, state variables, and
  parameters transparent and inspectable.
- **1.4 One class for NumPy/JAX.** Write one equation path (`xp`) that runs on both backends.
  A `Jax<Plant>` twin exists only when a single class would sacrifice textbook readability.
- **1.5 Be hard where the identity is.** Guard compile-versus-reference parity, JAX twin
  agreement, and discontinuous closed-loop solvers. Soften elsewhere.

---

## 2. What Is It in Domain Terms?

*Map every concept to the physics and control domain. Tools-as-verbs and
Systems-as-descriptions: CONSTITUTION.md §4.*

- **2.1 Name by diagram role, never by implementation technology.** Class and variable names reflect
  their functional role in the block diagram (e.g., `Plant`, `ImpedanceController`, `ErrorBlock`),
  not their software mechanism (e.g., avoid names like `JittedVectorizedEvaluatorLeaf`).
- **2.2 Reserve "Leaf" strictly for diagram roles.** Standalone systems or wrapper classes must use
  descriptive domain names (e.g., `DiscretizedDynamicSystem`), reserving the term `Leaf` exclusively
  for nodes inside a diagram structure.
- **2.3 Dual parameter pattern with student tolerance:** All internal library models follow the dual
  parameter pattern: `self.params` provides the default dictionary for high-level teaching, while
  equations accept an explicit argument (`f(x, u, t, params=None)`) for computing parameter gradients
  ($\partial f / \partial p$) and system identification. However, the core framework remains forgiving:
  models where a student hard-codes constants directly into equations must simulate without friction.
  The same rule holds for costs, sets, fields and shapes: `params is None` resolves to the
  object's own defaults; any dictionary replaces them at that level.
- **2.4 No run state on a System.** A `System` does not store simulation trajectories, solver state,
  or run history. Exception: `self.traj` is the one daily-use shortcut; a new exception needs a
  reason of that weight.
- **2.5 Prefer arrays and existing core objects.** New APIs take and return arrays or types
  students already know (`System`, `Trajectory`, `PlanningProblem`, sets, costs). Do not add
  request/response dataclasses, option bags, or adapter layers that only wrap those. A new
  named record is justified when it is a domain noun (a trajectory, a certificate, a plan) —
  not when it is a programming convenience. Before adding a record, name the object it already
  is: a `(lower, upper)` pair is a `BoxSet`, a reset rule is a `Distribution`, a sublevel set
  is a `Set`, a scalar function of the state is a `Field`, a sampled run is a `Trajectory`.

---

## 3. Where Does It Live?

*Maintain strict modularity and clear dependency boundaries.*

- **3.1 Placement algorithm:**
  - A dynamic system or physical model lives under `minilink.dynamics` (or `minilink.catalog`).
  - An operation or algorithm on a system lives in its respective tool band (`minilink.simulation`,
    `minilink.analysis`, `minilink.control`, `minilink.planning`, `minilink.optimization`).
  - Unproven prototypes and research experiments live in the research lane (`examples/experimental/`
    or `examples/projects/`). Runnable example layout, naming, and promotion live in
    examples/README.md. Developer benches live under `benchmarks/`.
- **3.2 The dependency law:** Domain libraries (`control`, `analysis`, `simulation`, `planning`) may
  import only from `minilink.core` and shared mathematical bases. Peer domain libraries do not import
  each other without explicit architectural justification.
- **3.3 Two lanes (Teaching vs. Research):** Philosophy in CONSTITUTION.md §3;
  operating contract (soft entry rule, wheel scope, CI-checked imports) in ROADMAP.md §2.
  - **Teaching surface** (the root prelude and the band facades, the set ROADMAP §2 registers
    and the tests check; and the examples that import only through it: `examples/tutorial/`,
    `examples/teaching/`, `examples/demos/`): Strict public contract, high stability, fully
    documented, runs on standard scientific Python.
  - **Research lane** (everything else: the provisional hybrid, MPC, realtime and spatial
    bands, the `minilink/experimental/` tier, `examples/projects/`,
    `examples/experimental/`): Free experimentation, unconstrained by teaching stability
    guarantees; `experimental/` and the research examples are repo-only.
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
- **3.7 Intro surfaces stay canonical.** README.md, the three showcase notebooks, and
  `examples/tutorial/` present the main core tools: `System`, diagrams, simulate,
  compile, analysis, planning trajopt, the hybrid step path (`StepSystem`,
  `StepDiagramSystem`, `Computer`, `HybridDiagram`), and MPC as the hybrid exemplar.
  Do not update them to track every new demo, compare script, or research-lane
  experiment. New demos land under `examples/`. Add a README examples-table row only
  when a demo is a canonical teaching entry for a core tool.
- **3.8 Notebooks are not a second map.** The catalog is `examples/README.md` (plus
  root README entry points). Do not duplicate it with See also / Where next /
  Scripts for depth / peer hrefs.
  - **Tutorial (`00`–`11`):** at most one same-folder **Next** link, numbered
    successor. No previous, no skip-ahead, no teaching/demo/project/benchmark/doc
    hrefs.
  - **Showcases:** self-contained. No Where next / See also hubs. Chapter names may
    stay as unlinked backticks (`00_core`).
  - **Teaching:** standalone. Zero hrefs to other notebooks, demos, tutorials, or
    library source. Keep Colab self-badge, `git clone`, and external library docs
    (Gymnasium, SB3). A repo-root `[minilink](https://github.com/alx87grd/minilink)`
    link may stay. `teaching/topics/` is the textbook; `teaching/courses/<id>/`
    is a live homework pin (names in `examples/README.md`).
  - **Demos / projects:** no markdown links to notebooks. Comment twins
    (`# Teaching twin: …`) stay unless the maintainer asks to drop them.

---

## 4. What Should Its API Look Like?

*Provide clean, zero-boilerplate access for students and power for researchers.*

- **4.1 Three import layers (prefer the shortest that remains clear):**
  - **Root prelude:** `from minilink import Pendulum, lqr, Simulator` (the primary teaching surface).
  - **Band facades:** `from minilink.control import PID` or `from minilink.analysis import bode`.
  - **Defining module:** `from minilink.control.lqr import lqr` (library internals, tests,
    and the research lane).
- **4.2 Student code imports through the teaching surface.** README, tutorials, teaching
  examples, and demos import strictly via the root prelude or band facades, never through
  internal paths.
  Reader-facing imports stay light in demos; internal packages may import richly when that is
  clearer. Where a band facade does not exist yet, use the shortest import that works and
  leave the facade to a planned step — do not invent a name. *(Exception: when a factory
  name matches its module, import from the module: `from minilink.control.lqr import lqr`).*
- **4.3 Preconditions met by named adapters.** Never ask the user to write a separate model for
  different tools. A tool requiring a linear model accepts any `System` via `linearize()`; a tool
  requiring discrete stepping accepts a continuous system via a discretization adapter; a tool
  requiring a stochastic problem accepts a deterministic one via `as_stochastic()`. Convert once
  at the boundary, then use the richer object directly: never probe it with `hasattr` or
  `getattr` defaults inside the math. A tool that needs a box asks the set
  (`X.bounding_box()`); it does not `isinstance` the set. The one `isinstance` on a set is the
  lowering at the solver boundary: a singleton to an equality residual, a box to decision bounds.
- **4.4 Parameter override rule:** `params is None` resolves to the system's default parameters.
  Any provided `params` dictionary replaces them entirely (never write `params or self.params`).
- **4.5 Unconnected input ports are silent.** Unconnected inputs automatically read their nominal
  values without emitting warnings or errors.
- **4.6 Libraries are silent.** Library functions and classes must never `print()`, except when
  an explicit `verbose=True` argument is passed. Delete debug scaffolding before handoff.
- **4.7 Unified flag names.** Standardize parameter names across tools (e.g., use `verbose`,
  `solver`, `compile_backend`). Unify the *name*, not the format: `verbose=True` keeps the
  framed simulation panel.
- **4.8 Diagram connection operators and feedback topologies:** Operators provide rapid algebraic
  composition: `>>` for series cascade, `+` for parallel sum, and `@` for feedback closure.
  While `@` provides rapid synthesis for standard negative feedback loops (`controller @ plant`),
  systems support multiple feedback topologies (unity feedback, sensor dynamics, observer loops,
  MIMO architectures) composed via operator chaining or explicit wiring through `DiagramSystem.connect()`
  and `feedback()`. Those five entry points are the whole continuous wiring dialect. The sibling
  hybrid algebra adds one operator, `block % schedule`, which returns a `Computer`; it is not a
  sixth way to wire a flow diagram. Do not add a seventh.
- **4.9 Canonical port terminology:** Prefer these names when they fit; this is not a rename sweep.
  Custom and multi-channel port names remain fully valid.
  - Inputs: `u` (control action / actuation), `r` (reference / setpoint), `w` (disturbance / exogenous input), `v` (measurement noise).
  - Outputs: `y` (measured output / sensor signal).
- **4.10 Early structural validation:** Diagram connection utilities (`@`, `>>`, `DiagramSystem.connect`)
  validate port existence and dimensional compatibility at wiring time, failing with explicit
  error messages naming the mismatched ports and blocks. This rule covers wiring, not `f`/`h`
  return-shape checks at ODE time.
- **4.11 Build vs run.** Wiring, validation, and `compile()` freeze diagram structure.
  Runtime stepping must not mutate topology.
- **4.12 Defaults that change the mathematics are announced.** When a tool fills in a value
  that changes what is optimized or reported (a discount, a horizon, a tolerance), it warns
  once with `warnings.warn`, stating the consequence in domain units (an effective horizon in
  seconds, not a factor), and stays silent when the caller sets the value.

---

## 5. What Should Its Code Look Like? (Textbook Style)

*Reading Minilink source code should feel like reading an engineering textbook.*

- **5.1 Bare signatures in equation paths:** Do not put type hints inside $f, h, tf$, port
  computations, or any other native-array equation path (`Set.margin`, `CostFunction.g` / `h`,
  `Field.value`, `Shape.sdf`, `Distribution.sample`). Document expected array shapes and physical units clearly in the NumPy-style docstring.
  Type hints belong on class constructors, public methods, and tool orchestrators.
- **5.2 The `xp` idiom for hybrid NumPy/JAX:** Immediately after unpacking parameters in any
  equation path ($f$, $h$, `margin`, `g`, `value`, `sdf`, ...), bind the array module:
  ```python
  xp = array_module(x)
  ```
  Write the subsequent mathematical algebra using `xp` so the exact same equation path executes
  on both NumPy and JAX arrays without branching.
- **5.3 Three beats in an equation path; no `self.` in math lines:** Every native-array
  equation method (`f`, `h`, `g`, `margin`, `value`, `sdf`, `forward_dynamics`, …) and
  every algorithm body (analysis tools, design functions, planners, solvers, learning
  updates) is three beats, with a blank line between them so the core equation is the thing the
  eye lands on:

  1. **Unpack.** Bind `params`, split `x`, and copy `self.` fields into short textbook
     names. No `self.` remains in the algebra.
  2. **Core math.** The equation as a **named assignment**. A short textbook
     comment belongs here only when it is not a copy of the code (rule 5.22).
     The equation is never on the `return` line.
  3. **Output machinery.** Stack, wrap, or `return` the named result. This beat is
     plumbing, not the lesson.

  ```python
  # Good — the comment is the implicit form; the code is the solved assignment:
  params = self.params if params is None else params
  m, l, g, d = params["m"], params["l"], params["g"], params["d"]
  xp = array_module(x)
  theta, omega = x
  tau = u[0]

  # (m l²) ω̇ = τ − m g l sin(θ) − d ω
  dtheta = omega
  domega = (tau - m * g * l * xp.sin(theta) - d * omega) / (m * l**2)

  return xp.array([dtheta, domega])

  # Good — the assignment is already the textbook; no copy-comment:
  dx = A @ x + B @ u

  return dx

  # Bad — self. in the algebra, and the equation lives on the return line:
  return (u - self.m * self.g * self.l * xp.sin(x[0])) / (self.m * self.l**2)

  # Bad — a textbook step hidden in a helper: the SVD, rank and basis of the
  # Kalman decomposition are the lesson, so they belong in the body:
  T_c = _orthonormal_range(ctrb, tol)

  # Good — each step a named line; only the default tolerance is plumbing:
  # 𝒞 = U Σ Vᵀ, keep the r directions with σᵢ > tol·σ₁
  U, sigma, _ = np.linalg.svd(ctrb)
  r = int(np.sum(sigma > _rank_tol(ctrb, sigma, tol)))
  T_c = U[:, :r]
  ```
- **5.4 Mathematical naming conventions:**
  - Matrices: uppercase (`A`, `B`, `C`, `D`, `H`, `M`, `K`).
  - Vectors: lowercase (`x`, `u`, `y`, `q`, `v`, `dq`, `dx`).
  - Dimensions: lowercase integers (`n`, `m`, `p`).
  - A symbol keeps one meaning within an object, and one meaning across the codebase wherever
    the literature allows: `A` is the state matrix, `rho` the discount rate, `z` the policy
    features. Name a local that would collide by its role (`advantage`, `ratio`, `eps`)
    rather than reusing the symbol. The textbook's own overloads across objects (`h`: output
    map on a `System`, terminal cost on a `CostFunction`; `g`: running cost, gravity vector,
    NLP inequality; `B`, `C`: mechanical vs state-space) are inherited, listed in DESIGN §4 as
    a closed list, and never added to.
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
- **5.8 No leading-underscore pseudo-privacy on public classes:** Do not mark methods with a leading
  underscore (`_`) on `System`, `Facade`, or `Simulator` classes. Separate public interfaces from
  internal machinery using standard file section comments (`# Public API` and `# Internal machinery`).
  Reserve leading underscores for module-level private constants and purely local closures.
- **5.9 The first-screen rule:** The module docstring and primary class definition must appear within
  the first screen (~50 lines). Place complex validation and helper machinery below.
- **5.10 Standard module section order:**
  1. Module docstring & imports
  2. Primary class / public contract
  3. Subclasses & specialized variants
  4. Public functions & convenience facades
  5. `# Internal machinery` (helpers, evaluators, validation)
- **5.11 Selector-orchestrator split:** Public math tools follow a clear pattern: inspect inputs
  $\to$ select algorithm backend $\to$ extract $f/h$ callables $\to$ execute core math in place.
  Heavy ceremony and validation belong in helper functions below.
- **5.12 Backend imports come from `minilink.core.backends`:** Never import from `minilink.core.compile`
  inside the system libraries (`blocks/`, `dynamics/`, `control/`, `estimation/`). Use
  `require_jax_numpy()`, `array_module()`, and `require_scipy()` from `core.backends`. Tools that
  compile a system for a living (`simulation/`, `analysis/`) import the compiler directly. A
  JAX-only function opens with one binding line, `jax, jnp = require_jax(), require_jax_numpy()`,
  never a bare `import jax`; the lazy-import rule (5.20) keeps that line inside the function.
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
  (`core/system.py`, `blocks/`, `dynamics/`, `control/`) read like a textbook. Library-developer
  modules (`core/compile/`, evaluators) may carry compiler machinery.
- **5.17 Native-array equation paths:** Keep every equation path (the list of 5.1) on native
  arrays, traceable in every argument, `params` included; convert at API boundaries only.
- **5.18 `__main__` hello-worlds:** Core modules may ship a `__main__` smoke that constructs the
  class and runs it once. Keep it short enough to read at a glance, and stop where the teaching
  starts: a smoke that grows plots, sweeps, or commentary has become a demo and belongs under
  `examples/`. Do not add plant-only demo scripts whose only job is to smoke a catalog class.
- **5.19 JAX float64 by default:** JAX evaluators enable 64-bit floats on construction;
  `MINILINK_JAX_X64=0` opts out. Tools never require the caller to call `configure_jax` first.
- **5.20 Match the neighborhood:** Change only what the task requires. Public APIs use type hints
  and NumPy docstrings except in equation paths (rule 5.1). Lazy optional imports. Prefer a low
  helper count in math tools; inline single-use helpers.
- **5.21 Validation in proportion:** Validate at boundaries, not in every helper. Frozen dataclasses
  for value objects — a `Trajectory`, a set, a shape, a certificate, a solver record; plain
  classes with `params` for parametric equation objects — systems, costs. Never dataclasses as
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
  docstring. Apply this to new math; when restyling an existing path (only if the
  maintainer asks), use the three beats of rule 5.3.
- **5.23 No preamble walls.** A module or demo opens with a one-line title docstring. Do not
  add a long introduction, section map, run recipe, or flag explanation at the top — the
  code plus inline comments must tell the story. Notebooks are course material: do not
  trim their markdown unless asked.

---

## 6. What Evidence Does It Owe?

*Trust in the toolbox is built on verifiable evidence, clean demos, and robust tests.*

- **6.1 A demo is the API, and nothing else.** *Simplicity is the ultimate sophistication*
  holds for a demo or a teaching notebook as it does for the core: the shortest sequence of
  library verbs that tells the story — build, compose, solve, evaluate, plot, animate — run
  from the top level, with constants at the top, a one-line title docstring and the story in
  short inline comments (5.22–5.23). Less is better. Reporting goes through the objects
  themselves: their `print` (`__str__`) and their native plots (`plot_trajectory`,
  `plot_solution`, `plot_control_law`, `plot_cost2go`, `plot_learning_curve`, `plot_tree`,
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
- **6.8 Demo-gate maturity.** Nothing enters the teaching surface without a demo or notebook,
  a both-backends test where it defines dynamics, and a docstring (ROADMAP.md §2).
- **6.9 Public-facing prose is foundational, not a bake-off.** README, the three showcases,
  and the docs landing page stay positive about minilink and never name
  other tools. Framing: minilink bridges capabilities that usually live in separate tools.
  No superlatives; quote measured notebook batches, never a per-call speedup. The main
  line is readable by an undergraduate; expert depth sits in short "under the hood"
  asides. GIF assets stay under 1 MB
  and use catalog plant framing (the MPC clip may follow the car). Before pushing those
  files, `grep -rniE "simulink|matlab|drake|casadi|mujoco"` over README and the
  showcase notebook markdown must be empty (the `-E` matters: without it the alternation is
  literal and the gate passes on anything). `test_repo_contract.py` runs the same check.
- **6.10 Flat demos.** No functions or classes in a demo or a teaching notebook except the
  model or cost it is about (its `f`, `h`, `g`) and a step the notebook's text teaches by hand
  (a hand-written RK4, a policy-gradient update). No wiring, tuning or plotting helpers, no
  configuration dictionaries feeding them, no `main()`, no `try / except ImportError` around
  plots (6.2). A loop over variants is fine; a factory is not.
- **6.11 Side work is quarantined.** A custom plot or a side analysis that earns its place is
  its own cell in a notebook, or its own block at the end of a script under one comment line.
  It never interleaves with the API lines, and the demo still reads with that cell removed.
- **6.12 Cheap rules are tests.** When a rule can be checked by an AST walk or a grep, the
  check lands with the rule (`test_teaching_imports.py`, `test_repo_contract.py`, the
  set-probe and flat-demo checks).
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
  # Bad — unreadable; the cell is a log, not the API:
  print(f"planner.gamma = {planner.gamma}: a horizon of {DT / -np.log(planner.gamma):.1f} s")
  default = ReinforcementLearningPlanner(problem, dt=DT, hidden=(8,), verbose=False)
  print(f"default gamma = {default.gamma}: a horizon of {DT / -np.log(default.gamma):.1f} s")

  # Bad — a plot or __str__ already said this:
  print("the five worst of", report.J.size, "trials:", np.round(np.sort(report.J)[-5:], 2))
  print("open-loop poles:", np.round(np.linalg.eigvals(plant.linearize(xbar).A()), 2))

  # Good — native object, or the last expression:
  print(solution)
  print(report)
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
  governance documents and `docs/plans/`, nothing else). Link on first mention only, and
  only when the reader must open the target to act. Otherwise name the document in
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
