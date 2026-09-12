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
5. [What should its code look like? (Textbook Engineering Style)](#5-what-should-its-code-look-like)
6. [What evidence does it owe? (Demos, Verification & Tests)](#6-what-evidence-does-it-owe)
7. [Is it worth keeping? (Lifecycle & Consolidation)](#7-is-it-worth-keeping)

---

## 1. Should It Exist in Minilink?

*An early "no" prevents expensive architectural mistakes.*

- **1.1 Stay inside the product scope.** Minilink is a code-first, causal, differentiable block-diagram
  toolbox for dynamical systems. It is not an interactive GUI, an acausal DAE solver, a heavy
  multibody contact physics engine, or a mathematical programming language.
- **1.2 Continuous-time core is primary.** The continuous dynamical model $\dot{x} = f(x, u, t; p)$
  is the foundation. Discrete components (`StepSystem`, `Computer`, hybrid diagrams) are subsidiary
  utilities for discrete control in the loop (e.g., sampled MPC or digital filtering). Never
  complicate continuous core classes (`DynamicSystem`, `DiagramSystem`, `Simulator`) to accommodate
  discrete conveniences.
- **1.3 Expose structure, never black-box dynamics.** Models make equations, state variables, and
  parameters transparent and inspectable. Do not bury physical dynamics inside closed, opaque wrappers.

---

## 2. What Is It in Domain Terms?

*Map every concept directly to the physics and control domain.*

- **2.1 Tools are verbs on a System.** Simulators, optimizers, linearizers, and plotting utilities
  operate on `System` objects and return data, systems, or figures. Tools *never* define dynamics.
- **2.2 Systems are descriptions; facades are shortcuts.** A `System` stores equations ($f, h, tf$),
  ports, parameters, and nominal states. It does not store simulation trajectories, solver state, or
  run history. (Exception: `self.traj` is reserved as a convenient shortcut on `System`).
- **2.3 Name by diagram role, never by implementation technology.** Class and variable names reflect
  their functional role in the block diagram (e.g., `Plant`, `ImpedanceController`, `ErrorBlock`),
  not their software mechanism (e.g., avoid names like `JittedVectorizedEvaluatorLeaf`).
- **2.4 Reserve "Leaf" strictly for diagram roles.** Standalone systems or wrapper classes must use
  descriptive domain names (e.g., `DiscretizedDynamicSystem`), reserving the term `Leaf` exclusively
  for nodes inside a diagram structure.
- **2.5 Dual parameter pattern with student tolerance:** All internal library models follow the dual
  parameter pattern: `self.params` provides the default dictionary for high-level teaching, while
  equations accept an explicit argument (`f(x, u, t, params=None)`) for computing parameter gradients
  ($\partial f / \partial p$) and system identification. However, the core framework remains forgiving:
  models where a student hard-codes constants directly into equations must simulate without friction.

---

## 3. Where Does It Live?

*Maintain strict modularity and clear dependency boundaries.*

- **3.1 Placement algorithm:**
  - A dynamic system or physical model lives under `minilink.dynamics` (or `minilink.catalog`).
  - An operation or algorithm on a system lives in its respective tool band (`minilink.simulation`,
    `minilink.analysis`, `minilink.control`, `minilink.planning`, `minilink.optimization`).
  - Unproven prototypes and research experiments live in the research lane (`examples/experimental/`
    or `examples/projects/`).
- **3.2 The dependency law:** Domain libraries (`control`, `analysis`, `simulation`, `planning`) may
  import only from `minilink.core` and shared mathematical bases. Peer domain libraries do not import
  each other without explicit architectural justification.
- **3.3 Two lanes (Teaching vs. Research):**
  - **Teaching surface** (`minilink/`, `examples/tutorial/`, `examples/teaching/`, `examples/demos/`):
    Strict public contract, high stability, fully documented, runs on standard scientific Python.
  - **Research lane** (`examples/projects/`, `examples/experimental/`): Free experimentation,
    repo-only, unconstrained by teaching stability guarantees.
- **3.4 Clean renaming:** When renaming a component or module, update all call sites across the
  entire repository in the same change. Do not leave deprecated aliases behind.
- **3.5 Decoupled rendering & headless dynamics:** Classes in `minilink.dynamics` must never import
  graphical rendering libraries (Matplotlib, Pygame, Meshcat). Physical geometries are declared using
  lightweight declarative primitives (`Circle`, `Rod`), and rendering engines are loaded lazily by
  user scripts or simulator wrappers, ensuring dynamics remains 100% headless.

---

## 4. What Should Its API Look Like?

*Provide clean, zero-boilerplate access for students and power for researchers.*

- **4.1 Three import layers (prefer the shortest that remains clear):**
  - **Root prelude:** `from minilink import Pendulum, lqr, Simulator` (the primary teaching surface).
  - **Band facades:** `from minilink.control import lqr` or `from minilink.analysis import bode`.
  - **Defining module:** `from minilink.control.lqr import lqr` (used by library internals and tests).
- **4.2 Student code imports through the teaching surface.** Tutorials, teaching examples, and
  demos import strictly via the root prelude or band facades, never through internal paths.
  *(Exception: when a factory name matches its module, import from the module: `from minilink.control.lqr import lqr`).*
- **4.3 Preconditions met by named adapters.** Never ask the user to write a separate model for
  different tools. A tool requiring a linear model accepts any `System` via `linearize()`; a tool
  requiring discrete stepping accepts a continuous system via a discretization adapter.
- **4.4 Parameter override rule:** `params is None` resolves to the system's default parameters.
  Any provided `params` dictionary replaces them entirely (never write `params or self.params`).
- **4.5 Unconnected input ports are silent.** Unconnected inputs automatically read their nominal
  values without emitting warnings or errors.
- **4.6 Libraries are silent.** Library functions and classes must never `print()`, except when
  an explicit `verbose=True` argument is passed.
- **4.7 Unified flag names.** Standardize parameter names across tools (e.g., use `verbose`,
  `solver`, `compile_backend`), even if internal output formatting differs.
- **4.8 Diagram connection operators and feedback topologies:** Operators provide rapid algebraic
  composition: `>>` for series cascade, `+` for parallel sum, and `@` for feedback closure.
  While `@` provides rapid synthesis for standard negative feedback loops (`controller @ plant`),
  systems support multiple feedback topologies (unity feedback, sensor dynamics, observer loops,
  MIMO architectures) composed via operator chaining or explicit wiring through `DiagramSystem.connect()`.
- **4.9 Canonical port terminology:** Standard dynamical systems and control blocks adopt canonical
  port nomenclature where applicable:
  - Inputs: `u` (control action / actuation), `r` (reference / setpoint), `w` (disturbance / exogenous input), `v` (measurement noise).
  - Outputs: `y` (measured output / sensor signal).
  Custom and multi-channel port names remain fully supported for specialized systems.
- **4.10 Early structural validation:** Diagram connection utilities (`@`, `>>`, `DiagramSystem.connect`)
  validate port existence and dimensional compatibility upon construction, failing early with explicit
  error messages naming the mismatched ports and blocks, rather than deferring failures to ODE integration steps.

---

## 5. What Should Its Code Look Like? (Textbook Style)

*Reading Minilink source code should feel like reading an engineering textbook.*

- **5.1 Bare signatures in equation paths:** Do not put type hints inside $f, h, tf$ or port
  computations. Document expected array shapes and physical units clearly in the NumPy-style docstring.
  Type hints belong on class constructors, public methods, and tool orchestrators.
- **5.2 The `xp` idiom for hybrid NumPy/JAX:** Immediately after unpacking parameters in $f$ or $h$,
  bind the array module:
  ```python
  xp = array_module(x)
  ```
  Write the subsequent mathematical algebra using `xp` so the exact same equation path executes
  on both NumPy and JAX arrays without branching.
- **5.3 Unpack parameters before equations; no `self.` in math lines:** Bind parameters to local
  variables first. Core equations must read as pure mathematics:
  ```python
  # Good:
  m, l, g, d = params["m"], params["l"], params["g"], params["d"]
  dtheta = omega
  domega = (u - m * g * l * xp.sin(theta) - d * omega) / (m * l**2)
  
  # Bad:
  domega = (u - self.m * self.g * self.l * xp.sin(x[0])) / (self.m * self.l**2)
  ```
- **5.4 Mathematical naming conventions:**
  - Matrices: uppercase (`A`, `B`, `C`, `D`, `H`, `M`, `K`).
  - Vectors: lowercase (`x`, `u`, `y`, `q`, `v`, `dq`, `dx`).
  - Dimensions: lowercase integers (`n`, `m`, `p`).
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
- **5.7 No shadow state:** Initialize all object attributes explicitly in `__init__`. Never dynamically
  attach attributes at call sites or rely on `hasattr`-and-create patterns.
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
  inside system libraries. Use `require_jax_numpy()`, `array_module()`, and `require_scipy()` from
  `core.backends`.
- **5.13 Familiar patterns first:** Favor familiar, idiomatic Python patterns (mixins, composition,
  duck typing) over esoteric constructs (e.g., custom metaclasses or complex generics) unless there
  is a compelling runtime requirement.
- **5.14 Standard SI units and radian convention:** All angles are in radians; all angular velocities
  in rad/s. State spaces keep unbounded continuous angle representations by default unless explicit
  manifold wrapping is declared. Units are strictly standard SI (kg, m, s, N, N·m).
- **5.15 JAX-compiled branchless algebra vs. student clarity:** Core catalog plants, trajectory
  optimization objectives, and controllers intended for JAX compilation or autodiff must avoid scalar
  Python conditionals (`if x > 0:`) in $f$ and $h$, using vectorizable operations (`xp.where`,
  `xp.clip`, `xp.maximum`). However, outside of JAX-compiled paths—such as in student models or
  non-performance analytical paths—standard Python `if` branching is fully supported and welcomed
  if it enhances pedagogical clarity.

---

## 6. What Evidence Does It Owe?

*Trust in the toolbox is built on verifiable evidence, clean demos, and robust tests.*

- **6.1 Demos are open-and-run scripts:** Demo scripts in `examples/demos/` must run from the top
  level without requiring a `main()` function wrapper. They feature a clear one-line title docstring
  and tell their pedagogical story through inline comments.
- **6.2 No test harness code in demos:** Never add test environment branches (`if CI: ...`), mock
  flags, or headless switches inside user-facing demos. Test runners adapt from the outside
  (e.g., by setting `MPLBACKEND=Agg` or setting timeouts).
- **6.3 Tests guard contracts, not implementation trivia:** Write tests for stable mathematical
  invariants, public APIs, boundary conditions, and cross-backend parity (NumPy vs. JAX). Do not
  write brittle tests that assert internal helper implementation details.
- **6.4 One parameterized test over many duplicates:** Consolidate related test configurations into
  parameterized fixtures rather than proliferating near-duplicate test files.
- **6.5 Guard optional dependencies:** Third-party dependencies (JAX, Pygame, Meshcat) must be
  lazily checked and skipped gracefully in the test suite when not installed.
- **6.6 Code formatting and linting:** All code must pass `ruff check .` and `ruff format --check .`
  repo-wide without errors.
- **6.7 Zero-friction execution:** Every teaching notebook and canonical demo must execute on Google
  Colab (CPU runtime) and on a standard student laptop in under 60 seconds without requiring external
  C++ build chains, proprietary licenses, or specialized GPU hardware.

---

## 7. Is It Worth Keeping? (Consolidation)

*Code health is maintained by reducing cognitive and maintenance burden, not by stripping features.*

- **7.1 Consolidate, never strip.** Simplification targets *maintenance cost*—redundant boilerplate,
  drift between twin classes, or dead code. Never remove an existing feature or public symbol without
  an explicit maintainer decision.
- **7.2 An intentional implementation ladder is not duplication.** Providing a clear pedagogical ladder
  (e.g., a simple loop implementation for students, a vectorized NumPy implementation for speed,
  and a JIT-compiled JAX implementation for research) is deliberate architectural design, not code duplication.
- **7.3 Strive for simplicity.** If a component has become over-engineered with helper layers and
  indirections, refactor to make it readable like a textbook chapter. Simplicity is the ultimate sophistication.
