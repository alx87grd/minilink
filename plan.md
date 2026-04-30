# JAX / NumPy Coexistence — Strategy and Guidelines

This is a *plan*, not a code change. It surveys how JAX and NumPy paths
currently coexist in `minilink`, picks a target architecture, and lays out
guidelines and a roadmap for getting there incrementally without breaking the
NumPy-only workflow.

The intent is:

- A user who installs `minilink` (no extras) gets a fully working **NumPy
  pipeline**: `System`, `Trajectory`, `Simulator`, `compute_forced`,
  `plot_trajectory`, `Optimizer` (SciPy / Ipopt), `TrajectoryOptimizationPlanner`
  with NumPy transcriptions, animation, etc.
- A user who installs `minilink[jax]` (or has JAX in their env) gets the
  **same APIs accelerated** where possible: JAX-traceable plants and costs,
  analytic gradients/Jacobians for trajectory optimization, JIT-compiled
  rollouts in the simulator, and JAX physics demos.
- The library code stays textbook-readable per `agent.md` §1–§2 and stays
  honest about which math belongs to the modeling layer vs. which belongs to
  the compile / simulate / optimize backends (`agent.md` §3, `DESIGN.md` §2).

## 1. Where we are today

`agent.md` §3 already captures the policy in spirit:

> Use **inheritance** for core system types and **composition** for diagrams
> and optional behaviors.
> Keep the readable modeling path clean; isolate optimization in `compile/`
> and `simulation/`. Support JAX when it stays clean; use specialized JAX
> paths when needed instead of complicating the main path.

`DESIGN.md` §2 / §3.5 / §4.5 then say: NumPy is the default, JAX is a
deliberately narrower, opt-in path; `Simulator(..., compile_backend="auto")`
tries JAX and falls back; user-facing `System.compute_trajectory` defaults to
NumPy for predictability.

In practice four different patterns have grown up:

| Pattern | Where | What it looks like |
| --- | --- | --- |
| **Twin classes** (NumPy + `Jax<X>`) | `MechanicalSystem` / `JaxMechanicalSystem` (`minilink/dynamics/abstraction/mechanical.py`); `CartPole` / `JaxCartPole` (`minilink/dynamics/catalog/pendulum/cartpole.py`); `QuadraticCost` / `JaxQuadraticCost` (`minilink/core/costs.py`); `NumpyPendulum` / `JaxPendulum` (`minilink/simulation/scenarios/basic.py`); now `DynamicBicycle` / `JaxDynamicBicycle` (`minilink/dynamics/catalog/vehicles/dynamic_bicycle.py`, branch `cursor/jax-dynamic-bicycle-672e`). | Two parallel classes with similar bodies; JAX class either subclasses the NumPy one and overrides math (cart-pole, bicycle, quadratic cost) or is a sibling subclass of `DynamicSystem` (mechanical, pendulum). |
| **Twin transcriptions** | `direct_collocation.py` / `jax_direct_collocation.py`, `shooting.py` / `jax_shooting.py`, `multiple_shooting.py` / `jax_multiple_shooting.py`. | JAX classes subclass the NumPy classes for layout/packing helpers but reimplement the full NLP closure with `jit` / `vmap` / `grad` / `jacfwd` / `hessian`. |
| **Twin evaluator backends** | `compile/evaluators/{numpy_evaluator.py, jax_evaluator.py}` behind a single `compile/evaluators/evaluator.py` ABC. | This is the *good* pattern: one contract, two implementations chosen by `compile(system, backend=...)`. |
| **Array-module dispatch** | `minilink/compile/jax_utils.array_module(a)` → `jnp` if `a` is a JAX array else `np`. Used in `core/blocks/sources.py` (e.g. `Step.h`) and a couple of tests; an unused legacy duplicate sits in `minilink/dynamics/abstraction/_array_api.py`. `physics/system.PhysicsWorldSystem.f` does its own input-type branching. | One function body that works for both backends — never written twice. |

Optional-import patterns are also inconsistent. The repo has at least four
ways of saying "JAX is optional":

1. Module-level `try/except ImportError` with a `_HAS_JAX` flag and a `jnp = None` fallback (`simulation/scenarios/basic.py`, `planning/trajectory_optimization/benchmark.py`).
2. `minilink.compile.jax_utils.require_jax_numpy()` called at method-body time (`JaxMechanicalSystem.*`, `JaxCartPole.*`, `JaxQuadraticCost.g/h`, the new `JaxDynamicBicycle.*`).
3. Deferred `import jax(.numpy)` inside a function body (`compile/compiler.py` when `backend == "jax"`, `compile/evaluators/jax_evaluator.py` `__init__`, every `Jax*Transcription.transcribe`, `physics/engine_jax._jnp()`, `Simulator` auto-probe).
4. Hard top-level `import jax` (the JAX-only `examples/scripts/physics/demo_physics_jax_*.py`, some `tests/benchmark/*` scripts).

There are also four small concrete papercuts surfaced by the survey:

- The `_array_api.array_module` duplicate of `jax_utils.array_module` is dead code.
- `tests/unittest/test_jax_direct_collocation.py` references `JaxQuadraticCost` as a default argument on a method *outside* the `try: import jax` block, so collection raises `NameError` when JAX is missing instead of skipping cleanly.
- `compile_backend` accepts `"numpy"` / `"jax"` / `"auto"` / `"direct"` / `None` with subtly different meanings between `Simulator`, `TrajectoryOptimizationOptions`, and the JAX transcriptions; the JAX trajopt modules also outright reject plain `QuadraticCost` (must use `JaxQuadraticCost`) — neither rule is centralized in code.
- `JaxDynamicBicycle` exists but the equivalent JAX twin is missing for most other vehicles; the catalog is uneven.

## 2. Architectural target

**Default rule (P0): keep the twin-class style for plants, costs, and
transcriptions; tighten the contract around it.** This is what already works
and matches `agent.md`'s preference for *inheritance for core system types*
and *isolating optimization in `compile/` and `simulation/`*.

The twin pattern's real cost is duplicated math. We reduce that cost without
abandoning the pattern by:

1. **Single source of truth at the equation layer.** When a `Jax<X>` plant
   only differs from `<X>` by `np.*` ↔ `jnp.*` and a couple of in-place
   updates, the JAX class **subclasses** the NumPy class and overrides only
   the equation methods (`H`, `C`, `B`, `g`, `d`, `f`, `h`, …). Geometry,
   visualization, parameter dataclasses, port labels, bounds — **always
   inherited**. Concrete examples already live in the tree:
   `JaxCartPole(JaxMechanicalSystem)` reuses `_configure_cartpole_metadata`
   and the visualization methods of `CartPole`; `JaxDynamicBicycle(DynamicBicycle)`
   inherits `get_kinematic_geometry` / `get_kinematic_transforms` and rewrites
   only the dynamics (~140 lines vs duplicating ~700 lines of geometry code).
   New `Jax<Plant>` classes **must** follow this pattern.
2. **Promote `array_module` from afterthought to first-class helper.** When
   the math is "thin" — a `where`, a `cos`, a couple of products — the helper
   in `minilink.compile.jax_utils` is enough to write **one** body that works
   for both backends. This is the right tool for `core/blocks/*` (sources,
   integrators, basic blocks) and any helper math that doesn't carry numerical
   trade-offs (no in-place mutation, no Python `if` on values, no
   stateful-numpy idioms). Replace the dead `dynamics/abstraction/_array_api.py`
   with a re-export so there is one canonical name.
3. **One contract per role, two implementations behind it.** Continue what
   `compile/evaluators/` already does for evaluators: `evaluator.py` is the
   ABC, `numpy_evaluator.py` and `jax_evaluator.py` are the two backends.
   Trajectory transcriptions should evolve toward the same shape: one
   `transcription.py` ABC, plus per-transcription NumPy/JAX siblings that share
   layout helpers via inheritance but stay swappable behind the ABC. We are
   already 90% there; the remaining task is a documented "transcription
   backend" contract instead of leaving it implicit in the JAX classes.
4. **Backend selection is explicit and centralized.** A small
   `minilink.compile.backend_policy` module (or just helpers in
   `compile/jax_utils.py`) owns the strings and the rules:

   - `"numpy"` — never imports JAX. Always works.
   - `"jax"` — requires JAX; raises `ImportError` with a one-line
     `pip install minilink[jax]` hint at the entry point.
   - `"auto"` — try JAX, fall back to NumPy on `ImportError` or any failure
     during evaluator construction. The simulator already does this; the
     planner and benchmarks should reuse the same helper.
   - `"direct"` — call `system.f` directly; valid for transcriptions that want
     unfrozen Python plants. Document that this is a transcription concept, not
     a compile-target.

   Centralizing avoids the current papercut where each module spells the rules
   slightly differently.

5. **Optional-import policy.** Settle on **two** patterns repo-wide:

   - **Library code that lives in a JAX-only module** (`Jax<X>`, every
     `compile/evaluators/jax_evaluator.py`, every
     `planning/trajectory_optimization/jax_*`, `physics/engine_jax`) — call
     `require_jax_numpy()` (or, if `jax` itself is needed, the parallel
     `require_jax()`) at call time inside methods. This keeps **module
     import** cheap and JAX-free; only running the method needs JAX.
   - **Hybrid library code** (`core/blocks/sources`, anywhere
     `array_module(t)` makes sense) — never import `jax` at module level; use
     `array_module(...)` to dispatch on the runtime input type.

   Module-level `try: import jax` + `_HAS_JAX` is allowed for *test files* and
   *runner scripts* only; library modules should not need it.

6. **Test files never break collection without JAX.** Library code already
   degrades gracefully; tests must follow. Two acceptable patterns:

   - `pytest.importorskip("jax")` at module top before any `Jax*` symbol is
     referenced.
   - All test classes / functions defined inside an `if HAS_JAX:` block, with
     no JAX symbols leaking into default arguments at parse time.

   This is the rule that fixes today's `test_jax_direct_collocation.py`
   collection error in environments without JAX.

The combined picture is therefore "**twin classes for plants, single ABC for
backends, array-module dispatch for thin helpers, central policy for backend
selection, lazy imports everywhere**".

## 3. Style guidelines (proposed addendum to `agent.md` §3 and `DESIGN.md` §2)

These are written so they can be copied verbatim into `agent.md` once
approved.

### 3.1 When to write a NumPy plant only

**Default for new plants.** A new `System` subclass starts NumPy-only. Add a
JAX twin only when there is a concrete user — currently:

- a JAX trajectory-optimization transcription (`JaxDirectCollocationTranscription`,
  `JaxShootingTranscription`, `JaxMultipleShootingTranscription`) needs analytic
  gradients on the plant's `f`;
- a `Simulator` use case needs `lax.scan` / `jit` rollouts for wall-clock;
- a physics or differentiable-simulation experiment needs the plant's `f` to
  trace.

If the use case is "compile NumPy `f` with the JAX evaluator", that is already
covered by `compile(system, backend="jax")` — no JAX twin needed.

### 3.2 How to write a JAX twin

- **Subclass the NumPy class** unless that subclass would have to override
  more than ~70% of the methods. Inherit geometry, parameter dataclasses, port
  labels, visualization, helper methods.
- **Override only the equation methods** (`H`, `C`, `B`, `g`, `d`, `f`, `h`,
  custom force terms). Inside an override, call `require_jax_numpy()` once,
  bind `jnp = require_jax_numpy()`, then write the textbook math against
  `jnp`. Use `jnp.where` instead of `if`-on-values; use `jnp.linalg.solve`
  instead of `inv`.
- **Match the NumPy semantics numerically.** Add a one-line smoke test in
  `tests/unittest/test_jax_<plant>.py` that asserts
  `np.allclose(np.asarray(jax_sys.f(x, u)), numpy_sys.f(x, u))` on at least
  one nominal point and one saturating / nonlinear point (the
  `JaxDynamicBicycle` tests are the model: linear regime + friction-circle
  saturation).
- **Naming.** `Jax<NumPyName>`, no other prefix. The JAX class lives in the
  **same module** as the NumPy one (`cartpole.py` already holds both;
  `dynamic_bicycle.py` already holds both after PR #12). One file per plant —
  no `jax_<plant>.py` mirror file.

### 3.3 How to write hybrid (array-module-dispatch) code

Use this for **blocks**, **simple math helpers**, and code that already lives
in a single module shared by both pipelines. Acceptable example
(`core/blocks/sources.py`):

```python
def h(self, x, u, t=0, params=None):
    xp = array_module(t)
    return xp.where(t < self.params["step_time"], lo, hi)
```

**Do not** use array-module dispatch inside a complex plant `f` — that path
quickly turns into spaghetti. Plants get the twin-class treatment.

### 3.4 Optional dependency hygiene

- Library modules: never `import jax` at module level.
- JAX-only library modules (`compile/evaluators/jax_evaluator.py`,
  `planning/trajectory_optimization/jax_*`, `physics/engine_jax`): import
  `jax` / `jax.numpy` lazily inside functions, or via
  `require_jax_numpy()` / `require_jax()` (a small companion to the existing
  helper).
- Twin classes: the JAX class **may** live in the same module as the NumPy
  class (recommended — see §3.2). The module itself must remain importable
  without JAX. JAX is loaded only when a `Jax<X>` method actually runs.
- Examples and runner scripts under `examples/scripts/` and `tests/benchmark/`
  may `import jax` at module level; they are opt-in entry points.
- Tests gating on JAX: `pytest.importorskip("jax")` at module top, or define
  classes inside `if HAS_JAX:`.

### 3.5 Backend strings (single vocabulary)

Defined once in `compile/jax_utils.py` (or a new
`compile/backend_policy.py`):

| value | meaning |
| --- | --- |
| `"numpy"` | NumPy evaluator. Always available. |
| `"jax"` | JAX evaluator. Hard-requires `minilink[jax]`. |
| `"auto"` | Try JAX, fall back to NumPy. Used by `Simulator` today. |
| `"direct"` | Use `system.f` directly (no compiled evaluator). Used by some transcriptions. |

`Simulator(compile_backend=...)`,
`TrajectoryOptimizationOptions(compile_backend=...)`, and the benchmark
modules all read these from one place. No new string is added without a
matching helper. `JaxQuadraticCost` is required for JAX trajopt objectives —
this rule is documented next to the strings, not buried in three transcription
modules.

## 4. Roadmap (incremental, no big-bang refactors)

Each step is independently mergeable and small enough to fit
`agent.md` §6's "ask before architectural refactors" rule. Order matters:
later steps build on earlier ones.

### P0 — Documentation and small cleanups (no behavior change)

1. Land this `plan.md` and, after sign-off, fold §3 into `agent.md` §3 and
   `DESIGN.md` §2.6/§3.5.
2. Delete or re-export `minilink/dynamics/abstraction/_array_api.py` so there
   is one `array_module`.
3. Fix `tests/unittest/test_jax_direct_collocation.py` to use
   `pytest.importorskip("jax")` at module top, eliminating the
   `JaxQuadraticCost` collection error in NumPy-only environments.
4. Add a short "JAX vs NumPy" decision-tree paragraph to `README.md` so users
   know when to install `minilink[jax]`.

### P1 — Centralize backend policy

5. Add `minilink/compile/backend_policy.py` (or grow `jax_utils.py`) with
   `BACKEND_NUMPY`, `BACKEND_JAX`, `BACKEND_AUTO`, `BACKEND_DIRECT`,
   `resolve_backend(name) -> str`, and `require_backend(name)`. Migrate
   `Simulator`, `TrajectoryOptimizationOptions`, `compile.compile`, and the
   benchmark modules to read constants from there.
6. Add `require_jax()` companion to `require_jax_numpy()` for places that need
   `jax.jit` etc., not just `jax.numpy`.
7. Document the `JaxQuadraticCost` requirement at one place
   (`compile/backend_policy.py` docstring + `DESIGN.md` §3.5) and have each
   `Jax*Transcription` raise the same error message.

### P2 — Tighten the twin-class contract for plants and costs

8. Add a small `tests/unittest/test_<plant>_jax_matches_numpy.py` template
   that any new JAX plant must instantiate. The current
   `test_jax_dynamic_bicycle.py` is the reference shape (linear + saturated
   regime + JAX trace + grad).
9. Audit existing twin pairs (`MechanicalSystem` / `JaxMechanicalSystem`,
   `CartPole` / `JaxCartPole`, `NumpyPendulum` / `JaxPendulum`,
   `QuadraticCost` / `JaxQuadraticCost`) against §3.2: where the JAX class
   could be a subclass and isn't (e.g. `JaxPendulum` could inherit from
   `NumpyPendulum`), move it to subclass form. This eliminates ~half of the
   duplicated math in those modules.
10. Decide explicitly whether the catalog grows JAX twins by-default for new
    plants or on-demand (recommended: **on-demand**, driven by use cases
    listed in §3.1; record the decision in `DESIGN.md` §2.6).

### P3 — Tighten the twin-class contract for transcriptions

11. Promote the implicit "transcription backend" contract to a documented
    module: each transcription family lives in one Python file, with a
    NumPy class (default) and a `Jax<X>Transcription` subclass that overrides
    only `transcribe(...)`. Today the layout is split across
    `direct_collocation.py` + `jax_direct_collocation.py` etc.; the merge can
    happen one family at a time.
12. Where layout/packing/bounds helpers are duplicated between NumPy and JAX
    transcriptions, hoist them to the NumPy parent class so the JAX subclass
    really only carries the JIT/grad closure.

### P4 — Stretch goals (out of scope for this plan but recorded)

13. *Differentiable simulation.* `JaxLeafEvaluator` already supports
    `jacfwd` and `lax.scan`; once §P1 lands, surface a typed
    `linearize(system, x, u)` helper that works on any `compile_backend="jax"`
    system without re-deriving math by hand.
14. *Custom-VJP / scan-based rollouts.* `ROADMAP.md` §7 already lists this;
    blocked on §P1 (clean backend policy) and §P3 (clean transcription
    contract).
15. *JAX twins for the rest of the vehicle catalog.* On-demand, per §3.1.

## 5. Success criteria

The plan is "done" when all of these hold:

- `pip install minilink` (no extras) gives a green `pytest tests/unittest/`
  with all `jax`-marked tests cleanly skipped — no collection errors, no
  import errors, no `_HAS_JAX = False` branches that quietly disable real
  features.
- `pip install minilink[jax]` gives the same green `pytest`, plus all
  `jax`-marked tests run; the matrix is one config × two extras.
- A new plant can be promoted from "NumPy only" to "JAX accelerated" by
  adding a `Jax<Plant>` subclass in the same module that overrides only the
  equation methods, with the canonical `test_<plant>_jax_matches_numpy.py`
  smoke test as evidence (~50–150 lines of code per plant, not 500).
- `compile_backend` strings have one home; no module spells them differently.
- `agent.md` §3 and `DESIGN.md` §2/§3.5 contain the guidelines from §3 of
  this plan, not a wider rewrite.

## 6. Non-goals

- No global "switch JAX vs NumPy" toggle (cf. `ROADMAP.md` §5: "prefer
  explicit backend selection over global mutable backend switches").
- No removal of any existing twin class. The plan is to make the pattern
  cheaper, not to delete it.
- No `Protocol`/generics machinery on top of the twin classes. `agent.md`
  §1–§2 explicitly prefer textbook readability over abstract typing layers.
- No automated AST-based "make this NumPy file JAX-traceable" tooling. JAX
  twins stay hand-written and reviewed.
- No new top-level `minilink.jax` package. JAX support is per-module, like
  it is today.
