# JAX/NumPy Coexistence — Detailed Implementation Plan

This document is the operational follow-up to `plan.md`. Where `plan.md`
sets the strategy ("twin classes for plants, ABC + two impls for backends,
`array_module` for thin helpers, central backend policy, lazy imports"), this
file says **what files to change, in what order, with what tests, and with
what acceptance criteria**.

It is structured as a series of small, independently mergeable PRs (each PR
≈ one section here). Every PR is small enough to be reviewed line-by-line per
`agent.md` §6 ("ask before architectural refactors"), and every PR leaves the
NumPy-only test suite green.

> The **bold rule** for every step below: a fresh `pip install minilink`
> environment (no extras) must pass `pytest tests/unittest/` with all JAX
> tests cleanly skipped. We never knowingly degrade the NumPy path.

## 0. Conventions used in this document

- **Files** are given as repo-relative paths (`minilink/...`, `tests/...`).
- Diff-shaped snippets use ``- `` for removed lines and ``+ `` for added
  lines; they are illustrative, not literal patches.
- "PR N" labels are sequencing only; the actual PR titles can match the
  step description.
- "(touch)" means a small, mechanical edit. "(new)" means a new file.
- Each PR ends with a "Done when" checklist. CI on the PR is one of the
  acceptance criteria.

---

## PR 1 — Documentation: land `plan.md` + fold §3 into `agent.md` / `DESIGN.md`

**Goal.** Make the policy *quotable*. Until the rules are in `agent.md` /
`DESIGN.md`, every later PR can be argued with by appealing to current code.

**Scope.**

- (touch) `agent.md` §3 — append a short subsection "JAX vs NumPy
  coexistence" referencing `plan.md` §3 (the five rules: subclass-and-override
  for plants, `array_module(t)` for thin helpers, ABC + two backends for
  evaluators/transcriptions, lazy imports, no top-level `import jax` in
  library code).
- (touch) `DESIGN.md` §2.6 — add a row to the "pluggable role naming" table
  for **JAX twin classes** (same module, `Jax<Name>` subclass, overrides only
  equation methods).
- (touch) `DESIGN.md` §3.5 — restate that `Jax*Transcription` requires
  `JaxQuadraticCost` and link to the future single-source-of-truth helper
  (PR 5).
- (touch) `README.md` "Examples" section — one paragraph: "JAX is optional;
  install `minilink[jax]` to unlock JAX-traced plants/costs and analytic
  gradients in trajectory optimization. The NumPy path is the default."

**Out of scope.** No code edits in `minilink/`. No test edits.

**Done when.**

- [ ] `agent.md` §3 contains the five-rule summary.
- [ ] `DESIGN.md` §2.6 / §3.5 explicitly mention the twin pattern and the
  `JaxQuadraticCost` rule.
- [ ] CI green.

---

## PR 2 — Fix `test_jax_direct_collocation.py` collection without JAX

**Goal.** A user without JAX should be able to run `pytest tests/unittest/`
and see all JAX tests **skipped**, not **errored at collection**.

**Problem.** `tests/unittest/test_jax_direct_collocation.py:62` has

```python
def make_single_integrator_problem(self, cost_cls=JaxQuadraticCost):
```

`JaxQuadraticCost` is bound inside a `try: import jax ...` block at the top
of the same file, so when JAX is missing, the default-argument expression
raises `NameError` during class-body evaluation, **before** `skipUnless`
can fire.

**Scope.**

- (touch) `tests/unittest/test_jax_direct_collocation.py` — replace the
  `try/except` shim with `pytest.importorskip("jax")` at module top, **before**
  any `Jax*` symbol is referenced. After that line, all JAX imports are safe.
- (touch) Same change pattern (audit) for any other JAX-only test file that
  references JAX symbols at class- or module-body parse time. Candidates from
  the survey: `tests/unittest/test_mechanical_jax.py`,
  `tests/unittest/test_physics_engine_jax.py`,
  `tests/unittest/test_physics_system_minilink.py`,
  `tests/unittest/test_jax_dynamic_bicycle.py` (already correct on the
  branch from PR #12; verify after merge).

  Pattern (illustrative):

  ```python
  -    try:
  -        import jax
  -        import jax.numpy as jnp
  -        from minilink.core.costs import JaxQuadraticCost, QuadraticCost
  -        ...
  -        HAS_JAX = True
  -    except ImportError:
  -        HAS_JAX = False
  +    import pytest
  +    pytest.importorskip("jax")
  +    import jax
  +    import jax.numpy as jnp
  +    from minilink.core.costs import JaxQuadraticCost, QuadraticCost
  ```

  Drop the `@unittest.skipUnless(HAS_JAX, ...)` decorators (they become
  redundant) but keep `@pytest.mark.optional` / `@pytest.mark.jax` markers
  so tests are still selectable.

**Out of scope.** No production code change. No new tests.

**Done when.**

- [ ] In a NumPy-only environment (e.g. fresh venv, `pip install -e .`),
  `pytest tests/unittest/` finishes with **zero collection errors**, all
  JAX tests reported as **skipped**.
- [ ] In a JAX environment (`pip install -e .[jax]`), JAX tests run as
  before.
- [ ] CI green.

---

## PR 3 — Remove the dead `_array_api.array_module` duplicate

**Goal.** One canonical `array_module(x)` in the repo.

**Scope.**

- `minilink/dynamics/abstraction/_array_api.py` — delete the file (no
  imports anywhere in `minilink/`, confirmed by `rg array_module`). If we
  prefer not to break any out-of-tree imports, replace the body with:

  ```python
  -import numpy as np
  -
  -
  -def array_module(x):
  -    """Return ``numpy`` or ``jax.numpy`` to match *x*."""
  -    if isinstance(x, np.ndarray):
  -        return np
  -    try:
  -        import jax.numpy as jnp
  -    except ImportError:
  -        return np
  -    return jnp
  +"""Deprecated: prefer ``minilink.compile.jax_utils.array_module``."""
  +from minilink.compile.jax_utils import array_module  # noqa: F401
  ```

  Recommended path is **delete** (the docstring already labels the file
  "legacy") and remove `dynamics/abstraction/__init__.py` re-exports if any.

**Done when.**

- [ ] `rg "_array_api"` returns no hits in `minilink/` or `tests/`.
- [ ] `rg "array_module"` shows only `minilink/compile/jax_utils.py` and
  call sites.
- [ ] CI green.

---

## PR 4 — Add `compile/backend_policy.py` (centralized backend strings)

**Goal.** Every place that reads `compile_backend` reads the same constants
from one module, with one helper for resolution and one for hard requirement.

**New file.** `minilink/compile/backend_policy.py`

Sketch (final shape lives in the PR):

```python
"""Central vocabulary for compile-backend strings.

One source of truth shared by Simulator, planning options, compile.compile,
and benchmarks. Avoids drift in the meaning of "numpy" / "jax" / "auto" /
"direct" across modules.
"""

BACKEND_NUMPY = "numpy"
BACKEND_JAX = "jax"
BACKEND_AUTO = "auto"
BACKEND_DIRECT = "direct"

VALID_COMPILE_BACKENDS = (BACKEND_NUMPY, BACKEND_JAX, BACKEND_AUTO)
VALID_TRANSCRIPTION_BACKENDS = (
    BACKEND_NUMPY, BACKEND_JAX, BACKEND_AUTO, BACKEND_DIRECT,
)


def normalize_backend(name, *, allow_direct=False):
    """Validate and lowercase a backend string.

    `allow_direct` switches between compile-target rules (Simulator,
    compile.compile) and transcription rules.
    """
    ...


def resolve_auto(name):
    """If `name` is "auto", try JAX, fall back to "numpy". Otherwise return name."""
    ...


def require_jax_backend():
    """Hard-require JAX. Raises ImportError with the install hint."""
    try:
        import jax  # noqa: F401
    except ImportError as e:
        raise ImportError(
            "This code path requires JAX. Install with `pip install minilink[jax]`."
        ) from e
```

Also add a `require_jax()` helper next to the existing `require_jax_numpy()`
in `minilink/compile/jax_utils.py` (or import-and-re-export from
`backend_policy`).

**Touched files (read constants from the new module):**

- `minilink/simulation/simulator.py` — replace literal `"numpy"`,
  `"jax"`, `"auto"`, `COMPILE_BACKEND_AUTO = "auto"` with the constants. Keep
  `COMPILE_BACKEND_AUTO` as a module re-export of `BACKEND_AUTO` for
  back-compat.
- `minilink/compile/compiler.py` — `compile(system, backend=...)` accepts
  the constants; the `try: import jax` branch becomes
  `require_jax_backend(); ...`.
- `minilink/planning/trajectory_optimization/planner.py` —
  `TrajectoryOptimizationOptions.compile_backend` keeps the same default
  (`BACKEND_NUMPY`).
- `minilink/compile/benchmark.py` — backend label rendering already lives
  in `format_benchmark_backend_label`; add a one-liner that imports the
  constants when comparing.
- `minilink/optimization/benchmark.py` (new in PR #10) — uses NumPy /
  Ipopt strings only; touch only if it grows JAX rows.

**Tests.**

- (new) `tests/unittest/test_backend_policy.py` — round-trip
  `normalize_backend`, `resolve_auto` (with and without JAX), and
  `require_jax_backend` raising the canonical message.

**Done when.**

- [ ] `rg '"numpy"|"jax"|"auto"|"direct"' minilink/` shows no literal
  backend strings outside `compile/backend_policy.py`,
  `simulation/simulator.py` (CLI-style messages and docstrings only), and
  the `Jax*Transcription` files (which still have one literal `"jax"` in
  their `compile_backend="jax"` default — that is OK, see PR 6).
- [ ] All previous tests still green; new `test_backend_policy.py` passes
  in both NumPy-only and JAX environments.

---

## PR 5 — Centralize the `JaxQuadraticCost` rule

**Goal.** One source of truth for "JAX trajopt requires `JaxQuadraticCost`"
and one error message.

**Today.** The rule is implemented three times, in:

- `minilink/planning/trajectory_optimization/jax_direct_collocation.py:76`
  (`_validate_jax_traceable_quadratic_pairing`)
- `minilink/planning/trajectory_optimization/jax_shooting.py:275`
- `minilink/planning/trajectory_optimization/jax_multiple_shooting.py:210`

Each is a near-verbatim copy.

**Scope.**

- (new) `minilink/core/costs.py` — add a small free function next to the
  cost classes:

  ```python
  def require_jax_traceable_cost(cost: CostFunction) -> None:
      """Raise if a quadratic cost is not the JAX-traceable variant.

      JAX trajectory-optimization transcriptions need cost functions whose
      `g`/`h` trace through `jax.numpy`. ``QuadraticCost`` materializes
      Python floats and breaks the trace; use ``JaxQuadraticCost``.
      """
      if isinstance(cost, QuadraticCost) and not isinstance(cost, JaxQuadraticCost):
          raise ValueError(
              "JAX trajectory optimization requires a JAX-traceable cost "
              "function; for quadratic costs use JaxQuadraticCost instead "
              "of QuadraticCost."
          )
  ```

- (touch) the three `jax_*.py` transcription modules — replace each local
  validator with a one-line call:

  ```python
  -    @staticmethod
  -    def _validate_jax_traceable_quadratic_pairing(cost: CostFunction) -> None:
  -        if isinstance(cost, QuadraticCost) and not isinstance(cost, JaxQuadraticCost):
  -            raise ValueError(...)
  -
  -    def transcribe(...):
  -        ...
  -        self._validate_jax_traceable_quadratic_pairing(cost)
  +    def transcribe(...):
  +        ...
  +        require_jax_traceable_cost(cost)
  ```

- (touch) `tests/unittest/test_jax_direct_collocation.py` — keep the
  existing `assertRaisesRegex(ValueError, "JaxQuadraticCost")` test; it now
  validates the centralized message.

**Done when.**

- [ ] `rg "JaxQuadraticCost instead of"` returns one hit (in
  `minilink/core/costs.py`).
- [ ] All three JAX transcriptions still raise the same `ValueError` with
  a `QuadraticCost`.
- [ ] CI green.

---

## PR 6 — Standardize optional-import patterns in library code

**Goal.** Two patterns repo-wide, no more (per `plan.md` §3.4):

- **Lazy `require_jax_numpy()`** inside JAX-only methods.
- **`array_module(x)`** for hybrid library code.

Module-level `try: import jax / _HAS_JAX = True/False` is allowed only in
**tests** and **runner scripts**.

**Scope.**

- (touch) `minilink/simulation/scenarios/basic.py`:

  ```python
  -try:
  -    import jax.numpy as jnp
  -    _HAS_JAX = True
  -except ImportError:
  -    jnp = None
  -    _HAS_JAX = False
  -
  -class JaxPendulum(DynamicSystem):
  -    ...
  -    def __init__(self, *, gravity=9.81, length=1.0, damping=0.0):
  -        if not _HAS_JAX:
  -            raise ModuleNotFoundError(...)
  -        super().__init__(n=2, m=1, p=2)
  -        ...
  -
  -    def f(self, x, u, t=0, params=None):
  -        ...
  -        return jnp.array([dq, ddq])
  +from minilink.compile.jax_utils import require_jax_numpy
  +
  +class JaxPendulum(NumpyPendulum):
  +    """Pendulum using JAX ops in `f`. Inherits everything from NumpyPendulum
  +    except the dynamics."""
  +
  +    def __init__(self, *, gravity=9.81, length=1.0, damping=0.0):
  +        super().__init__(gravity=gravity, length=length, damping=damping)
  +        self.name = "JaxPendulum"
  +
  +    def f(self, x, u, t=0, params=None):
  +        jnp = require_jax_numpy()
  +        q, dq = x[0], x[1]
  +        ddq = -(self.gravity / self.length) * jnp.sin(q) - self.damping * dq + u[0]
  +        return jnp.array([dq, ddq])
  ```

  Note this also turns `JaxPendulum` into a **subclass** of `NumpyPendulum`
  (matches the §3.2 rule from `plan.md`), eliminating the duplicated
  `__init__`. PR 7 below applies the same idea to `JaxMechanicalSystem`.

- (touch) `minilink/planning/trajectory_optimization/benchmark.py` — the
  current top-of-file `try: import` block stubs the JAX transcription
  classes to `None`. That is fine for a benchmark module (it is opt-in),
  so leave as-is but **document the exception** in `agent.md` §3 ("benchmark
  modules may use module-level `_HAS_JAX` guards because their public API
  is to skip rows when JAX is missing").

- (audit, no edits expected) `minilink/compile/compiler.py`,
  `minilink/compile/evaluators/jax_evaluator.py`,
  `minilink/planning/trajectory_optimization/jax_*.py`,
  `minilink/physics/engine_jax.py`,
  `minilink/symbolic/mechanics/export.py` — these already use lazy / deferred
  imports. Confirm during the PR.

**Tests.**

- Existing tests for `JaxPendulum` (`test_simulator.py`, `test_compile_pipeline.py`)
  still pass; `tests/unittest/test_make_pendulum_backends.py` (if it exists)
  still passes.
- Verify in a NumPy-only env that `from minilink.simulation.scenarios.basic
  import JaxPendulum` does **not** import JAX (only `JaxPendulum().f(...)`
  does).

**Done when.**

- [ ] `rg "_HAS_JAX" minilink/` returns at most the two allowed module-level
  guards (`benchmark.py` JAX-trajopt feature flag).
- [ ] `JaxPendulum` is a subclass of `NumpyPendulum`, ~12 lines instead of
  duplicating ~20.
- [ ] `make_pendulum(backend="jax")` and `make_pendulum(backend="numpy")`
  both still work.
- [ ] CI green in both environments.

---

## PR 7 — Restructure `JaxMechanicalSystem` as a subclass of `MechanicalSystem`

**Goal.** Apply `plan.md` §3.2 to the abstraction layer: the JAX class
**inherits** the NumPy class' `__init__`, port labels, bounds, output
metadata, and overrides only the equation methods.

**Today.**
`minilink/dynamics/abstraction/mechanical.py` defines two sibling subclasses
of `DynamicSystem`. The two `__init__`s are byte-for-byte identical
(~30 lines each). `JaxMechanicalSystem` overrides every equation method
with a `jnp` version.

**Target shape:**

```python
 class MechanicalSystem(DynamicSystem):
     """NumPy implementation of the manipulator equation."""
     def __init__(self, dof=1, actuators=None):
         ...
     def H(self, q, params=None):
         return np.eye(self.dof)
     def C(self, q, dq, params=None): ...
     def B(self, q, params=None): ...
     def g(self, q, params=None): ...
     def d(self, q, dq, params=None): ...
     def x2q(self, x): ...
     def q2x(self, q, dq): ...
     def f(self, x, u, t=0, params=None): ...
     def h(self, x, u, t=0, params=None): return x
     def ddq(self, q, dq, u, t=0, params=None): ...
     # generalized_forces, actuator_forces, kinetic_energy use np.linalg.solve

-class JaxMechanicalSystem(DynamicSystem):
+class JaxMechanicalSystem(MechanicalSystem):
     """JAX-traceable counterpart. Inherits `__init__`, port labels, x2q,
     and structural helpers from MechanicalSystem; overrides only the equation
     and linear-algebra methods so they trace through `jax.numpy`."""

-    def __init__(self, dof=1, actuators=None):
-        ...   # 30 lines duplicated from MechanicalSystem
-
     def H(self, q, params=None):
         jnp = require_jax_numpy()
         dt = getattr(q, "dtype", None) or jnp.float32
         return jnp.diag(jnp.ones(self.dof, dtype=dt))

     def C(self, q, dq, params=None):
         jnp = require_jax_numpy()
         ...

     def B(self, q, params=None):
         jnp = require_jax_numpy()
         B = jnp.zeros((self.dof, self.m), dtype=...)
         for i in range(min(self.m, self.dof)):
             B = B.at[i, i].set(jnp.asarray(1.0, dtype=...))
         return B

     def g(self, q, params=None): ...
     def d(self, q, dq, params=None): ...
     # Override q2x to use jnp.concatenate; x2q (slicing) is shape-agnostic
     # and inherited as-is.
     def q2x(self, q, dq):
         jnp = require_jax_numpy()
         ...

     # Override only the methods that call np.linalg.solve.
     def actuator_forces(self, q, dq, ddq, t=0, params=None):
         ...
         return jnp.linalg.solve(B, forces)

     def ddq(self, q, dq, u, t=0, params=None):
         ...
         return jnp.linalg.solve(H, rhs)

     def f(self, x, u, t=0, params=None):
         jnp = require_jax_numpy()
         u = jnp.asarray(u)
         q, dq = self.x2q(x)
         ddq = self.ddq(q, dq, u, t, params)
         return self.q2x(dq, ddq)
```

Net change: ~30 lines deleted (duplicated `__init__`), no behavior change.

**Tests.**

- `tests/unittest/test_mechanical_jax.py` — should already cover this (per
  the survey). Add one small test that confirms `JaxMechanicalSystem.dof` /
  `state.labels` / `inputs["u"].labels` are the same as
  `MechanicalSystem`'s.
- `tests/unittest/test_jax_direct_collocation.py::test_jax_cartpole_matches_numpy_dynamics`
  — must still pass.
- (new) `tests/unittest/test_jaxmechanical_inheritance.py`:
  ```python
  def test_jax_mechanical_inherits_from_numpy():
      assert issubclass(JaxMechanicalSystem, MechanicalSystem)

  def test_jax_mechanical_init_matches_numpy():
      a = MechanicalSystem(dof=3, actuators=2)
      b = JaxMechanicalSystem(dof=3, actuators=2)
      assert a.n == b.n and a.m == b.m and a.p == b.p
      assert a.state.labels == b.state.labels
      assert a.inputs["u"].labels == b.inputs["u"].labels
  ```

**Risk.** This is an inheritance change in the abstraction layer; it can
ripple into `JaxCartPole(JaxMechanicalSystem)`. The check is the existing
`test_jax_cartpole_matches_numpy_dynamics` test. If it stays green, the
ripple is benign.

**Done when.**

- [ ] `JaxMechanicalSystem` is a subclass of `MechanicalSystem`; its
  duplicated `__init__` is gone.
- [ ] `JaxCartPole` (which already subclasses `JaxMechanicalSystem`)
  still works; its dynamics still match `CartPole`.
- [ ] `pytest tests/unittest/test_mechanical_jax.py
  tests/unittest/test_jax_direct_collocation.py
  tests/unittest/test_jax_dynamic_bicycle.py` green in JAX env.
- [ ] CI green.

---

## PR 8 — Adopt the canonical "JAX matches NumPy" smoke test pattern

**Goal.** Every JAX twin plant comes with a tiny, identical-shape test that
proves the JAX `f` matches the NumPy `f` on at least one nominal point and one
non-trivial point. The reference is `tests/unittest/test_jax_dynamic_bicycle.py`
from PR #12.

**Scope.**

- (touch) `tests/unittest/test_mechanical_jax.py` — confirm it has both a
  linear-regime and a saturated/nonlinear-regime allclose check; add the
  missing one.
- (audit) `tests/unittest/test_compile_pipeline.py`,
  `tests/unittest/test_simulator.py` — they cover `JaxPendulum` indirectly;
  no edits expected.
- (touch) `agent.md` §4 (3-Level Verification) — append "JAX twin systems
  must include a `<plant>_jax_matches_numpy` test pair (linear regime +
  non-trivial regime)."

**Done when.**

- [ ] Every JAX twin plant in `minilink/dynamics/catalog/**` has a
  `tests/unittest/test_jax_<plant>.py` (or equivalent) with the matching
  pattern. Today: `test_jax_dynamic_bicycle.py` ✓, JAX cart-pole covered by
  `test_jax_direct_collocation.py::test_jax_cartpole_matches_numpy_dynamics`
  ✓, JAX pendulum (post PR 6) needs one ←✗ → add it.
- [ ] CI green.

---

## PR 9 — Documentation: decision tree in `README.md` and `DESIGN.md` §3.5

**Goal.** Users discover the rule without reading source code.

**Scope.**

- (touch) `README.md` — add a "JAX vs NumPy" subsection under "Examples":

  > Minilink runs end-to-end on NumPy alone. JAX is an optional accelerator:
  > install with `pip install minilink[jax]` and you unlock JAX-traced plants
  > (`Jax<X>` classes), JIT-compiled simulator rollouts
  > (`Simulator(..., compile_backend="jax")` or `"auto"`), and analytic
  > gradients in trajectory optimization (`Jax*Transcription`). Without JAX,
  > those classes still **import**; only their methods raise. See `plan.md`
  > §3 for the design rules.

- (touch) `DESIGN.md` §3.5 — append the "`JaxQuadraticCost` is required for
  JAX trajopt, validated by `require_jax_traceable_cost(...)` from
  `minilink.core.costs`" rule, with a `:func:` cross-reference.
- (touch) `DESIGN.md` §4.5 — add a one-liner that the simulator
  `compile_backend="auto"` heuristic uses
  `minilink.compile.backend_policy.resolve_auto` (PR 4).

**Done when.**

- [ ] README has the section; passes Markdown lint.
- [ ] CI green.

---

## PR 10 — Optional: stretch goals (deferred until P0–P3 land)

These are recorded for tracking but explicitly deferred (not part of the
"fix the pattern" milestone):

- **`linearize(system, x, u)` helper** that delegates to a JAX evaluator's
  `jacfwd` when `compile_backend="jax"`, else falls back to
  `scipy.optimize.approx_fprime`. (`ROADMAP.md` §7 stretch goal #2.)
- **`scan`/custom-VJP rollouts** for `JaxLeafEvaluator`. (`ROADMAP.md` §7.)
- **JAX twins for the rest of the vehicle catalog** — `JaxKinematicBicycle`,
  etc. — only when there is a concrete user (per `plan.md` §3.1).
- **`JaxStep`-style block twins** — currently `core/blocks/sources.py` uses
  `array_module(t)` dispatch; do not write a `Jax<Block>` unless the
  array-module pattern proves insufficient.

These are listed in `ROADMAP.md` §7 already; PR 10's only purpose is to make
sure they reference `plan.md` §4 P4 by section number once that lands.

---

## Sequencing and dependencies

```
                                   PR 1  (docs)
                                     │
                                     ▼
                        PR 2  ←──────┴──────→  PR 3
                  (test collection)    (delete _array_api)
                                     │
                                     ▼
                           PR 4  (backend_policy)
                                     │
                                     ▼
                           PR 5  (require_jax_traceable_cost)
                                     │
                                     ▼
                           PR 6  (optional-import patterns + JaxPendulum subclass)
                                     │
                                     ▼
                           PR 7  (JaxMechanicalSystem subclass)
                                     │
                                     ▼
                           PR 8  (smoke-test pattern)
                                     │
                                     ▼
                           PR 9  (README/DESIGN.md decision tree)
```

PR 1, 2, 3 are independent and can be opened in parallel.
PR 4 must precede PR 5 (the cost validator references it indirectly through
the trajopt modules).
PR 6 must precede PR 7 (PR 6 introduces the subclass-style change in the
simpler `JaxPendulum` first; PR 7 applies the same idea to the
abstraction-layer class with a wider blast radius).
PR 8, 9 wrap up.

## Effort and risk per PR

| PR | Files touched | Lines changed | Risk | Reviewer load |
| --- | --- | --- | --- | --- |
| 1 | `agent.md`, `DESIGN.md`, `README.md` | ~50 | none (docs) | low |
| 2 | 1–4 test files | ~30 | low (test-only) | low |
| 3 | 1–2 files | ~30 | none (dead code) | low |
| 4 | 1 new + ~5 touched | ~150 | low–medium (touches Simulator) | medium |
| 5 | 1 touched + 3 touched | ~40 | low | low |
| 6 | 1 touched (`basic.py`) + audits | ~30 | low–medium (changes `JaxPendulum` semantics) | medium |
| 7 | `mechanical.py` + 1 new test | ~50 (mostly deletions) | medium (rippling into `JaxCartPole`, planning) | medium |
| 8 | 1–2 test files + `agent.md` | ~30 | none (tests-only) | low |
| 9 | `README.md`, `DESIGN.md` | ~30 | none (docs) | low |

Total: ~420 lines of net change, ~70% of which is deletions and docs. No
single PR is large enough to violate `agent.md` §6's incremental-refactor
rule.

## Acceptance for the whole milestone

The "fix the pattern" milestone is done when **all** of these hold (these are
the operational version of `plan.md` §5):

1. `pip install -e .` (no extras) → `pytest tests/unittest/` is **green**,
   with all `jax`-marked tests reported as **skipped** and **zero collection
   errors**.
2. `pip install -e .[jax]` → `pytest tests/unittest/` is **green**, with all
   `jax`-marked tests **running**.
3. `rg "_HAS_JAX" minilink/` returns at most documented exceptions
   (benchmark module flag).
4. `rg '"numpy"|"jax"|"auto"|"direct"' minilink/` shows backend strings
   only in `compile/backend_policy.py` plus a small list of documented
   re-exports / docstrings / `compile_backend="jax"` defaults in the JAX
   trajopt modules.
5. `rg "JaxQuadraticCost instead of"` returns one hit (in `core/costs.py`).
6. `JaxPendulum` and `JaxMechanicalSystem` are subclasses of their NumPy
   counterparts, with the duplicated `__init__` removed.
7. `agent.md` §3 contains the five-rule summary and references `plan.md`.
8. `DESIGN.md` §2.6 / §3.5 / §4.5 are updated.
9. `README.md` "Examples" section has the JAX-vs-NumPy paragraph.
10. No new top-level `minilink.jax` package; no global JAX/NumPy toggle;
    no `Protocol`/generics machinery introduced.
