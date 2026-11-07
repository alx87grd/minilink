# Core API improvements from the core tutorial audit

**Status:** the `examples/tutorial/core/` (`01`–`08`) API review's proposals; CA-1, CA-2 and CA-5 open, scheduled in v0.4 (TODO §1.11, each **[ask — core]**); CA-3, CA-6 and CA-7 landed. The diffs' line numbers are from the review and drift.

Each item below records:
1. **The problem and concrete reproducer**
2. **Why it is worth doing and what evolves in [CONSTITUTION.md](../dev/CONSTITUTION.md), [RULES.md](../dev/RULES.md), or [DESIGN.md](../dev/DESIGN.md)**
3. **Current code vs. exact proposed code diff** (written to follow `RULES.md` §5: no leading-underscore helpers, standard `of` / `*_of` naming, and timeless refusal messages)

---

## CA-1. Standard port arguments on `System.__init__` (`input_dim`, `output_dim`, `expose_state`, `y_dependencies`)

**Ruled 2026-11-07:** go, with one change: on a static `System` given an `input_dim`, `y_dependencies`
defaults to `("u",)` (a static map feeds through by definition; `None`, no feedthrough, would
silently misorder a diagram); `DynamicSystem` and `StepSystem` keep `None`.
Ruled the same day, the naming: words in the constructor, textbook symbols as attributes. The
first argument `n` becomes `state_dim` (beside `input_dim` and `output_dim`, which create the
ports `"u"` and `"y"`), swept in the same change with no alias; the attributes stay `n`, `m`,
`p`, the textbook dimensions (`m` and `p` totals over the ports: `input_dim=1` with a `"d"` port
of 3 gives `m = 4`). RULES §2.7's vocabulary gains `state_dim`.

### Problem and reproducer
In the system hierarchy, there are three primary system classes:
- `System` (`minilink/core/system.py`) — memoryless/static systems ($n = 0$, $y = h(u, t; p)$) and base class of all systems.
- `DynamicSystem(System)` (`minilink/core/system.py`) — continuous-time stateful systems ($\dot{x} = f(x, u, t; p)$).
- `StepSystem(System)` (`minilink/core/discrete/step_system.py`) — discrete-time stateful systems ($x_{k+1} = \text{step}(x_k, u_k, k; p)$).

Both `DynamicSystem.__init__` and `StepSystem.__init__` accept keyword shortcuts (`input_dim`, `output_dim`, `expose_state`, `y_dependencies`) and delegate to `add_standard_ports`. However, `System.__init__(self, n: int = 0)` does not:

```python
from minilink import System


class CubicGain(System):
    def __init__(self):
        super().__init__(0, input_dim=1, output_dim=1, y_dependencies=("u",))

    def h(self, x, u, t=0.0, params=None):
        y = u**3
        return y


CubicGain()
# TypeError: System.__init__() got an unexpected keyword argument 'input_dim'
```
Instead, a user authoring a static `System` must write explicit `add_input_port` and `add_output_port` boilerplate that they do not have to write for `DynamicSystem` or `StepSystem`.

### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** `CONSTITUTION.md` §2 establishes memoryless static systems ($y = h(u, t; p)$) as a first-class mathematical contract. Making `System(0, input_dim=1, output_dim=1, y_dependencies=("u",))` work identically to `DynamicSystem` and `StepSystem` removes unnecessary port-wiring boilerplate for static maps and custom nonlinearities.
- **Design/Rule evolution:**
  - Update `DESIGN.md` §4 (`DynamicSystem shortcut`) to state that `System`, `DynamicSystem`, and `StepSystem` all share the `input_dim`, `output_dim`, `expose_state`, `y_dependencies` constructor shortcut.

### Proposed code diff
- `minilink/core/system.py`:
```diff
--- a/minilink/core/system.py
+++ b/minilink/core/system.py
@@ -82,7 +82,15 @@ class System(
 ):
     """Base system with state metadata, named input/output ports, and parameters."""
 
-    def __init__(self, n: int = 0):
+    def __init__(
+        self,
+        n: int = 0,
+        *,
+        input_dim: int | None = None,
+        output_dim: int | None = None,
+        expose_state: bool = False,
+        y_dependencies: tuple[str, ...] | None = None,
+    ):
         # Dimensions and naming
         self.n = n
         self.name = self.__class__.__name__
@@ -99,6 +107,7 @@ class System(
         # Ports
         self.inputs = {}
         self.outputs = {}
+        add_standard_ports(self, input_dim, output_dim, expose_state, y_dependencies)
 
         # Simulation and solver hints
         self.traj = None
@@ -611,9 +620,14 @@ class DynamicSystem(
         expose_state: bool = False,
         y_dependencies: tuple[str, ...] | None = None,
     ):
-        super().__init__(n)
-        add_standard_ports(self, input_dim, output_dim, expose_state, y_dependencies)
+        super().__init__(
+            n,
+            input_dim=input_dim,
+            output_dim=output_dim,
+            expose_state=expose_state,
+            y_dependencies=y_dependencies,
+        )
```
- `minilink/core/discrete/step_system.py`:
```diff
--- a/minilink/core/discrete/step_system.py
+++ b/minilink/core/discrete/step_system.py
@@ -83,8 +83,13 @@ class StepSystem(
         expose_state: bool = False,
         y_dependencies: tuple[str, ...] | None = None,
     ):
-        super().__init__(n)
-        add_standard_ports(self, input_dim, output_dim, expose_state, y_dependencies)
+        super().__init__(
+            n,
+            input_dim=input_dim,
+            output_dim=output_dim,
+            expose_state=expose_state,
+            y_dependencies=y_dependencies,
+        )
```

---

## CA-2. Add `.h(x, u=None, t=0.0, params=None)` on compiled evaluators (`Evaluator` and `DiscreteEvaluator`)

**Ruled 2026-11-07: one contract, enforced.** `h` *is* the `"y"` port everywhere, empty when a
block has none (a `PID`, whose output is the command `"u"`): `ev.h(x, u=None, t=0.0, params=None)`
returns the `"y"` port or an empty array, exactly as `sys.h` (no fallback to a lone port, which
would make `ev.h` on a PID its command while `pid.h` is empty). A defect found in the review is
fixed with it: `Mux`, `Saturation` and `Sum` compute their `"y"` port by another function and
`sys.h` returns zeros; a test pins `sys.h == outputs["y"]` on every block and catalog plant.

### Problem and reproducer
On any `System`, the two primary textbook equations (`CONSTITUTION.md` §2) are:
- `sys.f(x, u, t=0.0, params=None) -> ndarray (n,)`
- `sys.h(x, u, t=0.0, params=None) -> ndarray (p,)`

When a user compiles a system (`ev = sys.compile(backend="jax")`), `ev.f(x, u, t, params)` works with the exact same signature as `sys.f`, and `ev.jacobian("y", "x")` differentiates the default output port `"y"`. Yet calling `ev.h(x, u)` raises `AttributeError`:

```python
import numpy as np
from minilink.catalog import Pendulum

sys = Pendulum()
ev = sys.compile()
x, u = sys.x0, np.zeros(sys.m)

dx = ev.f(x, u)  # works -> ndarray (2,)
y = ev.h(x, u)   # AttributeError: 'DynamicEvaluator' object has no attribute 'h'
```
To get the primary output array `y`, the caller must write `ev.outputs(x, u)["y"]`.

### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** In textbook equations, state-estimation loops (EKF/UKF measurement updates $y_k = h(x_k, u_k)$), output-feedback sensitivity, and JAX `vmap`/`jacfwd` calls (`jax.jacfwd(ev.h)(x, u)`), having `ev.h(x, u, t=0.0, params=None)` return the `"y"` port array (or the single boundary output port if only one exists) makes `ev` a compiled twin of `sys` for both `f` and `h`, while keeping `ev.outputs(x, u, t, params) -> dict[str, ndarray]` for multi-port inspection.
- **Design/Rule evolution:**
  - Update `DESIGN.md` §5, which currently states *"Evaluators do not mirror model `h` as a separate evaluator API"* and lists the **Frozen evaluator subset** (`f`, `outputs`, `signals`, `step`, `rollout`, `rollout_batch`, `jacobian`, `nominal_input`), to include `ev.h(x, u=None, t=0.0, params=None)` as the primary-output array evaluation alongside `ev.outputs(...)`.

### Proposed code diff
- `minilink/core/compile/evaluators/base.py`:
```diff
--- a/minilink/core/compile/evaluators/base.py
+++ b/minilink/core/compile/evaluators/base.py
@@ -148,6 +148,22 @@ class Evaluator:
         """Evaluate the boundary output ports at ``(x, u, t)``."""
         raise NotImplementedError
 
+    @compiled
+    def h(self, x, u=None, t: float = 0.0, params=None):
+        """Evaluate the primary output ``y = h(x, u, t; p)`` as an array."""
+        outs = self.outputs(x, u, t, params)
+        if "y" in outs:
+            return outs["y"]
+        if len(outs) == 1:
+            return next(iter(outs.values()))
+        if not outs:
+            xp = self.xp
+            return xp.zeros(0, dtype=float)
+        ports = ", ".join(repr(k) for k in outs)
+        raise ValueError(
+            f"System has multiple outputs ({ports}) and no 'y' port; call outputs(x, u, t, params)."
+        )
+
     @compiled
     def signals(self, x, u=None, t: float = 0.0, params=None):
```
- `minilink/core/compile/evaluators/discrete.py`:
```diff
--- a/minilink/core/compile/evaluators/discrete.py
+++ b/minilink/core/compile/evaluators/discrete.py
@@ -52,6 +52,22 @@ class DiscreteEvaluator:
     def outputs(self, x, u=None, k: int = 0, params=None) -> dict:
         raise NotImplementedError
 
+    @compiled
+    def h(self, x, u=None, k: int = 0, params=None):
+        """Evaluate the primary output ``y = h(x, u, k; p)`` as an array."""
+        outs = self.outputs(x, u, k, params)
+        if "y" in outs:
+            return outs["y"]
+        if len(outs) == 1:
+            return next(iter(outs.values()))
+        if not outs:
+            xp = self.xp
+            return xp.zeros(0, dtype=float)
+        ports = ", ".join(repr(k_name) for k_name in outs)
+        raise ValueError(
+            f"System has multiple outputs ({ports}) and no 'y' port; call outputs(x, u, k, params)."
+        )
+
     def nominal_input(self):
```

---

## CA-4. Parameter ergonomics across core objects (closed)

Absorbed by the one parameters model (DESIGN §4, "Parameters"), ruled and merged 2026-11-05: `with_params` goes
rather than merging a partial dictionary (its ruling 1), costs hold `params` like every object
(ruling 3), and a composite's assignment takes the complete tree (ruling 6), so none of the
three proposals here (`CA-4a`–`CA-4c`) is built as written. Their reproducers are in git
history before this change.

---

## CA-5. Export core mathematical types on `minilink.core` (and optional root prelude `minilink`)

**Ruled 2026-11-07, with the root exports row (TODO §1.13):** `minilink.core` exports the core
objects' family, `MarginField`, `GridField`, `CallableSet`, `IntersectionSet`, `SumCost`,
`ScaledCost`, `SampledSystem` and `SampledDiagramSystem`, so none needs a deep import; the root
prelude gains only the concrete fields users build, `QuadraticField` and `GridField` (its rule:
the concrete objects you build); `Field` stays on `minilink.core` beside `Set` and
`Distribution`; `ModelPredictiveController` and `MPPIPlanner` stay on `minilink.control` and
`minilink.planning`, where every example imports them.

### Problem and reproducer
Several core classes used in the core tutorials are missing from `minilink/core/__init__.py`:
- `MarginField` (returned by `Set.as_field()`),
- `CallableSet` (user-defined constraint sets $m(x, t) \ge 0$),
- `IntersectionSet` (returned by `set1 & set2`),
- `SumCost` (returned by `cost1 + cost2`, with `SumCost.of(**named)` as a public factory),
- `ScaledCost` (returned by `w * cost`).

Furthermore, `Field` and `QuadraticField` are exported from `minilink.core` but not from root `minilink/__init__.py`.

### Why it is worth doing & design/rule evolution
- **Why it is worth doing:**
  1. Exporting `MarginField`, `CallableSet`, `IntersectionSet`, `SumCost`, and `ScaledCost` from `minilink/core/__init__.py` satisfies `RULES.md` §4.2 (*"README, tutorials, teaching examples, and demos import strictly via the root prelude or band facades, never through internal paths"*) and lets us shrink `tests/unittest/teaching_import_allowlist.txt` by removing `examples/tutorial/core/03_sets_fields_distributions.ipynb minilink.core.fields`.
  2. Exporting `Field` and `QuadraticField` on root `minilink/__init__.py` makes all six core mathematical nouns (`Trajectory`, `Set`/`BoxSet`/`BallSet`/`EllipsoidSet`, `Distribution`/`Gaussian`/`Uniform`, `Field`/`QuadraticField`, `CostFunction`/`QuadraticCost`) importable from `minilink`.
- **Design/Rule evolution:**
  - Updating `minilink/core/__init__.py` directly fulfills `RULES.md` §4.2.
  - Adding `Field` and `QuadraticField` to root `minilink/__init__.py` updates the root teaching surface set in `minilink/core/exports.py` and `tests/unittest/test_teaching_surface.py` (`DESIGN.md` §2).

### Proposed code diff
- `minilink/core/__init__.py`:
```diff
--- a/minilink/core/__init__.py
+++ b/minilink/core/__init__.py
@@ -34,6 +34,8 @@ _LAZY_EXPORTS = {
     "QuadraticCost": ("minilink.core.costs", "QuadraticCost"),
+    "ScaledCost": ("minilink.core.costs", "ScaledCost"),
+    "SumCost": ("minilink.core.costs", "SumCost"),
     "SampledCost": ("minilink.core.costs", "SampledCost"),
     "StepCost": ("minilink.core.costs", "StepCost"),
     "TimeCost": ("minilink.core.costs", "TimeCost"),
@@ -43,14 +45,17 @@ _LAZY_EXPORTS = {
     # Scalar fields over the state space
     "Field": ("minilink.core.fields", "Field"),
+    "MarginField": ("minilink.core.fields", "MarginField"),
     "QuadraticField": ("minilink.core.fields", "QuadraticField"),
     # Sets
     "BallSet": ("minilink.core.sets", "BallSet"),
     "BoxSet": ("minilink.core.sets", "BoxSet"),
+    "CallableSet": ("minilink.core.sets", "CallableSet"),
     "EllipsoidSet": ("minilink.core.sets", "EllipsoidSet"),
     "FieldSet": ("minilink.core.sets", "FieldSet"),
     "InputSet": ("minilink.core.sets", "InputSet"),
+    "IntersectionSet": ("minilink.core.sets", "IntersectionSet"),
     "Set": ("minilink.core.sets", "Set"),
     "SingletonSet": ("minilink.core.sets", "SingletonSet"),
```

---

## CA-7. Target / operating-point naming

Landed (2026-11-04): `x_bar` / `u_bar` / `y_bar` everywhere (RULES 2.7, `test_vocabulary.py`).

---

## Decisions taken during the automatic run (2026-11-07, branch `v04-build`)

Choices the rulings left open, each taken as recommended; the maintainer may overturn any.

- **CA-1, `StateSpaceSystem`:** its own keyword-only `n`, `m`, `p` became `state_dim`,
  `input_dim`, `output_dim`, so every system constructor reads the same words (the catalog's
  three mass-spring-damper plants and one test follow). The path-tracking project's vehicles
  take `state_dim` in their own constructors too.
- **CA-1, port order:** `System.__init__` creates the standard ports right after its port
  tables, before the model defaults (`x0`, `params`, hints); `DynamicSystem` and `StepSystem`
  pass their shortcuts through it instead of adding the ports after it.
- **CA-2, the same defect elsewhere:** the test over every block found three more blocks whose
  `h` disagreed with their `"y"` port: `Gain` and `MLP` (their `"y"` by `compute`, renamed `h`;
  `MLP.compute` callers in the RL critics and policies follow) and the transfer-function
  compensator (`Lead`, `ports="error"`): it has no `"y"` port, so its command moved to `ctl`
  and its `h` is empty, as a `PID`'s.
- **The controller/policy split:** the deep RL planner's own live block and exploration law
  were `self.controller` and `self.policy`, which would shadow the new mirror; they are
  `planner.actor` and `planner.exploration_policy`. `planner.controller` is the latest
  solution's (after a solve, the same weights). An open-loop solution's `policy(x, t)` reads
  `t` alone; a feedback one refuses no `x`. `get_controller(interpolation=)` variants go: build
  `LookupPolicy(grid, pi, interpolation=)` directly.
- **Workflow on the verb:** each planner's own workflow keywords moved (no dead `callback` on
  value iteration or LQR); RRT*'s `history_stride` went with `record_history`; a notebook whose
  planner was verbose keeps `verbose=True` on each later solve, so its output is unchanged.
- **`problem.compile`:** the library builds its faces with `StepFace(problem, dt=)` directly
  (value iteration byte-identical); a discretized problem's face now reads a random plant at
  its mean (a defect found with it).
- **Helpers to module functions:** `move_start` folded into the `x0` setter;
  `open_loop_controller_of` takes the plant; `store_solution`, `build_kwargs` and `parts_view`
  stay methods (the planner's own memory, overridden per planner, the deferred parameters
  fold); `require_goal(problem)` has no caller left (the final review decides).
