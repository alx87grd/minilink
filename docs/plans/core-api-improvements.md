# Core API improvements from the core tutorial audit

**Status:** draft plan from the `examples/tutorial/core/` (`01`–`08`) API review, saved for later execution.

Each item below records:
1. **The problem and concrete reproducer**
2. **Why it is worth doing and what evolves in [CONSTITUTION.md](../dev/CONSTITUTION.md), [RULES.md](../dev/RULES.md), or [DESIGN.md](../dev/DESIGN.md)**
3. **Current code vs. exact proposed code diff** (written to follow `RULES.md` §5: no leading-underscore helpers, standard `of` / `*_of` naming, and timeless refusal messages)

---

## CA-1. Standard port arguments on `System.__init__` (`input_dim`, `output_dim`, `expose_state`, `y_dependencies`)

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

## CA-3. Add `diagram.of(...)` and accept string subsystem IDs in `WiredDiagramMixin.subsystem_id` / `trajectory_of`

### Problem and reproducer
1. `SumCost`'s docstring (`minilink/core/costs.py:337`) states:
   > *"``SumCost.of(track=..., effort=...)`` builds a sum with the given ids (like :meth:`of` on a diagram or a trajectory)."*
   Yet `DiagramSystem` (`WiredDiagramMixin`) has **no `.of(...)` method**.
2. On a `Trajectory`, `traj.of("plant")` accepts a string subsystem ID or role name. On a `DiagramSystem`, `diagram.trajectory_of(sub, traj)` delegates to `WiredDiagramMixin.subsystem_id(subsystem)` (`minilink/core/wiring.py:275`), which checks `if sub is subsystem:` by object identity only:

```python
from minilink.catalog import Pendulum
from minilink.control import PID

loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ Pendulum()
traj = loop.compute_trajectory(2.0)

traj.of("plant")                  # works -> Trajectory
loop.trajectory_of("plant", traj) # ValueError: Subsystem is not part of this diagram.
loop.of("plant")                  # AttributeError: 'DiagramSystem' object has no attribute 'of'
```

### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** Unifies the `.of(...)` selector across `DiagramSystem`, `Trajectory`, and `SumCost`, and lets users pass either the subsystem object (`plant`) or its string ID (`"plant"`) to `diagram.of(...)`, `diagram.subsystem_id(...)`, `diagram.trajectory_of(...)`, and `diagram.compute_cost(..., of="plant")`.
- **Design/Rule evolution:**
  - Aligns with `RULES.md` §5.25 (`of` / `*_of` resolution methods) and fulfills the existing reference in `minilink/core/costs.py:337`.
  - Update `DESIGN.md` §4 (`DiagramSystem`) to note that `diagram.of(block_or_id)`, `diagram.trajectory_of(block_or_id, traj=None)`, and `sys.compute_cost(cost, of=block_or_id)` accept either the subsystem instance or its string ID.

### Proposed code diff
- `minilink/core/wiring.py`:
```diff
--- a/minilink/core/wiring.py
+++ b/minilink/core/wiring.py
@@ -275,9 +275,20 @@ class WiredDiagramMixin:
     def subsystem_id(self, subsystem: System | str) -> str:
         """Subsystem id for *subsystem* inside this diagram."""
+        if isinstance(subsystem, str):
+            if subsystem in self.subsystems:
+                return subsystem
+            known = ", ".join(repr(k) for k in self.subsystems)
+            raise ValueError(
+                f"Unknown subsystem id {subsystem!r}; known subsystems: {known}."
+            )
         for sys_id, sub in self.subsystems.items():
             if sub is subsystem:
                 return sys_id
         raise ValueError("Subsystem is not part of this diagram.")
+
+    def of(self, subsystem: str | System) -> System:
+        """Return the subsystem for a subsystem id or instance."""
+        return self.subsystems[self.subsystem_id(subsystem)]
```

---

## CA-4. Parameter ergonomics across core objects (`Distribution.with_params`, `CostFunction.with_params`, and `diagram.params`)

### CA-4a. `Distribution.with_params` fails on partial dictionaries
#### Problem and reproducer
`BoxSet.with_params`, `BallSet.with_params`, `EllipsoidSet.with_params`, and `Field.with_params` all merge partial dictionaries with `self.params`.
`Distribution.with_params` (`minilink/core/distributions.py:49`) calls `type(self)(**params)` without merging `self.params`:
```python
from minilink import Gaussian

g = Gaussian([0.0, 0.0], std=[0.1, 0.2])
g.with_params({"mean": [1.0, 0.0]})
# ValueError: Gaussian needs std or cov.
```

#### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** Fixes a bug in `Gaussian.with_params` when updating only `"mean"` or only `"std"`/`"cov"`, making `Distribution.with_params` consistent with `Set.with_params` and `Field.with_params`.
- **Design/Rule evolution:** Fully aligned with `DESIGN.md` §4 and `RULES.md` §5.8 (no leading-underscore helper).

#### Proposed code diff
- `minilink/core/distributions.py`:
```diff
--- a/minilink/core/distributions.py
+++ b/minilink/core/distributions.py
@@ -49,7 +49,8 @@ class Distribution:
     def with_params(self, params) -> Distribution:
         """Return a copy of this distribution with *params* as its parameters."""
         if params is None:
             return self
         own = self.params
         if not own:
             if params == {}:
                 return self
             raise ValueError(f"{type(self).__name__} has no parameters; got {set(params)}.")
         unknown = set(params) - set(own)
         if unknown:
             raise ValueError(
                 f"Unknown parameter(s) {sorted(unknown)} for {type(self).__name__}; "
                 f"expected {sorted(own)}."
             )
-        return type(self)(**params)
+        merged = {**own, **params}
+        return type(self)(**merged)
@@ -156,6 +157,23 @@ class Gaussian(Distribution):
         return {"mean": self.mean, "cov": self.cov}
 
+    def with_params(self, params) -> Gaussian:
+        if params is None:
+            return self
+        allowed = {"mean", "std", "cov"}
+        unknown = set(params) - allowed
+        if unknown:
+            raise ValueError(
+                f"Unknown parameter(s) {sorted(unknown)} for Gaussian; "
+                f"expected a subset of {sorted(allowed)}."
+            )
+        if "std" in params and "cov" in params:
+            raise ValueError("Gaussian takes std or cov, not both.")
+        mean = params.get("mean", self.mean)
+        if "std" in params:
+            return Gaussian(mean, std=params["std"])
+        if "cov" in params:
+            return Gaussian(mean, cov=params["cov"])
+        if self.std is not None:
+            return Gaussian(mean, std=self.std)
+        return Gaussian(mean, cov=self.cov)
```

---

### CA-4b. Add `.with_params(params)` on `CostFunction`
#### Problem and reproducer
Every `Set`, `Field`, and `Distribution` has `.with_params(params)` returning a new copy with updated parameters, leaving the original untouched. `CostFunction` (`QuadraticCost`, `TimeCost`, `FieldCost`, `ScaledCost`, `SumCost`) is the only core mathematical object in `PlanningProblem` without `.with_params(params)`:
```python
from minilink import QuadraticCost
import numpy as np

cost = QuadraticCost(Q=np.eye(2), R=np.eye(1))
cost2 = cost.with_params({"R": 10.0 * np.eye(1)})
# AttributeError: 'QuadraticCost' object has no attribute 'with_params'
```

#### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** When comparing planners or sweeping cost weights across problems (`problem.replace(cost=cost.with_params({"R": ...}))`), mutating `cost.params["R"] = ...` in place mutates the cost shared by earlier problems/solutions. Providing `cost.with_params(params)` makes non-mutating parameter copying work uniformly across all four mathematical object families (`Set`, `Field`, `Distribution`, `CostFunction`).
- **Design/Rule evolution:**
  - `DESIGN.md` §4 (`Parameters`) currently notes that library sets have `with_params` (because they are frozen dataclasses) while costs follow the mutable `System.params` pattern. Adding `CostFunction.with_params(params)` keeps `cost.params` mutable while also supporting `.with_params(params)` copies; update `DESIGN.md` §4 to mention `cost.with_params(params)`.

#### Proposed code diff
- `minilink/core/costs.py`:
```diff
--- a/minilink/core/costs.py
+++ b/minilink/core/costs.py
@@ -1,6 +1,7 @@
 """Cost functions for optimal control and reinforcement learning."""
 
 from __future__ import annotations
+import copy
 from typing import TYPE_CHECKING
@@ -49,6 +50,15 @@ class CostFunction:
     def h(self, x, t: float = 0.0, params=None):
         """Terminal cost h(x, t)."""
         return 0.0
+
+    def with_params(self, params) -> CostFunction:
+        """Return a copy of this cost with *params* merged into its parameters."""
+        if params is None:
+            return copy.deepcopy(self)
+        cloned = copy.deepcopy(self)
+        current = cloned.params
+        cloned.params = {**current, **params} if isinstance(current, dict) else params
+        return cloned
@@ -382,6 +392,13 @@ class SumCost(CostFunction):
         for tid, sub in value.items():
             self.terms[tid].params = sub
 
+    def with_params(self, params) -> SumCost:
+        if params is None:
+            return copy.deepcopy(self)
+        cloned = copy.deepcopy(self)
+        cloned.params = params
+        return cloned
```

---

### CA-4c. Allow partial subsystem dictionaries in `WiredDiagramMixin.params` setter
#### Problem and reproducer
In `SumCost.params` (`minilink/core/costs.py:379`) and `PlanningProblem.params` (`minilink/planning/problems.py:337`), assigning a partial dictionary updates the named components and leaves the rest untouched:
```python
total_cost.params = {"effort": {"R": [[10.0]]}}  # works!
problem.params = {"cost": {"R": [[10.0]]}}       # works!
```
In `WiredDiagramMixin.params` (`minilink/core/wiring.py:93`), assigning a partial dictionary raises `ValueError`:
```python
loop = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ Pendulum()
loop.params = {"plant": {**loop.of("plant").params, "m": 2.0}}
# ValueError: Diagram params must name every subsystem; missing ['Error', 'controller'].
```

#### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** All three composite objects (`DiagramSystem`, `SumCost`, `PlanningProblem`) expose a nested `.params` property keyed by component ID. Allowing `diagram.params = {"plant": ...}` to update the named subsystems on the host setter — while keeping `check_diagram_params` strict inside `f` / `outputs` / `step` where JAX traces complete pytrees — makes host-side `.params` assignment consistent across `DiagramSystem`, `SumCost`, and `PlanningProblem`.
- **Design/Rule evolution:**
  - Update `RULES.md` §2.3 and `DESIGN.md` §4 (`Diagram params are nested by subsystem id`): keep the complete-tree requirement for explicit `params=` arguments passed to equation/evaluator calls (`f`, `outputs`, `step`), while allowing partial subsystem dicts on the `diagram.params` property setter (matching `SumCost.params` and `PlanningProblem.params`, and updating the corresponding assertion in `tests/unittest/test_contract.py`).

#### Proposed code diff
- `minilink/core/wiring.py`:
```diff
--- a/minilink/core/wiring.py
+++ b/minilink/core/wiring.py
@@ -93,9 +93,14 @@ class WiredDiagramMixin:
     @params.setter
     def params(self, value: dict) -> None:
-        check_diagram_params(self, value)
-        for sys_id, sub in self.subsystems.items():
-            sub.params = value[sys_id]
+        if not isinstance(value, dict):
+            raise TypeError(
+                f"Diagram params must be a dict keyed by subsystem id; got {type(value).__name__}."
+            )
+        unknown = [k for k in value if k not in self.subsystems]
+        if unknown:
+            raise ValueError(
+                f"Unknown subsystem id(s) {unknown} in diagram params; "
+                f"known subsystems: {list(self.subsystems)}."
+            )
+        for sys_id, sub_params in value.items():
+            self.subsystems[sys_id].params = sub_params
```

---

## CA-5. Export core mathematical types on `minilink.core` (and optional root prelude `minilink`)

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

## CA-6. Derivative method aliases (`grad_J`, `hess_J`, `jac_h`, `jac_g`) on `MathematicalProgramEvaluator`

### Problem and reproducer
On `MathematicalProgram` (`minilink/optimization/programs.py`), the 7 mathematical callables are named:
- Values: `J`, `h`, `g`
- Derivatives: `grad_J`, `hess_J`, `jac_h`, `jac_g`

When compiled via `ev = prog.compile(backend="jax")`, `MathematicalProgramEvaluator` (`minilink/optimization/evaluators/program_evaluator.py`) provides short value aliases `ev.J(z)`, `ev.h(z)`, `ev.g(z)` alongside the verbose names `ev.objective(z)`, `ev.equality_residual(z)`, `ev.inequality_margin(z)`, but only provides the verbose derivative names (`ev.gradient(z)`, `ev.hessian(z)`, `ev.jacobian_h(z)`, `ev.jacobian_g(z)`):
```python
ev = prog.compile(backend="jax")
ev.J(z)      # works!
ev.grad_J(z) # AttributeError: 'JaxProgramEvaluator' object has no attribute 'grad_J'
```

### Why it is worth doing & design/rule evolution
- **Why it is worth doing:** Since `MathematicalProgramEvaluator` already exposes `ev.J`, `ev.h`, and `ev.g` to match `MathematicalProgram`'s textbook notation, completing the set with `ev.grad_J`, `ev.hess_J`, `ev.jac_h`, and `ev.jac_g` makes `MathematicalProgram` and its compiled evaluator symmetric, while keeping `gradient`, `hessian`, `jacobian_h`, `jacobian_g` for internal solver adapters.
- **Design/Rule evolution:**
  - Note in `DESIGN.md` §6 that `MathematicalProgramEvaluator` mirrors `MathematicalProgram`'s callable names (`J`, `grad_J`, `hess_J`, `h`, `jac_h`, `g`, `jac_g`) alongside the solver-adapter methods (`objective`, `gradient`, `hessian`, `equality_residual`, `jacobian_h`, `inequality_margin`, `jacobian_g`).

### Proposed code diff
- `minilink/optimization/evaluators/program_evaluator.py`:
```diff
--- a/minilink/optimization/evaluators/program_evaluator.py
+++ b/minilink/optimization/evaluators/program_evaluator.py
@@ -94,6 +94,9 @@ class MathematicalProgramEvaluator(ABC):
     def gradient(self, z: np.ndarray, params=None) -> np.ndarray:
         raise NotImplementedError
 
+    def grad_J(self, z: np.ndarray, params=None) -> np.ndarray:
+        return self.gradient(z, params)
+
     @abstractmethod
     def hessian(self, z: np.ndarray, params=None) -> np.ndarray:
         raise NotImplementedError
 
+    def hess_J(self, z: np.ndarray, params=None) -> np.ndarray:
+        return self.hessian(z, params)
+
     def equality_residual(self, z: np.ndarray, params=None) -> np.ndarray:
@@ -112,6 +118,9 @@ class MathematicalProgramEvaluator(ABC):
     def jacobian_h(self, z: np.ndarray, params=None) -> np.ndarray:
         raise NotImplementedError
 
+    def jac_h(self, z: np.ndarray, params=None) -> np.ndarray:
+        return self.jacobian_h(z, params)
+
     def inequality_margin(self, z: np.ndarray, params=None) -> np.ndarray:
@@ -130,6 +139,9 @@ class MathematicalProgramEvaluator(ABC):
     def jacobian_g(self, z: np.ndarray, params=None) -> np.ndarray:
         raise NotImplementedError
 
+    def jac_g(self, z: np.ndarray, params=None) -> np.ndarray:
+        return self.jacobian_g(z, params)
```

---

## CA-7. Target / operating-point naming (`xbar` vs `x_bar` vs `center` vs `point`)

### Problem and reproducer
Across the core and analysis APIs, target/nominal state and input points use two spelling conventions:
- **Without underscore (`xbar`, `ubar`):** `QuadraticCost(Q, R, S, xbar=..., ubar=...)`, `TimeCost(xbar=..., tol=...)`, `LTISystem.xbar` / `ubar` / `ybar`, `QuadraticFeatures(xbar=...)`.
- **With underscore (`x_bar`, `u_bar`):** `sys.jacobian(x_bar=..., u_bar=...)`, `sys.linearize(x_bar=..., u_bar=...)`, `sys.transfer_function(x_bar=..., u_bar=...)`, `sys.plot_bode(x_bar=..., u_bar=...)`.
- **Geometric nouns (`center`, `point`):** `SingletonSet(point)`, `BallSet(center, radius)`, `EllipsoidSet(center, P, level=1.0)`, `QuadraticField(center, P, t=None)`.

### Why it is worth doing & design/rule evolution
- Keeping geometric nouns (`center`, `point`) on sets and fields is natural, whereas having `xbar`/`ubar` on `QuadraticCost`/`LTISystem` and `x_bar`/`u_bar` on `linearize`/`jacobian` is a frequent keyword typo when moving between `sys.linearize(...)` and `QuadraticCost(...)`.
- Tracked in [QUESTIONS.md](../dev/QUESTIONS.md) for the v0.9 API freeze (`DESIGN.md` §4). Can either be resolved repo-wide at the v0.9 freeze or softened now by accepting optional `x_bar` / `u_bar` keyword aliases in `QuadraticCost.__init__` and `TimeCost.__init__` while keeping `.params` keys `"xbar"` and `"ubar"`.

