# Improvement scan — raw ledger (2026-09-22)

**Status:** unverified finder output, kept so the scan is not lost. The
triaged propositions are in
[2026-09-22-improvement-suggestions.md](2026-09-22-improvement-suggestions.md);
delete this file once every item there has a workboard row, a decision, or a
recorded rejection. One finder per band, no cross-check; line numbers are as
of `dev` at `8a7226c`. Ids match the appendix of the propositions document.


## core

### core#0 — Bubble smallest_time_constant to the diagram root

*bug · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/core/wiring.py:250-257`, `minilink/simulation/simulator.py:255-262`, `minilink/core/system.py:100-104`, `minilink/dynamics/catalog/vehicles/racecar.py:284`, `minilink/blocks/nonlinear.py:139`

**What.** In `WiredDiagramMixin.refresh_solver_info`, set `solver_info["smallest_time_constant"] = min(sub.solver_info["smallest_time_constant"] for sub in subsystems)` beside the existing `discontinuous_behavior = any(...)`. One test asserts `Simulator(ctl @ plant).t` has the same automatic grid as `Simulator(plant).t`.

**Why.** A plant's solver hint (set by the student, or by catalog plants such as the racecar and RateLimiter) is lost the moment the plant enters a loop; the loop silently integrates on the `System` default 1e-3 × scale — 500× more steps for a slow plant, or an under-resolved fast block.

**Evidence.** `refresh_solver_info` docstring says "Bubble subsystem solver hints" (plural) but wiring.py:254-257 copies only `discontinuous_behavior`. Measured: Pendulum with hint 0.5 → `Simulator(plant, solver="euler", tf=1)` auto dt 0.05 (21 points); `Simulator(K @ plant, ...)` auto dt 1e-4 (10001 points), root hint 0.001. simulator.py:261 reads only the root's `solver_info`.

### core#1 — Reject evolution-kind mismatches in add_subsystem

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/core/wiring.py:242-248`, `minilink/core/diagram.py:59-61`, `minilink/core/diagram.py:210-212`, `minilink/core/compile/compiler.py:251-256`, `minilink/core/compile/step_compiler.py:50-55`

**What.** `DiagramSystem.add_subsystem` raises `TypeError("a StepSystem belongs in a StepDiagramSystem or a Computer (block % dt @ plant)")` for a step block, and `StepDiagramSystem.add_subsystem` raises for a `DynamicSystem` with `n > 0`; the two identical checks in the compilers become redundant.

**Why.** RULES 4.10: wiring mistakes fail at wiring time. Today the reference `f` silently returns a short vector (its state slots are allocated but skipped) and the error only arrives at compile, naming the compiler rather than the fix.

**Evidence.** `dg = DiagramSystem(); dg.add_subsystem(Pendulum(), "p"); dg.add_subsystem(Counter(), "c")` (Counter a StepSystem, n=1) → `dg.n == 3`, `dg.f(zeros(3), u, 0).shape == (2,)` because diagram.py:59-61 skips non-DynamicSystem members while wiring.py:410-412 allocates their slots; `dg.compile()` → "StepSystem leaf 'c' cannot be compiled inside a flow DiagramSystem".

### core#2 — Probe undeclared feedthrough at compile time

*trap · effort M · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/core/compile/compiler.py:49-95`, `minilink/core/wiring.py:510-535`, `minilink/core/system.py:445`, `minilink/core/system.py:458-462`, `minilink/core/signals.py:252-260`

**What.** After the shape probe in `validate_equation_shapes`, perturb each input port that is *not* in an output port's `dependencies` at `(x0, u0)`; if the output moves, raise (or warn — maintainer's pick) `ValueError("y of 'Plant' changes with input 'u' but declares dependencies=(); pass y_dependencies=('u',) or 'all'")`.

**Why.** `y_dependencies=()` is the constructor default, so the first plant a student writes whose `h` reads `u` compiles, escapes algebraic-loop detection, and simulates with `u` pinned to its nominal value inside `h` — a wrong answer with no message.

**Evidence.** Plant with `h = x + u` and default deps; `ProportionalController(K=2) @ plant` compiles; `compute_internal_signals_dict` at x=1, r=0 returns `sys:y = 1.0`, `ctl:u = -2.0` (true fixed point y = 1/3, u = -2/3). wiring.py:524-525 substitutes `port.get_default_value()` for ports outside `dependencies`; the probe at compiler.py:79-95 checks shapes only.

### core#3 — Check per-block params dicts against the block's keys

*trap · effort S · owner maintainer · rung v0.2 wave A · planned: A2*  
Files: `minilink/core/wiring.py:24-55`, `minilink/core/wiring.py:481-487`, `minilink/core/wiring.py:489-503`, `DESIGN.md:413-421`

**What.** `validate_diagram_params` also checks that each per-block dict carries exactly the block's live `params` keys (skip blocks whose params are empty), raising `ValueError("params for 'sys' replace the whole block dict: missing l, g, d")`; one test through the setter, one through `f(params=...)`.

**Why.** DESIGN says a per-block dict is a full replacement, but the failure a student meets is `KeyError: 'l'` deep inside the pendulum's `f`, or, through the setter, a plant silently left with `{'m': 2.0}` that fails at the next simulation. A2 will add nested cost params under the same rule, so the check should be written once now.

**Evidence.** `loop.f(x, u, 0, params={"sys": {"m": 2.0}})` → `KeyError 'l'`; `loop.params = {"sys": {"m": 2.0}}` is accepted and `plant.params == {'m': 2.0}` afterwards. `validate_diagram_params` checks only unknown sys ids (wiring.py:49-55).

### core#4 — Carry discount_rate as a field that composites keep

*api · effort M · owner maintainer · rung v0.2 wave A · planned: A2*  
Files: `minilink/core/costs.py:29-36`, `minilink/core/costs.py:54-56`, `minilink/core/costs.py:72-76`, `minilink/core/costs.py:112-126`, `minilink/core/costs.py:261-326`, `minilink/planning/policy_synthesis/dp.py:653`, `minilink/planning/reinforcement_learning/planner.py:385`, `minilink/interfaces/gymnasium.py:176`

**What.** `discount_rate` becomes a constructor field (`QuadraticCost.from_system(sys, discount_rate=0.5)`, default 0) or, under A2, a `params` entry; `ScaledCost` forwards `cost.discount_rate`, `SumCost.of` takes the members' common rate and raises when they differ. `inspect_text` already prints `rho`.

**Why.** Today the only way to declare a discount is a subclass with a class attribute (tutorial 11 and four tests do this), and the declaration is lost by `2 * cost` and `a + b`, which every planner, `total_cost` and the gym adapter then read as undiscounted.

**Evidence.** costs.py:29-30: "override by assignment in a subclass or instance", but every library cost is `@dataclass(frozen=True)` (costs.py:129, 219, 261, 300) → `c.discount_rate = 0.5` raises `FrozenInstanceError`; with `Disc.discount_rate = 0.5`, `(2.0 * d).discount_rate == 0.0` and `(d + d).discount_rate == 0.0`; `evaluate_trajectory` reads `self.discount_rate` at costs.py:73.

### core#5 — Give Distribution.sample the sets' key=None default

*api · effort S · owner maintainer · rung v0.2 wave A · planned: A5*  
Files: `minilink/core/distributions.py:366-374`, `minilink/core/distributions.py:388`, `minilink/core/distributions.py:414`, `minilink/core/distributions.py:431`, `minilink/core/distributions.py:463`, `minilink/core/sets.py:54-64`, `minilink/core/sets.py:167`, `minilink/core/backends.py:609-613`

**What.** `sample(self, key=None, n=None)` on `Distribution` and the four laws, so an unseeded draw works as it does on `BoxSet`; DESIGN §4's distributions bullet gains "`None` (unseeded)".

**Why.** The two siblings share one draw convention by contract (DESIGN: "the same `sample` convention holds on sets"), yet `Gaussian([0], 1).sample()` is a `TypeError` while `BoxSet([0],[1]).sample()` draws; a student learns the convention from whichever object they meet first.

**Evidence.** `Gaussian.sample() missing 1 required positional argument: 'key'`; `BoxSet([0],[1]).sample()` → `[0.0615]`; `numpy_generator(None)` already returns `default_rng(None)` (backends.py:609-613), so the change is the default alone.

### core#6 — Flatten Shape unions like IntersectionSet and SumCost

*consolidation · effort S · owner agent · rung v0.2 wave A · planned: A3*  
Files: `minilink/core/geometry.py:39-41`, `minilink/core/geometry.py:126-157`, `minilink/core/sets.py:353-362`, `minilink/core/costs.py:272-281`

**What.** Add `Union.of(*shapes)` that flattens nested unions and use it in `Shape.__or__`, mirroring `IntersectionSet.of` and `SumCost.of`; `sdf` values are unchanged.

**Why.** The three algebras (`&` on sets, `+` on costs, `|` on shapes) should behave alike; a flat member list is what a scene overlay or a future parametric `sdf` iterates, and it is one fewer surprise when A3 grows the package.

**Evidence.** `Sphere | Sphere | Sphere` → `Union` whose `shapes` are `['Union', 'Sphere']`; `a & b & c` → one flat `IntersectionSet` of three members (sets.py:357-362).

### core#7 — Give StepRollout and HybridSimResult a print, one validator with Trajectory

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: D1.1*  
Files: `minilink/core/step_rollout.py:30-110`, `minilink/core/trajectory.py:46-135`, `minilink/core/inspect.py:22-70`, `minilink/simulation/hybrid_simulator.py:286-320`

**What.** `StepRollout.__str__` / `_repr_pretty_` through `inspect_text` ("StepRollout, N=4, k=0–3, x (1, 4), u (0, 4)") and a `HybridSimResult` row (plant span, tick count); the identical validation of the two containers becomes one `validate_sampled_arrays(axis, x, u, signals)` helper both `__post_init__` call.

**Why.** RULES 6.1 sends reporting through `print(obj)`; the step and hybrid paths dump raw arrays. Two copies of the shape validation already drift: `Trajectory` checks a monotone `t`, `StepRollout` checks nothing about `k`.

**Evidence.** `print(Counter().compute_rollout(3))` → `StepRollout(k=array([0., 1., 2., 3.]), x=array([[0., 1., 2., 3.]]), u=array([[0., 0., 0., 0.]]), signals=mappingproxy({}))`; `comm` of the two `__post_init__` regions: 42 identical lines; no `__str__` in step_rollout.py or hybrid_simulator.py.

### core#8 — Let each textbook object own its print text

*consolidation · effort M · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/core/inspect.py:6-21`, `minilink/core/inspect.py:57-70`, `minilink/core/inspect.py:105-141`, `DESIGN.md:129-147`

**What.** `inspect_text` dispatches to an `inspect_summary()` method (or a small registry the planning and optimization modules fill on import) and keeps only the core rows; `_problem_text` and `_program_text` move beside `PlanningProblem` and `MathematicalProgram`.

**Why.** `print(sys)` on a bare `System` imports `minilink.optimization` and six `minilink.planning.spatial` modules — core importing tool bands, against DESIGN §3 "tools import core" — and every new textbook object needs an `isinstance` row here; A1 and A3 will move `StateField` / `FieldSet`, so this ladder is edited on every wave.

**Evidence.** Modules newly imported by `str(System())`: `minilink.optimization.mathematical_program`, `minilink.planning.problems`, `minilink.planning.spatial.{collision,paths,scene,shaping,state_fields,track}`; inspect.py:6-21 holds the lazy imports, 57-70 the planning/optimization rows.

### core#9 — Export the operators' named forms together

*docs · effort S · owner maintainer · rung v0.2 wave A · planned: —*  
Files: `minilink/core/__init__.py:30-40`, `minilink/__init__.py:174`, `README.md:32`, `README.md:122-125`, `minilink/core/system.py:398-410`

**What.** Move `closed_loop_qdq` out of the `# sets` rows into a `# composition` group with `closed_loop`, `series` and `feedback` in `minilink.core`; at the root either export `closed_loop` and `series` beside `feedback` or drop `feedback` (maintainer's pick), so the named forms of `@`, `>>` and `sys @ 1` sit in one import layer.

**Why.** README teaches the operators, and `help(plant.__matmul__)` points at `closed_loop` and `feedback`, of which only `feedback` is importable from the root and neither from `minilink.core`; the misplaced row shows the table has no owner.

**Evidence.** `minilink.__all__`: feedback True, series False, closed_loop False, closed_loop_qdq True; `minilink.core.__all__`: closed_loop_qdq True (listed between `BallSet` and `SingletonSet` at core/__init__.py:38), feedback / series / closed_loop False.

### core#10 — One verbose default and one empty-cache rule across compute_* facades

*api · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/core/facades.py:66-77`, `minilink/core/facades.py:456-468`, `minilink/core/facades.py:1282-1291`, `minilink/core/facades.py:1324-1352`, `minilink/core/facades.py:228-234`, `minilink/core/hybrid_diagram.py:143-153`

**What.** `compute_rollout(verbose=…)` takes the same default as `compute_trajectory` (both `True`, the interactive default DESIGN documents, or both `False`), and `plot_rollout()` with no cached rollout computes one the way `plot_trajectory()` does instead of raising.

**Why.** RULES 4.7 unifies flag behaviour; the step path is the one place the same verb behaves differently, and tutorial 06 puts the two side by side.

**Evidence.** Signature defaults: `DynamicSystemFacades.compute_trajectory verbose=True`, `SharedSystemFacades.compute_trajectory verbose=True`, `HybridDiagram.compute_trajectory verbose=True`, `StepSystemFacades.compute_rollout verbose=False`; facades.py:1349-1352 raises "No rollout available" while facades.py:233-234 calls `compute_trajectory`.

### core#11 — One owner for the initial state: x0 or state.nominal_value

*consolidation · effort M · owner maintainer · rung v1.0 · planned: S29*  
Files: `minilink/core/system.py:86`, `minilink/core/system.py:93-94`, `minilink/core/wiring.py:429-441`, `minilink/core/signals.py:199-217`

**What.** Make `state.nominal_value` a view of `x0` (or stop concatenating it for diagrams); with S29 the diagram's `x0` becomes a live property and the `refresh()` before every simulate goes away.

**Why.** RULES 5.6, one owner per quantity: `x0` and `state.nominal_value` both mean the nominal state, nothing in the library reads the latter, and they disagree as soon as a plant sets `x0`. S29 speaks of `x0` / `n` / `state` as derived properties but not of this second copy.

**Evidence.** `p.x0 = [1, 0]; loop = K @ p` → `loop.x0 == [1, 0]` but `loop.state.nominal_value == [0, 0]` (wiring.py:438-441 concatenates both); grep of `state.nominal_value` outside signals.py/wiring.py: no reader; `loop.x0` is stale after `p.x0 = [0.7, 0]` until `loop.refresh()`.

### core#12 — One keyword vocabulary for the four feedback wires

*api · effort S · owner maintainer · rung v0.2 wave B · planned: P10*  
Files: `minilink/core/hybrid_composition.py:24-37`, `minilink/core/composition.py:284-296`, `minilink/core/composition.py:889-912`, `minilink/core/composition.py:1128-1139`

**What.** `hybrid_closed_loop` takes the `closed_loop` names (`measurement_port`, `control_port`, `plant_input_port`, `plant_output_port`) — or both take the resolver's (`measurement_in`, `control_out`, `plant_in`, `plant_out`) — and P10's DESIGN paragraph lists them once; the five `hybrid_closed_loop` call sites under examples/ update in the same change.

**Why.** The same four wires have three names today, so a student moving from `ctl @ plant` (tutorial 03) to `ctl % dt @ plant` (tutorial 06) relearns the keywords; P10 documents the dispatch paths but does not say the keywords should agree.

**Evidence.** `closed_loop(ref_port, measurement_port, control_port, plant_input_port, plant_output_port, output_port)`; `hybrid_closed_loop(computer_out, plant_in, plant_out, computer_in, ref_port, output_port)`; `StandardFeedbackWiring(control_out, measurement_in, plant_in, plant_out)`.

### core#13 — Refuse to rewire a connected subsystem input

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/core/wiring.py:288-356`, `minilink/core/wiring.py:347-350`, `minilink/core/composition.py:199-202`, `minilink/core/composition.py:573-576`, `minilink/control/mpc/controller.py:860`

**What.** `connect` raises `ValueError("i:u is already fed by a:y; set diagram.connections['i']['u'] = None first")` when the target is a subsystem input that already has a source; boundary `"output"` targets keep replace semantics, which the `>>` chain uses to move `y`.

**Why.** RULES 4.10 early structural validation and `autowire`'s own promise never to overwrite: a mistyped second `connect` silently steals the wire and the only symptom is a wrong simulation.

**Evidence.** `connect("a","y","i","u"); connect("b","y","i","u")` → `connections["i"]["u"] == ("b","y")`, no message (wiring.py:347-350 is a plain dict write). `feedback()` is the one library site that rewires and it clears the edge first (composition.py:199-202); the only other library `connect` is mpc/controller.py:860 on a fresh port.

### core#14 — Decide the core → simulation import with S31

*consolidation · effort L · owner maintainer · rung v1.0 · planned: S31*  
Files: `minilink/core/hybrid_composition.py:11-19`, `minilink/core/system.py:412-416`, `minilink/core/hybrid_diagram.py:62`, `minilink/core/hybrid_diagram.py:93-97`, `minilink/simulation/computer.py:40-43`, `DESIGN.md:129-147`

**What.** When S31 settles the sampled loop's home, also settle where `Computer` / `StepSchedule` live: either they move under `core/` (so `%` and `hybrid_composition` import downward) or `%` becomes a `simulation` verb; today core is the one package importing a tool band.

**Why.** DESIGN §3: "Tools import core". The reverse edge is why `simulation/computer.py` imports back into core lazily and why `hybrid_composition` needs a private cross-import from `composition`; S31 as written decides the type, not the import.

**Evidence.** hybrid_composition.py:19 `from minilink.simulation.computer import Computer, StepSchedule` (eager, module level); system.py:414 `from minilink.simulation.computer import as_computer`; computer.py:40-43 imports `minilink.core.hybrid_composition._as_step_diagram`.

### Bugs reported by the core finder

#### bug core#0 — (medium) `L @ plant` extends a shortcut-built left diagram in place

Files: `minilink/core/composition.py:333-338`, `minilink/core/composition.py:521-529`, `minilink/core/composition.py:150-154`, `minilink/core/composition.py:164-166`, `minilink/core/system.py:398-410`, `tests/unittest/test_feedback_composition.py:69-78`

```text
from minilink import Gain, Pendulum
L = Gain(2.0, dim=1) >> Gain(3.0, dim=1)
list(L.subsystems)            # ['gain', 'gain2']
loop = L @ Pendulum()
list(L.subsystems)            # ['gain', 'gain2', 'sys']  -- the user's L now contains the plant

`closed_loop` routes an error-driven left operand through `series(controller, plant)` (composition.py:337-338); `series` extends a diagram left operand in place (`_as_composition_diagram` returns it, 521-523). The frozen mutation rule (DESIGN §4) names only `+` and `>>`; `feedback()`'s docstring says "sys is not modified" and the `@` docstring names no mutation. The existing test covers only `L @ 1`, which does not mutate.
```

#### bug core#1 — (medium) `discount_rate` cannot be set on an instance and composite costs drop it

Files: `minilink/core/costs.py:29-36`, `minilink/core/costs.py:73`, `minilink/core/costs.py:112-126`, `minilink/core/costs.py:129`, `minilink/core/costs.py:261-326`, `minilink/planning/policy_synthesis/dp.py:653`, `minilink/planning/reinforcement_learning/planner.py:385`, `minilink/interfaces/gymnasium.py:176`

```text
from minilink import QuadraticCost, Pendulum
c = QuadraticCost.from_system(Pendulum())
c.discount_rate = 0.5                 # FrozenInstanceError: cannot assign to field 'discount_rate'

class Disc(QuadraticCost):
    discount_rate = 0.5
d = Disc.from_system(Pendulum())
(2.0 * d).discount_rate, (d + d).discount_rate   # (0.0, 0.0)

The base docstring promises "override by assignment in a subclass or instance"; all library costs are frozen dataclasses. `ScaledCost` / `SumCost` inherit the class default, and `evaluate_trajectory`, `total_cost`, `discount_factor`, DP, RL and the gym adapter read the composite's rate, so `base + w * obstacle` is scored undiscounted.
```

#### bug core#2 — (low) `refresh_solver_info` drops `smallest_time_constant`, so a loop ignores its plant's solver hint

Files: `minilink/core/wiring.py:250-257`, `minilink/simulation/simulator.py:255-262`

```text
import numpy as np
from minilink import Pendulum, StateFeedbackController, Simulator
p = Pendulum(); p.solver_info["smallest_time_constant"] = 0.5
K = StateFeedbackController(np.array([[10.0, 2.0]]))
Simulator(p, tf=1.0, solver="euler").t.size          # 21     (auto dt 0.05)
Simulator(K @ p, tf=1.0, solver="euler").t.size      # 10001  (auto dt 1e-4; root hint stays 0.001)

Only `discontinuous_behavior` is bubbled (wiring.py:254-257); the docstring says "hints". Explicit `dt` / `n_steps` are unaffected.
```

#### bug core#3 — (low) `IntersectionSet.margin` fails on a member whose margin returns a scalar

Files: `minilink/core/sets.py:377-386`, `minilink/core/sets.py:317-322`, `minilink/core/sets.py:51`

```text
import numpy as np
from minilink.core.sets import BoxSet, CallableSet
free = CallableSet(margin_fn=lambda z, t, p: 1.0 - float(z[0]))
(BoxSet([0], [2]) & free).margin(np.array([0.5]))
# AttributeError: 'float' object has no attribute 'reshape'

`Set.contains` tolerates a scalar margin through `np.asarray` (sets.py:51) but `IntersectionSet.margin` calls `.reshape(-1)` on the raw member result (sets.py:383); `xp.asarray(...).reshape(-1)` fixes it. DESIGN teaches `sys.state.box & free` as the standard composition.
```

#### bug core#4 — (low) `DiagramSystem.add_subsystem` accepts a StepSystem and the reference `f` returns a short vector

Files: `minilink/core/wiring.py:242-248`, `minilink/core/diagram.py:59-61`, `minilink/core/wiring.py:410-412`, `minilink/core/compile/compiler.py:251-256`

```text
import numpy as np
from minilink import DiagramSystem, Pendulum, StepSystem
class Counter(StepSystem):
    def __init__(self): super().__init__(n=1, input_dim=1, output_dim=1)
    def step(self, x, u, k=0, params=None): return x + 1
dg = DiagramSystem(); dg.add_subsystem(Pendulum(), "p"); dg.add_subsystem(Counter(), "c")
dg.n                                              # 3
dg.f(np.zeros(3), np.zeros(dg.m), 0.0).shape      # (2,)  -- silently short
dg.compile()                                      # TypeError only here

The mirror case (a `DynamicSystem` with n > 0 in a `StepDiagramSystem`) is likewise caught only by `compile_step_diagram`.
```


## compile-simulation

### compile-simulation#0 — Give the try-JAX-then-NumPy policy one owner and a typed error

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/core/compile/compiler.py:183-200`, `minilink/simulation/compile_backend.py:12-28`, `minilink/simulation/realtime/simulator.py:378-396`, `minilink/core/compile/evaluators/jax_evaluators.py:1569-1584`

**What.** Keep `compile_auto(system, verbose)` in `compiler.py` as the single policy; make `resolve_auto_backend` and `RealtimeSimulator._resolve_and_build_evaluator` call it (or delete `compile_backend.py`). Have `check_jax_compatible` raise a `NotTraceableError(RuntimeError)` so `compile_auto` catches the type instead of matching the substring "JAX-traceable" in the message.

**Why.** Three copies of one policy already disagree: `compile_auto` falls back only on ImportError or a not-traceable RuntimeError and surfaces every other error; `resolve_auto_backend` (used by `Simulator` and `StaticSimulator`) swallows *any* exception into a debug log; the realtime copy is a verbatim paste of the second. A student gets different behaviour for the same broken `f` depending on which verb they call, and a message-string coupling between two modules breaks silently on a reword.

**Evidence.** compiler.py:197-199 `except RuntimeError as exc: if "JAX-traceable" not in str(exc): raise`; compile_backend.py:24 `except Exception:` then `logging...debug(...)`; realtime/simulator.py:384-396 repeats compile_backend.py line for line; jax_evaluators.py:1575 `except (ConcretizationTypeError, TypeError, Exception)` (redundant tuple) builds the string the other module greps.

### compile-simulation#1 — Give the automatic dt one owner and make the discontinuous scale mean something (or drop the claim)

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: D3*  
Files: `minilink/simulation/simulator.py:90-92`, `minilink/simulation/simulator.py:255-262`, `minilink/simulation/realtime/simulator.py:398-405`, `minilink/simulation/solver_warnings.py:7`, `minilink/simulation/solver_warnings.py:54-55`, `DESIGN.md:826-831`

**What.** One function `auto_dt(sys) -> float` in `simulation/time_grid.py` (`smallest_time_constant × scale`, scale by `discontinuous_behavior`), called from `Simulator.select_time_vector`, `RealtimeSimulator._offline_auto_dt`, `collect_discontinuous_solver_notes` and the proposed hybrid default; delete `_DISCONTINUOUS_AUTO_DT_SCALE`. Then either set `DISCONTINUOUS_AUTO_DT_SCALE` to a genuinely finer value (e.g. 0.02) with a regression-baseline update, or rewrite DESIGN §5 and the `select_time_vector` docstring so they stop promising a finer dt.

**Why.** D3 lists the duplicated constant; the duplicated thing is the whole policy (three computations). Worse, the two scales are both 0.1, so the documented mitigation "auto `select_solver` picks Euler with finer default dt" is vacuous: a discontinuous loop gets exactly the smooth grid. A student reading DESIGN believes a protection exists that does not.

**Evidence.** simulator.py:91-92 `SMOOTH_AUTO_DT_SCALE = 0.1` / `DISCONTINUOUS_AUTO_DT_SCALE = 0.1` (equal since they were introduced, git -S shows no other value); simulator.py:256-261, realtime/simulator.py:400-405 and solver_warnings.py:54-55 each recompute `smallest_time_constant * scale`; DESIGN.md:829 "auto select_solver picks Euler with finer default dt"; simulator.py:250-252 "scaled by the smooth or discontinuous policy".

### compile-simulation#2 — Make the forced-input hold model one option honoured by every solver

*api · effort M · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/solvers/rk4_fixed.py:54`, `minilink/simulation/solvers/euler_fixed.py:64`, `minilink/simulation/solvers/euler.py:55`, `minilink/simulation/solvers/scipy_ivp.py:89`, `minilink/simulation/input_interpolation.py:16`, `minilink/simulation/simulator.py:32-76`

**What.** Add `input_interp="linear" | "zoh"` to `Simulator` / `compute_forced` (the key `INPUT_INTERP_KEY` already exists in `scipy_ivp.py` but no preset sets it, so it is unreachable), thread it into `solver_backend_options`, and have every backend obey it: RK4 fixed uses `rk4_integrate_linear` for linear and `rk4_integrate_zoh` for zoh; the Euler backends gain the linear case or document that they are ZOH-only. State the rule in DESIGN §5 next to the presets.

**Why.** Today the same `solve_forced(u)` call means a different input model per solver: `rk4_fixedsteps` and `scipy` interpolate `u` linearly between knots, `euler` and `euler_fixedsteps` hold it. A student comparing solvers on one forced input attributes the difference to integration error when it is the input model.

**Evidence.** Probe on an `Integrator` with a unit step at t=0.5, dt=0.1: x(1) = 0.5000 (euler), 0.5000 (euler_fixedsteps), 0.5500 (rk4_fixedsteps), 0.5495 (scipy). rk4_fixed.py:54 `evaluator.rk4_integrate_linear(x0, u.T, ...)`; euler_fixed.py:64 `evaluator.euler_integrate_zoh(...)`; scipy_ivp.py:89 `scheme = kw.pop(INPUT_INTERP_KEY, "linear")` with no preset in simulator.py:32-76 carrying that key (repo-wide grep finds no other use).

### compile-simulation#3 — State and test what `n_steps` counts on each simulation verb

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/simulator.py:126-127`, `minilink/core/facades.py:1282-1302`, `minilink/simulation/hybrid_simulator.py:36-37`, `minilink/core/compile/evaluators/jax_evaluators.py:348-372`, `DESIGN.md:771-779`

**What.** Add one sentence to DESIGN §5 and to each docstring: `Simulator(n_steps=N)` samples N points (N-1 steps, endpoints included); `compute_rollout(n_steps=N)`, `HybridSimulator(n_steps=N)` and `rollout_batch(n_steps=N)` apply N transitions (N+1 samples, or N ticks). Pin each with one parametrized test in `test_simulation.py`. Renaming is off the table during the GRO860 name freeze; documenting is not.

**Why.** The same keyword means "points" on the continuous verb and "transitions" on the three discrete ones. A student who writes `n_steps=100` on both gets 100 and 101 samples and a dt of 1/99 instead of 1/100.

**Evidence.** Probe: `Simulator(Integrator(), tf=1.0, n_steps=100).n_pts == 100` with `dt = 0.010101...`; `acc.compute_rollout(n_steps=100).k.size == 101`; `HybridSimulator(..., n_steps=100).n_ticks == 100`. simulator.py:126 "Number of time samples (including endpoints)"; facades.py:1301 "Number of step transitions to apply".

### compile-simulation#4 — Derive the hybrid plant sub-step from the plant time constant instead of one RK4 step per tick

*trap · effort M · owner maintainer · rung v0.2 wave D · planned: D3*  
Files: `minilink/simulation/hybrid_simulator.py:56`, `minilink/simulation/hybrid_simulator.py:187-193`, `minilink/simulation/simulator.py:255-262`

**What.** When `plant_dt_inner is None`, default it to `min(dt_base, auto_dt(plant))` using the same policy as `Simulator.select_time_vector`, and `warnings.warn` once in domain units (RULES 4.12: "plant sub-stepped at 1e-4 s inside the 0.01 s tick") when the derived step is finer than the tick; keep an explicit `plant_dt_inner` silent. Update the hybrid regression baseline in the same change.

**Why.** The offline `Simulator` protects a fast plant automatically (it picks scipy or a dt of 1e-5 for a 1 ms time constant); the hybrid path integrates the same plant with one RK4 step per computer tick and diverges silently. A student who moves a working continuous loop to `ctl % dt @ plant` sees numbers explode with no warning and no obvious knob.

**Evidence.** Probe: first-order plant with `smallest_time_constant = 0.001`, `hybrid_closed_loop(P(0.0), plant, schedule=0.01)`, `HybridSimulator(tf=0.1).solve()` → `plant.x[0, -1] == 4.35e24`, no warning; `Simulator(plant, tf=0.1)` auto-selects `scipy`, dt=1e-05. hybrid_simulator.py:187-193 passes `dt_inner=self.plant_dt_inner` (None → one step, numpy_evaluators.py:281-283); `solver_info` is never read in hybrid_simulator.py.

### compile-simulation#5 — Align HybridSimulator with its siblings: shared input coercion, positional `input_port_id`, the framed verbose panel

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: D3*  
Files: `minilink/simulation/hybrid_simulator.py:114-128`, `minilink/simulation/hybrid_simulator.py:572-632`, `minilink/simulation/input_coercion.py:12-53`, `minilink/simulation/hybrid_simulator.py:97-101`, `minilink/simulation/sim_reporting.py:10-55`, `minilink/simulation/hybrid_simulator.py:518-542`

**What.** `HybridSimulator.solve_forced(u, input_port_id=None)` calls `coerce_forced_input(self.computer.diagram, self.times, u, input_port_id)` (the diagram already has `m`, `inputs`, `get_u_from_input_ports`, `get_input_port_slice`) and the 60-line private copy is deleted; `verbose=True` prints `print_simulation_preamble` / `print_simulation_report` like `Simulator` and `RealtimeSimulator`; the never-taken `plant_signals=True` branch and `_collect_signal_names` go.

**Why.** D3 says "HybridSimulator conventions drift" without naming the drift. Concretely: a second owner of forced-input coercion (RULES 5.6), a keyword-only `input_port_id` that breaks the positional call the other two simulators accept, a one-line `verbose` where the others print a panel, and a dead branch.

**Evidence.** Probe: `Simulator.solve_forced(u, 'u')` works; `HybridSimulator.solve_forced(u, 'r')` → `TypeError: takes 2 positional arguments but 3 were given` (hybrid_simulator.py:114 `def solve_forced(self, u, *, input_port_id=None)`). hybrid_simulator.py:597-632 duplicates input_coercion.py:56-100 (`_coerce_forced_signal`, `_sample_forced_callable`). `_allocate_signal_hist` is only ever called with `plant_signals=False` (line 152-154), so lines 518-519 and 536-537 are dead. hybrid_simulator.py:97-101 prints one line versus simulator.py:224-241.

### compile-simulation#6 — Pin (or reconcile) the one-tick lag between chained blocks inside a Computer

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/computer.py:100-111`, `minilink/simulation/computer.py:201-229`, `minilink/core/compile/evaluators/numpy_evaluators.py:624-633`, `DESIGN.md:226-262`

**What.** A test that builds `Gain >> Gain` as a `StepDiagramSystem`, asserts `evaluator.outputs(u)` gives the composed gain at k=0 while `Computer.tick(u)` gives 0 at k=0 and the composed gain from k=1, and one DESIGN sentence in the hybrid contract naming the rule ("inside a Computer every internal wire carries one base-tick delay; the synchronous evaluator has none"). If the maintainer prefers direct feedthrough within a tick (the Simulink convention, and what `compute_rollout` already does), the fix is to gather from `write` for ports already computed this tick, an M-sized change.

**Why.** The `Computer` docstring says it uses "the same StepExecutionPlan lowering as synchronous compute_rollout", but the two disagree on the same diagram: `compute_rollout` evaluates ports in topological order within a step, `tick` reads only the previous tick's committed buffer. A digital `Error >> PID` built inside one computer therefore acts on a one-sample-old error, and nothing in DESIGN or the tests says so.

**Evidence.** Probe: `StepDiagramSystem` with `Gain(2)` → `Gain(3)`: `ev.outputs(u=1) == {'y': [6.]}`; `computer.tick(u=1)` returns `{'y': [0.]}` then `{'y': [6.]}`. computer.py:108-111 documents the read-buffer rule; computer.py:4-6 claims the same lowering; DESIGN mentions "parallel tick semantics" only inside the MPC dual-rate paragraph (DESIGN.md:361); no test in test_hybrid.py or test_step_discrete.py chains two blocks under a Computer.

### compile-simulation#7 — Make `has_trace_tier` a plain boolean on both backends and correct the frozen-subset sentence

*api · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/core/compile/evaluators/tiers.py:11-25`, `minilink/core/compile/evaluators/tiers.py:28-33`, `minilink/planning/trajectory_optimization/transcription.py:59`, `DESIGN.md:711-720`

**What.** `NoTraceTierMixin.has_trace_tier = False` (class attribute), so `evaluator.has_trace_tier` is `True` on JAX and `False` on NumPy instead of an `AttributeError`; `transcription.py:59` then reads the attribute directly (the T5 ledger already flags that `getattr` probe). In DESIGN §5, replace "and the JAX trace-tier twins of those names" with the actual list: `f_trace`, `f_trace_p`, `outputs_trace(_p)`, `step_trace(_p)`, `rk4_step_trace(_p)`; `integrate_zoh*` and `rollout` have no trace twin.

**Why.** DESIGN says the flag "is True on JAX evaluators" and a caller reasonably writes `if ev.has_trace_tier:`; on NumPy that raises. The frozen subset is the contract external code is told to build on, so it should not name methods that do not exist.

**Evidence.** Probe: `compile(Integrator()).has_trace_tier` → `AttributeError: 'NumpyDynamicEvaluator' object has no attribute 'has_trace_tier'`; tiers.py:12-13 special-cases the name only to fall through to the generic error. `grep -rn "def integrate_zoh_trace\|def rollout_trace" minilink` is empty while DESIGN.md:714-716 lists `integrate_zoh` / `integrate_zoh_rollout` / `integrate_zoh_p` / `rollout` "and the JAX trace-tier twins of those names".

### compile-simulation#8 — Stop `simulation` importing from `optimization` for three display constants

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/sim_reporting.py:5-9`, `minilink/optimization/reporting.py:1-23`

**What.** Move `DISP_LINE_WIDTH`, `DISP_RULE_MAIN`, `DISP_RULE_DIV` and `preview_vector` to `minilink/core/reporting.py` (or `core/display.py`); `optimization/reporting.py` and `simulation/sim_reporting.py` both import from there.

**Why.** RULES 3.2: domain libraries import only from `core` and shared bases; peer bands do not import each other without architectural justification. A panel rule width is not a justification, and the import means `Simulator(verbose=True)` transitively loads the optimization band.

**Evidence.** sim_reporting.py:5 `from minilink.optimization.reporting import (DISP_RULE_DIV, DISP_RULE_MAIN, preview_vector)`; optimization/reporting.py:3-5 describes itself as shared by `Optimizer` and the trajopt planner only. `grep -rn "from minilink.optimization" minilink/simulation minilink/core minilink/analysis minilink/control` shows this as the only cross-band import outside a TYPE_CHECKING guard.

### compile-simulation#9 — Deduplicate the evaluator internals that S37 will otherwise carry: ZOH sugar, the JAX step rollout, the four `_jac_probe`s, the undeclared batch cache

*consolidation · effort M · owner agent · rung v1.0 · planned: S37*  
Files: `minilink/core/compile/evaluators/numpy_evaluators.py:244-292`, `minilink/core/compile/evaluators/jax_evaluators.py:606-655`, `minilink/core/compile/evaluators/jax_evaluators.py:1279-1291`, `minilink/core/compile/evaluators/jax_evaluators.py:1546-1558`, `minilink/core/compile/evaluators/numpy_evaluators.py:381-399`, `minilink/core/compile/evaluators/numpy_evaluators.py:570-588`, `minilink/core/compile/evaluators/jax_evaluators.py:971-992`, `minilink/core/compile/evaluators/jax_evaluators.py:1454-1472`, `minilink/core/compile/evaluators/jax_evaluators.py:402`, `minilink/core/compile/evaluators/jax_evaluators.py:819-834`

**What.** One `ZOHHoldMixin` (`integrate_zoh_rollout`, `integrate_zoh`, `integrate_zoh_p`, `_zoh_hold_sequence`) in `step_rollout.py` or `evaluators.py`, mixed into both backends since it only calls `self.rk4_integrate_zoh(_p)`; one `rollout` on a small JAX step mixin shared by `JaxStepEvaluator` and `JaxStepDiagramEvaluator`; one diagram `_jac_probe` in `JacobianMixin` parametrised by `signals_of` / `next_of`; `self._rollout_batch_cache = {}` declared in `_setup_integration_tiers` next to the other jit slots (RULES 5.7); drop `JaxDiagramEvaluator`'s redundant `try/except ImportError` (compile_diagram already called `require_jax_numpy()`) and its `_jax`/`_jnp` twins of `jax`/`jnp`. Seeded baseline, byte-identical.

**Why.** About 150 lines exist twice or four times; every fix to the ZOH hold or the wire-injection probe must be made in two or four places before S37 even starts. The batch cache created through `__dict__.setdefault` is the one attribute in the class not visible from its constructor.

**Evidence.** numpy_evaluators.py:244-292 and jax_evaluators.py:606-655 are line-for-line identical; jax_evaluators.py:1279-1291 and 1546-1558 identical; the four `_jac_probe` bodies differ only in the `dtype` argument; jax_evaluators.py:402 `cache = self.__dict__.setdefault("_rollout_batch_cache", {})`; jax_evaluators.py:819-828 wraps `import jax` in its own ImportError message after compiler.py:303 `require_jax_numpy()`; lines 831-834 `self._jax = jax; self._jnp = jnp; self.jax = jax; self.jnp = jnp`.

### compile-simulation#10 — Make the `scipy_stiff` preset honest: explicit tolerances and a Jacobian on both backends

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/simulator.py:32-76`, `minilink/simulation/solvers/scipy_ivp.py:60-64`, `minilink/core/compile/evaluators/evaluators.py:98-101`, `minilink/core/compile/evaluators/jacobian.py:351-404`

**What.** Give `scipy_stiff` the same `rtol` / `atol` as `scipy` (1e-4 / 1e-7) so switching presets tightens nothing but the method; implement `as_scipy_jac` on the NumPy dynamics evaluators as `lambda t, x: self.jacobian("f", "x")(x, u_nominal, t, frozen_params)` so `use_jac=True` means the same thing on both backends, or drop `use_jac` from the preset and say in the docstring that only the JAX evaluator supplies an analytic Jacobian.

**Why.** A student who picks `scipy_stiff` because the default stalls gets SciPy's defaults (rtol 1e-3, atol 1e-6), ten times looser than `scipy`, and on NumPy the `use_jac: True` in the preset is silently dropped. RULES 4.12: a default that changes the mathematics is announced.

**Evidence.** simulator.py:42-45 `"scipy_stiff": ("scipy", {"method": "Radau", "use_jac": True})` with no tolerances, versus 33-41 for `scipy`; scipy_ivp.py:60 `if use_jac and getattr(evaluator, "backend", None) == "jax"`; probe: `Simulator(Integrator(), solver="scipy_stiff").solve(); sim.last_debug["jac_applied"] is False`; evaluators.py:98-101 raises `NotImplementedError` while `compile(Integrator()).jacobian("f", "x")` already returns the finite-difference Jacobian.

### compile-simulation#11 — Gate the quoted batch-rollout claim and give diagrams a frozen-params reference for the family rule

*tooling · effort S · owner agent · rung v0.2 wave D · planned: S53*  
Files: `README.md:182-186`, `DESIGN.md:663-667`, `ROADMAP.md:75`, `benchmarks/suites/core_perf.py:65-140`, `minilink/core/compile/evaluators/jax_evaluators.py:429-451`, `minilink/core/compile/evaluators/jax_evaluators.py:844`

**What.** Add a `rollout_batch` row to `core_perf` (pendulum, 1000 × 1000 RK4 steps, plain and params-family) so the 27 ms sentence in README, DESIGN and ROADMAP has a gate and S53's 278 ms regression is measured by CI rather than by hand. Separately, set `_frozen_params = diagram.subsystem_params` snapshot on the diagram evaluators so `_params_batch_axes` uses the ndim rule for diagrams as it does for leaves. Optional, needs an ask: a NumPy `rollout_batch` twin (a loop over `rk4_integrate_zoh(_p)`) so the README snippet runs without JAX.

**Why.** RULES 6.3: a benchmark exists exactly when a performance claim needs a gate; this claim is quoted in three public documents and gated nowhere (`grep rollout_batch benchmarks/` is empty). On diagrams the family detector falls back to "leading axis equals the batch size", so a shared (B, k) parameter is silently swept.

**Evidence.** README.md:182-183 "1000 rollouts of 1000 RK4 steps take 27 ms as a compiled batch"; core_perf.py gates `loop_s`, `compile_s`, `speedup_vs_native` for `f` and the simulator, never a batch rollout; jax_evaluators.py:844 `self._frozen_params = None` on `JaxDiagramEvaluator`, so jax_evaluators.py:439-449 takes the `reference` empty branch and `axis()` returns `0 if (ndim >= 1 and np.shape(leaf)[0] == batch)`.

### compile-simulation#12 — Batch the static-leaf time grid instead of dispatching 10 001 times

*performance · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/simulation/static_simulator.py:78-94`, `minilink/simulation/compile_backend.py:12-28`, `minilink/simulation/time_grid.py:11`

**What.** Evaluate the grid in one call: on JAX `jax.vmap(evaluator.outputs_trace)` over `(t, u.T)`; on NumPy keep the loop or vectorise when the block's `compute` broadcasts. Failing that, make `resolve_auto_backend` prefer NumPy for static leaves (n = 0), where JIT buys nothing.

**Why.** `compute_trajectory(compile_backend="auto")` on a `Gain` or a `Step` source picks JAX and is 25× slower than NumPy because the default 10 001-point reporting grid is walked one jitted dispatch at a time. The auto backend should never make the student's plot slower.

**Evidence.** Probe: `Gain(2.0, dim=1).compute_trajectory(tf=10.0, compile_backend="jax")` 0.51 s versus 0.02 s on NumPy; static_simulator.py:85-87 `for i, ti in enumerate(self.t): out = self.evaluator.outputs(empty_x, u_i, float(ti))`; time_grid.py:11 `DEFAULT_N_STEPS = 10001`.

### compile-simulation#13 — Let `compile(verbose=True)` read in order on every path

*trap · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/core/compile/compiler.py:260-291`, `minilink/core/compile/evaluators/jax_evaluators.py:849-867`, `minilink/core/compile/compiler.py:129-151`, `minilink/core/compile/compiler.py:142`

**What.** Number the steps once in `compile_diagram` (loops → plan → JAX compatibility → JIT) and pass the step index to the evaluator, or drop the numbers and print the step names; print the closing `[compile] Done.` line on the NumPy paths too (today only the JAX branches print it, and the NumPy leaf prints nothing at all).

**Why.** A student who turns on `verbose` to learn what compile does reads "Step 1, Step 2, Step 0, Step 3"; on NumPy the same flag is silent. Small, but this is the one place the compiler explains itself.

**Evidence.** Probe output for a JAX diagram: `Step 1: Checking for algebraic loops`, `Step 2: Building execution plan`, `Step 0: Checking JAX compatibility`, `Step 3: JIT-compiling`; `compile(plant, backend="numpy", verbose=True)` prints `''`. compiler.py:263 / 280 versus jax_evaluators.py:853 / 864; `[compile] Done.` only at compiler.py:142, 158, 179 and 309, all after a JAX branch.

### compile-simulation#14 — Share one uniform-grid check between the fixed-step backends and the simulator

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: T4*  
Files: `minilink/simulation/simulator.py:100-105`, `minilink/simulation/solvers/euler_fixed.py:8-19`, `minilink/simulation/solvers/rk4_fixed.py:26-27`, `minilink/simulation/solvers/rk4_fixed.py:52-53`

**What.** Move `_require_uniform_times(times) -> dt` to `simulation/time_grid.py` as `uniform_dt(times)`; `RK4SolverBackend.integrate` / `integrate_forced` call it (today they take `times[1] - times[0]` and integrate a non-uniform grid silently), `EulerFixedStepSolverBackend` keeps calling it, and `Simulator.select_solver` uses it in place of `_time_grid_is_uniform`.

**Why.** Two predicates for one property (RULES 5.6), and the RK4 backend, the one a JAX user is auto-routed to, is the one without the guard. Any caller that hands `RK4SolverBackend` a non-uniform grid gets a wrong trajectory with no error, where the Euler sibling raises.

**Evidence.** rk4_fixed.py:27 `dt = times[1] - times[0]` with no check; euler_fixed.py:8-19 `_require_uniform_times` raises `ValueError("euler_fixedsteps requires a uniform time grid...")`; simulator.py:100-105 `_time_grid_is_uniform` is a third spelling of the same `np.allclose(np.diff(times), ...)`.

### Bugs reported by the compile-simulation finder

#### bug compile-simulation#0 — (medium) `dt`-based time grids overshoot `tf` by one sample (float `arange`)

Files: `minilink/simulation/time_grid.py:47`, `minilink/simulation/time_grid.py:57`, `minilink/simulation/simulator.py:243-262`

```text
from minilink.blocks.basic import Integrator; from minilink.simulation.simulator import Simulator
sim = Simulator(Integrator(), tf=1.1, dt=0.1)
print(sim.n_pts, sim.t[-1])   # 13 1.2000000000000002  (expected 12 samples ending at 1.1)
sim = Simulator(Integrator(), tf=0.2, dt=0.1)
print(sim.n_pts, sim.t[-1])   # 4 0.30000000000000004
Scanning tf in 0.1..20 step 0.1 and dt in {0.1, 0.05, 0.02, 0.01, 0.001}: 65 of 995 pairs end past tf. Cause: `np.arange(t0, tf + dt, dt)` at time_grid.py:47 and :57. Fix: `n = int(round((tf - t0) / dt)); t = t0 + dt * np.arange(n + 1)` (and warn or extend when (tf - t0)/dt is not an integer), plus a test asserting `t[-1] <= tf + 1e-12` over a sweep. The only existing endpoint assertion (test_simulation.py:902) uses a benign pair.
```

#### bug compile-simulation#1 — (medium) `JaxDiagramEvaluator` casts `dx` and the signal buffer to the dtype of `x`, truncating derivatives for integer states

Files: `minilink/core/compile/evaluators/jax_evaluators.py:921-926`, `minilink/core/compile/evaluators/jax_evaluators.py:939-950`, `minilink/core/compile/evaluators/jax_evaluators.py:1049-1057`

```text
import numpy as np, jax.numpy as jnp
from minilink.blocks.basic import Integrator; from minilink.core.diagram import DiagramSystem; from minilink.core.compile.compiler import compile
d = DiagramSystem(); d.add_subsystem(Integrator(), 'plant'); d.add_input_port('u'); d.connect('input','u','plant','u'); d.connect_new_output_port('plant','y','y')
ev = compile(d, backend='jax')
print(ev.f(jnp.array([0]), jnp.array([0.7]), 0.0))    # [0] int64  + FutureWarning 'scatter inputs have incompatible types'
print(ev.f(np.array([0]), np.array([0.7]), 0.0))       # [0]
print(compile(Integrator(), backend='jax').f(jnp.array([0]), jnp.array([0.7]), 0.0))  # [0.7]  (leaf evaluator is right)
`_infer_dtype` returns `x.dtype` (int64 here, float32 when neither x nor u carries a dtype), then `jnp.zeros(state_dim, dtype=dtype).at[...].set(float_piece)` truncates; JAX says the implicit cast will become an error. `Simulator` masks it by coercing `x0` to float (simulator.py:401), so only direct evaluator users hit it, but `compile()` is a public verb and `x0 = np.array([0, 0])` is a natural student line. Fix: `dtype = jnp.result_type(x, u, 0.0)` (float64 under the default x64) in `_infer_dtype`, and one test on the diagram evaluator with integer `x`.
```

#### bug compile-simulation#2 — (low) `compile_step_diagram` probes every subsystem's `step` and `h` twice

Files: `minilink/core/compile/step_compiler.py:56-57`

```text
Define a StepSystem whose `step` / `h` increment counters, put it in a StepDiagramSystem and `compile(diagram)`: both counters read 2 (one compile). Lines 56 and 57 are the identical call `validate_equation_shapes(subsystem, label=f"{subsystem.name} ({sys_id})")` (blame: same commit). Harmless for pure hooks; a step hook that is expensive (an MPC `StepSystem` solving an NLP on probe) or stateful runs one extra time per compile. Fix: delete line 57.
```


## dynamics

### dynamics#0 — Drop the invented default bounds from the mechanical bases; each plant states its own

*trap · effort M · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/abstraction/mechanical.py:44-61`, `minilink/dynamics/abstraction/generalized_mechanical.py:42-60`, `minilink/core/signals.py:176-177`, `minilink/planning/policy_synthesis/discretizer.py:84-85,496-514`, `minilink/simulation/realtime/pygame_input.py:253-256`, `minilink/graphical/port_map.py:412-415,601-611`, `minilink/dynamics/catalog/aerial/rocket.py:1-40`, `minilink/dynamics/catalog/aerial/drone.py:26-47`

**What.** `MechanicalSystem.__init__` and `GeneralizedMechanicalSystem.__init__` stop writing ±2π / ±10 / ±5 into every state and input signal (the `VectorSignal` default is ±inf); each catalog plant sets physically meaningful `lower_bound` / `upper_bound` on `u` (and on the state where a grid is expected): `Pendulum` and `CartPole` keep today's numbers explicitly, `Drone2D` gets `[0, 2 m g]` per thruster, `Rocket` `[0, k m g]` thrust, `Plane2D/3D` and `Boat2D` their own. `StateSpaceGrid` then fails with its existing "needs finite bounds" error instead of gridding nonsense.

**Why.** The bounds are read by the DP grid, the realtime key mapping, the port-map plot ranges and the torque-arrow sweep, so a student who runs value iteration or the pygame game on a `Rocket` gets thrust in [-5, 5] N for a 1000 kg body, and the drone's hover thrust (4.905 N) sits at 98 % of its action box. Three teaching notebooks already define a `NormalizedDrone2D(Drone2D)` whose main job is to override these bounds.

**Evidence.** mechanical.py:44-61 writes `lim = 2 * np.pi` on every state (positions in metres included) and `±5.0 [Nm]` on every input. Probe: `Rocket().inputs['u'].upper_bound == [5., 5.]` with `mass = 1000`; `Drone2D` `[5., 5.]` with hover 4.905 N each; `Plane3D` four inputs at ±5; `Rocket().state.upper_bound[:3] == [6.28, 6.28, 6.28]` for x, y in metres. Consumers: discretizer.py:84-85 builds the action box from these; pygame_input.py:253-256 maps keys to them; port_map.py:412-415. Notebooks work around it: drone_ppo.ipynb:75-87 and gymnasium_interface.ipynb:314-326 (`NormalizedDrone2D`). The GRO860 pendulum notebooks set their bounds explicitly (grid_world_dynamic_programming.ipynb:466-469, gymnasium_interface.ipynb:121-124), so the change is name- and number-preserving for the term.

### dynamics#1 — Constructor hygiene in the pendulum and mass-spring-damper families

*api · effort S · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/catalog/pendulum/pendulum.py:18,114,151,171`, `minilink/dynamics/catalog/mass_spring_damper/linear.py:36-41,47,118,207,310,318,327`

**What.** Subclasses forward the parent's keywords (`InvertedPendulum(length=1.0, mass=1.0)`, `PendulumWithNoisePort(length=..., mass=...)`, `TwoIndependentPendulums(length=..., mass=...)`); one keyword name for the mass across the chain (`SingleMass(mass=...)` and `FloatingSingleMass(mass=...)`, or `m` everywhere); `output_mass` outside `1..count` raises `ValueError` instead of being clamped.

**Why.** A student who learned `Pendulum(length=2.0)` writes `InvertedPendulum(length=2.0)` and gets a `TypeError`; `SingleMass(m=1.0)` fails while `FloatingSingleMass(m=1.0)` works; `TwoMass(output_mass=5)` silently measures `x2`. These are the first plants of GRO501 (the mass chains) and GRO860 (the pendulums).

**Evidence.** pendulum.py:18 `def __init__(self, length=1.0, mass=1.0)`; :114, :151, :171 `def __init__(self)` calling `super().__init__()`. Probe: `InvertedPendulum(length=2.0)` → `TypeError: unexpected keyword argument 'length'`. linear.py:47 `SingleMass(mass=1.0, k=2.0, b=0.0)`, :118 `TwoMass(m=1.0, ...)`, :310 `FloatingSingleMass(m=1.0, b=0.0)`. Probe: `SingleMass(m=1.0)` → `TypeError`. linear.py:36-41 `_mass_output_matrix`: `if output_mass < 1 or output_mass > count: output_mass = count`; probe `TwoMass(output_mass=5).C() == [[0, 1, 0, 0]]`, label `x2`.

### dynamics#2 — One port layout across the bicycle rungs: the `named_ports` default and the `x` port

*trap · effort S · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py:76-101,491-535`, `minilink/dynamics/catalog/vehicles/racecar.py:154-207`, `DESIGN.md:217-219`

**What.** One default for `named_ports` on `DynamicBicycle`, `BicycleDynRate` and `UdeSRacecarDyn` (True, the diagram convention), the same `expose_state` choice on all three so `outputs['x']` exists on every rung or on none, `UdeSRacecarDyn.__init__` no longer builds the parent's ports to discard them, and a one-line DESIGN note that `named_ports=False` exists only because planners read a single `u` port — the flag retires when `PlanningProblem` stacks input ports.

**Why.** Today the default flips between siblings (True, False, True), so `BicycleDynRate()` and `DynamicBicycle()` cannot be swapped in one diagram without re-wiring; a diagram that reads `car.outputs['x']` works on `BicycleDynRate` only. Every project file passes the flag explicitly because nobody can remember the default. The comment also documents a planner limitation as a plant constructor argument.

**Evidence.** dynamic_bicycle.py:76 `def __init__(self, named_ports=True)`; :491 `BicycleDynRate.__init__(self, named_ports=False)`; racecar.py:154 `UdeSRacecarDyn.__init__(self, named_ports=True)`; racecar.py:155 `super().__init__(named_ports=True)` then :177 `self.inputs = {}`. Probe of output keys: `DynamicBicycle ['y']`, `BicycleDynRate ['y', 'x']`, `UdeSRacecarDyn ['y', 'speed', 'slip', 'grip', 'imu', 'power']`. dynamic_bicycle.py:92 and racecar.py:195: "one stacked command port (planning / trajopt convention)". Explicit flags everywhere: car_trajopt_compare.py:196,497; mpc_racecar_circuit.py:61,135; test_mpc.py:1000.

### dynamics#3 — Derive `y` / `q` / `dq` port labels from the state instead of copying them

*consolidation · effort M · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/dynamics/abstraction/mechanical.py:63-71`, `minilink/core/system.py:466-475`, `minilink/dynamics/catalog/pendulum/pendulum.py:28-33`, `minilink/dynamics/catalog/pendulum/cartpole.py:197-207`, `minilink/dynamics/catalog/marine/boat.py:28-33`

**What.** When an output port is the identity (`y = x`, `x`) or a slice (`q`, `dq`) of the state, its `labels` / `units` resolve from `state` on read — an optional `source=(signal, slice)` on `OutputPort` with `labels` / `units` as properties that fall back to it. Plants that measure something else keep setting labels. The 56 copy lines in the catalog and the two in `MechanicalSystem` go.

**Why.** One owner per quantity (RULES 5.6): 28 plants repeat `self.outputs['y'].labels = list(self.state.labels)` / `.units = list(self.state.units)`, and the copy in `MechanicalSystem` is already stale for every plant — the `q` / `dq` ports that `closed_loop_qdq` wires and plots still read `Angle 0` / `Velocity 0` after the plant renamed its states. Fewer lines in every plant a student reads, and the labels cannot drift.

**Evidence.** grep: 28 hits each for `outputs["y"].labels = list(self.state.labels)` and `.units = list(self.state.units)` under dynamics/catalog; 0 hits for `outputs["q"]` or `outputs["dq"]` outside the base. mechanical.py:68-71 copies `self.state.labels[:dof]` into the ports at construction; pendulum.py:28 then rebinds `self.state.labels = ['theta', 'dtheta']`. Probe: `Pendulum().outputs['q'].labels == ['Angle 0']`, `outputs['dq'].labels == ['Velocity 0']`, `state.labels == ['theta', 'dtheta']`.

### dynamics#4 — One owner for the wheelbase: `a`, `b` in params; `length` and the `.a` / `.b` attributes go

*consolidation · effort M · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/catalog/vehicles/steering.py:27,44,83-87,98,101,109,122-124`, `minilink/dynamics/catalog/vehicles/racecar.py:95-99`, `minilink/graphical/catalog/skins.py:47-53`, `tests/unittest/test_planning.py:790,915,937`, `tests/unittest/test_racecar_plant.py:31`, `examples/projects/car_trajopt/vehicles/ladder.py:217,285`, `examples/projects/car_trajopt/vehicles/extras.py:46,53`

**What.** Remove `params['length']` and the `self.a` / `self.b` attributes from `KinematicBicycle`, `KinematicCar` and `UdeSRacecar`; add a read-only `wheelbase` property (`params['a'] + params['b']`) used by `f`, the skins and the camera; `_axle_offsets` in skins reads params only; update the six consumers (`DubinsSteering(wheelbase=sys.wheelbase)`, the ladder and extras, the racecar test).

**Why.** Three owners of one length: `f` uses `a + b`, the car geometry and camera use `params['length']`, and the 2-D skin prefers the `.a` / `.b` attributes. A student who edits `car.params['a']` (the natural GRO501 experiment: move the c.g.) sees the equations change while the drawn car and the Dubins wheelbase keep the old value.

**Evidence.** steering.py:27 `self.params = {"a": 1.0, "b": 1.0, "length": 2.0}`; :44 `length = params["a"] + params["b"]` (ignores `length`); :98,101,109,122-124 use `params["length"]`; :83-87 `self.a = 2.0; self.b = 3.0; self.params["a"] = self.a`. skins.py:48-49 `if hasattr(plant, "a") and hasattr(plant, "b"): return float(plant.a), float(plant.b)` — attributes win over params. Probe: after `kb.params['a'] = 3.0`, the wheelbase in `f` is 4.0 and `params['length']` is still 2.0. test_planning.py:790 `DubinsSteering(wheelbase=sys.params["length"], ...)`; test_racecar_plant.py:31 asserts `length == a + b` — a test guarding a copy.

### dynamics#5 — Put `DynamicBicycle` on `GeneralizedMechanicalSystem`

*consolidation · effort M · owner maintainer · rung v1.0 · planned: S32*  
Files: `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py:66-77,150-194,284-302`, `minilink/dynamics/abstraction/generalized_mechanical.py:65-165`, `minilink/dynamics/catalog/marine/boat.py:58-100`

**What.** `class DynamicBicycle(GeneralizedMechanicalSystem)` with `dof=3, pos=3`: today's `M`, `C`, `N` become the base's hooks unchanged, `d(q, v, u, t, params)` returns today's `generalized_d(q, v, u_in)` with `u_in` read from the port vector, `generalized_force` returns zeros, and `f` is inherited. `BicycleDynRate` and `UdeSRacecarDyn` keep their `f` overrides for the extra states.

**Why.** The plant hand-writes the base's equation (`M v̇ + C v + d = 0`, `q̇ = N v`) and its `M` / `C` / `N` are line-for-line the `Boat2D` ones, so a reader learns two spellings of one rigid body. On the base the bicycle gains `x2qv`, `kinetic_energy`, `inverse_dynamics` and, with S32, the `q` / `dq` ports — and the GMC714 vehicle lesson (C2) can present the ladder as one family.

**Evidence.** dynamic_bicycle.py:66 `class DynamicBicycle(DynamicSystem)`; :284-302 `dv = xp.linalg.solve(M, -C @ v - d); dq = N @ v; dx = xp.concatenate([dq, dv])` versus generalized_mechanical.py:140-165 (`vdot = xp.linalg.solve(M, rhs)`, `dx = self.qv2x(qdot, vdot)`). `M` at dynamic_bicycle.py:150-158 and boat.py:58-67, `C` at :160-176 and boat.py:69-84, `N` at :178-194 and boat.py:86-100 are identical bodies. pyro-port-remaining.md drops `RigidBody2D` with "Use GeneralizedMechanicalSystem or catalog plant" — the bicycle does neither.

### dynamics#6 — Name the hidden `0.01` linear damping in `Drone2D.d` and `Rocket.d`

*trap · effort S · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/catalog/aerial/drone.py:89-103`, `minilink/dynamics/catalog/aerial/rocket.py:66-80`, `minilink/dynamics/catalog/aerial/plane.py:125-147`

**What.** `"linear_damping": 0.01` joins both `params` dicts and each `d` hook reads it: `tau_d = xp.array([cda * vx * |vx| + c * vx, cda * vy * |vy| + c * vy, c * omega])` with the rotational row explicit. Same value, same behaviour, one more named knob.

**Why.** `cda` is a parameter but the `0.01` is not: a student who sets `params['cda'] = 0` expecting a drag-free drone still gets damping on all three DoF, the yaw axis is always damped by a number nobody can see from the notebook, and a params family or identification (C4) cannot reach it. The two `d` bodies are identical copies (as are the `H` / `C` / `g` triplets with `Plane2D`, the accepted cost of the RigidBody2D drop), so the constant is edited in two places.

**Evidence.** drone.py:97-99 `cda * dq[0] * xp.abs(dq[0]) + 0.01 * dq[0]`, `... + 0.01 * dq[1]`, `0.01 * dq[2]`; rocket.py:74-76 the same three lines. drone.py:26-35 and rocket.py:28-34 list `cda` in params and no linear coefficient. `Drone2D` is a GRO860 plant (drone_ppo.ipynb, drone_learn_to_fly_rl.py); adding a key is name-freeze safe.

### dynamics#7 — `Manipulator` kinematics defaults raise instead of returning zeros; `link_lengths` becomes the base contract

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: T3*  
Files: `minilink/dynamics/abstraction/manipulator.py:56-64`, `minilink/dynamics/abstraction/state_space.py:75-81`, `minilink/dynamics/catalog/manipulators/arms.py:184-193,429-430`, `tests/unittest/test_mechanical_robotics.py:423-430`, `tests/unittest/test_catalog_backends.py:21-23`

**What.** `Manipulator.forward_kinematics` and `J` raise `NotImplementedError` like `StateSpaceSystem.A` / `B` (the one test that pins the zeros is updated); `Manipulator.link_lengths(params=None)` is declared on the base and implemented by the four arms from `params`; `SpeedControlledManipulator.link_lengths` calls it, so the `hasattr` chain and the never-set `self.l` go, and `_planar_kinematic_geometry` / `tf` follow `params`.

**Why.** A silent zero is worse than an error on a teaching surface: a bare `Manipulator(dof=2, task_dim=2)` — and the both-backends factory `SpeedControlledManipulator(2, 2)` — publishes a `p` port that is identically 0 and a `pdot` that is identically 0, so a task-space controller wired to it converges to nonsense with no message. One `link_lengths` contract also removes the sibling inconsistency where only `TwoLinkManipulator` has `lengths()`.

**Evidence.** manipulator.py:58-59 `return xp.zeros(self.task_dim)`; :63-64 `return xp.zeros((self.task_dim, self.dof))`; state_space.py:75-81 raise `NotImplementedError`. Probe: `Manipulator(dof=2, actuators=2, task_dim=2).h_p(x, u) == [0., 0.]`. test_mechanical_robotics.py:423 `test_default_kinematics_ports_are_zero` pins it. Probe: `hasattr(arm, 'lengths')` is True only for `TwoLinkManipulator`; arms.py:184-193 `link_lengths` probes `hasattr(arm, "lengths")`, `"l" in arm.params`, `hasattr(self, "l")`.

### dynamics#8 — Give `VanderPol` a real input or none

*api · effort S · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/dynamics/catalog/equations/oscillators.py:12-34,43-44`

**What.** Either the forced Van der Pol, `ddy = -y + mu * dy * (1 - y**2) + u` (one line; the describing-function and entrainment classic), or `input_dim=None` as `Lorenz` does. Not a port labelled `unused`.

**Why.** A student who wires a controller or a `Sine` source into the Van der Pol sees a port that accepts a signal and ignores it; the sibling `Lorenz` has no input, so the two autonomous oscillators disagree on what an autonomous plant looks like. The forced form makes the plant useful in the frequency-domain lessons of wave B.

**Evidence.** oscillators.py:13 `super().__init__(n=2, input_dim=1, output_dim=2, ...)`; :20 `self.inputs["u"].labels = ["unused"]`; :24-34 `f` never reads `u`. Probe: `VanderPol().f(x, [0.0]) == VanderPol().f(x, [5.0])`; `Lorenz().m == 0`.

### dynamics#9 — Make the physics own the drawn lengths: `DoublePendulum.l2`, `CartPole.pole_length`, `Boat2D.body_width`

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: S43*  
Files: `minilink/dynamics/catalog/pendulum/double_pendulum.py:32-43,51,144-188,194`, `minilink/dynamics/catalog/pendulum/cartpole.py:179-195`, `minilink/dynamics/catalog/marine/boat.py:53`, `minilink/dynamics/catalog/manipulators/arms.py:305`

**What.** `l2` joins `DoublePendulum.params` (the dynamics read only `lc2`, unchanged; the animation reads `params['l2']`), matching `TwoLinkManipulator`; `CartPole.pole_length` derives from `lcg` (`2 * lcg` for a uniform pole, or a `pole_length` param) instead of a fixed 3.0; `Boat2D.body_shape` reads `params['Afc']` when drawn instead of copying it at construction.

**Why.** Animation is the student's first check of a model. Today `params['lc2'] = 0.5` on the double pendulum leaves a 1 m second link on screen, the cart-pole draws a 3 m pole whose c.g. the equations put at 0.5 m, and a boat whose `Afc` was changed keeps its old beam. Each is a second owner of a physical length (RULES 5.6).

**Evidence.** double_pendulum.py:32-43 params have `lc2` and no `l2`; :51 `self.l2 = 1.0` under `# Graphic parameters`; :144-188 `get_kinematic_geometry` / `tf` read `self.l2`. cartpole.py:180 `"lcg": 0.5` and :188 `sys.pole_length = 3.0` under `# Graphic parameters (not part of the EoM)`. boat.py:53 `self.body_width = self.params["Afc"]` in `__init__`. arms.py:305 `TwoLinkManipulator.params["l2"]` shows the convention the double pendulum should share.

### dynamics#10 — The racecar solver hint must not depend on the pre-read `refresh()` that S29 removes

*trap · effort S · owner agent · rung v1.0 · planned: S29*  
Files: `minilink/dynamics/catalog/vehicles/racecar.py:251-285`, `minilink/simulation/simulator.py:174,261`, `tests/unittest/test_racecar_plant.py:56`, `docs/plans/TODO.md`

**What.** Add the dependency to the S29 row now ("`UdeSRacecarDyn.refresh()` is the one catalog consumer of the pre-read hook"); when S29 lands, `smallest_time_constant` becomes derived on read — a property on the plant computed from `params`, or a `sys.fastest_time_constant(params)` hook the `Simulator` reads — with a test that the hint follows a `params` edit.

**Why.** Today the racecar recomputes its stiffest-mode estimate in `refresh()`, and it works only because `Simulator` calls `sys.refresh()` before every solve. S29 drops that call; after it, a student who edits `car.params['c_kappa']` simulates with a stale `dt` hint and nothing says so.

**Evidence.** racecar.py:251 `self.refresh()` in `__init__`; :253-285 `refresh` writes `self.solver_info["smallest_time_constant"] = 0.5 * min(...)` from `self.params`. simulator.py:174 `self.sys.refresh()`; :261 `default_dt = sys.solver_info["smallest_time_constant"] * scale`. TODO.md §6 S29: "`Simulator` drops its pre-read `refresh()`". test_racecar_plant.py:56 reads the hint but does not edit params first.

### dynamics#11 — Extend the both-backends catalog test with a perturbed-`params` case

*test · effort S · owner agent · rung v0.2 wave D · planned: T3*  
Files: `tests/unittest/test_catalog_backends.py:37-66`, `minilink/dynamics/catalog/manipulators/arms.py:265-270,408-414,603-611,708-716`, `minilink/dynamics/catalog/mass_spring_damper/linear.py:59-94`, `minilink/dynamics/catalog/aerial/plane.py:230-256`

**What.** A second parametrized case per catalog plant: scale every float / array entry of `sys.params` by 1.1, then assert (a) the NumPy and JAX evaluators agree on `f`, the outputs and the `tf` frames with the explicit `params`, and (b) the result differs from the default-params result whenever the plant has params. Plants that fail today are allowlisted in a shrinking list, the pattern `NUMPY_ONLY` already uses.

**Why.** RULES 6.3 asks every JAX plant for a nominal case and a nontrivial parameter case; the sweep runs only the nominal one, so a hook that reads `self.params` and drops its `params` argument passes on both backends. This is the ratchet that makes the T3 fixes (FK / J reading `params`, the MSD `A` / `B` on `xp`, `Plane2D.tf`) testable and keeps them fixed.

**Evidence.** test_catalog_backends.py:37-46 `_random_points` perturbs `x`, `u`, `t` only; :49-66 never passes `params`. Probe: `TwoLinkManipulator().forward_kinematics(q, params={**params, 'l1': 5.0})` equals the default answer (arms.py:409 `l1 = self.params["l1"]`). linear.py:66-71 builds `A` with `np.array` from `params` values. plane.py:232 `params = self.params` inside `tf`.

### dynamics#12 — Docs: the Sphinx dynamics page covers 7 of 23 catalog modules, and DESIGN points at a `car_profile` that is not in the catalog

*docs · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `docs/api/dynamics.rst:4-22`, `DESIGN.md:278-280`, `minilink/catalog/__init__.py`

**What.** One `automodule` per catalog module in `docs/api/dynamics.rst` (`abstraction.manipulator`, `manipulators.arms`, `manipulators.ur5`, `aerial.drone` / `plane` / `rocket`, `marine.boat`, `vehicles.steering` / `racecar` / `propulsion` / `suspension` / `mountain_car` / `tires`, `equations.integrators` / `oscillators`, `mass_spring_damper.linear`, `astro.three_body`), and DESIGN.md:280 corrected to the `examples/projects/car_trajopt/vehicles/car_profile.py` path that :278 already gives.

**Why.** Every one of these classes is on the teaching alias `minilink.catalog` and in the both-backends sweep, yet a student searching the site finds neither the UR5, the drone, the racecar nor the mass chains GRO501 starts with. The DESIGN cross-reference names a module that does not exist under `dynamics/`.

**Evidence.** dynamics.rst:4-22 lists exactly `state_space`, `mechanical`, `generalized_mechanical`, `pendulum.pendulum`, `pendulum.cartpole`, `pendulum.double_pendulum`, `vehicles.dynamic_bicycle`. `ls minilink/dynamics/catalog/vehicles/` has no `car_profile.py`; DESIGN.md:280 `:func:~minilink.dynamics.catalog.vehicles.car_profile.apply_car_profile` while :278 says the envelopes "live in ``examples/projects/car_trajopt/vehicles/car_profile.py``" and test_dynamics_catalog.py:684 imports from there.

### dynamics#13 — Three catalog rows for the D2 consolidation pick list

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: D2*  
Files: `minilink/dynamics/catalog/aerial/drone.py:180-221`, `minilink/dynamics/catalog/vehicles/steering.py:135-182`, `minilink/dynamics/catalog/manipulators/ur5.py:345-347,375-377`, `minilink/dynamics/catalog/vehicles/propulsion.py:22-32,129-163`, `tests/unittest/test_dynamics_catalog.py:451`, `examples/teaching/topics/robotics/manipulator_eom.ipynb:286-290`

**What.** Add to TODO §5 D2, for the maintainer's pick: (a) retire `SpeedControlledDrone2D` in favour of `HolonomicMobileRobot` (same `dx = u`, n = 2; keep the name as an alias for one release if a notebook needs it); (b) drop the UR5 one-line aliases `forward_dynamics_aba` and `inverse_dynamics_rnea` (the one test calls `inverse_dynamics`); (c) `LongitudinalFrontWheelDriveCarWithTorqueInput.__init__` calls `super().__init__()` and extends `n` / params instead of bypassing its parent and re-typing the nine-key params dict.

**Why.** Maintenance cost, not features: two importable plants with one equation are two places to fix a bug; two names for one UR5 method is API surface with no lesson behind it; a duplicated params dict drifts the first time one copy is tuned. Each is a one-line pick for the standing simplify-and-consolidate principle of ROADMAP §5.3.

**Evidence.** drone.py:180-221 `SpeedControlledDrone2D`: `dx = array_module(u).asarray(u)`, labels `x, y` / `vx, vy`, `camera_scale = 10.0`; steering.py:135-182 `HolonomicMobileRobot`: identical `f`, labels, camera; 0 examples use the drone variant (grep). ur5.py:345-347 `forward_dynamics_aba` → `return self.forward_dynamics(...)`; :375-377 `inverse_dynamics_rnea` → `return self.inverse_dynamics(...)`; manipulator_eom.ipynb:289-290 wraps `arm.forward_dynamics` itself; only test_dynamics_catalog.py:451 calls an alias. propulsion.py:129 `DynamicSystem.__init__(self, n=4, ...)` from a subclass of the slip-input car; :138-150 repeats :22-32 plus two keys; `camera_follow_frame = "body"` at :154 and :163.

### dynamics#14 — Retire the `JaxMechanicalSystem` twin: the `xp` base already traces

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/dynamics/abstraction/mechanical.py:179-254`, `minilink/experimental/symbolic/mechanics/export.py:81-95`, `tests/unittest/test_mechanical_robotics.py:118-155`, `tests/unittest/test_symbolic.py:119-132`

**What.** The symbolic exporter's `backend="jax"` builds on `MechanicalSystem` (it already traces); `JaxMechanicalSystem = MechanicalSystem` stays as an alias for one release with the two inheritance tests rewritten as identity checks, then the 76-line twin is deleted.

**Why.** The constitution allows a backend twin only when one class would sacrifice textbook readability; here the `xp` base is the readable one and its own test proves it is JAX-traceable, so the twin is 76 lines of `jnp` and `dtype` juggling with one consumer (the experimental symbolic exporter). It is exactly the "twins one `xp` body covers" item of the standing simplify-and-consolidate principle, and a student reading `mechanical.py` meets a second, uglier spelling of the same equations.

**Evidence.** mechanical.py:179-254 overrides `H`, `C`, `B`, `g`, `d`, `q2x`, `forward_dynamics`, `f` with `jnp` and `dt = getattr(q, "dtype", None) or jnp.float32`; :194-197 `jnp.diag(jnp.ones(self.dof, dtype=dt))` versus :83-84 `xp.eye(self.dof)`. test_mechanical_robotics.py:118 `test_default_base_is_jax_traceable_if_available` on the plain base. Consumers of the twin: export.py:95 (experimental symbolic), test_symbolic.py:132 and test_mechanical_robotics.py:130-155 (inheritance/layout checks only); 0 examples.

### Bugs reported by the dynamics finder

#### bug dynamics#0 — (medium) QuarterCarOnRoughTerrain damps against the road slope instead of the road's vertical velocity

Files: `minilink/dynamics/catalog/vehicles/suspension.py:82`, `minilink/dynamics/catalog/vehicles/suspension.py:57-68`

```text
import numpy as np
from minilink.catalog import QuarterCarOnRoughTerrain
qc = QuarterCarOnRoughTerrain()
x = np.array([0.0, 0.0, 2.0]); u = np.zeros(1)   # [dy, y, x_pos]
qc.params['vx'] = 1.0; a1 = qc.f(x, u)[0]
qc.params['vx'] = 3.0; a3 = qc.f(x, u)[0]
print(a1, a3)   # 0.8427 0.8427 — identical, although the road rises 3x faster under the wheel
# The damper force is b * (dy - dz/dt) with dz/dt = dz/dx * dx_pos/dt = dz(x) * vx.
# suspension.py:82 uses `b * (dy - ground_slope)`, i.e. dz(x) alone; with vx = 3 the expected
# acceleration is 1.5437. The default vx = 1.0 hides the missing factor; no test pins the term.
```

#### bug dynamics#1 — (low) Plane2D.d / Plane3D.d index `u`, so `inverse_dynamics` (and the model-based controllers) crash on the base's `u=None` contract

Files: `minilink/dynamics/catalog/aerial/plane.py:156`, `minilink/dynamics/catalog/aerial/plane.py:595-597`, `minilink/dynamics/abstraction/mechanical.py:101,130`, `minilink/control/modelbased.py:85,104,188`

```text
import numpy as np
from minilink.catalog import Plane2D
Plane2D().inverse_dynamics(np.zeros(3), np.array([10.0, 0.0, 0.0]), np.zeros(3))
# TypeError: 'NoneType' object is not subscriptable  (plane.py:156 `delta = u[1]`)
# MechanicalSystem.inverse_dynamics(q, v, acceleration, u=None, ...) and d(q, v, u=None, ...)
# declare u optional, and ComputedTorqueController / SlidingModeController call
# plant.inverse_dynamics(q, dq, qdd) without u (modelbased.py:85, 104, 188).
```

#### bug dynamics#2 — (low) `q` / `dq` output-port labels are stale on every MechanicalSystem plant

Files: `minilink/dynamics/abstraction/mechanical.py:66-71`, `minilink/dynamics/catalog/pendulum/pendulum.py:28`

```text
from minilink.catalog import Pendulum, TwoLinkManipulator
p = Pendulum()
print(p.state.labels, p.outputs['q'].labels, p.outputs['dq'].labels)
# ['theta', 'dtheta'] ['Angle 0'] ['Velocity 0']
# mechanical.py:68-71 copies state.labels[:dof] into the ports at construction; every plant then
# rebinds state.labels and no plant relabels q/dq (0 grep hits), so the ports closed_loop_qdq
# wires and plots carry the placeholder names. Same on TwoLinkManipulator (['Angle 0', 'Angle 1']).
```

#### bug dynamics#3 — (low) TwoMass / ThreeMass silently clamp an out-of-range `output_mass`

Files: `minilink/dynamics/catalog/mass_spring_damper/linear.py:36-41`

```text
from minilink.catalog import TwoMass
tm = TwoMass(output_mass=5)
print(tm.C(), tm.outputs['y'].labels)
# [[0. 1. 0. 0.]] ['x2']  — no error; _mass_output_matrix rewrites output_mass = count.
# A typo in the measured mass is accepted and the student's transfer function is of the wrong output.
```


## control

### control#0 — Put the LQR family on the control band facade

*api · effort M · owner maintainer · rung v0.2 wave B · planned: —*  
Files: `minilink/control/__init__.py:9-10,17-66`, `minilink/control/lqr.py:18,34,44,95,135,158`, `minilink/__init__.py:69-72`, `RULES.md:139`, `DESIGN.md:40`, `tests/unittest/test_teaching_surface.py:14,65-80`

**What.** Rename `control/lqr.py` (e.g. `control/optimal.py` or `control/riccati.py`) so the function `lqr` — with `lqr_at_operating_point`, `lqr_finite_horizon`, `trajectory_lqr`, `lqr_gain_schedule`, `lqr_gain` — can join `_EXPORTS` like every other law and be registered under `minilink.control` in `TEACHING_SURFACE`. Until the rename lands, correct RULES 4.1's example (it names an import that raises) to `from minilink.control.lqr import lqr`, which is what DESIGN.md:40 already teaches.

**Why.** `from minilink.control import lqr` is the band import RULES 4.1 gives as canonical and it raises `AttributeError` today; the band facade is the only import layer where the whole LQR family is missing. A GRO501 notebook (P11) written on the band layer would otherwise pin the `control.lqr` module path that a later rename breaks — do the rename before P11 and after the GRO860 term (gate 7).

**Evidence.** `control/__init__.py:9-10` documents the shadowing ("`lqr` lives in the `control.lqr` module — it is not re-exported on the package attribute `control.lqr`"); `lazy_facade(..., modules={"mpc": ...})` at :63-64 has no `lqr` key. Session check: `import minilink.control as C; C.lqr` → `AttributeError: module 'minilink.control' has no attribute 'lqr'`. The root prelude exports all four factories (`minilink/__init__.py:69-72`); `test_teaching_surface.py` registers `lqr` at the root only (:14), not in the `minilink.control` tuple (:65-80). 26 files import through `minilink.control.lqr` or the root.

### control#1 — Add `P` to the siso family and align `ProportionalController`'s default layout

*api · effort S · owner maintainer · rung v0.2 wave B · planned: —*  
Files: `minilink/control/siso.py:10-16,50-52,62,201-268`, `minilink/control/output.py:74-99`, `ROADMAP.md:78,159`, `docs/plans/gro501-classical-control.md (P1 'As built')`

**What.** `class P(PID): has_integrator = False; has_filter = False` (about eight lines: `Kp`, `dof`, `ports`, `u_min`, `u_max`) beside `PI` and `PD`, exported on the band and the root; then decide whether `ProportionalController` (the MIMO matrix gain) keeps `ports="reference"` or follows the family default `"error"`; update ROADMAP §3 / §4.2 to the names that exist.

**Why.** ROADMAP already promises `P / PI / PD / PID` — the guide's own vocabulary per the P1 ruling — but the stateless member is `ProportionalController` with the opposite default layout: `PID()`, `PI()`, `PD()` are compensators (one input `e`) while `ProportionalController()` is two-port (`r`, `y`), so the four forms do not close the same loop the same way. `PID(Kp=5) @ Pendulum()` builds `[ctl, sys, demux, error]`; `ProportionalController(5) @ Pendulum()` raises "declares measurement 'y' dim 1, but sys 'y' output has dim 2 … pass feedback='qdq'" (same on `CartPole`; both work on `SingleMass` / `DoubleIntegrator`). A GRO501 student going from P to PI should change one letter, not the topology.

**Evidence.** siso.py:62 `ports: str = "error"` vs output.py:91 `ports="reference"`. Session check: a `PID` subclass with both flags off has `n=0`, ports `['e']`, `ctl(e=0.5) = [2.5]`, closes on `Pendulum()` as `['ctl','sys','demux','error']` and simulates. ROADMAP.md:78 and :159 list `P` although `grep -rn "^class P\b" minilink` is empty.

### control#2 — One time-interpolation for trajectories and gain schedules

*consolidation · effort M · owner maintainer · rung v0.2 wave D · planned: T2*  
Files: `minilink/core/trajectory.py:165-179,241-244`, `minilink/control/state.py:192-207`, `minilink/control/mpc/utilities.py:138-151,154-168,171-204,207-219`, `minilink/control/mpc/controller.py:136-204`, `minilink/blocks/sources.py:4,204,274`

**What.** One JAX-traceable 'value of a sampled signal at `t`' (today `state.interpolate_schedule`) living beside `Trajectory`, exposed as `Trajectory.at(t)` (or `sample(name, t, method="linear")`). `TrajectoryFeedbackController`, `TimeVaryingStateFeedbackController` and the MPC nominal path read it, so `NominalCache`, `build_nominal_cache`, `_finite_diff_knots` (which is `np.gradient(y, tau, axis=1)`), `_clamp_tau`, `eval_signal` and the four `get_nominal_*` methods reduce to `plan.at(t)` on the latched plan shifted by `t_solve`.

**Why.** The same quantity — a trajectory between its knots — has three answers: `Trajectory.t2x / t2u / sample` are nearest-index, the two control blocks and `Trajectory.resample` are linear, `TrajectorySource` goes through `interp1d`. A student checking `ctl_traj.ctl(...)` against `reference.t2u(t)` gets different numbers from the same reference, and the MPC package keeps a third copy of the plan arrays only to interpolate them (`tau`, `x`, `u`, `x_dot`, `u_dot`).

**Evidence.** trajectory.py:165-172 `_nearest_index` / `sample` "by nearest time index"; state.py:203-205 linear on `searchsorted`; utilities.py:213-219 `np.interp` per row of the cached plan; utilities.py:154-168 hand-rolled central / one-sided differences that equal `np.gradient` on the uniform collocation grid; controller.py:178-204 four getters over the cache.

### control#3 — Decide the time argument on the sampled seam once: ticks or seconds

*api · effort L · owner maintainer · rung v1.0 · planned: S31*  
Files: `minilink/simulation/computer.py:202,216-218,224-226`, `minilink/control/mpc/controller.py:106-111,406-407,413-420,484-485,493-498,613-614,780-817,821-849,877-902`

**What.** A design conversation to fold into S31 / T6: `Computer.tick` hands port computes the integer tick `k_tick` as their `t`; the MPC block converts it back to seconds with private copies of `t0` / `dt_mpc` held on the block, on `MPCTickLatch` and on `MPCBroadcastController`, and needs `_replan_divisor` because it cannot see the schedule. If the seam passed `t = t0 + k·dt_base` in seconds (the continuous contract's own argument), the block would derive `k = round((t − t0) / dt_mpc)`, the three copies and the divisor would go, the broadcast leaf would be `plan.at(t)`, and `export_mpc_to_computer`'s `dt_base == dt_mpc` cross-check would be unnecessary.

**Why.** Two owners of one period and one epoch (RULES 5.6): `dt_mpc` lives on the block, on the latch and on the schedule; `t0` on the block, the latch and the broadcast leaf; `export_mpc_dual_rate_computer` writes `block._replan_divisor` from outside (T6 lists the write, not its cause). One convention removes machinery that T6 would otherwise only rename, and it is the same question S31 asks of the sampled loop.

**Evidence.** computer.py:216-218 `op.compute_func(local_x, local_u, k_tick, op.bound_params)`; controller.py:106-111 `_replan_k` divides the tick; :613-614 the latch's own `_dt_mpc` / `_t0`; :892-902 the broadcast leaf's `_t0`, `_dt_broadcast`, `_abs_t`; :810-817 the cross-check that exists only because the block cannot read the schedule; :846 `block._replan_divisor = d`.

### control#4 — One record per MPC tick: fold `Command` and `MPCTickSolve` into the `PlanningSolution`

*consolidation · effort M · owner maintainer · rung v0.2 wave D · planned: T6*  
Files: `minilink/control/mpc/controller.py:38-58,210-273,581-590,630-694`, `DESIGN.md:339-349`, `tests/unittest/test_mpc.py:86,109-116,127`

**What.** The latch stores the tick's `PlanningSolution` plus `k` and `t_solve`; `u_ff`, `x_ff`, `z`, `success` become properties (`solution.trajectory.u[:, 0]`, `.x[:, 1]`, the solver record's `z`, `solution.success`) on that one record — or `compute_command` returns the `PlanningSolution` itself with the tick metadata on the latch. `MPCTickSolve` goes.

**Why.** RULES 2.5: two dataclasses wrap the same solve — `Command(solution, k, t_solve, u_ff, x_ff, z, success)` and `MPCTickSolve(plan, z, u_ff, x_ff, t_solve, k)` — and every field but `solution` is copied out of the trajectory the solution already holds. One record is less to explain in DESIGN §5 and one fewer thing for T6 to reorder.

**Evidence.** controller.py:261-269 builds `Command` from `tick.u_ff`, `tick.x_ff`, `tick.z` that :681-688 sliced from `traj.u[:, 0]`, `traj.x[:, 1]`, `result.z`; :55-58 `Command.solver` re-exposes `solution.solver`; :268 `success=bool(solution.success)`.

### control#5 — One warm-start helper on the plan `Trajectory`

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: T2*  
Files: `minilink/control/mpc/utilities.py:16-44,47-91,94-122`, `minilink/control/mpc/controller.py:581-590,655-665`, `tests/unittest/test_mpc.py:364,407-420,1110,1200`

**What.** Keep one function — `warm_start_guess(prev_plan: Trajectory | None, x_meas, planner, *, dt_shift, t_anchor)` (today `warm_start_guess_from_prev_plan`) — and make `mpc_warm_start_guess` the three-line unpack of `z` at the boundary, or retire it once the latch warm-starts from the `plan` it already stores (`MPCTickSolve.plan`).

**Why.** Two public helpers shift the same plan; `warm_start_guess_from_prev_plan` has no library or demo consumer (only the hand-loop baseline in test_mpc.py:1200), and `_shift_plan_trajectory`, the shared core, is the underscore function the tests import. Lower late (CONSTITUTION §2): carry the `Trajectory`, unpack `z` once.

**Evidence.** utilities.py:47-91 unpacks `z_prev` into a `Trajectory` (:76-80) and calls `_shift_plan_trajectory`; :94-122 does the same from a `Trajectory`; grep: `warm_start_guess_from_prev_plan` appears only in `utilities.py` and `test_mpc.py`.

### control#6 — One constructor contract for the robotic laws

*api · effort M · owner maintainer · rung v0.2 wave C · planned: C2*  
Files: `minilink/control/robotic.py:36-39,50,83,160-181,189,246,304-314,365-375,423-442`, `minilink/dynamics/abstraction/manipulator.py:28-64`, `tests/unittest/test_mechanical_robotics.py:507-511`

**What.** Every robotic law takes a `Manipulator` (which owns `dof`, `task_dim`, `forward_kinematics`, `J`, `g`) and says so with one named error; `TaskKinematic` reads `plant.task_dim` (the `SpeedControlledManipulator.from_manipulator` twin can carry it) instead of `getattr(plant, "task_dim", plant.effector_dim)`; the gravity hook has one signature `gravity(q)` — the two-argument form is never exercised because `model_params` is always `None` — so the `try / except TypeError` goes; `gravity_comp` moves from `params` to a constructor attribute like `tracking_ref`.

**Why.** Siblings validate three different ways: `ModelJointImpedance` uses `isinstance` plus `hasattr(plant, "g")`, `TaskImpedance` nothing (`TaskImpedance(Pendulum())` dies with a bare `AttributeError: 'Pendulum' object has no attribute 'task_dim'`), `TaskKinematic` duck-types with `hasattr` (RULES 4.3). The `except TypeError` swallows a hook's own bug and re-raises "missing 1 required positional argument: 'params'" in its place. A `bool` inside `params` breaks the one-signature promise: `jax.grad` over `JointImpedance(arm).params` or `TaskImpedance(arm).params` fails on the `gravity_comp` leaf while the other twelve laws' params differentiate.

**Evidence.** robotic.py:38 `hasattr(plant, "g")`; :305 and :366 `hasattr(plant, "forward_kinematics")`; :310 and :371 `getattr(plant, "task_dim", plant.effector_dim)`; :434-437 `try: g = gravity(q, model_params) except TypeError: g = gravity(q)`; :83 and :246 call `gravity_feedforward(plant, gravity, q)` with `model_params` omitted; :50 and :189 `"gravity_comp": bool(gravity_comp)` inside `params`. Session sweep: all fourteen `ctl`s trace under `jit`; `grad` w.r.t. `params` passes for twelve, fails for `JointImpedance(arm)` and `TaskImpedance(arm)` with "grad requires real- or complex-valued inputs".

### control#7 — Consolidate the impedance and robotic law bodies

*consolidation · effort M · owner agent · rung v0.2 wave D · planned: T2*  
Files: `minilink/control/impedance.py:47-81,95-104,157-181,183-206,228-233`, `minilink/control/robotic.py:18-86,88-132,290-331,356-394,445-458`

**What.** `ImpedanceIntegralController(ImpedanceController)` inherits the port and label setup and `split_measurement`; `ModelJointImpedance(ImpedanceController)` computes `tau = super().ctl(...) + g(q)`, so `impedance_joint_torque` goes and the `JointImpedance` factory shrinks to the plant-or-dof switch; `TaskKinematicNullspace.__init__` calls `super().__init__(plant, Kp=Kp)` and adds `r_null` (keeping the `(r, r_null, y)` slice order, or reslicing); the unreachable `else: raise` on `ref_dim` (fixed by the constructor) goes. The `y_labels / y_units / u_labels / u_units` kwargs, which no caller passes, are a maintainer pick to keep or drop.

**Why.** The same 35 lines of label defaults and three `add_*_port` calls appear twice in impedance.py; the same two-branch spring-damper law appears in `ImpedanceController.ctl` and in `impedance_joint_torque`; the same 25-line plant validation and port setup appears twice in robotic.py with a `super(TaskKinematic, self).__init__()` skip. One body per law is what the textbook pass is for; today a change to the impedance law needs two edits.

**Evidence.** impedance.py:50-57 vs :157-164 identical label defaults; :66-81 vs :166-181 identical ports; :95-100 vs robotic.py:447-456 identical law; robotic.py:305-331 vs :366-394 duplicated validation and ports; grep: `y_labels=` / `u_labels=` never passed outside `control/`.

### control#8 — One `ctl` for the model-based laws

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/control/modelbased.py:60-70,72-106,140-141,144-168`

**What.** `ComputedTorqueController.ctl(x, u, t, params)` unpacks `r` by `ref_dim` (as `ImpedanceController.ctl` does) and computes `qdd_des` and `tau` once; `ctl_regulation`, `ctl_tracking` and the `ctl` shim (`self.outputs["u"].compute(...)`) go; `SlidingModeController` overrides `ctl` only and drops the post-construction `self.outputs["u"].compute = ctl_fn` rebind.

**Why.** Four near-identical methods plus a shim per class; the shim exists only so `plot_control_law` finds a `ctl`. The rebind at modelbased.py:140-141 is a no-op — the bound method the parent stored already resolves to the subclass — and mutating a port after construction is against the build-vs-run rule (RULES 4.11). No example or test calls `ctl_regulation` / `ctl_tracking`.

**Evidence.** modelbased.py:68-70 shim; :72-87 and :89-106 differ only in the `r` slice; :140-141 rebind; session check `smc.outputs["u"].compute.__func__ is SlidingModeController.ctl_tracking` is already `True` before that line runs; grep of `ctl_regulation|ctl_tracking` outside `control/`: none.

### control#9 — Size PurePursuit's measurement from the vehicle, not a `state_dim=9` default

*trap · effort S · owner maintainer · rung v0.2 wave A · planned: A3*  
Files: `minilink/control/geometric.py:57-59,67-77,98,117`, `examples/projects/racecar/racecar_lap_3d.py:52-61`, `docs/plans/geometry-module.md`

**What.** When A3 gives `PurePursuit` a `Path`, also fix its port: read `[x, y, theta, vx]` as a dim-4 `y` (the layout the docstring states), or a `pose` / `speed` pair, or take `plant=` and size `y` from `plant.n`; drop the `9`.

**Why.** No catalog plant has `n = 9`, so the default fits nothing: `PurePursuit(wp) @ KinematicBicycle()` (n=3) raises "declares measurement 'y' dim 9, but sys 'y' output has dim 3"; the only library caller passes `state_dim=car.n` by hand; and a `KinematicBicycle` — the GRO501 APP4 plant — cannot drive the law at all because it reads `vx = u[3]`.

**Evidence.** geometric.py:76 `state_dim: int = 9`; :117 `X, Y, theta, vx = u[0], u[1], u[2], u[3]`; racecar_lap_3d.py:60 `state_dim=car.n`; `grep -rn "n=9" minilink/dynamics/catalog` empty; wiring failure verified in a session.

### control#10 — `print(controller)` shows the gains

*feature · effort S · owner agent · rung v0.2 wave D · planned: D1.1*  
Files: `minilink/core/system.py:120-121`, `minilink/control/state.py:37,93,159`, `minilink/control/siso.py:81-95`, `minilink/control/lqr.py:34-41,290-291`

**What.** One `params:` line in `inspect_text` (values for scalars and small matrices, shapes otherwise) — or a `__str__` on `Controller` only — so `print(lqr(A, B, Q, R))` shows `K` and `print(PID(2, 1))` shows `Kp, Ki, Kd, tau`.

**Why.** RULES 6.1 routes reporting through `print(obj)`; today `print(lqr(...))` gives "State Feedback Controller (StateFeedbackController), n=0 / inputs: x (2), r (2) / outputs: u (1)" and the gain a GRO860 student came for is reachable only as `controller.params["K"]` — which is exactly what `lqr.py`'s own `__main__` prints by hand.

**Evidence.** Session output as quoted; `print(Pendulum())` prints no params either; lqr.py:290-291 `K = controller.params["K"]; print("LQR gain K =\n", ...)`.

### control#11 — One parameterized both-backends test over every control law

*test · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/unittest/test_control_analysis.py:485-502`, `tests/unittest/test_geometric_control.py:304-320`, `tests/unittest/test_mechanical_robotics.py:486-560`

**What.** A table of `(block, x, u)` covering `ProportionalController`, `PID / PI / PD`, the three state-feedback blocks, the two impedance blocks, `JointImpedance(arm)`, `TaskImpedance`, `TaskKinematic(Nullspace)`, `ComputedTorqueController`, `SlidingModeController`, `NeuralPolicyController`, `PurePursuit`; one `pytest.mark.parametrize` asserting `jax.jit(ctl)` matches NumPy and that `jax.grad` of `ctl` with respect to `params` runs (the one-signature promise of CONSTITUTION §2).

**Why.** JAX tracing is pinned for `PID.f` and `PurePursuit.ctl` only. A session sweep traced all fourteen `ctl`s under `jit` and differentiated twelve through `params` (the two failures are the `gravity_comp` bool leaf). The test turns that into a gate before T2 restyles these bodies, catches the next `np.` slip (T3 found several in the catalog), and is one test rather than fourteen (RULES 6.3, 6.4).

**Evidence.** test_control_analysis.py:485-502 `test_f_is_jax_traceable` (PID only); test_geometric_control.py:304-320 (PurePursuit only); sweep results as above.

### control#12 — Control band docs housekeeping

*docs · effort S · owner agent · rung v0.1 close-out · planned: T2*  
Files: `docs/api/control.rst:4-22`, `DESIGN.md:94`, `minilink/control/__init__.py:1-11`, `minilink/control/mpc/__init__.py:1-12`, `minilink/control/mpc/utilities.py:104-108`, `minilink/control/mpc/viz.py:15-19,92,190-193`

**What.** Add `minilink.control.geometric`, `minilink.control.neural` and `minilink.control.mpc` to the Sphinx API page; list `geometric.py` and `neural.py` in DESIGN's `control/` row; when the LQR facade lands, delete the `control/__init__.py` note about the `lqr` shadowing; move `viz.py`'s `HybridSimResult` import under `TYPE_CHECKING` (it is an annotation only, RULES 3.2); flag that `viz.py:191` imports `planning.spatial.overlays`, which D2 / A3 retire, so the A3 landing must retarget it.

**Why.** `PurePursuit` and `NeuralPolicyController` are root-prelude names (`NeuralPolicyController` is on the GRO860 checklist, ROADMAP §4.1) with no autodoc page; DESIGN's module list drifted when the two modules landed; `viz.py` would break on the A3 delete.

**Evidence.** `docs/api/control.rst` automodules output, impedance, state, siso, robotic, modelbased, lqr only; DESIGN.md:94 names seven modules plus `mpc/`; viz.py:19 `from minilink.simulation.hybrid_simulator import HybridSimResult` used only in the signature at :92; viz.py:191 `from minilink.planning.spatial.overlays import TrackCorridorOverlay`.

### control#13 — Move `LookupTableController` beside the other laws

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/policy_synthesis/lookup_policy.py:13-16,21-40`, `minilink/control/neural.py:24`, `minilink/planning/__init__.py`, `minilink/__init__.py`

**What.** `control/lookup.py` (or `control/tabular.py`) holding `LookupTableController`; the root prelude and the `planning` facade keep exporting the name through `lazy_facade`, so no GRO860 notebook import changes (§4.1 gate 7).

**Why.** The band docstring says `control` is "Controller and static law blocks"; the DP / RL policy block is the one `Controller` subclass living in a planner package, while its neural sibling — also trained by `planning` — lives in `control/neural.py`. RULES 3.1: a law is control. The module imports only NumPy, SciPy and `core.feedback`, so nothing pins it to `planning`.

**Evidence.** lookup_policy.py:13-16 imports (`numpy`, `RegularGridInterpolator`, `minilink.core.feedback.Controller`); :21 `class LookupTableController(Controller)`; neural.py:24 the precedent.

### Bugs reported by the control finder

#### bug control#0 — (medium) `lqr_gain_schedule` returns a wrong or NaN schedule on stiff pairs with a coarse grid (no substep guard, unlike `riccati_step`)

Files: `minilink/control/lqr.py:120-128`, `minilink/control/lqr.py:235-242`, `minilink/control/lqr.py:245-257`

```text
import numpy as np
from minilink.control.lqr import lqr_gain_schedule, riccati_step
A = np.array([[0., 1.], [-1., -1000.]]); B = np.array([[0.], [1.]]); Q = np.eye(2); R = np.eye(1); S_f = np.zeros((2, 2))
t, K, S = lqr_gain_schedule(A, B, Q, R, S_f, 5.0, n_steps=101)   # K[0] = [0.0374, 0.0006], silently wrong
S_ref = riccati_step(A, B, Q, np.linalg.inv(R), S_f, 5.0)          # gives K = [0.005, 0.0005], same as n_steps=10001
lqr_gain_schedule(A, B, Q, R, S_f, 5.0, n_steps=11)                # LinAlgError: Singular matrix
lqr_gain_schedule(A, B, Q, R, S_f, 5.0, n_steps=2)                 # K = [nan, nan], no error

Cause: `riccati_transition` takes one `expm(-H dt)` for the whole interval; `riccati_step` (used by `trajectory_lqr`) cuts the interval so that ||H|| dt <= 1. Fix: compute `n_sub` once in `lqr_gain_schedule` and apply `riccati_map` `n_sub` times per interval (or call `riccati_step`); `n_steps` is a public knob, so the failure is reachable from the teaching surface.
```

#### bug control#1 — (medium) MPC tick latch memoizes by `k` only and returns a plan solved for a different measurement

Files: `minilink/control/mpc/controller.py:630-642`, `minilink/control/mpc/controller.py:210-273`, `minilink/simulation/hybrid_simulator.py:133`, `minilink/simulation/computer.py:157-175`

```text
# NumPy backend, no JAX needed (fixture as in tests/unittest/test_mpc.py:730-757)
sys = SingleIntegrator(); cost = QuadraticCost.from_system(sys, Q=np.eye(1), R=np.eye(1), S=np.zeros((1, 1)))
planner = TrajectoryOptimizationPlanner(PlanningProblem(sys=sys, x_start=np.array([0.]), cost=cost, tf=1.0), n_steps=5, transcription="direct_collocation", compile_backend="numpy")
mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=False)
c1 = mpc.compute_command(np.array([+2.0]), k=0)   # u_ff = [-1.296]
c2 = mpc.compute_command(np.array([-2.0]), k=0)   # u_ff = [-1.296]; c2.solution is c1.solution
mpc.outputs["u_ff"].compute(np.zeros(0), np.array([-2.0]), 0)   # also [-1.296]

`MPCTickLatch.solve_for_tick` returns the memo when `self._latch_k == k_int` without comparing `y` (controller.py:641-642). Simulation is safe because `HybridSimulator` calls `computer.reset()` per run, which resets the latch; any two deploy or port calls at the same `k` with a different measurement (an explicit `k`, a restarted clock, direct port evaluation) silently reuse the stale plan. Fix: memo on `(k, y)` — keep `y_arr` on the latch and re-solve unless `np.array_equal`.
```

#### bug control#2 — (low) `mpc % dt` bypasses the `dt_mpc` cross-check that `export_to_computer(dt)` enforces

Files: `minilink/core/system.py:412-416`, `minilink/control/mpc/controller.py:780-817`, `minilink/control/mpc/controller.py:805-808`

```text
mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True)
mpc.export_to_computer(0.5)   # ValueError: schedule dt_base=0.5 does not match block dt_mpc=0.2
computer = mpc % 0.5          # accepted: computer.schedule.dt_base == 0.5 while mpc.dt_mpc == 0.2

`System.__mod__` calls `as_computer` directly, so the Computer ticks every 0.5 s while `t_solve = t0 + k*0.2` and the warm-start shift assume 0.2 s — and the checked path's own error text recommends "or use mpc % dt" (controller.py:808). Fix: `ModelPredictiveControllerMixin.__mod__` delegating to `export_to_computer(schedule)`.
```

#### bug control#3 — (low) RULES 4.1's canonical band import `from minilink.control import lqr` raises

Files: `RULES.md:139`, `minilink/control/__init__.py:9-10,63-66`

```text
from minilink.control import lqr
# AttributeError: module 'minilink.control' has no attribute 'lqr'

The function is shadowed by the module of the same name and the lazy facade lists neither; the root prelude (`from minilink import lqr`) and `from minilink.control.lqr import lqr` work. Short fix: correct the RULES example; real fix: suggestion 1 (rename the module, export the family on the band).
```

#### bug control#4 — (low) `gravity_feedforward` masks a gravity hook's own `TypeError`

Files: `minilink/control/robotic.py:432-437`

```text
import numpy as np
from minilink.control.robotic import gravity_feedforward
def hook(q, params):
    raise TypeError("bug inside the hook")
gravity_feedforward(None, hook, np.zeros(2))
# TypeError: hook() missing 1 required positional argument: 'params'   (the hook's own message is lost)

The `try: gravity(q, model_params) except TypeError: gravity(q)` retry treats any TypeError as a signature mismatch. `model_params` is always None at both call sites (robotic.py:83, :246), so one documented signature `gravity(q)` removes the retry.
```


## analysis

### analysis#0 — Move dt off the params dict in discretize and copy the source's x0, labels and bounds

*api · effort M · owner maintainer · rung v0.2 wave B · planned: P6*  
Files: `minilink/analysis/discretize.py:13-29`, `minilink/analysis/discretize.py:66-79`, `minilink/analysis/discretize.py:119-135`, `minilink/core/wiring.py:24-56`, `tests/unittest/test_control_analysis.py:1055-1074`, `examples/demos/analysis/analysis_discretize.py:8-18`

**What.** Store the sample time on the wrapper (`disc.dt`, read as `dt = self.dt` in `step`/`h`) and pass `params` through to the source untouched (`f(x, u, k * dt, params)`), so `params=None` means the source's live params and any dict replaces them (RULES 4.4). In `__init__`, copy `source.x0`, `source.state` labels and bounds, and each input port's bounds. Keep `dt=` positional; drop the `params=` merge or make it the source's params only.

**Why.** One design flaw produces three failures: `discretize(ctl @ plant, dt)` cannot step (the flat `dt` key is an unknown subsystem id), `disc.step(x, u, 0, plant.params)` raises `KeyError: 'dt'`, and `jacobian(disc, 'step', 'params')` reports a spurious `dt` sensitivity an identification fit would try to tune. Dropping `x0` makes the shipped demo roll out from rest. P6's default ('teach with discretize + simulation') needs this tool to work on closed loops.

**Evidence.** `_merge_discretize_params` at discretize.py:124-128 does `merged = dict(getattr(system, 'params', {}))`, `merged.update(params)`, `merged['dt'] = float(dt)`. Probe: `discretize(PID(1,0,0) @ Pendulum(), dt=0.01).step(x, u)` -> `ValueError: Unknown subsystem ids in diagram params: 'dt'; available: 'ctl', 'sys', 'demux', 'error'`; `disc.step(x, u, 0, plant.params)` -> `KeyError: 'dt'`; `disc.jacobian('step','params')` keys `['I','d','dt','gravity','l','m']`. `plant.x0=[0.3,0]` -> `disc.x0=[0,0]`, labels `['x[0]','x[1]']`, `u` bounds `[-inf],[inf]` although the plant had `[-2],[2]`. analysis_discretize.py:9 sets `plant.x0=[0.5,0]` and line 17 `disc.compute_rollout(n_steps=40, u=zeros)` gives `max|x| = 0.0`. `test_step_params_override_dt` (test_control_analysis.py:1065) pins today's `params['dt']` contract, so this is a decision.

### analysis#1 — Add an exact zero-order-hold option to discretize and share it with step_response

*feature · effort S · owner maintainer · rung v0.2 wave B · planned: P6*  
Files: `minilink/analysis/linear.py:184-209`, `minilink/analysis/discretize.py:85-113`, `minilink/analysis/discretize.py:7`

**What.** One matrices-tier function `linear.zoh(A, B, dt) -> (A_d, B_d)` (the `expm([[A, B],[0, 0]] dt)` block already written inline in `step_response`), used by `step_response` and by `discretize(lti, dt, integrator='zoh')` when the source is an `LTISystem` (or any `StateSpaceSystem` at `t=0`), returning a step model with constant `(A_d, B_d, C, D)`.

**Why.** The textbook discrete model `x_{k+1} = e^{A dt} x_k + ...` is what a digital-implementation lesson writes on the board; today the library computes it (linear.py:201-202) but does not expose it, and `discretize` offers only Euler and RK4 approximations of a linear plant. It also removes a second owner of the ZOH pair once a discrete Kalman filter (the estimation follow-up in wave C) needs it.

**Evidence.** linear.py:199-202: `# expm([[A, B], [0, 0]] dt) = [[A_d, B_d], [0, I]]` ... `hold = linalg.expm(np.block([[A, B], [np.zeros((m, n + m))]]) * dt)`. discretize.py:7: `_INTEGRATORS = frozenset({'rk4', 'euler'})`. ROADMAP §4 row 162: 'Digital implementation — difference equations on the Arduino | `discretize` (Euler / RK4 step models)'.

### analysis#2 — Let controllability and observability take any System, with facades and a __str__

*api · effort M · owner maintainer · rung v0.2 wave B · planned: —*  
Files: `minilink/analysis/structural.py:28-47`, `minilink/analysis/structural.py:72-78`, `minilink/analysis/structural.py:15-24`, `minilink/core/facades.py:622-643`, `examples/demos/analysis/analysis_structural.py:10-15`, `examples/tutorial/04_analysis.ipynb:90-91`

**What.** `controllability(sys, x_bar=None, u_bar=None, t=0.0, params=None, *, method='auto', eps=1e-6)` linearizes like every sibling (arrays still accepted as `controllability(A, B)`), plus `sys.controllability()` / `sys.observability()` facades and `StructuralResult.__str__` (`'controllable: rank 2 / 2'`). Keep the current `(A, B)` and `LTISystem` paths.

**Why.** These are the only two analysis verbs that reject a nonlinear plant (`controllability(Pendulum())` -> TypeError) and the only two with no facade, against Constitution §4.1 and RULES 4.3 ('a tool requiring a linear model accepts any System via linearize()'). The `__str__` turns the demo's two f-string prints into `print(ctrl)` (RULES 6.13). P4's `luenberger()` and P3's `place()` need the same 'pair is not observable / controllable' precondition, so one verb serves both.

**Evidence.** structural.py:72-78 `_lti_matrices` requires `callable(getattr(lti, 'A'))`; probe: `controllability(Pendulum())` -> 'pass the two matrices (A, B) or one LTISystem; got Pendulum'. facades.py has no `controllability`. analysis_structural.py:14-15: `print('controllable:', ctrl.is_full_rank, f'(rank {ctrl.rank}/{ctrl.n})')`. TODO P3: 'an honest error when the pole set is not reachable'.

### analysis#3 — Add a poles verb on a System so notebooks stop writing np.linalg.eigvals(lin.A())

*api · effort S · owner maintainer · rung v0.2 wave B · planned: P8*  
Files: `minilink/analysis/linear.py:19-27`, `minilink/analysis/frequency.py:164-187`, `minilink/analysis/modal.py:9-50`, `examples/tutorial/04_analysis.ipynb:63`, `examples/tutorial/11_reinforcement_learning.ipynb:363`, `examples/tutorial/showcase_jax.ipynb:162`, `examples/tutorial/showcase_jax.ipynb:485`, `examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb:118`, `examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb:148`, `examples/demos/analysis/analysis_linearize.py:24`, `examples/teaching/topics/classical_control/frequency_response.ipynb:362`

**What.** `poles(sys, x_bar=None, u_bar=None, t=0.0, params=None, *, method, eps)` returning `eig(A)` of the linearization (the system-tier twin of `linear.poles(A)`), exported in the band and root prelude with a `sys.poles()` facade; P8's ζ / ω_n per complex pair hang off the same result.

**Why.** Nine teaching cells and demos compute the poles by hand through `lin.A()`; RULES 6.13 even blesses that line as the good form because no verb exists. Today the poles come back three ways (`modal_analysis()[0]`, `pzmap()[1]`, `eigvals(lin.A())`), none named `poles`. One noun makes the D1 sweep a one-liner and gives P8 its home.

**Evidence.** `grep -rn 'np.linalg.eigvals(' examples/` returns 9 sites (files listed). linear.py:19 `def poles(A)` exists on the matrices tier only; analysis/__init__.py `_EXPORTS` has no `poles`. ROADMAP §6 lists P8 as 'ζ and ω_n from a complex pole pair (fields on the pzmap result or a damping(sys) verb)'.

### analysis#4 — Let step_info take a System like every other verb in time_response

*api · effort S · owner maintainer · rung v0.2 wave B · planned: —*  
Files: `minilink/analysis/time_response.py:72-91`, `minilink/analysis/time_response.py:43-69`, `examples/teaching/topics/classical_control/frequency_response.ipynb:406-412`

**What.** `step_info(sys, x_bar=None, u_bar=None, t=0.0, params=None, *, of, wrt, tf, n, method, eps)` computing the response itself (the `(time, y)` array form stays, dispatched on the first argument), plus a `sys.step_info()` facade. `plot_step_response` already does exactly this internally.

**Why.** `step_info` is the one system-tier verb that takes arrays, so the notebook writes `step_info(*T_pd.step_response())`; the tier rule the band states (arrays in `linear.py`, systems in `time_response.py`) puts it on the wrong side, and a student comparing two loops wants `T_pd.step_info()`.

**Evidence.** time_response.py:72 `def step_info(time, y) -> StepInfo`; time_response.py:111-114 `plot_step_response` calls `step_response(...)` then `step_info(time, y)`. frequency_response.ipynb:412: `info_pd = step_info(*T_pd.step_response())`. Module docstring time_response.py:1-9: 'This module picks the channel and the horizon'.

### analysis#5 — Refine the margin crossings by a root solve so plot_bode and margins() report one number

*consolidation · effort S · owner agent · rung v0.2 wave B · planned: —*  
Files: `minilink/analysis/linear.py:124-135`, `minilink/analysis/linear.py:271-308`, `minilink/analysis/frequency.py:45`, `minilink/analysis/frequency.py:122`, `minilink/analysis/frequency.py:149`, `minilink/analysis/frequency.py:504-519`, `minilink/analysis/frequency.py:68-70`

**What.** In `_phase_margin_from_samples` / `_gain_margin_from_samples`, once a bracket `[w_k, w_{k+1}]` is found, solve `|G(jw)| = 1` (resp. `arg G = -180°`) exactly with `brentq` on `linear.frequency_response`, then evaluate the margin at that root; `linear.margins(A, B, C, D, w)` gets the matrices to do so. Guard `log10(|G|)` with `errstate` as `bode` does, and treat a sample that sits exactly on the crossing (`_sign_changes` ignores sign 0). Then refresh the `frequency_response` docstring, which still describes the pre-F1 band.

**Why.** The Bode note is computed on the 200-point plot grid while `margins()` uses 2000 points, so the figure and the number disagree in the second decimal and both depend on `n`; a root solve makes the margin grid-free and lets every default `n` (200 / 500 / 2000 across siblings) be a plotting choice only. Two owners of one quantity (RULES 5.6).

**Evidence.** frequency.py:45 `n: int = 200` (bode), :122 `n: int = 2000` (margins), :149 `n: int = 500` (nyquist); `_bode_figure` at :511-512 calls `linear.margins(w, G)` on the plot grid. Probe on `PID(K,1,0.5) >> Pendulum()`: K=2 -> plot note PM 18.392° vs `margins()` 18.381°; K=5 -> 9.766° vs 9.754°. linear.py:273-274 `_sign_changes`: `signs[:-1] * signs[1:] < 0` -> `[1, 0, -1]` gives no crossing. frequency.py:68-70 still says the grid 'runs one decade below the slowest pole or zero to one decade above the fastest', which F1's `_bracket_unit_gain` (linear.py:230-256) no longer guarantees.

### analysis#6 — Check the operating point's size and warn on an unstable step channel at the boundary

*trap · effort S · owner agent · rung v0.2 wave B · planned: —*  
Files: `minilink/analysis/derivatives.py:105-114`, `minilink/analysis/time_response.py:66-69`, `minilink/analysis/linear.py:212-222`

**What.** In `operating_point`, raise `ValueError(f'{sys.name}: x_bar has {k} entries, the system has {sys.n} states')` (same for `u_bar` against `sys.m`) before compiling. In `step_response`, when `tf is None` and `max(Re λ) >= 0`, `warnings.warn('channel is unstable (pole at +3.13): the step response diverges; pass tf=')` (RULES 4.12), and let `step_info` return `nan` figures for a diverging response instead of a rise time.

**Why.** A wrong-sized `x_bar` on a three-state loop today surfaces as a JAX or NumPy broadcast traceback deep in the evaluator; a student cannot tell it from a library bug. And `step_response(InvertedPendulum())` silently returns a response that reaches 152 with `rise_time = 0.99 s`, `overshoot = 0.0` on the figure note, because `settling_horizon` reads the stable poles only.

**Evidence.** Probe: `(Lead(1,2,20) >> InvertedPendulum()).pzmap(x_bar=[0.0, 0.0])` (n = 3) -> `ValueError: Incompatible types for broadcasting: input type=float64[0] and requested type=float64[1]` under auto, `could not broadcast input array from shape (0,) into shape (1,)` under fd; `bode(u_bar=[0, 0])` -> `dot_general requires contracting dimensions ...`. derivatives.py:105-114 `operating_point` only reshapes. Probe: `step_response(InvertedPendulum(), x_bar=[0, 0])` -> `tf = 3.612`, `|y| max = 152`, no warning; `step_info` -> `rise_time=0.99, overshoot=0.0, peak=151.8, steady_state=nan`. linear.py:214-215 keeps `decay[decay > 1e-9]` only.

### analysis#7 — Pin the band's calling pattern with a signature test

*test · effort S · owner agent · rung v0.2 wave B · planned: P7*  
Files: `minilink/analysis/__init__.py:1-5`, `minilink/analysis/__init__.py:12-44`, `minilink/analysis/derivatives.py:18-30`, `minilink/analysis/equilibria.py:9`, `minilink/analysis/time_response.py:72`, `minilink/analysis/structural.py:28`, `minilink/analysis/discretize.py:85-91`, `minilink/analysis/modal.py:53-72`, `tests/unittest/test_public_imports.py:24-33`

**What.** A test that walks `minilink.analysis.__all__` and, for every callable whose first parameter is a system, asserts the positional prefix `(sys, x_bar=None, u_bar=None, t=0.0, params=None)` and the shared keyword names (`method`, `eps`, `n`, `backend`, `show`, `title`), with an explicit allowlist naming today's deviations (`jacobian`'s `of, wrt` first; `find_equilibrium(x_guess)` required; `step_info(time, y)`; `controllability(A, B)`; `discretize(system, dt, integrator=)`; `animate_modal(n_steps=, renderer=)`; `plot_region_of_attraction(certificate)`), so the list can only shrink (RULES 6.12).

**Why.** The band docstring promises 'Every tool reads tool(sys, x_bar, u_bar, t, params, *, method="auto", eps)' and seven exported names break it; P7's generated facades will copy whatever signatures the verbs have, so the pattern must be pinned on the verbs first. A cheap rule as a test stops the next verb (P2 `minreal`, P5 `sensitivity`, P4 factories) from drifting.

**Evidence.** analysis/__init__.py:3-4 states the pattern. equilibria.py:9 `def find_equilibrium(sys, x_guess, u_bar=None, ...)`; time_response.py:72 `def step_info(time, y)`; structural.py:28 `def controllability(A, B=None)`; discretize.py:85-91 `def discretize(system, dt=None, *, integrator='rk4', params=None)`; modal.py:65-67 `n_steps=2001, time_factor_video=3.0, renderer='matplotlib'` where the plot verbs use `n` and `backend`. test_public_imports.py:24-33 only checks that `bode` and `modal_analysis` are callable.

### analysis#8 — Default find_equilibrium's guess to sys.x0 like the rest of the band

*api · effort S · owner maintainer · rung v0.2 wave B · planned: —*  
Files: `minilink/analysis/equilibria.py:9`, `minilink/analysis/lyapunov.py:324`, `minilink/core/facades.py:1176-1183`

**What.** `find_equilibrium(sys, x_guess=None, u_bar=None, t=0.0, params=None, *, tol=1e-9)` with `None` meaning `sys.x0` (the name stays, positional callers are untouched); drop the hand-filled default in `region_of_attraction`.

**Why.** It is the one verb where `plant.find_equilibrium()` raises `TypeError: missing 1 required positional argument`, while `linearize()`, `bode()`, `modal_analysis()` all start from `x0`; the Lyapunov tool already re-implements the default around it.

**Evidence.** equilibria.py:9 `def find_equilibrium(sys, x_guess, u_bar=None, t=0.0, params=None, *, tol=1e-9)`; lyapunov.py:324 `x_bar = find_equilibrium(sys, sys.x0 if x_bar is None else x_bar, u_bar, t, params)`. Probe: `Pendulum().find_equilibrium()` -> TypeError.

### analysis#9 — Give the five control plots one keyword set and the region plot the same return type

*api · effort S · owner agent · rung v0.2 wave D · planned: D2*  
Files: `minilink/analysis/frequency.py:254-271`, `minilink/analysis/frequency.py:287-300`, `minilink/analysis/frequency.py:311-325`, `minilink/analysis/frequency.py:339-354`, `minilink/analysis/time_response.py:94-109`, `minilink/analysis/lyapunov.py:370-383`, `minilink/analysis/lyapunov.py:437`, `minilink/analysis/modal.py:53-72`

**What.** `title: str | None = None` on `plot_pzmap`, `plot_root_locus`, `plot_nyquist`, `plot_step_response` as `plot_bode` already has; `plot_region_of_attraction` takes `backend='matplotlib'` and `show` and returns a `PlotResult` (keeping `ax=` for overlays) instead of `(fig, ax)`; `animate_modal(n_steps=, renderer=)` documented as the `animate()` vocabulary or aligned to `n` (the `renderer` name is shared with `animate`, so only `n_steps` -> `n` is a band-local fix).

**Why.** A student who learned `plant.plot_bode(title=...)` gets `TypeError` on `plot_pzmap(title=...)`; the region plot is the one analysis figure that cannot go to plotly and returns a different object, which blocks the D1 'one library verb per cell' sweep from treating it like its siblings.

**Evidence.** Probe over signatures: `plot_bode: title=True`, `plot_pzmap/plot_root_locus/plot_nyquist/plot_step_response: title=False`. lyapunov.py:381-383 `ax=None, show=True` and :437 `return ax.figure, ax`, versus frequency.py:271 `-> PlotResult`. modal.py:65 `n_steps=2001` vs time_response.py:53 `n: int = 500`.

### analysis#10 — Overlay trajectories on the region plot without touching sys.x0 or sys.traj

*trap · effort S · owner agent · rung v0.2 wave D · planned: T4*  
Files: `minilink/analysis/lyapunov.py:585-593`, `minilink/core/facades.py:456-470`, `minilink/core/facades.py:515`

**What.** In `draw_region`, replace the `x0` swap-and-restore around `cert.sys.compute_trajectory(tf=horizon, verbose=False)` with the facade's own `x0=` keyword or, better, the `Simulator` the facade delegates to, so the user's `sys.x0` is never written and the user's `sys.traj` is not overwritten by the last overlay.

**Why.** T4's ledger names the `x0` swap; it does not say that `compute_trajectory` already accepts `x0=` (so the swap is dead weight) nor that every call also assigns `self.traj`, so `loop.plot_region_of_attraction(trajectories=[...])` silently replaces the simulation the student just ran (RULES 2.4: `traj` is the one shortcut a student reads back).

**Evidence.** lyapunov.py:585-593: `x0_saved = ...; try: for x0 in trajectories: cert.sys.x0 = ...; traj = cert.sys.compute_trajectory(tf=horizon, verbose=False) ... finally: cert.sys.x0 = x0_saved`. facades.py:463 `compute_trajectory(..., x0=None, ...)`; facades.py:515 `self.traj = traj`.

### analysis#11 — Free linearize and discretize from the band-facade name collision

*trap · effort M · owner maintainer · rung v0.2 wave B · planned: P7*  
Files: `minilink/analysis/__init__.py:11-12`, `minilink/analysis/linearize.py:1`, `minilink/analysis/discretize.py:1`, `minilink/control/__init__.py:1-10`, `tests/unittest/test_public_imports.py:24-33`, `DESIGN.md:40`

**What.** Either rename the two modules (`analysis/linearization.py`, `analysis/discretization.py`, call sites in tests and DESIGN updated in the same change, RULES 3.4) so `linearize` and `discretize` join `_EXPORTS`, or, at minimum, add the note `control/__init__.py` carries for `lqr` to the analysis docstring and a `test_public_imports` line that records the constraint.

**Why.** `from minilink.analysis import bode` is the documented band layer (RULES 4.1, DESIGN row 40) but the same line with the flagship verb `linearize` returns the submodule and fails only at call time with `'module' object is not callable`; `discretize` behaves the same. The root prelude works, so the trap is silent until a student generalizes from the documented example.

**Evidence.** analysis/__init__.py:11 `# Only names that do not collide with submodule filenames.`; probe: `from minilink.analysis import linearize` -> `module`; calling it -> `TypeError: 'module' object is not callable`; same for `discretize`. control/__init__.py:8-10 documents the identical `lqr` case; the analysis docstring does not. test_public_imports.py:26 sidesteps it with `from minilink.analysis.linearize import linearize`.

### analysis#12 — Put the seven missing analysis modules on the API page and trim the two placeholder docstrings to their step ids

*docs · effort S · owner agent · rung v0.1 close-out · planned: P4*  
Files: `docs/api/analysis.rst:1-14`, `docs/index.rst:23`, `minilink/estimation/__init__.py:8-18`, `minilink/identification/__init__.py:6-17`, `DESIGN.md:143`, `docs/plans/TODO.md`

**What.** `analysis.rst` gains `automodule` entries for `derivatives`, `linear`, `frequency`, `time_response`, `lyapunov` and `discretize` (today only `linearize`, `structural`, `equilibria`, `modal` are documented). `estimation/__init__.py` and `identification/__init__.py` shrink to one line each plus the placement rule and the step id (P4, C4); DESIGN's factory example uses P4's name.

**Why.** `bode`, `margins`, `step_response`, `region_of_attraction`, `discretize` and `jacobian` have no page on the Sphinx site the R1 release links to. The placeholders cite a ROADMAP section that no longer exists and spell the Kalman factory two ways, so the P4 design is owned by three files that already disagree.

**Evidence.** docs/api/analysis.rst lists four `automodule` blocks. estimation/__init__.py:8 and identification/__init__.py:10: 'Planned modules (see ROADMAP.md teaching-release priorities)' (ROADMAP sections are 1-8, none by that name). estimation/__init__.py:11 `kalman.py — Kalman filter (+ kalman_design(A, C, Q, R) factory)`; DESIGN.md:143 `estimation.kalman_design(A, C, Q, R) -> KalmanFilter`; TODO P4: `kalman(A, B, C, Q, R)`.

### analysis#13 — Give the state-space base named input ports so observers and sensitivity blocks share one f

*api · effort M · owner maintainer · rung v0.2 wave B · planned: P4*  
Files: `minilink/dynamics/abstraction/state_space.py:65-73`, `minilink/dynamics/abstraction/state_space.py:122-136`, `minilink/analysis/linearize.py:131-135`, `minilink/analysis/derivatives.py:26-31`, `docs/plans/TODO.md`

**What.** `StateSpaceSystem(n=, inputs={'u': m, 'y': p}, ...)` (default `{'u': m}`) creating one port per name in order, with `B` read as the horizontally stacked matrix over those ports; `LuenbergerObserver(A, B, C, L)` is then `LTISystem(A - L C, [B, L], I)` with ports `u`, `y` -> `x_hat`, and P5's `S` / `T` / `PS` / `CS` and the Kalman filter reuse the same `f` instead of each writing `dx = ...`.

**Why.** P4 states the port shape `(u, y) -> x_hat` but not how a linear block gets two named inputs; today the base creates a single `u` port, so the first observer would hand-write an `f` the base already owns. The analysis side is ready: `jacobian(sys, 'f', 'u')` and `linearize`'s default `wrt` already stack every input port into one `B`, so `linearize(observer).B()` comes out as `[B, L]` without new code.

**Evidence.** state_space.py:65-73 `super().__init__(n=n, input_dim=m, output_dim=..., expose_state=True, ...)` (one `u` port). linearize.py:132-134 `if wrt is None: return [('u', None)] if sys.inputs else []` ('default every input stacked'); derivatives.py:29 `'u' (every input stacked)`. TODO P4: '`LuenbergerObserver(A, B, C, L)` as a `DynamicSystem` with ports `u`, `y` -> `x_hat`'.

### analysis#14 — Log dx and y on the simulated Trajectory so an equation-error fit has its data

*feature · effort S · owner maintainer · rung v0.2 wave C · planned: C4*  
Files: `minilink/core/trajectory.py:19-44`, `minilink/core/trajectory.py:90-181`, `minilink/simulation/simulator.py`, `minilink/identification/__init__.py:6-9`, `minilink/analysis/derivatives.py:18-75`

**What.** `Simulator` attaches `dx` (it already evaluates `f` on the output grid) and the primary output `y` as `signals`, or `Trajectory` gains a `derivative()` helper (central differences of `x` on `t`); `identification/fitting.py` then reads `traj.get_signal('dx')` for the equation-error residual `dx_k - f(x_k, u_k, t_k; p)` and falls back to the difference when a logged trajectory has none. Record in the C4 row that `jacobian(..., wrt='params')` drops non-float leaves (an `int` in `params` is silently not a decision variable).

**Why.** C4 says 'fitting.py on rollout_batch' but the equation-error fit the placeholder describes needs a measured derivative, and today a simulated `Trajectory` carries `(t, x, u)` only; the first fitting demo would otherwise hand-roll `np.gradient(traj.x, traj.t)`. Naming the float-leaf rule up front avoids a silent no-op when a plant's params carry an integer.

**Evidence.** Probe: `Pendulum().compute_trajectory(tf=1.0).signals` -> `[]`; trajectory.py has no derivative helper (methods listed at :90-181); `grep -n signals minilink/simulation/simulator.py` is empty. identification/__init__.py:6-9 promises 'equation-error and prediction-error fits'. Probe: with `plant.params['n_samples'] = 3`, `jacobian(plant, 'f', 'params')` keys are `['I','d','gravity','l','m']` on both backends (derivatives.py:60 documents 'float leaves only').

### Bugs reported by the analysis finder

#### bug analysis#0 — (high) discretize() on a diagram produces a StepSystem that cannot step: the flat dt key is rejected as an unknown subsystem id

Files: `minilink/analysis/discretize.py:119-135`, `minilink/analysis/discretize.py:66-79`, `minilink/core/wiring.py:24-56`

```text
import numpy as np
from minilink import PID, Pendulum, discretize
loop = PID(Kp=1.0, Ki=0.0, Kd=0.0) @ Pendulum()
disc = discretize(loop, dt=0.01)        # succeeds; sorted(disc.params) == ['ctl', 'demux', 'dt', 'error', 'sys']
disc.step(np.zeros(loop.n), np.zeros(loop.m))
# ValueError: Unknown subsystem ids in diagram params: 'dt'; available: 'ctl', 'sys', 'demux', 'error'
# cause: _merge_discretize_params writes merged['dt'] into the source's (nested) params and step() passes that dict to DiagramSystem.f, whose validate_diagram_params rejects it.
```

#### bug analysis#1 — (medium) discretize() drops the source's x0 (and state labels and input bounds); the shipped demo rolls out from rest

Files: `minilink/analysis/discretize.py:13-29`, `examples/demos/analysis/analysis_discretize.py:8-18`

```text
import numpy as np
from minilink import Pendulum, discretize
plant = Pendulum(); plant.x0 = np.array([0.5, 0.0])
plant.inputs['u'].lower_bound = np.array([-2.0]); plant.inputs['u'].upper_bound = np.array([2.0])
disc = discretize(plant, dt=0.05)
print(disc.x0)                                   # [0. 0.]  (plant.x0 is [0.5, 0])
print(disc.state.labels)                         # ['x[0]', 'x[1]']  (plant: ['theta', 'dtheta'])
print(disc.inputs['u'].lower_bound)              # [-inf]  (plant: [-2.])
traj = disc.compute_rollout(n_steps=40, u=np.zeros((40, plant.m)))
print(np.abs(traj.x).max())                      # 0.0 -> analysis_discretize.py plots a flat zero trajectory
```

#### bug analysis#2 — (low) DiscretizedDynamicSystem.step rejects the plant's own params and exposes dt as a parameter sensitivity

Files: `minilink/analysis/discretize.py:66-79`, `minilink/analysis/discretize.py:119-135`

```text
import numpy as np
from minilink import Pendulum, discretize
plant = Pendulum(); disc = discretize(plant, dt=0.01)
disc.step(plant.x0, np.zeros(1), 0, plant.params)   # KeyError: 'dt'
sorted(disc.jacobian('step', 'params'))             # ['I', 'd', 'dt', 'gravity', 'l', 'm']  -> a fit over params would tune dt
```

#### bug analysis#3 — (low) step_info rise time uses |y| thresholds, so an undershooting (non-minimum-phase) response reports the wrong 10-90 % time

Files: `minilink/analysis/time_response.py:138-145`, `minilink/analysis/time_response.py:132-135`

```text
import numpy as np
from minilink.analysis import linear
from minilink.analysis.time_response import step_info
# G(s) = (1 - s) / (s^2 + 2 s + 1): zero at +1, final value 1, undershoot to -0.213
A = np.array([[0.0, 1.0], [-1.0, -2.0]]); B = np.array([[0.0], [1.0]]); C = np.array([[1.0, -1.0]]); D = np.zeros((1, 1))
t = np.linspace(0, 12, 1201); y = linear.step_response(A, B, C, D, t)
step_info(t, y).rise_time                          # 4.5 s: the 10 % threshold is met by |y| during the undershoot
i10 = np.flatnonzero(y >= 0.1)[0]; i90 = np.flatnonzero(y >= 0.9)[0]; t[i90] - t[i10]   # 3.15 s (signed thresholds)
```

#### bug analysis#4 — (low) The margins printed on plot_bode differ from margins() because they are read off grids of different resolution

Files: `minilink/analysis/frequency.py:45`, `minilink/analysis/frequency.py:122`, `minilink/analysis/frequency.py:504-519`, `minilink/analysis/linear.py:283-308`

```text
from minilink import PID, Pendulum
from minilink.analysis import linear
from minilink.analysis.frequency import frequency_response
L = PID(Kp=2.0, Ki=1.0, Kd=0.5) >> Pendulum()
w200, G200 = frequency_response(L, n=200)      # the grid plot_bode's note is computed on
w2000, G2000 = frequency_response(L, n=2000)   # the grid margins() uses
linear.margins(w200, G200).phase_margin_deg    # 18.392
linear.margins(w2000, G2000).phase_margin_deg  # 18.381   (K=5: 9.766 vs 9.754)
```


## planning

### planning#0 — Give the Monte Carlo score one set of defaults and a backend that fits the law

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/evaluation.py:132-151`, `minilink/planning/evaluation.py:351-395`, `minilink/planning/planner.py:278-298`, `minilink/planning/reinforcement_learning/planner.py:468-479`, `minilink/planning/policy_synthesis/lookup_policy.py:61-78`, `examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb:480-481`

**What.** One owner for the evaluator defaults: `MonteCarloEvaluator(problem, dt=0.05, n_trials=100, backend="jax")` and `Planner.evaluate(policy, dt, n_trials=50, backend="numpy")` disagree on both `n_trials` and `backend`. Make `Planner.evaluate` read its defaults from the evaluator (or drop them and forward), and give `MonteCarloEvaluator` a default that works for every planner's law: `backend="numpy"`, or `"auto"` = JAX when `controller.compile(backend="jax")` succeeds, else NumPy (the two backends produce the same numbers, per DESIGN §6).

**Why.** The documented one-yardstick verb, `compare(...).evaluate(MonteCarloEvaluator(problem))`, raises a JAX-trace error on any DP or tabular solution because `LookupTableController.ctl` runs SciPy's `RegularGridInterpolator`; the GRO860 notebook had to learn to write `backend="numpy"`. A student who scores with `planner.solve(evaluate=True)` (50 trials, NumPy) and then with the evaluator (100 trials, JAX, a different random stream) sees two numbers for one law.

**Evidence.** evaluation.py:136-139 `dt=0.05, n_trials=100, ... backend="jax"`; planner.py:279 `def evaluate(self, policy, *, dt, n_trials=50, tf=None, backend="numpy")`; rl/planner.py:478 passes `backend="jax"` while DP/LQR/trajopt use the NumPy default. Probe: `MonteCarloEvaluator(problem, dt=0.1, n_trials=1).evaluate(vi_solution)` on the pendulum → `RuntimeError: Block 'Lookup Table Controller:u' is not JAX-traceable`; `backend="numpy"` → `J = 98.70`. The notebook cell at line 480 writes `backend="numpy"` for exactly this reason.

### planning#1 — Let a finite problem.tf pick the finite-horizon recursion in DynamicProgrammingPlanner.solve

*trap · effort M · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/planning/policy_synthesis/dp.py:108-138`, `minilink/planning/policy_synthesis/dp.py:657-660`, `minilink/planning/policy_synthesis/lqr.py:109-128`, `minilink/planning/evaluation.py:40-41`, `examples/teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb:150-162`

**What.** When `problem.tf` is finite, `solve()` should run `round(tf / grid.dt)` backward sweeps (what `solve_steps` does) and mark the record `fixed_horizon=True`, or at least warn once (RULES 4.12) that it is sweeping past `t = 0` to tolerance. `solve_steps(n)` stays for the lecture's explicit recursion.

**Why.** The 2026-09-17 ruling makes `problem.tf` the one owner of the horizon: `LQRPlanner` picks the differential Riccati equation from it, the evaluators charge `h` at it. DP reads a finite `tf` only as the time stamp of `h` (`final_time`) and then iterates to convergence into negative time, so the same problem gets a finite-horizon law from LQR and an infinite-horizon table from DP. A student comparing the two on one problem cannot tell why.

**Evidence.** dp.py:657-660 copies `problem.tf` into `final_time` only; dp.py:119 `self.value_iteration(self.options.max_iterations, stop_on_tol=True)` regardless of the horizon; dp.py:187 `t = tf - k * dt` runs negative. Probe: double integrator, `tf=1.0`, `dt=0.1` → `solve()` reports `converged in 193 sweeps` (the finite-horizon answer is 10 sweeps). lqr.py:109 `if self.problem.horizon_kind() == "finite": ... lqr_gain_schedule(...)`.

### planning#2 — Honour a callable infeasible_cost in the value-iteration table

*bug · effort S · owner maintainer · rung v0.2 wave C · planned: —*  
Files: `minilink/planning/policy_synthesis/dp.py:641-650`, `minilink/planning/policy_synthesis/dp.py:371-372`, `minilink/planning/policy_synthesis/dp_jax.py:136-138`, `minilink/planning/reinforcement_learning/environment.py:178-181`, `minilink/planning/problems.py:253-258`

**What.** `options_of` should announce with `warnings.warn` when it falls back to `out_of_bound_cost=1e6` because `problem.infeasible_cost` is a callable (minimal fix), and the table builders can honour it directly: `G[~admissible] = price(x_next[~admissible], t)` in `running_cost_table` and its `dp_jax` twin, since the successor of every inadmissible pair is already in the table. Then `problem.infeasible_penalty(x, t)` is one rule for DP, the learners and the Monte Carlo score.

**Why.** DESIGN §6 says every tool prices an exit with the problem's `infeasible_cost`, scalar or `infeasible_cost(x, t)`; the environment, RL and the score already do. DP silently replaces the callable with the 1e6 default, so a rocket-landing style exit price changes the table with no message, against RULES 4.12.

**Evidence.** dp.py:648 `if "out_of_bound_cost" not in given and isinstance(problem.infeasible_cost, float):` — a callable falls through to the dataclass default; dp.py:372 `G[~admissible] = INF` and dp_jax.py:138 charge one scalar; environment.py:181 `return price(x, t) if callable(price) else price`. Probe: `PlanningProblem(..., infeasible_cost=lambda x, t: 100.0)` → `DynamicProgrammingPlanner(...).options.out_of_bound_cost == 1000000.0`, zero warnings.

### planning#3 — One name and one keyword for the cost-to-go picture across the band

*api · effort S · owner maintainer · rung v0.2 wave A · planned: A4*  
Files: `minilink/planning/policy_synthesis/dp.py:420-438`, `minilink/planning/reinforcement_learning/tabular.py:297-300`, `minilink/planning/policy_synthesis/policy_eval.py:170-174`, `minilink/planning/policy_synthesis/plotting.py:15-27`, `minilink/planning/policy_synthesis/plotting.py:105-119`, `minilink/planning/results.py:427-437`, `minilink/planning/planner.py:244-246`, `minilink/planning/comparison.py:111`, `examples/teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb:165-191`

**What.** After the term's name freeze: `plot_cost_to_go` is the one name (on `PlanningSolution`, `Planner`, `Comparison`, `DynamicProgrammingPlanner`, `TabularLearningPlanner`, `PolicyEvaluator`), `plot_cost2go` kept as a thin alias until the notebooks move; and `jmax` is the one colour-scale keyword (`PolicyEvaluator.plot_cost2go` / `StateSpaceGrid.plot_value` take `vmax` today). Land with S54's clipping at `out_of_bound_cost`.

**Why.** Within one planner a student meets both `planner.plot_cost2go(jmax=INF)` and `planner.plot_cost_to_go(jmax=INF)`, and in the policy-evaluation notebook `evaluator.plot_cost2go(vmax=INF, ...)` two lines above `planner.plot_cost2go(jmax=INF, ...)`. RULES 4.7 asks for unified flag names; A4 is the naming quick-wins step.

**Evidence.** dp.py:420 `def plot_cost2go(self, **kwargs)` and planner.py:244 `def plot_cost_to_go(self, **kwargs)` on the same object; policy_eval.py:170-174 forwards to `grid.plot_value(..., vmax=)` while plotting.py:105-110 `plot_cost2go(result, *, jmax=None, ...)`; double_integrator_policy_evaluation.ipynb:165 `evaluator.plot_cost2go(vmax=INF, ...)` vs :191 `planner.plot_cost2go(jmax=INF, ...)`; docs/plans/naming.md does not list this pair.

### planning#4 — Give PolicyEvaluator the evaluators' verb and a field-shaped result

*api · effort S · owner maintainer · rung v0.2 wave A · planned: A1*  
Files: `minilink/planning/policy_synthesis/policy_eval.py:104-174`, `minilink/planning/evaluation.py:160-167`, `minilink/planning/results.py:352-358`, `docs/plans/fields.md:59-61`

**What.** `PolicyEvaluator.evaluate(policy=None) -> GridField` (A1's `GridField(grid, J)`, the same object DP's `cost_to_go` becomes), with `solve()` kept as an alias through the term and `value_at` / `plot_cost2go` reading the field. The policy could then be a call argument, as on `MonteCarloEvaluator.evaluate(controller)`, so one evaluator scores several laws on one grid.

**Why.** The two evaluators of the band do not read alike: `MonteCarloEvaluator(problem).evaluate(law) -> Evaluation`, `PolicyEvaluator(problem, grid=, policy=).solve() -> ndarray`. A bare node-indexed array is the one planning quantity that is not yet its object (the constitution's `Field`), and A1 already plans the field for this class.

**Evidence.** policy_eval.py:141 `def solve(self) -> np.ndarray` and :137 the policy bound in the constructor; evaluation.py:160 `def evaluate(self, controller) -> Evaluation`; fields.md:59 `PolicyEvaluator holds a GridField and value_at(x) delegates`; test_planning_solution.py:390-396 `np.all(np.isfinite(grid_eval.solve()))`.

### planning#5 — Derive the flat-kwarg key tuples from the option dataclasses, one overlay helper, and mirror the RRT pair's constructors

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/trajectory_optimization/planner.py:40-53`, `minilink/planning/trajectory_optimization/planner.py:185-242`, `minilink/planning/trajectory_optimization/planner.py:894-906`, `minilink/planning/search/rrt.py:39-54`, `minilink/planning/search/rrt.py:77-91`, `minilink/planning/search/rrt.py:151-195`, `minilink/planning/search/rrt_star.py:100-109`, `minilink/planning/search/rrt_star.py:142-203`, `minilink/planning/policy_synthesis/dp.py:641-645`

**What.** One helper on `planning/planner.py`, `overlay_options(bag, default_factory, **flat)`, that builds the allowed keys from `dataclasses.fields(default_factory)` and lays the non-`_UNSET` flats over the bag; the three copies (`_merge_trajopt_options`, `_merge_rrt_options`, `options_of`) call it and the three hand-copied key tuples go. A test asserts, per planner, that the constructor's keyword names equal the dataclass fields (RULES 6.12). While there: `RRTStarPlanner(problem, extender)` should default `extender=None` (KinodynamicExtender) and `metric=euclidean` like `RRTPlanner` (maintainer-owned signature, one line).

**Why.** Every new option is written three times (dataclass field, key tuple, constructor keyword) in each of three planners; a missed one is a silent `Unknown ... kwargs` error. The sibling RRT constructors read differently for no reason.

**Evidence.** planner.py:40-53 `_TRAJOPT_OPTION_KEYS = ("compile_backend", ...)` duplicates the twelve fields at :87-103; rrt.py:39-54 and rrt_star.py:100-109 do the same; the three overlay functions are line-for-line the same shape (planner.py:894-906, rrt.py:77-91, dp.py:641-645). rrt.py:151-157 `def __init__(self, problem, extender=None, *, metric=euclidean, ...)` vs rrt_star.py:142-147 `def __init__(self, problem, extender, *, metric=None, ...)`. The 4.4 ledger names only the optimizer-method tuple.

### planning#6 — Track RRT* goal nodes incrementally instead of rescanning the tree every extension

*performance · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/search/rrt_star.py:249-251`, `minilink/planning/search/rrt_star.py:346-362`, `minilink/planning/search/rrt_star.py:403-413`, `minilink/planning/search/tree.py:401-405`

**What.** Keep the goal-region nodes in a list on the planner: test only the new node in `_extend_once` (one `contains`), and after `_rewire_near` re-read the costs of the stored goal nodes (their membership does not change; `propagate_cost` already refreshed their `cost`). `_refresh_best_goal` becomes a `min` over that list.

**Why.** `_refresh_best_goal` runs `goal_region.contains` on every node of the tree after every successful extension: `max_nodes=5000` means about 12.5 million margin evaluations, which dominates the post-goal `optimize_after_goal` phase the DESIGN says RRT* is for; with a `FieldSet` goal each call is a scene query.

**Evidence.** rrt_star.py:249 `best_goal_node, improved = self._refresh_best_goal(goal_region, best_goal_node, cost_tol)` inside the `for _ in range(options.max_nodes)` loop; rrt_star.py:350-354 `for node in self.tree.nodes: if not goal_region.contains(node.x): continue ...`; tree.py:401-405 already propagates costs to rewired descendants.

### planning#7 — Make Planner.plot_solution the solution's own plot_trajectory

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: D1.4*  
Files: `minilink/planning/planner.py:230-238`, `minilink/planning/results.py:439-450`, `minilink/planning/comparison.py:238-255`, `examples/demos/core/readme_examples.py:104`, `examples/tutorial/showcase_minilink.ipynb:518`, `examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb:391`

**What.** Keep the name (17 student-facing call sites, name freeze) and delegate: `Planner.plot_solution(*, signals, backend, show=True)` returns `self.require_solution().plot_trajectory(...)`, so the figure carries the `plan (method)` / `nominal rollout (method)` title and the `show` keyword like every other planner shortcut.

**Why.** `plot_control_law` and `plot_cost_to_go` on the planner already delegate to the solution; `plot_solution` is the odd one, drawing an untitled `sys.plot_trajectory` and lacking `show=`, so the README example and the solution verb draw two different pictures of one plan. D1.4 moves the demos onto the solution verbs; this makes the two spellings equal first.

**Evidence.** planner.py:232-234 `return self.problem.sys.plot_trajectory(self.solution_trajectory(), signals=signals, backend=backend)` (no title, no `show`); results.py:439 `def plot_trajectory(self, *, signals=("x", "u"), backend="matplotlib", show=True)` → comparison.py:249-253 `kind = "plan" if solution.open_loop else "nominal rollout"; result.figure.suptitle(f"{kind} ({solution.method})")`; planner.py:240-246 the other two shortcuts delegate.

### planning#8 — One state-axis label helper for the three planning plot modules

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/comparison.py:297-305`, `minilink/planning/policy_synthesis/plotting.py:392-401`, `minilink/planning/search/plotting.py:403-413`

**What.** One function, e.g. `state_axis_labels(sys) -> list[str]` next to `graphical/signals/time_signals.py` (the module the comparison already imports), returning `"label [unit]"` per state axis; the three copies (`comparison.state_labels`, `policy_synthesis.plotting._label_axes`, `search.plotting._label_axes`) call it.

**Why.** Three modules own the same nine lines, each probing `getattr(sys.state, "labels", None)` and `units`, and they already drift (the search copy bounds-checks `i < len(labels)`, the other two do not). A1's `GridField` plots and D1.1's overlays will need the same labels a fourth and fifth time.

**Evidence.** comparison.py:298-299 `labels = getattr(sys.state, "labels", None); units = getattr(sys.state, "units", None)`; policy_synthesis/plotting.py:393-394 the same two lines; search/plotting.py:404-405 the same, then :408 `name = labels[i] if labels and i < len(labels) else f"x[{i}]"`.

### planning#9 — Route the parametric (MPC) optimizer through Optimizer's backend table

*api · effort S · owner agent · rung v0.2 wave D · planned: T5*  
Files: `minilink/planning/trajectory_optimization/planner.py:178-183`, `minilink/planning/trajectory_optimization/planner.py:637-652`, `minilink/optimization/optimizer.py:45-55`, `minilink/optimization/optimizer.py:127-149`

**What.** Delete `TrajectoryOptimizationPlanner._USER_OPTIMIZER_METHODS` and build the parametric backend with `Optimizer._select_backend(*_USER_OPTIMIZER_METHODS[method])` (made a public module function `select_backend(method, options)` in `optimizer.py`), so the offline and the compiled-parametric paths accept the same `optimizer_method` values.

**Why.** T5 ledgers the duplicated tuple as style; the behaviour gap is that `optimizer_method="ipopt"` or `"scipy_trust_constr"` solves offline but `compile_parametric_program()` raises `Unknown optimizer method ... for parametric compile`, so MPC is SLSQP-only for a reason no docstring states.

**Evidence.** planner.py:178-183 the planner's table holds one entry, `"scipy_slsqp"`; planner.py:639-644 `if method not in self._USER_OPTIMIZER_METHODS: raise ValueError(f"Unknown optimizer method {method!r} for parametric compile...")`; optimizer.py:45-55 lists three methods and :127-149 already turns a preset into a backend.

### planning#10 — Pin the scoring contract with a params.sets parity test across the three backends and the grid

*test · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/evaluation.py:30`, `minilink/planning/evaluation.py:199-202`, `minilink/planning/policy_synthesis/discretizer.py:214-217`, `minilink/planning/search/rrt.py:399-406`, `tests/unittest/test_planning_stochastic.py:264-305`

**What.** Extend `test_monte_carlo_backends_share_the_score_on_identical_starts` with a `PlanningProblem(..., X=ParamBox, params=ProblemParameters(sets={...}))` whose box shrinks under its params, and assert that `backend="jax"`, `"numpy"`, `"simulator"`, `Evaluation.of_trajectory` and `score_trajectory` agree on `failed` and `J`; a second assertion that `StateSpaceGrid` validity and `RRTPlanner._edge_is_free` reject the same point.

**Why.** A cheap rule-as-test (RULES 6.12) for the one scoring contract DESIGN §6 promises; today the NumPy and JAX paths disagree on a parametric set (see bugs) and nothing would catch a regression when A2/A5 make more sets read `params`.

**Evidence.** evaluation.py:30 `inside = np.array([problem.X.contains(traj.x[:, k]) for k in ...])` drops `params.sets`; evaluation.py:201 `env.X.margin(x_next, t_next, env.set_params)` passes them; the existing parity test (test_planning_stochastic.py:264) uses the default unconstrained `X`.

### planning#11 — One owner of the sets between StateSpaceGrid and the planner that uses it

*trap · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/policy_synthesis/discretizer.py:75-76`, `minilink/planning/policy_synthesis/discretizer.py:183-217`, `minilink/planning/policy_synthesis/dp.py:219-251`, `minilink/planning/policy_synthesis/dp.py:273-300`, `minilink/planning/policy_synthesis/dp.py:625-638`, `minilink/planning/policy_synthesis/policy_eval.py:124-139`

**What.** Minimal: `grid_of` and `PolicyEvaluator.__init__` raise when `grid.problem is not problem` (or when its `X`/`U`/`params.sets` differ). Cleaner, for A1's `GridField`: the grid stops holding a `problem` and is built from `(sys, X, U, dt)`, and every validity check reads the planner's problem.

**Why.** `X` and `U` have two owners once a grid is reused: `backward_step_loop` reads `self.problem.X` / `U`, `backward_step_table` reads the grid's precomputed masks built from `grid.problem.X` / `U`, so the `loop` and `numpy` backends can silently disagree, and `PolicyEvaluator(problem, grid=other_planner.grid, ...)` (the documented recipe) scores the grid's sets, not the problem's. RULES 5.6: one owner per quantity.

**Evidence.** discretizer.py:75 `self.problem = problem` and :186 `X, U = self.problem.X, self.problem.U` in the transition build; dp.py:221-222 `X = self.problem.X; U = self.problem.U` in the loop backend vs dp.py:282 `x_next, action_ok, x_next_ok = grid.transition(t)` in the table backend; dp.py:625-638 `grid_of` accepts any grid; test_planning_solution.py:390-396 passes another planner's grid to `PolicyEvaluator`.

### planning#12 — Lower the state set member-wise in the transcriptions

*performance · effort M · owner agent · rung v0.2 wave D · planned: D3*  
Files: `minilink/planning/trajectory_optimization/shooting.py:319-333`, `minilink/planning/trajectory_optimization/direct_collocation.py:345-367`, `minilink/planning/trajectory_optimization/direct_collocation.py:463-485`, `minilink/planning/problems.py:140`, `minilink/planning/problems.py:460-462`, `minilink/core/sets.py:158-165`

**What.** One lowering rule at the solver boundary (RULES 4.3): walk `problem.X` (an `IntersectionSet`'s members, else the set itself); a `BoxSet` member goes to decision bounds where the states are decision variables (collocation, multiple shooting) or to path margins only when finite (single shooting); every other member contributes margins. Same for `U`.

**Why.** Single shooting adds `2 n N` inequality rows of `+inf` on every default problem (`X` defaults to the unbounded box), dead work SLSQP happens to tolerate; collocation with `X = box & clearance` puts the box in the bounds and again in the margins through the intersection, doubling the Jacobian rows a JAX `jacfwd` allocates densely (the TODO at parametric_evaluator.py:81 already flags that allocation).

**Evidence.** shooting.py:319 `if problem.X is not None:` is always true since problems.py:140 `X = unconstrained(n) if self.X is None else self.X`, and sets.py:163 `margin = xp.concatenate((z - lower, upper - z))` is `inf` on it; direct_collocation.py:356-359 copies `X.bounding_box()` into the bounds and :471 `not isinstance(problem.X, BoxSet)` then adds the whole intersection's margins, box member included.

### planning#13 — Do not hand Ipopt the objective Hessian alone

*trap · effort S · owner agent · rung Later · planned: —*  
Files: `minilink/optimization/optimizers/ipopt.py:186-195`, `minilink/optimization/evaluators/jax_evaluator.py:417-421`, `minilink/optimization/optimizer.py:54`

**What.** Either drop `hess=` on the Ipopt adapter until the constraints carry theirs (Ipopt then uses its limited-memory approximation), or build the constraint Hessians on the JAX evaluator (`jax.jacfwd(jax.jacrev(h_raw))` contracted with the multipliers) and pass them as `constraints[i]["hess"]`. A test with `Optimizer(program, method="ipopt", compile_backend="jax", use_hessian=True)` on a constrained program, skipped without `cyipopt`.

**Why.** `cyipopt.minimize_ipopt` reads `hess` as the objective Hessian and assembles the Lagrangian from it plus each constraint's `hess`; a constrained program with `use_hessian=True` on JAX therefore either errors in cyipopt or runs on a Hessian missing the constraint curvature. Unverified here (cyipopt is not installed), hence a trap, not a bug; reachable only through `use_hessian=True` + JAX + `"ipopt"` + constraints.

**Evidence.** ipopt.py:190 `hess=program_evaluator.hessian if program_evaluator.has_hessian else None` beside constraint dicts built at :174-184 with `fun` and `jac` only; jax_evaluator.py:420-421 `elif use_hessian: self._jit_hess_J = jax.jit(jax.hessian(J_raw))` is the objective Hessian only.

### planning#14 — Decide PlanningProblem.metadata: document it or retire it

*docs · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/problems.py:113`, `minilink/planning/problems.py:145`, `minilink/planning/problems.py:218-224`, `minilink/planning/problems.py:427`, `minilink/planning/problems.py:454`, `tests/unittest/test_planning.py:192-194`

**What.** `metadata` is a public field of the core `PlanningProblem` that the docstring does not list and no library code reads; it is copied by `nominal()` / `as_stochastic()` and frozen into a `MappingProxyType`. Either add it to the Parameters section with its intended use (a demo tag), or remove it as dead API (maintainer pick, D2 style) once the two test lines move.

**Why.** The problem is the object every planner carries; an undocumented field on it is surface a student meets in `print(problem)` with no explanation, and a field nothing reads is maintenance cost with no owner.

**Evidence.** problems.py:113 `metadata: Mapping[str, object] | None = None` with no entry in the docstring at :59-100; grep over `minilink/`, `examples/`, `benchmarks/` finds no reader other than the coercion and the two copies; only test_planning.py:192-194 exercises it.

### Bugs reported by the planning finder

#### bug planning#0 — (medium) score_trajectory drops problem.params.sets, so the NumPy and simulator Monte Carlo backends disagree with the JAX backend on a parametric constraint set

Files: `minilink/planning/evaluation.py:30`, `minilink/planning/evaluation.py:199-202`, `minilink/planning/evaluation.py:253-254`, `minilink/planning/evaluation.py:292`

```text
import numpy as np
from minilink.core.system import DynamicSystem
from minilink.core.costs import QuadraticCost
from minilink.core.sets import BoxSet
from minilink.core.trajectory import Trajectory
from minilink.planning.problems import PlanningProblem, ProblemParameters
from minilink.planning.evaluation import score_trajectory

class DI(DynamicSystem):
    def __init__(self):
        super().__init__(n=2, input_dim=1, output_dim=2)
    def f(self, x, u, t=0, params=None):
        return np.array([x[1], u[0]])

class ParamBox(BoxSet):
    def margin(self, z, t=0.0, params=None):
        r = 1.0 if params is None else params["r"]
        return np.concatenate([z + r, r - z])

sys = DI(); X = ParamBox(-np.ones(2), np.ones(2))
problem = PlanningProblem(sys, x_start=[0, 0], x_goal=[0, 0], tf=np.inf, X=X,
    cost=QuadraticCost.from_system(sys, Q=np.eye(2), R=np.eye(1)),
    params=ProblemParameters(sets={"r": 0.5}), infeasible_cost=100.0)
traj = Trajectory(t=np.array([0.0, 0.1, 0.2]), x=np.array([[0.0, 0.7, 0.8], [0.0, 0.0, 0.0]]), u=np.zeros((1, 3)))
print(score_trajectory(problem, traj))   # (0.081, False): x = 0.7 is outside X with r = 0.5
print(X.contains([0.7, 0.0], params={"r": 0.5}))  # False

The JAX path (evaluation.py:201 `env.X.margin(x_next, t_next, env.set_params)`) and the DP grid (discretizer.py:215) pass the set params; `score_trajectory`, hence `evaluate_numpy`, `evaluate_simulator` and `Evaluation.of_trajectory`, score `X` on its defaults. Fix: `problem.X.contains(traj.x[:, k], float(traj.t[k]), problem.params.sets)`.
```

#### bug planning#1 — (medium) MonteCarloEvaluator's default backend cannot score a DP or tabular solution and the error blames the block

Files: `minilink/planning/evaluation.py:136-139`, `minilink/planning/evaluation.py:351-395`, `minilink/planning/policy_synthesis/lookup_policy.py:61-82`

```text
import numpy as np
from minilink import Pendulum
from minilink.core.costs import QuadraticCost
from minilink.planning import PlanningProblem, DynamicProgrammingPlanner, MonteCarloEvaluator

plant = Pendulum()
plant.state.lower_bound = np.array([-2*np.pi, -8.0]); plant.state.upper_bound = np.array([2*np.pi, 8.0])
plant.inputs["u"].lower_bound = np.array([-5.0]); plant.inputs["u"].upper_bound = np.array([5.0])
cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=[np.pi, 0])
problem = PlanningProblem(plant, x_start=[0, 0], x_goal=[np.pi, 0], cost=cost, tf=np.inf, X=plant.state.box, infeasible_cost=500.0)
vi = DynamicProgrammingPlanner(problem, x_grid=(21, 21), u_grid=(3,), dt=0.1, alpha=0.95, tol=0.5).solve()
MonteCarloEvaluator(problem, dt=0.1, n_trials=1).evaluate(vi)
# RuntimeError: Block 'Lookup Table Controller:u' is not JAX-traceable. Its f()/compute() likely performs in-place array mutation ...
# MonteCarloEvaluator(problem, dt=0.1, n_trials=1, backend="numpy").evaluate(vi) -> J = 98.70

`compare(VI=vi, ...).evaluate(MonteCarloEvaluator(problem))`, the documented one-yardstick verb, fails the same way; the GRO860 notebook `pendulum_value_iteration_vs_lqr_vs_rl.ipynb:480` works around it with `backend="numpy"`. `LookupTableController.ctl` runs SciPy's `RegularGridInterpolator`, so the `static_law` JAX compile at evaluation.py:369 cannot trace it.
```

#### bug planning#2 — (medium) DynamicProgrammingPlanner silently replaces a callable infeasible_cost with the 1e6 default

Files: `minilink/planning/policy_synthesis/dp.py:648-649`, `minilink/planning/policy_synthesis/dp.py:371-372`, `minilink/planning/policy_synthesis/dp_jax.py:138`

```text
import numpy as np, warnings
from minilink import Pendulum
from minilink.core.costs import QuadraticCost
from minilink.planning import PlanningProblem, DynamicProgrammingPlanner

plant = Pendulum()
cost = QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1))
problem = PlanningProblem(plant, x_start=[0, 0], x_goal=[0, 0], cost=cost, tf=np.inf,
                          X=plant.state.box, infeasible_cost=lambda x, t: 100.0)
with warnings.catch_warnings(record=True) as w:
    warnings.simplefilter("always")
    planner = DynamicProgrammingPlanner(problem, x_grid=(11, 11), u_grid=(3,), dt=0.1)
print(planner.options.out_of_bound_cost, len(w))   # 1000000.0 0

`options_of` tests `isinstance(problem.infeasible_cost, float)` (dp.py:648), so the declared callable price is ignored without the RULES 4.12 warning, while `RolloutEnvironment.infeasible_penalty` (environment.py:181) and `score_trajectory` (evaluation.py:38) honour it: the DP table and the Monte Carlo score of the same problem use different exit prices.
```

#### bug planning#3 — (low) DynamicProgrammingPlanner.value_at, plot_cost2go, plot_policy and animate_* raise AttributeError before solve instead of the planner's 'No solution' error

Files: `minilink/planning/policy_synthesis/dp.py:386-438`, `minilink/planning/policy_synthesis/dp.py:100`, `minilink/planning/planner.py:208-212`

```text
import numpy as np
from minilink import Pendulum
from minilink.core.costs import QuadraticCost
from minilink.planning import PlanningProblem, DynamicProgrammingPlanner

plant = Pendulum()
problem = PlanningProblem(plant, x_start=[0, 0], x_goal=[0, 0], tf=np.inf, X=plant.state.box,
                          cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1)))
planner = DynamicProgrammingPlanner(problem, x_grid=(11, 11), u_grid=(3,), dt=0.1)
planner.value_at([0.0, 0.0])
# AttributeError: 'NoneType' object has no attribute 'value_at'
# planner.plot_cost2go() -> AttributeError: 'NoneType' object has no attribute 'grid'
# planner.get_controller(interpolation="nearest") -> AttributeError on self.result.grid

`self.result` starts as `None` (dp.py:100) and these verbs read it directly, while `Planner.require_solution` (planner.py:208-212) exists for exactly this message; `plot_control_law` / `plot_cost_to_go` on the same planner raise the clear `ValueError`. Fix: a `require_result()` mirror, or route through `require_solution()`.
```


## graphics-blocks-interfaces

### graphics-blocks-interfaces#0 — Let plot_trajectory select a leaf's named output ports

*api · effort S · owner agent · rung v0.2 wave B · planned: —*  
Files: `minilink/graphical/signals/time_signals.py:114-135`, `minilink/graphical/signals/time_signals.py:313-339`, `minilink/core/facades.py:194-242`, `DESIGN.md:1152`

**What.** In build_signal_plot_spec, when sys is a leaf and signal_name is one of sys.outputs ("y", a manipulator's "p" / "pdot"), evaluate the port compute along traj (the same sweep reconstruct_internal_signals does for subsystems) and label the rows with the port's own labels and units; do the same for "id:port" traces instead of the synthetic f"{name}[{i}]" (line 321-326 keeps port.units but drops port.labels). API stays signals=("y",) / signals=("p",).

**Why.** The textbook output y = h(x, u) is the one channel a leaf cannot plot today: plant.plot_trajectory(signals=("y",)) raises "Unknown signal(s): y. Available signals: x, u". For a TransferFunction plant (GRO501) plot_trajectory shows the tf2ss realization states, not the output; for a manipulator the task-space p is unreachable without wrapping the leaf in a diagram. Ports that carry labels (SB3Controller copies the plant's onto its u port, gymnasium.py:305-333) lose them on the figure.

**Evidence.** Probe: TwoLinkManipulator().compute_trajectory(); build_signal_plot_spec(m, traj, signals=("y",)) -> ValueError 'Unknown signal(s): y. Available signals: x, u'; same for ("p",). _available_signal_names (time_signals.py:331-339) lists traj.signal_names = ('x','u') for a leaf and only enumerates subsystem ports for diagrams. DESIGN.md:1152 promises signals=("x", "u", "block:port") with no leaf output form. Diagram path works: ('sys:y',) -> labels ['sys:y[0]','sys:y[1]'] although the port has labels.

### graphics-blocks-interfaces#1 — Give Sys2Gym a Distribution for the start state

*api · effort M · owner agent · rung v0.2 wave C · planned: —*  
Files: `minilink/interfaces/gymnasium.py:60`, `minilink/interfaces/gymnasium.py:132-136`, `minilink/interfaces/gymnasium.py:180-197`, `minilink/interfaces/gymnasium.py:345-358`, `minilink/core/signals.py:176-177`

**What.** Replace the reset_mode string plus the three derived attributes (x0_lb, x0_ub, x0_std, computed as sys.x0 + 0.1 * state bounds) with one start law: Sys2Gym(sys, cost, x0=Uniform(lo, hi) | Gaussian(mean, std) | None) drawn with distribution.sample(key); default a Uniform over 0.1 of the state box when it is finite, else the singleton sys.x0 with a one-line warning (RULES 4.12). ProblemEnv.reset then calls problem.sample_x0 directly instead of drawing a uniform first (line 353). Keep reset_mode= as an alias for the term (the GRO860 notebooks pass it).

**Why.** A reset rule is a Distribution (RULES 2.5, CONSTITUTION §2); the bridge is the one place in the library that still hand-rolls uniform/gaussian draws, and its default breaks on any plant without finite state bounds (the README custom plant, every DynamicSystem subclass that sets none: signals.py:176-177 defaults to +-inf), including through from_problem where the problem already supplies the start set.

**Evidence.** Probe on an unbounded 2-state DynamicSystem: x0_lb=[-inf -inf], x0_ub=[inf inf], x0_std=[inf inf]; reset_mode='uniform' -> OverflowError: Range exceeds valid bounds; 'gaussian' -> observation [inf -inf]; Sys2Gym.from_problem(PlanningProblem(sys, x_goal=0, cost=..., tf=1)).reset() -> OverflowError because ProblemEnv.reset (line 353) runs the uniform draw before problem.sample_x0. Docstring (line 60) names the third mode 'determinist' while the code accepts any string as deterministic.

### graphics-blocks-interfaces#2 — Register the block names the facade exports and walk every band facade in the registry test

*test · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `minilink/blocks/__init__.py:32-41`, `tests/unittest/test_teaching_surface.py:61-81`, `tests/unittest/test_teaching_surface.py:184-228`, `docs/api/blocks.rst`, `docs/api/graphical.rst`

**What.** Add a test that walks each band facade's __all__ the way test_whole_root_prelude_is_checked_not_just_the_sample walks minilink.__all__ (docstring present, home in the teaching lane), so a facade name cannot exist outside the registry; then add NotchFilter, Washout, MLP (and NeuralNetwork or its replacement) to TEACHING_SURFACE["minilink.blocks"] and the missing modules to the site: blocks.step (ZOHHold is on the root prelude and in the table but not in docs/api/blocks.rst), blocks.neural, graphical.port_map (plot_control_law / plot_input_output_map), graphical.catalog and graphical.control.

**Why.** The registry claims 'tested as a set', but the band walk only covers the curated tuples: minilink.blocks exports 23 names and the table lists 19, so four student-facing blocks (used in examples/demos/blocks/signal_blocks.py) carry no docstring/lane guarantee, and the API site omits the modules that hold three teaching verbs. A rule that can be a loop is a test (RULES 6.12).

**Evidence.** Probe: sorted(set(minilink.blocks.__all__) - set(TEACHING_SURFACE['minilink.blocks'])) == ['MLP', 'NeuralNetwork', 'NotchFilter', 'Washout']. test_teaching_surface.py:184-207 iterates TEACHING_SURFACE only; the full walk at 209-228 is root-only. docs/api/blocks.rst lists basic, sources, transfer_function, routing, nonlinear, filters (no step, no neural); docs/api/graphical.rst lists time_signals, phase_plane, topology, export, animator, primitives, renderer (no port_map, catalog, control).

### graphics-blocks-interfaces#3 — Make minilink.graphical.catalog the one shapes-and-skins facade students import

*api · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/graphical/catalog/__init__.py:1-11`, `minilink/graphical/catalog/__init__.py:53-92`, `tests/unittest/test_teaching_imports.py:22-38`, `tests/unittest/teaching_import_allowlist.txt`, `tests/unittest/test_teaching_surface.py:158-170`, `README.md:78`, `minilink/graphical/catalog/racecar_skin.py:1-15`

**What.** Export racecar_skin_2d, racecar_skin_3d, racecar_frames, track_scene and frame_axes_skin from graphical.catalog; add "minilink.graphical.catalog" to TEACHING_MODULES and a "minilink.graphical.catalog" row to TEACHING_SURFACE (shapes, placement helpers, skins, camera factories, with the graphical prefix admitted to TEACHING_LANE_PREFIXES for that row); then, with the maintainer, move README.md:78 and showcase_minilink.ipynb from minilink.graphical.animation.primitives to the facade and delete the 17 graphical rows of the allowlist. S30 (glyph rename) stays separate.

**Why.** The facade's own docstring calls it 'the one-stop import surface for demos and student plants', yet no student material can use it under the import rule: TEACHING_MODULES names no graphical package, so the README custom plant, the showcase and ten demos import the internal graphical.animation.primitives, and five files (one of them the teaching notebook racecar_mpc.ipynb) import graphical.catalog.racecar_skin by its defining module. The allowlist 'only shrinks'; this is how it shrinks by 17 rows.

**Evidence.** teaching_import_allowlist.txt carries minilink.graphical.animation.primitives (readme_examples.py, showcase_minilink.ipynb, trajopt_holonomic_corridor.py), minilink.graphical.catalog (trajopt_holonomic_corridor.py), minilink.graphical.catalog.racecar_skin (mpc_racecar.py, mpc_racecar_dyn.py, racecar_mpc.ipynb) and more; grep over examples: 10 imports of graphical.animation.primitives, 5 of catalog.racecar_skin, 2 of catalog.skins. README.md:78 'from minilink.graphical.animation.primitives import Box, ground_line'. graphical/catalog/__all__ (lines 53-92) has no racecar entry.

### graphics-blocks-interfaces#4 — One keyword vocabulary across the plot verbs (title, ax, backend) and one PlotResult.axes shape

*api · effort M · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/core/facades.py:194-242`, `minilink/core/facades.py:591-620`, `minilink/graphical/signals/time_signals.py:173-207`, `minilink/graphical/signals/matplotlib_backend.py:136-139`, `minilink/graphical/port_map.py:69-88`, `minilink/graphical/port_map.py:160-179`, `minilink/graphical/phase_plane/phase_plane.py:120-172`, `minilink/graphical/control/matplotlib_backend.py:54`, `minilink/core/hybrid_diagram.py:225-280`

**What.** Write the shared keyword set in DESIGN §7 (backend, show, title; ax where matplotlib) and make every verb honour it: SharedSystemFacades.plot_trajectory forwards **kwargs and plot_time_signals takes title (drawn as a suptitle, not only the window title); plot_control_law / plot_input_output_map gain backend= (a plotly heatmap is one go.Heatmap) or raise a named error; plot_phase_plane refuses ax= with plotly instead of dropping it; HybridDiagram.plot_trajectory drops or implements its dead abscissa argument; PlotResult.axes is a list for stacked figures and an Axes for one panel, stated once and tested by walking the verbs with the shared kwargs.

**Why.** Students copy a kwarg from one verb to its sibling and get a TypeError or a silent no-op; the D1.1 side-by-side plots need ax= on the time-signal plot, which is the one verb without it.

**Evidence.** Probes: p.plot_trajectory(title='x') -> TypeError unexpected keyword 'title' (facade takes only signals/backend/show) while plot_time_signals(title=) works on plotly (kwargs -> fig.update_layout) and fails on matplotlib (_create_figure() got an unexpected keyword argument 'title'); plot_phase_plane(title=) OK; plot_trajectory(ax=) TypeError; plot_control_law(backend='plotly') TypeError; plot_phase_plane(backend='plotly', ax=1) silently OK. PlotResult.axes: plot_trajectory -> list (even one row), plot_bode -> list, plot_pzmap / plot_phase_plane / plot_control_law -> Axes (control/matplotlib_backend.py:54 'axes_out = axes[0] if n_panels == 1 else axes'). hybrid_diagram.py:230 accepts abscissa='t' and never reads it (line 276 always passes TIME_ABSCISSA_LABEL). The time-signal title only reaches set_window_title (matplotlib_backend.py:136-139), invisible in a notebook.

### graphics-blocks-interfaces#5 — One axis-label formatter and one unit convention (no brackets in the catalog)

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: T3*  
Files: `minilink/graphical/signals/matplotlib_backend.py:46-52`, `minilink/graphical/signals/plotly_backend.py:294`, `minilink/graphical/phase_plane/phase_plane.py:566-571`, `minilink/graphical/port_map.py:675-680`, `minilink/dynamics/abstraction/manipulator.py:45`, `minilink/dynamics/abstraction/manipulator.py:53`, `minilink/dynamics/abstraction/generalized_mechanical.py:44`, `minilink/dynamics/abstraction/generalized_mechanical.py:51`, `minilink/dynamics/catalog/manipulators/arms.py:139`, `minilink/dynamics/catalog/mass_spring_damper/linear.py:53`

**What.** One axis_label(label, unit) in graphical/common used by the four formatters (two of them byte-identical copies), plus a catalog-wide test that no state, input or output unit string starts with '[' so the formatter never has to guard; the manipulator bases then write units=["m"] like the rest of the catalog.

**Why.** Two conventions ('m' vs '[m]') and one formatter that does not guard give a manipulator task-space trace the label 'p[0] [[m]]' in plotly while matplotlib shows it correctly; three copies of the same bracket logic are three places to fix it.

**Evidence.** Probe: _create_figure(SignalPlotSpec(... SignalTrace('p', 0, 'p[0]', '[m]', ...))).layout.yaxis.title.text == 'p[0] [[m]]' (plotly_backend.py:294 f"{trace.label} [{trace.unit}]"), _ylabel_with_unit('p[0]', '[m]') == 'p[0]\n[m]'. TwoLinkManipulator outputs: q ['[rad]', '[rad]'], p ['[m]', '[m]'], pdot ['[m/s]', ...] versus state units ['rad', 'rad', 'rad/s', 'rad/s'] and mass_spring_damper 'm'. phase_plane.py:566 and port_map.py:675 are identical _format_axis_label bodies.

### graphics-blocks-interfaces#6 — One backend resolver and one optional-import helper for the graphical band

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/graphical/signals/time_signals.py:190-207`, `minilink/graphical/control/__init__.py:13-26`, `minilink/graphical/phase_plane/phase_plane.py:398-402`, `minilink/graphical/animation/animator.py:59-78`, `minilink/graphical/signals/plotly_backend.py:312-321`, `minilink/graphical/control/plotly_backend.py:156-165`, `minilink/graphical/phase_plane/phase_plane.py:308-315`, `minilink/graphical/animation/renderers/plotly_renderer.py:51-59`, `minilink/graphical/signals/matplotlib_backend.py:116`, `minilink/graphical/control/matplotlib_backend.py:25`, `minilink/graphical/phase_plane/phase_plane.py:204`, `minilink/graphical/port_map.py:308`, `minilink/graphical/diagrams/mermaid.py:39`, `minilink/graphical/diagrams/hybrid_mermaid.py:69`, `minilink/graphical/signals/signal_colors.py:22-34`, `minilink/graphical/animation/renderers/plotly_renderer.py:124`

**What.** graphical/common gains plot_backend(name) (aliases 'mpl'/'matplotlib', one error text, used by the signal, control, phase-plane dispatchers and make_renderer), require_plotly() / require_meshcat() / require_pygame() with one install hint each (the pattern of core.backends.require_jax, RULES 5.12), and apply_matplotlib_defaults() for the pdf/ps fonttype lines; hybrid_mermaid imports _mermaid_id from mermaid; plotly_color becomes matplotlib.colors.to_hex (matplotlib is a hard dependency; the hex table duplicates it).

**Why.** Four dispatchers disagree on aliases and error types today, four _import_plotly copies carry three messages, four renderers mutate global rcParams independently, and two exporters own the same id sanitizer — maintenance cost with no readability gain (RULES 5.6, 7.1).

**Evidence.** Probe: Pendulum().plot_trajectory(backend='mpl') -> ValueError "Unknown signal backend 'mpl'" while render_control_figure (control/__init__.py:16 key in ('matplotlib','mpl')) and _normalize_backend (phase_plane.py:398-402) accept it; plot_time_signals raises TypeError for a non-string backend, the others ValueError. grep: rcParams["pdf.fonttype"] = 42 at signals/matplotlib_backend.py:116, control/matplotlib_backend.py:25, phase_plane.py:204, port_map.py:308; _import_plotly at signals/plotly_backend.py:312, control/plotly_backend.py:156, plotly_renderer.py:51 plus an inline copy at phase_plane.py:308; _mermaid_id at mermaid.py:39 and hybrid_mermaid.py:69 (identical, the latter already imports two helpers from the former).

### graphics-blocks-interfaces#7 — Publish a renderer capability table and give each cell an honest fallback

*trap · effort M · owner agent · rung v0.2 wave D · planned: S43*  
Files: `minilink/graphical/animation/animator.py:289-295`, `minilink/graphical/animation/animator.py:318-340`, `minilink/graphical/animation/renderers/matplotlib_renderer.py:569-580`, `minilink/graphical/animation/renderers/meshcat_renderer.py:27-36`, `minilink/graphical/animation/renderers/meshcat_renderer.py:720-724`, `minilink/graphical/animation/renderers/plotly_renderer.py:311-380`, `DESIGN.md:380-386`, `DESIGN.md:1162-1164`

**What.** A table in DESIGN §7 (renderer x {2-D, 3-D, native, html, save, camera, live loop}) with a test that each cell works or warns; plotly export_animation = fig.write_html (one line, today save=True only warns and skips); matplotlib export through a gif_export_path twin of meshcat's html_export_path so 'clip.gif' is not written as 'clip.gif.gif'; meshcat honours the camera 4x4 (target, view-out column, T[3,3] as distance) through vis['/Cameras/default'] instead of ignoring it; the Colab meshcat branch stops overriding an explicit html=False.

**Why.** DESIGN §7 states 'one [camera] contract for all renderers' and the animate docstring says an explicit html flag is honoured; both are false for meshcat, the renderer the README GIF advertises for the UR5. Students hit the differences one warning at a time.

**Evidence.** meshcat_renderer.py:724 '# Meshcat uses the viewer default camera; camera is ignored.' so camera_follow_frame and animate(camera=...) do nothing there; animator.py:294-295 'if google.colab in sys.modules and renderer == meshcat: html = True' after the docstring 'Explicit True/False is honored'; probe: animate(traj, save=True, file_name='.../clip.gif') writes 'clip.gif.gif' (matplotlib_renderer.py:575 file_name + '.gif') while html_export_path (meshcat_renderer.py:27-36) keeps a given suffix; animate(renderer='plotly', save=True) -> UserWarning 'save=True is not supported for renderer=plotly; skipping export' (PlotlyRenderer defines no export_animation).

### graphics-blocks-interfaces#8 — Silence the renderers (RULES 4.6)

*trap · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `minilink/graphical/animation/renderers/matplotlib_renderer.py:573`, `minilink/graphical/animation/renderers/meshcat_renderer.py:711-714`, `minilink/graphical/animation/renderers/meshcat_renderer.py:733`, `minilink/graphical/animation/renderers/meshcat_renderer.py:736`, `minilink/graphical/animation/renderers/meshcat_renderer.py:783-788`, `minilink/graphical/animation/renderers/meshcat_renderer.py:809`, `minilink/graphical/animation/renderers/plotly_renderer.py:81`

**What.** Turn the frozen-geometry note (meshcat native playback freezes Arrow / TorqueArrow / trail geometry, which changes what is drawn) into warnings.warn; drop the 'Saving animation to ...' and 'Meshcat static frame ready.' prints or gate them on verbose=; have export_animation return the written path so a demo can show it when the path is the lesson; add the graphical tree to the no-print grep of test_repo_contract.

**Why.** Libraries are silent except under verbose=True (RULES 4.6); eight bare prints live in the renderers, and the one that matters (frozen dynamic geometry) is the one a student is most likely to scroll past because it is not a warning.

**Evidence.** grep 'print(' under minilink/graphical: matplotlib_renderer.py:573 f'Saving animation to {file_name}.gif ...'; meshcat_renderer.py:736 'Meshcat static frame ready.'; :784 'Note: meshcat native animation freezes per-frame dynamic geometry ...'; :809 f'Saving animation to {path} ...'; :711-714 Colab port notices; plotly_renderer.py:81 print(repr(self)). Probe stdout after animate(save=True, show=False): 'Saving animation to /tmp/.../clip.gif.gif ...'.

### graphics-blocks-interfaces#9 — Warn on the +-10 phase-plane window and stop duplicating the default bounds

*trap · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/graphical/phase_plane/phase_plane.py:509-540`, `minilink/graphical/phase_plane/phase_plane.py:432-453`, `minilink/graphical/port_map.py:382`, `minilink/graphical/port_map.py:600-614`, `minilink/core/signals.py:176-177`

**What.** When no bounds= is given, the state box is infinite and no trajectory is cached, warn once with the window used (RULES 4.12 in spirit: 'phase plane drawn on [-10, 10] x [-10, 10]; set state bounds or pass bounds=') instead of silently drawing it; move the one DEFAULT_BOUNDS constant to graphical/common so phase_plane.py:525/532 and port_map.py:382 read it; replace the getattr(getattr(sys, 'state', None), ...) probes with plain reads (every System has state.nominal_value / lower_bound / labels / units).

**Why.** A DynamicSystem subclass that sets no bounds (signals.py:176-177 defaults to +-inf, the README custom plant included) gets a +-10 rad x +-10 rad/s vector field with nothing to say and no hint why; the same magic number lives in two modules.

**Evidence.** Probe on a 2-state DynamicSystem with default bounds: build_phase_plane_spec(sys).x_bounds == (-10.0, 10.0) == y_bounds; phase_plane.py:525 'return -10.0, 10.0' and :532 again; port_map.py:382 '_DEFAULT_BOUNDS = (-10.0, 10.0)'. examples/demos/graphical/plot_phase_plane.py has to set pendulum.state.lower_bound / upper_bound by hand before plotting.

### graphics-blocks-interfaces#10 — A lazy facade for minilink.interfaces

*api · effort S · owner agent · rung v0.2 wave C · planned: —*  
Files: `minilink/interfaces/__init__.py:1-26`, `minilink/interfaces/gymnasium.py:28`, `minilink/interfaces/gymnasium.py:272`, `minilink/interfaces/gymnasium.py:374`, `tests/unittest/test_teaching_imports.py:37`, `ROADMAP.md (section 7 Later)`

**What.** interfaces/__init__.py uses core.facade.lazy_facade to export Sys2Gym, SB3Controller, ProblemEnv and to_gymnasium (gymnasium stays imported on first attribute access, so the Basic tier is untouched); TEACHING_MODULES then lists minilink.interfaces like every other band; the 25-line 'planned modules' docstring (torch.py / flax.py wrappers that ROADMAP §7 does not list) shrinks to the placement rule.

**Why.** Today 'from minilink.interfaces import Sys2Gym' raises ImportError, so the three GRO860 notebooks import the defining module (RULES 4.1 layer 3) and the import test had to admit minilink.interfaces.gymnasium as if it were a band; the docstring is prose that will drift from the roadmap.

**Evidence.** Probe: minilink.interfaces.__all__ is None; 'from minilink.interfaces import Sys2Gym' -> ImportError: cannot import name 'Sys2Gym'. gymnasium_interface.ipynb, drone_ppo_sb3.ipynb and pendulum_value_iteration_vs_lqr_vs_ppo_sb3.ipynb all read 'from minilink.interfaces.gymnasium import SB3Controller, Sys2Gym'; test_teaching_imports.py:37 lists 'minilink.interfaces.gymnasium' inside TEACHING_MODULES. interfaces/__init__.py:15-22 names torch.py / flax.py and cosimulation adapters; ROADMAP §7 Later lists MjxPlant, ROS2 / FMI, no torch/flax.

### graphics-blocks-interfaces#11 — Give Integrator and ZOHHold a dim like every static block

*api · effort S · owner maintainer · rung v0.2 wave C · planned: C3*  
Files: `minilink/blocks/basic.py:14-19`, `minilink/blocks/step.py:17-19`, `minilink/blocks/nonlinear.py:12`, `minilink/blocks/routing.py:48`, `minilink/blocks/routing.py:79`

**What.** Integrator(dim=1, k=1.0, x0=None) with ports of size dim and params['k'] broadcast (dx = k * u, y = x), ZOHHold(dim=1) likewise; defaults keep today's scalar behaviour so no notebook changes.

**Why.** Every static block takes dim (Saturation, DeadZone, Relay, Gain(dim=), Error(dim=), Sum(dim=)) but the two dynamic wiring blocks are hard-wired scalar, so integrating a 2-D velocity into a position, adding integral action on a vector error, or holding an m-dimensional command needs Mux / Demux scaffolding or a custom DynamicSystem. C3 adds Sine / Ramp / Chirp / Delay / Switch and does not mention the vector forms.

**Evidence.** Probe: inspect.signature(Integrator.__init__) == (self), Integrator().n == Integrator().m == 1; inspect.signature(ZOHHold.__init__) == (self), ZOHHold().n == 1 (basic.py:15 'super().__init__(n=1, input_dim=1, output_dim=1 ...)', step.py:17 the same); Saturation(lower, upper, dim=1), Gain(K, dim=None), Error(dim=1), Sum(signs, dim=1).

### graphics-blocks-interfaces#12 — Fold NeuralNetwork into MLP (one owner of the one-hidden-layer map)

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/blocks/neural.py:9-68`, `minilink/blocks/neural.py:71-141`, `minilink/blocks/__init__.py:40-41`, `examples/demos/compile/neural_controller_jax.py:9-16`, `tests/unittest/test_blocks.py:340-390`

**What.** NeuralNetwork(input_dim, output_dim, hidden_dim, seed, scale) is MLP(input_dim, output_dim, hidden=(hidden_dim,), activation='tanh') with a Gaussian-scale init and different parameter names (W1,b1,W2,b2 vs W0,b0,W1,b1). Maintainer pick: (a) MLP gains init='gaussian', scale= and NeuralNetwork becomes a two-line subclass / alias kept for the demo and the test, or (b) NeuralNetwork is retired (user-importable name: needs the explicit decision) with the demo moved to MLP.

**Why.** Two classes state the same equation y = W2 tanh(W1 u + b1) + b2 with two parameter vocabularies, so a student who trains one cannot load the weights into the other and the docs describe the block twice (RULES 7.1: text edited twice when code changes).

**Evidence.** Probe: NeuralNetwork(2, 1, hidden_dim=8).params shapes {'W1': (8, 2), 'b1': (8,), 'W2': (1, 8), 'b2': (1,)} vs MLP(2, 1, hidden=(8,)).params {'W0': (8, 2), 'b0': (8,), 'W1': (1, 8), 'b1': (1,)}; grep: NeuralNetwork is used by one demo (neural_controller_jax.py:16) and one test class; both classes are on the blocks facade (__init__.py:40-41) and neither is in the teaching-surface table.

### graphics-blocks-interfaces#13 — Add the graphical band to the textbook pass (T7)

*docs · effort M · owner agent · rung v0.2 wave D · planned: T1*  
Files: `docs/plans/TODO.md (section 5, T rows)`, `minilink/graphical/phase_plane/phase_plane.py:434`, `minilink/graphical/phase_plane/phase_plane.py:490-492`, `minilink/graphical/phase_plane/phase_plane.py:553-563`, `minilink/graphical/animation/camera.py:59-68`, `minilink/graphical/animation/animator.py:102`, `minilink/graphical/animation/animator.py:166`, `minilink/graphical/animation/animator.py:227`, `minilink/graphical/catalog/skins.py:46-54`, `minilink/graphical/catalog/skins.py:91-98`, `minilink/graphical/animation/renderers/matplotlib_renderer.py:618`, `minilink/graphical/animation/renderers/renderer.py:60`, `minilink/graphical/common/matplotlib_style.py:24-39`, `minilink/graphical/meshes.py:1-21`, `minilink/core/system.py:110-118`

**What.** A T7 row in TODO §5 with the same recipe (draw-list baseline through graphics_contract_helpers, byte-identical after; the CONSTITUTION §6 infrastructure exemption keeps renderer internals out of the equation-style rules). Rows: plain reads instead of getattr chains on attributes every System sets in __init__ (state.nominal_value / lower_bound / labels / units in phase_plane; camera_target / camera_plot_axes / camera_scale / camera_follow_frame in camera.py and animator.py, all assigned at system.py:114-118); skins probing hasattr(plant, 'a') and getattr(plant, 'track', 1.6); _native_animation created outside __init__ (5.7); the stale pointer to Animator._prepare_transforms (renderer.py:60); the preamble walls of matplotlib_style.py and meshes.py (5.23).

**Why.** The pass ladder T1–T6 skips the largest remaining band (12 k lines, 60 underscore methods on public classes, 200 module-level underscore helpers), so its probes and stale docs have no scheduled home; the animator's getattr defaults also hide typos in a plant's camera hints instead of failing (RULES 4.3).

**Evidence.** docs/reviews/2026-09-22-consolidation-review.md §4.1 lists five bands (core; blocks and control; dynamics; analysis and simulation; planning). phase_plane.py:434 getattr(getattr(sys, 'state', None), 'nominal_value', None); camera.py:59-68 getattr(source, 'camera_target' / 'camera_plot_axes' / 'camera_scale' / 'camera_follow_frame', default) while System.__init__ sets all four (system.py:114-118); animator.py:102/166/227 getattr(self.sys, 'camera_scale' | 'camera_plot_axes'); skins.py:48 hasattr(plant, 'a') and hasattr(plant, 'b'); renderer.py:60 ':meth:`minilink.graphical.animation.Animator._prepare_transforms`' (no such method; the animator has _resolve_frame / _build_frames); grep counts: 60 'def _' methods inside classes, 200 module-level.

### graphics-blocks-interfaces#14 — Retire the sources demo and __main__ that only exist for show_signal

*consolidation · effort S · owner maintainer · rung v0.2 wave D · planned: D2*  
Files: `minilink/blocks/sources.py:30-111`, `minilink/blocks/sources.py:296-325`, `minilink/blocks/filters.py:55-63`, `examples/demos/blocks/blocks_sources.py`, `examples/tutorial/01_blocks.ipynb (Sources cells)`

**What.** Whatever the D2 pick on Source.show_signal, two things the row does not say: (1) sources.py's __main__ is a 30-line eight-variant parameter sweep that RULES 5.18 classifies as a demo, and blocks_sources.py is the same sweep again with ax.set_title() calls mixed into API lines (RULES 6.11); both collapse to source.compute_trajectory(tf=...) + source.plot_trajectory() once (or source.show_signal once) — one line per source. (2) If show_signal stays, it should return a PlotResult like every other plot verb (today a (fig, ax) tuple) and take backend= / show=.

**Why.** The demo and the __main__ teach matplotlib handles instead of the library verb, and show_signal is the only plot method on the teaching surface whose return type is not PlotResult (DESIGN §8: 'plot_* returns PlotResult; show=False skips display').

**Evidence.** sources.py:111 'return fig, ax'; sources.py:296-325 __main__ loops demo_changes over eight params calling show_signal each time; blocks_sources.py repeats the loop with three variants and 'ax.set_title(f"Changed {key} -> {value}")'; filters.py:55-63 __main__ prints an amplitude the RULES 6.13 rule says should be a plot or a __str__. 01_blocks.ipynb cells 'step.show_signal(t0=0.0, tf=5.0)' / 'noise.show_signal(...)'.

### Bugs reported by the graphics-blocks-interfaces finder

#### bug graphics-blocks-interfaces#0 — (medium) Sys2Gym.reset fails or returns inf on any plant without finite state bounds (also through from_problem)

Files: `minilink/interfaces/gymnasium.py:134-136`, `minilink/interfaces/gymnasium.py:181-188`, `minilink/interfaces/gymnasium.py:352-358`, `minilink/core/signals.py:176-177`

```text
import numpy as np
from minilink import DynamicSystem, QuadraticCost, PlanningProblem
from minilink.interfaces.gymnasium import Sys2Gym
class P(DynamicSystem):
    def __init__(self): super().__init__(n=2, input_dim=1, output_dim=2, expose_state=True)
    def f(self, x, u, t=0, params=None): return np.array([x[1], -np.sin(x[0]) + u[0]])
sys = P(); cost = QuadraticCost.from_system(sys, Q=np.eye(2), R=np.eye(1))
Sys2Gym(sys, cost).reset(seed=0)                    # OverflowError: Range exceeds valid bounds
Sys2Gym(sys, cost, reset_mode='gaussian').reset(seed=0)   # observation [inf, -inf]
Sys2Gym.from_problem(PlanningProblem(sys, x_goal=np.zeros(2), cost=cost, tf=1.0)).reset(seed=0)  # OverflowError, although the problem supplies sample_x0
# cause: x0_lb = x0 + 0.1 * (-inf), x0_std = 0.1 * inf; ProblemEnv.reset draws the uniform before problem.sample_x0
```

#### bug graphics-blocks-interfaces#1 — (low) Animation frame schedule never draws the last sample and crashes on a one-sample trajectory

Files: `minilink/graphical/animation/renderers/timing.py:30`, `minilink/graphical/animation/renderers/timing.py:35`

```text
from minilink import Pendulum
from minilink.graphical.animation.renderers.timing import trajectory_frame_schedule, sim_index_for_frame
p = Pendulum(); p.x0[0] = 1.0
traj = p.compute_trajectory(tf=10.0, verbose=False)   # default 10 001-point grid
s = trajectory_frame_schedule(traj, 1.0)               # skip_steps 33, n_frames int(10001/33) = 303
float(traj.t[sim_index_for_frame(s.n_frames - 1, s)])  # 9.966, not 10.0: the settled final state is never drawn (GIF, html, native)
import numpy as np
from minilink.core.trajectory import Trajectory
p.animate(Trajectory(t=np.array([0.0]), x=traj.x[:, :1], u=traj.u[:, :1]), show=False, html=False)  # ValueError: cannot convert float NaN to integer (sim_dt = 0/0)
# fix: n_frames = ceil(nsteps / skip_steps) (sim_index_for_frame already clamps) and a guard for nsteps < 2
```

#### bug graphics-blocks-interfaces#2 — (low) animate(save=True, file_name='clip.gif') writes clip.gif.gif

Files: `minilink/graphical/animation/renderers/matplotlib_renderer.py:573-575`, `minilink/graphical/animation/renderers/meshcat_renderer.py:27-36`

```text
import os, tempfile
from minilink import Pendulum
from minilink.graphical.common.environment import override_env
override_env('script')
p = Pendulum(); p.x0[0] = 1.0
traj = p.compute_trajectory(tf=1.0, verbose=False)
d = tempfile.mkdtemp()
p.animate(traj, save=True, show=False, file_name=os.path.join(d, 'clip.gif'))
os.listdir(d)   # ['clip.gif.gif'] (matplotlib_renderer.py:575 file_name + '.gif'); the meshcat sibling html_export_path keeps a given suffix
```

#### bug graphics-blocks-interfaces#3 — (low) Plotly time-signal y-labels double the brackets of '[m]'-style units

Files: `minilink/graphical/signals/plotly_backend.py:294`, `minilink/dynamics/abstraction/manipulator.py:45`, `minilink/dynamics/abstraction/manipulator.py:53`

```text
import numpy as np
from minilink import DiagramSystem
from minilink.catalog import TwoLinkManipulator
from minilink.graphical.signals import SignalPlotSpec, SignalTrace
from minilink.graphical.signals.plotly_backend import _create_figure
t = np.linspace(0, 1, 5)
spec = SignalPlotSpec(title='t', t=t, traces=(SignalTrace('p', 0, 'p[0]', '[m]', t, 'tab:blue'),))
_create_figure(spec).layout.yaxis.title.text   # 'p[0] [[m]]'
# the same unit reaches a real plot through any diagram holding a manipulator: d.plot_trajectory(signals=('arm:p',), backend='plotly')
# since TwoLinkManipulator().outputs['p'].units == ['[m]', '[m]']; the matplotlib backend guards (_ylabel_with_unit), plotly does not
```

#### bug graphics-blocks-interfaces#4 — (low) HybridDiagram.plot_trajectory accepts a dead abscissa argument

Files: `minilink/core/hybrid_diagram.py:230`, `minilink/core/hybrid_diagram.py:276`

```text
# signature (hybrid_diagram.py:225-232): plot_trajectory(self, traj=None, *, signals=None, abscissa='t', show=True, backend='matplotlib', **kwargs)
# grep abscissa minilink/core/hybrid_diagram.py -> only line 230 (the parameter); line 276 always passes abscissa_label=TIME_ABSCISSA_LABEL and the last_result.plot(...) branch does not forward it
# hybrid.plot_trajectory(abscissa='k') therefore draws exactly what abscissa='t' draws, with no error (HybridSimResult.plot_computer is the tick-indexed view)
```


## examples-teaching

### examples-teaching#0 — Declare controller ports from `feedback_profile` so a student writes only `ctl`

*api · effort M · owner maintainer · rung v0.2 wave B · planned: P11 (the two GRO501 notebooks would be the first consumers)*  
Files: `examples/teaching/courses/udes_gro501/cartpole_static_controller.ipynb (code cell 12)`, `examples/teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb (code cell 7)`, `examples/teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb (code cell 6)`, `examples/tutorial/00_core.ipynb (code cell 24)`, `examples/demos/compile/diagram_compiling.py:27-41`, `examples/demos/compile/params_gradient.py:40-61`, `minilink/core/feedback.py`

**What.** `Controller(feedback="state" | "output", n_y=4, m=1, reference_dim=1)` (or the existing `feedback_profile` class attribute plus dims) adds the `y`/`x`, `r` and `u` ports itself and binds `u` to `self.ctl` with the right dependencies; the student's class becomes gains in `__init__` plus `ctl(self, x, u, t=0, params=None)`. `add_input_port` / `get_port_values_from_u` stay for multi-port laws.

**Why.** C5.3, C5.4, the policy-evaluation homework and tutorial 00 teach the same static law with four different port idioms; the declarations are boilerplate a first-hour student copies without understanding, and each copy drifts (`dependencies=("y",)` / `"all"` / `()`; `y = get_port_values_from_u(u, "y")` vs `p, v = u`). One declared shape removes the lesson's largest non-mathematical cell.

**Evidence.** cartpole_static_controller cell 12: `self.add_input_port("y", dim=4); self.add_input_port("r", dim=1, nominal_value=0.0); self.add_output_port("u", dim=1, function=self.ctl, dependencies=("y",))` then `y = self.get_port_values_from_u(u, "y")`. double_integrator_policy_evaluation cell 6: `self.add_input_port("x", dim=2); self.add_output_port("u", dim=1, function=self.ctl, dependencies="all")` then `p, v = u`. cartpole_dynamic_controller cell 7: `dependencies=()` plus `add_output_port("z", dim=4, function=self.compute_state)`. 00_core cell 24 `PropController(System)` with `dependencies=("r", "y")`. diagram_compiling.py:32-33 gives `r` a scalar `nominal_value=0.0` and `y` an array `np.array([0.0])`.

### examples-teaching#1 — Make `plot_diagram()` warn, not raise, when the `graphviz` wrapper is missing

*trap · effort S · owner maintainer · rung v0.1 close-out · planned: R1 (the wheel's dependency list is what the tag freezes)*  
Files: `minilink/graphical/diagrams/dot.py:37-41`, `minilink/graphical/diagrams/dot.py:132-148`, `pyproject.toml:36-38`, `pyproject.toml:67-69`, `install.md:6`, `install.md:169`, `examples/tutorial/00_core.ipynb (code cells 19, 21)`, `examples/tutorial/02_dynamics.ipynb (code cell 3)`, `examples/tutorial/06_hybrid.ipynb (code cells 5, 7)`

**What.** Either put the pure-Python `graphviz` wrapper in the base dependencies so only the `dot` binary is optional, or catch the wrapper's `ImportError` in the facade the way the binary's `ExecutableNotFound` is caught (dot.py:132-148) and warn. Then install.md's Basic tier stops telling students to skip `plot_diagram()`.

**Why.** Every tutorial from 00_core on calls `plot_diagram()`. On `pip install minilink` (the documented Basic tier) the first notebook stops at cell 19 with an ImportError, while a missing binary only warns and keeps the notebook running; two failure modes for one optional feature is a first-hour trap.

**Evidence.** dot.py:37-41 `except ImportError as exc: raise ImportError("Graphviz topology export requires the graphviz Python package.")`; dot.py:132-148 `warnings.warn("Could not render the diagram inline. Is the Graphviz binary installed? ...")` for the binary; pyproject.toml:67-69 `diagrams = ["graphviz"]` is an extra, not in `dependencies` (:36); install.md:6 "Skip `plot_diagram()` (needs Graphviz)"; 00_core cell 19 `auto.plot_diagram()`, cell 21 `diagram.plot_diagram()`.

### examples-teaching#2 — Let the linear records print themselves and add a `poles` verb

*api · effort S · owner maintainer · rung v0.2 wave B · planned: P7 (generated facades) and P8 (ζ / ω_n need a pole pair to read from); the `__str__` half is agent-lane*  
Files: `minilink/analysis/linear.py:19`, `minilink/analysis/linear.py:143`, `minilink/analysis/__init__.py:41`, `minilink/core/facades.py:622-723`, `minilink/dynamics/abstraction/state_space.py`, `minilink/blocks/transfer_function.py`, `examples/tutorial/04_analysis.ipynb (code cells 3, 5, 7)`, `examples/tutorial/01_blocks.ipynb (code cell 7)`, `examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb (code cells 6, 8)`, `examples/teaching/courses/udes_gro501/numpy_state_space.ipynb (code cells 50-58)`, `examples/teaching/topics/classical_control/frequency_response.ipynb (code cells 6, 29)`, `examples/demos/analysis/analysis_linearize.py:19-24`, `examples/demos/analysis/analysis_structural.py:13-15`, `examples/demos/analysis/analysis_frequency.py:20-22`

**What.** `LTISystem.__str__` prints A, B, C, D rounded; `TransferFunction.__str__` prints the numerator over the denominator as polynomials in s; `StructuralResult.__str__` prints "controllable: rank 2/2". Export `poles` (it already exists at analysis/linear.py:19) and add the facade `sys.poles(x_bar=None, u_bar=None)` beside `pzmap`; `closed_loop_poles` (linear.py:143) becomes `(K @ plant).poles()`.

**Why.** RULES 6.1/6.13: a report the library cannot give is first a `__str__` gap. 14 sites in 7 files write `np.linalg.eigvals(lin.A())` or `eigvals(A - B @ K)`; 4 files hand-print `np.round(lin.A(), 3)`; 4 files hand-print `G.numerator` / `G.denominator`. Tutorial 01's `print(tf)` cell today shows the generic System summary and never the transfer function it is about.

**Evidence.** Verified: `str(plant.linearize(x))` → 'Linearized Pendulum (LTISystem), n=2\n  inputs: u (1)\n  outputs: y (2), x (2)'; `str(TransferFunction([4],[1,1.2,4]))` → 'Transfer Function (TransferFunction), n=2 ...'. `poles(A)` and `closed_loop_poles(A, B, C, D, K)` exist in analysis/linear.py:19,143 but are absent from the `minilink.analysis` lazy table and from every facade (root-export probe: `poles []`). 04_analysis cell 3 `print("poles:", np.round(np.linalg.eigvals(lin.A()), 2))`; cartpole_lqr cell 8 `np.linalg.eigvals(A - B @ K_inf)`; frequency_response cell 29 `np.linalg.eigvals(T.jacobian("f", "x"))`; analysis_linearize.py:21-24 four `print("A =\n", np.round(lin.A(), 4))` lines.

### examples-teaching#3 — Fill the placeholder sections of tutorials 01, 02, 03, 05, 08 and 09

*docs · effort M · owner maintainer · rung v0.2 wave D · planned: D1.3 (its tutorial pass is a flatness sweep; the missing content is not in its scope)*  
Files: `examples/tutorial/01_blocks.ipynb (code cells 5, 7)`, `examples/tutorial/02_dynamics.ipynb (markdown cell 6)`, `examples/tutorial/03_control.ipynb (markdown cell 2, code cells 3, 7)`, `examples/tutorial/05_simulation.ipynb (markdown cell 4)`, `examples/tutorial/08_optimization.ipynb (code cell 8)`, `examples/tutorial/09_planning.ipynb (markdown cell 4)`, `examples/tutorial/10_graphical.ipynb (code cells 13-23)`, `README.md:225`

**What.** Each heading gets the verbs it promises: 01 puts `Sum` in a diagram instead of `print("Sum block:", Sum)`; 02 "More plants" hosts the three-plant loop that now sits under 03's LQR heading; 03 moves its LQR cell under the heading that announces it and replaces `print(ComputedTorqueController.__name__, ...)` with one computed-torque loop; 05 "Simulator and solver modes" shows `Simulator(..., solver="scipy_ivp")` beside `rk4_fixedsteps`; 08 "Optimizer backends" runs `compile_backend="jax"` instead of `inspect.signature(Optimizer.__init__)`; 09 adds one `RRTPlanner` and one `DynamicProgrammingPlanner` cell under "Spatial, search, and DP"; 10_graphical drops its duplicated second half (cells 13-23 rebuild the pendulum under stale "## 9. / ## 10." numbering).

**Why.** README:225 promises "Tutorial series 00–11, one notebook per package". A student reaching 03 reads "then LQR stabilization" under a section that ends with a forced response; 09's planning chapter shows one of the four planners the README table lists; three cells print class objects or a signature instead of running anything.

**Evidence.** 03_control markdown cell 2: "Below: catalog sweep, forced cart-pole response, then LQR stabilization." while code cell 3 ends at `cartpole.plot_trajectory()`; cell 7 `print(ComputedTorqueController.__name__, SlidingModeController.__name__)`; 01_blocks cell 5 `print("Sum block:", Sum)`; 08 cell 8 `print(inspect.signature(Optimizer.__init__))`; 09 markdown cell 4 is a three-bullet list with no code after it; 05 markdown cell 4 ("construct a `Simulator`") is followed directly by markdown cell 5; 10_graphical cell 13 re-imports `Pendulum` and cell 14 rebuilds `sys`.

### examples-teaching#4 — Pin one Colab setup cell per tier with a test, and align install.md's tiers to it

*test · effort S · owner agent · rung v0.1 close-out · planned: R1 (the `%pip install minilink` switch), S49 (retires the SB3 variant)*  
Files: `examples/tutorial/*.ipynb (code cell 1)`, `examples/teaching/**/*.ipynb (code cell 1)`, `examples/teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb (code cell 1)`, `examples/teaching/topics/robotics/manipulator_eom.ipynb (code cell 1)`, `examples/tutorial/showcase_minilink.ipynb (code cell 1)`, `tests/demo_checks/run_notebook_checks.py`, `tests/unittest/test_repo_contract.py`, `install.md:17-25`, `install.md:138`

**What.** Three canonical cells (Basic; Basic + meshcat; Basic + gymnasium) stored once under tests/demo_checks, a test asserting every notebook under tutorial/ and teaching/ opens with one of them verbatim, and install.md's Colab snippets are those same strings. The 0.1.0 switch to `%pip install minilink` then edits one place; install.md:25 "PPO notebooks need Full" is rewritten for the native JAX PPO that runs on the Basic cell.

**Why.** 38 notebooks carry 11 distinct variants of the same seven-line cell ("Local conda" vs "Local", with or without meshcat, one `git clone -b main`, one computing an unused `_OPTIMIZER_METHOD`, one importing `sys` twice with an unused `Path`); RULES 6.12 says a rule a grep can check is a test, and no check reads the cell today.

**Evidence.** Exact-text census of the first code cell: 14 tutorial-style "+ meshcat"; 11 teaching "clone + path"; 2 GRO501 with a trailing blank line; 3 SB3 variants with three different comment lines; cartpole_rollout_gradients.ipynb `git clone -b main https://github.com/alx87grd/minilink`; manipulator_eom.ipynb cell 1 `import sys` twice and `_OPTIMIZER_METHOD` never used; 4 notebooks with no Colab cell (numpy_state_space, grid_world_dynamic_programming, sgd_line, least_squares_sgd). run_notebook_checks.py has no reference to the setup cell. install.md:138 installs `stable-baselines3` in the Full Colab cell; install.md:25 "PPO notebooks need **Full**" while the native `drone_ppo.ipynb` runs on the Basic cell (JAX ships on Colab).

### examples-teaching#5 — One name and one keyword for the cost-to-go plot across planner, evaluator and comparison

*api · effort S · owner maintainer · rung v0.2 wave D · planned: S54 (the colour-scale default of the same plot)*  
Files: `minilink/planning/policy_synthesis/dp.py:420`, `minilink/planning/planner.py:117`, `minilink/planning/policy_synthesis/policy_eval.py:88-92`, `minilink/planning/policy_synthesis/plotting.py:105-119`, `minilink/planning/comparison.py:111`, `minilink/planning/results.py:129`, `examples/teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb (code cells 8, 10)`, `examples/demos/value_iteration/vi_pendulum_lqr.py:44-51`

**What.** `PolicyEvaluator.plot_cost2go` takes `jmax` like `DynamicProgrammingPlanner.plot_cost2go` and `Comparison.plot_cost_to_go` (keeping `vmax` as the `grid.plot_value` alias); after the term's name freeze, `plot_cost2go` becomes an alias of the base-class `plot_cost_to_go`, so one verb draws the field on planner, solution, evaluator and comparison.

**Why.** In the double-integrator homework a student writes `planner.plot_cost2go(jmax=INF)` in one cell and `evaluator.plot_cost2go(vmax=INF)` in the next for the same field. `DynamicProgrammingPlanner` already carries both `plot_cost2go` (the grid table) and the inherited `plot_cost_to_go` (sampled on the box), two names for two pictures of one function.

**Evidence.** Band census: 16 `plot_cost2go(jmax`, 2 `plot_cost2go(vmax` (policy_evaluation cell 8), 3 `plot_cost_to_go(jmax`. policy_eval.py:92 `return self.grid.plot_value(self.last_J, **kwargs)` (no `jmax`); plotting.py:108 `jmax=None`; planner.py:117 `def plot_cost_to_go(self, **kwargs)` on the base class while dp.py:420 adds `def plot_cost2go(self, **kwargs)`.

### examples-teaching#6 — Overlay several trajectories on one phase plane and two fields on one grid surface

*feature · effort S · owner agent · rung v0.2 wave D · planned: D1.1 (lists five native plots; neither of these is on it), D1.4 (the comparison verbs)*  
Files: `minilink/graphical/phase_plane/phase_plane.py:120-135`, `minilink/core/facades.py:591`, `minilink/planning/policy_synthesis/discretizer.py:427`, `minilink/planning/policy_synthesis/plotting.py:15`, `minilink/planning/policy_synthesis/plotting.py:309`, `examples/tutorial/showcase_minilink.ipynb (code cell 33)`, `examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr.ipynb (code cells 27-28)`, `examples/teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb (code cells 24, 26, 28)`, `examples/demos/value_iteration/vi_double_pendulum_jax.py:116-117`

**What.** `plot_phase_plane(traj)` accepts a sequence or a `{label: trajectory}` dict and draws a legend; `grid.plot_value(J, compare=J_hat, show_3d=True)` draws the second field as a wireframe on the same axes. `Comparison.plot_phase_plane()` follows once D1.4 lands.

**Why.** The showcase's "four planners on one problem" ends in ten lines of matplotlib because the native plot takes one trajectory; the VI-vs-LQR homework shows two phase planes side by side where the lesson is the overlay; the cost-to-go approximation homework repeats a four-line `meshgrid` + `plot_wireframe` block three times. RULES 6.1: reporting goes through the objects' native plots.

**Evidence.** phase_plane.py:120-122 `def plot_phase_plane(sys, traj=None, ...)`; showcase cell 33 `phase = pend.plot_phase_plane(show=False)` then `phase.axes.plot(traj.x[0], traj.x[1], label=label, **style)` in a four-way loop and `phase.axes.legend()`; pendulum_value_iteration_vs_lqr cells 27-28 `plant.plot_phase_plane(traj_vi)` / `plant.plot_phase_plane(traj_lqr)`; pendulum_cost_to_go_approximation cells 24/26/28 `Z2 = grid_sys.slice_2d(grid_sys.grid_from_array(J_hat), 0, 1); X, Y = np.meshgrid(...); ax.plot_wireframe(X, Y, Z2.T)` three times.

### examples-teaching#7 — Stop teaching code from rebinding `sys`; add it to the flatness ratchet

*trap · effort M · owner maintainer · rung v0.2 wave D · planned: D1.2 (one more rule for its ratchet; the rule itself is agent lane), A4 / ROADMAP §6 naming (`sys` vs `plant` as the loop's subsystem id)*  
Files: `examples/tutorial/00_core.ipynb (code cells 1, 4, 21, 27)`, `examples/tutorial/09_planning.ipynb (code cell 3)`, `examples/tutorial/10_graphical.ipynb (code cell 14)`, `examples/teaching/courses/udes_gro501/cartpole_static_controller.ipynb (code cells 2, 5)`, `examples/teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb (code cells 2, 5, 9)`, `examples/teaching/courses/udes_gro860/pendulum_cost_to_go_approximation.ipynb (code cell 4)`, `examples/demos/mpc/mpc_car_minimal.py:20`, `examples/demos/rrt/rrt_pendulum_swingup.py:9`, `examples/demos/dynamics/lorenz_attractor.py:11`, `tests/unittest/test_teaching_imports.py`

**What.** Rename the plant variable to `plant` (README's own name) in the 25 files that bind `sys = <Plant>()`, and add one AST rule to the D1.2 ratchet: no assignment to the name `sys` under examples/tutorial, teaching, demos (today's files allowlisted, the list shrinking with the sweep).

**Why.** Every notebook opens with `import sys` for the Colab path; a few cells later `sys = Pendulum()` shadows the module, so `sys.x0` and `sys.animate()` read as if the standard library had them, and a student who types `sys.path` in the same kernel gets `AttributeError: 'Pendulum' object has no attribute 'path'`. The habit also loses state: the GRO501 dynamic-controller notebook sets input bounds and `x0` on `sys`, then rebinds `sys = CartPole()` and they are gone. `sys` is at the same time the subsystem id the `@` shortcut assigns (`"sys:x"`).

**Evidence.** 25 files in the band bind `sys = <Plant>(...)`. 00_core cell 1 `import sys` … cell 4 `sys = Pendulum()`; cartpole_static_controller cell 2 `import sys` … cell 5 `sys  = CartPole()`; cartpole_dynamic_controller cell 5 `sys.inputs["u"].upper_bound[0] = +20; sys.x0[0] = 0.5` then cell 9 `sys = CartPole()`; showcase cell 10 `loop.plot_trajectory(signals=("ref:y", "sys:x", "ctl:u"))` uses `sys` as a subsystem id in the same notebook where `plant` is the variable.

### examples-teaching#8 — Keep each course pin byte-identical to its topic twin with a `cmp` test

*test · effort S · owner maintainer · rung v0.1 close-out · planned: —*  
Files: `examples/teaching/courses/udes_gro860/pendulum_value_iteration_vs_lqr_vs_rl.ipynb (code cells 4, 5, 18, 20-23)`, `examples/teaching/topics/reinforcement_learning/pendulum_value_iteration_vs_lqr_vs_rl.ipynb (code cells 4, 5, 18, 20-21)`, `examples/teaching/courses/udes_gro860/drone_ppo.ipynb`, `examples/teaching/topics/reinforcement_learning/drone_ppo.ipynb`, `examples/README.md:94`, `examples/teaching/courses/udes_gro860/README.md`, `tests/unittest/test_repo_contract.py`

**What.** A `(course, topic)` pair table in `test_repo_contract.py` asserting the code cells are identical (nbstripout already strips outputs), so an edit to the textbook page is the edit to the homework; when the course wants other constants it edits the topic page, or the pair leaves the table on purpose.

**Why.** Two owners of one lesson. The `drone_ppo` twins are identical; the `pendulum_value_iteration_vs_lqr_vs_rl` twins have drifted in torque, infeasible cost, pendulum mass, state box, training budget, a variable name and one extra cell, so the course notes and the textbook page now teach different problems under one name.

**Evidence.** `diff` of the code-cell dumps: `< TORQUE = 2.0 / > TORQUE = 1.0`, `< INF = 2000.0 / > INF = 500.0`, `< plant.params["m"] = 1.0 / > plant.params["m"] = 0.1`, `< plant.state.lower_bound = np.array([-3.0 * np.pi, -20]) / > ... [-2.0 * np.pi, -12]`, `< sol_ppo = ppo.solve(timesteps=400_000) / > ... 200_000`, `< race = compare(...) / > vi_lqr_ppo = compare(...)`, `< race.plot_cost_to_go(jmax=INF)` only in the course pin; the drone twins diff is empty.

### examples-teaching#9 — `Trajectory.from_rollout(x0, xs, us, dt)` for compiled and scanned rollouts

*api · effort S · owner maintainer · rung v0.2 wave A · planned: V1 (the differentiable closed-loop cost will need to hand back its rollout as a Trajectory)*  
Files: `minilink/core/trajectory.py:19-45`, `examples/teaching/topics/optimal_control/cartpole_rollout_gradients.ipynb (code cell 5, `as_trajectory`)`, `examples/demos/compile/cartpole_rollout_gradients.py:71-79`, `examples/demos/compile/pid_autotuning_jax.py:62-65`, `examples/tutorial/11_reinforcement_learning.ipynb (code cell 26)`, `examples/tutorial/showcase_minilink.ipynb (code cell 45)`

**What.** A classmethod `Trajectory.from_rollout(x0, xs, us, dt, t0=0.0)` (states `(N, n)` or `(N+1, n)`, inputs `(N, m)`, last input held) so a `jax.lax.scan` or `rollout_batch` result plots and animates with `plant.plot_trajectory(traj)`; the evaluator rollouts can grow an `as_trajectory=True` on top of it.

**Why.** Three files hand-write the same six-line wrapper (`np.vstack([x0, states])`, `u = np.vstack([u, u[-1:]])`, `t = dt * np.arange(...)`, transposes) and the two scan-based tutorial cells never wrap theirs, so those lessons end in raw arrays. CONSTITUTION §2: a tool that returns a trajectory returns the object, never its arrays.

**Evidence.** cartpole_rollout_gradients.py:71-79 `def make_trajectory(x0_vec, states, u_seq, dt_step): ... u = np.vstack([u, u[-1:]])  # hold last force at tf`; the notebook twin cell 5 `def as_trajectory(states, U)` with the same body; pid_autotuning_jax.py:65 `return Trajectory(t=plot_t, x=xs.T, u=plot_u_knots.T)`; trajectory.py exposes `resample`, `save`, `load` but no rollout constructor.

### examples-teaching#10 — Give the double-integrator homework its two missing verbs: entry time into a set and the Bellman residual

*api · effort S · owner maintainer · rung v0.2 wave A · planned: A5 (later nouns, each with a first consumer: the set-entry verb is one), T5 (grid internals)*  
Files: `examples/teaching/courses/udes_gro860/double_integrator_minimum_time.ipynb (code cells 14, 16)`, `examples/teaching/courses/udes_gro860/double_integrator_policy_evaluation.ipynb (code cell 14)`, `minilink/core/trajectory.py`, `minilink/core/sets.py`, `minilink/planning/policy_synthesis/dp.py:388-434`, `minilink/planning/policy_synthesis/discretizer.py`

**What.** `traj.first_time_in(Xf)` (a `Set`; the first sample time with `margin >= 0`, `inf` otherwise) and `planner.bellman_residual(J=None)` returning the residual field `J - min_u [g dt + J(x + f dt)]` on the grid, so the notebook keeps its hand-written definition as the lesson and checks it against the verb.

**Why.** Two notebooks hand-roll the same four-line arrival time, one as a bare cell and one as a helper (RULES 6.10); the residual cell reads `grid.x_next` and `grid.x_next_ok`, tables that T5 records as existing only under `precompute=True` (they start as `None` otherwise), so the cell breaks the moment a student copies `precompute=False` from the neighbouring cost-to-go homework.

**Evidence.** double_integrator_minimum_time cell 14 and policy_evaluation cell 14 (`def arrival_time(traj)`): `inside = np.linalg.norm(traj.x, axis=0) < EPS; t_arrival = traj.t[inside][0] if inside.any() else np.inf`; minimum_time cell 16 `x_next = grid.x_next  # (nodes, actions, n)` … `Q[~grid.x_next_ok] = INF`; pendulum_cost_to_go_approximation cell 21 builds a grid with `precompute=False`; TODO T5: "`x_next` / `action_ok` / `x_next_ok` only created under `precomputed`".

### examples-teaching#11 — Accept `optimizer_method="auto"` that picks Ipopt when installed

*api · effort S · owner maintainer · rung v0.2 wave D · planned: —*  
Files: `minilink/planning/trajectory_optimization/planner.py:90`, `minilink/planning/trajectory_optimization/planner.py:632-638`, `minilink/optimization/optimizer.py:139-142`, `examples/tutorial/showcase_minilink.ipynb (code cell 1)`, `examples/teaching/courses/udes_gro860/cartpole_lqr.ipynb (code cell 27)`, `examples/teaching/topics/robotics/manipulator_eom.ipynb (code cell 1)`, `examples/demos/control/trajectory_lqr_cartpole.py:20`, `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py:42`

**What.** `TrajectoryOptimizationPlanner` / `Optimizer` accept `optimizer_method="auto"` (Ipopt if `cyipopt` imports, else `scipy_slsqp`, announced once per RULES 4.12); `"scipy_slsqp"` stays the default so no baseline moves. The four `importlib.util.find_spec("cyipopt")` lines and the one hard-coded `"ipopt"` become a keyword.

**Why.** The Ipopt → SciPy fallback is user UX that RULES 6.2 explicitly allows, yet it lives as `import importlib.util` boilerplate a student reads past in the showcase's first cell and in a GRO860 homework; one copy is dead code, and the demo that hard-codes `"ipopt"` raises on pip Full and Colab.

**Evidence.** showcase cell 1 `_OPTIMIZER_METHOD = ("ipopt" if importlib.util.find_spec("cyipopt") is not None else "scipy_slsqp")`; cartpole_lqr cell 27 `OPTIMIZER = "ipopt" if importlib.util.find_spec("cyipopt") else "scipy_slsqp"`; trajectory_lqr_cartpole.py:20 the same line; manipulator_eom cell 1 computes `_OPTIMIZER_METHOD` and never uses it; trajopt_cartpole_collocation_jax.py:42 `optimizer_method="ipopt"`; optimizer.py:141 `raise ImportError("IpoptOptimizer requires the optional 'cyipopt' package.")`.

### examples-teaching#12 — Let `mpc @ inner_loop` close on a multi-block plant diagram

*api · effort M · owner maintainer · rung v0.2 wave D · planned: T6 (the mpc/controller.py conversation), P10 (documenting the `@` dispatch paths), S31 (the sampled loop as a System)*  
Files: `minilink/simulation/computer.py:237-251`, `minilink/core/hybrid_composition.py:24-70`, `minilink/core/hybrid_composition.py:182-190`, `minilink/control/mpc/controller.py:294-296`, `examples/teaching/topics/optimal_control/racecar_mpc.ipynb (code cells 2, 17)`, `examples/demos/udes_racecar/mpc_racecar_dyn.py:132-143`, `examples/demos/hybrid/hybrid_multi_rate.py:57-63`

**What.** When the plant is a `DiagramSystem` with boundary ports `u` and `y`, the `computer @ plant` path wires those instead of demanding a single-subsystem diagram, so the racecar notebook's `hybrid_closed_loop(computer.diagram, inner, schedule=computer.schedule, computer=computer, computer_out="u_ff", computer_in="y", plant_in="u", plant_out="y")` becomes `mpc @ inner`. `hybrid_closed_loop` stays as the explicit entry, exported through `minilink.simulation`.

**Why.** The kinematic demo teaches `mpc @ car`; the very next lesson (a PID speed loop under the same MPC) falls off the operator into an internal import and eight keyword arguments, inside a teaching notebook that RULES 4.2 says must import through the facades.

**Evidence.** hybrid_composition.py:182-188 `if isinstance(plant, DiagramSystem): if len(plant.subsystems) != 1: raise ValueError("auto wiring requires a single-subsystem plant diagram or leaf plant")`; computer.py:244-246 the `@` path imports `hybrid_closed_loop, resolve_hybrid_feedback_ports`; racecar_mpc.ipynb cell 2 `from minilink.core.hybrid_composition import hybrid_closed_loop` (a teaching_import_allowlist.txt row); mpc_racecar_dyn.py:132 comment "closed loop: hybrid, because the inner plant is a diagram".

### examples-teaching#13 — Close the facade gaps the import allowlist records

*api · effort S · owner maintainer · rung v0.2 wave A · planned: A3 (geometry re-exports), S31 (the hybrid names), D1.3 (the per-file sweep that empties the list)*  
Files: `tests/unittest/teaching_import_allowlist.txt (57 rows)`, `tests/unittest/test_teaching_imports.py:22-38`, `minilink/__init__.py`, `minilink/simulation/__init__.py`, `minilink/planning/__init__.py`, `examples/demos/realtime/game_cartpole.py:9`, `examples/demos/rrt/rrt_star_live.py:9-13`, `examples/demos/rrt/rrt_holonomic_obstacles.py:13-18`, `examples/demos/hybrid/sampled_smc_pendulum.py:6`, `examples/teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb (code cell 5)`, `examples/tutorial/showcase_minilink.ipynb (code cell 13)`, `README.md (section "What is a System", the `from minilink.core.kinematics import translation` and `from minilink.graphical.animation.primitives import Box, ground_line` lines)`

**What.** Export through the band facades the names demos and notebooks use that have no facade at all: `hybrid_closed_loop` and `PygameInput` from `minilink.simulation`; `RRTStarOptions`, `StraightLineSteering`, `disc`, `GaussianField`, `SceneHistory` from `minilink.planning` (or from A3's geometry package); `DynamicController` at the root beside `Controller`; `translation`, `Box`, `ground_line` from the root, since README's own custom-plant example needs them. The matching allowlist rows then delete.

**Why.** The allowlist "only shrinks" but holds 57 rows, four in tutorial or teaching notebooks, and README's first custom plant imports from two internal paths; the surface the R1 tag freezes should not need an allowlist for its own README.

**Evidence.** Root/facade export probe: `hybrid_closed_loop []`, `RRTStarOptions []`, `StraightLineSteering []`, `disc []`, `GaussianField []`, `PygameInput []`, `DynamicController ['minilink.core']` only; allowlist rows such as `examples/teaching/topics/optimal_control/racecar_mpc.ipynb minilink.core.hybrid_composition`, `examples/tutorial/showcase_minilink.ipynb minilink.graphical.animation.primitives`, `examples/demos/realtime/game_cartpole.py minilink.simulation.realtime`, `examples/demos/core/readme_examples.py minilink.core.kinematics`.

### examples-teaching#14 — `WhiteNoise` samples are rebuilt only by Simulator's pre-read `refresh()`; S29 would silently break the noise demos

*trap · effort M · owner maintainer · rung v0.2 wave A · planned: A5 (`NoiseSource`), T2 (`WhiteNoise.h` cannot trace), S29*  
Files: `minilink/blocks/sources.py:176-226`, `minilink/core/system.py:152-158`, `examples/demos/core/diagram_noise_ports.py:49-66`, `examples/demos/blocks/blocks_sources.py:27-31`, `examples/tutorial/00_core.ipynb (code cell 27)`, `examples/teaching/courses/udes_gro501/cartpole_dynamic_controller.ipynb (code cell 11)`

**What.** Let the noise block derive its draw from `params` at call time (A5's `NoiseSource(distribution, sample_period)`: seed, period and variance read inside `h`, no `_interpolators` cache, no `refresh()`), and until then add a test that `params["var"] = ...` followed by `compute_trajectory` changes the output, so S29 cannot land without it.

**Why.** `diagram_noise_ports.py` edits `params["var"]` and `params["sample_period"]` between three runs and never calls `refresh()`; it is correct today only because `Simulator` refreshes every block before solving, the pre-read `refresh()` S29 plans to drop. The sibling demo `blocks_sources.py:29` calls `noise.refresh()` by hand, so a student sees two rules for one block; and `WhiteNoise.h` raises whenever a `params` dict is passed, so any diagram with noise can neither trace nor take a parameter family.

**Evidence.** Verified: after `params["var"] = 1.0` with no `refresh()`, the `compute_trajectory` output already differs from the var = 0 run and equals the run after an explicit `refresh()` (the Simulator refreshed it). sources.py:214-220 `if params is None: params = self.params else: raise ValueError("The block needs to be refreshed to reflect changes in parameters")`; sources.py:174 "refresh() rebuilds them from params"; diagram_noise_ports.py:62-63 `params["var"] = 100.0` / `params["sample_period"] = 0.2` with no refresh; TODO S29: "`Simulator` drops its pre-read `refresh()`".

### Bugs reported by the examples-teaching finder

#### bug examples-teaching#0 — (low) Tutorial 00 §2 prints the plant's state labels on the "Diagram" line

Files: `examples/tutorial/00_core.ipynb (code cell 11)`

```text
Cell 11 reads `print("Diagram: ", diagram.n, sys.state.labels, sys.state.units)` — the diagram line reports the plant's labels and units, not `diagram.state.labels`. It is invisible today only because `diagram.n == sys.n == 2` (Step and ImpedanceController are static). Replace `ImpedanceController()` in cell 9 by `PID(Kp=1.0, Ki=1.0, Kd=0.1, tau=0.05)` and rerun cells 9 and 11: the line prints a state dimension of 3 or more next to the two pendulum labels.
```

#### bug examples-teaching#1 — (medium) `trajopt_cartpole_collocation_jax.py` hard-codes `optimizer_method="ipopt"` and fails without cyipopt

Files: `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py:42`, `minilink/optimization/optimizer.py:139-142`

```text
On the documented pip Full tier (`pip install "minilink[full]"`, which install.md says omits Ipopt/cyipopt) or a Colab runtime: `PYTHONPATH=. python examples/demos/trajopt/trajopt_cartpole_collocation_jax.py` raises `ImportError: IpoptOptimizer requires the optional 'cyipopt' package.` at `planner.solve()`. The nightly sweep passes only because the conda Full environment ships Ipopt; RULES 6.7 requires canonical demos to run on a standard student laptop without external build chains. Every sibling (showcase, cartpole_lqr.ipynb, trajectory_lqr_cartpole.py) probes `find_spec("cyipopt")` first.
```


## tests-ci-tooling

### tests-ci-tooling#0 — Run pytest with JAX and Ipopt in the CI regression job so the optional-extra tests are gated

*tooling · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `.github/workflows/test.yml:31`, `.github/workflows/test.yml:40`, `.github/workflows/test.yml:83-114`, `.github/workflows/nightly.yml:24-29`, `tests/demo_checks/flagship_manifest.json`

**What.** In the `regression` job (the only merge-gate job that installs jax), add a step `pytest -m "jax or ipopt"` (or plain `pytest`) after `pip install -e ".[dev,jax,visualization,plotting,diagrams]"`, and add `ipopt` to that extras list plus `coinor-libipopt-dev pkg-config` to its apt line (nightly.yml already does both). Bundle with R2 since it is the same file. Then `trajopt_cartpole_jax` (requires jax+cyipopt) and the `ipopt`-marked tests run in the gate instead of only in the non-gate nightly.

**Why.** AGENTS.md calls `test.yml` the merge gate and says it runs `pytest`, but the `test` matrix installs only `.[dev,rl,diagrams]` (none pull jax), and the `regression` job never calls pytest. Every `@pytest.mark.jax` test, all 49 tests of test_mpc.py (module-level importorskip at line 7), test_jax_planning.py, test_ur5_jax.py, test_engine_jax.py, test_catalog_backends.py and the five modules listed in the bugs are dark in CI; a broken JAX backend or catalog parity would merge green.

**Evidence.** .github/workflows/test.yml:31 `pip install -e ".[dev,rl,diagrams]"`; :40 `run: pytest`; :88 `pip install -e ".[dev,jax,visualization,plotting,diagrams]"` followed only by run_regression_check / run_flagship_demos / run_notebook_checks (lines 94-114); :98 comment `JAX flagships skip in the test job (no jax extra); gate them here`. pyproject extras: rl = ["gymnasium"], diagrams = ["graphviz"] (no jax). nightly.yml:24 installs `coinor-libipopt-dev`, :29 `.[dev,jax,visualization,plotting,rl,ipopt,diagrams]`. flagship_manifest.json entry `trajopt_cartpole_jax` requires ["jax","cyipopt"] → always `skip (missing cyipopt)` in the gate. Local run with jax hidden: 5 whole-module SKIPPED lines (see bugs).

### tests-ci-tooling#1 — Guard optional extras per test instead of per module: move mid-file importorskip into the JAX classes and fix marker misuse

*test · effort M · owner agent · rung v0.1 close-out · planned: —*  
Files: `tests/unittest/test_dynamics_catalog.py:533`, `tests/unittest/test_mechanical_robotics.py:274`, `tests/unittest/test_geometric_control.py:11`, `tests/unittest/test_racecar_plant.py:10`, `tests/unittest/test_racecar_tires.py:14`, `tests/unittest/test_catalog_backends.py:17`, `tests/unittest/test_catalog_backends.py:67`, `tests/unittest/test_analysis_lyapunov.py:247`, `tests/unittest/test_control_analysis.py:606-624`, `tests/unittest/conftest.py`

**What.** Replace the module-level `pytest.importorskip("jax")` calls that sit mid-file (or at the top of files whose tests are mostly NumPy) with `pytest.mark.jax` on the JAX classes/functions plus a `jax = pytest.importorskip("jax")` inside those tests or a module-scoped fixture; keep `jax` module-level only in files that are JAX end to end (test_mpc.py, test_engine_jax.py). In conftest.py, make the `jax` marker itself skip when `importlib.util.find_spec("jax") is None` so the marker is the single guard. Same pass: drop `@pytest.mark.plotting` on the matplotlib-only test at test_analysis_lyapunov.py:247 and the bare `@pytest.mark.optional` on the matplotlib animate tests (matplotlib is a core dependency), and move `test_catalog_check_registry_covers_every_catalog_plant` out from under `pytestmark = [optional, jax]` in test_catalog_backends.py so the registry contract runs on the Basic tier.

**Why.** `pytest.importorskip` at module level skips the whole module at collection, so the 29 NumPy catalog tests before test_dynamics_catalog.py:533, the 22 tests before test_mechanical_robotics.py:274, and ~29 of 31 NumPy PurePursuit/RateLimiter tests in test_geometric_control.py (only 2 use jax) are skipped whenever jax is absent — which is exactly the CI `test` matrix and a Basic-tier student install. Marker misuse compounds it: `-m "not optional"` drops tests that need nothing optional.

**Evidence.** Simulated with `sys.modules['jax']=None`: `SKIPPED [1] tests/unittest/test_dynamics_catalog.py:533: could not import 'jax'`, same for test_mechanical_robotics.py:274, test_geometric_control.py:11, test_racecar_plant.py:10, test_racecar_tires.py:14 (whole modules; 51+59+33+14+12 = 169 collected tests). test_geometric_control.py has 23 `def test_` at lines 41-389; `grep -c "jnp\.\|jax\."` = 3. test_catalog_backends.py:17 `pytestmark = [pytest.mark.optional, pytest.mark.jax]` covers line 67 `test_catalog_check_registry_covers_every_catalog_plant` (pure registry check). test_analysis_lyapunov.py:247 `@pytest.mark.plotting` on `test_plot_draws_the_slice_and_the_basin` which only imports matplotlib. conftest.py OPTIONAL_MARKERS only aggregates markers; it never skips.

### tests-ci-tooling#2 — Make the flagship manifests honest: `requires` must list every optional import a demo makes, and drop or validate dead `demo_id`s

*bug · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `tests/demo_checks/flagship_manifest.json`, `tests/demo_checks/run_flagship_demos.py:84-92`, `tests/fixtures/flagship_graphics/manifest.json`, `tests/unittest/test_flagship_graphics_contract.py`, `examples/demos/mpc/mpc_integrator_numpy.py:64`, `examples/demos/mpc/mpc_car_minimal.py:55`

**What.** Add `"graphviz"` to `requires` of `mpc_integrator_numpy` and `mpc_car_minimal` (they call `hybrid.plot_diagram()` which raises ImportError without the graphviz Python package), and add a contract test that every `requires` entry is a module name that `importlib.util.find_spec` understands and that every flagship-graphics `demo_id` names a flagship manifest id (today `cascade_path_tracking` and `mpc_minimal` name nothing; the field is otherwise unused, so alternatively delete it). Do not touch the demos (maintainer-owned).

**Why.** `requires` is the runner's only skip mechanism; a missing entry turns an environment difference into a red `pytest` run because test_demo_check_runners.py wraps the runner. In this session `pytest` (no diagrams extra) failed at `test_flagship_demos_exit_zero` while CI passes only because the `test` job happens to install `.[diagrams]`; the dead `demo_id`s mislead anyone reconciling the two manifests.

**Evidence.** run_flagship_demos.py output: `mpc_integrator_numpy fail (exit 1: ... hybrid_dot.py line 22 ... ImportError: Graphviz hybrid export requires the graphviz Python package.)`, `mpc_car_minimal fail (...)`, `8 passed, 2 failed, 2 skipped`; manifest entries have `'requires': []` for both. minilink/graphical/diagrams/hybrid_dot.py:19-24 raises on `import graphviz`. flagship_graphics/manifest.json rows: `{'id': 'bicycle_cascade_vehicle', 'demo_id': 'cascade_path_tracking', ...}`, `{'id': 'mpc_minimal_bicycle', 'demo_id': 'mpc_minimal', ...}` — flagship ids are signal_blocks, computed_torque_pendulum, diagram_closed_loop, plot_readme, diagram_compiling, rrt_holonomic_obstacles, animation_renderers, mpc_integrator_numpy, trajopt_cartpole_jax, mpc_car_minimal, c_export, c_export_proportional.

### tests-ci-tooling#3 — Gate the subprocess demo-check bridge tests behind a marker so the unit suite stays a unit suite

*performance · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/unittest/test_demo_check_runners.py:67-101`, `tests/unittest/test_packaging.py:29-48`, `.github/workflows/test.yml:40`, `.github/workflows/test.yml:42-68`, `.github/workflows/test.yml:105-107`, `tests/README.md:176`

**What.** Mark `test_catalog_checks_fast_exit_zero`, `test_flagship_demos_exit_zero`, `test_flagship_graphics_exit_zero` and `TestBuiltWheel` with a `demo_checks` marker (registered in pyproject `[tool.pytest.ini_options] markers`) and run the `test` matrix as `pytest -m "not demo_checks"`; keep one place that runs them (`regression` job already runs the same three runners; `packaging` job already builds and checks the wheel). Or reuse the existing opt-in env pattern (`MINILINK_NOTEBOOK_CHECKS=1`) as `MINILINK_DEMO_CHECKS=1`.

**Why.** The bridge tests spawn every flagship script (up to 12 subprocesses × 120 s timeout) and `python -m build` inside `pytest`, four times in the matrix, then CI runs the identical runners again in `regression`/`packaging`. That makes local `pytest` (the AGENTS 'before handoff' command) slow and environment-sensitive (see the graphviz failure) without adding coverage.

**Evidence.** test_demo_check_runners.py:78-84 `proc = self._run("tests/demo_checks/run_flagship_demos.py")`; :86-92 flagship graphics; :67-76 catalog checks; :103-105 notebook bridge is already opt-in (`if os.environ.get("MINILINK_NOTEBOOK_CHECKS") != "1": self.skipTest`). test_packaging.py:36-40 `subprocess.check_call([sys.executable, "-m", "build", "--outdir", str(out)], cwd=REPO)`. test.yml:105 `python tests/demo_checks/run_flagship_demos.py` in `regression`; :42-68 `packaging` job builds and runs check_wheel.py. Background full `pytest` in this session was still at 67 % after ~40 min on a loaded host.

### tests-ci-tooling#4 — One owner for the CI regression-gate flags shared by tests/run, tests/README and test.yml

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/run/_common.py:50-67`, `tests/run/run_regression_gates.py:22-27`, `.github/workflows/test.yml:94-96`, `tests/README.md`, `AGENTS.md:107`

**What.** Define `CI_GATE_ARGS = ["--suite", "all", "--tiny", "--factor", "10", "--speed-gate-suffixes", "solve_s,nlp_s,speedup"]` once in `benchmarks/run_regression_check.py` (exposed as `--preset ci` or a constant), make `tests/run/_common.run_regression(ci_mode=True)` use it, and add a tiny contract test that greps `.github/workflows/test.yml` and AGENTS.md for the same string so the three cannot drift.

**Why.** `tests/run/run_regression_gates.py` promises 'same flags as GitHub CI' but passes `--factor 6` while CI and AGENTS.md use `--factor 10`; a launcher that is stricter than CI produces local red / CI green confusion, and the flags are copied in four places.

**Evidence.** tests/run/_common.py:57-66 `if ci_mode: cmd.extend(["--tiny", "--factor", "6", "--speed-gate-suffixes", "solve_s,nlp_s,speedup"])`; run_regression_gates.py:22 comment `True → same flags as GitHub CI regression job`; test.yml:94 `python benchmarks/run_regression_check.py --suite all --tiny \` … `--factor 10`; AGENTS.md:107 same `--factor 10` command.

### tests-ci-tooling#5 — Give every teaching notebook a job that executes it: nightly `--all`, unique ids, and drop the stale intro branch

*tooling · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `tests/demo_checks/run_notebook_checks.py:48-57`, `tests/demo_checks/run_notebook_checks.py:170-178`, `tests/demo_checks/notebook_overrides.json`, `.github/workflows/nightly.yml:45`, `.github/workflows/test.yml:114`

**What.** Add `--all` (run `smoke: false` notebooks too, with their own timeout) and call it from nightly.yml; make `_notebook_id` include the course/topic folder for `examples/teaching/courses/*` and `examples/teaching/topics/*` (e.g. `teaching_udes_gro860_drone_ppo`) so `--notebook` and the overrides are unambiguous; delete the `/learn/intro/` branch and the `intro_00_core` help example (no such paths). Update the override keys in the same change.

**Why.** Ten notebooks are `smoke: false` (racecar_mpc, manipulator_eom, the value-iteration trio, the SB3 twins, gymnasium_interface, showcase_from_rl_to_bode) and no workflow ever runs them, so 'every teaching notebook' in AGENTS.md is not true; two ids collide so an override or `--notebook` filter silently applies to both copies.

**Evidence.** run_notebook_checks.py:52 `if "/tutorial/" in rel_path or "/learn/intro/" in rel_path:`; :177 help `e.g. showcase_minilink, intro_00_core`; `--help` lists only `--notebook` and `--timeout`. Collisions computed over 38 notebooks: `teaching_drone_ppo -> [courses/udes_gro860/drone_ppo.ipynb, topics/reinforcement_learning/drone_ppo.ipynb]`, `teaching_pendulum_value_iteration_vs_lqr_vs_rl -> [courses/udes_gro860/..., topics/reinforcement_learning/...]`. nightly.yml:45 and test.yml:114 both run `python tests/demo_checks/run_notebook_checks.py` with no extra flag. AGENTS.md:88 claims nightly runs 'every teaching notebook'.

### tests-ci-tooling#6 — Add ruff check and ruff format hooks to pre-commit so the 'always before push' gate runs itself

*tooling · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `.pre-commit-config.yaml`, `AGENTS.md:90-98`, `pyproject.toml`

**What.** Append the `astral-sh/ruff-pre-commit` repo with `ruff` (args `--fix`) and `ruff-format` hooks to `.pre-commit-config.yaml`; pin its rev to the ruff version in the dev extra. No new dependency: `pre-commit` and `ruff` are already in `dev`.

**Why.** AGENTS.md makes `ruff check .` and `ruff format --check .` mandatory before every push and warns CI lints the whole repo, but the only hook installed is nbstripout, so the step depends on memory; a two-hook addition makes the local gate identical to the CI `test` job's first two steps.

**Evidence.** .pre-commit-config.yaml: single repo `kynan/nbstripout rev 0.9.1`, hook `nbstripout` on `^examples/.*\.ipynb$`. AGENTS.md:90-98 'Always before push … ruff check . ; ruff format --check .' and 'CI runs these on the whole repo'. pyproject dev extra lists `pre-commit`, `ruff`.

### tests-ci-tooling#7 — One owner for the teaching-facade list shared by test_teaching_imports and test_teaching_surface

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: D1.2*  
Files: `tests/unittest/test_teaching_imports.py:22-38`, `tests/unittest/test_teaching_surface.py:21`, `tests/unittest/test_teaching_surface.py:171-172`, `tests/unittest/test_teaching_surface.py:279`

**What.** Move the facade list to one module (e.g. `tests/unittest/teaching_lane.py` exposing `TEACHING_FACADES` and `RESEARCH_LANE_PREFIXES`) and import it from both tests; decide explicitly whether `minilink.control.mpc` and `minilink.interfaces.gymnasium` are teaching facades (allowed in examples) or research lane (excluded from the Basic-tier surface) — today the two tests answer differently. While there, make `TestBasicTier` set `PYTHONPATH` from `REPO` rather than `os.getcwd()` so the subprocess probe works from any working directory.

**Why.** Two hand-copied lists with different membership means the import ratchet (D1.2 lives here) and the surface contract can disagree about the same module; the cwd-dependent PYTHONPATH makes the Basic-tier probe pass or fail depending on where pytest was launched.

**Evidence.** test_teaching_imports.py:28 `"minilink.control.mpc"`, :37 `"minilink.interfaces.gymnasium"` inside `TEACHING_MODULES`; test_teaching_surface.py:171-172 `RESEARCH_LANE_PREFIXES = ("minilink.control.mpc", …)` used at :197/:225/:318 to assert names do NOT live there; :279 `env["PYTHONPATH"] = os.getcwd()`.

### tests-ci-tooling#8 — Pin the Sphinx API pages to the teaching surface with a test and build docs with -W

*docs · effort M · owner agent · rung v0.1 close-out · planned: —*  
Files: `docs/api/`, `docs/conf.py`, `.github/workflows/docs.yml:42`, `tests/unittest/test_teaching_surface.py:21`, `tests/unittest/test_repo_contract.py`

**What.** Add `test_docs_api_covers_teaching_surface`: for each name in `TEACHING_SURFACE`, its `__module__` must appear in an `automodule::` directive under docs/api (or the facade page must use `:imported-members:`). Then switch docs.yml to `sphinx-build -W --keep-going -b html docs docs/_build/html` once it is clean, so a broken cross-reference fails the (non-gate) Docs workflow instead of silently shipping.

**Why.** Fourteen modules that define teaching-surface names have no API page: the whole frequency-analysis band (bode, pzmap, margins, transfer_function), step_response, jacobian, Gaussian/Uniform, Controller, MonteCarloEvaluator/Evaluation, ReinforcementLearningPlanner, TabularLearningPlanner, NeuralPolicyController, PurePursuit, StaticSimulator, ZOHHold, StateSpaceGrid, LookupTableController — students following the site cannot find what the README exports, and nothing catches the drift because the build never warns-as-error.

**Evidence.** Computed: `documented automodules: 85 | undocumented homes: 14` — minilink.analysis.frequency (transfer_function, bode, plot_bode, pzmap, plot_pzmap, margins …), minilink.analysis.time_response, minilink.analysis.derivatives, minilink.core.distributions, minilink.core.feedback, minilink.planning.evaluation, minilink.planning.reinforcement_learning.planner, .tabular, minilink.control.neural, minilink.control.geometric, minilink.simulation.static_simulator, minilink.blocks.step, minilink.planning.policy_synthesis.discretizer, .lookup_policy. docs/api/*.rst use `:members:` only (no `:imported-members:`). docs.yml:42 `run: sphinx-build -b html docs docs/_build/html` (no `-W`).

### tests-ci-tooling#9 — Move wall-clock speed claims out of unit tests into the regression gates

*test · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/unittest/test_mpc.py:511-523`, `tests/unittest/test_ur5_jax.py:163`, `tests/unittest/test_simulation.py:946-958`, `tests/unittest/test_simulation.py:918-944`, `benchmarks/suites/solve_speed.py`, `benchmarks/host_profiles.py`

**What.** Keep the structural assertions (`compile_time_s is not None`, a compile counter of 1, `n_overruns > 0`) and delete the timing comparisons `second_step_s < first_step_s`, `second_step_s < 0.5 * compile_s`, `t_jx < 0.8 * t_np`; where a speed claim matters, add a `speedup`-suffixed metric to the `solve_speed` suite, which already has host profiles and `--factor` tolerances. For the realtime calibration test, assert the ceiling rule from a stubbed probe (inject the measured step time) rather than from a live wall-clock probe.

**Why.** Strict wall-clock orderings on a loaded CI runner (four Python versions in parallel) flip randomly: two warm MPC steps can differ by scheduler jitter, and `sim_dt == frame_dt/10` only holds if the wall-clock probe finds more than ten steps fit in a frame. The repo already owns the right tool for speed claims (regression gates with host profiles); RULES 6.3 asks for tests only when they justify themselves.

**Evidence.** test_mpc.py:521-522 `self.assertLess(second_step_s, first_step_s)` / `self.assertLess(second_step_s, 0.5 * compile_s)`; test_ur5_jax.py:163 `self.assertLess(t_jx, 0.8 * t_np)`; test_simulation.py:955-958 comment `Default plant offline_dt=1e-4 → many steps fit; live ceiling (10) binds.` then `assertAlmostEqual(rt_sim.sim_dt, frame_dt / 10)` before and after `rt_sim.run()`; :918-944 relies on `time.sleep(0.03)` against `frame_dt=0.005`. test.yml:94-96 speed gates use `--factor 10` and `--speed-gate-suffixes solve_s,nlp_s,speedup`.

### tests-ci-tooling#10 — Expose `Animator.resolve_frame` so the graphics harness stops calling a private method

*api · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `minilink/graphical/animation/animator.py:142`, `tests/demo_checks/helpers.py:39`, `tests/fixtures/kinematic_baseline/render.py:35`, `tests/unittest/graphics_contract_helpers.py:17`, `tests/unittest/test_graphics.py`

**What.** `Animator.resolve_frame(self, x, u, t, *, kinematic=False, camera_override=None, overlays=()) -> Frame` as the public name (keep `_resolve_frame` as a one-line alias until the tests move), and use it from the three harness helpers and test_graphics.py.

**Why.** Three separate harness files and the kinematic-baseline fixture renderer reach into `Animator._resolve_frame`, so the flagship-graphics contract (primitive counts, kinematic manifests) depends on a name the library is free to rename; a public frame resolver is also what a student who wants 'one frame as primitives' needs.

**Evidence.** minilink/graphical/animation/animator.py:142 `def _resolve_frame(self, x, u, t, *, kinematic, camera_override=None, overlays=()):`; tests/demo_checks/helpers.py:39 `frame = animator._resolve_frame(x, u, t, kinematic=kinematic)`; tests/fixtures/kinematic_baseline/render.py:35 same; tests/unittest/graphics_contract_helpers.py:17 `return animator._resolve_frame(x, u, t, kinematic=kinematic)`; further direct calls in test_graphics.py (lines 531, 543, 1118, 1155, 1588).

### tests-ci-tooling#11 — Turn the RULES 5.8 underscore check into a repo-wide ratchet with a shrinking allowlist

*test · effort M · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/unittest/test_repo_contract.py:57-65`, `tests/unittest/test_teaching_imports.py:18`, `tests/unittest/teaching_import_allowlist.txt`, `docs/plans/TODO.md:177-236`

**What.** Replace the five-module `NAMED_CLASS_MODULES` tuple with a walk over `minilink/` (excluding `experimental/` and `simulation/realtime/`) plus a `underscore_allowlist.txt` listing today's offenders as `path:name` rows; the test fails when an offender is missing from the allowlist (new one) or listed but gone (stale row), mirroring the `teaching_import_allowlist.txt` mechanism. Each T1–T6 textbook pass then shrinks the file in the same diff.

**Why.** The 5.8 rule ('no `self._x` bookkeeping in the classes the rule names') is enforced for five files, but the wave-D textbook passes touch `analysis/lyapunov.py`, `trajectory_optimization/planner.py`, `core/composition.py` and more; without a ratchet those passes can regress silently and the review has no cheap way to see what remains (RULES 6.12: cheap rules are tests).

**Evidence.** test_repo_contract.py:57-65 `NAMED_CLASS_MODULES = ("minilink/core/system.py", "minilink/core/facades.py", "minilink/core/diagram.py", "minilink/simulation/simulator.py", "minilink/simulation/static_simulator.py")` with comment 'simulation/realtime/ is provisional research lane … out of scope'. test_teaching_imports.py:18 `ALLOWLIST = pathlib.Path(__file__).with_name("teaching_import_allowlist.txt")` (the existing shrinking-allowlist pattern). TODO.md:177-236 lists T1–T6 as file-by-file textbook passes.

### tests-ci-tooling#12 — Run the test suite before publishing to PyPI and put timeouts on the long CI jobs

*tooling · effort S · owner maintainer · rung v0.1 close-out · planned: R1*  
Files: `.github/workflows/publish.yml:8`, `.github/workflows/publish.yml:30-42`, `.github/workflows/publish.yml:50-51`, `.github/workflows/test.yml:12`, `.github/workflows/test.yml:70`, `.github/workflows/nightly.yml:37`

**What.** In publish.yml add a `test` job (ruff + `pytest` on one Python, same install as test.yml) that `publish` `needs:` alongside `build`, or gate on the `test.yml` workflow via `workflow_run`; add `timeout-minutes:` (e.g. 30 for `test`, 45 for `regression`, 90 for nightly) so a hung flagship or notebook does not consume the 6-hour default.

**Why.** A `0.*` tag push builds and uploads whatever is at the tag with only `check_wheel.py` and `twine check`; nothing verifies the tests pass at that commit (tags can be pushed from a branch CI never ran, since test.yml only triggers on three branches). R1 makes this the path students install from.

**Evidence.** publish.yml:8 `tags:` trigger; :35-36 `python tests/demo_checks/check_wheel.py dist/*.whl dist/*.tar.gz` / `python -m twine check dist/*`; :42 `ls "dist/minilink-${tag}-"*.whl`; :50-51 `needs: build` + `if: … startsWith(github.ref, 'refs/tags/')` — no pytest anywhere. test.yml:7 `branches: [main, refactor-v4, dev-alex]`; no `timeout-minutes` in test.yml, nightly.yml or publish.yml (grep returns nothing). nightly.yml:37 `run_all_demos.py --timeout 180 --continue-on-error` over every demo.

### tests-ci-tooling#13 — Trap: showcase_jax.ipynb imports minilink.experimental.c_export, which the wheel does not ship

*trap · effort S · owner maintainer · rung v0.1 close-out · planned: —*  
Files: `examples/tutorial/showcase_jax.ipynb`, `pyproject.toml`, `tests/demo_checks/check_wheel.py`, `tests/unittest/teaching_import_allowlist.txt:50`, `tests/demo_checks/flagship_manifest.json`

**What.** Maintainer decision: either (a) guard cell 33 with a try/except that prints 'experimental tier: run from a repo checkout' and mark the section as such, (b) move the C-export section to `examples/experimental/`, or (c) ship `c_export` in the wheel. Whichever is chosen, add a check to `check_wheel.py` or a notebook-check mode that runs the tutorial notebooks against the built wheel (pip install dist/*.whl in a venv, as the `packaging` job already does for import) so the tutorial that PyPI users follow is what CI executes.

**Why.** The tutorial notebook is the user guide (AGENTS.md doc map) and is smoke-checked only from the repo checkout with `PYTHONPATH=.`; a student on `pip install minilink[jax]` hits `ModuleNotFoundError: minilink.experimental` at cell 33 and no test can see it because the wheel deliberately excludes `minilink/experimental/**`. The flagship manifest also lists two `examples/experimental/c_export/*` scripts as flagships, blurring the tier line.

**Evidence.** showcase_jax.ipynb cell 33 (code): `from minilink.experimental.c_export import export_system_to_c, load_exported_c` after markdown cell 32 '## 9. From the trace to C (experimental)'; pyproject wheel excludes `minilink/experimental/**` (verified: built wheel contains no experimental/ and check_wheel.py asserts that); teaching_import_allowlist.txt:50 `examples/tutorial/showcase_jax.ipynb minilink.experimental.c_export` (the import ratchet already flags it); flagship_manifest.json entries `c_export`, `c_export_proportional` point at `examples/experimental/c_export/`.

### tests-ci-tooling#14 — Stop unit tests from importing examples/projects and benchmarks/systems so the suite runs from the sdist and the Basic tier

*consolidation · effort M · owner agent · rung v0.2 wave D · planned: —*  
Files: `tests/unittest/test_dynamics_catalog.py:47`, `tests/unittest/test_dynamics_catalog.py:535`, `tests/unittest/test_dynamics_catalog.py:684-807`, `tests/unittest/test_jax_planning.py:190`, `tests/unittest/test_mechanical_robotics.py:390`, `tests/unittest/test_mechanical_robotics.py:397`, `pyproject.toml`

**What.** Move the car-ladder / car-profile tests (which test `examples/projects/car_trajopt/vehicles/*`, research lane) into a `tests/projects/` folder collected only when `examples/projects` exists (or via a `projects` marker), and have `test_mechanical_robotics` build its NumPy/JAX pendulum pair from `minilink.catalog` instead of `benchmarks.systems.basic`. Keep `testpaths = tests/unittest` for the teaching-lane contract.

**Why.** The sdist excludes `examples/projects` (verified from the built tarball) so `pytest` on an unpacked sdist fails at collection of test_dynamics_catalog.py and test_jax_planning.py; and a catalog test file that imports project code inverts the layering the constitution draws (library → examples, never back), tying the teaching-lane test module to research-lane vehicles.

**Evidence.** test_dynamics_catalog.py:47 `from examples.projects.car_trajopt.vehicles.extras import (`; :535 `from examples.projects.car_trajopt.vehicles.ladder import (`; :684-807 eight `from examples.projects.car_trajopt.vehicles.car_profile import …`; test_jax_planning.py:190 `from examples.projects.car_trajopt.vehicles.ladder import BicycleAcc, BicycleKin`; test_mechanical_robotics.py:390/:397 `from benchmarks.systems.basic import JaxPendulum, NumpyPendulum`. Built sdist listing (this session) contains tests/ and benchmarks/ but no examples/projects/ (pyproject sdist exclude).

### Bugs reported by the tests-ci-tooling finder

#### bug tests-ci-tooling#0 — (high) CI merge gate never executes any JAX-dependent test: no job installs jax and runs pytest

Files: `.github/workflows/test.yml:31`, `.github/workflows/test.yml:40`, `.github/workflows/test.yml:88-114`, `.github/workflows/nightly.yml:29-45`, `pyproject.toml`

```text
Read .github/workflows/test.yml: the `test` job (line 31) installs `.[dev,rl,diagrams]` — pyproject extras `rl = ["gymnasium"]`, `diagrams = ["graphviz"]`, no jax — then runs `pytest` (line 40); the `regression` job installs `.[dev,jax,...]` (line 88) but runs only run_regression_check.py, run_flagship_demos.py and run_notebook_checks.py (lines 94-114); nightly.yml also never calls pytest. Locally: `python -c "import sys; sys.modules['jax']=None; import pytest; sys.exit(pytest.main(['-q','-rs','--co','tests/unittest/test_mpc.py','tests/unittest/test_dynamics_catalog.py','tests/unittest/test_geometric_control.py','tests/unittest/test_racecar_plant.py','tests/unittest/test_racecar_tires.py','tests/unittest/test_mechanical_robotics.py']))"` prints whole-module SKIPPED lines for each file (49+51+33+14+12+59 = 218 tests), and every `@pytest.mark.jax` test elsewhere (test_catalog_backends.py, test_engine_jax.py, test_ur5_jax.py, test_jax_planning.py, test_control_analysis.py jax classes) is skipped the same way, so a JAX backend regression merges green.
```

#### bug tests-ci-tooling#1 — (medium) Module-level pytest.importorskip("jax") placed mid-file (or atop mostly-NumPy files) skips the NumPy tests too

Files: `tests/unittest/test_dynamics_catalog.py:533`, `tests/unittest/test_mechanical_robotics.py:274`, `tests/unittest/test_geometric_control.py:11`, `tests/unittest/test_racecar_plant.py:10`, `tests/unittest/test_racecar_tires.py:14`

```text
`python -c "import sys; sys.modules['jax']=None; import pytest; sys.exit(pytest.main(['-q','-rs','tests/unittest/test_dynamics_catalog.py','tests/unittest/test_geometric_control.py']))"` → `SKIPPED [1] tests/unittest/test_dynamics_catalog.py:533: could not import 'jax'` and `SKIPPED [1] tests/unittest/test_geometric_control.py:11: the JAX parity checks need jax` with 0 tests run, although 29 `def test_` in test_dynamics_catalog.py precede line 533 (NumPy catalog checks) and only 2 of test_geometric_control.py's 23 tests touch jax (`grep -c 'jnp\.\|jax\.'` = 3). Same for test_mechanical_robotics.py (22 tests before line 274). This is the state of the CI `test` matrix and of a Basic-tier install.
```

#### bug tests-ci-tooling#2 — (medium) Flagship manifest omits graphviz for the two MPC demos, so `pytest` fails without the diagrams extra

Files: `tests/demo_checks/flagship_manifest.json`, `tests/demo_checks/run_flagship_demos.py:84-92`, `tests/unittest/test_demo_check_runners.py:78-84`, `minilink/graphical/diagrams/hybrid_dot.py:19-24`

```text
In an environment with `pip install -e .[dev]` but no `graphviz` Python package (`python -c 'import graphviz'` → ModuleNotFoundError): `MPLBACKEND=Agg PYTHONPATH=. python tests/demo_checks/run_flagship_demos.py` prints `mpc_integrator_numpy fail (exit 1: ... hybrid_dot.py, line 22, in export_hybrid_graphviz raise ImportError( ImportError: Graphviz hybrid export requires the graphviz Python package.)` and the same for `mpc_car_minimal`; `8 passed, 2 failed, 2 skipped`, exit 1. Consequently `pytest tests/unittest/test_demo_check_runners.py::TestDemoCheckRunners::test_flagship_demos_exit_zero` fails (observed as the single F in this session's full run). Both manifest entries carry `'requires': []` although examples/demos/mpc/mpc_integrator_numpy.py:64 and mpc_car_minimal.py:55 call `hybrid.plot_diagram()`; the runner only skips on `requires`.
```

#### bug tests-ci-tooling#3 — (low) run_notebook_checks.py assigns the same id to two different notebooks, so overrides and --notebook filters are ambiguous

Files: `tests/demo_checks/run_notebook_checks.py:48-57`, `tests/demo_checks/notebook_overrides.json`

```text
`PYTHONPATH=. python -c "import sys, pathlib; sys.path.insert(0,'tests/demo_checks'); import run_notebook_checks as r; from collections import defaultdict; d=defaultdict(list); [d[r._notebook_id(p.as_posix())].append(p.as_posix()) for p in pathlib.Path('examples').rglob('*.ipynb')]; print({k:v for k,v in d.items() if len(v)>1})"` → `teaching_drone_ppo: [examples/teaching/courses/udes_gro860/drone_ppo.ipynb, examples/teaching/topics/reinforcement_learning/drone_ppo.ipynb]` and `teaching_pendulum_value_iteration_vs_lqr_vs_rl: [courses/udes_gro860/…, topics/reinforcement_learning/…]`. `_notebook_id` uses only the stem for `/teaching/` paths (line 55) and still special-cases a non-existent `/learn/intro/` path (line 52); the `--notebook` help cites `intro_00_core`, which matches nothing.
```

#### bug tests-ci-tooling#4 — (low) tests/run regression launcher claims CI parity but uses --factor 6 where CI uses --factor 10

Files: `tests/run/_common.py:57-66`, `tests/run/run_regression_gates.py:22`, `.github/workflows/test.yml:94-96`

```text
`sed -n 57,66p tests/run/_common.py` shows `"--factor", "6"`; `grep -n factor .github/workflows/test.yml AGENTS.md` shows `--factor 10`; `sed -n 22p tests/run/run_regression_gates.py` reads `True → same flags as GitHub CI regression job`. Running the IDE launcher can fail a speed gate that CI would pass (or vice versa) with no way to tell which is authoritative.
```

#### bug tests-ci-tooling#5 — (low) tests/README.md and the flagship-graphics manifest carry stale facts (file count, removed folders, dead demo_id links)

Files: `tests/README.md:135`, `tests/README.md:180`, `tests/fixtures/flagship_graphics/manifest.json`

```text
`ls tests/unittest/test_*.py | wc -l` → 45 while tests/README.md:135 says `Domain modules (22 files after contract-test consolidation)`; :180 still explains that `tests/manual/ and tests/bugs/ are removed` (neither exists; the note is history, not policy). `python -c "import json; a={e['id'] for e in json.load(open('tests/demo_checks/flagship_manifest.json'))}; print([e['demo_id'] for e in json.load(open('tests/fixtures/flagship_graphics/manifest.json')) if e.get('demo_id') not in a])"` → `['cascade_path_tracking', 'mpc_minimal']` (no such flagship ids; the field is read by nothing).
```


## docs-governance

### docs-governance#0 — Generate docs/api from the teaching-surface registry

*tooling · effort M · owner agent · rung v0.2 wave D · planned: —*  
Files: `docs/api/analysis.rst:1-13`, `docs/api/dynamics.rst:1-22`, `docs/api/planning.rst:1-30 and 32-52`, `docs/api/core.rst:1-20`, `docs/api/simulation.rst:1-27`, `docs/conf.py:27-28`, `AGENTS.md:37`, `DESIGN.md:113-115`, `minilink/__init__.py (__all__, 150 names)`

**What.** Replace the hand-written `automodule` lists with pages generated from `minilink.__all__` (autosummary over each name's defining module, grouped by band), or at minimum add a check to `tests/unittest/test_repo_contract.py` asserting that every root-prelude name's `__module__` appears in some `docs/api/*.rst` and that no research-lane module does. While touching the docs contract, add the RULES 6.9 check that every GIF under `docs/_static` is under 1 MB.

**Why.** The site claims to be the teaching-lane API reference, yet a student looking up `bode`, `margins`, `ReinforcementLearningPlanner`, `MonteCarloEvaluator`, `StateSpaceGrid`, `LookupTableController` or sixteen catalog plants finds nothing, while seven provisional `planning.spatial` modules that A3 retires are documented. A1 and A3 move files; a generated index follows the moves for free instead of drifting again.

**Evidence.** A script over `minilink.__all__` (150 names): 30 defining modules, 64 names, have no `automodule` entry — `analysis.frequency` (bode, margins, nyquist, pzmap, root_locus, transfer_function, plot_*), `analysis.time_response`, `analysis.derivatives` (jacobian), `analysis.discretize`, `core.distributions` (Gaussian, Uniform), `core.feedback` (Controller), `control.neural`, `control.geometric`, `blocks.step` (ZOHHold), `planning.evaluation` (Evaluation, MonteCarloEvaluator), `policy_synthesis.discretizer` / `lookup_policy`, `reinforcement_learning.planner` / `tabular`, `simulation.static_simulator`, and 16 `dynamics/catalog` modules (docs/api/dynamics.rst lists 4 of 20). docs/api/planning.rst:32-52 documents `planning.spatial.{scene,collision,state_fields,track,paths,shaping,workspace_fields}`, provisional per ROADMAP.md:31. AGENTS.md:37: "Sphinx autodoc of the teaching-lane API".

### docs-governance#1 — Make the RULES 5.8 check a ratchet over every System-family class

*test · effort S · owner agent · rung v0.2 wave D · planned: T2, T3, T5, T6 (rename pass)*  
Files: `tests/unittest/test_repo_contract.py:55-61 and 136-152`, `minilink/control/mpc/controller.py (MPCStatelessController, MPCStatefulController, MPCBroadcastController)`, `minilink/dynamics/catalog/vehicles/dynamic_bicycle.py (_u_in, _contact_fields)`, `minilink/planning/search/rrt.py (13 _methods)`, `minilink/planning/search/rrt_star.py (8)`, `minilink/planning/trajectory_optimization/planner.py (11)`, `RULES.md:272-275`

**What.** Replace `NAMED_CLASS_MODULES` with an AST walk of every class under `minilink/` (outside `experimental/` and `core/compile/`, which RULES 5.16 exempts) whose bases end in `System`, `Controller`, `Simulator`, `Facades` or `Planner`, plus a committed allowlist of today's `module:Class._name` offenders; the test fails when a new underscore method appears and when an allowlisted entry is gone but still listed, so the list only shrinks. Same shape as the D1.2 flatness ratchet.

**Why.** 5.8 is the rule the maintainer chose to enforce (2026-09-12 audit, decision 3), and every T-step renames some of these; without a ratchet the count grows back between passes — the 2026-09-05 census found 43 on System subclasses, the count today including planners is 80. A test that checks five files lets the other twelve classes drift silently.

**Evidence.** `test_repo_contract.py:55-61` checks five modules only. An AST count over the rest of the library: 80 leading-underscore methods on 13 System-family / planner classes — `MPCStatelessController` 4, `MPCStatefulController` 5, `MPCBroadcastController` 3 (controller.py), `DynamicBicycle` 2 (`_u_in`, `_contact_fields`), `RRTPlanner` 13, `RRTStarPlanner` 8, `TrajectoryOptimizationPlanner` 11, and 34 on the NumPy/JAX diagram evaluators (exempt under 5.16). The 2026-09-22 review §4.4 lists the same names under T2/T3/T5/T6; TODO.md:208-209 "Rename pass, the rest" is prose only.

### docs-governance#2 — Test that ROADMAP §5 step ids and TODO rows agree

*test · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `ROADMAP.md:69-70, 197-208, 271-289`, `docs/plans/TODO.md:1-8, 36-48, 170-171, 226-238`, `tests/unittest/test_repo_contract.py`

**What.** A test that collects every bold step id (`R\d+`, `S\d+`, `A\d`, `P\d+`, `C\d`, `T\d`, `D\d(\.\d)?`, `V\d`) in ROADMAP §5 and every `**ID**` row in TODO.md and asserts the two sets are equal (wave labels B1–B5 excluded by a short list), and that each id sits under the same rung heading in both files.

**Why.** ROADMAP says the steps "are the rows of TODO.md"; today R3 has no row and one bug is filed under D3 in ROADMAP and under T1 in TODO. The 2026-09-12 audit's lesson — prose drifts within days — applies to the workboard itself, and this is a grep-cheap rule (RULES 6.12).

**Evidence.** ROADMAP.md:202-203 `R3` (ruff green on every push) has no TODO.md row — TODO §1 (TODO.md:36-48) lists R1, R2, S49, S54 only. ROADMAP.md:288 lists "`StepDiagramSystem.step` writes in place under JAX" under D3; TODO.md:170-171 files it under T1 and TODO D3 (TODO.md:226-238) does not mention it. ROADMAP.md:69-70: "Step ids (`S29`, `P3`, `T2`, …) are the rows of docs/plans/TODO.md".

### docs-governance#3 — Give plan-doc steps ids that cannot collide with the workboard's

*docs · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `docs/plans/cbf-safety-filter.md:79-83`, `docs/plans/optimizer-parametric-wiring.md:132, 151, 165, 173, 180`, `docs/plans/articulated-mechanism.md:347, 354, 361, 367`, `docs/plans/pyro-port-remaining.md:285-296`, `docs/plans/gro501-classical-control.md:19-34`, `docs/plans/TODO.md:100-115`, `docs/plans/README.md:1-12`

**What.** Rename the local steps of the unscheduled plan docs with a doc prefix (`CBF-1…5`, `OPW-1…5`, `AM-1…4`) and the pyro backlog's `P2/P3/P4` priority labels to `prio 2/3/4`; add one line to docs/plans/README.md: step ids are global and assigned in TODO.md, and a plan doc numbers its own steps under its parent id (as fields.md does with 4.1–4.8) or with a prefix. Fold a uniqueness check into the ROADMAP↔TODO parity test if that lands.

**Why.** `C1` today means both "Pyro parity" (ROADMAP §5.2, TODO §4) and "BicubicGridSDF" (cbf plan); `P1` means both "PI / PD" (landed) and "shared optimizer backend factory" (unscheduled). A reviewer citing an id, or an agent grepping for one, lands on the wrong document.

**Evidence.** cbf-safety-filter.md:79-83 `C1`–`C5` vs TODO.md:100-115 `C1`–`C5` (Pyro parity, GMC714, Blocks, Identification, RL follow-ups); optimizer-parametric-wiring.md:132-180 `P1`–`P5` and articulated-mechanism.md:347-370 `P1`–`P4` vs gro501-classical-control.md:19-34 `P1`–`P11` (the ids ROADMAP §5.2 wave B uses); pyro-port-remaining.md:287-296 "P2 … P3 … P4" as priorities. ROADMAP.md:69-70 declares one id space.

### docs-governance#4 — Align RULES 3.3's teaching-surface definition with ROADMAP §2

*docs · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `RULES.md:91-96`, `ROADMAP.md:28-35`, `DESIGN.md:59-67`, `tests/unittest/test_public_imports.py:36-45`, `docs/reviews/2026-09-12-governance-stack-audit.md:90-93`

**What.** Rewrite 3.3's two bullets so the placement shorthand names the set the tests check: teaching surface = the root prelude and band facades (ROADMAP §2) plus `examples/tutorial/`, `examples/teaching/`, `examples/demos/`; research lane = the provisional bands (hybrid, MPC, realtime, spatial), `experimental/`, `examples/projects/`, `examples/experimental/`.

**Why.** An agent reading RULES 3.3 alone concludes `control/mpc` and `simulation/realtime` carry a "strict public contract, high stability"; ROADMAP §2 and the suite say the opposite (`ModelPredictiveController` and `HybridDiagram` are asserted absent from the prelude). The 2026-09-12 audit accepted four copies of the lane contract as paraphrases; this copy contradicts.

**Evidence.** RULES.md:93-94: "**Teaching surface** (`minilink/`, `examples/tutorial/`, `examples/teaching/`, `examples/demos/`): Strict public contract, high stability"; RULES.md:95-96 lists only `examples/projects/`, `examples/experimental/` as research lane. ROADMAP.md:30: teaching surface = "root prelude … and the band facades"; ROADMAP.md:31: research lane = "provisional bands (hybrid, MPC, realtime, spatial), the `minilink/experimental/` tier, …". test_public_imports.py:44-45 asserts `ModelPredictiveController` and `HybridDiagram` are not on the prelude.

### docs-governance#5 — Keep the CI commands in one place and name every job

*consolidation · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `AGENTS.md:82-124`, `tests/README.md:48-71`, `benchmarks/README.md:104-121`, `.github/workflows/test.yml:12, 42, 70`

**What.** AGENTS.md keeps its proportionate table as *when* rows and points at tests/README.md's "Agent" table for the *commands*; delete the second "Big review pass" block (AGENTS.md:118-124), which repeats the table row; benchmarks/README.md keeps only the flag semantics. AGENTS.md:86 names the three CI jobs (`test`, `packaging`, `regression`).

**Why.** The regression-gate command is written out four times (twice in AGENTS alone), so the next flag change (`--speed-gate-suffixes`, `--factor`) is edited in four files or drifts — RULES 7.1's "text edited twice when code changes". AGENTS' "runs exactly" sentence is already wrong by one job.

**Evidence.** `python benchmarks/run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes solve_s,nlp_s,speedup` at AGENTS.md:107, tests/README.md:56, benchmarks/README.md:107-109; `--suite all` again at AGENTS.md:122 and benchmarks/README.md:121. AGENTS.md:86: "CI … runs exactly: `ruff check .`, `ruff format --check .`, `pytest` …, then the **`regression`** job" — test.yml:42 has a `packaging` job (tests/README.md:70 lists it). AGENTS.md:110 already says "full command … tests/README.md (entry points)": the pointer and the copies coexist.

### docs-governance#6 — Delete DESIGN §8's Package roles table (a stale copy of §3)

*consolidation · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `DESIGN.md:1194-1206`, `DESIGN.md:80-110`, `AGENTS.md:41`, `minilink/control/ (neural.py, geometric.py, output.py, state.py)`

**What.** Remove the "Package roles" table from §8 and keep only the "Main chains" block; §3 is the package map. If a one-screen summary is wanted, make it a pointer to §3.

**Why.** Two owners of one table, and the §8 copy is already stale: its `analysis` row omits the frequency family, Lyapunov and `discretize`; its `control` row omits `neural`, `geometric`, `output` and `state`. AGENTS asks to keep DESIGN's call-chains section minimal.

**Evidence.** DESIGN.md:1201 "`analysis` | `linearize`, `structural`, `equilibria`, `modal` …" vs DESIGN.md:105 (bode / pzmap / nyquist / margins / root_locus / step_response / region_of_attraction / discretize) and `ls minilink/analysis/` (11 modules). DESIGN.md:1200 `control` row vs DESIGN.md:94 and `ls minilink/control/`. AGENTS.md:41: "Keep DESIGN.md call chains minimal."

### docs-governance#7 — Move DESIGN's inline TODOs to the workboard and fix its retired pointers

*docs · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `DESIGN.md:438-444`, `DESIGN.md:619`, `DESIGN.md:779-780`, `DESIGN.md:788`, `DESIGN.md:790`, `DESIGN.md:824-831`, `minilink/estimation/__init__.py:8`, `minilink/identification/__init__.py:10`, `docs/plans/TODO.md:82-88, 326-342`, `docs/plans/README.md:3`

**What.** (1) Delete DESIGN.md:790 "TODO: Add a hard warning" — the Mitigation paragraph seven lines later says the `UserWarning` landed. (2) Replace DESIGN.md:439-440 "TODO: Prioritize threading p" with a pointer to the existing Later row "Scene params / J(z, p) bind". (3) Add a `SimulationOptions` Later row or drop the "Planned:" clause at :779-780. (4) Replace ":788 ROADMAP teaching-release hardening" with the D3 row (`HybridSimulator` conventions). (5) Drop the `scratch/confirm_smc_solver_bug.py` line or move the probe under `benchmarks/`. (6) Point the two package docstrings at ROADMAP §5 (P4, C4) instead of "teaching-release priorities". (7) DESIGN.md:444 says A5 tracks `Shape` params; A5's text names sets, distributions and fields only — add shapes to A5.

**Why.** plans/README says three backlog homes, do not invent a fourth; DESIGN is carrying five work items, two of them done or already rows. Pointers to sections retired on 2026-09-22 and to a file that does not exist are the first things a new contributor follows.

**Evidence.** DESIGN.md:790 vs :827-831 ("Mitigation (landed): … `UserWarning` on every discontinuous solve"); DESIGN.md:439-440 vs TODO.md:326-327; DESIGN.md:779-780 "Planned: `SimulationOptions` (TODO.md Later)" — `grep -n SimulationOptions docs/plans/TODO.md` is empty; DESIGN.md:788 "teaching-release hardening" — no such ROADMAP heading (ROADMAP.md:183-321 is the §5 ladder); DESIGN.md:824 — `ls scratch` fails; estimation/__init__.py:8 and identification/__init__.py:10 "see ROADMAP.md teaching-release priorities"; DESIGN.md:441-444 vs TODO.md:82-88 (A5).

### docs-governance#8 — Scrub retired phase numbers and line pointers from the plan docs; settle the optimizer-wiring rung

*docs · effort S · owner agent · rung v0.2 wave D · planned: —*  
Files: `docs/plans/fields.md:41, 83`, `docs/plans/cost-params.md:19`, `docs/plans/gro501-classical-control.md:229`, `docs/plans/optimizer-parametric-wiring.md:3-8`, `docs/plans/README.md:32-36`, `docs/plans/TODO.md:196-197, 229-230, 326`

**What.** fields.md's Verification names its baseline by content (RoA level and `verify` report, DP table, fitted weights), not "phase 2 / phase 4"; `tabular.py:262` becomes "the `TabularLearningPlanner` docstring"; gro501's "1 228 lines" becomes "~400 lines of facades.py". optimizer-parametric-wiring.md is cited by T5 (duplicated optimizer-method tuple), D3 (parametric evaluator placement) and a Later row (`SolverFactory`): pick one — schedule it under D3 and delete the Later row and the "unscheduled" status, or keep it Later and drop the D3 citation — and make plans/README's row match.

**Why.** A plan doc a reader opens because a rung names it must not send them to a phase log that no longer exists. A doc that is "unscheduled" in its own header and in the plans index but scheduled in two wave-D rows will be worked twice or never.

**Evidence.** fields.md:83 "The list of phase 2 plus … the phase 4 baseline"; ROADMAP.md:185-187 "One ladder replaces the phase log kept here until 2026-09-22"; fields.md:41 "`tabular.py:262` docstring"; cost-params.md:19 "the ROADMAP review queue"; gro501-classical-control.md:229 "`facades.py`'s 1 228 lines" (`wc -l` → 1382); optimizer-parametric-wiring.md:3 "unscheduled — a Later idea in TODO.md §7"; plans/README.md:34 "Research lane, unscheduled"; TODO.md:229-230 (D3) and :196-197 (T5) cite it; TODO.md:326 Later "`SolverFactory`".

### docs-governance#9 — Name the estimation API once, in the P4 plan

*api · effort S · owner maintainer · rung v0.2 wave B · planned: P4*  
Files: `DESIGN.md:141-144`, `minilink/estimation/__init__.py:8-14`, `ROADMAP.md:167, 244-246`, `docs/plans/TODO.md:121-127`, `docs/plans/gro501-classical-control.md:170-201`

**What.** Decide the factory and block names in gro501-classical-control.md P4 — `LuenbergerObserver(A, B, C, L)`, `luenberger(A, B, C, poles)`, `kalman(A, B, C, Q, R)` returning the same block, mirroring `lqr` — and make DESIGN's dependency-law example and the package docstring cite the plan instead of naming an API, until P4 lands and DESIGN §4 gets its real bullet.

**Why.** Four documents give three different names for a factory that does not exist yet; whichever agent lifts the hold implements the one it read first and the others become a rename. The plan's shape is the settled one (mirrors `lqr`); DESIGN's `kalman_design(A, C, Q, R)` also drops `B`, which the observer needs.

**Evidence.** DESIGN.md:143 "`estimation.kalman_design(A, C, Q, R) -> KalmanFilter`"; estimation/__init__.py:11 "`kalman.py` — Kalman filter (+ `kalman_design(A, C, Q, R)` factory)"; ROADMAP.md:167 "`LuenbergerObserver` and steady-state `KalmanFilter` closing the loop"; ROADMAP.md:244-245, TODO.md:121-123 and gro501-classical-control.md:173-179 "`LuenbergerObserver(A, B, C, L)` … `luenberger(A, B, C, poles)` … `kalman(A, B, C, Q, R)` … returning the same block".

### docs-governance#10 — Add the DESIGN research-lane trim to D2 as a row with its three moves

*consolidation · effort M · owner maintainer · rung v0.2 wave D · planned: D2 (inventory item 15 is named in the review, but D2 has no row)*  
Files: `docs/plans/TODO.md:239-252`, `docs/reviews/2026-09-22-consolidation-review.md:70-73`, `docs/reviews/2026-09-05-consolidation-inventory.md:27`, `DESIGN.md:295-329 (realtime)`, `DESIGN.md:330-365 (dual-rate MPC)`, `DESIGN.md:1092-1123 (spatial pipeline)`, `minilink/simulation/realtime/__init__.py`, `minilink/control/mpc/__init__.py`

**What.** A D2 row: "DESIGN research-lane mechanism (~250 lines): the realtime clocks / `sync` bullet → `simulation/realtime/__init__.py` docstring; the dual-rate MPC packaging paragraph (option A/B) → `control/mpc/__init__.py` (T2 trims that docstring — land both together) or the fidelity-maps plan; the spatial-pipeline paragraphs → geometry-module.md until A3's DESIGN bullet replaces them; DESIGN keeps one-line pointers." Maintainer picks the split; agent executes.

**Why.** The 2026-09-22 review names DESIGN as "the next drift risk" and says it "stays a D2 pick", but D2 has no such row, so the pick can never be made from the workboard. The realtime bullet alone documents `frame_dt` / `sim_dt` / `sync` semantics of a TRL-2 band inside the frozen contract.

**Evidence.** 2026-09-22-consolidation-review.md:70-73 "DESIGN.md is the next drift risk … it stays a D2 pick because it is the maintainer's document"; TODO.md:239-252 D2 rows: `Source.show_signal`, dead modules, benchmark shims, MPC debug figure, `HybridDiagram` facades, plotting homes — no DESIGN row; 2026-09-05-consolidation-inventory.md:27 item 15 "~250 lines"; DESIGN.md:295-329, :330-365, :1092-1123; ROADMAP.md:91 realtime TRL 2.

### docs-governance#11 — Re-audit pyro-port-remaining against the code before the migration guide

*docs · effort M · owner agent · rung v0.2 wave C · planned: C1*  
Files: `docs/plans/pyro-port-remaining.md:34, 43-45, 285-296`, `minilink/control/lqr.py:158`, `minilink/control/state.py:126`, `minilink/analysis/frequency.py:217`, `DESIGN.md:577`, `benchmarks/run_pyro_minilink_parity.py`

**What.** Walk the TODO / Partial library rows and the §6 priority backlog against the AST (as the 2026-09-05 review did for the Done rows) and mark landed: `ss2tf` → `transfer_function` (2026-09-07), `TrajectoryLQRController` → `trajectory_lqr` / `TrajectoryFeedbackController`, PID "dedicated wrapper pending" → `PI` / `PD` (P1); then re-status the `trajectory_stabilization/` and `transfer_functions/` demo rows blocked on them. Add a test that every "Minilink symbol" named in the table imports, so the table cannot lag the code again.

**Why.** C1 builds the README migration guide from this table; a table that marks shipped features as TODO under-reports parity (the 2026-09-05 review's D-1 warned a wrong parity audit is worse than none) and sends an agent to re-implement `trajectory_lqr`.

**Evidence.** pyro-port-remaining.md:44 "TrajectoryLQRController | `minilink/control/lqr.py` | — | **TODO** | Trajectory stabilization demos" vs control/lqr.py:158 `def trajectory_lqr` and control/state.py:126 `class TrajectoryFeedbackController` (DESIGN.md:577 documents it). :34 "ss2tf() | `minilink/analysis/` | — | **TODO** | Frequency backlog" vs analysis/frequency.py:217 `def transfer_function` (ROADMAP.md:136-138: frequency tools landed 2026-09-07). :288 and :293 repeat both in the priority backlog; :43 "Dedicated PID wrapper pending" vs P1 landed (gro501-classical-control.md:53-98).

### docs-governance#12 — Drop tests/README's stale module census and the duplicated smoke-policy lines

*docs · effort S · owner agent · rung v0.1 close-out · planned: —*  
Files: `tests/README.md:135-141`, `examples/README.md:94`, `examples/README.md:182-190`, `RULES.md:103-107`

**What.** Delete the "Domain modules (22 files …)" paragraph (a directory listing, stale by 23 files) and keep the Philosophy sentence. In examples/README keep the CI-smoke rule once (the "CI smoke" section) and make the topics-table note a pointer to it.

**Why.** RULES 3.6 says a README must not merely list files; a listing that is wrong by half tells a new contributor the map is unmaintained. Two statements of the "long notebooks are skipped" rule will disagree the first time `notebook_overrides.json` changes.

**Evidence.** tests/README.md:135 "**Domain modules** (22 files after contract-test consolidation): `test_core`, …" — `ls tests/unittest/test_*.py` → 45 files (`test_racecar_*`, `test_rl_*`, `test_lqr_planner`, `test_planning_stochastic`, `test_feedback_composition`, … absent from the list). examples/README.md:94 "Long notebooks (UR5 EoM, DP grids, PPO training) are skipped in fast CI smoke checks" and :184-185 "except long notebooks with `"smoke": false` — UR5 EoM, DP grids, PPO".

### docs-governance#13 — Name who owns ROADMAP §5 and §6 in the AGENTS lanes

*docs · effort S · owner maintainer · rung v0.1 close-out · planned: —*  
Files: `AGENTS.md:49-62`, `ROADMAP.md:183-190, 323-327`, `docs/plans/TODO.md:11-14`, `docs/reviews/2026-09-22-consolidation-review.md:131-137, 403-404`

**What.** One clause in the "Ask first" / "Agent-managed" lists: §5 rung membership and step wording are agent-managed like TODO.md (the [ask] flags mark the maintainer's steps; moving a step between rungs is a report, dropping one is an ask); §6 is maintainer-owned (adding an open decision is fine, closing one is not).

**Why.** AGENTS names §1, §2, §4 (maintainer) and §3 (agent) and is silent on the two sections that changed most on 2026-09-22; an agent cannot tell whether reordering a wave or recording a decision as settled needs a yes, and the last pass asked confirmation only for §1.

**Evidence.** AGENTS.md:49-50 "ROADMAP §1, §2 and §4" (ask first); AGENTS.md:59 "the TRL ledger (ROADMAP §3)" (agent-managed); ROADMAP.md:326-327 "Each open item needs the maintainer"; 2026-09-22-consolidation-review.md:131-137 rewrote §5 and §6 and :403-404 requested confirmation for §1 only.

### Bugs reported by the docs-governance finder

#### bug docs-governance#0 — (low) DESIGN §6 documents `Distribution.mean()` as a method; it is an attribute and the call raises

Files: `DESIGN.md:925`, `DESIGN.md:620`, `CONSTITUTION.md:58`, `minilink/core/distributions.py:55, 82, 99, 130`

```text
import numpy as np
from minilink import Gaussian
g = Gaussian(np.zeros(2), 1.0)
g.mean      # array([0., 0.])
g.mean()    # TypeError: 'numpy.ndarray' object is not callable
# DESIGN.md:925 reads "a duck type — `dim`, `mean()`, `sample(key)`"; DESIGN.md:620 and CONSTITUTION.md:58 say `mean` (an array).
```

#### bug docs-governance#1 — (low) DESIGN §5 points at a diagnostics script that does not exist

Files: `DESIGN.md:824-825`

```text
grep -n 'scratch/confirm_smc_solver_bug.py' DESIGN.md   # -> line 824
ls scratch                                            # -> No such file or directory
git ls-files | grep confirm_smc                       # -> empty
```

#### bug docs-governance#2 — (low) RULES 7.6 says there is no link checker in CI; `test_repo_contract.py` checks links in the CI `test` job

Files: `RULES.md:456-458`, `tests/unittest/test_repo_contract.py:36-50, 96-128`, `.github/workflows/test.yml:12-40`

```text
sed -n '456,458p' RULES.md   # "renames and section moves break them silently (no link checker in CI)"
sed -n '36,50p' tests/unittest/test_repo_contract.py   # LINKED_DOCS covers 11 documents plus docs/plans/
pytest tests/unittest/test_repo_contract.py -k links    # runs in the `test` job (ruff + pytest)
# The rule's justification is false; an agent may skip anchor hygiene believing nothing checks it.
```

#### bug docs-governance#3 — (low) tests/README.md claims 22 domain test modules; 45 exist

Files: `tests/README.md:135-141`

```text
sed -n '135,141p' tests/README.md          # "Domain modules (22 files after contract-test consolidation)"
ls tests/unittest/test_*.py | wc -l        # -> 45
```

#### bug docs-governance#4 — (low) AGENTS.md says CI runs "exactly" three things; the workflow also has a `packaging` job

Files: `AGENTS.md:86`, `.github/workflows/test.yml:42-68`, `tests/README.md:70`

```text
sed -n '86p' AGENTS.md                                 # "runs exactly: ruff check ., ruff format --check ., pytest …, then the regression job"
grep -n '^  [a-z_]*:$' .github/workflows/test.yml      # -> test:, packaging:, regression:
```

#### bug docs-governance#5 — (medium) pyro-port-remaining marks landed features as TODO (`ss2tf`, `TrajectoryLQRController`, dedicated PI/PD)

Files: `docs/plans/pyro-port-remaining.md:34, 43, 44, 288, 293`, `minilink/analysis/frequency.py:217`, `minilink/control/lqr.py:158`, `minilink/control/state.py:126`, `minilink/control/siso.py (PI, PD)`

```text
sed -n '34p;43,44p;288p;293p' docs/plans/pyro-port-remaining.md   # ss2tf TODO, PID wrapper pending, TrajectoryLQRController TODO (twice)
python -c "from minilink.analysis import transfer_function; from minilink.control import trajectory_lqr, TrajectoryFeedbackController, PI, PD; print('all landed')"
# The table drives the C1 parity criterion (ROADMAP.md:97) and the README migration guide.
```

#### bug docs-governance#6 — (low) `estimation/` and `identification/` package docstrings point at a ROADMAP section that no longer exists and pre-name an API the plan contradicts

Files: `minilink/estimation/__init__.py:8, 11`, `minilink/identification/__init__.py:10`, `ROADMAP.md:183-190`, `docs/plans/gro501-classical-control.md:173-179`

```text
python -c "import minilink.estimation as e, minilink.identification as i; print(e.__doc__.splitlines()[7]); print(i.__doc__.splitlines()[9])"
# -> "Planned modules (see ROADMAP.md teaching-release priorities):" in both
grep -n 'teaching-release' ROADMAP.md      # -> empty (the phase log was replaced by the §5 ladder on 2026-09-22)
grep -n 'kalman_design' minilink/estimation/__init__.py docs/plans/gro501-classical-control.md   # docstring names kalman_design(A, C, Q, R); the plan names kalman(A, B, C, Q, R)
```

