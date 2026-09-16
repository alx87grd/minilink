# Foundations review: the core mathematical objects

**Date:** 2026-09-15
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Branch `dev-alex` @ `62ed82e`. A review of the foundation Minilink keeps building
on — the textbook objects the code is organised around — with a specific look at `Set` and
`Distribution`, the governance stack, and the tools coming next. Evidence base: CONSTITUTION,
RULES, AGENTS, DESIGN, ROADMAP, TODO, the three 2026-09-12 governance reviews, the 2026-09-05
architecture review, `core/{system,signals,sets,costs,trajectory,geometry}.py`,
`planning/{problems,distributions,results,evaluation}.py`, every library consumer of a set or
a distribution, and the plans that will consume them next (cost-params, cbf-safety-filter,
standard-planning-problems, naming).
**Applied:** Phase 1 landed the same day on the maintainer's go: the wording of Part B is in
CONSTITUTION §1.4, §2 and §4; RULES 2.3, 2.5, 4.3, 5.1, 5.2, 5.4, 5.17, 5.21, 6.1 and
6.10–6.12; AGENTS (the ask-first list); DESIGN §4; ROADMAP §6; `examples/README.md`. DESIGN
bullets that describe code not yet written (distributions in `core/`, `.box`, `bounding_box`,
fields) land with their phases. Part B is the decision record. The code plan (Phases 2–6) is
docs/plans/core-math-objects.md.

---

## Context

The maintainer wants Minilink organised around the textbook's own mathematical objects
(system, trajectory, set, distribution, cost, planning problem, planning solution), on the
thesis that forcing the right abstraction leads to good software design. Questions: is the
foundation solid enough to keep building on; are `Set` and `Distribution` used everywhere they
should be and written to the textbook standard; does the governance stack say what the code
should do; are the next tools (Kalman / estimation, CBF safety filter, robust and stochastic
problems, identification) served by what exists. The maintainer agreed with the first round of
governance amendments and asked for the rules to be reviewed first, as Phase 1. Added the same
day: a style rule for demos and teaching notebooks in the maintainer's own words — *less is
better; simplicity is the ultimate sophistication* — demos rely on the objects' native prints
and plots (upgrade the natives when it helps), a custom plot or side analysis lives in its own
cell so the API lines stay readable, and demos are flat (no helper functions or classes
without a good reason).

---

# Part A — The review

## A.1 Verdict

**The foundation is solid. Keep building on it.** The noun vocabulary is complete for
everything on the roadmap through v1.0: `System`, `Trajectory`, `Set`, `Distribution`,
`CostFunction`, `PlanningProblem` / `StochasticPlanningProblem`, `PlanningSolution`,
`Evaluation`, `MathematicalProgram`, `Shape`. Nothing found here argues for a new core type or
for re-cutting an existing one.

The maintainer's suspicion is confirmed with numbers: **two of the nouns are declared but not
spoken.** `Set` and `Distribution` exist, are exported and tested, yet the tools keep
re-deriving them from raw bound arrays and `isinstance` probes:

| Evidence | Count |
| --- | --- |
| `isinstance(..., <Set type>)` probes in tools (planning, analysis, RL env) | 21 in 9 files |
| Raw `state.lower_bound` / `upper_bound` reads in *tools* (not plant constructors) | 12 files |
| Library sites that rebuild a box the problem already holds | 6 |
| Objects that are a set or a distribution without the type | 7 (§A.5) |
| Dedicated tests of `core/sets.py` / `planning/distributions.py` | 3 / 2 functions |

One re-derivation is a live semantic gap (F1): reinforcement learning and the JAX Monte Carlo
score test membership in the *bounding box* of `X`, while the shared scoring contract tests `X`
itself. No shipped demo hits it yet; the first student who learns a policy around the
obstacles of the RRT chapter will. The rest is textbook-style debt in two short files (§A.6)
and one placement decision the dependency law forces (F2): distributions must live in `core/`
before `estimation/` and `blocks/` can use them.

**The governance gap behind the findings:** the constitution never names the mathematical
objects it wants the code to speak. §4.6 mentions "System, Trajectory, PlanningProblem, sets,
costs" inside a negative rule (no DTO layers). Every finding below is a consequence of that
sentence's absence, so Phase 1 writes it.

## A.2 The nouns today

| Object | Home | Role in the textbook | State |
| --- | --- | --- | --- |
| `System`, `DynamicSystem`, `StepSystem`, `DiagramSystem` | `core/system.py`, `diagram.py` | the model `f, h, tf` and its ports | solid (TRL 7) |
| ports (`VectorSignal`, `InputPort`, `OutputPort`) | `core/signals.py` | labels, units, bounds, nominal value | solid; bounds are a `BoxSet` in raw form |
| `Trajectory` | `core/trajectory.py` | sampled `(t, x, u)` | solid; "exactly right" (2026-09-05 review) |
| `Set`, `InputSet` + `Box`/`Singleton`/`Ball`/`Callable`/`Intersection` | `core/sets.py` (355 lines, last real edit 2026-06-22) | `z ∈ Z(t)`, `u ∈ U(x, t)`, `margin ≥ 0` | correct, traceable, under-used, style debt |
| `FieldSet` | `planning/spatial/state_fields.py` | free space / corridor as a set | the one `Set` subclass outside core; correct |
| `Distribution` + `Gaussian`/`Uniform`/`Particles`/`Sampler` | `planning/distributions.py` (170 lines, born 2026-09-10) | `x ~ p`, `sample(key)`, `mean()` | correct, in the wrong band |
| `CostFunction` + algebra | `core/costs.py` | `J = ∫ e^{-ρt} g dt + h` | solid; params plan agreed 2026-09-13 |
| `Shape` + `Sphere`/`Box`/`Union`/`Inflated` | `core/geometry.py` | occupied space, `sdf`; dual of `Set` | solid; glyph name collision (S30, known) |
| `PlanningProblem`, `StochasticPlanningProblem`, `ProblemParameters` | `planning/problems.py` | sys + sets + cost + horizon (+ laws) | solid |
| `PlanningSolution`, solver records | `planning/results.py` | policy + evidence | solid (S48, 2026-09-15) |
| `Evaluation`, `score_trajectory` | `planning/evaluation.py` | the distribution of `J` over draws | solid; one scoring contract |
| `MathematicalProgram`, `OptimizationResult` | `optimization/` | the NLP | solid |
| `LyapunovCertificate` | `analysis/lyapunov.py` | `{V ≤ c}` | a `Set` in disguise (§A.5) |
| `StateSpaceGrid` | `policy_synthesis/discretizer.py` | a discretised `BoxSet × BoxInputSet` | rebuilds its boxes from raw arrays |

## A.3 What is solid (do not touch)

- **The precedence ladder works in practice.** Every finding resolves under it: purity and
  closure are untouched; "domain readability" (rank 4) is where "speak the noun" lives.
- **`Set.margin` is already a native-array equation path** on every library set, jit-tested
  (`test_backends.py:119`); `FieldSet` differentiates through a scene. The CBF plan gets its
  barrier and its gradient from this for free.
- **`Distribution` traces under JAX** (`sample(key)` inside `jit` / `vmap` / `scan`). The
  duck-type surface (`dim`, `mean`, `sample`, optional `support`) is the right size.
- **One description, two verbs** (solve, evaluate), the stochastic sibling with `nominal()` and
  `as_stochastic()`, one scoring contract: the taxonomy plan delivered. Strongest piece of the
  planning foundation.
- **The sets ↔ shapes duality** (`margin ≥ 0` free, `sdf < 0` occupied), one sign convention.

## A.4 Findings

Severity: **S** semantics (results differ), **D** design (blocks a planned tool), **M**
maintenance, **T** textbook style. Lines as of `62ed82e`.

**F1 (S) — RL and the JAX Monte Carlo score use the box of `X`, not `X`.**
`reinforcement_learning/environment.py:209-216` (`allowed_box`) reads `X.lower/upper` when `X`
is a `BoxSet`, otherwise silently falls back to the *system's* state bounds; `step()` and
`evaluation.py:233-236` (`evaluate_jax`) test `x < x_lb | x > x_ub`; `interfaces/gymnasium.py:347`
(`ProblemEnv.step`) does the same. `score_trajectory` (`evaluation.py:63`) and `evaluate_numpy`
test `problem.X.contains`. For `X = BoxSet.from_system_state(sys) &
scene.clearance_field(body).as_constraint()` (the idiom of `rrt_car_parking.py` and the corridor
trajopt demo) the RL learner never sees an obstacle exit, `backend="jax"` and `"numpy"` report
different failure rates, and the parity test (`test_planning_stochastic.py:265`) only covers box
problems. Fix: `xp.all(X.margin(x_next, t) >= 0)`; every library margin already traces;
bit-identical on box problems. Latent: no RL demo or notebook uses a non-box `X` today.

**F2 (D) — distributions live in `planning/`; their next consumers are library bands.**
RULES 3.2: libraries import only `core`. The Kalman filter (ROADMAP v0.2, P4) is a `Gaussian`
with covariance pushed through `A` and conditioned on `C`; `blocks/sources.WhiteNoise` is a
`Gaussian` drawn on a grid; `identification/` will return a posterior. None may import
`minilink.planning.distributions`. The taxonomy plan already listed "thin `core/` helpers" as the
alternative. Move to `core/distributions.py` beside `sets.py` (sets are support, distributions
are probability: taxonomy law 4). Call sites: `problems.py:20`, an `evaluation.py` docstring,
two test files; no example or notebook imports the deep path; `minilink.planning` keeps
re-exporting the four names (ROADMAP §4.1 gate 7 holds).

**F3 (M) — `Uniform` is `BoxSet` twice; the two nouns draw samples two ways.**
`Uniform(lb, ub)` builds a `BoxSet(lb, ub)` as `support`, re-derives the same `mean`, and
re-implements the uniform draw `BoxSet.sample` has (NumPy only there). `lb/ub` vs
`lower/upper`. `Set.sample(rng, n=1, params)` always returns `(n, dim)` from a NumPy
`Generator | None`; `Distribution.sample(key, n=None)` returns `(dim,)` or `(n, dim)` from a
`Generator | int | JAX key`. Every library caller of `Set.sample` writes `[0]` (`rrt.py:392,399`,
`extenders.py:70`, `tabular.py:326`). One convention: the distribution's, on both; `Uniform`
then holds a `BoxSet`, and a traceable `BoxSet.sample` is the uniform law on the box.

**F4 (M) — the RNG helpers exist twice.** `is_jax_key` / `generator` (`distributions.py:160-169`)
and `split_keys` (`problems.py:475-483`) inline the same checks; `core/backends.array_module`
already owns the JAX duck test. One home, next to the distributions.

**F5 (D) — seven objects that are a set or a distribution without the type.** §A.5.

**F6 (D) — 21 `isinstance` probes where one method would do.** Two are legitimate lowerings at
the NLP boundary and stay: `SingletonSet → equality residual`, `BoxSet → decision bounds`
(`direct_collocation.py:356-363, 442`; `shooting.py:199, 292`; RULES 4.3 "convert once at the
boundary"). The other 19 are capability probes ("is `X` a box so I can read its extent?":
`discretizer.py:335, 348, 434, 605`, `environment.py:212`, `rrt.py:197` via `from_system_state`,
`lyapunov.py:654`, `search/plotting.py:396`). Consequence: an `IntersectionSet` cannot say it
contains a box, so `X = bounds & free` loses the box→bounds lowering in collocation
(`decision_bounds` returns `±inf`; every bound becomes an inequality margin) and the DP grid
refuses it (`discretizer.py:336`). One hook answers all of them: `Set.bounding_box() -> BoxSet |
None` (and on `InputSet`). A ratchet test then keeps `isinstance(..., BoxSet)` out of the tools.

**F7 (T) — `core/sets.py` against the textbook rules.** §A.6.

**F8 (D, later) — two set types the next tools will ask for.** `EllipsoidSet(center, P, level)`:
the Lyapunov region, the LQR-natural set, a Gaussian's confidence region; today
`LyapunovCertificate` carries `contains`, `extent`, `sample_in_ellipsoid` by hand. `UnionSet`
(`|`, margin = max): the day reachability, invariant-set or multi-goal work lands; one class.

**F9 (D, before `estimation/`) — the noise convention is unstated.** `disturbances={port:
Distribution}` draws one value per control period and holds it, so its variance does not scale
with `dt`: `Gaussian(0, 0.1)` is a different physical disturbance at `dt = 0.01` and `0.1`.
`WhiteNoise` (`blocks/sources.py`) has `var` + `sample_period`, the same ambiguity. A Kalman
filter needs the rule once: per-step covariance `Q_d`, or spectral density `Q_c` with
`Q_d = Q_c / dt`. RULES 4.12 (announced defaults) applies. Decide with P4; recorded here.

**F10 (T) — `planning/distributions.py`.** `Gaussian._mean` underscore attribute because `mean()`
is a method while `std` is an attribute; `Evaluation.mean` and every `x_bar` are attributes.
`lb/ub`. `Particles.sample` binds `xp = array_module(key)` on a key (works, reads oddly). A
13-line module docstring. The math is clean.

**F11 (T, small) — `PlanningProblem`** carries `_coerce_*` / `_require_*` underscore statics and a
`_default_input_set` that is `BoxInputSet.from_system_inputs(sys)` waiting to be named (twin of
`BoxSet.from_system_state`). Not a rule violation; the neighbourhood's style.

**F12 (D) — the one missing noun: the scalar function of the state.** The textbook has one
object that appears under six names — a Lyapunov function `V(x)`, a barrier `h(x)`, a
cost-to-go `J(x)`, a critic `V_w(x)`, a clearance `d(x)`, a fitted approximator `wᵀφ(x)` —
and Minilink has it in six shapes, none of them core:

| Today | Shape | Where |
| --- | --- | --- |
| `StateField` (`value(x, u, t, params)`, `as_constraint()`, `as_cost()`) | the right noun, in the spatial band | `planning/spatial/state_fields.py:27` |
| `LyapunovCertificate.V`, `V_dot`, `contains`, `extent` | methods on a record | `analysis/lyapunov.py:135-168` |
| `PlanningSolution.cost_to_go` | a bare callable (DP: interpolated table; RL: `critic_cost_to_go()`) | `results.py:50`, `rl/planner.py:297` |
| `critics.ValueFunction`, `QFunction` | `value(w, x)` with explicit weights | `rl/critics.py:10, 47` |
| `LinearApproximator` over `Features` (`QuadraticFeatures`, `RadialBasisFeatures`) | `fit`, `sgd_step`, then a callable | `policy_synthesis/approximation.py` |
| `PolicyEvaluator.value_at(x)` | a method on a tool | `policy_eval.py:80` |

The quadratic form `(x − x̄)ᵀ P (x − x̄)` alone is written three times (`costs.py:233`,
`lyapunov.py:138, 695`, `approximation.py:31`) and is also the LQR value function no tool
exposes. The noun already exists — `StateField` — and its two exports are exactly the bridges the
constitution wants: a *sublevel set* `field.as_constraint(upper=c)` is a `Set` (the region of
attraction, the safe set of a barrier), a *shaping* `field.as_cost()` is a `CostFunction`.
Promoting it to `core/fields.py` with two concrete fields that have consumers today —
`QuadraticField(center, P)` (Lyapunov `V`, `QuadraticFeatures`, the LQR value) and `GridField`
(the DP table, `PolicyEvaluator`) — makes `LyapunovCertificate.V` a field, `region =
V.sublevel(level)` a `Set` with `sample` (so F8's `EllipsoidSet` is not a new class), the CBF
barrier a field, and fitted value iteration (TODO row) "a `StateField` fitted by
`LinearApproximator`". Later rows, not now: the RL critic as a field once training is over.

**Time and action (maintainer, 2026-09-15).** The noun already has the room: the existing
signature is `value(x, u=None, t=0.0, params=None)`, the model's own `(x, u, t; p)`. So
`V(x)`, `J(x, t)`, `Q(x, u)` and `Q(x, u, t)` are one object that reads different arguments,
and the name `StateField` is a misnomer once `u` counts: the noun is `Field`. Consumers with
the extra arguments exist today: `lqr_gain_schedule` returns `(t, K, S)` and `xᵀS(t)x` is the
finite-horizon LQR value function; `DynamicProgrammingPlanner(record_history=True)` keeps the
per-sweep tables, the finite-horizon `J(x, t)`; `TabularLearningPlanner.Q` is a table over
`(x, u)` whose docstring already reads `J = min_u Q`, `pi = argmin_u Q`; the RL `QFunction`
is `Q_w(x, a)`; and the CBF condition `∇h · f(x, u) + α h(x) ≥ 0` is a `(x, u)` field whose
superlevel set is the state-dependent `InputSet` `U(x)` the safety QP constrains — the third
bridge (`as_input_constraint`) beside `as_constraint` and `as_cost`.

## A.5 Sets and distributions in disguise

| Where | What it really is | Today |
| --- | --- | --- |
| `RolloutEnvironment.x_lb, x_ub` (`environment.py:72, 209`) | `problem.X` | box or system bounds (F1) |
| `StateSpaceGrid.x_lb, x_ub, u_lb, u_ub` (`discretizer.py:198`) | the `BoxSet` / `BoxInputSet` it was built from | raw arrays; `TabularLearningPlanner.grid_box = BoxSet(grid.x_lb, grid.x_ub)` rebuilds it (`tabular.py:137`) |
| `LyapunovCertificate` region (`lyapunov.py:149-168, 755`) | `EllipsoidSet(x_bar, P, level)`; `verify` draws `Uniform` on it | `contains`, `extent`, `sample_in_ellipsoid` by hand; `state_box` / `as_box` / `clip_box` re-derive boxes |
| `Sys2Gym.reset_mode` (`gymnasium.py:131-135, 167-172`) | `Uniform` / `Gaussian` / `Particles` on `x0` | three hand-rolled modes; `ProblemEnv.reset` already draws from the problem; `ProblemEnv.step` still tests the box (F1) |
| `RRTPlanner._sample_box` (`rrt.py:197`) | `problem.X.bounding_box()` | the *system's* box even when `X` is narrower, then rejection |
| `NeuralPolicyController.u_mid, u_half` (`neural.py:116-135`) | centre and half-width of the input box | raw port bounds (fine for a block; `BoxSet.center` / `half_width` would name them) |
| `GaussianHead`, `SquashedGaussianHead` (`policy.py`) | conditional laws `π(a \| x)` with `sample`, `log_prob`, `entropy` | parametric, so not `Distribution` subclasses; a `Gaussian.log_prob` in core would let the density line read as the textbook's |
| `WhiteNoise` source (`sources.py:163`) | a `Gaussian` drawn on a time grid | hand-rolled; a `NoiseSource(distribution, sample_period)` later |

## A.6 The two files against RULES §5

`core/sets.py`

| Rule | Where it slips | Fix |
| --- | --- | --- |
| 5.1 bare signatures | `margin(self, z: np.ndarray, t: float = 0.0, params=None) -> np.ndarray` on every set; `residual` | bare, as `costs.py` `g` / `h`; shapes in the docstring |
| 5.3 no `self.` in math | `BoxSet.margin` (`z - self.lower`), `SingletonSet.residual` (`z - self.point`), `BallSet.margin` | unpack first, as `geometry.py:93-97` |
| 5.22 comment the step | `SingletonSet.margin = -|z - p|` (equality as a degenerate inequality) | one step comment |
| 5.23 no preamble walls | 13-line module docstring restating DESIGN §4 | title + two lines |
| one class style (5.21 after Phase 1) | four frozen dataclasses, two plain classes | frozen dataclasses for all six |
| 7.1 consolidate | `(a & b) & c` nests; `SumCost.of` flattens | flatten |
| 4.3 named adapters | `_default_input_set` on `PlanningProblem` | `BoxInputSet.from_system_inputs(sys)` |

`planning/distributions.py`

| Rule | Where it slips | Fix |
| --- | --- | --- |
| 5.4 one symbol | `lb/ub` here, `lower/upper` in sets and geometry | `lower/upper` |
| 5.7 / 5.8 | `Gaussian._mean` to make room for `mean()` | `mean` as an attribute on every distribution |
| 7.1 consolidate | `Uniform` rebuilds `BoxSet`; two RNG helper copies | `Uniform` holds a `BoxSet`; one helper home |
| 5.23 | 13-line module docstring | title + two lines |

Everything else reads well: short methods, named steps, margins and draws that trace. A
half-day pass, not a rewrite.

## A.7 Readiness for the next tools

| Tool (plan) | Needs from the foundation | Ready? |
| --- | --- | --- |
| Kalman / Luenberger / EKF (`estimation/`, v0.2 P4) | `Gaussian` in `core` with full covariance, `log_prob`; one noise convention (F9) | after F2 and a `cov=` keyword (additive; `std=` stays) |
| CBF safety filter (`cbf-safety-filter.md`) | a traceable barrier `h(x) ≥ 0` and `∇h` | **ready now**: `h = X.margin`, `jax.grad` through `FieldSet`; the plan should say "barrier = a `Set`", not a new `Scene.as_cbf`; HOCBF is `grad(margin) · f` |
| Robust problem class (taxonomy §3) | `θ ∈ Θ` with `Θ` a set over params | needs a flatten rule for params dicts; defer with the class |
| Chance constraints / stochastic MPC | `Set` and `Distribution` together, both native | ready in principle once F2 lands |
| Reachability, invariant sets, Lyapunov | `EllipsoidSet`, `UnionSet` | F8, one class each when the consumer lands |
| Identification (`fitting.py`) | `Trajectory` in, params out | nothing new |
| Differentiable closed-loop cost (cost-params) | cost params; `x0` and sets as inputs | sets' `params` argument exists but no library set reads it (DESIGN §4 deferred row); do it after cost-params, by the same rule |
| Tabular RL / DP / approximate DP | a grid that *is* a discretised box | `grid.X`, `grid.U` (Phase 2) |

---

# Part B — Phase 1: vision, constitution, rules (docs only; maintainer-owned)

## 1.1 Why first

Three code steps depend on a rule: the style pass needs 5.1 to cover `margin` (today it names
only `f`, `h`, `tf`, so the `np.ndarray` hints in `sets.py` do not even break a rule); the
frozen-dataclass question needs 5.21 to say which objects are values; the `bounding_box` hook
needs 4.3 to say that a tool asks the set. And the 2026-09-12 audit's lesson holds: prose rules
drift within days unless the check lands with the rule, so Phase 2 carries one ratchet test per
new sentence.

## 1.2 Amendments the maintainer agreed to (2026-09-15), exact wording

**CONSTITUTION §2, new subsection after "Memoryless components ...":**

> ### The Mathematical Objects
>
> Around the model, the tools speak the textbook's own nouns, and a tool that takes or returns
> one of these quantities takes or returns the object, never its arrays:
>
> - `Trajectory` — the sampled evolution `(t, x, u)` every simulation and plan returns.
> - `Set` — an allowable region, `z ∈ Z(t)`, written as `margin(z, t) ≥ 0`: the state box, a
>   goal ball, free space, a level set.
> - `Distribution` — a probability law over a vector, `sample(key)` and `mean`. Sets are
>   support, distributions are probability; never one for the other.
> - `Field` — a scalar function on the model's `(x, u, t)`, the signature of `f` and `g`: a
>   Lyapunov function `V(x)`, a cost-to-go `J(x, t)`, a Q-function `Q(x, u)`, a barrier `h(x)`,
>   a clearance `d(x)`. Its sublevel and superlevel sets are `Set`s over `x`, or `InputSet`s
>   over `u` at a given `x`; its shaping is a `CostFunction`. *(agreed 2026-09-15; time and
>   action arguments added the same day)*
> - `CostFunction` — the running cost `g(x, u, t)` and the terminal cost `h(x, t)` of `J`.
> - `PlanningProblem` — a system, its sets, a cost and a horizon; the stochastic sibling adds
>   distributions over starts, parameters and disturbances.
> - `PlanningSolution` — a policy, itself a `System`, and the evidence for it.
> - `MathematicalProgram` — the one NLP shape every transcription lowers to.
>
> **Lower late.** A tool carries these objects to the solver boundary and converts to arrays
> there, once.

**RULES 2.5, appended sentence:**

> Before adding a record, name the object it already is: a `(lower, upper)` pair is a `BoxSet`,
> a reset rule is a `Distribution`, a sublevel set is a `Set`, a sampled run is a `Trajectory`.

**RULES 4.3, appended sentences:**

> A tool that needs a box asks the set (`X.bounding_box()`); it does not `isinstance` the set.
> The one `isinstance` on a set is the lowering at the solver boundary: a singleton to an
> equality residual, a box to decision bounds.

**RULES 5.1, first sentence widened:**

> Do not put type hints inside $f, h, tf$, port computations, or any other native-array
> equation path (`Set.margin`, `CostFunction.g` / `h`, `Shape.sdf`, `Distribution.sample`).

## 1.3 Further comments on the stack (new this round; each needs a yes)

**C1 — CONSTITUTION §1.4, a boundary that keeps the two nouns small.** Add a bullet:

> - It is **not** a probability or set-algebra library. Sets and distributions are the few
>   textbook objects the tools need — a box, a ball, a level set; a Gaussian, a uniform law,
>   particles — each with a two-method surface, not a general library.

Why: `distributions.py`'s own docstring already says "a duck type, not a probability library";
the constitution is where that sentence stops the first `scipy.stats`-shaped proposal.

**C2 — CONSTITUTION invariant 2, purity covers every equation path.** Today: "Equation paths
($f, h$) have no internal mutable state". Proposed: "Equation paths — $f$, $h$, and the equation
methods of sets, costs, shapes and distributions (`margin`, `g`, `h`, `sdf`, `sample`) — have no
internal mutable state, no cached side effects, and no memory between calls." Why: a
`CallableSet` around a stateful closure or a `Sampler` that advances its own generator breaks
`vmap` exactly as a stateful `f` would; the invariant should name them.

**C3 — CONSTITUTION §4.2, tools return the nouns.** "take a `System` (or a `PlanningProblem`)
and return domain objects — trajectories, plans, certificates — or matrices and figures."
Today it says "return trajectories, matrices, or figures", written before `PlanningSolution`
and `LyapunovCertificate` existed.

**C4 — RULES 2.3, the dual parameter pattern is one rule for every parametric object.** Append:
"The same rule holds for costs, sets and shapes: `params is None` resolves to the object's own
defaults; any dictionary replaces them at that level." This generalises cost-params decision 1
(2026-09-13) and is what DESIGN §4's "deferred" rows will satisfy.

**C5 — RULES 5.4, say what a symbol collision is allowed to be.** The rule says "a symbol keeps
one meaning across the codebase". The literature does not: `h` is the output map on a `System`
and the terminal cost on a `CostFunction`; `g` the running cost, the gravity vector `g(q)`, and
the NLP inequality; `C` the Coriolis and the output matrix; `B` the actuator map and the input
matrix. The 2026-09-05 review flagged it and suggested renaming `CostFunction.h`; every
notebook cost class defines `def h(self, x, t)`, so a rename is off the table during the term.
Proposed wording: "A symbol keeps one meaning within an object, and one meaning across the
codebase wherever the literature allows. The textbook overloads a few letters across objects
(`h`: output map on a `System`, terminal cost on a `CostFunction`; `g`: running cost, gravity
vector, NLP inequality; `B`, `C`: mechanical vs state-space). Those collisions are inherited,
listed in DESIGN §4, and never added to." → **question Q3 below**.

**C6 — RULES 5.21, which objects are values.** Today: "Use dataclasses for transparent domain
records (`Trajectory`, a certificate, a plan)". The cost-params plan chose plain classes for
costs (their `params` are mutated in place by notebooks); `sets.py` mixes both styles.
Proposed: "Frozen dataclasses for value objects — a `Trajectory`, a set, a shape, a certificate,
a solver record. Plain classes with `params` for parametric equation objects — systems, costs.
Never dataclasses as input/output wrappers around arrays (rule 2.5)." This settles the mixed
style in `sets.py` by rule rather than by taste.

**C7 — RULES §6, a new 6.12 "Cheap rules are tests".** "When a rule can be checked by an AST
walk or a grep, the check lands with the rule (`test_teaching_imports.py`,
`test_repo_contract.py`, the set-probe check in `test_planning.py`, the flat-demo check)." The
2026-09-12 audit said this in prose; making it a rule is what keeps 4.3 and 6.10 true.

**C8 — AGENTS.md "Ask first" list names the math objects.** "core architecture and the API of
the main tools (`System` family, the core mathematical objects — `Trajectory`, sets,
distributions, costs, `PlanningProblem`, `PlanningSolution` — diagrams, compile, `Simulator`,
planners, `Optimizer`, controllers)". Today `core/sets.py` and `core/costs.py` are core by
implication only.

**C9 — DESIGN §4 "Trajectory, sets, costs, geometry".** Add a distributions bullet (home
`core/distributions.py`, the draw convention, `support`); the `bounding_box` hook on the sets
bullet; the notation-collision list (C5); the noise-convention open item (F9). DESIGN §3
package map: `core/` row adds "distributions". DESIGN §6 stochastic paragraph: the new path.

**C10 — ROADMAP.** §6 review queue: "Open (v0.2, before `estimation/`): the disturbance
convention — per-step covariance or spectral density." §1 north-star claim 2 may name the
nouns ("a task is a `PlanningProblem`: system, sets, cost, horizon; or its stochastic sibling")
— pitch-facing, maintainer's call. §3 TRL: no change until Phase 2 lands.

**C11 — What I would not change.** The precedence ladder; the length of RULES (the 2026-09-12
audit left trimming §1–§2 to the maintainer, and nothing here needs it); AGENTS.md's workflow
(its behaviour-preserving-refactor recipe is exactly the one Phases 2–3 follow); no fifth
document.

**C12 — `Field` in the constitution's object list (F12): agreed 2026-09-15**, folded into the
1.2 wording (renamed from `StateField` the same day: with `u` and `t` in play the noun is the
field on `(x, u, t)`; `StateField` is not a student-facing name, two DESIGN §6 mentions only). It closes the triangle set ↔ field ↔ cost that the spatial band already draws
(`as_constraint`, `as_cost`), and it is the object the Lyapunov, CBF, DP and approximate-DP
chapters all stand on. Phase 4 is its promotion.

**C15 — One signature, stated once (maintainer, 2026-09-15).** The idea is spread over six
places and none says it whole: CONSTITUTION invariant 2 (purity, for `f` and `h` only),
invariant 5 and §6 "backend-native math" (one path for NumPy and JAX), RULES 1.4 / 5.2 / 5.17
(`xp`, native arrays, for `f` and `h`), RULES 2.3 (the dual `params` pattern, for systems),
DESIGN §4 "native-array equation rule" (its list: `f`, `h`, ports, sets, costs, transcriptions,
the NLP callables) and DESIGN §4 parameters ("deferred: call-time `params` on `Shape`, `Set`,
`CostFunction`"). Missing is the sentence that every object's mathematics is one stateless
method on the model's arguments, traceable in the arguments and in `params` alike. Proposed,
in CONSTITUTION §2 right after the objects list:

> **One signature.** An object's mathematics is a method on the model's arguments
> `(x, u, t; p)`, or the subset it reads: `f`, `h`, `tf` on a system; `g`, `h` on a cost;
> `margin` on a set; `value` on a field; `sdf` on a shape; `sample(key)` on a distribution. It
> is stateless, takes and returns native arrays on NumPy or JAX through one equation path,
> and traces under JAX in every argument, `params` included — so `jit`, `grad`, `vmap` and a
> parameter family reach any object the way they reach `f`. An object that does not yet read
> `params` is a gap to close, not a design.

And the rules that follow from it: RULES 5.2 "in `f` or `h`" → "in every equation path"; RULES
5.17 "Keep `f` / `h` on native arrays" → "Keep every equation path (the list of 5.1) on native
arrays"; RULES 2.3 as C4; DESIGN §4's native-array list gains fields and `Distribution.sample`,
and its "deferred" `params` rows become the tracked gap (TODO Later already carries
"parametric `Shape` / `Set` / `Cost` overrides"; Phase 5.4 here). ROADMAP §1 claim 3 may read
"the same *objects* are differentiable and compiled" instead of "the same `f`".

**C13 — What is deliberately *not* an object** (so nobody proposes it later): an operating point
(`x_bar, u_bar` arrays; a record would be a DTO), a pose or transform (4×4 arrays, `tf`), a
metric (a callable), a transition batch (arrays), a criterion (a string until CVaR earns an
object). RULES 2.5 already covers these; listing them in the review doc saves the argument.

**C14 — Demos and teaching notebooks: less is better (maintainer, 2026-09-15; wording for a
yes).** Today RULES 6.1 asks for open-and-run scripts with a one-line docstring and the story
in comments; nothing says what a demo may *contain*. The census (`examples/demos/`, 73 scripts;
`examples/tutorial/` + `examples/teaching/`, 33 notebooks):

| | Demos | Notebooks |
| --- | --- | --- |
| Clean (no helper function, no custom plot code) | 53 / 73 | 16 / 33 |
| With top-level helper functions | 11 (40 functions) | 17 (72 functions) |
| With hand-written matplotlib code | 15 (85 lines) | 9 (20 cells) |
| Cells mixing an API call with matplotlib code | — | 6 cells in 3 notebooks |
| `print(...)` statements | 121 (10 scripts with five or more) | 293 |

Worst: `rrt_holonomic_obstacles.py` (8 helpers, 252 lines), `cartpole_rollout_gradients.py`
(9 helpers), `optim_plot.py`, `neural_controller_jax.py`, `car_circuit_rl.py`,
`trajopt_holonomic_corridor.py` (218 lines); `articulated_robot_eom.ipynb` (26 helpers),
`from_value_iteration_to_ppo.ipynb` (4 mixed cells), `11_reinforcement_learning.ipynb`. Most
helpers are library gaps in disguise: the hand-written rollout and loss in the three
`compile/` demos are the differentiable closed-loop cost of the ROADMAP review queue; the
`make_*_extender` and `path_cost` helpers are `RRTOptions` and the search record; the track
`features` helpers are the composite policy block of the RL plan. Proposed wording:

> **6.1 A demo is the API, and nothing else.** *Simplicity is the ultimate sophistication*
> holds for a demo or a teaching notebook as it does for the core: the shortest sequence of
> library verbs that tells the story — build, compose, solve, evaluate, plot, animate — with
> constants at the top, a one-line title docstring and the story in short inline comments
> (5.22–5.23). Less is better. Reporting goes through the objects themselves: their `print`
> (`__str__`) and their native plots (`plot_trajectory`, `plot_solution`, `plot_control_law`,
> `plot_cost2go`, `plot_learning_curve`, `plot_tree`, `scene.plot`, ...). A report the library
> cannot give is first a library gap — a missing `__str__` or `plot_*`, in the agent's plotting
> lane — and only then a demo cell.
>
> **6.10 Flat demos.** No functions or classes in a demo or a teaching notebook except the
> model or cost it is about (its `f`, `h`, `g`) and a step the notebook's text teaches by hand
> (a hand-written RK4, a policy-gradient update). No wiring, tuning or plotting helpers, no
> configuration dictionaries feeding them, no `main()`, no `try / except ImportError` around
> plots (6.2). A loop over variants is fine; a factory is not.
>
> **6.11 Side work is quarantined.** A custom plot or a side analysis that earns its place is
> its own cell in a notebook, or its own block at the end of a script under one comment line.
> It never interleaves with the API lines, and the demo still reads with that cell removed.

`examples/README.md` "Demo style" then points at 6.1, 6.10, 6.11. The sweep of the offenders
and the native plots they need is Phase 6 (student-facing: per file, maintainer reviews).

## 1.4 Rulings (maintainer, 2026-09-15)

**Q1 — Where do bounds live? → derived `.box` property.** Ports keep `lower_bound` /
`upper_bound` arrays as the student's line; `VectorSignal` gains a read-only `box` property
(`sys.state.box`, `port.box`) returning a `BoxSet` derived on each read (RULES 5.6), so a tool
reads one set and never two arrays. `BoxSet.from_system_state(sys)` stays (used by
`rrt_car_parking.py`) and delegates to it. DESIGN §4 ports bullet records the property.

**Q2 — `InputSet` stays** as the textbook's `u ∈ U(x, t)`: actuator envelopes depend on state
(a DC motor's torque–speed curve is GRO501 material), and the cost is one short ABC.
`BoxInputSet` gains `bounding_box()` like the rest; nothing renames.

**Q3 — The `h` / `g` collision → amend 5.4** as in C5. One meaning within an object; across
objects the literature's own overloads (`h`, `g`, `B`, `C`) are inherited, listed in DESIGN §4
as a closed list, never added to. Nothing renames; notebooks untouched.

**Q4 — open: the Hamiltonian and the costate.** The maximum principle appears in
`cartpole_lqr.ipynb` (costate `λ = S x`, the Hamiltonian matrix behind `lqr_gain_schedule`),
and nowhere else: the toolbox is direct methods (collocation, shooting), DP and RL. If the
course goes further — indirect shooting, a bang-bang law read off the switching function — the
Hamiltonian `H(x, u, λ, t) = g + λᵀ f` is one method on `PlanningProblem` (`jax.grad` gives the
costate equation `λ̇ = −∂H/∂x`) and the costate is a `Trajectory` signal; if not, nothing is
missing. Affects Phase 4 only; answer on review.

**Q5 — open: sets and distributions over parameter dictionaries.** Both nouns are over vectors;
parameters are pytrees. `params_distribution={name: Distribution}` draws each leaf
independently, so a correlated posterior over `(m, l)` (identification), a joint grid of
parameter pairs (the Buckingham-π family sweeps of `rollout_batch`), or a robust set `θ ∈ Θ`
cannot be written. Not a new noun: a flatten rule (`ravel_pytree`) that lets `Gaussian`,
`Particles`, `BoxSet` take and return a dict. Lands with its first consumer (identification or
the robust class); named here so it is designed once.

## 1.5 Phase 1 done when

CONSTITUTION, RULES, AGENTS, DESIGN, ROADMAP carry the agreed wording; the review doc records
the amendments and the rulings on C1–C10 and Q1–Q3; `docs/plans/sets-and-distributions.md`
exists with Phases 2–4 adjusted to the rulings; TODO has the rows; `ruff` untouched (no Python).
`test_repo_contract.py` (link check) passes on the new documents.
